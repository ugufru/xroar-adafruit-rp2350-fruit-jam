// SPDX-License-Identifier: GPL-3.0-or-later
// xroar-adafruit-rp2350-fruit-jam — Boot Color BASIC to the monitor (FRUITJAM-25).
//
// The integration milestone: the hand-written CoCo machine (FRUITJAM-22) runs
// Color BASIC loaded from SD (FRUITJAM-06); its frame-batched VDG renderer
// (FRUITJAM-24) produces a 256x192 indexed frame; we map that through the CoCo
// palette into the 320x240 RGB565 framebuffer (FRUITJAM-04) and the HSTX engine
// scans it out over DVI at 640x480p60.
//
// Core split (previews FRUITJAM-11): core 0 runs emulation + composes the
// framebuffer; core 1 runs the pico_hdmi video engine (near-zero CPU). The
// framebuffer is a single buffer for this milestone — occasional tearing is
// acceptable; a front/back pointer swap is FRUITJAM-11.
//
// 252 MHz / 1.25 V operating point (FRUITJAM-03); pico_hdmi built with
// MODE_HSTX_CLK_DIV=2 -> clk_hstx = 126 MHz.

#include <Arduino.h>
#include <string.h>
#include <Wire.h>
#include <I2S.h>
#include <Adafruit_TLV320DAC3100.h>
#include <Adafruit_NeoPixel.h>
#include "pico/multicore.h"
#include "hardware/vreg.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/structs/bus_ctrl.h"
#include "pico/stdlib.h"
#include "pio_usb.h"
#include "Adafruit_TinyUSB.h"

extern "C" {
#include "pico_hdmi/video_output.h"
#include "pico_hdmi/video_output_precomposed.h"   // stale/resync counters (FRUITJAM-14 probe)
#include "pico_hdmi/hstx_data_island_queue.h"     // silence counter (FRUITJAM-14 probe)
}

// FRUITJAM-37: 1 = DVI mode (no data islands, max sink compatibility);
// 0 = HDMI mode (data islands, needed for future HDMI audio).
#ifndef COCO_DVI_MODE
#define COCO_DVI_MODE 1
#endif

// FRUITJAM-53: boot-progress NeoPixels, OFF by default until the video
// interaction is understood. Bisected evidence that they cost HSTX stability:
//   aaa5593 (pre-NeoPixel)              0 desyncs / 3 min
//   1525a48 (fonts, pre-NeoPixel)       0
//   main with the strip disabled        0
//   main as committed                   15
// begin() claims a PIO state machine for GPIO32 and holds it for the whole run;
// releasing it before core 1 starts helped (15 -> 3) but did not clearly reach
// zero, and run-to-run variance is too high for short A/B runs to settle it.
// Set to 1 to get the boot progress bar back.
#ifndef COCO_NEOPIXEL
#define COCO_NEOPIXEL 0
#endif

// FRUITJAM-45/48: how long setup() waits for a serial host before printing the
// boot banner. 1000 ms: 300 ms proved too tight — a monitor measured 430-520 ms
// to become ready, so captures lost the banner and the first two STAGE lines.
// 1000 ms clears that with ~2x margin and still saves 500 ms per boot against the
// original 1500. Build with -DSERIAL_READY_WAIT_MS=1500 to be certain of a
// complete log on a slow host, or a small value to boot as fast as possible.
// The wait is no longer dead time to look at: LED 1 blinks red throughout
// (FRUITJAM-48), so a board sitting here is visibly waiting rather than hung.
#ifndef SERIAL_READY_WAIT_MS
#define SERIAL_READY_WAIT_MS 1000
#endif

// Blink period for LED 1 during that wait.
#define SERIAL_WAIT_BLINK_MS 100

extern "C" {
#include "ff.h"
#include "f_util.h"
#include "coco_machine.h"
#include "dkbd.h"          // DSCAN_* CoCo keyboard scancodes
}

// Public in pico_hdmi's video_output.c but not its header: full HSTX+DMA restart
// to recover from a desynced command stream (FIFO underrun -> permanent loss of
// lock). Used by the desync watchdog below.
extern "C" void video_output_force_resync(void);

#include "framebuffer.h"    // dvi::Framebuffer, dvi::rgb565, FB_WIDTH/HEIGHT

// Scanline callback runs from the core-1 DMA IRQ — keep it out of flash.
#define RAM_FUNC __attribute__((section(".time_critical.coco_main")))

// CoCo VDG palette -> RGB565. Index order is the contract with coco_machine's
// PAL_* constants (FRUITJAM-24): 0 GREEN .. 9 DARK_GREEN.
// NOT const: scanline_cb reads this per-pixel from the core-1 DMA IRQ, so it
// must live in RAM (.data), never flash — an XIP stall there starves the HSTX
// FIFO (FRUITJAM-04 rule). A const array would sit in .rodata/flash.
static uint16_t g_pal[16] = {
    dvi::rgb565(0x30, 0xD2, 0x10),  // 0 GREEN (the CoCo screen green)
    dvi::rgb565(0xF0, 0xE0, 0x30),  // 1 YELLOW
    dvi::rgb565(0x20, 0x30, 0xF0),  // 2 BLUE
    dvi::rgb565(0xC0, 0x18, 0x10),  // 3 RED
    dvi::rgb565(0xF0, 0xF0, 0xE0),  // 4 WHITE / buff
    dvi::rgb565(0x20, 0xC0, 0xC0),  // 5 CYAN
    dvi::rgb565(0xC0, 0x20, 0xC0),  // 6 MAGENTA
    dvi::rgb565(0xE0, 0x80, 0x10),  // 7 ORANGE
    dvi::rgb565(0x00, 0x00, 0x00),  // 8 BLACK
    dvi::rgb565(0x00, 0x38, 0x00),  // 9 DARK GREEN
    0, 0, 0, 0, 0, 0
};

// Single RGB565 framebuffer, matching the proven-stable FRUITJAM-04 display_test.
// The palette lookup happens here in the blit (core 0), so scanline_cb stays a
// plain copy — a per-pixel palette lookup in the callback overran the HSTX line
// budget and desynced the link (black after a few seconds). RGB565 double-
// buffering would be tear-free but two 150 KB buffers overflow SRAM; the
// residual blit/scanout tearing is a FRUITJAM-11/23 refinement.
static dvi::Framebuffer g_fb;
static uint8_t          g_rom[16384];

// pico_hdmi 2.0-beta precomposed/native scanout ring (README "Minimal pattern").
// Each entry precomposes one active-line header off the ISR so the per-scanline
// work is ~1.5us (patch a data island) instead of a 320-px fill — the path built
// for "320->640 scaling while HDMI runs" (emulators). ~312 B/entry.
static video_output_precomposed_line_t g_compose_ring[48];

// Authentic NTSC CoCo: ~0.895 MHz / 60 Hz = 14915 6809 cycles per field.
static const uint32_t CYCLES_PER_FRAME = 14915;
static const uint32_t FRAME_US         = 16762;   // 60 Hz field period

// CoCo 256x192 active area centered in the 320x240 framebuffer.
static const int OX = (dvi::FB_WIDTH  - COCO_VDG_W) / 2;   // 32
static const int OY = (dvi::FB_HEIGHT - COCO_VDG_H) / 2;   // 24

// Scanline POINTER callback (native pixel mode): return the address of the
// framebuffer row and let the DMA read it directly, doubling each pixel in
// hardware. The per-scanline IRQ does essentially no work — no per-line fill to
// fall behind under core-0 emulation load, which is what was underrunning the
// HSTX FIFO and desyncing the link. Vertical 2x via active_line >> 1.
static const uint32_t *RAM_FUNC scanline_ptr_cb(uint32_t v_scanline, uint32_t active_line) {
    (void)v_scanline;
    return (const uint32_t *)g_fb.px[active_line >> 1];
}

// Classic (non-native) scanline callback: fill one 640-px active line from the
// 320-wide row, duplicating each pixel horizontally. Used by COCO_DVI_MODE,
// which cannot use the pointer/precomposed path — video_output_compose_service()
// early-returns when dvi_mode is set (video_output_rt.c:864), so the compose
// ring never builds and the two are not a supported combination.
// This is a PLAIN COPY, no palette lookup: the palette is applied in blit_frame()
// on core 0. A per-pixel lookup here overran the line budget historically.
// Byte-for-byte the callback display_test.cpp runs, which is stable on this sink.
static void RAM_FUNC scanline_cb(uint32_t v_scanline, uint32_t active_line, uint32_t *dst) {
    (void)v_scanline;
    const uint16_t *row = g_fb.px[active_line >> 1];
    for (uint32_t i = 0; i < dvi::FB_WIDTH; i++) {
        uint32_t px = row[i];
        dst[i] = px | (px << 16);
    }
}

// Compose the CoCo frame (nibble-packed indices) into g_fb as RGB565, centered.
// The palette lookup lives here (core 0), keeping scanline_cb a plain copy.
static void blit_frame() {
    const uint8_t *vb = coco_machine_get_vdg_buffer();
    for (int y = 0; y < COCO_VDG_H; y++) {
        const uint8_t *src = &vb[y * (COCO_VDG_W / 2)];
        uint16_t *dst = &g_fb.px[OY + y][OX];
        for (int x = 0; x < COCO_VDG_W; x++) {
            uint8_t idx = (x & 1) ? (src[x >> 1] >> 4) : (src[x >> 1] & 0x0F);
            dst[x] = g_pal[idx];
        }
    }
}

// - - - FRUITJAM-44: NeoPixel boot progress - - - - - - - - - - - - - - - - - -
// Cumulative progress bar across all five onboard pixels: each stage lights the
// next LED and leaves the earlier ones lit, so the strip fills as boot proceeds. Because it is cumulative, a boot
// that HANGS leaves the last completed step lit, so the strip says where it
// stopped with no serial attached — the boot-phase counterpart to the
// FRUITJAM-35 heartbeat LED, which only starts once loop() runs.
// Safe to bit-bang (the driver disables interrupts): every write happens before
// core 1 is launched, and nothing in the run loop touches the strip.
// FRUITJAM-53: heap-allocated and DESTROYED before core 1 launches. begin()
// claims a PIO state machine for GPIO32 and holds it for the life of the object;
// leaving it claimed cost ~15 HSTX desyncs per 3 minutes (bisected: 0 on
// aaa5593/1525a48, 15 on a031ea2, 0 again with the strip disabled). The
// destructor calls rp2040releasePIO(), handing the SM back. The WS2812s latch
// their last colour, so the finished boot pattern stays lit with nothing driving
// it — which is exactly what a boot-progress display wants anyway.
static Adafruit_NeoPixel *g_pixels = nullptr;

// All five pixels, filling left to right as boot proceeds. The three you named
// keep their positions and colours — LED 1 red at power-on, LED 3 green at SD
// mount, LED 5 blue at completion — and LEDs 2 and 4 fill the gaps, so the strip
// sweeps red -> blue as a progress bar.
//
// Stage points chosen from MEASURED boot timing rather than code order, so the
// five light at roughly even intervals (monitor attached, ms from reset):
//   LED 1  ~47   power + 252 MHz clock + PSRAM retune
//   LED 2  ~568  USB host up   (even-split target ~373)
//   LED 3  ~818  SD mounted    (target ~747)
//   LED 4  ~1093 machine ready (target ~1120)
//   LED 5  ~1493 boot complete (target ~1493)
// CAVEAT: with NO serial host attached the ready-wait in setup() runs its full
// 1500 ms instead of the ~430 ms measured here, so LED 1 -> LED 2 stretches by
// about a second. Everything after LED 2 keeps these intervals.
static const uint8_t BOOT_PIX_CLOCK   = 0;   // LED 1 — red
static const uint8_t BOOT_PIX_USB     = 1;   // LED 2 — chartreuse
static const uint8_t BOOT_PIX_SD      = 2;   // LED 3 — green
static const uint8_t BOOT_PIX_MACHINE = 3;   // LED 4 — blue
static const uint8_t BOOT_PIX_DONE    = 4;   // LED 5 — indigo

static void boot_pixel(uint8_t idx, uint8_t r, uint8_t g, uint8_t b) {
#if !COCO_NEOPIXEL
    (void)idx; (void)r; (void)g; (void)b; return;
#else
    if (!g_pixels || idx >= NUM_NEOPIXEL) return;
    g_pixels->setPixelColor(idx, g_pixels->Color(r, g, b));
    g_pixels->show();
#endif
}

// Has BASIC finished its cold start and reached the "OK" prompt?
// Detected from the 32x16 text screen at $0400 rather than from a ROM address:
// measured PC sits in the command-input loop at $A7D3/$A7D5, but those belong to
// Disk Extended Color BASIC 1.1 specifically, and this port boots Color,
// Extended+Color or Disk depending on what is on the card. Screen text is the
// same in all three. Measured timeline (probe, FRUITJAM-46): f=0 screen is
// uninitialised 0x00; f=30-60 blank while BASIC clears RAM (PC=$A089); f=90
// banner + OK present. So this fires around f=60-90, roughly a second earlier
// than the fixed 150-field AUTO.BIN timer assumes.
// VDG screen codes: glyph = ch & 0x3F, and 0x00-0x1F map to '@'..'_', so
// 'O' = 0x0F and 'K' = 0x0B.
static bool basic_at_prompt(void) {
    const uint8_t *scr = coco_machine_peek_ram(0x0400);
    if (!scr) return false;
    for (int row = 0; row < 16; row++) {
        const uint8_t *r = &scr[row * 32];
        if ((r[0] & 0x3F) == 0x0F && (r[1] & 0x3F) == 0x0B) return true;   // "OK"
    }
    return false;
}

// Slow hue rotation across all five pixels once BASIC is up. Each pixel is offset
// by a fifth of the wheel so the strip reads as a drifting rainbow rather than
// five LEDs blinking in unison.
// Cost is small but not free: show() bit-bangs with interrupts disabled on THIS
// core for ~150 us (5 px x 24 bits). At the 80 ms interval below that is ~0.2% of
// core 0, which matters because core 0 is the constrained one (FRUITJAM-38).
// Core 1's video IRQ is unaffected — interrupt masking is per-core on RP2350.
// DISABLED (FRUITJAM-47). Measured: enabling this desynced the HSTX link within
// ~100 fields and kept it there — 9 desync events / 8 resyncs in 14 s, video_frames
// running at 159-161 fps instead of 60. Isolation run with the cycle off and
// everything else identical: 0 desyncs, steady 59-60 fps. The cause is show():
// Adafruit's RP2 backend drives WS2812 from PIO via pio_sm_put_blocking, spinning
// core 0 for ~450 us per update while core 1 feeds the HSTX FIFO — the same
// underrun-to-desync path as FRUITJAM-13/-37. A rarer interval does not fix it;
// every update risks a glitch. Needs a DMA-fed writer so core 0 never spins.
#define NEO_IDLE_CYCLE       0
#define NEO_IDLE_INTERVAL_MS 80
#define NEO_IDLE_HUE_STEP    256      // of 65536 -> full wheel in ~20 s

static void neo_idle_cycle(void) {
    static uint16_t hue = 0;
    for (uint8_t i = 0; i < NUM_NEOPIXEL; i++) {
        uint16_t h = (uint16_t)(hue + (uint32_t)i * (65536UL / NUM_NEOPIXEL));
        g_pixels->setPixelColor(i, Adafruit_NeoPixel::gamma32(
            Adafruit_NeoPixel::ColorHSV(h, 255, 255)));
    }
    g_pixels->show();
    hue += NEO_IDLE_HUE_STEP;
}


// - - - FRUITJAM-50: button disk picker - - - - - - - - - - - - - - - - - - - -
// Button 2 = previous, button 3 = next, button 1 = mount + close. Pressing 2 or
// 3 while closed opens the picker, so there is no separate "open" gesture — the
// equivalent of the amoled port's swipe-up.
// Mounting is a DISK SWAP, not a reset: coco_machine_mount_dsk() swaps the image
// pointer and re-derives JVC geometry, exactly what changing a floppy does. Disk
// BASIC keeps running; DIR shows the new disk.
#define PICKER_MAX 128   // 128 x 13 B name table; card had >32

// Defined further down (with the other SD loaders / the PSRAM allocator block);
// forward-declared here so the picker can sit next to the rest of the board UI.
static size_t load_psram_file(const char *path, const uint8_t **out);
void __psram_free(void *);   // C++ linkage, matching the core's psram.h

static char     g_dsk_names[PICKER_MAX][13];
static int      g_dsk_count = 0;
static int      g_dsk_cur   = -1;      // index currently mounted, -1 = none
static int      g_pick_sel  = 0;
static bool     g_pick_open = false;
static uint8_t *g_dsk_img   = nullptr; // PSRAM image backing the mounted disk
static char     g_pick_msg[40] = "";
static bool     g_pick_dirty = false;   // redraw the overlay only when it changes

// Buttons are active-low with internal pull-ups. Note button 1 is GPIO0, which
// is also USB-BOOT — sampled only at reset, so it is free to use at runtime, but
// never tell a user to hold it while power-cycling.
struct Btn { uint8_t pin; bool last; uint32_t t; };
static Btn g_btn[3] = { { PIN_BUTTON1, true, 0 },
                        { PIN_BUTTON2, true, 0 },
                        { PIN_BUTTON3, true, 0 } };

static bool btn_fell(Btn &b) {
    bool now = digitalRead(b.pin);
    uint32_t ms = millis();
    if (now != b.last && (ms - b.t) > 25) {
        b.last = now; b.t = ms;
        if (!now) return true;          // high -> low = press
    }
    return false;
}

static void scan_dsk_dir(void) {
    DIR d; FILINFO fno;
    g_dsk_count = 0;
    if (f_opendir(&d, "0:/coco/dsk") != FR_OK) return;
    while (g_dsk_count < PICKER_MAX) {
        if (f_readdir(&d, &fno) != FR_OK || fno.fname[0] == 0) break;
        if (fno.fattrib & (AM_DIR | AM_HID | AM_SYS)) continue;
        // Skip dotfiles: macOS writes an AppleDouble "._NAME.DSK" beside every
        // real file on a FAT card, and those match *.dsk but are metadata, not
        // disk images. Also covers .DS_Store, .Spotlight-V100 and friends.
        if (fno.fname[0] == '.') continue;
        const char *ext = strrchr(fno.fname, '.');
        if (!ext || strcasecmp(ext, ".dsk") != 0) continue;
        strncpy(g_dsk_names[g_dsk_count], fno.fname, 12);
        g_dsk_names[g_dsk_count][12] = '\0';
        g_dsk_count++;
    }
    // Never truncate silently: say so if the card holds more than we can list.
    if (g_dsk_count == PICKER_MAX)
        Serial.printf("[picker: hit PICKER_MAX=%d, further .dsk ignored] ", PICKER_MAX);
    f_closedir(&d);
}

// Load image i into PSRAM and hand it to the FDC, freeing the previous one.
// Blocks core 0 for the length of the SD read (~160 KB) — see FRUITJAM-50.
static bool mount_dsk_index(int i) {
    if (i < 0 || i >= g_dsk_count) return false;
    char path[48];
    snprintf(path, sizeof(path), "0:/coco/dsk/%s", g_dsk_names[i]);
    const uint8_t *img = nullptr;
    size_t len = load_psram_file(path, &img);
    if (!len) { snprintf(g_pick_msg, sizeof(g_pick_msg), "LOAD FAILED"); return false; }
    if (g_dsk_img) __psram_free(g_dsk_img);
    g_dsk_img = (uint8_t *)img;
    g_dsk_cur = i;
    coco_machine_mount_dsk(g_dsk_img, len);
    snprintf(g_pick_msg, sizeof(g_pick_msg), "MOUNTED %s", g_dsk_names[i]);
    return true;
}

// - - - framebuffer text overlay (VDG font, 8x12 -> 40x20 cells) - - - - - - - -
// Reuses font_6847, already linked for the emulator's own text rendering
// (FRUITJAM-42), so the picker needs no font of its own.
extern "C" const uint8_t font_6847[];

// Cells are relative to the CoCo's 256x192 ACTIVE AREA (origin OX,OY), not to the
// 320x240 framebuffer. Two reasons, and the second is the important one:
//   1. it lines the overlay up with the emulated text the user is looking at;
//   2. blit_frame() repaints exactly this region every field, so the overlay
//      ERASES ITSELF the moment we stop drawing it. Anything drawn outside the
//      active area lands in the border, which nothing ever repaints, and would
//      stay on screen after the picker closes.
// So the grid is 32x16 cells, not 40x20.
#define OVL_COLS (COCO_VDG_W / 8)    // 32
#define OVL_ROWS (COCO_VDG_H / 12)   // 16

static void fb_char(int cx, int cy, char c, uint16_t fg, uint16_t bg) {
    if (c >= 'a' && c <= 'z') c -= 32;
    uint8_t idx;
    if (c >= '@' && c <= '_')      idx = (uint8_t)(c - '@');        // 0x00-0x1F
    else if (c >= ' ' && c <= '?') idx = (uint8_t)c;                // 0x20-0x3F
    else                           idx = 0x20;                      // space
    if (cx < 0 || cy < 0 || cx >= OVL_COLS || cy >= OVL_ROWS) return;
    int px = OX + cx * 8, py = OY + cy * 12;
    for (int r = 0; r < 12; r++) {
        uint8_t bits = font_6847[idx * 12 + r];
        uint16_t *row = &g_fb.px[py + r][px];
        for (int b = 0; b < 8; b++) row[b] = (bits & (0x80 >> b)) ? fg : bg;
    }
}

static void fb_text(int cx, int cy, const char *s, uint16_t fg, uint16_t bg) {
    for (int i = 0; s[i]; i++) fb_char(cx + i, cy, s[i], fg, bg);
}

// Drawn over the emulator frame each field while the picker is open.
static void draw_picker(void) {
    const uint16_t fg  = g_pal[0];    // reuse the machine palette
    const uint16_t hi  = g_pal[8];
    const uint16_t bg  = g_pal[9];
    const int      COLS = OVL_COLS;             // 32, the CoCo text width
    const int      rows_visible = 12;

    char line[OVL_COLS + 1];
    for (int i = 0; i < COLS; i++) line[i] = ' ';
    line[COLS] = '\0';

    // Paint the full 32x16 panel, not just the rows with text: any cell left
    // unpainted would still hold the last blitted emulator frame and show through.
    for (int y = 0; y < OVL_ROWS; y++)
        for (int x = 0; x < OVL_COLS; x++) fb_char(x, y, ' ', fg, bg);

    int top = g_pick_sel - rows_visible / 2;
    if (top > g_dsk_count - rows_visible) top = g_dsk_count - rows_visible;
    if (top < 0) top = 0;

    fb_text(0, 0, "SELECT DISK", fg, bg);
    fb_text(0, 1, "UP/DN=2,3 ENTER=1 F12=CANCEL", fg, bg);

    for (int r = 0; r < rows_visible; r++) {
        int i = top + r;
        for (int c = 0; c < COLS; c++) line[c] = ' ';
        if (i < g_dsk_count) {
            const char *mark = (i == g_dsk_cur) ? "*" : " ";
            snprintf(line, sizeof(line), "%s%-12s%s", (i == g_pick_sel) ? ">" : " ",
                     g_dsk_names[i], mark);
        }
        fb_text(0, 2 + r, line, (i == g_pick_sel) ? bg : fg, (i == g_pick_sel) ? hi : bg);
    }
    if (g_dsk_count == 0) fb_text(0, 2, "NO .DSK FILES IN /COCO/DSK", fg, bg);
    if (g_pick_msg[0])    fb_text(0, 2 + rows_visible + 1, g_pick_msg, fg, bg);
}

// Poll the buttons and drive the picker. Called once per field from loop().
static void picker_task(void) {
    bool sel  = btn_fell(g_btn[0]);   // button 1
    bool prev = btn_fell(g_btn[1]);   // button 2
    bool next = btn_fell(g_btn[2]);   // button 3
    if (!sel && !prev && !next) return;

    if (Serial && Serial.availableForWrite() >= 48)
        Serial.printf("[btn] %s%s%s open=%d sel=%d/%d\n",
                      sel ? "1" : "", prev ? "2" : "", next ? "3" : "",
                      (int)g_pick_open, g_pick_sel, g_dsk_count);

    if (prev || next) {
        if (!g_pick_open) { g_pick_open = true; g_pick_msg[0] = '\0'; g_pick_dirty = true; return; }
        g_pick_dirty = true;
        if (g_dsk_count) {
            g_pick_sel += next ? 1 : -1;
            if (g_pick_sel < 0)             g_pick_sel = g_dsk_count - 1;
            if (g_pick_sel >= g_dsk_count)  g_pick_sel = 0;
        }
        return;
    }
    if (sel) {
        if (!g_pick_open) { g_pick_open = true; g_pick_msg[0] = '\0'; g_pick_dirty = true; return; }
        mount_dsk_index(g_pick_sel);
        g_pick_open = false;
    }
}

static FATFS g_fs;
static bool mount_sd() {
    for (int a = 0; a < 5; a++) {
        if (f_mount(&g_fs, "0:", 1) == FR_OK) return true;
        delay(200);
    }
    return false;
}
static size_t load_rom_at(const char *path, size_t off, size_t max) {
    FIL f;
    if (f_open(&f, path, FA_READ) != FR_OK) return 0;
    UINT br = 0;
    f_read(&f, g_rom + off, max, &br);
    f_close(&f);
    return br;
}

// Cassette image (FRUITJAM-28): loaded from SD into PSRAM (the cold/bulk home,
// FRUITJAM-08 policy) and handed to the machine's tape feeder. PSRAM is retuned
// for 252 MHz in setup() (psram_reinit_timing) before this runs.
extern size_t __psram_size;
void  *__psram_malloc(size_t);
void   __psram_free(void *);
void   psram_reinit_timing(uint32_t hz);

// Disk BASIC cartridge ROM (FRUITJAM-29): executes at $C000, so it lives in
// SRAM (hot path), unlike the bulk cassette image. Needs a 16 KB Extended+Color
// main ROM (bas.rom) for Disk Extended Color BASIC to actually come up.
static uint8_t g_cart[8192];
static size_t load_cart_rom(const char *path) {
    FIL f;
    if (f_open(&f, path, FA_READ) != FR_OK) return 0;
    UINT br = 0;
    f_read(&f, g_cart, sizeof(g_cart), &br);
    f_close(&f);
    return br;
}

// Load a whole file into a fresh PSRAM buffer (tape/binary images: read in bulk,
// cold path). Returns the buffer via *out and its size, or 0.
// Chunk size for runtime PSRAM loads — small enough to leave the bus free, large
// enough that per-call FatFs overhead stays negligible.
#define PSRAM_READ_CHUNK 4096

static size_t load_psram_file(const char *path, const uint8_t **out) {
    FIL f;
    if (f_open(&f, path, FA_READ) != FR_OK) return 0;
    size_t sz = f_size(&f);
    if (sz == 0 || sz > (1u << 20)) { f_close(&f); return 0; }
    uint8_t *buf = (uint8_t *)__psram_malloc(sz);
    if (!buf) { f_close(&f); return 0; }
    // Read in chunks rather than one 160 KB burst (FRUITJAM-50). At BOOT this is
    // irrelevant — core 1 is not running yet — but the picker loads disks at
    // RUNTIME, and a single large read starves the video path: the transfer
    // hammers the QMI, which also serves XIP flash, so any pico_hdmi code not in
    // RAM stalls behind it. That corrupts TMDS WITHOUT changing the frame count,
    // so fps stays 60 and the desync watchdog never fires while the sink quietly
    // drops lock. Diagnosis: after a mount the CPU, screen RAM and framebuffer
    // were all verified intact (fb_nonblack=768/768) while the monitor was black.
    // Chunking leaves gaps for XIP and the HSTX DMA to catch up.
    UINT br = 0, total = 0;
    while (total < sz) {
        UINT want = (UINT)((sz - total > PSRAM_READ_CHUNK) ? PSRAM_READ_CHUNK : sz - total);
        if (f_read(&f, buf + total, want, &br) != FR_OK || br == 0) break;
        total += br;
    }
    br = total;
    f_close(&f);
    if (br != sz) { __psram_free(buf); return 0; }
    *out = buf;
    return sz;
}

static const uint8_t *g_cas_img = nullptr;
static size_t load_cas(const char *path) { return load_psram_file(path, &g_cas_img); }

// DECB .bin auto-run (FRUITJAM-19): loaded at boot, injected + EXEC'd after BASIC
// settles (its cold-boot RAM clear would otherwise wipe the payload).
static const uint8_t *g_bin_img = nullptr;
static size_t         g_bin_len = 0;

// - - - CoCo audio -> TLV320 DAC over I2S (FRUITJAM-13) ------------------------
// The machine produces 48 kHz mono PCM from its PIA sound tap (coco_machine
// render_audio); this side is the sink: the TLV320DAC3100 configured over I2C0
// exactly as the FRUITJAM-07 bring-up, fed by the earlephilhower PIO-I2S master.
//
// Producer (this core, once per field) and the I2S DMA (hardware) share the I2S
// library's ring: we write each field's samples non-blocking, then top the ring
// back up to its half mark with the last sample (hold-last) so a late field
// can't underrun into a click. The 2048-frame ring (~43 ms) spans >2 fields, so
// the once-per-field feed keeps the DMA fed through the pacing delay.
#define PIN_I2C_SDA    20
#define PIN_I2S_DATA   24
#define PIN_I2S_BCLK   26    // LRCLK = BCLK+1 = 27
#define PIN_PERIPH_RST 22    // shared TLV320 + ESP32-C6 reset (pulse once)

// 0 = headphone jack only (feeding an external amp); 1 = onboard class-D speaker.
#define SPEAKER_OUTPUT 0

static Adafruit_TLV320DAC3100 g_codec;
static I2S                    g_i2s(OUTPUT);

// Ring geometry: setBuffers(16, 128) = 2048 stereo frames = 8192 bytes. Keep the
// ring ~half full: pad up to HALF_BYTES free with hold-last, bounding latency to
// ~21 ms while leaving room for the next field's samples.
static const int I2S_HALF_BYTES = 4096;

static int16_t g_audio_buf[1024];   // one field is ~805 samples
static int16_t g_audio_last = 0;    // hold-last value on underrun
static int     g_audio_peak = 0;    // loudest |sample| since last report (level tuning)

static inline void i2s_put(int16_t s) {
    g_audio_last = s;
    g_i2s.write((int32_t)(((uint32_t)(uint16_t)s << 16) | (uint16_t)s), false);  // L=R, non-blocking
}

static bool configure_codec() {
    if (!g_codec.begin()) { Serial.println("codec.begin() FAILED"); return false; }
    delay(10);
    bool ok = true;
    ok &= g_codec.setCodecInterface(TLV320DAC3100_FORMAT_I2S, TLV320DAC3100_DATA_LEN_16);
    ok &= g_codec.setCodecClockInput(TLV320DAC3100_CODEC_CLKIN_PLL);
    ok &= g_codec.setPLLClockInput(TLV320DAC3100_PLL_CLKIN_BCLK);
    ok &= g_codec.setPLLValues(1, 2, 32, 0);       // P=1 R=2 J=32 D=0
    ok &= g_codec.setNDAC(true, 8);
    ok &= g_codec.setMDAC(true, 2);
    ok &= g_codec.powerPLL(true);
    ok &= g_codec.setDACDataPath(true, true, TLV320_DAC_PATH_NORMAL,
                                 TLV320_DAC_PATH_NORMAL, TLV320_VOLUME_STEP_1SAMPLE);
    ok &= g_codec.configureAnalogInputs(TLV320_DAC_ROUTE_MIXER, TLV320_DAC_ROUTE_MIXER,
                                        false, false, false, false);
    ok &= g_codec.setDACVolumeControl(false, false, TLV320_VOL_INDEPENDENT);
    ok &= g_codec.setChannelVolume(false, 18);     // left  +0 dB
    ok &= g_codec.setChannelVolume(true, 18);      // right +0 dB
    ok &= g_codec.configureHeadphoneDriver(true, true, TLV320_HP_COMMON_1_35V, false);
    ok &= g_codec.configureHPL_PGA(0, true);
    ok &= g_codec.configureHPR_PGA(0, true);
    ok &= g_codec.setHPLVolume(true, 6);
    ok &= g_codec.setHPRVolume(true, 6);
    // Internal speaker DISABLED: output goes to the 3.5mm headphone jack only
    // (feeding an external amp). Set SPEAKER_OUTPUT to 1 to use the onboard
    // speaker instead.
    //
    // Auto-mute-on-insert via the TLV320's headset detection (setHeadsetDetect /
    // getHeadsetStatus, IRQ on GPIO23) was tried and did NOT work for a line
    // cable into an amp: the detector is impedance-based and reads a high-Z amp
    // input as "nothing plugged". Revisit only if real (low-Z) headphone use is
    // wanted AND it's confirmed the Fruit Jam wires the detection usefully.
    ok &= g_codec.enableSpeaker(SPEAKER_OUTPUT);
    if (SPEAKER_OUTPUT) {
        ok &= g_codec.configureSPK_PGA(TLV320_SPK_GAIN_6DB, true);
        ok &= g_codec.setSPKVolume(true, 0);
    }
    return ok;
}

// Bring up the TLV320 + I2S master and prime the ring with silence. Mirrors the
// FRUITJAM-07 sequencing: pulse the shared reset, I2C config, I2S master up with
// BCLK live *before* the DAC PLL tries to lock.
static bool audio_init() {
    pinMode(PIN_PERIPH_RST, OUTPUT);
    digitalWrite(PIN_PERIPH_RST, LOW);  delay(100);
    digitalWrite(PIN_PERIPH_RST, HIGH); delay(100);

    Wire.setSDA(PIN_I2C_SDA);
    Wire.setSCL(PIN_I2C_SDA + 1);       // SCL = 21
    Wire.begin();

    g_i2s.setDATA(PIN_I2S_DATA);
    g_i2s.setBCLK(PIN_I2S_BCLK);        // LRCLK = 27
    g_i2s.setBitsPerSample(16);
    g_i2s.setFrequency(48000);
    g_i2s.setBuffers(16, 128);          // 2048 frames ring
    if (!g_i2s.begin()) { Serial.println("I2S begin FAILED"); return false; }

    // Prime to the half mark with silence so the DMA has a cushion before the
    // first field's samples arrive.
    while (g_i2s.availableForWrite() > I2S_HALF_BYTES) i2s_put(0);

    return configure_codec();
}

// Per-field feed: push this field's PCM, then hold-last-pad back to the half mark.
static void audio_feed() {
    int n = coco_machine_render_audio(g_audio_buf, (int)(sizeof(g_audio_buf) / sizeof(g_audio_buf[0])));
    int field_peak = 0;
    for (int i = 0; i < n; i++) {
        int a = g_audio_buf[i] < 0 ? -g_audio_buf[i] : g_audio_buf[i];
        if (a > field_peak) field_peak = a;
        if (a > g_audio_peak) g_audio_peak = a;
        if (g_i2s.availableForWrite() < 4) break;   // ring full (overrun) — drop rest
        i2s_put(g_audio_buf[i]);
    }
    (void)field_peak;
    // Refill toward half with the last sample so a starved DMA repeats rather
    // than clicking to silence (hold-last on underrun).
    while (g_i2s.availableForWrite() > I2S_HALF_BYTES) i2s_put(g_audio_last);
}

// - - - USB HID keyboard -> CoCo matrix (FRUITJAM-12) --------------------------
// PIO-USB host behind the CH334F hub (proven in FRUITJAM-05). Reports are diffed
// per frame and driven into the machine's PIA0 matrix via coco_machine_press/
// release_key. Implemented fresh; the mapping knowledge is ported from pizero.
#define HOST_PIN_DP 1
#define USB_5V_EN   11

static Adafruit_USBH_Host USBHost;

// HID boot-keyboard usage code -> CoCo DSCAN. 0xFF = no mapping.
static uint8_t g_hid_to_dscan[256];

static void hid_map_init(void) {
    memset(g_hid_to_dscan, 0xFF, sizeof(g_hid_to_dscan));
    for (int i = 0; i < 26; i++) g_hid_to_dscan[0x04 + i] = (uint8_t)(DSCAN_A + i);  // a-z
    static const uint8_t digit[10] = {              // HID 0x1E..0x27 = 1..9,0
        DSCAN_1, DSCAN_2, DSCAN_3, DSCAN_4, DSCAN_5,
        DSCAN_6, DSCAN_7, DSCAN_8, DSCAN_9, DSCAN_0 };
    for (int i = 0; i < 10; i++) g_hid_to_dscan[0x1E + i] = digit[i];
    g_hid_to_dscan[0x28] = DSCAN_ENTER;     // Enter
    g_hid_to_dscan[0x29] = DSCAN_BREAK;     // Esc  -> BREAK
    g_hid_to_dscan[0x2A] = DSCAN_LEFT;      // Backspace -> Left (CoCo rubout)
    g_hid_to_dscan[0x2B] = DSCAN_RIGHT;     // Tab -> Right
    g_hid_to_dscan[0x2C] = DSCAN_SPACE;     // Space
    g_hid_to_dscan[0x2D] = DSCAN_MINUS;     // -
    g_hid_to_dscan[0x33] = DSCAN_SEMICOLON; // ;
    g_hid_to_dscan[0x34] = DSCAN_COLON;     // ' -> : (CoCo has a dedicated colon)
    g_hid_to_dscan[0x36] = DSCAN_COMMA;     // ,
    g_hid_to_dscan[0x37] = DSCAN_FULL_STOP; // .
    g_hid_to_dscan[0x38] = DSCAN_SLASH;     // /
    g_hid_to_dscan[0x2F] = DSCAN_AT;        // [ -> @ (CoCo @ key)
    g_hid_to_dscan[0x4C] = DSCAN_CLEAR;     // Delete -> CLEAR
    g_hid_to_dscan[0x4F] = DSCAN_RIGHT;     // arrows
    g_hid_to_dscan[0x50] = DSCAN_LEFT;
    g_hid_to_dscan[0x51] = DSCAN_DOWN;
    g_hid_to_dscan[0x52] = DSCAN_UP;
    g_hid_to_dscan[0x29] = DSCAN_BREAK;
}

static uint8_t g_prev_codes[6] = {0};
static bool    g_shift_prev = false;

// HID modifier bits: L/R Ctrl 0x01/0x10, L/R Alt 0x04/0x40, L/R Shift 0x02/0x20.
static void hid_keyboard_apply(const uint8_t *report) {
    uint8_t mods = report[0];
    const uint8_t *codes = &report[2];

    // Reset chord: Ctrl+Alt+Delete (HID 0x4C) -> warm reset. Checked before the
    // matrix diff so the chord itself never leaks keys into BASIC.
    bool ctrl = mods & 0x11, alt = mods & 0x44;
    for (int i = 0; i < 6; i++) {
        if (codes[i] == 0x4C && ctrl && alt) {
            coco_machine_reset();
            g_shift_prev = false; memset(g_prev_codes, 0, sizeof(g_prev_codes));
            return;
        }
    }

    // FRUITJAM-51: picker keys, checked before the matrix diff for the same reason
    // as the reset chord above — so they never leak through into BASIC.
    //   F12    toggle the picker open / cancel
    //   UP/DN  move the selection      ENTER  mount and close
    // These double up on buttons 2/3/1; either input works at any time.
    // While the picker is open the overlay is MODAL: every key is swallowed, so
    // typing cannot run away into BASIC behind an overlay the user is reading.
    {
        static uint8_t pk_prev[6] = { 0 };
        auto newly = [&](uint8_t code) {
            bool now = false, before = false;
            for (int i = 0; i < 6; i++) {
                if (codes[i]  == code) now    = true;
                if (pk_prev[i] == code) before = true;
            }
            return now && !before;
        };
        const bool k_f12 = newly(0x45), k_up = newly(0x52),
                   k_dn  = newly(0x51), k_ent = newly(0x28);
        memcpy(pk_prev, codes, 6);

        if (k_f12) {
            g_pick_open = !g_pick_open;
            g_pick_dirty = true;
            if (g_pick_open) {
                g_pick_msg[0] = '\0';
                // Drop anything the emulator currently thinks is held, or a key
                // down at the moment F12 arrives would stay stuck for the whole
                // time the picker is up.
                coco_machine_release_all_keys();
                g_shift_prev = false;
                memset(g_prev_codes, 0, sizeof(g_prev_codes));
            }
        }
        if (g_pick_open) {
            if ((k_up || k_dn) && g_dsk_count) {
                g_pick_sel += k_dn ? 1 : -1;
                if (g_pick_sel < 0)            g_pick_sel = g_dsk_count - 1;
                if (g_pick_sel >= g_dsk_count) g_pick_sel = 0;
                g_pick_dirty = true;
            }
            if (k_ent) {
                mount_dsk_index(g_pick_sel);
                g_pick_open = false;
            }
            return;               // modal: swallow everything else
        }
    }

    bool shift = (mods & 0x22) != 0;
    if (shift && !g_shift_prev) coco_machine_press_key(DSCAN_SHIFT);
    if (!shift && g_shift_prev) coco_machine_release_key(DSCAN_SHIFT);
    g_shift_prev = shift;

    // Releases: was in prev, not in current.
    for (int i = 0; i < 6; i++) {
        uint8_t p = g_prev_codes[i];
        if (!p) continue;
        bool still = false;
        for (int j = 0; j < 6; j++) if (codes[j] == p) { still = true; break; }
        if (!still && g_hid_to_dscan[p] != 0xFF) coco_machine_release_key(g_hid_to_dscan[p]);
    }
    // Presses: in current, not in prev.
    for (int i = 0; i < 6; i++) {
        uint8_t c = codes[i];
        if (!c) continue;
        bool was = false;
        for (int j = 0; j < 6; j++) if (g_prev_codes[j] == c) { was = true; break; }
        if (!was && g_hid_to_dscan[c] != 0xFF) coco_machine_press_key(g_hid_to_dscan[c]);
    }
    memcpy(g_prev_codes, codes, 6);
}

extern "C" void tuh_hid_mount_cb(uint8_t daddr, uint8_t idx,
                                 uint8_t const *desc, uint16_t len) {
    (void)desc; (void)len;
    tuh_hid_receive_report(daddr, idx);
}
extern "C" void tuh_hid_umount_cb(uint8_t daddr, uint8_t idx) {
    (void)daddr; (void)idx;
    coco_machine_release_all_keys();     // drop stuck keys if the keyboard leaves
    memset(g_prev_codes, 0, sizeof(g_prev_codes));
    g_shift_prev = false;
}
extern "C" void tuh_hid_report_received_cb(uint8_t daddr, uint8_t idx,
                                           uint8_t const *report, uint16_t len) {
    // Only the boot-keyboard interface (8-byte reports). Others (consumer/etc.)
    // are ignored but must still be re-armed.
    if (tuh_hid_interface_protocol(daddr, idx) == HID_ITF_PROTOCOL_KEYBOARD && len >= 8)
        hid_keyboard_apply(report);
    tuh_hid_receive_report(daddr, idx);  // re-arm
}

// Deferred HSTX resync (freeze fix). The desync watchdog runs on core 0, but the
// video DMA-IRQ handler and all HSTX/DMA state live on core 1. Calling
// video_output_force_resync() from core 0 tears that state down while core 1's
// IRQ is still firing into it — and its irq_set_enabled(DMA_IRQ_1) only gates
// core 0's NVIC, not core 1's — a cross-core race that hard-locks the board.
// Instead core 0 just requests a resync; core 1 performs it from its background
// task, where disabling DMA_IRQ_1 actually holds off the local handler.
static volatile bool     g_want_resync  = false;
static volatile uint32_t g_resync_count = 0;

static void core1_background(void) {
    video_output_compose_service();
    if (g_want_resync) {
        g_want_resync = false;
        video_output_force_resync();   // safe here: runs on core 1, gates its own IRQ
        g_resync_count++;
    }
}

// Core 1: the pico_hdmi video engine (never returns). Launched manually from
// setup() after USB host init — see the multicore_launch_core1 note there.
static void core1_video_entry() {
    video_output_core1_run();
}

void setup() {
    vreg_set_voltage(VREG_VOLTAGE_1_25);
    delay(2);
    set_sys_clock_khz(252000, true);
    psram_reinit_timing(clock_get_hz(clk_sys));   // retune QMI PSRAM for 252 MHz (FRUITJAM-08)

    // FRUITJAM-44 step 0: powered, clocked, PSRAM retuned.
#if COCO_NEOPIXEL
    g_pixels = new Adafruit_NeoPixel(NUM_NEOPIXEL, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
    g_pixels->begin();
    g_pixels->setBrightness(40);   // gentle — these are bright at full scale
    g_pixels->clear();
    g_pixels->show();
#endif
    boot_pixel(BOOT_PIX_CLOCK, 255, 0, 0);    // LED 1 red

    // Host 5V on early so the CH334F hub PHY settles before the host starts.
    pinMode(USB_5V_EN, OUTPUT);
    digitalWrite(USB_5V_EN, HIGH);

    // Onboard LED (GPIO29, active-low) doubles as the FRUITJAM-35 per-core
    // liveness heartbeat driven from loop(). Not using the shared IR receiver.
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);   // off (active-low)

    // FRUITJAM-50: board buttons, active-low with internal pull-ups.
    pinMode(PIN_BUTTON1, INPUT_PULLUP);
    pinMode(PIN_BUTTON2, INPUT_PULLUP);
    pinMode(PIN_BUTTON3, INPUT_PULLUP);

    Serial.begin(115200);
    // FRUITJAM-45: 300 ms, down from 1500. This wait exists only to give a serial
    // monitor time to attach before the banner prints; nothing else happens during
    // it, and with no host attached it ran the full 1500 ms on EVERY boot — the
    // largest single lump in a ~1500 ms startup. Trade-off: a monitor measured
    // 430-520 ms to become ready, so captures may now miss the banner and the
    // earliest STAGE lines.
    // FRUITJAM-48: blink LED 1 red while we spin here, so this wait reads as
    // "alive, waiting for a serial host" instead of a board that might be hung.
    // Safe to drive the strip from this loop: core 1 has not been launched yet,
    // so there is no HSTX stream for the PIO write to starve (contrast
    // FRUITJAM-47, where the same call at RUNTIME desyncs video).
    const uint32_t start = millis();
    {
        bool     lit        = true;   // LED 1 was set solid red just above
        uint32_t next_blink = millis() + SERIAL_WAIT_BLINK_MS;
        while (!Serial && (millis() - start) < SERIAL_READY_WAIT_MS) {
            if ((int32_t)(millis() - next_blink) >= 0) {
                lit = !lit;
                boot_pixel(BOOT_PIX_CLOCK, lit ? 255 : 0, 0, 0);
                next_blink = millis() + SERIAL_WAIT_BLINK_MS;
            }
            delay(5);
        }
        boot_pixel(BOOT_PIX_CLOCK, 255, 0, 0);   // leave it solid red on exit
    }
    const uint32_t serial_wait = millis() - start;

    Serial.println();
    Serial.println("=== FRUITJAM-25 Boot Color BASIC -> DVI ===");
    Serial.printf("[%lu ms] serial-ready wait took %lu ms\n", millis(), (unsigned long)serial_wait);
    Serial.flush(); delay(50);

    // DMA arbitration: pico_hdmi hardcodes DMA channels 0 and 1, but the SD
    // driver grabs channels via dma_claim_unused_channel() at mount and would
    // take 0/1 first, making video_output_init()'s dma_channel_claim(0/1) panic
    // (silent hang). Reserve 0/1 now so SD skips them, then release them just
    // before video init so pico_hdmi can claim them as it expects.
    dma_channel_claim(0);
    dma_channel_claim(1);

    // USB host FIRST — before SD/video. In the standalone test begin() worked
    // when it ran on a clean machine; in the combined build it hung, because SD
    // or pico_hdmi init first grabs a hardware resource PIO-USB needs in begin()
    // (a hardware alarm for alarm_pool_create). Init it up front so it gets that
    // resource. DMA 0/1 are reserved above for pico_hdmi, so PIO-USB's
    // dma_claim_unused takes 2/3; SD then takes 4/5; video reclaims 0/1.
    Serial.printf("[%lu ms] ", millis()); Serial.print("STAGE usb host... "); Serial.flush(); delay(20);
    hid_map_init();
    pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
    pio_cfg.pin_dp     = HOST_PIN_DP;
    pio_cfg.pio_tx_num = 0;
    pio_cfg.pio_rx_num = 0;
    // PIO-USB's begin() does dma_claim_mask(1 << tx_ch); its default tx_ch is 0,
    // which collides with pico_hdmi's hardcoded DMA 0 (reserved above) and hard-
    // asserts (silent hang). Point it at a free channel: pico_hdmi=0/1, SD=2/3,
    // so USB TX = 4.
    pio_cfg.tx_ch = 4;
    USBHost.configure_pio_usb(1, &pio_cfg);
    tuh_hid_set_default_protocol(HID_PROTOCOL_BOOT);
    USBHost.begin(1);
    // Pulled off pure yellow toward green: WS2812 red reads hot, so (255,255,0)
    // sits warmer than it looks on paper (same effect that made #4B0082 magenta).
    boot_pixel(BOOT_PIX_USB, 128, 255, 0);    // FRUITJAM-44: USB host up — LED 2 chartreuse
    Serial.println("ok"); Serial.flush(); delay(20);

    Serial.printf("[%lu ms] ", millis()); Serial.print("STAGE mount SD... "); Serial.flush(); delay(20);
    if (!mount_sd()) { Serial.println("FATAL: no SD"); while (1) delay(500); }
    boot_pixel(BOOT_PIX_SD, 0, 255, 0);       // FRUITJAM-44 step 2: SD mounted — LED 3 green
    Serial.println("ok"); Serial.flush(); delay(20);

    Serial.printf("[%lu ms] ", millis()); Serial.print("STAGE load ROM... "); Serial.flush(); delay(20);
    // Prefer 16 KB Extended+Color BASIC (Extended enables PLAY and is required by
    // Disk BASIC): extbas11 -> $8000-$9FFF, bas12 -> $A000-$BFFF. Fall back to
    // Color BASIC alone if Extended isn't present.
    size_t ext = load_rom_at("0:/coco/roms/extbas11.rom", 0, 8192);
    size_t col = load_rom_at("0:/coco/roms/bas12.rom", 8192, 8192);
    size_t got = (ext == 8192 && col == 8192) ? 16384 : 0;
    if (!got) got = load_rom_at("0:/coco/roms/bas12.rom", 0, sizeof(g_rom));
    if (got != 8192 && got != 16384) {
        Serial.printf("FATAL: bad ROM size %u\n", (unsigned)got);
        while (1) delay(500);
    }
    Serial.printf("ok (%u bytes, %s)\n", (unsigned)got,
                  got == 16384 ? "Extended+Color" : "Color"); Serial.flush(); delay(20);

    Serial.printf("[%lu ms] ", millis()); Serial.print("STAGE machine init... "); Serial.flush(); delay(20);
    if (!coco_machine_init(g_rom, got)) { Serial.println("FATAL: machine init"); while (1) delay(500); }
    // Optional cassette: drop a .cas at 0:/coco/tapes/AUTO.CAS, then CLOAD in BASIC.
    {
        size_t cas = load_cas("0:/coco/tapes/AUTO.CAS");
        if (cas) { coco_machine_cas_load(g_cas_img, cas);
                   Serial.printf("[cassette AUTO.CAS loaded: %u bytes -> CLOAD to run] ", (unsigned)cas); }
    }
    // Optional Disk BASIC cartridge (FRUITJAM-29): needs a 16 KB Ext+Color main
    // ROM to boot Disk Extended Color BASIC. Disk I/O (FDC engine) still TODO.
    {
        size_t dsk = load_cart_rom("0:/coco/roms/disk11.rom");
        if (dsk) { coco_machine_load_cart(g_cart, dsk);
                   Serial.printf("[disk cart: %u bytes] ", (unsigned)dsk); }
    }
    // Optional disk: mount a JVC .dsk (DIR/LOAD/RUN). AUTO.DSK preferred, else
    // fall back to coco.dsk for a first test. Mutable PSRAM buffer for writes.
    {
        const uint8_t *dsk_img = nullptr;
        size_t dsk = load_psram_file("0:/coco/dsk/AUTO.DSK", &dsk_img);
        if (!dsk) dsk = load_psram_file("0:/coco/roms/coco.dsk", &dsk_img);
        if (dsk) { g_dsk_img = (uint8_t *)dsk_img;   // FRUITJAM-50: the picker frees this on swap
                   coco_machine_mount_dsk(g_dsk_img, dsk);
                   Serial.printf("[disk mounted: %u bytes -> DIR/LOAD] ", (unsigned)dsk); }
        // FRUITJAM-50: enumerate what else is on the card for the button picker.
        scan_dsk_dir();
        for (int i = 0; i < g_dsk_count; i++)
            if (strcasecmp(g_dsk_names[i], "AUTO.DSK") == 0) { g_dsk_cur = g_pick_sel = i; break; }
        if (g_dsk_count) Serial.printf("[picker: %d .dsk] ", g_dsk_count);
    }
    // Optional: drop a DECB .bin at 0:/coco/bin/AUTO.BIN to auto-run it on boot.
    g_bin_len = load_psram_file("0:/coco/bin/AUTO.BIN", &g_bin_img);
    if (g_bin_len) Serial.printf("[AUTO.BIN queued: %u bytes -> auto-EXEC after boot] ",
                                 (unsigned)g_bin_len);
    // Match the resampler cadence to our real-time pacing so the I2S ring neither
    // starves nor overflows (see coco_machine_audio_init).
    coco_machine_audio_init(CYCLES_PER_FRAME, FRAME_US);
    boot_pixel(BOOT_PIX_MACHINE, 0, 0, 255);   // FRUITJAM-44: machine ready — LED 4 blue
    Serial.println("ok"); Serial.flush(); delay(20);

    Serial.printf("[%lu ms] ", millis()); Serial.print("STAGE audio (TLV320+I2S)... "); Serial.flush(); delay(20);
    if (audio_init()) Serial.println("ok");
    else              Serial.println("FAILED (continuing, video only)");
    Serial.flush(); delay(20);

    Serial.printf("[%lu ms] ", millis()); Serial.print("STAGE fill fb... "); Serial.flush(); delay(20);
    g_fb.fill(g_pal[8]);   // black border
    Serial.println("ok"); Serial.flush(); delay(20);

    Serial.printf("[%lu ms] ", millis()); Serial.print("STAGE video init... "); Serial.flush(); delay(20);
    dma_channel_unclaim(0);   // hand 0/1 back for pico_hdmi to claim (SD now on 2/3)
    dma_channel_unclaim(1);
#if COCO_DVI_MODE
    // DVI mode: no data islands (this monitor will not sync to them — FRUITJAM-37).
    // MUST use the classic path: compose_service() early-returns on dvi_mode, so
    // the precomposed ring never builds; forcing them together desyncs the link
    // (~160 fps, a resync every second). Init at the FULL 640x480 output size and
    // let the callback do the 320->640 duplication, exactly as display_test does.
    video_output_init(dvi::H_ACTIVE, dvi::V_ACTIVE);    // 640x480 OUTPUT size
    video_output_set_dvi_mode(true);
    // Native pixel mode WITHOUT the compose ring: in DVI mode the active line is
    // the static vactive_line_dvi list, which never touches the ring (the ring
    // only carries audio islands), so the pointer path works here — but ONLY if
    // init used the full 640 output width. vactive_line_dvi[8] is
    // HSTX_CMD_TMDS | rt_h_active_pixels; init at 320 emits a half-length line
    // against a 320-halfword DMA the expander doubles to 640, which desyncs the
    // link at ~160 fps (the first FRUITJAM-37 attempt). At 640 the command list
    // and the DMA agree, and the per-line ISR is a pointer handoff again — no
    // 320-word copy stealing SRAM bandwidth from core-0 emulation.
    video_output_set_native_pixel_mode(true);
    video_output_set_scanline_pointer_callback(scanline_ptr_cb);
    video_output_set_background_task(core1_background); // services the resync request
#else
    // HDMI mode + pico_hdmi 2.0-beta native/precomposed path: init at the NATIVE
    // 320x240 source size, compose ring + native pixel mode + a pointer callback
    // returning the 320-wide row (HSTX doubles to 640 in hardware), with the
    // compose service as core-1 background work so per-line ISR cost stays ~1.5us
    // and can't be starved by core-0 emulation load. Required for HDMI audio
    // (FRUITJAM-14), but invisible on any sink that rejects data islands.
    video_output_init(dvi::FB_WIDTH, dvi::FB_HEIGHT);   // 320x240 native source
    video_output_set_dvi_mode(false);
    video_output_set_compose_ring(g_compose_ring, sizeof(g_compose_ring) / sizeof(g_compose_ring[0]));
    video_output_set_native_pixel_mode(true);
    video_output_set_scanline_pointer_callback(scanline_ptr_cb);
    video_output_set_background_task(core1_background);
#endif
    Serial.println("ok"); Serial.flush(); delay(20);

    Serial.printf("clk_sys=%lu clk_hstx=%lu MHz. Starting video on core 1.\n",
                  (unsigned long)(clock_get_hz(clk_sys) / 1000000),
                  (unsigned long)(clock_get_hz(clk_hstx) / 1000000));
    Serial.flush(); delay(20);

    // Launch core 1 LAST — after USB host init on core 0 — so PIO-USB's begin()
    // cross-core setup can't deadlock against a core 1 already running the video
    // DMA-IRQ loop (the pizero ordering). We drive core 1 manually rather than
    // via setup1(), which the framework would auto-launch too early.
    // FRUITJAM-53: hand the PIO state machine back BEFORE core 1 starts driving
    // HSTX. The LEDs keep their last colour (WS2812 latch); only the SM is freed.
    boot_pixel(BOOT_PIX_DONE, 40, 0, 200);
#if COCO_NEOPIXEL
    delete g_pixels;
    g_pixels = nullptr;
    // The destructor leaves the pin as INPUT. Five WS2812 data inputs on a
    // floating line is an avoidable noise source next to a marginal TMDS link —
    // drive it to the idle level instead.
    pinMode(PIN_NEOPIXEL, OUTPUT);
    digitalWrite(PIN_NEOPIXEL, LOW);
#endif

    multicore_launch_core1(core1_video_entry);
    Serial.printf("[%lu ms] ", millis()); Serial.println("STAGE core1 launched — entering loop."); Serial.flush(); delay(20);

}

// Core 0: emulate one field, compose, pace to ~60 Hz (resync-not-debt).
void loop() {
    static uint32_t deadline = 0;
    static uint32_t frames = 0, last_report = 0, run_us_acc = 0;

    USBHost.task();   // service PIO-USB host: fires the HID callbacks -> keys

    // Auto-run AUTO.BIN once BASIC has cold-booted and cleared RAM (~2.5 s),
    // then inject the payload and EXEC it. One-shot.
    static uint32_t boot_fields = 0;
    static bool     bin_launched = false;
    if (!bin_launched && g_bin_img && ++boot_fields >= 150) {
        uint16_t exec = coco_machine_load_bin(g_bin_img, g_bin_len);
        if (exec) { coco_machine_exec(exec); Serial.printf("[AUTO.BIN exec @ $%04X]\n", exec); }
        else        Serial.println("[AUTO.BIN: parse error / no exec block]");
        bin_launched = true;
    }

    // FRUITJAM-46: once BASIC reaches its prompt, hand the strip over from the
    // boot progress bar to a slow idle colour cycle.
    {
        static bool     basic_up   = false;
        static uint32_t next_neo   = 0;
        if (!basic_up) {
            if (basic_at_prompt()) {
                basic_up = true;
                if (Serial && Serial.availableForWrite() >= 48)
                    Serial.printf("[BASIC at prompt after %lu fields]\n",
                                  (unsigned long)frames);
            }
#if NEO_IDLE_CYCLE
        } else if (millis() >= next_neo) {
            neo_idle_cycle();
            next_neo = millis() + NEO_IDLE_INTERVAL_MS;
#endif
        }
    }

    picker_task();          // FRUITJAM-50: board buttons -> disk picker

    uint32_t t0 = micros();
    coco_machine_run_cycles(CYCLES_PER_FRAME);
    coco_machine_render_frame();
    // FRUITJAM-50: while the picker is open, do NOT blit the emulator frame.
    // There is one framebuffer and no double-buffering (see g_fb), so core 1
    // scans out while core 0 writes. Painting the emulator frame and then
    // painting the overlay over it every field leaves a window in which scanout
    // catches the intermediate state — seen as a bar of emulator screen slowly
    // scanning down the picker. Freezing the buffer removes the race entirely:
    // the emulator keeps running, only its DISPLAY pauses, and the overlay is
    // redrawn just when it changes. Closing the picker resumes the blit, which
    // repaints the whole active area and erases the overlay on the next field.
    if (g_pick_open) {
        if (g_pick_dirty) { draw_picker(); g_pick_dirty = false; }
    } else {
        blit_frame();
    }
    audio_feed();     // drain this field's PCM to the TLV320 over I2S
    run_us_acc += micros() - t0;

    // Pace to real time; resync if we fell behind rather than spiral (pizero
    // pattern). delayMicroseconds yields so the USB CDC task still runs.
    if (deadline == 0) deadline = micros();
    deadline += FRAME_US;
    int32_t rem = (int32_t)(deadline - micros());
    if (rem > 0) delayMicroseconds((uint32_t)rem);
    else         deadline = micros();

    if (++frames - last_report >= 60) {
        static uint32_t last_vf = 0, last_ms = 0;
        uint32_t now_ms = millis();
        uint32_t vf = video_frame_count;
        uint32_t fps = (last_ms && now_ms > last_ms) ? (vf - last_vf) * 1000 / (now_ms - last_ms) : 0;

        // Guard every serial write so a connected-but-not-draining (or absent)
        // USB CDC host can never block core 0 in a full-TX-FIFO write — the
        // leading FRUITJAM-35 freeze suspect (KALEIDSC ran 2h40m clean *with* a
        // monitor draining serial). availableForWrite() is the free TX FIFO
        // byte count; if the whole line won't fit we drop the report rather than
        // stall the emulator. The fps/desync LOGIC below still runs regardless.
        if (Serial && Serial.availableForWrite() >= 160) {
            Serial.printf("emu %lu fields, avg run %lu us/field (%lu.%02lux real-time), video_frames=%lu (%lu fps), audio_peak=%d/32767\n",
                          (unsigned long)frames,
                          (unsigned long)(run_us_acc / (frames - last_report)),
                          (unsigned long)(FRAME_US * (frames - last_report) / run_us_acc),
                          (unsigned long)((uint64_t)FRAME_US * (frames - last_report) * 100 / run_us_acc % 100),
                          (unsigned long)vf, (unsigned long)fps, g_audio_peak);
            if (g_resync_count && Serial.availableForWrite() >= 40)
                Serial.printf("  (resyncs so far: %lu)\n", (unsigned long)g_resync_count);
            // FRUITJAM-14 probe: is the HDMI/precomposed path actually healthy?
            // stale  = active lines posted before the compose ring was built
            //          (falls back to vactive_di_null -> malformed active lines)
            // silence = data islands the queue had to fill with silence
            // lib_resync = resyncs counted inside pico_hdmi itself
            if (Serial.availableForWrite() >= 80)
                Serial.printf("  [di] stale=%lu silence=%lu lib_resync=%lu\n",
                              (unsigned long)video_output_precomposed_stale_count,
                              (unsigned long)hstx_di_queue_silence_count,
                              (unsigned long)video_output_resync_count);
        }
        // FRUITJAM-60: emulator liveness probe. A frozen SCREEN and a hung
        // emulator look identical from outside, and the fields/fps counters
        // cannot tell them apart — they kept advancing normally through a
        // KALEIDSC freeze while nothing moved on screen. Three cheap signals
        // separate the cases:
        //   PC     — sampled once a second. Pinned or oscillating in a narrow
        //            range means the 6809 is SPINNING, not that we are hung.
        //   irq/s  — field-sync IRQs taken, ~60 when healthy. 0 means interrupts
        //            are masked (CWAI/SYNC, or a crash that left F/I set).
        //   vdg    — hash of the VDG buffer. Unchanged across seconds means the
        //            program stopped drawing, which is what "frozen" looks like.
        // Sampled every 4th byte of the 24 KB buffer, once per second: a few
        // thousand cycles, far below the noise floor of a 16 ms field.
        {
            static uint32_t last_irq = 0, last_hash = 0, static_s = 0;
            const uint8_t *vb = coco_machine_get_vdg_buffer();
            uint32_t h = 0;
            for (int i = 0; i < (COCO_VDG_W / 2) * COCO_VDG_H; i += 4)
                h = h * 31u + vb[i];
            uint32_t irq = coco_machine_get_irq_count();
            static_s = (h == last_hash) ? static_s + 1 : 0;
            if (Serial && Serial.availableForWrite() >= 96)
                Serial.printf("  [cpu] PC=$%04X irq/s=%lu vdg=%08lx%s\n",
                              coco_machine_get_pc(),
                              (unsigned long)(irq - last_irq), (unsigned long)h,
                              static_s >= 5 ? "  ! SCREEN STATIC" : "");
            last_irq = irq; last_hash = h;
        }

        g_audio_peak = 0;

        // Desync watchdog, BACKSTOP ONLY. The 150 ms detector above should reach
        // any desync first; if this one ever fires it means the fast path missed
        // it, which is worth knowing. Kept because it costs nothing and it is the
        // only check that runs if the fast detector is ever refactored away.
        if (last_ms && fps > 90) {
            g_want_resync = true;   // core 1 performs the resync safely (see core1_background)
            // Log the MEASURED rate, not just the fact. A marginal 95 (a report
            // window that straddled a glitch) and a latched 160 (the half-line
            // free-run) are completely different faults, and the old message
            // could not tell them apart. The resync count makes consecutive
            // bursts legible in a scripts/serial_logger.py capture.
            if (Serial && Serial.availableForWrite() >= 96)
                Serial.printf("  ! HSTX desync: %lu fps (%lu frames in %lu ms) -> resync #%lu requested (core1)\n",
                              (unsigned long)fps, (unsigned long)(vf - last_vf),
                              (unsigned long)(now_ms - last_ms),
                              (unsigned long)(g_resync_count + 1));
        }
        last_vf = vf; last_ms = now_ms;
        last_report = frames; run_us_acc = 0;
    }

    // --- fast desync detection (FRUITJAM-59) ---------------------------------
    // The desync check used to live in the once-per-second serial report, which
    // made detection latency up to 2 s: a link that desyncs partway through a
    // report window reads somewhere between 60 and 160 fps in proportion to WHEN
    // it broke, so a late onset reads under the 90 threshold and is missed until
    // the NEXT window. Measured in docs/logs: windows of 68, 71, 71, 74, 81 and
    // 83 fps, each one a real desync second that went unnoticed, each costing an
    // extra second of black screen on top of the second it takes to catch it.
    //
    // Detection does not need to be tied to the report interval. Sampling
    // video_frame_count every 150 ms catches onset within ~150 ms instead of up
    // to 2 s, which is the difference between a 2 s blank and a flicker.
    //
    // 150 ms was chosen for margin, not speed. A healthy 60p link delivers 9
    // frames per window, a desynced one 24; the >90 fps threshold sits at 13.5
    // frames, comfortably clear of the +-1 frame sampling jitter. A shorter 100 ms
    // window would only give 6 healthy frames and a much thinner margin — and a
    // FALSE positive is expensive here, because a needless resync is itself a
    // visible dropout.
    {
        static uint32_t dw_ms = 0, dw_vf = 0, dw_hold = 0;
        uint32_t now = millis();
        if (dw_ms == 0) {
            dw_ms = now; dw_vf = video_frame_count;
        } else if (now - dw_ms >= 150) {
            uint32_t vf  = video_frame_count;
            uint32_t fps = (vf - dw_vf) * 1000 / (now - dw_ms);
            // Hold off briefly after firing: core 1 performs the resync
            // asynchronously from its background task, so without this the next
            // window or two would still measure the pre-resync rate and request
            // a second, redundant resync — each of which is another dropout.
            if (fps > 90 && (uint32_t)(now - dw_hold) >= 400) {
                g_want_resync = true;
                dw_hold = now;
                if (Serial && Serial.availableForWrite() >= 64)
                    Serial.printf("  ! desync %lu fps (150ms) -> resync #%lu\n",
                                  (unsigned long)fps,
                                  (unsigned long)(g_resync_count + 1));
            }
            dw_ms = now; dw_vf = vf;
        }
    }

    // --- per-core liveness heartbeat (FRUITJAM-35) ---------------------------
    // Diagnose a freeze with NO serial host attached. Core 0 toggles the onboard
    // LED (GPIO29, active-low) from here, so if this loop stalls the LED FREEZES.
    // The blink RATE encodes core-1 health, read from video_frame_count (bumped
    // by the core-1 video engine):
    //   slow blink (~2 Hz) -> both cores alive
    //   fast blink (~8 Hz) -> core 0 alive but core 1 (video) stalled
    //   frozen LED         -> core 0 hung (e.g. blocked in a USB CDC write)
    {
        static uint32_t hb_vf = 0, hb_eval = 0, hb_toggle = 0;
        static bool     hb_c1_ok = true, hb_on = false;
        uint32_t now = millis();
        if (now - hb_eval >= 250) {                  // re-check core-1 liveness 4x/s
            uint32_t vf = video_frame_count;
            hb_c1_ok = (vf != hb_vf);
            hb_vf = vf; hb_eval = now;
        }
        uint32_t period = hb_c1_ok ? 250 : 60;       // healthy slow vs core1-stalled fast
        if (now - hb_toggle >= period) {
            hb_on = !hb_on;
            digitalWrite(PIN_LED, hb_on ? LOW : HIGH);   // active-low: LOW = lit
            hb_toggle = now;
        }
    }

    delay(1);   // yield so the USB CDC task runs (serial + 1200-baud reset)
}
