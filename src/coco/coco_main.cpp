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

// FRUITJAM-61: crop the VDG border and scale the 256x192 active area 2.5x to
// fill 640x480, instead of centring it at 2x with a border. OFF by default
// until it has been measured against the default path — it costs core-0 blit
// time, and core-0 headroom is the variable that correlates with HSTX desync
// rate (FRUITJAM-58). See the g_scan declaration for the full rationale.
// FRUITJAM-88: SD write-back is OFF by default until the in-memory disk image
// is trustworthy. SAVEM corrupts the image (a pre-existing FDC multi-sector
// write bug), and write-back faithfully persists that corruption to the card —
// turning a fault that a reboot used to clear into permanent damage to the
// .dsk. The mechanism itself is verified working (FRUITJAM-81: SAVE survives a
// power cycle), so it is gated rather than removed. Set to 1 once FRUITJAM-88
// is fixed.
#ifndef COCO_DSK_WRITEBACK
#define COCO_DSK_WRITEBACK 0
#endif

#ifndef COCO_CROP_BORDER
#define COCO_CROP_BORDER 0
#endif

// FRUITJAM-62: smoothing across the 2.5x horizontal map. Only meaningful with
// COCO_CROP_BORDER. 0 = hard 2,3,2,3 edges; 1 = full linear interpolation;
// 2 = boundary blend only (default — see the g_mix comment for why).
#ifndef COCO_CROP_SMOOTH
#define COCO_CROP_SMOOTH 2
#endif

// FRUITJAM-77: boot-progress NeoPixels, ON by default. Was gated off by
// FRUITJAM-53, whose evidence does not survive re-measurement — see that issue.
// The strip is driven only during boot and then LEFT LIT: WS2812s latch, so the
// last colour persists with no state machine held and nothing touching it at
// runtime. That is deliberate; it is the pre-FRUITJAM-53 behaviour.
// NOTE the measured cost, so it is a choice and not an accident: desync rate
// with the strip LIT is ~21/hr against ~6/hr DARK. Five WS2812s draw ~50-100 mA
// and the rate tracks ILLUMINATION, not whether the firmware drives them — the
// COCO_NEOPIXEL=0 build measured the same 21/hr while the strip stayed lit
// through the WS2812 latch. Set this to 0 AND power-cycle to get the strip dark.
// FRUITJAM-53 (superseded): boot-progress NeoPixels were off by default until the video
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
#define COCO_NEOPIXEL 1
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
    dvi::rgb565(0x30, 0x50, 0xE0),  // 10 ARTIFACT BLUE   (FRUITJAM-73)
    dvi::rgb565(0xE0, 0x60, 0x18),  // 11 ARTIFACT ORANGE (FRUITJAM-73)
    0, 0, 0, 0
};

// Single RGB565 framebuffer, matching the proven-stable FRUITJAM-04 display_test.
// The palette lookup happens here in the blit (core 0), so scanline_cb stays a
// plain copy — a per-pixel palette lookup in the callback overran the HSTX line
// budget and desynced the link (black after a few seconds). RGB565 double-
// buffering would be tear-free but two 150 KB buffers overflow SRAM; the
// residual blit/scanout tearing is a FRUITJAM-11/23 refinement.
// FRUITJAM-61: border-cropped 2.5x scaling (COCO_CROP_BORDER=1).
//
// The default path centres the CoCo's 256x192 active area in a 320x240
// framebuffer and lets scanout double it to 640x480, so the picture occupies
// 512x384 with a 64/48 px border. That border is FAITHFUL — the real VDG draws
// one — but it costs 44% of the screen area.
//
// Cropping it makes the arithmetic exact, not messy: 640/256 and 480/192 are
// both 2.5, so the active area alone fills 640x480 with the aspect ratio
// preserved. The catch is that 2.5 is non-integer, so source pixels alternate
// 2 and 3 output pixels wide. On text that shows as vertical strokes of uneven
// weight within a glyph. It is tolerable because the sink already resamples
// 640x480 to the panel non-integrally — this substitutes for that first scaling
// stage rather than adding one.
//
// The implementation exploits an asymmetry: VERTICAL scaling is free and
// HORIZONTAL is not.
//   - Vertical is pure row selection. The scanline callback returns a row
//     POINTER, so g_row[] maps 480 output lines onto 192 source rows at no
//     per-pixel cost at all.
//   - Horizontal is fixed by HSTX: expand_shift is ENC_N_SHIFTS=2/ENC_SHIFT=16,
//     so one 32-bit word always feeds two pixels and an active line is always
//     320 words. The default path gets its free 2x from native_pixel_mode's
//     16-bit reads, which BUS-REPLICATE each halfword into both pixels. Turn
//     native_pixel_mode OFF and those same 320 words carry 640 DISTINCT pixels
//     — 1:1, still a pure pointer handoff, still zero per-line CPU on core 1.
// So the buffer is scaled horizontally (640 wide) but stays 1:1 vertically
// (192 rows), and costs 640*192*2 = 245,760 bytes.
#if COCO_CROP_BORDER
static uint16_t        g_scan[COCO_VDG_H][dvi::H_ACTIVE];
static const uint32_t *g_row[dvi::V_ACTIVE];   // output line -> source row

#if COCO_CROP_SMOOTH
// FRUITJAM-62: smoothing the 2.5x map, without paying for a blend per pixel.
//
// Nearest-neighbour at 2.5x makes each source pixel 2 or 3 output pixels wide,
// so a glyph's vertical strokes come out at uneven weight — the "sketchy" text.
// Linear interpolation fixes it, but blending RGB565 per output pixel means
// unpack/blend/repack on 122,880 pixels a field, which core 0 cannot afford.
//
// It does not have to. The VDG has only 16 colours, so there are just 16x16
// possible source PAIRS, and the 2.5x pattern uses only four blend ratios.
// Sampling output pixel j at source position j*0.4 gives, per 5-pixel group:
//   o0 = s0            o1 = 40% s1     o2 = 80% s1
//   o3 = 20% s2        o4 = 60% s2
// so four 16x16 tables — 2 KB total — turn every blend into one lookup. The
// blit keeps its 5 stores per source byte and swaps 2 palette reads for 5
// table reads. Note o3/o4 straddle the byte boundary and need the NEXT byte's
// low nibble, hence the one-pixel lookahead.
// MODE 2 (default) blends ONLY the boundary pixel, and it is not merely the
// cheap option — it is arguably the correct one. The complaint is uneven STROKE
// WEIGHT (a source pixel 2 output px wide next to one 3 px wide), not hard
// edges as such; full interpolation fixes that but softens edges which were
// already right, which on a 16-colour VDG reads as blur rather than antialias.
//
// Give each source pixel two solid output pixels and let them SHARE the fifth
// 50/50, and every source pixel occupies 2.5 px exactly:
//   s0 = o0, o1, half of o2      s1 = half of o2, o3, o4
// The s1|s2 boundary then falls between groups, so exactly one blend per five
// pixels — and no lookahead into the next byte at all, unlike mode 1.
#if COCO_CROP_SMOOTH == 1
static uint16_t g_mix[4][16][16];
#else
static uint16_t g_mix[1][16][16];
#endif

// w is the weight of b, 0..256.
static uint16_t mix565(uint16_t a, uint16_t b, uint32_t w) {
    uint32_t ar = (a >> 11) & 0x1F, ag = (a >> 5) & 0x3F, ab = a & 0x1F;
    uint32_t br = (b >> 11) & 0x1F, bg = (b >> 5) & 0x3F, bb = b & 0x1F;
    uint32_t r = (ar * (256 - w) + br * w) >> 8;
    uint32_t g = (ag * (256 - w) + bg * w) >> 8;
    uint32_t l = (ab * (256 - w) + bb * w) >> 8;
    return (uint16_t)((r << 11) | (g << 5) | l);
}
#endif
#else
static dvi::Framebuffer g_fb;
#endif
static uint8_t          g_rom[16384];

// pico_hdmi 2.0-beta precomposed/native scanout ring (README "Minimal pattern").
// Each entry precomposes one active-line header off the ISR so the per-scanline
// work is ~1.5us (patch a data island) instead of a 320-px fill — the path built
// for "320->640 scaling while HDMI runs" (emulators). ~312 B/entry.
static video_output_precomposed_line_t g_compose_ring[48];

// Authentic NTSC CoCo: ~0.895 MHz / 60 Hz = 14915 6809 cycles per field.
static const uint32_t CYCLES_PER_FRAME = 14915;
static const uint32_t FRAME_US         = 16762;   // 60 Hz field period

#if !COCO_CROP_BORDER
// CoCo 256x192 active area centered in the 320x240 framebuffer.
static const int OX = (dvi::FB_WIDTH  - COCO_VDG_W) / 2;   // 32
static const int OY = (dvi::FB_HEIGHT - COCO_VDG_H) / 2;   // 24
#endif

// Output column span of CoCo pixel x under the 2.5x map: every two source
// pixels become five output ones, giving the 2,3,2,3 pattern.
#define SPAN_LO(x) (((x) * 5) / 2)
#define SPAN_HI(x) ((((x) + 1) * 5) / 2)

// Scanline POINTER callback (native pixel mode): return the address of the
// framebuffer row and let the DMA read it directly, doubling each pixel in
// hardware. The per-scanline IRQ does essentially no work — no per-line fill to
// fall behind under core-0 emulation load, which is what was underrunning the
// HSTX FIFO and desyncing the link. Vertical 2x via active_line >> 1.
static const uint32_t *RAM_FUNC scanline_ptr_cb(uint32_t v_scanline, uint32_t active_line) {
    (void)v_scanline;
#if COCO_CROP_BORDER
    return g_row[active_line];      // 2.5x vertical, precomputed: one load
#else
    return (const uint32_t *)g_fb.px[active_line >> 1];
#endif
}

// Classic (non-native) scanline callback: fill one 640-px active line from the
// 320-wide row, duplicating each pixel horizontally. Used by COCO_DVI_MODE,
// which cannot use the precomposed COMPOSE RING — video_output_compose_service()
// early-returns when dvi_mode is set, so the ring never builds.
// CORRECTION: the earlier version of this comment said DVI cannot use the
// pointer path either, citing video_output_rt.c:864. That file is NOT COMPILED
// (library.json excludes it). In the built video_output.c,
// video_output_handle_active_start() checks scanline_pointer_callback FIRST and
// unconditionally, before it ever looks at dvi_mode — which is why the default
// DVI build has been using scanline_ptr_cb all along. The compose ring and the
// pointer path are separate things; only the ring is unavailable in DVI mode.
// This is a PLAIN COPY, no palette lookup: the palette is applied in blit_frame()
// on core 0. A per-pixel lookup here overran the line budget historically.
// Byte-for-byte the callback display_test.cpp runs, which is stable on this sink.
#if !COCO_CROP_BORDER
static void RAM_FUNC scanline_cb(uint32_t v_scanline, uint32_t active_line, uint32_t *dst) {
    (void)v_scanline;
    const uint16_t *row = g_fb.px[active_line >> 1];
    for (uint32_t i = 0; i < dvi::FB_WIDTH; i++) {
        uint32_t px = row[i];
        dst[i] = px | (px << 16);
    }
}
#endif

// Compose the CoCo frame (nibble-packed indices) into g_fb as RGB565, centered.
// The palette lookup lives here (core 0), keeping scanline_cb a plain copy.
// FRUITJAM-75: RAM-resident. This is the largest flash-resident chunk of core
// 0's per-field path (~3.1 ms/field), and executing it from flash via the XIP
// cache makes it — and the whole HSTX link — sensitive to where the linker
// happens to put it. Measured: adding 110 lines of NEVER-EXECUTED code elsewhere
// in this file cost +1.5 ms/field and took desyncs from 0/hr to 943/hr.
// .time_critical is single-cycle RAM, so the timing stops depending on layout
// and the fetches stop contending with the HSTX DMA for flash.
static void RAM_FUNC blit_frame() {
    const uint8_t *vb = coco_machine_get_vdg_buffer();
#if COCO_CROP_BORDER
    for (int y = 0; y < COCO_VDG_H; y++) {
        const uint8_t *src = &vb[y * (COCO_VDG_W / 2)];
        uint16_t *dst = g_scan[y];
        // One packed byte is two CoCo pixels, and 2.5x turns exactly two source
        // pixels into five output ones — so the nibble pair and the 2,3 output
        // pattern line up exactly, one byte per iteration. That HALVES the loop
        // count (128 iterations per row, not 256) while raising stores 2.5x,
        // which is why this is not simply 2.5x the work of the default blit.
        for (int i = 0; i < COCO_VDG_W / 2; i++) {
            uint8_t b = src[i];
#if COCO_CROP_SMOOTH == 1
            uint8_t n0 = b & 0x0F, n1 = b >> 4;
            // o3/o4 straddle into the next source pixel; the last byte of the
            // row has none, so hold n1 rather than read past the row.
            uint8_t n2 = (i + 1 < COCO_VDG_W / 2) ? (src[i + 1] & 0x0F) : n1;
            *dst++ = g_pal[n0];
            *dst++ = g_mix[0][n0][n1];
            *dst++ = g_mix[1][n0][n1];
            *dst++ = g_mix[2][n1][n2];
            *dst++ = g_mix[3][n1][n2];
#elif COCO_CROP_SMOOTH == 2
            uint8_t  n0 = b & 0x0F, n1 = b >> 4;
            uint16_t a = g_pal[n0], c = g_pal[n1];
            *dst++ = a; *dst++ = a;
            *dst++ = g_mix[0][n0][n1];   // the shared half-pixel
            *dst++ = c; *dst++ = c;
#else
            uint16_t a = g_pal[b & 0x0F];   // even x -> 2 px wide
            uint16_t c = g_pal[b >> 4];     // odd  x -> 3 px wide
            *dst++ = a; *dst++ = a;
            *dst++ = c; *dst++ = c; *dst++ = c;
#endif
        }
    }
#else
    for (int y = 0; y < COCO_VDG_H; y++) {
        const uint8_t *src = &vb[y * (COCO_VDG_W / 2)];
        uint16_t *dst = &g_fb.px[OY + y][OX];
        for (int x = 0; x < COCO_VDG_W; x++) {
            uint8_t idx = (x & 1) ? (src[x >> 1] >> 4) : (src[x >> 1] & 0x0F);
            dst[x] = g_pal[idx];
        }
    }
#endif
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
static bool RAM_FUNC basic_at_prompt(void) {
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
#define PICKER_MAX 128   // card had >32

// FatFs is built with long filenames on (ffconf.h: FF_USE_LFN 3, FF_MAX_LFN
// 255), so the old 12-char cap was self-imposed, not a FAT limit — it just
// threw the long name away at scan time. 31 chars is what the 32-column overlay
// can show beside the cursor and drive digit, and it keeps the mount path
// ("0:/coco/dsk/" + name) inside g_dsk_path's 48 bytes.
#define DSK_NAME_MAX 32          // 31 chars + NUL
#define DSK_NAME_COLS 27         // 32 cols - 5 for the "N -> " prefix

// Defined further down (with the other SD loaders / the PSRAM allocator block);
// forward-declared here so the picker can sit next to the rest of the board UI.
static size_t load_psram_file(const char *path, const uint8_t **out);
void __psram_free(void *);   // C++ linkage, matching the core's psram.h

static char     g_dsk_names[PICKER_MAX][DSK_NAME_MAX];
static int      g_dsk_count = 0;
static int      g_pick_sel  = 0;
static bool     g_pick_open = false;

// FRUITJAM-78: four drives, tracked independently. g_dsk_cur[d] is the index
// into g_dsk_names of the image assigned to drive d, -1 = unassigned.
static int      g_dsk_cur[COCO_NDRIVE] = { -1, -1, -1, -1 };
static uint8_t *g_dsk_img[COCO_NDRIVE] = { nullptr, nullptr, nullptr, nullptr };
static size_t   g_dsk_len[COCO_NDRIVE] = { 0, 0, 0, 0 };

// Which drive the overlay is currently assigning. Set from DSKREG when the
// overlay opens, so it comes up on the drive the machine last addressed rather
// than always on 0 — after a `DRIVE 1` + `DIR`, the overlay opens on drive 1.
// FRUITJAM-79: PSRAM image cache, populated at BOOT.
//
// Every disk image the card holds (up to a PSRAM budget) is read once during
// setup(), BEFORE multicore_launch_core1(). That timing is the whole point: no
// video is being scanned out yet, so SD activity at boot is provably harmless —
// it is why booting has never shown the fault that every runtime mount does.
//
// Mounting then becomes a POINTER ASSIGNMENT. No SD read, no flash write, no
// copy: nothing that can disturb the HSTX link (FRUITJAM-97, where the best
// tuned configuration still corrupted TMDS on 35% of mounts).
//
// The cached buffer is handed to the FDC directly rather than copied into a
// per-drive working buffer. Writes therefore mutate the cache, which matches
// existing behaviour: with COCO_DSK_WRITEBACK=0 changes are already session-only
// and lost on power-down, and with it enabled they are flushed to the card
// anyway. The one consequence worth naming is that mounting the SAME image in
// two drives shares one buffer, so writes alias — a real drive cannot hold the
// same physical disk twice, so this is an unreachable state in practice.
//
// Beyond the budget, mount_dsk_index falls back to the old SD read, with all of
// FRUITJAM-97's cost. Budget is deliberately well under the 8 MB part.
#define DSK_CACHE_BUDGET (6u * 1024u * 1024u)
static uint8_t *g_dsk_cache[PICKER_MAX]     = { nullptr };
static size_t   g_dsk_cache_len[PICKER_MAX] = { 0 };
static int      g_dsk_cached_n   = 0;
static size_t   g_dsk_cached_sz  = 0;

// The overlay has NO "current drive" mode. It is one list of the card's disks,
// and 0-3 assign the highlighted disk to that drive — pressing the digit a
// second time unassigns it. That removes the modal state entirely: there is no
// drive to be "in", so nothing to step through and nothing to get lost in, and
// the whole four-drive layout is set from one screen.
//
// It also makes [EMPTY] unnecessary: emptying a drive is the same keystroke
// that filled it, so a dedicated row for it would be a second way to do one
// thing. Rows are just the disks — no [CANCEL] either, since with ENTER inert
// it was an unreachable row for the keyboard; the button path closes on the
// 1+3 chord instead (see picker_task).
#define PICK_COUNT        (g_dsk_count)
// Visible list rows: 0 = title bar, 1-14 = list, 15 = status bar. Shared with
// the key handler so PgUp/PgDn move by exactly one screen — if this and the
// drawing disagreed, paging would silently skip or repeat rows.
#define PICK_ROWS         14


// FRUITJAM-81: write-back. The FDC reports each sector it writes; we record the
// offset and flush at a FIELD BOUNDARY, never from inside the emulation call.
//
// Offsets only — the data is already in g_dsk_img, so there is nothing to copy
// and nothing that can go stale. A sector written twice before a flush is
// deduplicated, so a program rewriting one sector in a loop costs one card
// write, not thousands.
//
// The queue is small on purpose. RSDOS writes a handful of sectors per SAVE (the
// data granules plus the directory and FAT), so 16 covers a normal save with
// room to spare; overflow degrades to a FULL-IMAGE flush rather than silently
// losing data.
// FRUITJAM-87: the queue is PER DRIVE. A single shared queue held only one
// g_dsk_path, so as soon as four drives existed a sector written on drive 1
// would be flushed into drive 0's file — silent cross-disk corruption. Offsets
// alone cannot say which image they belong to, which is why the FDC callback
// now reports the drive.
#define DSKW_MAX 16
static char     g_dsk_path[COCO_NDRIVE][48] = { "", "", "", "" };
static uint32_t g_dskw_off[COCO_NDRIVE][DSKW_MAX];
static int      g_dskw_n[COCO_NDRIVE] = { 0, 0, 0, 0 };
static bool     g_dskw_overflow[COCO_NDRIVE] = { false, false, false, false };

static void dsk_sector_written(int drive, uint32_t off, uint32_t len) {
    (void)len;
    if ((unsigned)drive >= COCO_NDRIVE) return;
    for (int i = 0; i < g_dskw_n[drive]; i++)
        if (g_dskw_off[drive][i] == off) return;                       // dedup
    if (g_dskw_n[drive] >= DSKW_MAX) { g_dskw_overflow[drive] = true; return; }
    g_dskw_off[drive][g_dskw_n[drive]++] = off;
}
static char     g_pick_msg[40] = "";
static bool     g_pick_dirty = false;   // redraw the overlay only when it changes

// Buttons are active-low with internal pull-ups. Note button 1 is GPIO0, which
// is also USB-BOOT — sampled only at reset, so it is free to use at runtime, but
// never tell a user to hold it while power-cycling.
struct Btn { uint8_t pin; bool last; uint32_t t; };
static Btn g_btn[3] = { { PIN_BUTTON1, true, 0 },
                        { PIN_BUTTON2, true, 0 },
                        { PIN_BUTTON3, true, 0 } };

static bool RAM_FUNC btn_fell(Btn &b) {
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
        strncpy(g_dsk_names[g_dsk_count], fno.fname, DSK_NAME_MAX - 1);
        g_dsk_names[g_dsk_count][DSK_NAME_MAX - 1] = '\0';
        g_dsk_count++;
    }
    // Never truncate silently: say so if the card holds more than we can list.
    if (g_dsk_count == PICKER_MAX)
        Serial.printf("[picker: hit PICKER_MAX=%d, further .dsk ignored] ", PICKER_MAX);
    f_closedir(&d);

    // Sort case-insensitively: f_readdir returns FAT DIRECTORY ORDER, which is
    // insertion order and effectively arbitrary to a reader — a card written
    // over months lists in the order files happened to be copied.
    //
    // Safe to reorder here because g_dsk_cur[] indices are assigned AFTER this
    // runs, and the saved assignments resolve by NAME rather than index (the
    // same reasoning FRUITJAM-71 used for storing a filename, not an index).
    // scan_dsk_dir() is called once, at boot; if it ever gains a rescan caller
    // it must re-resolve g_dsk_cur[] by name, since sorting would otherwise
    // leave those indices pointing at the wrong disks.
    //
    // Insertion sort: PICKER_MAX is 128, this runs once at boot, and it costs
    // no scratch buffer beyond one row.
    for (int i = 1; i < g_dsk_count; i++) {
        char key[DSK_NAME_MAX];
        memcpy(key, g_dsk_names[i], DSK_NAME_MAX);
        int j = i - 1;
        while (j >= 0 && strcasecmp(g_dsk_names[j], key) > 0) {
            memcpy(g_dsk_names[j + 1], g_dsk_names[j], DSK_NAME_MAX);
            j--;
        }
        memcpy(g_dsk_names[j + 1], key, DSK_NAME_MAX);
    }
}

// FRUITJAM-71: remember the last disk the USER mounted, so a reboot comes back
// with the same one inserted instead of resetting to AUTO.DSK.
//
// Stored on SD as a bare filename, not an index: indices shift the moment a
// .dsk is added or removed from the card, which would silently mount the wrong
// image. The file is plain text and hand-editable, and deleting it simply falls
// back to AUTO.DSK.
//
// It lives in /coco/ rather than /coco/dsk/ so scan_dsk_dir() can never see it,
// and is named .txt so it cannot be mistaken for a disk image.
#define LAST_DSK_PATH "0:/coco/lastdsk.txt"

// FRUITJAM-78 extends this to FOUR lines, one per drive, in drive order; an
// empty line means that drive is unassigned. Still bare filenames, still plain
// text, still hand-editable, and still safe to delete.
//
// BACKWARD COMPATIBLE by construction: a pre-FRUITJAM-78 file is a single name
// with no newline, which parses as "drive 0 = that name, 1-3 empty" — exactly
// the old behaviour. No migration step, and downgrading simply reads the first
// line and ignores the rest.
static void save_dsk_assignments(char names[][DSK_NAME_MAX], const int *cur, int ndrive) {
    FIL f;
    if (f_open(&f, LAST_DSK_PATH, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) {
        // Not fatal — a read-only or full card just means no persistence.
        if (Serial && Serial.availableForWrite() >= 48)
            Serial.println("[picker: could not save drive assignments]");
        return;
    }
    UINT bw = 0;
    for (int d = 0; d < ndrive; d++) {
        if (cur[d] >= 0) f_write(&f, names[cur[d]], strlen(names[cur[d]]), &bw);
        f_write(&f, "\n", 1, &bw);
    }
    f_close(&f);
}

// Parse up to ndrive lines into out[d]. Returns how many drives got a name.
static int load_dsk_assignments(char out[][DSK_NAME_MAX], int ndrive) {
    for (int d = 0; d < ndrive; d++) out[d][0] = '\0';
    FIL f;
    if (f_open(&f, LAST_DSK_PATH, FA_READ) != FR_OK) return 0;
    char buf[COCO_NDRIVE * DSK_NAME_MAX + 8];
    UINT br = 0;
    f_read(&f, buf, (UINT)(sizeof(buf) - 1), &br);
    f_close(&f);
    buf[br] = '\0';
    int found = 0, d = 0;
    const char *p = buf;
    while (d < ndrive && *p) {
        const char *e = strchr(p, '\n');
        size_t n = e ? (size_t)(e - p) : strlen(p);
        if (n > DSK_NAME_MAX - 1) n = DSK_NAME_MAX - 1;
        memcpy(out[d], p, n); out[d][n] = '\0';
        // Tolerate a hand-edited file: strip trailing CR / whitespace.
        while (n && (out[d][n - 1] == '\r' || out[d][n - 1] == ' ')) out[d][--n] = '\0';
        if (n) found++;
        d++;
        if (!e) break;
        p = e + 1;
    }
    return found;
}

// Write back the sectors the FDC has touched. Called from loop() at a field
// boundary, and with all=true before a disk swap.
//
// Why this is affordable: during a disk operation the emulated CPU is HALTED
// waiting on the FDC, and real CoCo software already parks its UI while the
// drive spins. The card write lands inside latency the software expects — the
// goal is to be faster and quieter than a real drive, not to pretend the drive
// is instant.
//
// One f_open per flush rather than a held handle: a held FIL would have to
// survive disk swaps, SD removal and the picker, and FatFs offers no way to
// validate one cheaply. Opening costs a directory lookup; a save is rare.
static void flush_dsk_writes(bool all) {
#if !COCO_DSK_WRITEBACK
    // Gated off (FRUITJAM-88). Keep draining the queue so the FDC-side path and
    // its dedup still run exactly as they will when this is re-enabled — only
    // the card write is suppressed. Writes still land in the PSRAM image, so
    // SAVE/LOAD work within a session and are simply lost on power-down, which
    // is the behaviour before FRUITJAM-81.
    (void)all;
    for (int d = 0; d < COCO_NDRIVE; d++) {
        if (!g_dskw_n[d] && !g_dskw_overflow[d]) continue;
        if (Serial && Serial.availableForWrite() >= 72)
            Serial.printf("  [dsk%d: %d sector(s) NOT written back - COCO_DSK_WRITEBACK=0]\n",
                          d, g_dskw_overflow[d] ? -1 : g_dskw_n[d]);
        g_dskw_n[d] = 0; g_dskw_overflow[d] = false;
    }
    return;
#else
    for (int d = 0; d < COCO_NDRIVE; d++) {
        if ((!g_dskw_n[d] && !g_dskw_overflow[d]) || !g_dsk_path[d][0] || !g_dsk_img[d])
            continue;
        FIL f;
        if (f_open(&f, g_dsk_path[d], FA_WRITE) != FR_OK) {
            if (Serial && Serial.availableForWrite() >= 48)
                Serial.printf("[dsk%d: write-back open failed]\n", d);
            g_dskw_n[d] = 0; g_dskw_overflow[d] = false;   // do not spin on a dead card
            continue;
        }
        UINT bw = 0;
        int wrote = 0;
        if (g_dskw_overflow[d]) {
            // Lost track of which sectors changed — rewrite the whole image rather
            // than persist a partial, inconsistent picture.
            f_lseek(&f, 0);
            f_write(&f, g_dsk_img[d], (UINT)g_dsk_len[d], &bw);
            wrote = -1;
        } else {
            // Bound per field. The budget is PER DRIVE, which is deliberate: a
            // field that touches two drives does at most 4 sectors on each,
            // rather than starving the second drive behind the first.
            int n = all ? g_dskw_n[d] : (g_dskw_n[d] > 4 ? 4 : g_dskw_n[d]);
            for (int i = 0; i < n; i++) {
                f_lseek(&f, g_dskw_off[d][i]);
                f_write(&f, g_dsk_img[d] + g_dskw_off[d][i], 256, &bw);
            }
            wrote = n;
            for (int i = n; i < g_dskw_n[d]; i++) g_dskw_off[d][i - n] = g_dskw_off[d][i];
            g_dskw_n[d] -= n;
        }
        f_close(&f);
        g_dskw_overflow[d] = false;
        if (Serial && Serial.availableForWrite() >= 48)
            Serial.printf("[dsk%d: wrote %s]\n", d, wrote < 0 ? "FULL IMAGE" : "sectors");
    }
#endif
}

// Load image i into PSRAM and hand it to the FDC, freeing the previous one.
// Blocks core 0 for the length of the SD read (~160 KB) — see FRUITJAM-50.
// remember=true persists the choice (FRUITJAM-71); boot passes false, because
// restoring a disk is not the user choosing it and would rewrite the marker
// identically on every boot.
// Empty a drive: free its image and tell the FDC there is no disk, so the drive
// reports NOT READY exactly as an unassigned one does after a cold boot.
static bool dsk_buf_is_cached(const uint8_t *p);   // defined with mount_dsk_index

static void eject_drive(int drive, bool remember) {
    if ((unsigned)drive >= COCO_NDRIVE) return;
    flush_dsk_writes(true);                 // do not strand unwritten sectors
    if (g_dsk_img[drive] && !dsk_buf_is_cached(g_dsk_img[drive]))
        __psram_free(g_dsk_img[drive]);
    g_dsk_img[drive]  = nullptr;
    g_dsk_len[drive]  = 0;
    g_dsk_cur[drive]  = -1;
    g_dsk_path[drive][0] = '\0';
    g_dskw_n[drive] = 0; g_dskw_overflow[drive] = false;
    coco_machine_mount_dsk_drive(drive, nullptr, 0);
    if (remember) save_dsk_assignments(g_dsk_names, g_dsk_cur, COCO_NDRIVE);
    snprintf(g_pick_msg, sizeof(g_pick_msg), "DRIVE %d EMPTY", drive);
}

// Row to sit on when the overlay opens: whatever is in drive 0, else the top.
static int pick_row_for_drive(int drive) {
    return (g_dsk_cur[drive] >= 0) ? g_dsk_cur[drive] : 0;
}

// FRUITJAM-97: instrument a mount. Default OFF — per CLAUDE.md serial load is
// itself a variable, and this prints from the picker path.
#ifndef COCO_MOUNT_PROBE
#define COCO_MOUNT_PROBE 0
#endif
#if COCO_MOUNT_PROBE
// Both counters are defined further down (with the deferred-resync machinery and
// in pico_hdmi respectively); declared here so the mount path can sample them.
extern volatile uint32_t g_resync_count;
extern "C" volatile uint32_t video_output_resync_count;
#endif

// True when the buffer a drive holds belongs to the boot cache, and so must
// never be freed on swap — the cache owns it for the life of the session.
static bool dsk_buf_is_cached(const uint8_t *p) {
    if (!p) return false;
    for (int k = 0; k < g_dsk_count; k++) if (g_dsk_cache[k] == p) return true;
    return false;
}

static bool mount_dsk_index(int i, int drive, bool remember) {
    if (i < 0 || i >= g_dsk_count) return false;
    if ((unsigned)drive >= COCO_NDRIVE) return false;

    // FRUITJAM-79 fast path: the image is already in PSRAM from boot, so this
    // is a pointer assignment and touches no I/O at all.
    if (g_dsk_cache[i]) {
        flush_dsk_writes(true);
        if (g_dsk_img[drive] && !dsk_buf_is_cached(g_dsk_img[drive]))
            __psram_free(g_dsk_img[drive]);
        g_dsk_img[drive] = g_dsk_cache[i];
        g_dsk_len[drive] = g_dsk_cache_len[i];
        snprintf(g_dsk_path[drive], sizeof(g_dsk_path[drive]), "0:/coco/dsk/%s", g_dsk_names[i]);
        g_dsk_cur[drive] = i;
        coco_machine_mount_dsk_drive(drive, g_dsk_img[drive], g_dsk_len[drive]);
        if (remember) save_dsk_assignments(g_dsk_names, g_dsk_cur, COCO_NDRIVE);
        snprintf(g_pick_msg, sizeof(g_pick_msg), "%.21s IN DRIVE %d", g_dsk_names[i], drive);
#if COCO_MOUNT_PROBE
        Serial.printf("[mount] %lu ms  CACHED  underruns %lu\n",
                      (unsigned long)(millis() - t_start),
#if PICO_HDMI_FIFO_PROBE
                      (unsigned long)(hstx_fifo_underruns - u0));
#else
                      0UL);
#endif
#endif
        return true;
    }
#if COCO_MOUNT_PROBE
    uint32_t t_start = millis();
    uint32_t r0 = g_resync_count, l0 = video_output_resync_count;
#if PICO_HDMI_FIFO_PROBE
    extern volatile uint32_t hstx_fifo_underruns;
    uint32_t u0 = hstx_fifo_underruns;
#endif
#endif
    char path[48];
    snprintf(path, sizeof(path), "0:/coco/dsk/%s", g_dsk_names[i]);
    const uint8_t *img = nullptr;
    size_t len = load_psram_file(path, &img);
    if (!len) { snprintf(g_pick_msg, sizeof(g_pick_msg), "LOAD FAILED"); return false; }
    // Flush BEFORE freeing: a swap abandons any unflushed sectors for the image
    // being replaced, and after the free its buffer is gone.
    flush_dsk_writes(true);
    if (g_dsk_img[drive] && !dsk_buf_is_cached(g_dsk_img[drive]))
        __psram_free(g_dsk_img[drive]);
    g_dsk_img[drive] = (uint8_t *)img;
    g_dsk_len[drive] = len;
    snprintf(g_dsk_path[drive], sizeof(g_dsk_path[drive]), "%s", path);
    g_dsk_cur[drive] = i;
    coco_machine_mount_dsk_drive(drive, g_dsk_img[drive], len);
    if (remember) save_dsk_assignments(g_dsk_names, g_dsk_cur, COCO_NDRIVE);
    // " IN DRIVE d" is 11 of the panel's 32 columns, so the name gets 21 —
    // long names are truncated here rather than running off the row.
    snprintf(g_pick_msg, sizeof(g_pick_msg), "%.21s IN DRIVE %d", g_dsk_names[i], drive);

#if COCO_MOUNT_PROBE
    // FRUITJAM-97 discriminator. The question this answers is the ONLY one that
    // matters next: does the firmware resync during a mount, or does the sink
    // drop lock by itself?
    //   both counters move  -> our resync path fired; the 1-2 s is OUR cost, and
    //                          FRUITJAM-56's pin-parking is not doing its job
    //   neither moves       -> the sink lost lock unaided, i.e. TMDS was corrupted
    //                          and nothing in the firmware ever noticed
    // Printed AFTER the load so the serial write cannot itself perturb it.
    Serial.printf("[mount] %lu ms  resync %lu->%lu  lib %lu->%lu  UNDERRUNS %lu  %s\n",
                  (unsigned long)(millis() - t_start),
                  (unsigned long)r0, (unsigned long)g_resync_count,
                  (unsigned long)l0, (unsigned long)video_output_resync_count,
#if PICO_HDMI_FIFO_PROBE
                  (unsigned long)(hstx_fifo_underruns - u0),
#else
                  0UL,
#endif
                  g_dsk_cache[i] ? "CACHED" : "sd-read");
#endif
    return true;
}

// - - - FRUITJAM-72: auto-run the first program on a button-mounted disk - - - -
// After button 2 mounts a disk and cold-boots (FRUITJAM-69), read the disk's
// RSDOS directory, find the first runnable program, and type the right command at
// the DECB prompt. Buttons only - the no-keyboard path, where the user has no way
// to type RUN"..." themselves.

// RSDOS directory: track 17, sectors 3-11 (1-based), 32-byte entries, 8 per
// 256-byte sector. Entry: [0..7] name space-padded, [8..10] ext, [11] file type,
// [12] ASCII flag. First byte 0xFF ends the directory, 0x00 marks deleted.
// Types: 0 = BASIC, 1 = BASIC data, 2 = machine language, 3 = text source. Only
// 0 and 2 are runnable, so a data file sitting first cannot win.
static bool dsk_first_program(int drive, char *out, size_t outn, int *type_out) {
    if ((unsigned)drive >= COCO_NDRIVE) return false;
    if (!g_dsk_img[drive] || !g_dsk_len[drive]) return false;
    size_t hdr = g_dsk_len[drive] % 256;   // same JVC rule the mount uses
    for (int sec = 3; sec <= 11; sec++) {
        size_t off = hdr + (17u * 18u + (size_t)(sec - 1)) * 256u;
        if (off + 256 > g_dsk_len[drive]) return false;
        const uint8_t *sp = g_dsk_img[drive] + off;
        for (int e = 0; e < 8; e++) {
            const uint8_t *d = sp + e * 32;
            if (d[0] == 0xFF) return false;
            if (d[0] == 0x00) continue;
            int t = d[11];
            if (t != 0 && t != 2) continue;
            size_t n = 0;
            for (int i = 0; i < 8 && d[i] != ' ' && n < outn - 1; i++) out[n++] = (char)d[i];
            out[n] = 0;
            *type_out = t;
            return n > 0;
        }
    }
    return false;
}

// Autotype. The ROM polls the key matrix, so a key must be HELD several fields to
// register and RELEASED for a few more, or debounce misses it or doubles it.
#define TYPE_HOLD_FIELDS 3
#define TYPE_GAP_FIELDS  2
static char g_type_buf[48];
static int  g_type_pos  = -1;   // -1 = idle
static int  g_type_tick = 0;    // <0 = inter-key gap

static bool RAM_FUNC dscan_for(char c, uint8_t *dscan, bool *shift) {
    *shift = false;
    if (c >= 'A' && c <= 'Z') { *dscan = (uint8_t)(DSCAN_A + (c - 'A')); return true; }
    if (c >= '0' && c <= '9') { *dscan = (uint8_t)(DSCAN_0 + (c - '0')); return true; }
    switch (c) {
        case '"': *dscan = DSCAN_2; *shift = true; return true;   // CoCo " is SHIFT+2
        case ':': *dscan = DSCAN_COLON;     return true;
        case '.': *dscan = DSCAN_FULL_STOP; return true;
        case '/': *dscan = DSCAN_SLASH;     return true;
        case '-': *dscan = DSCAN_MINUS;     return true;
        case ' ': *dscan = DSCAN_SPACE;     return true;
        case 10 : *dscan = DSCAN_ENTER;     return true;
        default : return false;
    }
}

static void RAM_FUNC autotype_task(void) {
    if (g_type_pos < 0) return;
    if (g_type_tick < 0) { g_type_tick++; return; }
    char c = g_type_buf[g_type_pos];
    if (!c) { g_type_pos = -1; return; }
    uint8_t d = 0; bool sh = false;
    bool ok = dscan_for(c, &d, &sh);
    if (g_type_tick == 0 && ok) {
        if (sh) coco_machine_press_key(DSCAN_SHIFT);
        coco_machine_press_key(d);
    }
    if (++g_type_tick >= TYPE_HOLD_FIELDS) {
        if (ok) {
            coco_machine_release_key(d);
            if (sh) coco_machine_release_key(DSCAN_SHIFT);
        }
        g_type_pos++;
        g_type_tick = -TYPE_GAP_FIELDS;
    }
}

// 0 = idle. 1 = cold boot requested, waiting for the screen to CLEAR. 2 = waiting
// for a SETTLED OK prompt. Clear-then-OK, not just OK: the pre-reset screen may
// still show an OK, and firing on that would type into the machine we are about
// to wipe. The cold reset zeroes RAM so the screen really does go blank first.
// The first OK is not enough either. Measured: the prompt appeared 1.18 s after
// the button and typing began at once, and leading characters were swallowed -
// DECB prints its banner and OK before it is polling the key matrix.
//   STABLE - OK must be continuously present; kills a transient match while the
//            screen is still filling (the check reads only two cells of one row).
//   SETTLE - a further wait after that, covering the gap between the prompt
//            appearing and DECB accepting keys, which no screen test can see.
// Characters lost at the start means raise SETTLE.
#define AUTORUN_OK_STABLE_FIELDS 45
#define AUTORUN_SETTLE_FIELDS    60
static int g_autorun    = 0;
static int g_autorun_ok = 0;

static void autorun_start(void) {
    char name[16]; int type = 0;
    // DRIVE 0 specifically, not whichever drive was just assigned: the command
    // typed below is a bare RUN"NAME", which RSDOS resolves against its DEFAULT
    // drive. Reading the directory of drive 1 and then typing a command that
    // searches drive 0 would name a program the machine cannot find.
    if (!dsk_first_program(0, name, sizeof(name), &type)) {
        if (Serial && Serial.availableForWrite() >= 48)
            Serial.println("[autorun: no runnable program in directory]");
        return;
    }
    if (type == 0) snprintf(g_type_buf, sizeof(g_type_buf), "RUN\"%s\"\n", name);
    else           snprintf(g_type_buf, sizeof(g_type_buf), "LOADM\"%s\":EXEC\n", name);
    g_type_pos = 0; g_type_tick = 0;
    if (Serial && Serial.availableForWrite() >= 64)
        Serial.printf("[autorun: %s (type %d)]\n", name, type);
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

// The picker list is just the disks (FRUITJAM-78).
//
// It carried a trailing [CANCEL] row from FRUITJAM-70, whose purpose was to give
// the NO-KEYBOARD path a cancel: ESC and F12 both need a keyboard, and button 2
// always commits, which since FRUITJAM-69 also means a COLD BOOT — so opening
// the picker by accident with no keyboard left no way out but to mount
// something and reboot. FRUITJAM-70 chose a list row over a chord because a row
// is discoverable and needs no new gesture.
//
// FRUITJAM-78 removed the row, because assignment moved to the 0-3 keys and
// ENTER went inert: the row became unreachable from the keyboard and existed
// only for button 2. The no-keyboard exit it provided is REAL and still
// required, so it moved to the outer-button chord in picker_task rather than
// being dropped. That reintroduces the discoverability cost FRUITJAM-70 was
// avoiding — an undocumented chord — which is tracked in FRUITJAM-67.
//
// Note the empty case it also used to cover: with no .dsk files the list is now
// empty, so the overlay prints NO .DSK FILES and the chord is the way out.

static void fb_char(int cx, int cy, char c, uint16_t fg, uint16_t bg) {
    if (c >= 'a' && c <= 'z') c -= 32;
    uint8_t idx;
    if (c >= '@' && c <= '_')      idx = (uint8_t)(c - '@');        // 0x00-0x1F
    else if (c >= ' ' && c <= '?') idx = (uint8_t)c;                // 0x20-0x3F
    else                           idx = 0x20;                      // space
    if (cx < 0 || cy < 0 || cx >= OVL_COLS || cy >= OVL_ROWS) return;
#if COCO_CROP_BORDER
    // Rows are 1:1 with the CoCo here (vertical scaling happens at scanout), so
    // only the columns need the 2.5x map. cx*8 is even, so a cell starts at
    // exactly cx*20 and glyphs stay cell-aligned.
    int py = cy * 12;
    for (int r = 0; r < 12; r++) {
        uint8_t bits = font_6847[idx * 12 + r];
        uint16_t *row = g_scan[py + r];
        for (int b = 0; b < 8; b++) {
            uint16_t c = (bits & (0x80 >> b)) ? fg : bg;
            for (int o = SPAN_LO(cx * 8 + b); o < SPAN_HI(cx * 8 + b); o++)
                row[o] = c;
        }
    }
#else
    int px = OX + cx * 8, py = OY + cy * 12;
    for (int r = 0; r < 12; r++) {
        uint8_t bits = font_6847[idx * 12 + r];
        uint16_t *row = &g_fb.px[py + r][px];
        for (int b = 0; b < 8; b++) row[b] = (bits & (0x80 >> b)) ? fg : bg;
    }
#endif
}

static void fb_text(int cx, int cy, const char *s, uint16_t fg, uint16_t bg) {
    for (int i = 0; s[i]; i++) fb_char(cx + i, cy, s[i], fg, bg);
}

// Drawn over the emulator frame each field while the picker is open.
static void draw_picker(void) {
    // Reuse the machine palette. The selection is TRUE REVERSE VIDEO — black on
    // green — not the old dark-green-on-black, which paired the two darkest
    // entries in the palette and read as dimmed rather than highlighted.
    const uint16_t fg  = g_pal[0];    // green      — normal text
    const uint16_t hi  = g_pal[8];    // black      — text on the selected row
    const uint16_t bg  = g_pal[9];    // dark green — panel background
    const int      COLS = OVL_COLS;             // 32, the CoCo text width
    const int      rows_visible = PICK_ROWS;

    char line[OVL_COLS + 1];
    for (int i = 0; i < COLS; i++) line[i] = ' ';
    line[COLS] = '\0';

    // Paint the full 32x16 panel, not just the rows with text: any cell left
    // unpainted would still hold the last blitted emulator frame and show through.
    for (int y = 0; y < OVL_ROWS; y++)
        for (int x = 0; x < OVL_COLS; x++) fb_char(x, y, ' ', fg, bg);

    int top = g_pick_sel - rows_visible / 2;
    if (top > PICK_COUNT - rows_visible) top = PICK_COUNT - rows_visible;
    if (top < 0) top = 0;

    // Header names the drive being assigned, because with four drives the list
    // alone is ambiguous — the same disk list means something different
    // depending on which drive it lands in.
    char hdr[OVL_COLS + 1];
    // Title as a full-width bar. The key legends are gone: they cost two of
    // sixteen rows permanently to teach four keys once.
    // Starts at column 0, directly over the four drive columns, so it reads as
    // a heading for them rather than as a floating panel title.
    snprintf(hdr, sizeof(hdr), "DISK ASSIGNMENTS");
    for (size_t k = strlen(hdr); k < (size_t)COLS; k++) hdr[k] = ' ';
    hdr[COLS] = '\0';
    // Green on black: a recessed bar, which stays inside the CoCo green scheme
    // and still reads as distinct from the selection's bright green bar below.
    fb_text(0, 0, hdr, fg, hi);

    for (int r = 0; r < rows_visible; r++) {
        int i = top + r;
        for (int c = 0; c < COLS; c++) line[c] = ' ';
        if (i < g_dsk_count) {
            // "N -> NAME". The drive digit sits to the LEFT of the name, so the
            // digits read down the edge as the whole drive map at a glance —
            // which a trailing marker could not do with names of varying length.
            // A disk assigned to more than one drive shows the lowest.
            //
            // The arrow is drawn ONLY on assigned rows: a bare "-> NAME" with no
            // number ahead of it reads as a dangling arrow, so unassigned rows
            // get plain indent instead. No cursor character — the reverse-video
            // bar is the selection, and a '>' beside it was redundant.
            // Columns 0-3 are the four DRIVES, one fixed column each: column d
            // shows the digit d when drive d holds this disk, blank otherwise.
            //
            // A fixed column per drive rather than one shared digit, because a
            // disk can legitimately sit in more than one drive and a single
            // column could only ever show one of them. Here "0 2  " reads
            // directly as "drives 0 and 2", and because each drive owns a fixed
            // column, scanning straight down column d shows where drive d is —
            // and that at most one row in it can be lit.
            char pre[COCO_NDRIVE + 2];
            for (int d = 0; d < COCO_NDRIVE; d++)
                pre[d] = (g_dsk_cur[d] == i) ? (char)('0' + d) : ' ';
            pre[COCO_NDRIVE]     = ' ';
            pre[COCO_NDRIVE + 1] = '\0';
            snprintf(line, sizeof(line), "%s%-*.*s", pre,
                     DSK_NAME_COLS, DSK_NAME_COLS, g_dsk_names[i]);
        }
        // Selected row: black on green, and the line is padded to the full 32
        // columns above so the highlight reads as a solid bar.
        fb_text(0, 1 + r, line, (i == g_pick_sel) ? hi : fg, (i == g_pick_sel) ? fg : bg);
    }
    if (g_dsk_count == 0) fb_text(0, 1, "NO .DSK FILES IN /COCO/DSK", fg, bg);

    // Status bar, same treatment as the title: green on black, padded to the
    // full width, and drawn ALWAYS rather than only when there is a message —
    // so the panel keeps a bar top and bottom instead of the list appearing to
    // change height as one comes and goes.
    char sts[OVL_COLS + 1];
    snprintf(sts, sizeof(sts), "%s", g_pick_msg);
    for (size_t k = strlen(sts); k < (size_t)COLS; k++) sts[k] = ' ';
    sts[COLS] = '\0';
    fb_text(0, 1 + rows_visible, sts, fg, hi);
}

// Poll the buttons and drive the picker. Called once per field from loop().
// FRUITJAM-66: buttons are 3=UP, 2=MOUNT+RESET, 1=DOWN. Button 2 sits in the
// middle of the physical row, which is where a select key belongs, and leaves
// the two outer buttons as the up/down pair.
static void RAM_FUNC picker_task(void) {
    bool next = btn_fell(g_btn[0]);   // button 1 -> down / next
    bool sel  = btn_fell(g_btn[1]);   // button 2 -> mount + warm reset
    bool prev = btn_fell(g_btn[2]);   // button 3 -> up / prev

    // FRUITJAM-67: BOTH OUTER BUTTONS HELD closes the overlay with no mount and
    // no reboot. This replaces the [CANCEL] row, and it has to exist: every
    // other button-2 press mounts a disk and cold-boots, so without it a user
    // with no keyboard could not leave the picker without rebooting into some
    // disk. Buttons are active low, and btn_fell() has already debounced
    // .last this pass, so it doubles as the current held level.
    //
    // Checked BEFORE the no-edge early return below, because a chord is a HELD
    // state and may present no new edge on the pass that completes it.
    if (g_pick_open && !g_btn[0].last && !g_btn[2].last) {
        g_pick_open = false; g_pick_dirty = true;
        return;
    }
    if (!sel && !prev && !next) return;

    if (Serial && Serial.availableForWrite() >= 48)
        Serial.printf("[btn] %s%s%s open=%d sel=%d/%d\n",
                      // Label by PHYSICAL button, not by action. These were
                      // 1/2/3 when sel/prev/next mapped to buttons 1/2/3; after
                      // the FRUITJAM-66 remap the old labels reported the wrong
                      // button — pressing 3 logged "[btn] 2".
                      sel ? "2" : "", prev ? "3" : "", next ? "1" : "",
                      (int)g_pick_open, g_pick_sel, g_dsk_count);

    if (prev || next) {
        if (!g_pick_open) {
            g_pick_open = true; g_pick_msg[0] = '\0'; g_pick_dirty = true;
            // Same rule as the keyboard path: open on the drive DSKREG says the
            // machine last addressed.
            int d0 = coco_machine_fdc_drive();
            if ((unsigned)d0 >= COCO_NDRIVE) d0 = 0;
            g_pick_sel = pick_row_for_drive(d0);
            return;
        }
        g_pick_dirty = true;
        g_pick_sel += next ? 1 : -1;
        if (g_pick_sel < 0)            g_pick_sel = PICK_COUNT - 1;
        if (g_pick_sel >= PICK_COUNT)  g_pick_sel = 0;
        return;
    }
    if (sel) {
        if (!g_pick_open) {
            g_pick_open = true; g_pick_msg[0] = '\0'; g_pick_dirty = true;
            // Same rule as the keyboard path: open on the drive DSKREG says the
            // machine last addressed.
            int d0 = coco_machine_fdc_drive();
            if ((unsigned)d0 >= COCO_NDRIVE) d0 = 0;
            g_pick_sel = pick_row_for_drive(d0);
            return;
        }
        // Button 2 assigns DRIVE 0 and cold-boots. The digit keys that drive
        // the four-drive assignment are unreachable without a keyboard, so the
        // button path stays what it has always been: pick a disk, boot it.
        mount_dsk_index(g_pick_sel, 0, true);
        // FRUITJAM-66/69: button 2 mounts and then FULLY restarts — a cold boot
        // with the selected disk in the drive, not a floppy swapped under a
        // running DOS. Warm reset was not enough: it preserves RAM, so BASIC
        // warm-starts with the previous program and variables still in place.
        // Mount BEFORE the reset, so the disk is already in drive 0 when the
        // ROM boots.
        // NOTE this DIFFERS from keyboard ENTER, which still does a live floppy
        // swap with no reset at all — the FRUITJAM-50 behaviour. The two inputs
        // are deliberately no longer equivalent; see FRUITJAM-67.
        coco_machine_cold_reset();
        g_autorun   = 1;     // FRUITJAM-72: auto-run once DECB reaches its prompt
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
static void RAM_FUNC audio_feed() {
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
                   k_dn  = newly(0x51), k_ent = newly(0x28),
                   k_esc = newly(0x29), k_f11 = newly(0x44);
        // HID: Home 0x4A, PgUp 0x4B, End 0x4D, PgDn 0x4E.
        const bool k_home = newly(0x4A), k_pgup = newly(0x4B),
                   k_end  = newly(0x4D), k_pgdn = newly(0x4E);
        // HID 0x1E-0x21 are '1'..'4'; 0x27 is '0'. Only read while the overlay
        // is open, so the digits stay ordinary typing the rest of the time.
        int k_drive = -1;
        if      (newly(0x27)) k_drive = 0;
        else if (newly(0x1E)) k_drive = 1;
        else if (newly(0x1F)) k_drive = 2;
        else if (newly(0x20)) k_drive = 3;

        memcpy(pk_prev, codes, 6);

        // FRUITJAM-73: F11 cycles PMODE 4 artifact colour, OFF -> phase A ->
        // phase B -> OFF. Three states rather than a plain toggle: without OFF
        // there is no way back to plain mono, and BOTH phases are needed because
        // which one a given game expects is arbitrary, exactly as on real
        // hardware. Handled before the modal check so it works whether or not the
        // picker is open. F11 is unmapped in g_hid_to_dscan, so it cannot leak.
        if (k_f11) {
            int m = (coco_machine_get_artifact() + 1) % 3;
            coco_machine_set_artifact(m);
            if (Serial && Serial.availableForWrite() >= 48)
                Serial.printf("[artifact: %s]\n",
                              m == 0 ? "off" : m == 1 ? "phase A" : "phase B");
        }

        // F12 OPENS the overlay, and once open STEPS THROUGH THE DRIVES
        // (0->1->2->3->0) rather than closing it. Closing is ESC's job alone.
        // That makes F12 a single "next drive" key, so assigning all four is one
        // visit — press F12 to advance, ENTER to assign, repeat — instead of
        // reopening the overlay once per drive.
        // F12 toggles the overlay. Assignment is 0-3, so F12 has no second job.
        if (k_f12) {
            g_pick_open  = !g_pick_open;
            g_pick_dirty = true;
            if (g_pick_open) {
                g_pick_msg[0] = '\0';
                // Open on whatever the machine last addressed (DSKREG), so after
                // `DRIVE 1` + `DIR` the highlight starts on drive 1's disk.
                int d = coco_machine_fdc_drive();
                if ((unsigned)d >= COCO_NDRIVE) d = 0;
                g_pick_sel = pick_row_for_drive(d);
                // Drop anything the emulator currently thinks is held, or a key
                // down at the moment F12 arrives would stay stuck for the whole
                // time the picker is up.
                coco_machine_release_all_keys();
                g_shift_prev = false;
                memset(g_prev_codes, 0, sizeof(g_prev_codes));
            }
        }
        if (g_pick_open) {
            // FRUITJAM-68: ESC cancels — closes the overlay without mounting,
            // which also unhalts the 6809, since the halt in loop() is gated
            // purely on g_pick_open.
            //
            // ESC is CANCEL-ONLY, never a toggle like F12, because HID 0x29 maps
            // to DSCAN_BREAK: the CoCo BREAK key. If ESC opened the overlay too,
            // BREAK would become unreachable while the picker is closed, which
            // is most of the time.
            //
            // Closing needs more care than F12 does. F12 is unmapped, so letting
            // it fall through is harmless; ESC is not. While the overlay is open
            // this block returns early, so g_prev_codes is never updated — and
            // the open path zeroed it. Simply closing would leave the NEXT report
            // seeing a still-held ESC as newly pressed and fire BREAK into BASIC.
            // Copying the live codes into g_prev_codes marks it already-down, so
            // no press is generated; the eventual release then targets a key that
            // was never pressed, which the matrix ignores.
            if (k_esc) {
                g_pick_open  = false;
                g_pick_dirty = true;
                coco_machine_release_all_keys();
                g_shift_prev = false;
                memcpy(g_prev_codes, codes, 6);
                return;
            }
            // 0-3 switch which drive is being assigned. The selection follows
            // that drive's current image, so switching drives shows you what is
            // in the one you moved to rather than stranding the cursor.
            // 0-3 TOGGLE the highlighted disk in that drive: assign it, or
            // unassign if it is already the one there. One key does both, so
            // emptying a drive needs no separate gesture or row.
            if (k_drive >= 0 && g_dsk_count) {
                if (g_dsk_cur[k_drive] == g_pick_sel) eject_drive(k_drive, true);
                else                                  mount_dsk_index(g_pick_sel, k_drive, true);
                g_pick_dirty = true;
            }
            if ((k_up || k_dn) && g_dsk_count) {
                g_pick_sel += k_dn ? 1 : -1;
                if (g_pick_sel < 0)           g_pick_sel = PICK_COUNT - 1;
                if (g_pick_sel >= PICK_COUNT) g_pick_sel = 0;
                g_pick_dirty = true;
            }
            // Home/End/PgUp/PgDn. These CLAMP rather than wrap, unlike Up/Down:
            // wrapping a page jump makes it impossible to say where you will
            // land, whereas Home and End are only useful if they are absolute.
            // A page is exactly one screenful, so PgDn then PgUp returns you to
            // the row you started on wherever the list is long enough.
            if ((k_home || k_end || k_pgup || k_pgdn) && g_dsk_count) {
                if      (k_home) g_pick_sel = 0;
                else if (k_end)  g_pick_sel = PICK_COUNT - 1;
                else             g_pick_sel += k_pgdn ? PICK_ROWS : -PICK_ROWS;
                if (g_pick_sel < 0)           g_pick_sel = 0;
                if (g_pick_sel >= PICK_COUNT) g_pick_sel = PICK_COUNT - 1;
                g_pick_dirty = true;
            }
            // ENTER is deliberately inert: assignment is 0-3 and closing is
            // F12 or ESC. It stays swallowed rather than reaching the machine.
            (void)k_ent;
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

// FRUITJAM-18 step 0: identify what actually enumerated, before writing any
// gamepad mapping. Confirmed by reading TinyUSB hid_host.c: itf_protocol is only
// populated for HID_SUBCLASS_BOOT interfaces (:524) and SET_PROTOCOL is only
// issued when it is non-NONE (:581), so tuh_hid_set_default_protocol(BOOT) never
// touches a gamepad — it stays in report mode, and reads back as protocol NONE.
// That makes "protocol == NONE" the gamepad discriminator here.
// Raw-report logging. Default OFF: it is a bring-up aid, and per CLAUDE.md
// serial load is itself a variable in this repo.
//   1 = log raw reports on CHANGE — identifies a new pad's byte layout.
//   2 = per-axis min/max/current once a second — AT-REST calibration.
// Mode 2 exists because mode 1 cannot measure a stationary stick: a still axis
// emits no changes, so "no drift" and "pad unplugged" produce identical output
// (zero lines), and the resting VALUE is never shown at all. FRUITJAM-92.
#ifndef COCO_JOY_PROBE
#define COCO_JOY_PROBE 0
#endif

// - - - gamepad -> CoCo joysticks (FRUITJAM-18) -------------------------------
// Report layout MEASURED on the user's pad (VID 054C PID 09CC, the DualShock 4
// identity of a multi-mode pad; it also enumerates as 057E:2009 Switch Pro, which
// sends no input reports without a handshake and is deliberately NOT supported).
// Every field below was confirmed by pressing the control and reading the bytes,
// not taken from a datasheet:
//
//   b0  report ID, always 0x01
//   b1  LX   00 = left,  80 = centre, FF = right
//   b2  LY   00 = up,    80 = centre, FF = down
//   b3  RX,  b4 RY   (same convention)
//   b5  low nibble  = D-pad hat: 0=N 1=NE 2=E 3=SE 4=S 5=SW 6=W 7=NW, 0x0F = idle
//       high nibble = face buttons, bit4 square, bit5 cross, bit6 circle, bit7 triangle
//   b6  bit0 L1, bit1 R1, bit2 L2, bit3 R2, bit6 L3, bit7 R3
//   b8/b9  L2 / R2 analog
//
// NOTE the hat idle value is 0x0F, not the 0x08 a stock DS4 reports — hence the
// ">= 8 means centred" test rather than a compare against 8.
//
// MAPPING (FRUITJAM-18, as specified): left stick -> RIGHT CoCo joystick (port 0,
// the one nearly all software reads), right stick -> LEFT (port 1). The D-pad
// OVERRIDES the left stick when pressed, snapped to the rail, so the pad is
// usable even in digital mode where the sticks read dead-centre.
#define DS4_VID 0x054C
#define DS4_PID 0x09CC

static uint8_t g_pad_daddr = 0;      // 0 = no pad bound
static uint8_t g_pad_idx   = 0;

// Deadzone. MEASURED at rest, FRUITJAM-92: 105 one-second windows with the pad
// untouched on the desk, tracking per-window min/max of all four axes. 86 of
// them read exactly 80-80 on every axis — zero LSBs of jitter — and the 19 that
// moved were one contiguous 23-second burst of the pad being handled, with a
// clean unbroken 58 seconds after it. Centre is exactly 0x80 on all four axes,
// so there is no per-axis offset to calibrate either.
//
// This pad therefore needs NO drift compensation, and the 0x18 first guess was
// discarding 19% of half-travel for nothing. Kept small but non-zero for two
// reasons: insurance against unit variation and wear, and — the concrete one —
// it snaps rest to EXACT centre, which the scaling below cannot do on its own.
// 8-bit 0..255 has no exact midpoint, so 0x80 * 257 = 32896 rather than 32767;
// without a deadzone the emulated stick would sit ~129/65535 off centre forever.
#define PAD_DEADZONE 0x06

static inline uint16_t pad_axis(uint8_t v) {
    int d = (int)v - 0x80;
    if (d > -PAD_DEADZONE && d < PAD_DEADZONE) return 32767;
    return (uint16_t)(v * 257);      // 0..255 -> 0..65535, 0xFF -> 0xFFFF exactly
}

static void pad_apply(const uint8_t *r, uint16_t len) {
    if (len < 7 || r[0] != 0x01) return;

    // Port 0 (RIGHT CoCo stick) from the left stick, then let the D-pad override.
    uint16_t x0 = pad_axis(r[1]), y0 = pad_axis(r[2]);
    uint8_t hat = (uint8_t)(r[5] & 0x0F);
    if (hat < 8) {                    // 8..0x0F all mean "not pressed"
        // 0=N 1=NE 2=E 3=SE 4=S 5=SW 6=W 7=NW
        static const int8_t HX[8] = {  0, +1, +1, +1,  0, -1, -1, -1 };
        static const int8_t HY[8] = { -1, -1,  0, +1, +1, +1,  0, -1 };
        if (HX[hat]) x0 = HX[hat] > 0 ? 65535 : 0;
        if (HY[hat]) y0 = HY[hat] > 0 ? 65535 : 0;
    }
    coco_machine_set_joystick_axis(0, 0, x0);
    coco_machine_set_joystick_axis(0, 1, y0);

    // Port 1 (LEFT CoCo stick) from the right stick.
    coco_machine_set_joystick_axis(1, 0, pad_axis(r[3]));
    coco_machine_set_joystick_axis(1, 1, pad_axis(r[4]));

    // Fire. Cross is the primary right-hand fire; circle mirrors it so either
    // thumb position works. Square drives the left joystick's button.
    bool cross = (r[5] & 0x20) != 0, circle = (r[5] & 0x40) != 0;
    bool square = (r[5] & 0x10) != 0;
    coco_machine_set_joystick_fire(0, cross || circle || (r[6] & 0x02));  // + R1
    coco_machine_set_joystick_fire(1, square || (r[6] & 0x01));           // + L1
}

extern "C" void tuh_hid_mount_cb(uint8_t daddr, uint8_t idx,
                                 uint8_t const *desc, uint16_t len) {
    (void)desc; (void)len;
#if COCO_JOY_PROBE
    uint16_t vid = 0, pid = 0;
    tuh_vid_pid_get(daddr, &vid, &pid);
    Serial.printf("[HID] mount daddr=%u idx=%u VID=%04X PID=%04X proto=%u descLen=%u\n",
                  daddr, idx, vid, pid, tuh_hid_interface_protocol(daddr, idx), len);
    Serial.flush();
#else
    uint16_t vid = 0, pid = 0;
    tuh_vid_pid_get(daddr, &vid, &pid);
#endif
    // Bind the first DS4-identity interface we see as THE pad. Matching on
    // VID/PID rather than "any non-keyboard interface" matters because the
    // keyboard itself presents two extra non-keyboard interfaces (measured:
    // 3434:0430 idx 1 and 2), which would otherwise be decoded as gamepad
    // reports and jam the joystick.
    if (!g_pad_daddr && vid == DS4_VID && pid == DS4_PID) {
        g_pad_daddr = daddr; g_pad_idx = idx;
    }
    tuh_hid_receive_report(daddr, idx);
}
extern "C" void tuh_hid_umount_cb(uint8_t daddr, uint8_t idx) {
    coco_machine_release_all_keys();     // drop stuck keys if the keyboard leaves
    memset(g_prev_codes, 0, sizeof(g_prev_codes));
    g_shift_prev = false;
    // Centre the sticks and drop fire if the PAD leaves, so unplugging it does
    // not strand the emulated joystick at full deflection.
    if (daddr == g_pad_daddr && idx == g_pad_idx) {
        g_pad_daddr = 0;
        coco_machine_release_all_joysticks();
    }
}
extern "C" void tuh_hid_report_received_cb(uint8_t daddr, uint8_t idx,
                                           uint8_t const *report, uint16_t len) {
    // Only the boot-keyboard interface (8-byte reports). Others (consumer/etc.)
    // are ignored but must still be re-armed.
    uint8_t proto = tuh_hid_interface_protocol(daddr, idx);
    if (proto == HID_ITF_PROTOCOL_KEYBOARD && len >= 8)
        hid_keyboard_apply(report);
    else if (proto != HID_ITF_PROTOCOL_KEYBOARD && daddr == g_pad_daddr && idx == g_pad_idx)
        pad_apply(report, len);
#if COCO_JOY_PROBE >= 2
    // At-rest calibration: running min/max per axis, reported once a second.
    // Unconditional (not change-gated) so a motionless stick still produces
    // output, and so absence of output means "no reports", unambiguously.
    // NOTE this is a STANDALONE if, deliberately not part of the dispatch chain
    // above: the bound pad is consumed by the pad_apply branch, so an else-if
    // here would only ever see devices that are NOT the pad — i.e. never fire.
    if (proto != HID_ITF_PROTOCOL_KEYBOARD && len >= 5 && report[0] == 0x01) {
        static uint8_t  mn[4] = { 255, 255, 255, 255 };
        static uint8_t  mx[4] = { 0, 0, 0, 0 };
        static uint32_t last = 0, n = 0;
        for (int i = 0; i < 4; i++) {
            uint8_t v = report[1 + i];
            if (v < mn[i]) mn[i] = v;
            if (v > mx[i]) mx[i] = v;
        }
        n++;
        if (millis() - last >= 1000) {
            last = millis();
            Serial.printf("[JOY] n=%lu cur %02X %02X %02X %02X | LX %02X-%02X LY %02X-%02X "
                          "RX %02X-%02X RY %02X-%02X\n", (unsigned long)n,
                          report[1], report[2], report[3], report[4],
                          mn[0], mx[0], mn[1], mx[1], mn[2], mx[2], mn[3], mx[3]);
            // Min/max are PER-INTERVAL, reset after each print. Cumulative-since-
            // boot was useless here: it latched the extremes thrown by the pad's
            // Switch->DS4 re-enumeration and then reported 00-FF forever, which
            // looks exactly like catastrophic drift and is not.
            for (int i = 0; i < 4; i++) { mn[i] = 255; mx[i] = 0; }
        }
    }
#elif COCO_JOY_PROBE
    // Log non-keyboard reports, but ONLY when the bytes change. A pad streams at
    // 100+ Hz; logging every report would swamp the link and, per CLAUDE.md,
    // serial load is itself a variable in this repo. Change-only turns a stream
    // into one line per physical input event, which is what we actually want to
    // read off.
    else if (proto != HID_ITF_PROTOCOL_KEYBOARD && len) {
        // Only the first 12 bytes: on a DS4-style 64-byte report everything past
        // the buttons is gyro/accel/touchpad, which changes every single report
        // and would flood the link even with change-only logging.
        static uint8_t  prev[24];
        static uint16_t prev_len = 0;
        uint16_t n = len > sizeof(prev) ? (uint16_t)sizeof(prev) : len;
        if (n != prev_len || memcmp(prev, report, n) != 0) {
            memcpy(prev, report, n); prev_len = n;
            Serial.printf("[HID] rpt idx=%u len=%u:", idx, len);
            for (uint16_t i = 0; i < n; i++) Serial.printf(" %02X", report[i]);
            Serial.println();
        }
    }
#endif
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
volatile uint32_t g_resync_count = 0;

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
    coco_machine_set_dsk_write_callback(dsk_sector_written);   // FRUITJAM-81
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
        // FRUITJAM-50: enumerate the card FIRST — resolving a remembered NAME
        // back to an index needs the list to exist.
        scan_dsk_dir();

        // FRUITJAM-79: fill the PSRAM image cache. This runs BEFORE
        // multicore_launch_core1(), so no video is being scanned out and the SD
        // traffic cannot disturb the HSTX link — the reason boot has never shown
        // the fault that every runtime mount does (FRUITJAM-97).
        {
            uint32_t t0 = millis();
            for (int i = 0; i < g_dsk_count; i++) {
                char path[48];
                snprintf(path, sizeof(path), "0:/coco/dsk/%s", g_dsk_names[i]);
                const uint8_t *img = nullptr;
                size_t len = load_psram_file(path, &img);
                if (!len) continue;                         // unreadable: leave uncached
                if (g_dsk_cached_sz + len > DSK_CACHE_BUDGET) {
                    __psram_free((void *)img);              // over budget: stop caching
                    break;
                }
                g_dsk_cache[i]     = (uint8_t *)img;
                g_dsk_cache_len[i] = len;
                g_dsk_cached_sz   += len;
                g_dsk_cached_n++;
            }
            Serial.printf("[dsk cache: %d/%d images, %u KB, %lu ms] ",
                          g_dsk_cached_n, g_dsk_count,
                          (unsigned)(g_dsk_cached_sz / 1024),
                          (unsigned long)(millis() - t0));
        }

        // FRUITJAM-71/78 precedence: the saved per-drive assignments, then
        // AUTO.DSK in drive 0, then coco.dsk. The user's most recent explicit
        // choice outranks the static AUTO.DSK marker; delete lastdsk.txt to
        // fall back to it.
        char saved[COCO_NDRIVE][DSK_NAME_MAX];
        load_dsk_assignments(saved, COCO_NDRIVE);
        int  restored = 0;

        // Drives 1-3 first: they are pure restore, with no AUTO.DSK fallback and
        // no autorun, so they cannot affect how drive 0 is chosen below.
        for (int d = 1; d < COCO_NDRIVE; d++) {
            if (!saved[d][0]) continue;
            int idx = -1;
            for (int i = 0; i < g_dsk_count; i++)
                if (strcasecmp(g_dsk_names[i], saved[d]) == 0) { idx = i; break; }
            if (idx >= 0 && mount_dsk_index(idx, d, false)) restored++;
            else if (Serial && Serial.availableForWrite() >= 64)
                Serial.printf("[drive %d disk '%s' not on card] ", d, saved[d]);
        }

        int  want = -1, from_last = 0;
        if (saved[0][0]) {
            from_last = 1;
            for (int i = 0; i < g_dsk_count; i++)
                if (strcasecmp(g_dsk_names[i], saved[0]) == 0) { want = i; break; }
            // A remembered disk that has since been removed from the card falls
            // through to AUTO.DSK rather than failing to boot.
            if (want < 0 && Serial && Serial.availableForWrite() >= 64)
                Serial.printf("[last disk '%s' not on card] ", saved[0]);
        }
        if (want < 0)
            for (int i = 0; i < g_dsk_count; i++)
                if (strcasecmp(g_dsk_names[i], "AUTO.DSK") == 0) { want = i; from_last = 0; break; }

        if (want >= 0 && mount_dsk_index(want, 0, false)) {
            // Say WHY this disk was chosen — otherwise a remembered disk and a
            // plain AUTO.DSK boot are indistinguishable in the log.
            Serial.printf("[disk mounted: %s (%s) -> DIR/LOAD] ", g_dsk_names[want],
                          from_last ? "remembered" : "AUTO.DSK");
        } else {
            const uint8_t *dsk_img = nullptr;
            size_t dsk = load_psram_file("0:/coco/roms/coco.dsk", &dsk_img);
            if (dsk) { g_dsk_img[0] = (uint8_t *)dsk_img;  // FRUITJAM-50: picker frees this on swap
                       g_dsk_len[0] = dsk;
                       coco_machine_mount_dsk_drive(0, g_dsk_img[0], dsk);
                       Serial.printf("[disk mounted: coco.dsk %u bytes -> DIR/LOAD] ", (unsigned)dsk); }
        }
        if (restored) Serial.printf("[drives 1-3: %d restored] ", restored);
        // Open the picker on whatever is in drive 0, so the highlight and the
        // drive agree the first time it is opened.
        g_pick_sel   = pick_row_for_drive(0);
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
#if COCO_CROP_BORDER
    // No border to paint — the active area now covers the whole screen. Build
    // the output-line -> source-row map once: 480 lines over 192 rows is the
    // same 2.5x, giving a 3,2,3,2 line-repeat pattern.
    for (int y = 0; y < COCO_VDG_H; y++)
        for (int x = 0; x < dvi::H_ACTIVE; x++) g_scan[y][x] = g_pal[8];
    for (int i = 0; i < dvi::V_ACTIVE; i++)
        g_row[i] = (const uint32_t *)g_scan[(i * COCO_VDG_H) / dvi::V_ACTIVE];
#if COCO_CROP_SMOOTH
    // Mode 1: 0.4/0.8/0.2/0.6 as 0..256 weights, in blit read order.
    // Mode 2: a single 50/50 table for the shared boundary pixel.
    static const uint16_t W[4] = { 102, 205, 51, 154 };
    for (int k = 0; k < (int)(sizeof(g_mix) / sizeof(g_mix[0])); k++)
        for (int a = 0; a < 16; a++)
            for (int b = 0; b < 16; b++)
                g_mix[k][a][b] =
                    mix565(g_pal[a], g_pal[b], COCO_CROP_SMOOTH == 1 ? W[k] : 128);
#endif
#else
    g_fb.fill(g_pal[8]);   // black border
#endif
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
#if COCO_CROP_BORDER
    // native_pixel_mode OFF: 32-bit transfers, so each of the 320 words carries
    // two DISTINCT pixels from the pre-scaled 640-wide row instead of one
    // bus-replicated pixel twice. Same transfer count, same pointer handoff,
    // 640 native pixels instead of 320 doubled ones.
    video_output_set_native_pixel_mode(false);
#else
    video_output_set_native_pixel_mode(true);
#endif
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
// FRUITJAM-76: loop() itself is the largest flash-resident piece of core 0's
// per-field path, so it goes to RAM along with everything it calls every field.
// This is not about these functions being slow — it is about making the timing
// INDEPENDENT OF WHERE THE LINKER PUTS THEM. FRUITJAM-75 measured 110 lines of
// never-executed code costing +1.5 ms/field and 964 desyncs/hour purely by
// shifting addresses; until the hot path is layout-immune, every A/B in this
// repo is confounded by the act of adding code.
// Still in flash by necessity: USBHost.task() (TinyUSB) and the Serial.printf
// report block. The report runs once a second, so it evicts cache lines rarely.


// FRUITJAM-97 BISECT. A mount does two very different things at once: it streams
// ~160 KB off the SD card over SPI, and it writes those bytes into PSRAM across
// the QMI. Every fix aimed at the bus has failed, so rather than guess again,
// run the two halves SEPARATELY and see which one drops the picture.
//
//   phase A - PSRAM writes only, no SD at all (memset a PSRAM buffer)
//   phase B - SD reads only, no PSRAM at all (read into a small SRAM scratch)
//
// Same byte count, same duration, one subsystem each. The user watches the screen
// and reports which marker coincides with a drop; the log cannot answer this,
// because the watchdog is blind to the silent mode (two of three mounts dropped
// the screen with the resync counters unmoved).
#ifndef COCO_MOUNT_BISECT
#define COCO_MOUNT_BISECT 0
#endif
#ifndef COCO_BISECT_GAP_US
#define COCO_BISECT_GAP_US 0
#endif
#if COCO_MOUNT_BISECT && PICO_HDMI_FIFO_PROBE
// Counted IN FIRMWARE and reported cumulatively, because the loads under test
// knock out the USB CDC link and per-trial lines get lost — the earlier attempt
// kept only 7-13 of 55 trials. Cumulative counters survive a dropout.
//
// g_bisect_clean is the metric that actually matches the symptom: a mount either
// visibly drops or it does not, so what matters is the FRACTION OF TRIALS WITH
// ZERO UNDERRUNS, not the mean. The distributions are heavy-tailed (mostly 0-1
// with occasional bursts of 13-29), so a mean measures outlier severity rather
// than how often a mount survives.
static uint32_t g_bisect_trials = 0, g_bisect_underruns = 0, g_bisect_clean = 0;
static uint32_t g_bisect_worst  = 0;
#endif
#if COCO_MOUNT_BISECT
static void mount_bisect_task(void) {
    // ONE PHASE PER BUILD. The four-phase cycling version was a bad experiment:
    // the phase markers went to SERIAL while the fault is only visible on the
    // SCREEN, so there was no way to attribute a drop to a phase and the readings
    // it produced were worthless. COCO_MOUNT_BISECT now SELECTS the phase —
    // 1=A, 2=B, 3=C, 4=D — and runs only that one, so the question is reduced to
    // "did the picture drop during this build, yes or no".
    static uint32_t next_ms = 0;
    static int      phase   = (COCO_MOUNT_BISECT - 1) & 3;
    static uint8_t *pbuf    = nullptr;
    const size_t    SZ      = 161280;          // one standard 35-track image
    uint32_t now = millis();
    if (now < 8000) return;                    // let boot settle
    if (next_ms && now < next_ms) return;
    next_ms = now + 5000;

    if (!pbuf) pbuf = (uint8_t *)__psram_malloc(SZ);
    if (!pbuf) { Serial.println("[bisect] PSRAM alloc failed"); return; }

    // Phase C is what a REAL mount does: SD read landing directly in PSRAM, so
    // the SPI transfer and the QMI writes overlap. Phase D is the candidate fix —
    // read into an SRAM bounce buffer, THEN copy to PSRAM, so the two never run
    // at the same time. A and B each proved harmless on their own once the SD
    // clock drive was fixed; if C drops and D does not, the fault is the OVERLAP.
    if (phase >= 2) {
        const bool bounce = (phase == 3);
        Serial.printf("[bisect] %s: SD read %u B -> PSRAM%s\n",
                      bounce ? "D" : "C", (unsigned)SZ,
                      bounce ? " via SRAM bounce" : " DIRECT (= real mount)");
        Serial.flush();
        uint32_t t = millis();
#if PICO_HDMI_FIFO_PROBE
        extern volatile uint32_t hstx_fifo_underruns;
        uint32_t ur0 = hstx_fifo_underruns;
#endif
        static uint8_t bbuf[PSRAM_READ_CHUNK];
        FIL f;
        if (f_open(&f, g_dsk_path[0][0] ? g_dsk_path[0] : "0:/coco/dsk/AUTO.DSK", FA_READ) == FR_OK) {
            UINT br = 0; size_t total = 0;
            while (total < SZ) {
                UINT want = (UINT)((SZ - total > PSRAM_READ_CHUNK) ? PSRAM_READ_CHUNK : SZ - total);
                FRESULT rc = bounce ? f_read(&f, bbuf, want, &br)
                                    : f_read(&f, pbuf + total, want, &br);
                if (rc != FR_OK || br == 0) { f_lseek(&f, 0); continue; }
                if (bounce) memcpy(pbuf + total, bbuf, br);
                total += br;
#if COCO_BISECT_GAP_US
                // Re-test of the Saturday pacing experiment, which was run at
                // 12 mA SCK when an SD read alone already dropped the link — it
                // never got a fair trial. Now that SD-alone survives (phase B),
                // a gap here lets the card idle between bursts.
                delayMicroseconds(COCO_BISECT_GAP_US);
#endif
            }
            f_close(&f);
#if PICO_HDMI_FIFO_PROBE
            // Each run is a TRIAL. Totals are accumulated and reported with the
            // per-second stats too, so a CDC dropout (which these loads cause)
            // cannot lose the measurement — the running mean is always readable.
            uint32_t d = hstx_fifo_underruns - ur0;
            g_bisect_trials++;
            g_bisect_underruns += d;
            if (d == 0) g_bisect_clean++;
            if (d > g_bisect_worst) g_bisect_worst = d;
            Serial.printf("[bisect] %s done in %lu ms, underruns +%lu\n",
                          bounce ? "D" : "C", (unsigned long)(millis() - t),
                          (unsigned long)d);
#else
            Serial.printf("[bisect] %s done in %lu ms\n", bounce ? "D" : "C",
                          (unsigned long)(millis() - t));
#endif
        } else {
            Serial.println("[bisect] no image to read");
        }
        return;
    }
    if (phase == 0) {
        Serial.printf("[bisect] A: PSRAM write %u B, no SD\n", (unsigned)SZ);
        Serial.flush();
        uint32_t t = millis();
        // Chunked exactly like load_psram_file, so the write pattern matches.
        for (size_t off = 0; off < SZ; off += PSRAM_READ_CHUNK)
            memset(pbuf + off, (int)(off & 0xFF),
                   (SZ - off > PSRAM_READ_CHUNK) ? PSRAM_READ_CHUNK : (SZ - off));
        Serial.printf("[bisect] A done in %lu ms\n", (unsigned long)(millis() - t));
    } else {
        Serial.printf("[bisect] B: SD read %u B -> SRAM, no PSRAM\n", (unsigned)SZ);
        Serial.flush();
        uint32_t t = millis();
        static uint8_t scratch[PSRAM_READ_CHUNK];
        FIL f;
        if (f_open(&f, g_dsk_path[0][0] ? g_dsk_path[0] : "0:/coco/dsk/AUTO.DSK", FA_READ) == FR_OK) {
            UINT br = 0; size_t total = 0;
            while (total < SZ) {
                if (f_read(&f, scratch, PSRAM_READ_CHUNK, &br) != FR_OK || br == 0) {
                    f_lseek(&f, 0);            // wrap so byte count matches phase A
                    continue;
                }
                total += br;
            }
            f_close(&f);
            Serial.printf("[bisect] B done in %lu ms\n", (unsigned long)(millis() - t));
        } else {
            Serial.println("[bisect] B: no image to read");
        }
    }
}
#endif

void RAM_FUNC loop() {
    static uint32_t deadline = 0;
    static uint32_t frames = 0, last_report = 0, run_us_acc = 0, blit_us_acc = 0;

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
    if (!g_pick_open) flush_dsk_writes(false);   // FRUITJAM-81: bounded write-back

    // FRUITJAM-72: after a button-2 cold boot, wait for a settled OK prompt then
    // type the disk's first program. Gated on the emulator actually running -
    // while the picker is open the 6809 is halted (FRUITJAM-66), so keystrokes
    // would never be consumed and the screen check would read a frozen frame.
    if (!g_pick_open) {
        if (g_autorun == 1) {
            if (!basic_at_prompt()) { g_autorun = 2; g_autorun_ok = 0; }
        } else if (g_autorun == 2) {
            g_autorun_ok = basic_at_prompt() ? g_autorun_ok + 1 : 0;
            if (g_autorun_ok >= AUTORUN_OK_STABLE_FIELDS + AUTORUN_SETTLE_FIELDS) {
                g_autorun = 0;
                autorun_start();
            }
        }
        autotype_task();
    }

    uint32_t t0 = micros();
    // FRUITJAM-50: while the picker is open, do NOT blit the emulator frame.
    // There is one framebuffer and no double-buffering (see g_fb), so core 1
    // scans out while core 0 writes. Painting the emulator frame and then
    // painting the overlay over it every field leaves a window in which scanout
    // catches the intermediate state — seen as a bar of emulator screen slowly
    // scanning down the picker. Freezing the buffer removes the race entirely:
    // the emulator keeps running, only its DISPLAY pauses, and the overlay is
    // redrawn just when it changes. Closing the picker resumes the blit, which
    // repaints the whole active area and erases the overlay on the next field.
    // FRUITJAM-66: HALT the 6809 while the overlay is up. Previously the machine
    // kept running and only its DISPLAY paused. Halting is better on two counts:
    // swapping a floppy under a STOPPED DOS is safer than under a running one,
    // and it hands ~13 ms/field of core 0 back exactly while the overlay is on
    // screen — which is when the user is looking at it and would most notice a
    // desync (core-0 headroom tracks desync rate, FRUITJAM-58).
    // Audio needs no special handling: audio_feed() already tops the I2S ring up
    // with g_audio_last on underrun, and a held constant sample is DC, i.e.
    // silence — not a sustained tone.
    if (g_pick_open) {
        if (g_pick_dirty) { draw_picker(); g_pick_dirty = false; }
    } else {
#if COCO_MOUNT_BISECT
        mount_bisect_task();
#endif
        coco_machine_run_cycles(CYCLES_PER_FRAME);
        coco_machine_render_frame();
        // FRUITJAM-61: time the blit separately from the rest of the field. It
        // is the one cost that changes between the default 2x path and the
        // cropped 2.5x one, and core-0 headroom is the variable that correlates
        // with desync rate (FRUITJAM-58) — so this number, not the total, is
        // what decides whether the cropped path is affordable.
        uint32_t tb = micros();
        blit_frame();
        blit_us_acc += micros() - tb;
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
            Serial.printf("emu %lu fields, avg run %lu us/field (%s%lu.%02lux real-time), video_frames=%lu (%lu fps), audio_peak=%d/32767\n",
                          (unsigned long)frames,
                          (unsigned long)(run_us_acc / (frames - last_report)),
                          // A halted emulator reports a meaningless multiple —
                          // 28x "real-time" when it is in fact not running at
                          // all. Mark it rather than let the log read as speed.
                          g_pick_open ? "HALTED, " : "",
                          // run_us_acc can now be ~0 for a whole report period
                          // if the overlay was open throughout (the emulator is
                          // halted), so guard the divide.
                          (unsigned long)(run_us_acc ? FRAME_US * (frames - last_report) / run_us_acc : 0),
                          (unsigned long)(run_us_acc ? (uint64_t)FRAME_US * (frames - last_report) * 100 / run_us_acc % 100 : 0),
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
#if PICO_HDMI_FIFO_PROBE
            {
                extern volatile uint32_t hstx_fifo_underruns, hstx_fifo_min_level;
                static uint32_t prev_ur = 0;
                uint32_t ur = hstx_fifo_underruns;
                if (Serial && Serial.availableForWrite() >= 64)
                    Serial.printf("  [fifo] underruns %lu (+%lu)  min_level %lu\n",
                                  (unsigned long)ur, (unsigned long)(ur - prev_ur),
                                  (unsigned long)hstx_fifo_min_level);
                prev_ur = ur;
                hstx_fifo_min_level = 0xFFFFFFFF;   // per-report minimum
#if COCO_MOUNT_BISECT
                if (g_bisect_trials && Serial.availableForWrite() >= 64)
                    Serial.printf("  [arm] trials %lu  CLEAN %lu (%lu%%)  underruns %lu  worst %lu\n",
                                  (unsigned long)g_bisect_trials,
                                  (unsigned long)g_bisect_clean,
                                  (unsigned long)(g_bisect_clean * 100 / g_bisect_trials),
                                  (unsigned long)g_bisect_underruns,
                                  (unsigned long)g_bisect_worst);
#endif
            }
#endif
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
                Serial.printf("  [cpu] PC=$%04X irq/s=%lu vdg=%08lx sam=$%04X mode=%02X%s\n",
                              coco_machine_get_pc(),
                              (unsigned long)(irq - last_irq), (unsigned long)h,
                              coco_machine_get_sam_f(), coco_machine_get_vdg_mode(),
                              static_s >= 5 ? "  ! SCREEN STATIC" : "");
            last_irq = irq; last_hash = h;
        }

        if (Serial && Serial.availableForWrite() >= 64)
            Serial.printf("  [blit] %lu us/field (%s, %d bpl)\n",
                          (unsigned long)(blit_us_acc / (frames - last_report)),
                          COCO_CROP_BORDER ? (COCO_CROP_SMOOTH == 1 ? "crop 2.5x linear"
                                            : COCO_CROP_SMOOTH == 2 ? "crop 2.5x edge"
                                                                    : "crop 2.5x hard")
                                            : "border 2x",
                          COCO_CROP_BORDER ? dvi::H_ACTIVE : COCO_VDG_W);
        blit_us_acc = 0;
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
