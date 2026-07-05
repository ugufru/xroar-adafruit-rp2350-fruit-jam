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
#include "pico/multicore.h"
#include "hardware/vreg.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/structs/bus_ctrl.h"
#include "pico/stdlib.h"

extern "C" {
#include "pico_hdmi/video_output.h"
#include "ff.h"
#include "f_util.h"
#include "coco_machine.h"
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
static volatile bool    g_video_ready = false;
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

static FATFS g_fs;
static bool mount_sd() {
    for (int a = 0; a < 5; a++) {
        if (f_mount(&g_fs, "0:", 1) == FR_OK) return true;
        delay(200);
    }
    return false;
}
static size_t load_rom(const char *path) {
    FIL f;
    if (f_open(&f, path, FA_READ) != FR_OK) return 0;
    UINT br = 0;
    f_read(&f, g_rom, sizeof(g_rom), &br);
    f_close(&f);
    return br;
}

void setup() {
    vreg_set_voltage(VREG_VOLTAGE_1_25);
    delay(2);
    set_sys_clock_khz(252000, true);

    Serial.begin(115200);
    const uint32_t start = millis();
    while (!Serial && (millis() - start) < 1500) delay(10);

    Serial.println();
    Serial.println("=== FRUITJAM-25 Boot Color BASIC -> DVI ===");
    Serial.flush(); delay(50);

    // DMA arbitration: pico_hdmi hardcodes DMA channels 0 and 1, but the SD
    // driver grabs channels via dma_claim_unused_channel() at mount and would
    // take 0/1 first, making video_output_init()'s dma_channel_claim(0/1) panic
    // (silent hang). Reserve 0/1 now so SD skips them, then release them just
    // before video init so pico_hdmi can claim them as it expects.
    dma_channel_claim(0);
    dma_channel_claim(1);

    Serial.print("STAGE mount SD... "); Serial.flush(); delay(20);
    if (!mount_sd()) { Serial.println("FATAL: no SD"); while (1) delay(500); }
    Serial.println("ok"); Serial.flush(); delay(20);

    Serial.print("STAGE load ROM... "); Serial.flush(); delay(20);
    size_t got = load_rom("0:/coco/roms/bas12.rom");
    if (got != 8192 && got != 16384) {
        Serial.printf("FATAL: bad ROM size %u\n", (unsigned)got);
        while (1) delay(500);
    }
    Serial.printf("ok (%u bytes)\n", (unsigned)got); Serial.flush(); delay(20);

    Serial.print("STAGE machine init... "); Serial.flush(); delay(20);
    if (!coco_machine_init(g_rom, got)) { Serial.println("FATAL: machine init"); while (1) delay(500); }
    Serial.println("ok"); Serial.flush(); delay(20);

    Serial.print("STAGE fill fb... "); Serial.flush(); delay(20);
    g_fb.fill(g_pal[8]);   // black border
    Serial.println("ok"); Serial.flush(); delay(20);

    Serial.print("STAGE video init... "); Serial.flush(); delay(20);
    dma_channel_unclaim(0);   // hand 0/1 back for pico_hdmi to claim (SD now on 2/3)
    dma_channel_unclaim(1);
    // pico_hdmi 2.0-beta native/precomposed path (README minimal pattern): init
    // at the NATIVE 320x240 source size; HDMI mode (not DVI — compose_service is
    // a no-op in DVI); compose ring + native pixel mode + a pointer callback that
    // returns the 320-wide RGB565 row (HSTX doubles to 640 in hardware); and the
    // compose service as core-1 background work so per-line ISR cost stays ~1.5us
    // and can't be starved by core-0 emulation load.
    video_output_init(dvi::FB_WIDTH, dvi::FB_HEIGHT);   // 320x240 native source
    video_output_set_dvi_mode(false);                   // HDMI mode (data islands) — required for compose
    video_output_set_compose_ring(g_compose_ring, sizeof(g_compose_ring) / sizeof(g_compose_ring[0]));
    video_output_set_native_pixel_mode(true);
    video_output_set_scanline_pointer_callback(scanline_ptr_cb);
    video_output_set_background_task(video_output_compose_service);
    Serial.println("ok"); Serial.flush(); delay(20);

    g_video_ready = true;   // release core 1

    Serial.printf("clk_sys=%lu clk_hstx=%lu MHz. Entering emulation loop.\n",
                  (unsigned long)(clock_get_hz(clk_sys) / 1000000),
                  (unsigned long)(clock_get_hz(clk_hstx) / 1000000));
    Serial.flush(); delay(20);
}

// Core 1: video engine.
void setup1() {
    while (!g_video_ready) tight_loop_contents();
    video_output_core1_run();   // never returns
}

// Core 0: emulate one field, compose, pace to ~60 Hz (resync-not-debt).
void loop() {
    static uint32_t deadline = 0;
    static uint32_t frames = 0, last_report = 0, run_us_acc = 0;

    uint32_t t0 = micros();
    coco_machine_run_cycles(CYCLES_PER_FRAME);
    coco_machine_render_frame();
    blit_frame();
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

        Serial.printf("emu %lu fields, avg run %lu us/field (%lu.%02lux real-time), video_frames=%lu (%lu fps)\n",
                      (unsigned long)frames,
                      (unsigned long)(run_us_acc / (frames - last_report)),
                      (unsigned long)(FRAME_US * (frames - last_report) / run_us_acc),
                      (unsigned long)((uint64_t)FRAME_US * (frames - last_report) * 100 / run_us_acc % 100),
                      (unsigned long)vf, (unsigned long)fps);

        // Desync watchdog: a healthy 60p link advances ~60 fps. If frames race
        // (>90 fps) the HSTX stream has desynced (FIFO underran) — restart it.
        if (last_ms && fps > 90) {
            video_output_force_resync();
            Serial.println("  ! HSTX desync detected -> forced resync");
        }
        last_vf = vf; last_ms = now_ms;
        last_report = frames; run_us_acc = 0;
    }
    delay(1);   // yield so the USB CDC task runs (serial + 1200-baud reset)
}

void loop1() { /* unreached */ }
