// SPDX-License-Identifier: GPL-3.0-or-later
// xroar-adafruit-rp2350-fruit-jam — HSTX DVI test pattern (FRUITJAM-04).
//
// First light on the DVI port: a 640x480p60 signal carrying our SMPTE color-bar
// test pattern, generated from a 320x240 RGB565 framebuffer scanned out 2x.
//
// Video path: vendored pico_hdmi (v0.0.7, see PROVENANCE.md) drives the RP2350
// HSTX hardware TMDS serializer + double-buffered DMA on core 1 — near-zero CPU.
// We supply pixels through its scanline callback, doing both the vertical 2x
// (active_line >> 1) and horizontal 2x (word duplication) there.
//
// Clock: we set clk_sys = 252 MHz + vreg 1.25 V (FRUITJAM-03); the library is
// built with MODE_HSTX_CLK_DIV=2 so it derives clk_hstx = 126 MHz.
//
// Core split (previews FRUITJAM-11): core 0 sets everything up and idles; core 1
// runs the video engine. setup1() waits for core 0's init before starting it.

#include <Arduino.h>
#include "pico/multicore.h"
#include "hardware/vreg.h"
#include "hardware/clocks.h"
#include "pico/stdlib.h"

extern "C" {
#include "pico_hdmi/video_output.h"
}

#include "framebuffer.h"

// Keep the scanline callback out of flash: it runs from the core-1 DMA IRQ, so
// an XIP stall there would starve the HSTX FIFO. (__not_in_flash_func no-ops
// under the core's PICO_NO_HARDWARE, so use an explicit RAM section — FRUITJAM-02.)
#define RAM_FUNC __attribute__((section(".time_critical.display_test")))

static dvi::Framebuffer   g_fb;                 // 320x240 RGB565, SRAM (150 KB)
static volatile bool      g_video_ready = false;

// Fill one 640-px active line: 320 words, each holding the source pixel twice
// (horizontal 2x). Vertical 2x is active_line >> 1 into the 320x240 buffer.
static void RAM_FUNC scanline_cb(uint32_t v_scanline, uint32_t active_line, uint32_t *dst) {
    (void)v_scanline;
    const uint16_t *row = g_fb.px[active_line >> 1];
    for (uint32_t i = 0; i < dvi::FB_WIDTH; i++) {
        uint32_t px = row[i];
        dst[i] = px | (px << 16);
    }
}

void setup() {
    // vreg must lead the clock (FRUITJAM-03).
    vreg_set_voltage(VREG_VOLTAGE_1_25);
    delay(2);
    set_sys_clock_khz(252000, true);

    Serial.begin(115200);
    const uint32_t start = millis();
    while (!Serial && (millis() - start) < 1500) delay(10);

    pinMode(29, OUTPUT);           // red LED (active-low), heartbeat
    digitalWrite(29, HIGH);

    dvi::test_pattern(g_fb);

    // 640x480p60 default mode; DVI (no HDMI data islands) for widest sink support.
    video_output_init(dvi::H_ACTIVE, dvi::V_ACTIVE);   // (640, 480)
    video_output_set_dvi_mode(true);
    video_output_set_scanline_callback(scanline_cb);

    g_video_ready = true;          // release core 1

    Serial.println();
    Serial.println("=== FRUITJAM-04 HSTX DVI test pattern ===");
    Serial.print("clk_sys=");   Serial.print(clock_get_hz(clk_sys) / 1000000);  Serial.print(" MHz  ");
    Serial.print("clk_hstx=");  Serial.print(clock_get_hz(clk_hstx) / 1000000); Serial.println(" MHz");
    Serial.println("640x480p60, 320x240 RGB565 doubled. Look for SMPTE bars on the monitor.");
}

// Core 1: the video engine. Wait for core 0's init, then run forever.
void setup1() {
    while (!g_video_ready) tight_loop_contents();
    video_output_core1_run();      // does not return
}

void loop() {
    static uint32_t last = 0;
    digitalWrite(29, LOW);  delay(50);   // blink = alive
    digitalWrite(29, HIGH); delay(950);
    Serial.print("frame_count=");
    Serial.println(video_frame_count);
    (void)last;
}

void loop1() { /* unreached: setup1() never returns */ }
