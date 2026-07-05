// SPDX-License-Identifier: GPL-3.0-or-later
// 320x240 RGB565 framebuffer + a bring-up test pattern.
// FRUITJAM-04. Header-only, driver-agnostic: the HSTX scanout (whichever
// option we choose) reads this buffer and doubles it to 640x480.
//
// RGB565 layout: RRRRRGGG GGGBBBBB (bit 15..0). Whether the HSTX path wants the
// two bytes swapped is a driver-side concern (cf. amoled Rule 7 byte-swap
// gotcha) — this module stores native little-endian uint16 and leaves any swap
// to the scanout.
#pragma once
#include <stdint.h>
#include "dvi_timing.h"

namespace dvi {

static inline constexpr uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
    return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

// One screen's worth of pixels. 320*240*2 = 150 KB — SRAM only (Rule 3).
struct Framebuffer {
    uint16_t px[FB_HEIGHT][FB_WIDTH];

    inline void fill(uint16_t c) {
        for (uint16_t y = 0; y < FB_HEIGHT; y++)
            for (uint16_t x = 0; x < FB_WIDTH; x++)
                px[y][x] = c;
    }
};

// SMPTE-style vertical color bars + a 1px white border. A miscabled lane or a
// byte-swap bug shows up immediately as wrong bar colors; a timing error shows
// as no-lock / rolling. Deterministic (no per-call state) so it's host-testable.
static inline void test_pattern(Framebuffer &fb) {
    // 8 bars, brightest→black left to right (classic bar order).
    static const uint16_t bars[8] = {
        rgb565(255, 255, 255), // white
        rgb565(255, 255,   0), // yellow
        rgb565(  0, 255, 255), // cyan
        rgb565(  0, 255,   0), // green
        rgb565(255,   0, 255), // magenta
        rgb565(255,   0,   0), // red
        rgb565(  0,   0, 255), // blue
        rgb565(  0,   0,   0), // black
    };
    for (uint16_t y = 0; y < FB_HEIGHT; y++) {
        for (uint16_t x = 0; x < FB_WIDTH; x++) {
            fb.px[y][x] = bars[(x * 8) / FB_WIDTH];
        }
    }
    // Bottom strip: a horizontal luminance ramp so gamma/level issues are visible.
    for (uint16_t y = FB_HEIGHT - 24; y < FB_HEIGHT; y++) {
        for (uint16_t x = 0; x < FB_WIDTH; x++) {
            uint8_t v = (uint8_t)((x * 255) / (FB_WIDTH - 1));
            fb.px[y][x] = rgb565(v, v, v);
        }
    }
    // 1px white border to confirm the full active area reaches the panel edges.
    for (uint16_t x = 0; x < FB_WIDTH; x++) {
        fb.px[0][x] = 0xFFFF;
        fb.px[FB_HEIGHT - 1][x] = 0xFFFF;
    }
    for (uint16_t y = 0; y < FB_HEIGHT; y++) {
        fb.px[y][0] = 0xFFFF;
        fb.px[y][FB_WIDTH - 1] = 0xFFFF;
    }
}

} // namespace dvi
