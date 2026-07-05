// SPDX-License-Identifier: GPL-3.0-or-later
// 640x480p60 DVI timing — VESA DMT / CEA standard VGA polarity.
// FRUITJAM-04. See docs/display-hstx.md for the derivation.
//
// Header-only constants; driver-agnostic. Whichever HSTX scanout path we pick
// (pico_hdmi / pico-examples encoder / bespoke) programs these numbers.
//
// Pixel clock 25.2 MHz / (800 * 525) = 60.0 Hz exactly (FRUITJAM-03).
// HSync and VSync are both ACTIVE-LOW.
#pragma once
#include <stdint.h>

namespace dvi {

// Horizontal (pixels)
static constexpr uint16_t H_ACTIVE      = 640;
static constexpr uint16_t H_FRONT_PORCH = 16;
static constexpr uint16_t H_SYNC        = 96;
static constexpr uint16_t H_BACK_PORCH  = 48;
static constexpr uint16_t H_TOTAL       = H_ACTIVE + H_FRONT_PORCH + H_SYNC + H_BACK_PORCH; // 800

// Vertical (lines)
static constexpr uint16_t V_ACTIVE      = 480;
static constexpr uint16_t V_FRONT_PORCH = 10;
static constexpr uint16_t V_SYNC        = 2;
static constexpr uint16_t V_BACK_PORCH  = 33;
static constexpr uint16_t V_TOTAL       = V_ACTIVE + V_FRONT_PORCH + V_SYNC + V_BACK_PORCH; // 525

static constexpr bool     HSYNC_ACTIVE_LOW = true;
static constexpr bool     VSYNC_ACTIVE_LOW = true;

// Derived (compile-time sanity)
static constexpr uint32_t PIXEL_CLOCK_HZ = 25'200'000u;      // clk_hstx(126M) drives 2 TMDS bits/cycle
static constexpr uint32_t REFRESH_MILLIHZ =
    (uint64_t)PIXEL_CLOCK_HZ * 1000u / ((uint32_t)H_TOTAL * V_TOTAL);   // == 60000 (60.000 Hz)

static_assert(H_TOTAL == 800, "H_TOTAL must be 800 for 640x480p60");
static_assert(V_TOTAL == 525, "V_TOTAL must be 525 for 640x480p60");
static_assert(REFRESH_MILLIHZ == 60000, "timing must yield exactly 60.000 Hz");

// Internal framebuffer, scanned out 2x to fill H_ACTIVE x V_ACTIVE.
static constexpr uint16_t FB_WIDTH  = H_ACTIVE / 2;   // 320
static constexpr uint16_t FB_HEIGHT = V_ACTIVE / 2;   // 240
static constexpr uint8_t  FB_SCALE  = 2;

} // namespace dvi
