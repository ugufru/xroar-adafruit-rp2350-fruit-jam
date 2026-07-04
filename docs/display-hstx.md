# Fruit Jam — HSTX DVI display (FRUITJAM-04)

Bring up the DVI-D output driven by the RP2350's HSTX peripheral: a stable
**640×480p60** signal on a monitor, fed from a **320×240 RGB565** internal
framebuffer scanned out at **2×** (each source pixel → a 2×2 block).

This is the first port board where video is a **hardware serializer** (HSTX +
DMA), not PIO bit-bang or a pushed SPI/QSPI panel — so scanout is ~0 CPU, which
is why the porting matrix predicts the *static-framebuffer + pointer-swap on
core 1* consumer model here (`porting-rp2350.md` Rule 4 / Step 2).

Clock foundation is already validated (FRUITJAM-03): `clk_sys = 252 MHz`,
`clk_hstx = clk_sys ÷ 2 = 126 MHz` (measured exact on this board), HSTX emits
2 bits/`clk_hstx` cycle → 252 Mbit/s TMDS/lane → 25.2 MHz pixel.

## Pins (from docs/hardware-pinout.md, matches the earlephilhower variant)

| Lane | N pin | P pin |
|---|---|---|
| Clock | GPIO12 (`PIN_CKN`) | GPIO13 (`PIN_CKP`) |
| D0 (blue) | GPIO14 (`PIN_D0N`) | GPIO15 (`PIN_D0P`) |
| D1 (green) | GPIO16 (`PIN_D1N`) | GPIO17 (`PIN_D1P`) |
| D2 (red) | GPIO18 (`PIN_D2N`) | GPIO19 (`PIN_D2P`) |

## 640×480p60 timing (VESA DMT / CEA, standard VGA polarity)

| Axis | Active | Front porch | Sync | Back porch | Total |
|---|---|---|---|---|---|
| Horizontal (px) | 640 | 16 | 96 | 48 | **800** |
| Vertical (lines) | 480 | 10 | 2 | 33 | **525** |

Pixel clock 25.2 MHz ÷ (800 × 525) = **60.0 Hz exactly**. HSync and VSync are
both **active-low** (negative), the standard 640×480 VGA polarity. (25.2 vs the
nominal 25.175 MHz is +0.1% — within every DVI sink's tolerance; it's what buys
the exact PIO-USB divider, see FRUITJAM-03.)

## Framebuffer + 2× scale

- Internal buffer: **320×240 RGB565** = 320 × 240 × 2 = **153,600 B (150 KB)**.
- Scanout doubles both axes: each source line is emitted twice; each source
  pixel is emitted twice horizontally → 640×480 on the wire.
- CoCo geometry (later, FRUITJAM-10): the VDG's 256×192 renders centered in the
  320×240 buffer → 512×384 on screen with a border — the pizero-proven layout.

### SRAM budget note (decision seed for FRUITJAM-11)

520 KB SRAM total. Options for the *emulator* (not the bring-up pattern):
- **Double-buffer + pointer swap** (pizero model, tear-free): 2 × 150 KB = 300 KB.
  Leaves ~220 KB for CoCo RAM (64 KB) + hot code + stacks. Fits, but tight.
- **Single-buffer + dirty flag** (amoled model): 150 KB, but risks tearing on a
  self-clocked scanout unless writes are vsync-fenced.

PSRAM is **not** an option for the framebuffer — Rule 3 (poison in the per-frame
hot path). Decide the buffering model when core 1 split lands (FRUITJAM-11); the
bring-up pattern below needs only a single static buffer.

## Driver evaluation (the FRUITJAM-04 "evaluate A vs B" ask)

The earlephilhower core bundles **no** DVI/HSTX library (confirmed: its
`libraries/` has I2S, SD, TinyUSB-host, HID, PWMAudio — nothing for video). So
HSTX DVI is code we vendor or write. Three candidates:

| Option | What | Pros | Cons |
|---|---|---|---|
| **A. Vendor `fliperama86/pico_hdmi`** | HSTX-native HDMI lib (Unlicense, v0.0.7) | Scanline-callback API matches our VDG consumer model; double-buffered DMA; **TERC4 audio data-islands built in → makes FRUITJAM-14 nearly free**; the prior art Rule 9 names | External dep to vendor + PROVENANCE/THIRD_PARTY_LICENSES tracking; written for bare Pico SDK, may need adaptation under the Arduino build |
| **B. Adapt `pico-examples` `dvi_out_hstx_encoder`** | Minimal HSTX DVI (BSD) | Small, well-understood, ~one file; video-only is all bring-up needs | We own the DMA/TMDS command-list plumbing; no audio-island path (FRUITJAM-14 starts from zero) |
| **C. Write our own from HSTX registers** | Bespoke driver | Zero external deps; no license/provenance overhead; tailored to exactly 320×240→640×480 | Most from-scratch register work (TMDS encode config, DMA command chain, sync regions); highest risk to get right without a monitor to debug against |

**Recommendation: A (vendor pico_hdmi)** — its scanline-callback shape is the
right long-term fit for the indexed-frame → consumer handoff (Rule 4), and it
folds FRUITJAM-14 (HDMI audio, the second audio sink) in for almost free. **B**
is the low-commitment fallback if pico_hdmi fights the earlephilhower build.

**Status: awaiting maintainer decision.** Vendoring an external dependency is a
licensing/provenance choice the project owns, so the driver itself is not yet
committed. The driver-agnostic pieces (timing constants, framebuffer, test
pattern) are landed in `src/display/` so whichever option is chosen only has to
supply the scanout.

_Deferred alternative:_ if A is chosen and later proves heavy, revisit B/C.
Revisit trigger for reopening the audio-island question: **FRUITJAM-14**.

## Acceptance

Stable, correctly-colored test pattern on a real monitor over the DVI port,
confirmed at true 60 Hz (monitor's OSD reporting 640×480 @ 60 Hz). Requires the
maintainer's monitor — not verifiable from the build host.
