# Fruit Jam — clock plan (252 MHz)

Resolves **FRUITJAM-03**. Validates that a **252 MHz system clock** serves HSTX 480p60,
PIO-USB, SD, and leaves emulation headroom — and pins down exactly how `clk_hstx` is
sourced and divided, which is the one thing the HSTX board changes versus the prior
PIO/libdvi ports.

## The headline

| Clock | Value | How | Consumer |
|---|---|---|---|
| `clk_sys` | **252 MHz** | `set_sys_clock_khz(252000, true)` → PLL_SYS | 6809 emulation, cores, DMA |
| `clk_hstx` | **126 MHz** | AUXSRC = `clk_sys`, `CLK_HSTX_DIV_INT = 2` | HSTX DVI serializer (FRUITJAM-04) |
| PIO-USB SM | 48 MHz eff. | PIO frac divider `252 / 48 = 5.25` (exact in 16.8) | USB host (FRUITJAM-05) |
| `clk_usb` | 48 MHz | PLL_USB ÷ 1 | (device CDC; PIO-USB does not use it for timing) |
| `clk_peri` | **48 MHz** (from `pll_usb`, SDK default) | SPI/UART divide down (SD ~12.5 MHz) | SD (FRUITJAM-06), serial |
| vreg | **1.25 V** | `vreg_set_voltage(VREG_VOLTAGE_1_25)` | headroom at 252 (pizero-proven; see below) |

**Rule (carried from prior ports):** set the clock with `set_sys_clock_khz()`, never by
hand-editing `PLL_SYS_KHZ` — that path skips the QSPI clock-divider recompute and browns
out flash access (amoled lesson).

## Why 252, and why it makes everything line up

Two hard constraints have to share one PLL-derived system clock:

1. **HSTX DVI needs a 25.2 MHz pixel clock** for standard 640×480p60 (800×525 total →
   25.2 MHz × ... = 60.0 Hz). Each pixel is a 10-bit TMDS symbol per lane, so the serial
   line rate is **252 Mbit/s per lane**.
2. **PIO-USB needs an effective 48 MHz** full-speed timing base, derived by the PIO state
   machine's fractional clock divider from `clk_sys`.

252 MHz satisfies both *exactly*:
- **USB:** `252 / 48 = 5.25`. The PIO SM divider is 16.8 fractional, and 0.25 is exactly
  representable, so there is no timing jitter. (The old "PIO-USB only runs at 120/240 MHz"
  belief was a myth — the dividers come off `clk_sys`; 252 MHz was hardware-confirmed on
  the pizero port at commit 457f0ad, PIZERO-44.)
- **HSTX:** the serializer emits **2 bits per `clk_hstx` cycle**, so 252 Mbit/s needs
  `clk_hstx = 126 MHz`. The `clk_hstx` divider is a **2-bit integer divider (÷1/2/3 only,
  no fractional part)** — but `252 / 2 = 126` is an exact integer divide, so `clk_hstx`
  sources straight from `clk_sys` with `DIV_INT = 2`. No second PLL needed.

This is the key difference from the pizero: **there**, libdvi bit-banged TMDS in PIO at
`clk_sys`, so `clk_sys` *was* the 252 Mbit/s bit clock. **Here**, HSTX is a hardware
serializer with its own `clk_hstx = clk_sys / 2 = 126 MHz`, and `clk_sys` is free to be
whatever emulation wants — we simply keep it at 252 so the ÷2 stays integer and the USB
÷5.25 stays exact.

### `clk_hstx` sourcing options (RP2350, from the SDK)

`CLOCKS_CLK_HSTX_CTRL_AUXSRC` can select: `clk_sys` (0), `pll_sys` (1), `pll_usb` (2),
`gpin0` (3), `gpin1` (4). We use **`clk_sys` ÷ 2**. Sourcing from `pll_sys` directly is
possible but pointless here (clk_sys already is pll_sys ÷ 1); `pll_usb` (48 MHz base)
can't reach 126 MHz with the ÷1/2/3 divider, so it's out.

## vreg

The pizero needed **1.25 V** to be solid at 252 MHz (PIZERO-45; stock 1.10 V is spec'd
only to 150 MHz). We start there. Whether *this* Fruit Jam silicon is stable at a lower
1.20 V (or needs 1.25) is board-specific and is measured empirically below. Lowering vreg
is a late optimization, not a bring-up need.

## Headroom

The pizero sustains locked real-time CoCo (~0.895 MHz 6809) with slack at 252 MHz even
while spending CPU on libdvi TMDS + HDMI audio islands. The Fruit Jam removes the single
biggest core-1 cost the pizero carried — **HSTX scanout is a hardware serializer + DMA,
~0 CPU** — so at the same 252 MHz this board should have *more* headroom than the pizero,
not less. Concrete headroom numbers get logged in `docs/perf-log.md` from first boot
(FRUITJAM-15); this issue only has to establish that 252 MHz is the right, achievable
target — which it is.

## Empirical validation on this board

Diagnostic build `env:clockprobe` (`src/clock_probe.cpp`) sets 252 MHz + vreg 1.25 V,
muxes `clk_hstx` to `clk_sys / 2`, measures every clock with the RP2350 frequency counter
(`frequency_count_khz`), and runs a CRC compute-stress loop to confirm the core executes
correctly at speed. Flash with:

```
pio run -e clockprobe -t upload   # or ./deploy.sh after setting default_envs
```

### Measured results — real Fruit Jam silicon, 2026-07-02

Captured from `env:clockprobe` serial (`frequency_count_khz`), vreg 1.25 V:

| Clock | Expected | Measured | Verdict |
|---|---|---|---|
| `clk_sys` | 252000 kHz | **252001 kHz** | ✅ |
| `clk_hstx` | 126000 kHz | **126000 kHz** | ✅ exact — `clk_sys ÷ 2` gives HSTX its 126 MHz |
| `clk_usb` | 48000 kHz | **48000 kHz** | ✅ |
| `clk_peri` | 48000 kHz | **48000 kHz** | ✅ SDK default (pll_usb); ample for SD ~12.5 MHz |
| `pll_sys` | 252000 kHz | **252001 kHz** | ✅ |
| `pll_usb` | 48000 kHz | **48000 kHz** | ✅ |
| `set_sys_clock_khz(252000)` | true | **true** | ✅ achieved |
| CRC compute-stress | stable checksum | **0x32D337AD, stable across all iters** | ✅ core executes correctly at speed |
| vreg | 1.25 V | 1.25 V | ✅ stable |

**Conclusion:** the 252 MHz plan is validated on hardware. `clk_hstx` lands at exactly
126 MHz off `clk_sys ÷ 2`, PIO-USB's `252/48 = 5.25` divider is exact, and the core runs a
stress load with a bit-stable result at 252 MHz / 1.25 V. The 1.25 V rail is confirmed
good; probing whether a lower rail also holds is deferred (late optimization, not needed
for bring-up). Full multi-subsystem coexistence stability (HSTX + USB + SD + emulation all
live) can only be re-confirmed as those subsystems land — but the clock foundation they
share is proven.
