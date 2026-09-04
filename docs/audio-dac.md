# Audio: TLV320DAC3100

Reference notes for the board's audio codec, written against the TI datasheet
(**SLAS671C**, Feb 2010 / rev. Jan 2017; the local `tlv320dac3100.pdf`, 111 pp.) and checked against
what this port actually programs. Companion to `docs/hardware-pinout.md` (pins) and the
audio section of `src/coco/coco_main.cpp` (the code).

Register citations are `page / register (hex)` in the datasheet's own notation, with the
datasheet table number where it helps. Page/register numbers are **decimal**; the hex in
brackets is the register address as written on the wire.

## The part

Low-power **stereo audio DAC** (playback only, no ADC) with an integrated stereo
headphone/line driver and a **mono class-D speaker amp** (2.5 W into 4 Ω). 24-bit,
8 kHz–192 kHz, 95 dB SNR, 32-VQFN. Controlled entirely over I²C; audio arrives over a
separate I²S/PCM bus.

Beyond the plain DAC it carries a fractional PLL, 25 selectable DSP processing blocks
(biquads, FIR, DRC, 3-D), a digital sine-wave beep generator, two analog inputs with a
mixer, MICBIAS, and impedance-based headset detection. **This port uses almost none of
that**. See [What we deliberately don't use](#what-we-deliberately-dont-use).

### On the Fruit Jam

| | |
|---|---|
| I²C address | **0x18** (7-bit `0011000`; fixed, no strap) on I²C0 = SDA GPIO20 / SCL GPIO21 |
| I²S | DIN GPIO24, BCLK GPIO26, WCLK/LRCLK GPIO27, MCLK GPIO25 |
| Reset | GPIO22, **shared with the ESP32-C6.** Toggling it resets both. |
| IRQ | GPIO23 (shared name with the ESP path); unused by this port |
| Outputs | 3.5 mm headphone jack (HPL/HPR) and a JST-SH mono speaker port (class-D) |

**MCLK is wired but unused.** The DAC's PLL derives everything from BCLK, so GPIO25 is
free. The RP2350 is the I²S **master**; the codec is the slave (register 27's BCLK/WCLK
direction bits are left at their reset value, "input").

## Signal chain

```
coco_machine (PIA sound tap, 48 kHz mono PCM)
   └─ coco_main audio_feed(), once per field
        └─ earlephilhower PIO-I2S master  ── BCLK 1.536 MHz, WCLK 48 kHz, 16-bit L=R
             └─ TLV320 digital: PLL → NDAC → MDAC → DOSR → DAC modulator
                  └─ DAC_L/DAC_R → output mixer amp        (p1/r35)
                       └─ analog volume control, 0 → −78 dB (p1/r36, r37, r38)
                            ├─ HPL/HPR driver PGA 0–9 dB   (p1/r40, r41) → 3.5 mm jack
                            └─ class-D PGA 6/12/18/24 dB   (p1/r42)      → speaker port
```

There are **four** gain stages in series and it is easy to lose track of which one you are
turning: digital DAC volume (p0/r65–66, −63.5…+24 dB), analog volume (p1/r36–38, 0…−78 dB,
non-linear), the driver PGA, and, upstream of all of it, `AUDIO_GAIN` in
`coco_machine.cpp`. See [Levels as configured](#levels-as-configured).

## Clocking: how 48 kHz is reached

The datasheet's clock tree (§6.3.11, Fig. 6-19):

```
PLL_CLK   = PLL_CLKIN × R × J.D / P
CODEC_CLKIN = PLL_CLK                       (p0/r4 D1–D0 = 11)
DAC_CLK     = CODEC_CLKIN / NDAC
DAC_MOD_CLK = CODEC_CLKIN / (NDAC × MDAC)
DAC_fS      = CODEC_CLKIN / (NDAC × MDAC × DOSR)
```

This port feeds the PLL from **BCLK** (p0/r4 D3–D2 = 01), which the RP2350 generates as
`48000 × 16 bits × 2 channels = 1.536 MHz`. With **P=1, R=2, J=32, D=0, NDAC=8, MDAC=2** and
DOSR left at its reset value of 128:

| Quantity | Value | Datasheet limit | |
|---|---|---|---|
| `PLL_CLKIN / P` | 1.536 MHz | 512 kHz – 20 MHz (D = 0) | ✅ |
| `PLL_CLK` | 98.304 MHz | 80 – 110 MHz | ✅ |
| `R × J` | 64 | 4 – 259 | ✅ |
| `CODEC_CLKIN` | 98.304 MHz | ≤ 110 MHz | ✅ |
| `DAC_CLK` | 12.288 MHz | ≤ 49.152 MHz | ✅ |
| `DAC_MOD_CLK` | 6.144 MHz | ≤ 6.758 MHz | ✅ |
| `DAC_fS` | **48000.0 Hz exactly** | ≤ 192 kHz | ✅ |

Two things worth knowing about that table:

- **It is exact, not approximate.** No fractional D term is needed, which keeps the PLL in
  its relaxed `D = 0` regime (PLL_CLKIN as low as 512 kHz is legal; with `D ≠ 0` the floor
  jumps to 10 MHz and R is forced to 1).
- **Table 6-28 does not list this case.** TI's worked PLL examples start at PLL_CLKIN =
  2.048 MHz. A 1.536 MHz BCLK reference (the natural one for 16-bit stereo at 48 kHz) is
  ours, derived rather than copied. It satisfies every stated constraint above.

DOSR = 128 also has to be an integral multiple of the processing block's interpolation
ratio (8 for a Filter-A block). The reset default is PRB_P1, a Filter-A block, so 128 is
legal; **changing the processing block can invalidate DOSR.**

## Bring-up sequence

`audio_init()` / `configure_codec()` in `src/coco/coco_main.cpp`, mirroring the FRUITJAM-07
bring-up in `src/audio/audio_test.cpp`. Order matters at three points, all of them
datasheet requirements:

1. **Pulse the shared reset** (GPIO22 low 100 ms, high 100 ms). RESET must be low ≥ 10 ns
   with IOVDD and DVDD up (§6.3.2). *This also resets the ESP32-C6*, acceptable here only
   because the ESP is out of scope for the port.
2. **Wait ≥ 1 ms after RESET rises** before any register access (§6.3.3, start-up lockout).
   No coefficient read/write and no block power-up during that window. The 100 ms delay
   plus `delay(10)` after `begin()` covers this by a wide margin.
3. **Start the I²S master before configuring the PLL**, so BCLK is already running when the
   PLL is told to lock onto it. The codec cannot lock to a clock that is not there.
4. Configure interface → clock sources → PLL dividers → power the PLL → power the DAC →
   route → volumes → power the drivers. **If CODEC_CLKIN comes from the PLL, the PLL must
   be powered up first and powered down last** (§6.3.11.1).
5. **PLL_CLK is only available ~10 ms after power-up** (§6.3.4). The port never reads back
   or waits on it; the DAC simply stays silent for that window, which is invisible during
   boot.

Shutdown, if it is ever implemented, has its own ordering rule: NDAC and MDAC **must not**
be powered down while the DAC channel is shutting down. Poll the power-status flags at
p0/r37 D7 and D3, then drop MDAC, then NDAC: children before parents (§6.3.11).

## The port's register configuration

Every call the port makes, and what it lands on. The Adafruit library
(`Adafruit_TLV320DAC3100`) handles page selection; each row's page/register is the
datasheet address it writes.

| Call in `configure_codec()` | Page/reg | Effect |
|---|---|---|
| `setCodecInterface(FORMAT_I2S, DATA_LEN_16)` | 0 / 27 (0x1B) | I²S, 16-bit. BCLK/WCLK direction bits untouched ⇒ **codec is slave** |
| `setCodecClockInput(CODEC_CLKIN_PLL)` | 0 / 4 (0x04) D1–D0 = 11 | CODEC_CLKIN = PLL_CLK |
| `setPLLClockInput(PLL_CLKIN_BCLK)` | 0 / 4 D3–D2 = 01 | PLL reference = BCLK pin |
| `setPLLValues(1, 2, 32, 0)` | 0 / 5–8 | P=1, R=2, J=32, D=0 |
| `setNDAC(true, 8)` / `setMDAC(true, 2)` | 0 / 11, 12 | dividers powered up (D7=1) with values 8 and 2 |
| (not called) | 0 / 13–14 | DOSR left at reset = **128** |
| (not called) | 0 / 60 | processing block left at reset = **PRB_P1** (Filter A) |
| `powerPLL(true)` | 0 / 5 D7 | PLL on |
| `setDACDataPath(true, true, NORMAL, NORMAL, STEP_1SAMPLE)` | 0 / 63 (0x3F) | both DAC channels up, L←left data, R←right data, soft-step one step per sample |
| `configureAnalogInputs(ROUTE_MIXER, ROUTE_MIXER, false×4)` | 1 / 35 (0x23) | DAC_L→left mixer amp, DAC_R→right mixer amp; AIN1/AIN2 not routed; HPL not routed to HPR |
| `setDACVolumeControl(false, false, VOL_INDEPENDENT)` | 0 / 64 (0x40) | both channels **unmuted**, independent L/R volume |
| `setChannelVolume(false/true, 18)` | 0 / 65, 66 | digital volume **+18 dB** (reg value 36 = 0x24). See note below |
| `configureHeadphoneDriver(true, true, HP_COMMON_1_35V, false)` | 1 / 31 (0x1F) | HPL+HPR powered, output common mode 1.35 V, current-limit (not power-down) on short |
| `configureHPL_PGA(0, true)` / `configureHPR_PGA(0, true)` | 1 / 40, 41 | driver PGA 0 dB, unmuted |
| `setHPLVolume(true, 6)` / `setHPRVolume(true, 6)` | 1 / 36, 37 | analog volume routed to driver (D7=1), gain index 6 = **−3 dB** |
| `enableSpeaker(SPEAKER_OUTPUT)` | 1 / 32 (0x20) D7 | class-D driver, **off** in the shipped build |
| `configureSPK_PGA(SPK_GAIN_6DB, true)` *(if enabled)* | 1 / 42 (0x2A) | class-D output stage 6 dB, unmuted |
| `setSPKVolume(true, 0)` *(if enabled)* | 1 / 38 (0x26) | analog volume routed to class-D, gain index 0 = **0 dB** |

### Levels as configured

Headphone path, in order:

```
PCM  →  +18 dB digital (p0/r65,66)  →  −3 dB analog (p1/r36,37)  →  0 dB PGA (p1/r40,41)  →  jack
```

**The comment in the source is wrong.** `setChannelVolume(false, 18)` is documented by
Adafruit as taking **dB**, not a register step: it computes `reg = dB × 2` and writes 36
(0x24): digital volume **+18 dB**, not the "+0 dB" the trailing comment claims. That is a
comment defect, not a behaviour bug: the level was tuned by ear and by `audio_peak` on real
hardware with this value in place, so **do not "fix" it by changing the 18**. Fix the
comment. If the tuning is ever revisited, note that +18 dB of digital gain ahead of a
16-bit path is headroom spent, and `AUDIO_GAIN = 32000.0f` in `coco_machine.cpp` is already
sized so note-edge transients land at ~0.82 of full scale.

Analog volume (p1/r36–38, Table 6-24) is **non-linear** and only approximately 0.5 dB per
step over its useful range: index 6 = −3 dB, 30 = −15 dB, 60 = −30.1 dB, 90 = −45.2 dB,
117–127 all = −78.3 dB. With D7 = 0 and D6–D0 = 127, the stage is **muted**. Don't
interpolate this table; read it.

### Speaker vs. headphone

`SPEAKER_OUTPUT` (0 in the shipped build) selects the sink. Both drivers *can* run
concurrently at different levels (the analog volume control is per-driver, §6.3.10.11),
so a build that wants both only has to enable the class-D path as well.

**Headset auto-detect (p0/r67) was tried and abandoned.** The detector is impedance-based;
a high-Z line input on an external amp reads as "nothing plugged in", so auto-mute-on-insert
did not work for the intended use. Revisit only for real low-impedance headphones, and only
after confirming the Fruit Jam wires the detection usefully.

## Gotchas

- **Reserved bits are load-bearing.** The datasheet is explicit: do not read or write
  reserved pages/registers, and write only reset values to reserved bits: "otherwise,
  device functionality failure can occur". Several in-use registers have reserved bits with
  a reset value of **1** (p1/r31 D2, p1/r40–41 D1), so blind `reg = value` writes are unsafe;
  read-modify-write, as the Adafruit library's `RegisterBits` does.
- **Paged register map.** Pages 0, 1, 3, 8–9, 12–13 exist; page 0 is the reset default.
  You switch by writing the page number to register 0 *of the current page*. A driver that
  forgets which page it is on writes garbage to a plausible-looking address.
- **All blocks are powered down after reset.** Silence after a config change usually means
  a power bit was never set, not that the audio path is broken.
- **`D ≠ 0` changes the PLL rules.** A fractional PLL setting forces R = 1 and raises the
  PLL_CLKIN/P floor to 10 MHz. Also, p0/r7 must be written *immediately* before p0/r8,
  because the new D value doesn't take effect until the r8 write completes. Same for DOSR (r13
  then r14).
- **Software reset exists** (p0/r1 D0), which is the escape hatch when you'd rather not
  reset the ESP32-C6 along with the codec via GPIO22.
- **Short-circuit behaviour is configurable, and latching.** After an overcurrent shutdown,
  the output stage is re-enabled by a per-stage power-stage reset (p1/r31 D7/D6 for HPL/HPR,
  p1/r32 D7 for the speaker) without disturbing the rest of the register state.

## What we deliberately don't use

Listed so nobody re-derives them from the datasheet believing they're in play: analog
inputs AIN1/AIN2 and the input mixer; MICBIAS; the beep/key-click generator (p0/r71–79);
DRC (p0/r68–70); the programmable biquad/FIR coefficient pages (8–9, 12–13); alternative
processing blocks PRB_P2–P25; headset detection (p0/r67, see above); the interrupt/INT1–2
machinery and GPIO1 (p0/r48–51); CLKOUT and BCLK-output modes; the VOL/MICDET pin's SAR ADC
volume control (p0/r116–117); TDM/DSP/left- and right-justified interface modes; and the
programmable delay timer on page 3.

## Datasheet map

For when you need to go back to the source.

| Looking for | §, page |
|---|---|
| Pin functions and package | §3, pp. 5–6 |
| Electrical characteristics, power dissipation | §4.5–4.6, pp. 8–10 |
| I²S/LJF/RJF timing, master and slave | §4.7–4.8, pp. 10–11 |
| I²C timing | §4.11, p. 12 |
| Power-supply sequence, reset, lockout, PLL start-up | §6.3.1–6.3.5, pp. 19–20 |
| DAC, analog outputs, routing, volume (incl. Table 6-24) | §6.3.10, pp. 25–50 |
| Clock tree, dividers, PLL and worked examples | §6.3.11, pp. 51–55 |
| Digital audio interface modes | §6.3.13, pp. 56–63 |
| **Register map** (page 0) | §6.4.2.1, pp. 65–80 |
| **Register map** (page 1: DAC, drivers, volumes) | §6.4.2.2, pp. 81–86 |
| Typical application, external components | §7.2, pp. 96–98 |
| Power-supply recommendations / layout | §8–9, pp. 99–100 |

The PDF is a local reference copy; the canonical source is TI's product folder for
**TLV320DAC3100** (literature number SLAS671C).

## See also

- `docs/hardware-pinout.md`: the verified pin map, schematic-derived
- `src/coco/coco_main.cpp`: `audio_init()`, `configure_codec()`, `audio_feed()`
- `src/audio/audio_test.cpp`: the FRUITJAM-07 standalone bring-up (`pio run -e audiotest`)
- `lib/coco_machine/src/coco_machine.cpp`: the producer side: PIA sound tap, per-sample
  integration, DC blocker, `AUDIO_GAIN`
