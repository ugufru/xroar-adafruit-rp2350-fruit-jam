<!-- SPDX-License-Identifier: GPL-3.0-or-later -->
# Performance log (FRUITJAM-15)

Running record of the port's real-time performance and the changes that moved
it. Convention carried from the 43b port, whose 11% → 99.9%-locked trajectory
was its most useful porting artifact.

## Target

Authentic NTSC Tandy Color Computer: the 6809 runs at ~0.895 MHz, one 60 Hz
field = **14915 CPU cycles per 16762 µs**. "100% real-time" means the emulator
executes those 14915 cycles in ≤ 16762 µs of wall-clock, every field, sustained.

The run loop (`src/coco/coco_main.cpp`) runs a fixed 14915 cycles per field and
then deadline-spins to the 16762 µs boundary (resync-not-debt on overrun,
FRUITJAM-23). So the emulation rate is **capped at real-time**; the headroom is
what's left of the field budget after the work is done.

## The metric

The serial report prints, per 60 fields:

```
emu N fields, avg run <us>/field (<X>x real-time), video_frames=.. (fps), audio_peak=..
```

* `avg run us/field` — wall-clock spent executing one field's 14915 cycles +
  the frame-batched VDG render + framebuffer blit + audio resample. **Lower is
  better.**
* `Xx real-time` = 16762 / avg-run. The multiplier of real-time the emulator
  *could* sustain if unpaced; equivalently `1/X` of the field budget is spare.

Caveat: at the BASIC `OK` prompt the 6809 sits in a tiny keyboard-scan +
cursor-blink loop that fits the XIP cache, so **idle avg-run is a weak, noisy
benchmark (±~3% run-to-run)** and *understates* the SRAM hot-set benefit — that
benefit scales with code diversity (running real programs thrashes the cache).

## Architecture (the levers that matter, per porting-rp2350.md)

Applied from day one, before touching the clock:

* **CoCo RAM in SRAM** — the 64 KB `g_ram[]` is a plain SRAM array, never PSRAM
  (COCO-41). PSRAM is unused so far (0 bytes; FRUITJAM-08).
* **Frame-batched VDG render** — per-scanline render is suppressed
  (`SUPPRESS_RENDER_SCANLINE`); the whole field is rendered once via a palette
  LUT (FRUITJAM-24). ~2× vs per-scanline.
* **HSTX scanout is ~0 CPU** — hardware TMDS serializer + DMA on core 1; the
  scanline callback just returns a framebuffer row pointer. This is why this is
  the highest-headroom board of the ports so far.
* **HOT_FUNC hot set in SRAM** — see below.

### SRAM hot set (`.time_critical`, via `HOT_FUNC` / hot.h)

The RP2350 XIP cache is only ~16 KB; the 6809 dispatch alone exceeds it, so in
flash every instruction pays flash-fetch latency (AMOLED-29). Functions on the
per-cycle / per-field-hot path live in SRAM:

| Function | File | Role |
|---|---|---|
| `mc6809_run` (~15 KB) | mc6809.c | 6809 opcode dispatch — the hottest fn (mc680x_ops.c is `#include`d into it) |
| `mc6821_read` / `mc6821_write` | mc6821.c | PIA bus access (keyboard scan every field; dense during SOUND) |
| `coco_mem_cycle` | coco_machine.cpp | SAM-decoded bus cycle (runs every 6809 memory cycle) |
| `coco_vdg_fetch` / `coco_vdg_signal_fs/hs` | coco_machine.cpp | VDG video fetch + HS/FS timing |
| `coco_pia*_postwrite`, `snd_compute` | coco_machine.cpp | PIA sound tap (dense during SOUND, FRUITJAM-13) |
| `coco_machine_render_frame` + `render_*_frame` | coco_machine.cpp | frame-batched renderer (once/field) |

**Do NOT spray HOT_FUNC wider than this core set** — there is a knee
(AMOLED-31). Also measured flat on prior ports, don't chase: hand-devirtualizing
`mem_cycle` without inlining (AMOLED-56), manual portalib stripping (AMOLED-40).

## Operating point

252 MHz sys clock, VREG 1.25 V (FRUITJAM-03). Clock headroom is the *last*
lever, not the first — untouched so far.

## Measurements

Idle (BASIC `OK` prompt), 252 MHz. Idle is noisy — treat as a floor, not the
headline; the hot set's real payoff is under program load.

| Build | idle avg run | headroom | RAM | notes |
|---|---|---|---|---|
| Pre-core-HOT_FUNC (FRUITJAM-13 done) | ~13140 µs | ~1.27× | 70.9% | only coco_machine glue in SRAM; vendored core in flash |
| + `mc6809_run` → SRAM | ~12600 µs | ~1.33× | 73.8% | +14.8 KB SRAM; whole opcode dispatch in RAM |
| + PIA r/w + sound tap → SRAM | ~12700 µs | ~1.29× | 74.0% | +1.2 KB; recovers the SOUND-time cost of the FRUITJAM-13 flash hooks |

Observed under load:
* **SOUND playing** (dense PIA-DAC writes) measured ~14400 µs (1.16×) *before*
  the PIA/sound-tap hot set — still real-time, but the biggest dip seen. The
  hot set targets exactly this path; recovery to be confirmed in normal use.
* Sustained idle runs of 4–5 min hold 59–60 fps with no desync and no drift.

**Status: 100% real-time is locked with ≥ ~16% headroom in the worst case seen
(SOUND) and ~29–33% at idle.** Acceptance met; further gains (wider hot set,
clock) are optional and not currently needed.

## Open perf items

* Precise under-load benchmark (a fixed CPU-bound routine) would give a cleaner
  headline number than the noisy idle figure — not yet built.
* Real-time dip during SOUND: re-measure with the sound-tap hot set to confirm
  the flash-hook cost is recovered.
