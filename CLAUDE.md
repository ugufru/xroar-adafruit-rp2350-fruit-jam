# CLAUDE.md — agent working notes

XRoar (Tandy CoCo) on the **Adafruit Fruit Jam** (RP2350B, HSTX DVI, 8 MB PSRAM).

**All project documentation lives in `README.md` and `docs/`** — hardware pinout, clock plan, the
HSTX display path, the PSRAM policy, dual-core split and perf log are written up there, not here.
Don't duplicate them. This file is pointers and working conventions.

## Read this first

**`docs/hstx-lessons.md`** — read before touching the video path. Field notes on driving DVI from
HSTX: the command-list failure mode that latches (160 fps at 60p is the fingerprint), RP2350-E5,
the code-layout sensitivity below, the theories that turned out wrong, and an honest for/against
ledger on HSTX itself.

**`docs/retrospective-2026-08-26.md`** — historical. Useful for how the port got here, but several
of its conclusions have since been overturned by measurement; it is superseded on the desync
question by `hstx-lessons.md`. In particular its "behaviour degrades across a long session" theory
and its cold-boot recommendation did **not** survive: a 25-hour run showed no degradation.

**Desync status.** Two root causes were found and fixed — the resync was re-desyncing the link
(RP2350-E5, FRUITJAM-49) and core-0 code layout was starving the HSTX FIFO (FRUITJAM-75/76).
Standing baseline is **7.6 desyncs/hour over 25 hours**, each a brief flicker that recovers, versus
"black screen until power-cycle" before. **Onset is still unexplained (FRUITJAM-58)** — the leading
lead is that the rate tracks board *current* (a lit LED strip alone is worth ~3.5x), which makes
power-rail integrity a live suspect rather than anything in the firmware.

## Measurement discipline (this repo has been burned by all of these)

- **A clean serial log means "transmitting", never "displaying."** The firmware, pico_hdmi's own
  counters and the framebuffer have all read perfect while the screen was black.
- **Confirm the instrument is in the flashed binary before believing its silence (FRUITJAM-92).**
  One level down from the rule above: a clean log can mean *the probe was never there*. Both traps
  have bitten — `PLATFORMIO_BUILD_FLAGS` set on `pio run` but **not** on `pio run -t upload`, so the
  upload rebuilt at the default and flashed probe-less firmware three times; and instrumentation
  written as an `else if` in a dispatch chain the target device never reached. Both produced zero
  output, which was then misread as hardware behaviour and misattributed twice. Check with
  `strings .pio/build/coco/firmware.elf | grep '<probe string>'` — cheap, and it settles it.
- **Change-only logging cannot measure a steady state (FRUITJAM-92).** A probe that prints only on
  change renders "no drift" and "device unplugged" identical — both are zero lines — and never shows
  the resting value. For at-rest or baseline questions, sample unconditionally on an interval. Make
  min/max **per-interval**, not cumulative: cumulative latched the extremes from a USB
  re-enumeration transient and then read full-scale forever, which looks exactly like a catastrophic
  fault and is not.
- **Localise before you treat (FRUITJAM-97).** Three targeted fixes were tried against the
  disk-mount video dropout — QMI read pacing, a forced resync, DMA channel priority — and all
  three failed, because none of them had established *which subsystem* was at fault. Bisecting
  the mount into its two halves and running them separately (PSRAM writes alone / SD reads
  alone) settled it in one flash cycle: SD reads dropped the link, PSRAM writes never did, and
  the timings showed why — 151 ms of SD against 15 ms of PSRAM, so every earlier fix had been
  aimed at the negligible half. The root cause was the SD clock's GPIO drive strength, an
  **electrical** variable no bus-level fix could ever have reached.
- **A human watching an episodic fault is not an instrument (FRUITJAM-97).** Three arms of
  eyeball A/B/A — 12 mA drops, 4 mA does not, 12 mA drops again — produced a change that
  MEASUREMENT later showed more than DOUBLED the fault rate (5.54 vs 2.50 underruns per SD read,
  ~55 trials per arm). Two same-config runs at n=19 gave 4.57 and 3.10, a 47% spread, so nothing
  at that sample size was ever distinguishable. Reversal protects against luck in one direction;
  it does nothing about a window too short to resolve a rate. Build a counter, then measure.
- **An experiment the observer cannot read is not an experiment.** The same issue again: a
  four-phase bisect printed its phase markers to SERIAL while the fault was only visible on the
  SCREEN, so drops could not be attributed and two readings had to be thrown away. Either put
  the label where the observer is looking, or run ONE arm per build so no attribution is needed.
- **`PICO_HDMI_FIFO_PROBE=1` counts TMDS FIFO underruns directly** (HSTX_FIFO_STAT sampled in
  the scanout ISR). Healthy `min_level` is 6-7; it hits 0 exactly when the picture breaks. Use
  this rather than the desync watchdog for anything involving a starved FIFO — the watchdog
  detects by frame rate and this fault does not change the frame rate.
- **Short captures cannot see episodic faults.** Desyncs arrive in bursts; episode onset has ranged
  from 6 s to 4 min after boot. A 15 s capture proves nothing; a 3 min capture proves little.
- **Use `scripts/serial_logger.py`** (wall-clock timestamps, survives reboots) and compare
  **episodes per hour** between builds.
- **The resync counter is the discriminator.** It increments → the RP2350 lost the link. A dropout
  with the counter unchanged → the fault is downstream (cable, connector, sink re-acquiring).
- **Code LAYOUT changes the desync rate by orders of magnitude (FRUITJAM-75).** Core 0's per-field
  path runs from flash via the XIP cache. Adding ~110 lines of *never-executed* code took desyncs
  from 0/hr to 964/hr and cost +1.5 ms/field; moving `blit_frame()` to `.time_critical` RAM took it
  back to 31/hr. So **adding code is itself a variable** — an A/B of any change is invalid until the
  hot path is layout-immune, and `avg run` is not a clean measure of core-0 work either.
- **The noise floor is large, and desyncs are episodic.** Idle in an *identical* configuration has
  measured 20.6, 21.2 and 31.5/hr across ~20-minute windows, and three events have arrived inside
  23 seconds. Treat anything under an hour per arm as indicative only, and define arms in advance
  and flash them separately — segmenting one log by inferred machine state has produced wrong
  conclusions here more than once.
- **Boot-latched faults need many boot cycles, not one long run.**
- **Verify before concluding, and before filing.** Reading the source has repeatedly turned a
  planned experiment into a no-op or answered a question before it was asked.

## Build flags worth knowing

| Flag | Default | Meaning |
|---|---|---|
| `COCO_DVI_MODE` | 1 | DVI mode, no data islands. `0` = HDMI/precomposed (needed for FRUITJAM-14). |
| `COCO_VDG_T1` | 0 | Original MC6847 font + part variant. `1` = 6847T1. |
| `COCO_NEOPIXEL` | 1 | Boot progress LEDs, left lit after boot. Costs ~15/hr desyncs (FRUITJAM-77). |
| `COCO_DSK_WRITEBACK` | 0 | Persist disk writes to SD. Off pending FRUITJAM-88. |
| `COCO_JOY_PROBE` | 0 | Log raw HID gamepad reports. `1` to identify a new pad (FRUITJAM-18). |
| `COCO_MOUNT_PROBE` | 0 | Log a disk mount's duration and both resync counters (FRUITJAM-97). |
| `COCO_MOUNT_BISECT` | 0 | Run ONE load repeatedly every 5 s to isolate what disturbs the video: 1=PSRAM only, 2=SD→SRAM, 3=SD→PSRAM (a real mount), 4=SD→SRAM→PSRAM (FRUITJAM-97). |
| `PICO_HDMI_FIFO_PROBE` | 0 | Count HSTX FIFO underruns in the scanout ISR. The only instrument that can see a starved-FIFO fault (FRUITJAM-97). |
| `COCO_CROP_BORDER` | 0 | Crop the VDG border, scale 2.5x to fill 640x480 (FRUITJAM-61). |
| `COCO_CROP_SMOOTH` | 2 | With the above: 0 hard, 1 linear, 2 boundary blend. |
| `NEO_IDLE_CYCLE` | 0 | Idle colour cycle. Off — desyncs video, FRUITJAM-47. |
| `SERIAL_READY_WAIT_MS` | 1000 | Boot wait for a serial host. `1500` for a guaranteed full boot log. |

## Conventions

- Work is tracked in `issues.jsonl`; see `CONTRIBUTING.md`. Every issue needs a completion
  condition — standing practices are SOP and live in `CONTRIBUTING.md`, not as tickets.
- `./deploy.sh -e coco --capture --secs N` builds, flashes and reads serial. `--read-only` reads
  without reflashing (use it to check state without losing the counters).
- `lib/xroar_core` is a verbatim XRoar 1.11 extraction apart from documented deviations; see
  `PROVENANCE.md` before editing anything under it.
