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
