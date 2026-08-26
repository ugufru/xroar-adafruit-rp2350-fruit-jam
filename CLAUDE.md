# CLAUDE.md — agent working notes

XRoar (Tandy CoCo) on the **Adafruit Fruit Jam** (RP2350B, HSTX DVI, 8 MB PSRAM).

**All project documentation lives in `README.md` and `docs/`** — hardware pinout, clock plan, the
HSTX display path, the PSRAM policy, dual-core split and perf log are written up there, not here.
Don't duplicate them. This file is pointers and working conventions.

## Read this first

**`docs/retrospective-2026-08-26.md`** — read it before touching the video path or picking up
FRUITJAM-49 / -53 / -55. It records what shipped, five hypotheses proposed for the HSTX desync and
which ones survived contact with evidence, and the measurement mistakes that cost the most time.
Several conclusions that *read* as settled in commit messages are explicitly marked as not
established there. Reading it first will save re-deriving a long session.

The live thread is an **intermittent HSTX desync** (link runs at 159-161 fps instead of 60). Root
cause is open (FRUITJAM-49). The highest-value untried step is a **cold-boot test**: behaviour
degraded across a long session in a way no code change explains, on a board running for hours at
252 MHz with vreg at 1.25 V.

## Measurement discipline (this repo has been burned by all of these)

- **A clean serial log means "transmitting", never "displaying."** The firmware, pico_hdmi's own
  counters and the framebuffer have all read perfect while the screen was black.
- **Short captures cannot see episodic faults.** Desyncs arrive in bursts; episode onset has ranged
  from 6 s to 4 min after boot. A 15 s capture proves nothing; a 3 min capture proves little.
- **Use `scripts/serial_logger.py`** (wall-clock timestamps, survives reboots) and compare
  **episodes per hour** between builds.
- **The resync counter is the discriminator.** It increments → the RP2350 lost the link. A dropout
  with the counter unchanged → the fault is downstream (cable, connector, sink re-acquiring).
- **Boot-latched faults need many boot cycles, not one long run.**
- **Verify before concluding, and before filing.** Reading the source has repeatedly turned a
  planned experiment into a no-op or answered a question before it was asked.

## Build flags worth knowing

| Flag | Default | Meaning |
|---|---|---|
| `COCO_DVI_MODE` | 1 | DVI mode, no data islands. `0` = HDMI/precomposed (needed for FRUITJAM-14). |
| `COCO_VDG_T1` | 0 | Original MC6847 font + part variant. `1` = 6847T1. |
| `COCO_NEOPIXEL` | 0 | Boot progress LEDs. Off — see FRUITJAM-53. |
| `NEO_IDLE_CYCLE` | 0 | Idle colour cycle. Off — desyncs video, FRUITJAM-47. |
| `SERIAL_READY_WAIT_MS` | 1000 | Boot wait for a serial host. `1500` for a guaranteed full boot log. |

## Conventions

- Work is tracked in `issues.jsonl`; see `CONTRIBUTING.md`. Every issue needs a completion
  condition — standing practices are SOP and live in `CONTRIBUTING.md`, not as tickets.
- `./deploy.sh -e coco --capture --secs N` builds, flashes and reads serial. `--read-only` reads
  without reflashing (use it to check state without losing the counters).
- `lib/xroar_core` is a verbatim XRoar 1.11 extraction apart from documented deviations; see
  `PROVENANCE.md` before editing anything under it.
