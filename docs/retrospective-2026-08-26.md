<!-- SPDX-License-Identifier: GPL-3.0-or-later -->
# Session retrospective — 2026-08-26

What was committed, what was learned, and what is still open. Written to be read
before picking up FRUITJAM-49 / -53 / -55, which are the live threads.

Range: `54fb1e2..50a878b` (13 commits) plus uncommitted work noted at the end.

> **SUPERSEDED IN PART (2026-08-28).** Accurate as history, but several conclusions
> below were overturned by later measurement. Read `docs/hstx-lessons.md` for the
> current state of the desync question. Specifically:
> - **FRUITJAM-49 is fixed** — the resync itself was re-desyncing the link, via a
>   missing RP2350-E5 DMA-abort workaround.
> - **"Behaviour degraded across a long session" did not hold.** A 25-hour run showed
>   no degradation, and the recommended cold-boot test is correspondingly low value.
>   The trajectory was better explained by code layout (FRUITJAM-75): those sessions
>   reflashed constantly, and each build had a different core-0 flash layout, which
>   alone swings the desync rate by three orders of magnitude.
> - **The NeoPixel PIO theory is disproved** (FRUITJAM-53). The observation was real
>   but the mechanism was wrong: the rate tracks strip *illumination*, i.e. current
>   draw, not the state machine. Magnitude was overstated ~15x.
> - **FRUITJAM-55 was closed as superseded** — with E5 fixed, the native path
>   recovers, so there is no failure-mode difference left to buy.
> - **FRUITJAM-38 is closed** — the core-0 timing step is emulation load, not a
>   timer, so `perf-log.md`'s headroom figure is no longer suspect.

---

## 1. What shipped

| Commit | Issue | What |
|---|---|---|
| `aaa5593` | 37 | DVI mode by default — fixes no-picture on the dev monitor |
| `4220d2b` | 20/21 | Closed both; filed 39/40 for the undone remainder |
| `3bf106d` | 41 | CONTRIBUTING: every issue needs a completion condition |
| `01b633c` | 14 | HDMI data-island probe (`[di] stale/silence/lib_resync`) |
| `71f7b7a` | 42 | Original MC6847 font instead of 6847T1 glyphs |
| `1525a48` | 42/43 | Font keyed off the VDG variant, as upstream does |
| `a031ea2` | 44/45/46/48 | NeoPixel boot progress, BASIC-ready detection, faster boot |
| `5e37723` | 44 | LED 2 to chartreuse |
| `8ca9872` | 53 | Gate boot NeoPixels off — they cost HSTX stability |
| `e39d755` | 53/49 | Record stability; flag 49 as likely subsumed |
| `299f67e` | 50/51 | Button + keyboard disk picker (branch, later merged) |
| `a1bb29b` | 50/51 | Merge the picker to main |
| `50a878b` | 53/49 | Desyncs are episodic — the 3-minute bisect was too short |

### Features that are solid

- **DVI mode (`COCO_DVI_MODE`, default 1).** Data islands off. The port sends no
  HDMI audio, so they carried nothing. `=0` restores the HDMI/precomposed path.
- **Original MC6847 font (`COCO_VDG_T1`, default 0).** One flag drives both the
  part variant and the renderer's table, mirroring upstream `mc6847.c:495-509`.
  Required lifting a `library.json` exclusion *and* deleting a zeroed
  `font_6847[768]` stub in `xroar_stubs.c` that existed to satisfy the linker.
- **Disk picker (FRUITJAM-50/51).** Buttons 2/3/1 and F12/arrows/Enter. Overlay
  drawn with the VDG font on a 32x16 grid anchored to the CoCo active area.
  Dotfiles skipped (macOS AppleDouble `._NAME.DSK` doubled an apparent 42 disks
  to a real 21). Mount is a floppy swap, not a reset.
- **BASIC-ready detection (FRUITJAM-46).** Screen-based, ROM-agnostic, ~78 fields.
- **`SERIAL_READY_WAIT_MS`** (default 1000, overridable).
- **`[di]` probe.** Reports pico_hdmi's own counters every second.

### Shipped but disabled

- **Boot NeoPixels (`COCO_NEOPIXEL`, default 0).** See FRUITJAM-53.
- **NeoPixel idle cycle (`NEO_IDLE_CYCLE`, default 0).** See FRUITJAM-47.

---

## 2. The unresolved problem

**HSTX desyncs.** The link enters a state where `video_frame_count` advances at
159-161 fps instead of 60, the core-0 watchdog detects it and requests a resync,
and core 1 performs one. Sometimes it recovers; sometimes it does not.

What is established:

- **160 fps is 2.67x of 60 — the half-length-line ratio.** The same signature
  appeared during FRUITJAM-37 when `video_output_init()` was called at 320 rather
  than the 640 output width. Something is halving the line.
- **Desyncs arrive in EPISODES, not at a steady rate.** One measured episode ran
  19 consecutive resyncs, one per second, then went quiet. Episode onset has been
  seen anywhere from 6 seconds to 4 minutes after boot.
- **Our instrumentation cannot see the corruption.** During a black screen the
  CPU was idling healthily in BASIC (`PC=$A7D3`), screen RAM held the full banner,
  and `fb_nonblack=768/768` — the framebuffer was completely painted. pico_hdmi's
  own counters read `stale=0 lib_resync=0`. A clean log proves the RP2350 is
  transmitting; it never proves the sink is displaying.
- **The two scanout paths fail differently.** `native_pixel_mode` was observed to
  latch permanently (96+ resyncs, never recovering). The classic copy path was
  observed to recover. Mechanism consistent with this: native swaps the scanout
  DMA channel between 16- and 32-bit control every line
  (`video_output_rt.c:1116`), so a resync can re-enter the broken state; classic
  never changes width. **Not proven**, and a later boot showed the classic path
  behaving badly too.
- **Behaviour degraded across the session.** Same configurations: 1 desync in 10
  minutes early on; first episode ~4 minutes in mid-session; 4 desyncs in 15
  seconds late on. No code change explains that trajectory. The board had been
  running for hours at 252 MHz with vreg at 1.25 V. **A cold-boot test was never
  run and is the highest-value next step.**

### Mechanisms proposed and their status

| Hypothesis | Status |
|---|---|
| Monitor rejects HDMI data islands | Partly right: DVI mode fixed the original no-picture, but the stronger claim ("this monitor can't do data islands") was wrong — the user's monitor handles them elsewhere. Instrumented proof the stream was well-formed. |
| Runtime PSRAM/QMI traffic costs sink lock | **Disproved** by its own isolation test (5 s PSRAM-on / 5 s off; no correlation). FRUITJAM-52 closed. |
| NeoPixel `begin()` holding a PIO SM | **Not established.** Bisect showed 0/0/0/15 across builds, but every capture was 2.5-3 min against a fault whose episodes can start at 4 min. Gated off pending real measurement. |
| HSTX DMA needs `HIGH_PRIORITY` | Tried, no benefit observed, reverted. |
| `native_pixel_mode` width swap causes the latch | Consistent with the 2.67x ratio and the differing failure modes; not proven. |

---

## 3. Measurement lessons

These cost the most time and are the most reusable part of this session.

1. **A clean serial log means "transmitting", never "displaying."** Multiple
   times the firmware, the library counters and the framebuffer all read perfect
   while the screen was black.
2. **Short captures cannot see episodic faults.** A 15-25 s capture cannot see a
   once-a-minute burst; a 2.5-3 min capture cannot see an episode that starts at
   4 min. Several conclusions in this session were drawn from windows too short
   to support them, and had to be withdrawn.
3. **Use `scripts/serial_logger.py`.** Wall-clock timestamps, survives reboots.
   Compare **episodes per hour** across builds, not desyncs per capture.
4. **The resync counter is the discriminator.** If it increments, the RP2350 lost
   the link. If a dropout happens with the counter unchanged, the fault is
   downstream — cable, connector, sink re-acquiring.
5. **Boot-latched faults need many boot cycles, not one long run.** At ~1-in-5,
   a single clean run proves nothing.
6. **Check before filing, and check before concluding.** FRUITJAM-43 was filed as
   an open question that one grep answered. The `pico_hdmi_set_audio_sample_rate`
   experiment would have been a no-op — the library already defaults to 48 kHz —
   and reading the source first is what caught it.

---

## 4. Corrections worth carrying forward

- **The classic scanout path does not cost headroom.** An earlier claim that it
  dropped core 0 to 1.04x was wrong; that was FRUITJAM-38's step, which happens
  on any build. Measured: 13.10 ms classic vs 13.19 ms native, both 1.27x.
- **`perf-log.md`'s "~23% headroom" is suspect.** Core 0 steps from ~13.2 ms to
  ~16.0 ms a couple of minutes after boot (FRUITJAM-38). If the documented figure
  came from the cold window, everything reasoning from it — notably FRUITJAM-14's
  conclusion that HSTX audio fits on core 0 — needs rechecking.
- **`#4B0082` reads magenta on WS2812.** Pick LED colours by eye on hardware.
  Final palette: `(255,0,0) (128,255,0) (0,255,0) (0,0,255) (40,0,200)`.
- **Boot phase and run phase have different rules.** Driving the NeoPixels before
  `multicore_launch_core1()` is safe; the same call at runtime desyncs video. But
  note `begin()` claims a PIO SM for the whole run even when the writes are all
  at boot — that reasoning error is recorded on FRUITJAM-47.

---

## 5. Open threads, in priority order

1. **FRUITJAM-49** (high) — HSTX desync root cause. Start with the cold-boot test,
   then a many-boot harness using the timestamped logger.
2. **FRUITJAM-53** (high) — NeoPixel PIO claim. Cheapest first check: *which* PIO
   gets claimed, and whether it collides with PIO-USB. GPIO32 needs a PIO
   `gpio_base` of 16; PIO-USB on GPIO1/2 needs 0; `gpio_base` is per-PIO-block and
   unclaiming does not restore it.
3. **FRUITJAM-55** (high, uncommitted) — ship the classic scanout path so desyncs
   recover instead of latching. Written, then held back when a later boot
   contradicted the evidence.
4. **FRUITJAM-38** (medium) — core-0 timing step; blocks trusting `perf-log.md`.
5. **FRUITJAM-40** (medium) — doc debt: `COCO_DVI_MODE`, `COCO_VDG_T1`,
   `COCO_NEOPIXEL`, the picker, and the boot-timing profile.
6. **FRUITJAM-36** (medium) — audit prior ports for the unguarded `Serial.print`.
7. **FRUITJAM-14** (medium) — HDMI audio. Interacts with FRUITJAM-55: it needs the
   precomposed path.

## 6. Uncommitted at session end

- The FRUITJAM-55 classic-path switch in `src/coco/coco_main.cpp`, plus the
  FRUITJAM-55 issue entry. Deliberately not committed.
- `main` is at `50a878b`: native path, picker included, NeoPixels gated off.
