<!-- SPDX-License-Identifier: GPL-3.0-or-later -->
# HSTX: what this port learned the hard way

Field notes on driving DVI from the RP2350's HSTX peripheral, written after a long
debugging session that took the desync rate from "black screen until power-cycle" to
single digits per hour. `display-hstx.md` describes how the path *works*; this file
records what surprised us, including the theories that turned out to be wrong.

Read it before changing anything on the video path, and before deciding whether to
use HSTX on a board of your own.

---

## 1. The command-list model fails differently from bit-banged DVI

HSTX is fed a stream of command words by DMA. The expander interprets each word as an
instruction (raw repeat, TMDS run, and so on). This is cheap — scanout costs one
pointer handoff per line and core 1 is otherwise idle — but it has a failure mode that
PIO-based DVI does not:

> **One mis-sized or corrupted command word desynchronises the stream permanently.**

Everything after the bad word is misread. The FIFO stops back-pressuring, so lines
"complete" at bus speed and the frame counter races to roughly 2.67x — **160 fps at
60p**. That ratio is the fingerprint: it is the half-length-line case, and seeing it
means the command stream, not the sink, is broken.

A bit-banged PIO implementation starved for a line produces one bad line. HSTX starved
at the wrong moment produces a link that never recovers on its own. That is the
central trade: less CPU, worse failure mode. Everything else here follows from it.

Because the fault latches, you need an explicit resync path, and that path becomes
safety-critical — see §2.

## 2. RP2350-E5: aborting chained DMA channels re-triggers them

`hstx_resync()` aborts the two scanout channels. They are chained to *each other*,
which is the worst case for **RP2350-E5**:

> An abort does **not** suppress the aborted channel's `CHAIN_TO` trigger. The `EN`
> bit of the aborted channel *and of every channel chained to it* must be cleared
> **before** the abort.

The SDK's `dma_channel_abort()` does not do this — it writes `dma_hw->abort` and spins
on `BUSY`. `hardware/dma.h` documents E5 as the caller's responsibility. Aborting PING
re-triggered PONG and vice versa, leaving a live channel while the code rewrote its
`read_addr`/`transfer_count` and restarted it.

The result: the resync **caused** the 160 fps desync it existed to clear. Watchdog
fires, resync re-desyncs, watchdog fires again a second later — which is why desyncs
appeared as bursts of consecutive once-per-second resyncs, and why some episodes
latched at 96+ resyncs and never recovered.

Fix: explicit EN-clear, simultaneous abort of both channels, EN-restore. Measured 964
desyncs/hour to 21/hour once combined with §3.

**Generalisable:** any DMA abort on RP2350 involving chained channels needs this. It is
not specific to video.

## 3. Core-0 CODE LAYOUT changes the desync rate by orders of magnitude

This is the least obvious thing in this document and the most important.

Core 0's per-field path originally executed from flash via the XIP cache. Adding ~110
lines of **never-executed** code elsewhere in the same file cost **+1.5 ms/field** and
took the desync rate from **0/hr to 964/hr**.

The experiment that proved it: put the new block behind a `volatile bool = false`, so
every byte stays in the image at the same addresses but the body cannot run. The
regression persisted. The cost is the code's *presence*, not its execution — shifted
addresses reshuffle XIP cache-line mapping, and flash-fetch stalls starve the HSTX
FIFO.

Fix: move the whole per-field path into `.time_critical` RAM (`RAM_FUNC`). Cost 4 KB of
SRAM. It also made things *faster* — `blit` 2999 to 2588 us, `avg run` 13049 to 12583.

Verified by a **negative test**: add ~100 lines of flash-resident dead code and confirm
the rate does *not* move. `avg run` agreed to 38 us (0.3%), against +1500 us (12%)
before the fix.

**Consequences, which are severe and retroactive:**

- **Any A/B measurement is invalid unless the hot path is layout-immune.** The act of
  adding instrumentation changes the thing being measured.
- **`avg run` is not a clean measure of core-0 work.** It moves with layout too.
- Several earlier findings in this project were confounded by exactly this, including a
  bisect that appeared to implicate NeoPixel initialisation (§6).

The principle was already known here — the palette is forced into RAM with a comment
saying an XIP stall starves the FIFO, and the 6809 core uses `HOT_FUNC` — it just had
never been applied to the host-side loop.

## 4. Detect onset in ~150 ms, not once per second

A desync free-runs at ~160 fps. If you measure over a 1 s window, a link that breaks
partway through reads *between* 60 and 160 in proportion to when it broke. Captured
values included 68, 71, 74, 81 and 83 fps — every one a real desync second that fell
under a >90 fps threshold and went unnoticed until the next window.

Sampling `video_frame_count` every 150 ms catches onset within 150 ms instead of up to
2 s. 150 ms not 100 ms for margin: a healthy link delivers 9 frames per window against
24 desynced, so a >90 fps threshold sits at 13.5 frames, comfortably clear of ±1 frame
jitter. **A false positive is expensive** — a needless resync is itself a visible
dropout.

## 5. Parking the pins during a resync does *not* shorten the dropout

A tempting theory: the sink re-acquires because it sees malformed TMDS while HSTX is
restarting, so park the pins to SIO first, restart, wait for the first valid line, then
reconnect. (The upstream `video_output_rt.c` does exactly this.)

**Measured: no improvement.** Visible drops stayed at 0.5-2 s.

The monitor does not care about garbage; it cares that **the TMDS clock stops at all**.
Parking pins prevents malformed data, not absence of signal, and re-acquisition time is
dominated by the latter.

If the dropout duration matters, the only promising direction is a resync that **never
disables HSTX** — hold the command stream on valid blanking lines, resynchronise the
DMA at a frame boundary, and resume, so the sink sees continuous sync and a black frame
or two instead of a dropped link. Unproven; it is not clear the FIFO can be brought back
into step without the `EN` toggle that flushes it.

## 6. Electrical load moves the rate too

Desync rate with the five onboard NeoPixels **lit** was ~21/hr, against ~5.9/hr with them
**dark** — roughly 3.5x, for 50-100 mA of extra draw.

It tracks *illumination*, not code: a build with the NeoPixel code compiled out measured
the same ~21/hr while the strip stayed lit, because WS2812s latch and nothing had
cleared them. That accident is what let the two be separated.

This retired a long-standing theory that `Adafruit_NeoPixel::begin()` cost stability by
holding a PIO state machine and colliding with PIO-USB over `gpio_base`. The original
*observation* was real; the *mechanism* was wrong, and the magnitude was overstated
about 15x. Its supporting bisect is uninterpretable for the reason in §3.

**Power-rail integrity is a real class of suspect for HSTX stability**, and it is cheap
to test — vary the load rather than the code.

## 7. Measurement discipline specific to this fault

Beyond the general rules in `CLAUDE.md`:

- **Desyncs arrive in EPISODES.** Three inside 23 seconds, then nothing for seven
  minutes. A window that catches an episode reads high; one that misses reads zero.
  Short captures cannot produce a rate.
- **The noise floor is large.** Idle in an *identical* configuration measured 20.6,
  21.2 and 31.5/hr across ~20-minute windows. Treat anything under an hour per arm as
  indicative only.
- **The resync counter is the discriminator.** It increments only when the RP2350 lost
  the link. A visible glitch with the counter unchanged is downstream — cable,
  connector, or the sink re-acquiring on its own.
- **Define arms in advance and flash them separately.** Segmenting one continuous log
  by inferred machine state produced a wrong conclusion here more than once.
- **A clean serial log means "transmitting", never "displaying."**

## 8. Would we use HSTX again?

Honest ledger, for anyone choosing for a custom board.

**For:**
- Scanout is nearly free. Core 1 is a pointer handoff per line and otherwise idle;
  a PIO/bit-banged implementation spends a whole core on TMDS encoding.
- It leaves the PIO blocks alone, which matters when PIO-USB already wants them.
- That spare headroom is real and spendable — it is what made a 2.5x scaled
  framebuffer affordable at all.

**Against:**
- The latching failure mode of §1, and the safety-critical resync path it forces.
- Extreme sensitivity to bus and flash timing (§3). Debugging it is genuinely hard,
  because the measurement apparatus perturbs the measurement.
- Dropouts are long (§5) and we do not yet know how to make them short.

**Unknown:** whether the residual ~20/hr onset is an HSTX problem or a board/power
problem. If §6 generalises, a bit-banged implementation would suffer it too.

Net: HSTX cost far more debugging than bit-banging would have, and bought headroom that
has since been spent on features. Whether that is a good trade depends on whether your
design needs the cores and PIO blocks for something else.
