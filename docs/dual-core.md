<!-- SPDX-License-Identifier: GPL-3.0-or-later -->
# Dual-core split (FRUITJAM-11)

Decision record for how work is divided across the RP2350's two cores, the
core-to-core handoff, and the presentation-buffering choice. The architecture
was realized incrementally (FRUITJAM-25 milestone, FRUITJAM-13 audio, and the
resync freeze fix); this formalizes and measures it.

## The split

```
        CORE 0  (emulation + everything cheap)      CORE 1  (presentation)
        ------------------------------------        ----------------------
loop(): USBHost.task()   -- USB HID host            video_output_core1_run()
        coco_machine_run_cycles(14915)  -- 6809        (never returns):
        coco_machine_render_frame()     -- VDG          per-scanline pointer
        blit_frame()      -- indexed -> RGB565            callback -> g_fb row
        audio_feed()      -- 48 kHz -> I2S DMA          background task:
        pace to 16762 us  -- resync-not-debt              compose_service()
                                                          + deferred HSTX resync
setup() also on core 0: clocks, USB, SD mount, ROM load, machine + audio + video
init, then multicore_launch_core1().
```

**Core 0** carries the entire emulation plus all the cheap per-frame work
(frame compose, the audio resample + non-blocking I2S feed, USB servicing,
pacing). **Core 1** does nothing but the pico_hdmi video engine, which is
near-zero CPU — the HSTX hardware TMDS serializer + DMA do the scanout, and the
per-scanline ISR just returns a framebuffer row pointer (native pixel mode,
hardware 320->640 doubling). Core 1 has *more* idle headroom than any prior port
(vs the pizero's libdvi TMDS or the 43b's ISR pixel streaming).

### Deviation from the original plan

The issue envisioned core 1 also doing the **audio I2S feed and SD service**. In
practice both run on **core 0**:

* `video_output_core1_run()` is a `while(1)` loop that never returns, so core 1
  can't also run an Arduino-style service loop. Audio is fed from core 0's
  `loop()` instead; the earlephilhower I2S library's own DMA ring is the buffer
  (fed non-blocking once per field, hold-last on underrun — FRUITJAM-13). This
  is fine because the I2S DMA is autonomous; core 0 only tops up the ring.
* SD is touched only at boot (ROM load), so it lives in `setup()` on core 0.

Core 1's one bit of non-video work is the **deferred HSTX resync**: core 0 sets
a flag, core 1 performs the resync from its background task (so it can gate its
own DMA IRQ before touching HSTX — the cross-core-race freeze fix).

## Core-to-core handoff (lock-free)

RP2350 SRAM is shared and cache-coherent across cores (no per-core data cache),
so the handoff is plain `volatile` shared state — **no mutexes, no spinlocks**:

| State | Writer | Reader | Mechanism |
|---|---|---|---|
| `g_fb` (320x240 RGB565 framebuffer) | core 0 `blit_frame()` | core 1 scanline callback | single buffer, direct pointer read |
| `g_want_resync` | core 0 watchdog | core 1 background task | `volatile bool` request flag |
| `g_compose_ring` | core 1 compose_service | core 1 ISR | core-1 internal |

## Presentation-buffering decision: single buffer

`g_fb` is a **single** framebuffer read by core 1 while core 0 writes it, so a
blit that lands mid-scanout can tear. This is **accepted**:

* Double-buffering would need two 320x240 RGB565 buffers = **300 KB**. RAM is at
  ~74% (~136 KB free after the SRAM hot set, FRUITJAM-15), so two full buffers
  **do not fit**.
* Double-buffering only the 24 KB indexed VDG frame wouldn't help — the tear is
  in the RGB565 `g_fb` that core 1 scans out.
* The CoCo screen is mostly static (text), so tearing is rarely visible.

If tear-free presentation is ever needed, the route is a smaller/region-based or
palette (indexed-scanout) scheme, not two full RGB565 buffers. Deferred; not
needed for any current milestone.

## Measured (252 MHz, idle at the BASIC prompt)

| | value | notes |
|---|---|---|
| Core 0 per-field run | ~12.9 ms / 16.762 ms budget | **~1.29x real-time, ~23% headroom** |
| Core 1 scanout | 59–60 fps steady | near-zero CPU; hardware DMA |
| Coupling | none observed | core 0 run-time is stable regardless of scanout; no locks, no waits |

**Acceptance met:** emulation timing is unaffected by display activity (separate
core, lock-free SRAM handoff, no blocking), and the headroom is documented here
and in `docs/perf-log.md`. Worst case seen is SOUND at ~1.16x — still real-time.

## Caveat learned

After running the `psram` memtest env, cold-boot (power cycle) before trusting
`coco` timing: the memtest's `psram_reinit_timing()` retunes the *shared* QMI,
and a soft-reset reflash can leave the flash-XIP timing sub-optimal (~1.0x
instead of ~1.29x) until a full reset. A clean rebuild+flash or power cycle
restores it.
