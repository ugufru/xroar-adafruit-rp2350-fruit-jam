<!-- SPDX-License-Identifier: GPL-3.0-or-later -->
# PSRAM usage policy (FRUITJAM-08)

The Fruit Jam has **8 MB of QMI PSRAM** on the RP2350B — the first port board
with usable PSRAM. This documents what it is, how to use it, and the one rule
that matters.

## What it is / how it's set up

* 8 MB APS6404-class PSRAM on the QMI, memory-mapped by the earlephilhower core
  at boot (`&__psram_start__`, size in `__psram_size` / `rp2040.getPSRAMSize()`).
  We do **not** hand-roll QMI init (the 43b `psramlib` existed only because the
  bare Pico SDK didn't expose the commands; the Arduino core does).
* Heap allocator: `pmalloc()` / `__psram_malloc()` / `__psram_free()`, separate
  from the SRAM `malloc` heap. `__psram_largest_free_block()` reports headroom.
* **CRITICAL — re-tune timing after any clock change.** The core tunes the QMI
  read strobe for the clock in effect at boot (~150 MHz). We run at 252 MHz
  (FRUITJAM-03), so `psram_reinit_timing(clock_get_hz(clk_sys))` **must** be
  called after `set_sys_clock_khz()` or high-address reads corrupt silently.
  (The AMOLED PSRAM-vs-clock lesson.)

## Measured performance (FRUITJAM-08 memtest, 252 MHz)

Full-8 MB memtest passes: address-in-address + walking 0x00/0xFF/0x55/0xAA.

| | throughput |
|---|---|
| PSRAM write (mapped window) | **~9.9 MB/s** |
| PSRAM read (mapped window)  | **~25 MB/s** |

For scale: SRAM is single-cycle at 252 MHz (hundreds of MB/s to GB/s). PSRAM is
**~10–25× slower**, and every access can stall the bus.

## The rule (porting-rp2350.md Rule 3)

> **PSRAM is poison in the per-frame hot path. Use it for cold / bulk data only.**

The single biggest perf regression on the 43b port (4.3×, COCO-42) was a PSRAM
framebuffer `memcpy` running from an ISR. At ~10 MB/s write, one 150 KB
framebuffer copy is ~15 ms — most of a 16.7 ms frame, gone.

**Allowed in PSRAM (cold / bulk / touched rarely):**
* ROM library cache (multiple ROM images held resident)
* Cassette / disk images loaded from SD (FRUITJAM-19)
* Machine snapshots / save states
* Anything read/written in bulk outside the 60 Hz loop

**MUST stay in SRAM (hot / per-frame / per-cycle):**
* CoCo 64 KB RAM (`g_ram[]`) — touched every 6809 memory cycle (COCO-41)
* Framebuffers (`g_fb`) and the VDG palette-index buffer — scanned/blit every frame
* Hot code — the HOT_FUNC `.time_critical` set (FRUITJAM-15)
* Any per-frame or per-cycle buffer

Rule of thumb: if it's touched inside `loop()`/the emulation/scanout path, it is
SRAM. PSRAM is a warehouse, not a workbench.

## Status

Populated and verified (8 MB memtest PASS). Currently **unused** (0 bytes) — it
comes online when a consumer needs it (ROM cache / cassette-disk images,
FRUITJAM-19). The `psram` PlatformIO env holds the memtest.
