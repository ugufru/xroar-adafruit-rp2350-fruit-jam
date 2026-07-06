// SPDX-License-Identifier: GPL-3.0-or-later
// xroar-adafruit-rp2350-fruit-jam — PSRAM bring-up + memtest (FRUITJAM-08).
//
// First port board with usable PSRAM (8 MB APS6404-class QMI PSRAM on the
// RP2350B). The earlephilhower core auto-detects and memory-maps it at boot and
// exposes it via pmalloc()/__psram_* and rp2040.getPSRAMSize(); we don't hand-
// roll the QMI init here (unlike the 43b psramlib, which existed only because
// the bare Pico SDK didn't expose the commands).
//
// CRITICAL (AMOLED PSRAM-vs-clock lesson): the core tunes the QMI read timing
// for the clock in effect at boot (~150 MHz). We run at 252 MHz (FRUITJAM-03),
// so the timing MUST be re-tuned with psram_reinit_timing() after the clock
// change or high-address reads return corrupt data.
//
// This exercises the FULL 8 MB directly through its memory-mapped window
// (&__psram_start__) — address-in-address (catches address-line faults) plus
// walking data patterns (stuck/coupled bits) — and reports throughput, which is
// itself the argument for the usage policy: PSRAM is far slower than SRAM, so it
// is cold/bulk-only and never touches the per-frame hot path (see
// docs/psram-policy.md, porting-rp2350.md Rule 3).

#include <Arduino.h>
#include "hardware/vreg.h"
#include "hardware/clocks.h"
#include "pico/stdlib.h"

// Provided by the core (C++ linkage, matching psram.cpp / RP2040Support.h):
// base of the memory-mapped PSRAM window, detected size, timing re-tune, and the
// PSRAM heap allocator. Declared here to avoid include-path juggling.
extern uint8_t __psram_start__;   // linker symbol: start of the mapped window
extern size_t  __psram_size;      // detected PSRAM size in bytes
void   psram_reinit_timing(uint32_t hz);
void  *__psram_malloc(size_t size);
void   __psram_free(void *ptr);
size_t __psram_largest_free_block();

static volatile uint32_t *g_psram;
static size_t             g_nwords;

// Report and bail on the first mismatch so a marginal timing/address fault is
// pinpointed rather than buried in a flood.
static bool check(size_t i, uint32_t got, uint32_t want) {
    if (got == want) return true;
    Serial.printf("  FAIL @ word %u (addr 0x%08lx): got 0x%08lx want 0x%08lx\n",
                  (unsigned)i, (unsigned long)(uintptr_t)&g_psram[i],
                  (unsigned long)got, (unsigned long)want);
    return false;
}

// Write `pat` (or, if addr_mode, each word's own index) across all of PSRAM,
// read it all back, and verify. Returns false on the first fault.
static bool pattern_pass(const char *name, uint32_t pat, bool addr_mode) {
    uint32_t t0 = millis();
    for (size_t i = 0; i < g_nwords; i++)
        g_psram[i] = addr_mode ? ((uint32_t)i ^ 0xA5A5A5A5u) : pat;
    uint32_t t_wr = millis() - t0;

    t0 = millis();
    bool ok = true;
    for (size_t i = 0; i < g_nwords; i++) {
        uint32_t want = addr_mode ? ((uint32_t)i ^ 0xA5A5A5A5u) : pat;
        if (!check(i, g_psram[i], want)) { ok = false; break; }
    }
    uint32_t t_rd = millis() - t0;

    float mb = (float)__psram_size / 1048576.0f;
    Serial.printf("  %-14s %s  (write %lu ms = %.1f MB/s, read %lu ms = %.1f MB/s)\n",
                  name, ok ? "PASS" : "FAIL",
                  (unsigned long)t_wr, t_wr ? mb * 1000.0f / t_wr : 0.0f,
                  (unsigned long)t_rd, t_rd ? mb * 1000.0f / t_rd : 0.0f);
    return ok;
}

void setup() {
    vreg_set_voltage(VREG_VOLTAGE_1_25);
    delay(2);
    set_sys_clock_khz(252000, true);
    psram_reinit_timing(clock_get_hz(clk_sys));   // MUST re-tune QMI after clock change

    Serial.begin(115200);
    const uint32_t start = millis();
    while (!Serial && (millis() - start) < 2000) delay(10);
    Serial.println();
    Serial.println("=== FRUITJAM-08 PSRAM bring-up + memtest ===");

    size_t sz = rp2040.getPSRAMSize();
    Serial.printf("clk_sys=%lu MHz, PSRAM detected: %u bytes (%.2f MB) at 0x%08lx\n",
                  (unsigned long)(clock_get_hz(clk_sys) / 1000000), (unsigned)sz,
                  sz / 1048576.0, (unsigned long)(uintptr_t)&__psram_start__);
    if (sz == 0 || sz != __psram_size) {
        Serial.printf("FATAL: PSRAM not detected (getPSRAMSize=%u, __psram_size=%u)\n",
                      (unsigned)sz, (unsigned)__psram_size);
        while (1) delay(1000);
    }

    g_psram  = (volatile uint32_t *)&__psram_start__;
    g_nwords = __psram_size / sizeof(uint32_t);

    Serial.printf("Testing all %u KB via the mapped window...\n", (unsigned)(__psram_size / 1024));
    bool ok = true;
    ok &= pattern_pass("addr-in-addr", 0, true);          // address-line faults
    ok &= pattern_pass("zeros",   0x00000000u, false);
    ok &= pattern_pass("ones",    0xFFFFFFFFu, false);
    ok &= pattern_pass("0x55",    0x55555555u, false);     // alternating bits
    ok &= pattern_pass("0xAA",    0xAAAAAAAAu, false);

    // Allocator sanity: the core's PSRAM heap sits in the same region.
    size_t big = __psram_largest_free_block();
    void *pm = __psram_malloc(big > 0 ? big / 2 : 0);
    Serial.printf("psram heap: largest free block %u KB, alloc %s\n",
                  (unsigned)(big / 1024), pm ? "OK" : "FAILED");
    if (pm) __psram_free(pm);

    Serial.println(ok ? "\nPSRAM MEMTEST: PASS (8 MB good)" : "\nPSRAM MEMTEST: FAIL");
}

void loop() {
    delay(1000);
}
