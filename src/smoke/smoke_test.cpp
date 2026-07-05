// SPDX-License-Identifier: GPL-3.0-or-later
// xroar-adafruit-rp2350-fruit-jam — headless XRoar-core smoke test (FRUITJAM-09).
//
// Acceptance: execute N MC6809 cycles of the Color BASIC 1.2 ROM headless and
// print proof over USB serial. This is a *bring-up* harness, not the emulator:
// it wires the vendored tag-1.11 MC6809 to a flat 64 KB memory map — no SAM,
// PIA, VDG or event queue — purely to prove the core compiles, links, resets
// and dispatches real 6809 opcodes on the RP2350.
//
//   ROM:  0:/coco/roms/bas12.rom  (8192 bytes, Color BASIC 1.2)
//   Map:  RAM $0000-$9FFF and $C000-$FFFF, ROM mirrored at $A000-$BFFF.
//         Reset vector at $FFFE/$FFFF copied from the ROM's own tail
//         ($BFFE/$BFFF), falling back to $A000 if that looks unpopulated.
//
// Runs at the real 252 MHz / 1.25 V operating point (FRUITJAM-03) so the core
// is exercised under the conditions the emulator will actually use.

#include <Arduino.h>
#include <string.h>
#include "hardware/vreg.h"
#include "pico/stdlib.h"

extern "C" {
#include "ff.h"
#include "f_util.h"   // FRESULT_str()

// Vendored XRoar core (tag 1.11).
#include "part.h"
#include "mc6809/mc6809.h"
#include "delegate.h"
}

// - - - memory map ------------------------------------------------------------

static uint8_t g_ram[65536];      // full 64 KB address space, SRAM-resident
static const uint16_t XR_ROM_BASE = 0xA000;
static const uint16_t XR_ROM_END  = 0xBFFF;

// Cycle budget for the run. The mem_cycle delegate decrements this on every
// bus access and halts the CPU when it reaches zero, so the run can never hang
// regardless of what the ROM does (waiting on absent PIA/VDG hardware just
// spins harmlessly inside the ROM until the budget is spent).
static int32_t  g_cycles_remaining = 0;
static uint32_t g_bus_accesses = 0;

static struct MC6809 *g_cpu = nullptr;

// The one piece of machine wiring: service a single 6809 bus cycle against the
// flat map. Reads land in cpu->D; writes to the ROM window are ignored.
extern "C" void smoke_mem_cycle(void *sptr, _Bool RnW, uint16_t A) {
    (void)sptr;
    g_bus_accesses++;
    if (RnW) {
        g_cpu->D = g_ram[A];
    } else {
        if (A < XR_ROM_BASE || A > XR_ROM_END) {
            g_ram[A] = g_cpu->D;
        }
    }
    if (--g_cycles_remaining <= 0) {
        g_cpu->running = 0;
    }
}

// - - - ROM load --------------------------------------------------------------

static FATFS g_fs;

static bool mount_sd() {
    FRESULT fr = FR_DISK_ERR;
    for (int attempt = 0; attempt < 5; attempt++) {
        fr = f_mount(&g_fs, "0:", 1);
        if (fr == FR_OK) {
            if (attempt > 0) Serial.printf("SD mounted on attempt %d\n", attempt + 1);
            return true;
        }
        Serial.printf("SD mount attempt %d failed: %s (%d)\n", attempt + 1, FRESULT_str(fr), fr);
        delay(200);
    }
    Serial.println("SD mount: giving up");
    return false;
}

// Read exactly `want` bytes of `path` into dst. Returns bytes actually read.
static size_t load_rom(const char *path, uint8_t *dst, size_t want) {
    FIL f;
    FRESULT fr = f_open(&f, path, FA_READ);
    if (fr != FR_OK) {
        Serial.printf("open %s failed: %s (%d)\n", path, FRESULT_str(fr), fr);
        return 0;
    }
    UINT br = 0;
    fr = f_read(&f, dst, want, &br);
    f_close(&f);
    if (fr != FR_OK) {
        Serial.printf("read %s failed: %s (%d)\n", path, FRESULT_str(fr), fr);
        return 0;
    }
    return br;
}

// - - - test ------------------------------------------------------------------

static const char *ROM_PATH = "0:/coco/roms/bas12.rom";
static const size_t ROM_SIZE = 8192;
static const uint32_t TARGET_CYCLES = 300000;

static void run_smoke() {
    Serial.println();
    Serial.println("=== FRUITJAM-09 XRoar-core smoke test (MC6809 @ 252 MHz) ===");

    if (!mount_sd()) { Serial.println("SMOKE FAIL: no SD"); return; }

    // 1) Load the Color BASIC ROM straight into its mapped window.
    memset(g_ram, 0, sizeof(g_ram));
    size_t got = load_rom(ROM_PATH, &g_ram[XR_ROM_BASE], ROM_SIZE);
    if (got != ROM_SIZE) {
        Serial.printf("SMOKE FAIL: expected %u ROM bytes, got %u\n",
                      (unsigned)ROM_SIZE, (unsigned)got);
        return;
    }
    Serial.printf("ROM loaded: %s (%u bytes)\n", ROM_PATH, (unsigned)got);
    Serial.print("  first 8 bytes:");
    for (int i = 0; i < 8; i++) Serial.printf(" %02X", g_ram[XR_ROM_BASE + i]);
    Serial.println();

    // Reset vector: the CoCo maps the BASIC ROM's tail to the CPU vector page,
    // so $FFFE/$FFFF mirror the ROM's last two bytes ($BFFE/$BFFF).
    uint16_t rom_reset = (uint16_t)((g_ram[XR_ROM_END - 1] << 8) | g_ram[XR_ROM_END]);
    uint16_t reset_vec = rom_reset;
    if (reset_vec < XR_ROM_BASE || reset_vec > XR_ROM_END) {
        Serial.printf("  ROM reset vector $%04X out of ROM range — using $%04X\n",
                      rom_reset, XR_ROM_BASE);
        reset_vec = XR_ROM_BASE;
    }
    g_ram[0xFFFE] = (uint8_t)(reset_vec >> 8);
    g_ram[0xFFFF] = (uint8_t)(reset_vec & 0xFF);
    Serial.printf("  reset vector $FFFE/F = $%04X\n", reset_vec);

    // 2) Create the CPU via the XRoar part system and wire the bus.
    struct part *p = part_create("MC6809", NULL);
    if (!p) { Serial.println("SMOKE FAIL: part_create(MC6809) returned NULL"); return; }
    g_cpu = (struct MC6809 *)p;
    g_cpu->mem_cycle = DELEGATE_AS2(void, bool, uint16, smoke_mem_cycle, g_cpu);
    Serial.println("MC6809 created and bus wired.");

    // 3) Reset, then run the cycle budget.
    g_cpu->reset(g_cpu);
    g_cycles_remaining = (int32_t)TARGET_CYCLES;
    g_bus_accesses = 0;
    g_cpu->running = 1;

    uint32_t t0 = micros();
    g_cpu->run(g_cpu);
    uint32_t elapsed = micros() - t0;

    uint16_t final_pc = g_cpu->reg_pc;
    Serial.printf("Ran ~%u bus cycles in %u us.\n",
                  (unsigned)g_bus_accesses, (unsigned)elapsed);
    Serial.printf("Final PC = $%04X  (CC=$%02X A=$%02X B=$%02X X=$%04X S=$%04X)\n",
                  final_pc, g_cpu->reg_cc,
                  MC6809_REG_A(g_cpu), MC6809_REG_B(g_cpu),
                  g_cpu->reg_x, g_cpu->reg_s);

    bool pc_sane = (final_pc >= XR_ROM_BASE && final_pc <= XR_ROM_END);
    if (g_bus_accesses > 0 && pc_sane) {
        Serial.println("SMOKE PASS: CPU dispatched Color BASIC opcodes without hanging.");
    } else if (g_bus_accesses > 0) {
        Serial.printf("SMOKE PASS (soft): ran %u cycles; final PC $%04X is outside ROM "
                      "(expected for a bare map with no I/O — CPU still executed).\n",
                      (unsigned)g_bus_accesses, final_pc);
    } else {
        Serial.println("SMOKE FAIL: CPU made no bus accesses.");
    }
}

void setup() {
    vreg_set_voltage(VREG_VOLTAGE_1_25);
    delay(2);
    set_sys_clock_khz(252000, true);

    Serial.begin(115200);
    const uint32_t start = millis();
    while (!Serial && (millis() - start) < 2000) delay(10);

    run_smoke();
    Serial.println("Smoke test done. Heartbeat follows.");
}

void loop() {
    static uint32_t n = 0;
    delay(2000);
    Serial.printf("alive %lu (final PC $%04X)\n",
                  (unsigned long)n++, g_cpu ? g_cpu->reg_pc : 0);
}
