// xroar-adafruit-rp2350-fruit-jam — headless CoCo boot test (FRUITJAM-22).
//
// Proves the hand-written coco_machine glue boots Color BASIC on real silicon,
// with no display yet: load the ROM off SD, build the machine, run ~2 seconds of
// emulated time, and print evidence over serial —
//   * PC settled inside the Color BASIC ROM ($A000-$BFFF) idle scan loop
//   * the 60 Hz field-sync IRQ source ticking (~120 pulses over 2 s)
//   * the text screen page ($0400) filled with the sign-on message
//
// Runs at the real 252 MHz / 1.25 V operating point (FRUITJAM-03).

#include <Arduino.h>
#include "hardware/vreg.h"
#include "pico/stdlib.h"

extern "C" {
#include "ff.h"
#include "f_util.h"
#include "coco_machine.h"
}

static FATFS   g_fs;
static uint8_t g_rom[16384];

// Authentic NTSC CoCo: 0.895 MHz / 60 Hz ~= 14915 6809 cycles per field.
static const uint32_t CYCLES_PER_FRAME = 14915;
static const int      FRAMES = 120;             // ~2 s emulated

static bool mount_sd() {
    for (int a = 0; a < 5; a++) {
        if (f_mount(&g_fs, "0:", 1) == FR_OK) return true;
        delay(200);
    }
    return false;
}

// Load a Color BASIC ROM. Accepts 8 KB (Color only) or 16 KB (Extended+Color).
static size_t load_rom(const char *path) {
    FIL f;
    if (f_open(&f, path, FA_READ) != FR_OK) return 0;
    UINT br = 0;
    f_read(&f, g_rom, sizeof(g_rom), &br);
    f_close(&f);
    return br;
}

static void dump_screen_row(int row) {
    // Text page $0400, 32 columns/row; VDG code $60 = space (green).
    const uint8_t *scr = coco_machine_peek_ram(0x0400 + row * 32);
    Serial.printf("  $%04X:", 0x0400 + row * 32);
    for (int i = 0; i < 32; i++) Serial.printf(" %02X", scr[i]);
    Serial.print("   \"");
    for (int i = 0; i < 32; i++) {
        // VDG alphanumeric code -> ASCII: $00-$3F are @A-Z.., $60 space. Rough decode.
        uint8_t c = scr[i] & 0x3F;
        char ch = (c >= 0x00 && c <= 0x1F) ? (char)('@' + c) : (char)(0x20 + (c - 0x20));
        Serial.print((scr[i] == 0x60) ? ' ' : ch);
    }
    Serial.println("\"");
}

static void run_boot_test() {
    Serial.println();
    Serial.println("=== FRUITJAM-22 CoCo machine headless boot ===");

    if (!mount_sd()) { Serial.println("BOOT FAIL: no SD"); return; }

    const char *path = "0:/coco/roms/bas12.rom";
    size_t got = load_rom(path);
    if (got != 8192 && got != 16384) {
        Serial.printf("BOOT FAIL: %s is %u bytes (need 8192 or 16384)\n", path, (unsigned)got);
        return;
    }
    Serial.printf("ROM: %s (%u bytes)\n", path, (unsigned)got);

    if (!coco_machine_init(g_rom, got)) { Serial.println("BOOT FAIL: machine init"); return; }
    Serial.println("Machine built: MC6809 + SAM + 2xPIA + VDG(T1).");

    uint32_t t0 = micros();
    for (int f = 0; f < FRAMES; f++) coco_machine_run_cycles(CYCLES_PER_FRAME);
    uint32_t us = micros() - t0;

    uint16_t pc    = coco_machine_get_pc();
    uint32_t mc    = coco_machine_get_total_mem_cycles();
    uint32_t irqs  = coco_machine_get_irq_count();
    uint32_t emu_us = (uint32_t)((uint64_t)FRAMES * CYCLES_PER_FRAME * 1000000ull / 894886ull);

    Serial.printf("Ran %d frames (%lu emulated cycles) in %lu us wall.\n",
                  FRAMES, (unsigned long)((uint32_t)FRAMES * CYCLES_PER_FRAME), (unsigned long)us);
    Serial.printf("Emulated time = %lu us  ->  real-time ratio = %lu.%02lux\n",
                  (unsigned long)emu_us,
                  (unsigned long)(emu_us / us),
                  (unsigned long)(((uint64_t)emu_us * 100 / us) % 100));
    Serial.printf("PC=$%04X  mem_cycles=%lu  field_syncs(IRQ src)=%lu\n",
                  pc, (unsigned long)mc, (unsigned long)irqs);

    Serial.println("Text page $0400 (first 4 rows):");
    for (int r = 0; r < 4; r++) dump_screen_row(r);

    bool pc_in_rom = (pc >= 0xA000 && pc <= 0xBFFF);
    bool irq_alive = (irqs >= 100);   // expect ~120 over 2 s
    if (pc_in_rom && irq_alive) {
        Serial.println("BOOT PASS: CPU idling in Color BASIC ROM, 60 Hz IRQ source live.");
    } else {
        Serial.printf("BOOT PARTIAL: pc_in_rom=%d irq_alive=%d — see numbers above.\n",
                      pc_in_rom, irq_alive);
    }
}

void setup() {
    vreg_set_voltage(VREG_VOLTAGE_1_25);
    delay(2);
    set_sys_clock_khz(252000, true);

    Serial.begin(115200);
    const uint32_t start = millis();
    while (!Serial && (millis() - start) < 2000) delay(10);

    run_boot_test();
    Serial.println("Boot test done. Heartbeat follows.");
}

void loop() {
    static uint32_t n = 0;
    delay(2000);
    Serial.printf("alive %lu (PC $%04X, IRQs %lu)\n",
                  (unsigned long)n++, coco_machine_get_pc(),
                  (unsigned long)coco_machine_get_irq_count());
}
