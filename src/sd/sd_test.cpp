// SPDX-License-Identifier: GPL-3.0-or-later
// xroar-adafruit-rp2350-fruit-jam — microSD bring-up test (FRUITJAM-06).
//
// Proves the SD path end to end: mount a FAT32 card over SPI0, list the root
// directory, then find and read a ROM image — the FRUITJAM-06 acceptance.
//
// Carries the prior-port lessons:
//   - cold-boot SD init is flaky: retry f_mount up to 5x with 200 ms gaps.
//   - skip macOS "._*" and other dotfiles when enumerating (PIZERO-10).
//
// Runs at the real 252 MHz / 1.25 V operating point (FRUITJAM-03) so the SD
// bring-up is validated under the conditions the emulator will actually use.

#include <Arduino.h>
#include "hardware/vreg.h"
#include "pico/stdlib.h"

extern "C" {
#include "ff.h"
#include "f_util.h"   // FRESULT_str()
}

static FATFS g_fs;

// A directory entry we skip for ROM search: "." / ".." and macOS "._*"/dotfiles.
static inline bool is_skippable(const char *name) {
    return name[0] == '.';
}

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

// List a directory; return the first ROM-ish filename found (into out), skipping
// dotfiles. ROM-ish = extension .rom/.ccc/.bin/.dgn (case-insensitive).
static bool list_dir(const char *dir, char *out, size_t out_sz) {
    DIR d;
    FRESULT fr = f_opendir(&d, dir);
    if (fr != FR_OK) {
        Serial.printf("opendir %s failed: %s (%d)\n", dir, FRESULT_str(fr), fr);
        return false;
    }
    Serial.printf("Listing %s :\n", dir);
    bool found = false;
    int count = 0, skipped = 0;
    for (;;) {
        FILINFO fi;
        fr = f_readdir(&d, &fi);
        if (fr != FR_OK || fi.fname[0] == 0) break;
        if (is_skippable(fi.fname)) { skipped++; continue; }
        Serial.printf("  %c %10lu  %s\n",
                      (fi.fattrib & AM_DIR) ? 'D' : 'F',
                      (unsigned long)fi.fsize, fi.fname);
        count++;
        if (!found && !(fi.fattrib & AM_DIR)) {
            const char *dot = strrchr(fi.fname, '.');
            if (dot && (!strcasecmp(dot, ".rom") || !strcasecmp(dot, ".ccc") ||
                        !strcasecmp(dot, ".bin") || !strcasecmp(dot, ".dgn"))) {
                snprintf(out, out_sz, "%s/%s", (dir[strlen(dir)-1] == '/') ? "" : dir, fi.fname);
                // normalize "0:/" + name
                snprintf(out, out_sz, "%s%s%s", dir,
                         (dir[strlen(dir)-1] == '/') ? "" : "/", fi.fname);
                found = true;
            }
        }
    }
    f_closedir(&d);
    Serial.printf("(%d entries, %d dotfiles skipped)\n", count, skipped);
    return found;
}

// Read a file, print size + first 16 bytes + a 32-bit sum checksum.
static bool read_rom(const char *path) {
    FIL f;
    FRESULT fr = f_open(&f, path, FA_READ);
    if (fr != FR_OK) {
        Serial.printf("open %s failed: %s (%d)\n", path, FRESULT_str(fr), fr);
        return false;
    }
    FSIZE_t size = f_size(&f);
    uint8_t buf[512];
    UINT br = 0;
    uint32_t sum = 0, total = 0;
    bool first = true;
    for (;;) {
        fr = f_read(&f, buf, sizeof(buf), &br);
        if (fr != FR_OK) { Serial.printf("read error: %s\n", FRESULT_str(fr)); f_close(&f); return false; }
        if (br == 0) break;
        if (first) {
            Serial.printf("First 16 bytes of %s:\n  ", path);
            for (int i = 0; i < 16 && (UINT)i < br; i++) Serial.printf("%02X ", buf[i]);
            Serial.println();
            first = false;
        }
        for (UINT i = 0; i < br; i++) sum += buf[i];
        total += br;
    }
    f_close(&f);
    Serial.printf("Read %s: %lu bytes (f_size=%lu), sum32=0x%08lX\n",
                  path, (unsigned long)total, (unsigned long)size, (unsigned long)sum);
    return total == (uint32_t)size;
}

void setup() {
    vreg_set_voltage(VREG_VOLTAGE_1_25);
    delay(2);
    set_sys_clock_khz(252000, true);

    Serial.begin(115200);
    const uint32_t start = millis();
    while (!Serial && (millis() - start) < 2000) delay(10);

    Serial.println();
    Serial.println("=== FRUITJAM-06 microSD bring-up (SPI0) ===");

    if (!mount_sd()) return;

    // List the CoCo asset dirs so we can see exact filenames (FRUITJAM-19/29).
    static const char *dirs[] = { "0:/coco/roms", "0:/coco/dsk", "0:/coco/bin" };
    char rom_path[128] = {0};
    bool have_rom = false;
    for (size_t i = 0; i < sizeof(dirs) / sizeof(dirs[0]); i++) {
        FILINFO fi;
        if (!(f_stat(dirs[i], &fi) == FR_OK && (fi.fattrib & AM_DIR))) {
            Serial.printf("(%s not present)\n", dirs[i]);
            continue;
        }
        if (list_dir(dirs[i], rom_path, sizeof(rom_path))) have_rom = true;
    }

    if (have_rom) {
        Serial.printf("Found ROM candidate: %s\n", rom_path);
        read_rom(rom_path);
    } else {
        Serial.println("No .rom/.ccc/.bin/.dgn file found — listing succeeded though.");
        Serial.println("(Drop a ROM on the card to exercise the read path.)");
    }
    Serial.println("SD test done.");
}

void loop() {
    static uint32_t n = 0;
    delay(2000);
    Serial.printf("idle %lu\n", (unsigned long)n++);
}
