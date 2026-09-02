// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * microSD hardware configuration for the Adafruit Fruit Jam (FRUITJAM-06).
 * carlk3 no-OS-FatFS-SD-SDIO-SPI-RPi-Pico, SPI mode.
 *
 * Pin map from docs/hardware-pinout.md (verified against the schematic):
 *   SPI0 SCK  = GPIO34  (SD_SCK)
 *   SPI0 MOSI = GPIO35  (SD_MOSI, SPI0 TX)
 *   SPI0 MISO = GPIO36  (SD_MISO, SPI0 RX)
 *        CS   = GPIO39  (SD_CS, software-driven plain GPIO)
 *   card-detect = GPIO33 (SD_DETECT) — not used yet (polarity unverified,
 *                 FRUITJAM-01 open item); enable once confirmed on hardware.
 *
 * GPIO34/35/36 are the SPI0 alt-function pins on the RP2350B; CS is a GPIO.
 * Internal MISO pull-up left enabled (no_miso_gpio_pull_up unset) since the
 * board's external pull-up situation isn't confirmed — harmless if one exists.
 */

#include "hw_config.h"
#include "hardware/spi.h"

static spi_t g_sd_spi = {
    .hw_inst    = spi0,
    .miso_gpio  = 36,
// FRUITJAM-97: both knobs are build-settable so measurement arms are one flag
// each and can be flashed separately, per CLAUDE.md's rule about defining arms
// in advance rather than segmenting one log.
#ifndef COCO_SD_BAUD
#define COCO_SD_BAUD 12500000
#endif
// 12 mA, the ORIGINAL bring-up value. It was changed to 4 mA on 2026-09-02 on the
// strength of an eyeball A/B/A, and that was WRONG: measured with the FIFO probe
// over ~55 trials per arm, 4 mA averages 5.54 underruns per SD read against
// 12 mA's 2.50 — the "fix" more than DOUBLED the fault it was meant to cure.
// Do not lower this again without a FIFO-probe measurement (FRUITJAM-97).
#ifndef COCO_SD_SCK_DRIVE
#define COCO_SD_SCK_DRIVE GPIO_DRIVE_STRENGTH_12MA
#endif

    .mosi_gpio  = 35,
    .sck_gpio   = 34,
    .set_drive_strength       = true,
    .mosi_gpio_drive_strength = GPIO_DRIVE_STRENGTH_2MA,
    // FRUITJAM-97: 4 mA, NOT the 12 mA this started at. The SD clock switches at
    // 12.5 MHz for ~151 ms during an image load, and at 12 mA that read reliably
    // dropped the HSTX link — the picture cut out for 1-2 s on every disk mount.
    //
    // CONFIRMED BY A/B/A on hardware: 12 mA drops, 4 mA does not, 12 mA drops
    // again immediately. Reads still complete in the same 151 ms at 4 mA, so the
    // card is comfortable and the change costs nothing.
    //
    // The fault is SUPPLY NOISE, not bus contention. 12 mA was the same drive as
    // the HSTX pins themselves and 6x everything else on this bus, making the SD
    // clock the loudest aggressor on the board for the duration of a read. This
    // is the first hard confirmation of the "rate tracks board current" lead in
    // FRUITJAM-58 (see also FRUITJAM-77: a lit LED strip alone costs 3.5x).
    //
    // Do not raise this without re-running the A/B/A.
    .sck_gpio_drive_strength  = COCO_SD_SCK_DRIVE,
    .baud_rate  = COCO_SD_BAUD,   // ~12.5 MHz — conservative first bring-up (pizero-proven)
};

static sd_spi_if_t spi_if = {
    .spi     = &g_sd_spi,
    .ss_gpio = 39,
    .set_drive_strength     = true,
    .ss_gpio_drive_strength = GPIO_DRIVE_STRENGTH_2MA,
};

static sd_card_t sd_card = {
    .type     = SD_IF_SPI,
    .spi_if_p = &spi_if,
};

size_t sd_get_num(void) { return 1; }

sd_card_t *sd_get_by_num(size_t num) {
    return (num == 0) ? &sd_card : NULL;
}
