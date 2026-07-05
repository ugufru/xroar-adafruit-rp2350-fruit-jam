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
    .mosi_gpio  = 35,
    .sck_gpio   = 34,
    .set_drive_strength       = true,
    .mosi_gpio_drive_strength = GPIO_DRIVE_STRENGTH_2MA,
    .sck_gpio_drive_strength  = GPIO_DRIVE_STRENGTH_12MA,
    .baud_rate  = 12500000,   // ~12.5 MHz — conservative first bring-up (pizero-proven)
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
