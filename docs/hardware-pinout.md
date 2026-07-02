# Fruit Jam — verified hardware pinout

Resolves **FRUITJAM-01**. This is the authoritative pin map for the port. Where the
Adafruit learn guide's pinout pages conflict with the board schematic, **the schematic
wins** and the conflict is called out below.

**Sources**
- Schematic & fab print: `adafruit-fruit-jam.pdf` p.310 ("Adafruit Fruit Jam, Sheet 1/2",
  dated 7/8/2025), read directly off the RP2350B 80-pin symbol's net labels.
- Pinout prose: same PDF pp.26–29.
- Board: RP2350B (QFN-80). GPIO0–47 available.

## Conflicts resolved against the schematic

| Item | Learn guide says | Schematic says | Resolution |
|---|---|---|---|
| **SD card-detect** | `SD_CARD_DETECT/GPIO34` (pp.28) — but GPIO34 is *also* listed as `SD_SCK`, an internal contradiction | Net `SD_DETECT` → **pin 41 → GPIO33** (`GPIO33/CS0/RX0/SCL0/PWM8B`). GPIO34 is `SCK/SDIO_CLK` only. | **Card-detect = GPIO33.** The guide's "GPIO34" for card-detect is a typo. |
| **PSRAM chip-select** | not stated in the pinout pages | Net `PSRAM_CS` → **pin 56 → GPIO47** (`GPIO47/ADC7/VMOSI1/RTS0/SCL1/PWM11B`) | **PSRAM_CS = GPIO47** (Adafruit's standard RP2350 PSRAM-CS pin). |

## Verified pin table

Read directly off the RP2350B symbol on schematic sheet 1/2. "Pin" = QFN-80 package pin.

### Video — HSTX DVI (FRUITJAM-04)
| Net | GPIO | Pin | Signal |
|---|---|---|---|
| HSTX12 | 12 | 11 | DVI CK− (per guide: CKn/p=12/13) |
| HSTX13 | 13 | 12 | DVI CK+ |
| HSTX14 | 14 | 13 | DVI D0− |
| HSTX15 | 15 | 14 | DVI D0+ |
| HSTX16 | 16 | 15 | DVI D1− |
| HSTX17 | 17 | 16 | DVI D1+ |
| HSTX18 | 18 | 17 | DVI D2− |
| HSTX19 | 19 | 18 | DVI D2+ |

### Audio — TLV320DAC3100 I2S + shared reset (FRUITJAM-07/13)
| Net | GPIO | Signal |
|---|---|---|
| I2S_DIN | 24 | I2S data in to DAC |
| I2S_MCLK | 25 | I2S master clock (optional; DAC PLL can derive from BCLK) |
| I2S_BCLK | 26 | I2S bit clock |
| I2S_LRCLK | 27 | I2S word select (WS) |
| I2S_ESP_IRQ | 23 | IRQ (name shared with ESP path) |
| PERIPH_RST | 22 | **Reset — single net shared by the DAC *and* the ESP32-C6.** Toggling it resets both. |
| SDA / SCL | 20 / 21 | I2C0 (Stemma QT bus) — DAC is configured over this bus |

### USB host — PIO-USB behind CH334F hub (FRUITJAM-05)
| Net | GPIO | Pin | Signal |
|---|---|---|---|
| USBH_D+ | 1 | 78 | USB host D+ (PIO-USB) |
| USBH_D− | 2 | 79 | USB host D− (PIO-USB) |
| USBH_PWR | 11 | 9 | USB host 5V gate (drive to enable host power) |

- D+/D− order confirmed: **D+ = GPIO1, D− = GPIO2** (consecutive, D+ lower — matches guide).
- Both USB-A jacks are ports on an onboard **CH334F** 3-port hub; the third hub port's
  D+/D− are broken out on the 2×16 header (guide pp.28–29). ⇒ `CFG_TUH_HUB=1` mandatory.
- USB *device* (USB-C) uses the dedicated `USB_D−`/`USB_D+` at pins 66/67.

### Storage — microSD (FRUITJAM-06)
| Net | GPIO | Pin | SPI role | SDIO role |
|---|---|---|---|---|
| SCK/SDIO_CLK | 34 | 42 | SCK | CLK |
| MOSI/SDIO_CMD | 35 | 43 | MOSI | CMD |
| MISO/SDIO_DAT0 | 36 | 44 | MISO | DAT0 |
| SDCS/SDIO_DAT3 | 39 | 47 | CS | DAT3 |
| SDIO_DAT1 | 37 | 45 | — | DAT1 |
| SDIO_DAT2 | 38 | 46 | — | DAT2 |
| SD_DETECT | **33** | 41 | card-detect | card-detect |

### Board UI & misc (FRUITJAM-16)
| Net | GPIO | Notes |
|---|---|---|
| Button 1 / BOOT | 0 | also USB-BOOT at power-on — don't trap users in bootloader |
| Button 2 | 4 | |
| Button 3 | 5 | |
| NEOPIX | 32 | 5× NeoPixel |
| LED (red) | 29 | active-low; shared with IR receiver — leave alone |
| A0 | 40 | JST-PH / ADC0 (cross-check: guide says GPIO40 ✓) |
| PSRAM_CS | 47 | see conflicts table |
| ESP_CS | 46 | ESP32-C6 (out of scope for the port) |

## Not resolvable from the PDF — verify on hardware / in the BSP

1. **GPIO11 (USBH_PWR) gate polarity.** Confirmed as the host-5V enable pin, but the
   load-switch sense (active-high vs -low) is on schematic sheet 2 (hub/USB-A block, not
   rendered legibly). Assume **drive-high-to-enable** (Adafruit convention) and confirm
   against the board support package or on the bench.
2. **RP2350B silicon stepping.** Guide claims A2 (⇒ E9 erratum applies). Read the actual
   chip marking on the board — prior Waveshare experience saw rev-3 silicon where
   Pico-PIO-USB's E9 workaround compiles out (`chip_version <= 2` gate), which changes
   hot-replug behavior (see FRUITJAM-05/17).
3. **CH334F hub internal topology** (per-port power, enumeration order) — sheet 2; verify
   during FRUITJAM-05 bring-up.
