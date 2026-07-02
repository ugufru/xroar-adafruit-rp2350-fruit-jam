# XRoar on the Adafruit Fruit Jam

A port of **[XRoar](https://www.6809.org.uk/xroar/)** — Ciaran Anscomb's Dragon / Tandy Color
Computer (CoCo) emulator — to the **[Adafruit Fruit Jam](https://www.adafruit.com/product/6200)**,
an RP2350B-based "credit card sized" mini computer with DVI video, USB host, and I2S audio.

The port targets the **latest XRoar release (1.11)**.

**Status: planning.** No port code yet. Prior experience comes from three proof-of-concept XRoar
ports to Waveshare RP2350 boards; this repo starts fresh from that experience — it is a new port,
not a copy of any of them. The next step is identifying and prioritizing the work as tracked
issues before any coding starts.

## About the Fruit Jam

The [Fruit Jam](https://learn.adafruit.com/adafruit-fruit-jam) is Adafruit's RP2350B "Mini Comp":
a 3.375" × 2.125" board explicitly designed to be a little stand-alone computer — plug in an HDMI
monitor, a USB keyboard, and you have something very much in the spirit of the 8-bit machines
XRoar emulates. That makes it a natural CoCo.

| Spec | Fruit Jam |
|---|---|
| MCU | RP2350B (QFN-80, A2 stepping), dual Cortex-M33 @ 150 MHz stock, 520 KB SRAM |
| Memory | 16 MB QSPI flash + **8 MB PSRAM** |
| Video | **DVI-D over HDMI connector, driven by the RP2350's HSTX peripheral** (GPIO12–19) |
| Audio | **TLV320DAC3100 I2S stereo DAC** — 3.5 mm headphone jack + JST-SH mono speaker port (DIN=GPIO24, MCLK=25, BCLK=26, WS=27; reset on GPIO22, shared with the ESP32-C6) |
| USB host | Two USB-A ports via a **CH334F hub** fed by **PIO-USB on GPIO1 (D+) / GPIO2 (D−)**; GPIO11 gates host 5 V |
| USB device | USB-C for power, bootloading, and USB client |
| Storage | microSD — SPI (SCK=34, MOSI=35, MISO=36, CS=39) or SDIO (extra data pins GPIO37/38) |
| Input | 3 tactile buttons (GPIO0/4/5; button 1 doubles as BOOT), IR receiver (GPIO29) |
| Wireless | ESP32-C6 "AirLift" co-processor (SPI + UART on GPIO8/9) |
| Misc | 5 NeoPixels (GPIO32), red LED (GPIO29, active-low), Stemma QT I2C0 (SDA=20, SCL=21), 2×16 GPIO header, on/off switch, SWD debug port |

Sources: the [Adafruit Fruit Jam learn guide](https://learn.adafruit.com/adafruit-fruit-jam)
(a PDF snapshot, `adafruit-fruit-jam.pdf`, is kept locally in this directory but not committed)
and the [product page](https://www.adafruit.com/product/6200). Note: the board is the A2 stepping
of the RP2350, so the **E9 erratum** (GPIO input leakage) applies. The full verified pin map is in
[`docs/hardware-pinout.md`](docs/hardware-pinout.md) (FRUITJAM-01). It resolved the guide's
one internal contradiction — the SD pin listing gives GPIO34 for both SD_SCK and SD_CARD_DETECT —
against the schematic: **card-detect is GPIO33** (GPIO34 is SD_SCK), and the otherwise-unlisted
**PSRAM chip-select is GPIO47**.

## Why this board is interesting for XRoar

Compared with the three Waveshare boards already explored, the Fruit Jam changes the porting
calculus in four ways:

- **HSTX hardware DVI.** The RP2350's High-Speed Transmit peripheral drives the DVI port directly
  — no PIO TMDS bit-banging, no libdvi CPU/PIO cost as on the Pi-Zero-form-factor port. How the
  HSTX clock relates to the system clock (and therefore how much overclock headroom remains for
  emulation) is a key research item.
- **A real audio path.** The TLV320DAC3100 I2S DAC (configured over I2C) is the first target board
  with proper audio hardware — earlier ports ended at a null audio sink or HDMI audio
  experiments. This board can finally give the CoCo its voice through a real DAC.
- **PSRAM is populated (8 MB).** Earlier porting work established that PSRAM must stay out of the
  per-frame hot path, but as cold/bulk storage (ROM images, cassette/disk images, snapshots) it is
  a real asset the Waveshare pizero board simply didn't have.
- **USB host behind a hub.** Keyboard input arrives via PIO-USB (GPIO1/2) through an onboard
  CH334F 3-port hub. PIO-USB historically pins the system clock to exactly 120 or 240 MHz, and
  hub + full-speed-only behaviour needs verification — this constraint interacts directly with
  the HSTX clocking question above.

## Approach and prior art

- **Upstream:** [XRoar](https://www.6809.org.uk/xroar/) by Ciaran Anscomb (GPL-3.0-or-later),
  tracked locally at `~/github/xroar`, release **1.11**.
- **Prior ports (experience, not code):** three proof-of-concept XRoar ports to Waveshare RP2350
  boards — a 4.3" parallel-RGB LCD, a Pi-Zero form factor driving HDMI via PIO, and a 1.8" QSPI
  AMOLED. They established the RP2350 porting playbook: keep the emulation hot loop in SRAM,
  render frames in one pass rather than beam-chasing scanlines, keep PSRAM out of the hot path,
  and pace the 6809 to authentic real time independent of display refresh. This port applies
  those lessons to a clean implementation for the Fruit Jam's very different hardware.

## Roadmap

1. ~~Initialize repo and README~~ (this commit)
2. File issues for everything the port needs — board bring-up research (HSTX + PIO-USB clocking,
   audio DAC, SD, PSRAM), core integration, display/input/audio seams, performance targets —
   then prioritize them.
3. Start coding, issue by issue.

## License

GPL-3.0-or-later, matching upstream XRoar. See [LICENSE](LICENSE).
