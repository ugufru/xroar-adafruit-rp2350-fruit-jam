# XRoar on the Adafruit Fruit Jam

A port of **[XRoar](https://www.6809.org.uk/xroar/)** — Ciaran Anscomb's Dragon / Tandy Color
Computer (CoCo) emulator — to the **[Adafruit Fruit Jam](https://www.adafruit.com/product/6200)**,
an RP2350B-based "credit card sized" mini computer with DVI video, USB host, and I2S audio.

The port targets the **latest XRoar release (1.11)**.

**Status: it boots and runs.** Color BASIC, Extended Color BASIC, and Disk Extended Color BASIC
all come up on real hardware — DVI video on a monitor, a USB keyboard through the onboard hub,
sound through the headphone jack, and programs loaded from a microSD card (including `DIR` /
`LOAD` / `RUN` off a `.dsk` floppy image). The emulator runs at authentic CoCo speed (~0.895 MHz
6809) with headroom to spare. Remaining work is polish and stretch features — see
[Status](#status) and the tracked issues.

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

## Status

| Area | State | Notes |
|---|---|---|
| HSTX DVI video | ✅ working | 640×480p60; 320×240 RGB565 framebuffer, CoCo 256×192 centered, hardware pixel-doubled |
| Emulation core | ✅ working | XRoar 1.11 (6809, SAM, PIA×2, VDG) vendored fresh from upstream |
| Color / Extended / Disk BASIC | ✅ working | boots to the prompt at authentic speed |
| USB keyboard | ✅ working | HID boot protocol through the CH334F hub (either USB-A port) |
| Audio (TLV320 DAC) | ✅ working | CoCo `SOUND` audible on the 3.5 mm headphone jack over I2S |
| microSD | ✅ working | SPI + FatFS; ROMs and disk/cassette images loaded from a FAT32 card |
| Disk (`.dsk`) | ✅ working | WD2797 FDC + Disk BASIC cartridge; `DIR` / `LOAD` / `LOADM` / `RUN` off a JVC image |
| Cassette (`.cas`) | ✅ working | FSK feeder → PIA1 for `CLOAD`; drop `AUTO.CAS` in `/coco/tapes` (FRUITJAM-28) |
| PSRAM (8 MB) | ✅ working | cold/bulk only (disk images, snapshots); kept out of the per-frame hot path |
| Disk picker overlay | ✅ working | VDG-font list, buttons **or** F12/arrows/Enter; mounts, cold-boots and auto-runs (FRUITJAM-50/66/70/72) |
| Last disk remembered | ✅ working | survives power-cycle via `/coco/lastdsk.txt` (FRUITJAM-71) |
| PMODE 4 artifact colour | ✅ working | F11 cycles off → phase A → phase B (FRUITJAM-73) |
| Buttons / NeoPixels | ✅ working | 3=up 2=mount+boot 1=down; boot-progress strip left lit (FRUITJAM-77) |
| Disk **write-back** to SD | 🚧 gated off | mechanism verified, `COCO_DSK_WRITEBACK=0` pending FRUITJAM-88 |
| Performance | ✅ locked | ~1.33× real-time idle (12.6 ms/field); steady 59–60 fps |
| HSTX link stability | ✅ usable | ~7.6 desyncs/hr over 25 h, each a brief recovering flicker; onset open (FRUITJAM-58) |
| USB joystick / gamepad | ⬜ planned | **next up** (FRUITJAM-18) |
| Multi-drive (drives 1–3) | ⬜ planned | picker manages drive 0 only (FRUITJAM-78) |
| ESP32-C6 co-processor | ⬜ planned | networked disk images, remote console, OTA (FRUITJAM-90) |
| CoCo 3 (GIME) | ⬜ research | needs ~2× CPU throughput for fast mode; analysis in FRUITJAM-84 |
| Cassette **audio** (`.wav`/`.aiff`) | ❌ out of scope | decided 2026-08-26 (FRUITJAM-30/34) |
| HDMI audio (HSTX data islands) | ⬜ stretch | second audio sink alongside the DAC (FRUITJAM-14) |

**Known issues:**
- `mount_sd()` can hang on a loose / unresponsive microSD instead of timing out — reseat the card
  (FRUITJAM-26).
- Disk write-back to SD is gated off by default pending FRUITJAM-88; saves work within a session
  and are lost on power-down.
- HSTX desync onset is unexplained (FRUITJAM-58). Each event is a brief flicker that recovers;
  the rate tracks board current, so a lit LED strip roughly triples it.

**Operating point:** 252 MHz system clock at 1.25 V core; `clk_hstx` = 126 MHz (FRUITJAM-03).
252 MHz divides exactly for PIO-USB and serves HSTX 480p with room left for emulation.

## Build and run

Built with [PlatformIO](https://platformio.org/) against the
[earlephilhower arduino-pico](https://github.com/earlephilhower/arduino-pico) core (the Fruit Jam
has official BSP support there). The integration firmware is the `coco` environment:

```sh
# Build
pio run -e coco

# Flash: hold BOOT, tap reset to enter the UF2 bootloader, then either
pio run -e coco -t upload
#   ...or drag .pio/build/coco/firmware.uf2 onto the RPI-RP2 drive.

# Watch the serial log (boot stages + per-field performance)
pio device monitor -b 115200
```

There are also smaller single-purpose environments for hardware bring-up and regression
(`display`, `sdtest`, `audiotest`, `psram`, `usbkbd`, `clockprobe`, `smoke`, `cocoboot`) — see
`platformio.ini`.

## SD card layout

A FAT32 microSD supplies the ROMs and any disk/cassette/binary images. ROMs are **not** included
(source them yourself). Expected paths:

```
0:/coco/roms/bas12.rom      Color BASIC 1.2 (8K)          — required
0:/coco/roms/extbas11.rom   Extended Color BASIC 1.1 (8K) — enables Extended (16K Ext+Color)
0:/coco/roms/disk11.rom     Disk Extended Color BASIC     — enables the disk cartridge at $C000
0:/coco/dsk/AUTO.DSK        JVC .dsk floppy image, auto-mounted (falls back to roms/coco.dsk)
0:/coco/tapes/AUTO.CAS      cassette image for CLOAD
0:/coco/bin/AUTO.BIN        DECB .bin, auto-EXEC'd on boot
```

With just `bas12.rom` you get plain Color BASIC; add `extbas11.rom` for Extended; add
`disk11.rom` + a `.dsk` for Disk BASIC.

## Why this board is interesting for XRoar

Compared with the three Waveshare boards previously explored, the Fruit Jam changed the porting
calculus in four ways — all now validated on hardware:

- **HSTX hardware DVI.** The RP2350's High-Speed Transmit peripheral drives the DVI port directly
  — no PIO TMDS bit-banging, no libdvi CPU/PIO cost as on the Pi-Zero-form-factor port. Scanout is
  near-zero CPU (hardware serializer + DMA + a per-scanline pointer callback), which frees core 1
  and leaves generous overclock headroom for emulation. See [`docs/display-hstx.md`](docs/display-hstx.md).
- **A real audio path.** The TLV320DAC3100 I2S DAC (configured over I2C) is the first target board
  with proper audio hardware — earlier ports ended at a null audio sink or HDMI audio experiments.
  The CoCo now speaks through a real DAC.
- **PSRAM is populated (8 MB).** It stays out of the per-frame hot path, but as cold/bulk storage
  (disk/cassette images, snapshots) it is a real asset the Waveshare pizero board lacked. Policy in
  [`docs/psram-policy.md`](docs/psram-policy.md).
- **USB host behind a hub.** Keyboard input arrives via PIO-USB (GPIO1/2) through an onboard
  CH334F 3-port hub — TinyUSB hub support (`CFG_TUH_HUB`) that no prior port had needed. USB is
  full-speed only; simple wired keyboards and 2.4 GHz dongles enumerate.

## Approach and prior art

- **Upstream:** [XRoar](https://www.6809.org.uk/xroar/) by Ciaran Anscomb (GPL-3.0-or-later),
  tracked locally at `~/github/xroar`, release **1.11**.
- **Prior ports (experience, not code):** three proof-of-concept XRoar ports to Waveshare RP2350
  boards — a 4.3" parallel-RGB LCD, a Pi-Zero form factor driving HDMI via PIO, and a 1.8" QSPI
  AMOLED. They established the RP2350 porting playbook: keep the emulation hot loop in SRAM,
  render frames in one pass rather than beam-chasing scanlines, keep PSRAM out of the hot path,
  and pace the 6809 to authentic real time independent of display refresh. This port applies those
  lessons to a clean implementation for the Fruit Jam's very different hardware — the core was
  re-vendored fresh from upstream 1.11, not copied from any prior port.

## Documentation

- [`docs/hstx-lessons.md`](docs/hstx-lessons.md) — HSTX field notes: the latching
  failure mode, RP2350-E5, code-layout sensitivity, and the theories that were wrong.
- [`docs/hardware-pinout.md`](docs/hardware-pinout.md) — verified pin map (FRUITJAM-01)
- [`docs/clock-plan.md`](docs/clock-plan.md) — 252 MHz / HSTX / PIO-USB clocking (FRUITJAM-03)
- [`docs/display-hstx.md`](docs/display-hstx.md) — HSTX DVI scanout
- [`docs/dual-core.md`](docs/dual-core.md) — core 0 / core 1 split (FRUITJAM-11)
- [`docs/psram-policy.md`](docs/psram-policy.md) — PSRAM usage policy (FRUITJAM-08)
- [`docs/perf-log.md`](docs/perf-log.md) — running performance record (FRUITJAM-15)

## License

GPL-3.0-or-later, matching upstream XRoar. See [LICENSE](LICENSE).
</content>
</invoke>
