#!/bin/bash
set -euo pipefail

# xroar-adafruit-rp2350-fruit-jam — Build & Deploy Script (FRUITJAM-02)
# Usage: ./deploy.sh [--build-only] [--monitor]

cd "$(dirname "$0")"

BUILD_ONLY=false
MONITOR=false

for arg in "$@"; do
    case $arg in
        --build-only) BUILD_ONLY=true ;;
        --monitor)    MONITOR=true ;;
        --help|-h)
            echo "Usage: $0 [--build-only] [--monitor]"
            echo ""
            echo "  --build-only  Build without uploading"
            echo "  --monitor     Open serial monitor after upload"
            exit 0
            ;;
        *)
            echo "Unknown argument: $arg"
            exit 1
            ;;
    esac
done

if ! command -v pio &> /dev/null; then
    echo "Error: PlatformIO CLI (pio) not found."
    echo "Install: https://docs.platformio.org/en/latest/core/installation/methods/installer-script.html"
    exit 1
fi

echo "=== Building for Fruit Jam (RP2350B) ==="
START=$(date +%s)
pio run -e fruitjam
END=$(date +%s)
echo "Build completed in $((END - START))s"

FIRMWARE=$(find .pio/build/fruitjam -name "firmware.elf" 2>/dev/null | head -1)
if [ -n "$FIRMWARE" ] && command -v arm-none-eabi-size &> /dev/null; then
    echo ""
    arm-none-eabi-size "$FIRMWARE"
fi
echo ""

if [ "$BUILD_ONLY" = true ]; then
    echo "Build-only mode — skipping upload."
    exit 0
fi

echo "=== Uploading to Fruit Jam ==="
echo "Connect the board via the USB-C port."
echo "If upload fails, hold BOOT (button 1) while tapping RESET, then retry."
echo "  (Or unplug, hold BOOT, plug in, release once the RP2350 drive appears.)"
echo ""
pio run -e fruitjam -t upload
echo ""
echo "Upload complete!"

if [ "$MONITOR" = true ]; then
    echo "=== Opening serial monitor (115200 baud) ==="
    echo "Press Ctrl+C to exit."
    pio device monitor -b 115200
fi
