#!/bin/bash
set -euo pipefail

# xroar-adafruit-rp2350-fruit-jam — Build / Flash / Capture helper (FRUITJAM-02).
#
# Usage:
#   ./deploy.sh [-e ENV] [ACTION] [options]
#
# Actions (default: --flash):
#   --build-only       Build only, no upload
#   --flash            Build + upload
#   --capture          Build + upload + read serial and print it (headless-friendly)
#   --monitor          Build + upload + open interactive serial monitor
#   --read-only        Just read serial from the running board (no build/upload)
#
# Options:
#   -e, --env ENV      PlatformIO env (default: fruitjam). e.g. coco, cocoboot, smoke
#   --secs N           Capture duration seconds (default: 15)
#   --until REGEX      Stop capture early once a line matches REGEX
#   --no-build         Skip the build step (upload/capture the existing firmware)
#
# Examples:
#   ./deploy.sh -e coco --capture --until 'real-time' --secs 12
#   ./deploy.sh -e cocoboot --capture --until 'BOOT (PASS|FAIL)'
#   ./deploy.sh -e coco --read-only --secs 8

cd "$(dirname "$0")"

ENV="fruitjam"
ACTION="flash"
SECS=15
UNTIL=""
DO_BUILD=true

while [ $# -gt 0 ]; do
    case "$1" in
        -e|--env)     ENV="$2"; shift 2 ;;
        --build-only) ACTION="build"; shift ;;
        --flash)      ACTION="flash"; shift ;;
        --capture)    ACTION="capture"; shift ;;
        --monitor)    ACTION="monitor"; shift ;;
        --read-only)  ACTION="read"; DO_BUILD=false; shift ;;
        --no-build)   DO_BUILD=false; shift ;;
        --secs)       SECS="$2"; shift 2 ;;
        --until)      UNTIL="$2"; shift 2 ;;
        -h|--help)    sed -n '3,30p' "$0"; exit 0 ;;
        *) echo "Unknown argument: $1"; exit 1 ;;
    esac
done

command -v pio >/dev/null || { echo "Error: PlatformIO CLI (pio) not found."; exit 1; }

# Read serial: wait for the CDC port, open at 115200, print for SECS seconds
# (or until UNTIL regex matches). Uses pyserial (present in this toolchain).
read_serial() {
    python3 - "$SECS" "$UNTIL" <<'PY'
import sys, time, glob
try:
    import serial
except ImportError:
    print("(pyserial not installed: pip install pyserial)"); sys.exit(1)
secs = float(sys.argv[1]); pat = sys.argv[2]
import re
rx = re.compile(pat) if pat else None
port = None
for _ in range(120):                       # wait up to ~12s for re-enumeration
    m = glob.glob('/dev/cu.usbmodem*')
    if m: port = m[0]; break
    time.sleep(0.1)
if not port:
    print("!! no serial port appeared (board may be crashing / in BOOTSEL)"); sys.exit(2)
s = None
for _ in range(60):
    try: s = serial.Serial(port, 115200, timeout=1); break
    except Exception: time.sleep(0.15)
if not s:
    print("!! could not open", port); sys.exit(3)
print("--- serial %s (%.0fs%s) ---" % (port, secs, ", until /%s/" % pat if pat else ""))
end = time.time() + secs
buf = b""
while time.time() < end:
    d = s.read(256)
    if not d: continue
    buf += d
    while b"\n" in buf:
        line, buf = buf.split(b"\n", 1)
        text = line.decode("utf-8", "replace")
        print(text)
        if rx and rx.search(text):
            time.sleep(0.3); s.close(); print("--- (matched) ---"); sys.exit(0)
s.close(); print("--- (timeout) ---")
PY
}

if [ "$DO_BUILD" = true ] && [ "$ACTION" != "read" ]; then
    echo "=== Build [$ENV] ==="
    pio run -e "$ENV"
    FW=$(find ".pio/build/$ENV" -name firmware.elf 2>/dev/null | head -1)
    [ -n "$FW" ] && command -v arm-none-eabi-size >/dev/null && arm-none-eabi-size "$FW" || true
fi

case "$ACTION" in
    build) echo "Build-only — done." ;;
    read)  read_serial ;;
    flash)
        echo "=== Upload [$ENV] ==="
        pio run -e "$ENV" -t upload
        echo "Upload complete."
        ;;
    capture)
        echo "=== Upload [$ENV] ==="
        pio run -e "$ENV" -t upload
        echo "=== Capture ==="
        read_serial
        ;;
    monitor)
        echo "=== Upload [$ENV] ==="
        pio run -e "$ENV" -t upload
        echo "=== Monitor (Ctrl+C to exit) ==="
        pio device monitor -b 115200
        ;;
esac
