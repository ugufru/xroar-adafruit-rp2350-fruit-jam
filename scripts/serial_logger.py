#!/usr/bin/env python3
# Robust, self-reconnecting serial logger for the Fruit Jam board.
# - Timestamps every line (wall clock) so we can tell WHEN a freeze happened.
# - Survives USB disconnects / reboots / frozen-port termios errors by
#   closing and retrying, logging every state transition.
# Usage: serial_logger.py <device> <logfile> [baud]
import sys, time, datetime
import serial

dev  = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbmodem2101"
path = sys.argv[2] if len(sys.argv) > 2 else "kaleidsc-freeze.log"
baud = int(sys.argv[3]) if len(sys.argv) > 3 else 115200

def ts():
    return datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]

def mark(f, msg):
    f.write(f"[{ts()}] === {msg} ===\n"); f.flush()

with open(path, "a", buffering=1) as f:
    mark(f, f"LOGGER START dev={dev} baud={baud}")
    last_rx = None
    while True:
        try:
            sp = serial.Serial(dev, baud, timeout=1)
            mark(f, "PORT OPENED")
            buf = b""
            while True:
                chunk = sp.read(256)
                if chunk:
                    buf += chunk
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        text = line.decode("utf-8", "replace").rstrip("\r")
                        f.write(f"[{ts()}] {text}\n"); f.flush()
                        last_rx = time.monotonic()
                else:
                    # No bytes this second. If the board has been silent a
                    # long time while the port is still open, note it — that
                    # is the freeze signature (port alive, core 0 not printing).
                    if last_rx is not None and time.monotonic() - last_rx > 10:
                        mark(f, f"SILENT for {int(time.monotonic()-last_rx)}s (port still open — possible core-0-alive-but-quiet or freeze)")
                        last_rx = time.monotonic()  # rate-limit the notice
        except Exception as e:
            mark(f, f"PORT ERROR: {type(e).__name__}: {e} (device likely frozen/disconnected; retrying)")
            try:
                sp.close()
            except Exception:
                pass
            time.sleep(2)
