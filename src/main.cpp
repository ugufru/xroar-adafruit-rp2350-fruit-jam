// SPDX-License-Identifier: GPL-3.0-or-later
// xroar-adafruit-rp2350-fruit-jam — bring-up scaffold (FRUITJAM-02).
//
// The very first thing that runs on the board: no XRoar yet. It proves the
// toolchain, the deploy flow, and that we can talk to the hardware at all —
//   1. USB-serial "hello" so we know the board booted and enumerated, and
//   2. a blinking LED so a headless board still shows a sign of life.
//
// Once this blinks and prints over serial, FRUITJAM-02's acceptance is met and
// the board is a known-good platform to build the display/USB/SD bring-up on.
//
// LED: the Fruit Jam's only discrete LED is the red one on GPIO29, and it is
// ACTIVE-LOW (drive the pin LOW to light it). GPIO29 is shared with the IR
// receiver, which is out of scope for this port (docs/hardware-pinout.md), so
// using it as a heartbeat here is fine.

#include <Arduino.h>

// Verified against docs/hardware-pinout.md (FRUITJAM-01).
static constexpr uint8_t LED_PIN        = 29;    // red LED, shared with IR
static constexpr bool     LED_ACTIVE_LOW = true;

static inline void led_write(bool on) {
    digitalWrite(LED_PIN, (on ^ LED_ACTIVE_LOW) ? HIGH : LOW);
}

void setup() {
    pinMode(LED_PIN, OUTPUT);
    led_write(false);

    Serial.begin(115200);
    // Don't block forever on a missing host: give the USB CDC a moment, then go.
    const uint32_t start = millis();
    while (!Serial && (millis() - start) < 2000) {
        delay(10);
    }

    Serial.println();
    Serial.println("=== XRoar on Adafruit Fruit Jam (RP2350B) ===");
    Serial.println("bring-up scaffold: FRUITJAM-02 blink + serial hello");
    Serial.print("CPU clock: ");
    Serial.print(F_CPU / 1000000);
    Serial.println(" MHz");
    Serial.println("If you can read this and the LED is blinking, the board is good.");
}

void loop() {
    static uint32_t beats = 0;
    led_write(true);
    delay(100);
    led_write(false);
    delay(900);

    // A once-a-second serial tick doubles as a liveness signal on the monitor.
    Serial.print("tick ");
    Serial.println(beats++);
}
