// SPDX-License-Identifier: GPL-3.0-or-later
// xroar-adafruit-rp2350-fruit-jam — TLV320DAC3100 audio bring-up (FRUITJAM-07).
//
// First audio on this board: bring up the TLV320DAC3100 over I2C0 and stream a
// clean 48 kHz sine over I2S, out the 3.5mm headphone jack (and speaker). No
// prior port had a real DAC (the pizero used HDMI data-island audio), so this
// is new territory — driven by Adafruit's TLV320 library.
//
// Pins (FRUITJAM-01): I2C0 SDA=20/SCL=21; I2S DATA=24, BCLK=26, LRCLK=27
// (=BCLK+1), MCLK unused (the DAC PLL derives its clock from BCLK). Reset=GPIO22
// is SHARED with the ESP32-C6 — pulsing it also resets the C6, which is out of
// scope for the port, so we just pulse once at boot before configuring the DAC.
//
// Clocking (from Adafruit's basicI2Sconfig): PLL from BCLK, P=1 R=2 J=32 D=0,
// NDAC=8 MDAC=2 -> with BCLK=48000*16*2=1.536 MHz the DAC runs at exactly 48 kHz.
// The RP2350 I2S master produces that BCLK, so no MCLK is needed.

#include <Arduino.h>
#include <Wire.h>
#include <I2S.h>
#include <math.h>
#include <Adafruit_TLV320DAC3100.h>

#include "hardware/vreg.h"
#include "pico/stdlib.h"

#define PIN_I2C_SDA   20
#define PIN_I2C_SCL   21
#define PIN_I2S_DATA  24
#define PIN_I2S_BCLK  26   // LRCLK is BCLK+1 = 27
#define PIN_PERIPH_RST 22  // shared DAC + ESP32-C6 reset

static Adafruit_TLV320DAC3100 codec;
static I2S i2s(OUTPUT);

// 256-entry signed 16-bit sine, ~1/3 full scale to leave headroom.
static int16_t g_sine[256];
static void build_sine() {
    for (int i = 0; i < 256; i++)
        g_sine[i] = (int16_t)lroundf(10000.0f * sinf(2.0f * (float)M_PI * i / 256.0f));
}

// 440 Hz tone at 48 kHz: phase step = 440/48000 * 256 in 16.16 fixed point.
static const uint32_t PHASE_STEP = (uint32_t)((440.0 / 48000.0) * 256.0 * 65536.0);

static bool configure_codec() {
    if (!codec.begin()) { Serial.println("codec.begin() FAILED"); return false; }
    delay(10);
    bool ok = true;
    ok &= codec.setCodecInterface(TLV320DAC3100_FORMAT_I2S, TLV320DAC3100_DATA_LEN_16);
    ok &= codec.setCodecClockInput(TLV320DAC3100_CODEC_CLKIN_PLL);
    ok &= codec.setPLLClockInput(TLV320DAC3100_PLL_CLKIN_BCLK);
    ok &= codec.setPLLValues(1, 2, 32, 0);       // P=1 R=2 J=32 D=0
    ok &= codec.setNDAC(true, 8);
    ok &= codec.setMDAC(true, 2);
    ok &= codec.powerPLL(true);
    ok &= codec.setDACDataPath(true, true, TLV320_DAC_PATH_NORMAL,
                               TLV320_DAC_PATH_NORMAL, TLV320_VOLUME_STEP_1SAMPLE);
    ok &= codec.configureAnalogInputs(TLV320_DAC_ROUTE_MIXER, TLV320_DAC_ROUTE_MIXER,
                                      false, false, false, false);
    ok &= codec.setDACVolumeControl(false, false, TLV320_VOL_INDEPENDENT);
    ok &= codec.setChannelVolume(false, 18);     // left  +0 dB
    ok &= codec.setChannelVolume(true, 18);      // right +0 dB
    // Headphone driver
    ok &= codec.configureHeadphoneDriver(true, true, TLV320_HP_COMMON_1_35V, false);
    ok &= codec.configureHPL_PGA(0, true);
    ok &= codec.configureHPR_PGA(0, true);
    ok &= codec.setHPLVolume(true, 6);
    ok &= codec.setHPRVolume(true, 6);
    // Speaker (mono class-D)
    ok &= codec.enableSpeaker(true);
    ok &= codec.configureSPK_PGA(TLV320_SPK_GAIN_6DB, true);
    ok &= codec.setSPKVolume(true, 0);
    return ok;
}

void setup() {
    vreg_set_voltage(VREG_VOLTAGE_1_25);
    delay(2);
    set_sys_clock_khz(252000, true);

    Serial.begin(115200);
    const uint32_t start = millis();
    while (!Serial && (millis() - start) < 2000) delay(10);
    Serial.println();
    Serial.println("=== FRUITJAM-07 TLV320DAC3100 audio bring-up ===");

    build_sine();

    // Pulse the shared reset (also resets the ESP32-C6, which we don't use).
    pinMode(PIN_PERIPH_RST, OUTPUT);
    digitalWrite(PIN_PERIPH_RST, LOW);  delay(100);
    digitalWrite(PIN_PERIPH_RST, HIGH); delay(100);

    // I2C0 for codec control.
    Wire.setSDA(PIN_I2C_SDA);
    Wire.setSCL(PIN_I2C_SCL);
    Wire.begin();

    // Start I2S master FIRST so BCLK is live before the DAC PLL tries to lock.
    i2s.setDATA(PIN_I2S_DATA);
    i2s.setBCLK(PIN_I2S_BCLK);          // LRCLK = 27
    i2s.setBitsPerSample(16);
    i2s.setFrequency(48000);
    if (!i2s.begin()) { Serial.println("I2S begin FAILED"); }
    else              { Serial.println("I2S master up (48 kHz, BCLK=26, LRCLK=27, DIN=24)."); }

    if (configure_codec()) Serial.println("TLV320 configured — 440 Hz tone on HP + speaker.");
    else                   Serial.println("TLV320 config FAILED — check I2C (addr 0x18) on SDA20/SCL21.");
}

void loop() {
    static uint32_t phase = 0;
    // Keep the I2S DMA fed; write() blocks when the buffer is full, pacing us.
    for (int n = 0; n < 256; n++) {
        int16_t s = g_sine[(phase >> 16) & 0xFF];
        phase += PHASE_STEP;
        i2s.write16(s, s);   // same sample L+R
    }
}
