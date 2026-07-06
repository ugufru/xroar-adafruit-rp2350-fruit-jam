// SPDX-License-Identifier: GPL-3.0-or-later
// xroar-adafruit-rp2350-fruit-jam — USB HID keyboard bring-up (FRUITJAM-05).
//
// First light for USB host on this board: a PIO-USB host on D+=GPIO1/D-=GPIO2
// feeding the on-board CH334F hub (both USB-A jacks are hub ports), so TinyUSB
// hub support (CFG_TUH_HUB=1) is mandatory — new territory vs the prior ports.
// GPIO11 gates host 5V. Acceptance: HID boot-keyboard reports arrive with a
// keyboard on either USB-A port; we print the keycodes over serial.
//
// Runs at 252 MHz / 1.25 V (FRUITJAM-03) — the divider 252/48 = 5.25 is exact
// in the PIO 16.8 fractional divider, so PIO-USB enumerates (PIZERO-44). HSTX
// video is hardware here, so both PIO blocks are free for USB (a Fruit Jam
// advantage over the pizero, where libdvi owned a PIO).

#include <Arduino.h>
#include "hardware/vreg.h"
#include "pico/stdlib.h"
#include "pio_usb.h"
#include "Adafruit_TinyUSB.h"

#define HOST_PIN_DP 1     // D+ = GPIO1, D- = GPIO2 (FRUITJAM-01)
#define USB_5V_EN   11    // host 5V gate (polarity per FRUITJAM-01; try HIGH first)

static Adafruit_USBH_Host USBHost;

static volatile uint32_t g_mounts = 0;
static volatile uint32_t g_reports = 0;
static volatile uint32_t g_recv_fail = 0;

// HID boot-keyboard usage -> ASCII, just for a legible serial dump (0x04='a').
static char hid_to_ascii(uint8_t u, bool shift) {
    if (u >= 0x04 && u <= 0x1d) return (shift ? 'A' : 'a') + (u - 0x04);  // a-z
    if (u >= 0x1e && u <= 0x26) return '1' + (u - 0x1e);                  // 1-9
    if (u == 0x27) return '0';
    if (u == 0x2c) return ' ';
    if (u == 0x28) return '\n';
    return 0;
}

void setup() {
    vreg_set_voltage(VREG_VOLTAGE_1_25);
    delay(2);
    set_sys_clock_khz(252000, true);

    // Enable host 5V FIRST so the CH334F hub has time to power up its PHY and
    // settle before we start the host controller — the ~2 s Serial wait below
    // gives it plenty. (Enabling it 10 ms before begin() left the hub not ready,
    // so nothing enumerated.)
    pinMode(USB_5V_EN, OUTPUT);
    digitalWrite(USB_5V_EN, HIGH);      // drive-high-to-enable (FRUITJAM-01)

    Serial.begin(115200);
    const uint32_t start = millis();
    while (!Serial && (millis() - start) < 2000) delay(10);

    Serial.println();
    Serial.println("=== FRUITJAM-05 USB HID keyboard bring-up ===");

    pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
    pio_cfg.pin_dp     = HOST_PIN_DP;
    pio_cfg.pio_tx_num = 0;             // PIO 0 free (HSTX is hardware, not PIO)
    pio_cfg.pio_rx_num = 0;
    USBHost.configure_pio_usb(1, &pio_cfg);

    // Boot protocol: 8-byte keyboard report, how wired boards + 2.4GHz dongles
    // want to be talked to. Must be set before begin().
    tuh_hid_set_default_protocol(HID_PROTOCOL_BOOT);
    USBHost.begin(1);

    Serial.printf("USB host up: PIO-USB D+=%d/D-=%d, CFG_TUH_HUB=%d. "
                  "Plug a keyboard into either USB-A port.\n",
                  HOST_PIN_DP, HOST_PIN_DP + 1, CFG_TUH_HUB);
    Serial.flush();
}

void loop() {
    USBHost.task();

    static uint32_t last = 0;
    if (millis() - last >= 2000) {
        last = millis();
        Serial.printf("[status] mounts=%lu reports=%lu recv_fail=%lu\n",
                      (unsigned long)g_mounts, (unsigned long)g_reports,
                      (unsigned long)g_recv_fail);
    }
}

// ---- TinyUSB host callbacks ------------------------------------------------

extern "C" void tuh_mount_cb(uint8_t daddr) {
    Serial.printf("[usb] device mounted, addr=%u\n", daddr);
}
extern "C" void tuh_umount_cb(uint8_t daddr) {
    Serial.printf("[usb] device unmounted, addr=%u\n", daddr);
}

extern "C" void tuh_hid_mount_cb(uint8_t daddr, uint8_t idx,
                                 uint8_t const *desc_report, uint16_t desc_len) {
    (void)desc_report; (void)desc_len;
    g_mounts++;
    uint8_t proto = tuh_hid_interface_protocol(daddr, idx);
    Serial.printf("[usb] HID mount daddr=%u idx=%u protocol=%u (%s)\n",
                  daddr, idx, proto,
                  proto == HID_ITF_PROTOCOL_KEYBOARD ? "keyboard" :
                  proto == HID_ITF_PROTOCOL_MOUSE ? "mouse" : "other");
    if (!tuh_hid_receive_report(daddr, idx))
        Serial.printf("[usb] tuh_hid_receive_report FAILED idx=%u\n", idx);
}

extern "C" void tuh_hid_umount_cb(uint8_t daddr, uint8_t idx) {
    Serial.printf("[usb] HID unmount daddr=%u idx=%u\n", daddr, idx);
}

extern "C" void tuh_hid_report_received_cb(uint8_t daddr, uint8_t idx,
                                           uint8_t const *report, uint16_t len) {
    g_reports++;

    // Log EVERY report raw so any input is visible, whatever the layout.
    Serial.printf("[rpt] idx=%u len=%u:", idx, len);
    for (uint16_t i = 0; i < len && i < 12; i++) Serial.printf(" %02X", report[i]);

    // Boot-keyboard decode. Some keyboards prepend a report-ID byte, so accept
    // either an 8-byte report ([mods][resv][k0..k5]) or 9-byte ([id][8]).
    const uint8_t *kb = (len == 9) ? report + 1 : report;
    if (len == 8 || len == 9) {
        uint8_t mods = kb[0];
        bool shift = mods & 0x22;
        Serial.print("   keys:");
        for (int i = 2; i < 8; i++) {
            uint8_t u = kb[i];
            if (!u) continue;
            char c = hid_to_ascii(u, shift);
            if (c == '\n') Serial.printf(" 0x%02X(\\n)", u);
            else if (c)    Serial.printf(" 0x%02X(%c)", u, c);
            else           Serial.printf(" 0x%02X", u);
        }
    }
    Serial.println();

    if (!tuh_hid_receive_report(daddr, idx)) g_recv_fail++;   // re-arm
}
