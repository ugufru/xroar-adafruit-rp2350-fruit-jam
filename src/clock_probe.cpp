// xroar-adafruit-rp2350-fruit-jam — clock-plan probe (FRUITJAM-03).
//
// Diagnostic build (env:clockprobe). Validates the 252 MHz clock plan from
// docs/clock-plan.md on the actual Fruit Jam silicon:
//   1. set_sys_clock_khz(252000) at vreg 1.25 V,
//   2. mux clk_hstx = clk_sys / 2 = 126 MHz (the HSTX DVI serializer clock),
//   3. measure every clock with the RP2350 frequency counter, and
//   4. run a CRC compute-stress loop to prove the core executes correctly at
//      speed (a too-aggressive clock/vreg shows up as a wrong checksum or a
//      dead serial port).
//
// This does NOT bring up HSTX/DVI — it only configures and *measures* the HSTX
// clock, de-risking FRUITJAM-04's clock assumptions without the display stack.

#include <Arduino.h>
#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "hardware/structs/clocks.h"
#include "pico/stdlib.h"

static constexpr uint32_t SYS_KHZ  = 252000;
static constexpr uint32_t HSTX_KHZ = 126000;   // clk_sys / 2

static bool     g_sysclk_ok = false;
static uint32_t g_crc       = 0;

// A cheap, deterministic compute stress: a rolling CRC32 over a pseudo stream.
// If the core misexecutes at 252 MHz / 1.25 V the result won't match the value
// the same code produces at stock speed.
static uint32_t crc_stress(uint32_t iters) {
    uint32_t crc = 0xFFFFFFFFu;
    uint32_t x = 0x12345678u;
    for (uint32_t i = 0; i < iters; i++) {
        x ^= x << 13; x ^= x >> 17; x ^= x << 5;   // xorshift
        crc ^= x;
        for (int b = 0; b < 8; b++)
            crc = (crc >> 1) ^ (0xEDB88320u & (-(int32_t)(crc & 1)));
    }
    return crc ^ 0xFFFFFFFFu;
}

static void print_clk(const char *name, uint src, uint32_t expect_khz);

// Measure and print every clock in the plan. Called at boot and periodically
// from loop() so the table can be captured even if serial attaches late.
static void report_clocks() {
    Serial.println("Measured clocks (RP2350 frequency counter):");
    print_clk("clk_sys",  CLOCKS_FC0_SRC_VALUE_CLK_SYS,                252000);
    print_clk("clk_hstx", CLOCKS_FC0_SRC_VALUE_CLK_HSTX,               126000);
    print_clk("clk_usb",  CLOCKS_FC0_SRC_VALUE_CLK_USB,                48000);
    print_clk("clk_peri", CLOCKS_FC0_SRC_VALUE_CLK_PERI,               48000);   // SDK default: pll_usb
    print_clk("pll_sys",  CLOCKS_FC0_SRC_VALUE_PLL_SYS_CLKSRC_PRIMARY, 252000);
    print_clk("pll_usb",  CLOCKS_FC0_SRC_VALUE_PLL_USB_CLKSRC_PRIMARY, 48000);
}

static void print_clk(const char *name, uint src, uint32_t expect_khz) {
    uint32_t k = frequency_count_khz(src);
    Serial.print("  ");
    Serial.print(name);
    for (size_t i = strlen(name); i < 10; i++) Serial.print(' ');
    Serial.print(": ");
    Serial.print(k);
    Serial.print(" kHz (expect ");
    Serial.print(expect_khz);
    Serial.print(")");
    // frequency counter is ~1 part in 1000; accept 0.5% tolerance
    uint32_t tol = expect_khz / 200 + 1;
    Serial.println((k + tol >= expect_khz && k <= expect_khz + tol) ? "  OK" : "  <-- MISMATCH");
}

void setup() {
    // 1.25 V first, then raise the clock (vreg must lead the frequency).
    vreg_set_voltage(VREG_VOLTAGE_1_25);
    delay(2);
    g_sysclk_ok = set_sys_clock_khz(SYS_KHZ, false);   // false: don't hang if unachievable

    // Mux clk_hstx off clk_sys with integer divide 2 -> 126 MHz.
    clock_configure(clk_hstx,
                    0,                                             // no glitchless mux on HSTX
                    CLOCKS_CLK_HSTX_CTRL_AUXSRC_VALUE_CLK_SYS,     // source: clk_sys
                    SYS_KHZ * 1000u,                               // source freq
                    HSTX_KHZ * 1000u);                             // target 126 MHz

    Serial.begin(115200);
    const uint32_t start = millis();
    while (!Serial && (millis() - start) < 2000) delay(10);

    Serial.println();
    Serial.println("=== FRUITJAM-03 clock probe (252 MHz plan) ===");
    Serial.print("set_sys_clock_khz(252000) -> ");
    Serial.println(g_sysclk_ok ? "true (achieved)" : "FALSE (not achievable!)");
    Serial.print("vreg: 1.25 V   F_CPU compile-const: ");
    Serial.print(F_CPU / 1000000);
    Serial.println(" MHz");

    report_clocks();

    Serial.print("CRC compute-stress (1,000,000 iters)... ");
    uint32_t t0 = micros();
    g_crc = crc_stress(1000000);
    uint32_t dt = micros() - t0;
    Serial.print("crc=0x");
    Serial.print(g_crc, HEX);
    Serial.print(" in ");
    Serial.print(dt);
    Serial.println(" us");
    Serial.println("If clk_sys=252000 and clk_hstx=126000 read OK, the plan holds.");
}

void loop() {
    static uint32_t n = 0;
    // Re-run the stress each second; a stable crc across iterations = stable silicon.
    uint32_t c = crc_stress(200000);
    Serial.print("tick ");
    Serial.print(n++);
    Serial.print("  stress_crc=0x");
    Serial.print(c, HEX);
    Serial.print("  sysclk_ok=");
    Serial.println(g_sysclk_ok ? 1 : 0);
    if (n % 5 == 0) report_clocks();   // reprint the table every 5s for capture
    delay(1000);
}
