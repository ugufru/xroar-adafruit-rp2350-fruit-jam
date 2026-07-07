// SPDX-License-Identifier: GPL-3.0-or-later
// xroar-adafruit-rp2350-fruit-jam — Boot Color BASIC to the monitor (FRUITJAM-25).
//
// The integration milestone: the hand-written CoCo machine (FRUITJAM-22) runs
// Color BASIC loaded from SD (FRUITJAM-06); its frame-batched VDG renderer
// (FRUITJAM-24) produces a 256x192 indexed frame; we map that through the CoCo
// palette into the 320x240 RGB565 framebuffer (FRUITJAM-04) and the HSTX engine
// scans it out over DVI at 640x480p60.
//
// Core split (previews FRUITJAM-11): core 0 runs emulation + composes the
// framebuffer; core 1 runs the pico_hdmi video engine (near-zero CPU). The
// framebuffer is a single buffer for this milestone — occasional tearing is
// acceptable; a front/back pointer swap is FRUITJAM-11.
//
// 252 MHz / 1.25 V operating point (FRUITJAM-03); pico_hdmi built with
// MODE_HSTX_CLK_DIV=2 -> clk_hstx = 126 MHz.

#include <Arduino.h>
#include <string.h>
#include <Wire.h>
#include <I2S.h>
#include <Adafruit_TLV320DAC3100.h>
#include "pico/multicore.h"
#include "hardware/vreg.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/structs/bus_ctrl.h"
#include "pico/stdlib.h"
#include "pio_usb.h"
#include "Adafruit_TinyUSB.h"

extern "C" {
#include "pico_hdmi/video_output.h"
#include "ff.h"
#include "f_util.h"
#include "coco_machine.h"
#include "dkbd.h"          // DSCAN_* CoCo keyboard scancodes
}

// Public in pico_hdmi's video_output.c but not its header: full HSTX+DMA restart
// to recover from a desynced command stream (FIFO underrun -> permanent loss of
// lock). Used by the desync watchdog below.
extern "C" void video_output_force_resync(void);

#include "framebuffer.h"    // dvi::Framebuffer, dvi::rgb565, FB_WIDTH/HEIGHT

// Scanline callback runs from the core-1 DMA IRQ — keep it out of flash.
#define RAM_FUNC __attribute__((section(".time_critical.coco_main")))

// CoCo VDG palette -> RGB565. Index order is the contract with coco_machine's
// PAL_* constants (FRUITJAM-24): 0 GREEN .. 9 DARK_GREEN.
// NOT const: scanline_cb reads this per-pixel from the core-1 DMA IRQ, so it
// must live in RAM (.data), never flash — an XIP stall there starves the HSTX
// FIFO (FRUITJAM-04 rule). A const array would sit in .rodata/flash.
static uint16_t g_pal[16] = {
    dvi::rgb565(0x30, 0xD2, 0x10),  // 0 GREEN (the CoCo screen green)
    dvi::rgb565(0xF0, 0xE0, 0x30),  // 1 YELLOW
    dvi::rgb565(0x20, 0x30, 0xF0),  // 2 BLUE
    dvi::rgb565(0xC0, 0x18, 0x10),  // 3 RED
    dvi::rgb565(0xF0, 0xF0, 0xE0),  // 4 WHITE / buff
    dvi::rgb565(0x20, 0xC0, 0xC0),  // 5 CYAN
    dvi::rgb565(0xC0, 0x20, 0xC0),  // 6 MAGENTA
    dvi::rgb565(0xE0, 0x80, 0x10),  // 7 ORANGE
    dvi::rgb565(0x00, 0x00, 0x00),  // 8 BLACK
    dvi::rgb565(0x00, 0x38, 0x00),  // 9 DARK GREEN
    0, 0, 0, 0, 0, 0
};

// Single RGB565 framebuffer, matching the proven-stable FRUITJAM-04 display_test.
// The palette lookup happens here in the blit (core 0), so scanline_cb stays a
// plain copy — a per-pixel palette lookup in the callback overran the HSTX line
// budget and desynced the link (black after a few seconds). RGB565 double-
// buffering would be tear-free but two 150 KB buffers overflow SRAM; the
// residual blit/scanout tearing is a FRUITJAM-11/23 refinement.
static dvi::Framebuffer g_fb;
static uint8_t          g_rom[16384];

// pico_hdmi 2.0-beta precomposed/native scanout ring (README "Minimal pattern").
// Each entry precomposes one active-line header off the ISR so the per-scanline
// work is ~1.5us (patch a data island) instead of a 320-px fill — the path built
// for "320->640 scaling while HDMI runs" (emulators). ~312 B/entry.
static video_output_precomposed_line_t g_compose_ring[48];

// Authentic NTSC CoCo: ~0.895 MHz / 60 Hz = 14915 6809 cycles per field.
static const uint32_t CYCLES_PER_FRAME = 14915;
static const uint32_t FRAME_US         = 16762;   // 60 Hz field period

// CoCo 256x192 active area centered in the 320x240 framebuffer.
static const int OX = (dvi::FB_WIDTH  - COCO_VDG_W) / 2;   // 32
static const int OY = (dvi::FB_HEIGHT - COCO_VDG_H) / 2;   // 24

// Scanline POINTER callback (native pixel mode): return the address of the
// framebuffer row and let the DMA read it directly, doubling each pixel in
// hardware. The per-scanline IRQ does essentially no work — no per-line fill to
// fall behind under core-0 emulation load, which is what was underrunning the
// HSTX FIFO and desyncing the link. Vertical 2x via active_line >> 1.
static const uint32_t *RAM_FUNC scanline_ptr_cb(uint32_t v_scanline, uint32_t active_line) {
    (void)v_scanline;
    return (const uint32_t *)g_fb.px[active_line >> 1];
}

// Compose the CoCo frame (nibble-packed indices) into g_fb as RGB565, centered.
// The palette lookup lives here (core 0), keeping scanline_cb a plain copy.
static void blit_frame() {
    const uint8_t *vb = coco_machine_get_vdg_buffer();
    for (int y = 0; y < COCO_VDG_H; y++) {
        const uint8_t *src = &vb[y * (COCO_VDG_W / 2)];
        uint16_t *dst = &g_fb.px[OY + y][OX];
        for (int x = 0; x < COCO_VDG_W; x++) {
            uint8_t idx = (x & 1) ? (src[x >> 1] >> 4) : (src[x >> 1] & 0x0F);
            dst[x] = g_pal[idx];
        }
    }
}

static FATFS g_fs;
static bool mount_sd() {
    for (int a = 0; a < 5; a++) {
        if (f_mount(&g_fs, "0:", 1) == FR_OK) return true;
        delay(200);
    }
    return false;
}
static size_t load_rom(const char *path) {
    FIL f;
    if (f_open(&f, path, FA_READ) != FR_OK) return 0;
    UINT br = 0;
    f_read(&f, g_rom, sizeof(g_rom), &br);
    f_close(&f);
    return br;
}

// Cassette image (FRUITJAM-28): loaded from SD into PSRAM (the cold/bulk home,
// FRUITJAM-08 policy) and handed to the machine's tape feeder. PSRAM is retuned
// for 252 MHz in setup() (psram_reinit_timing) before this runs.
extern size_t __psram_size;
void  *__psram_malloc(size_t);
void   __psram_free(void *);
void   psram_reinit_timing(uint32_t hz);

static const uint8_t *g_cas_img = nullptr;
static size_t load_cas(const char *path) {
    FIL f;
    if (f_open(&f, path, FA_READ) != FR_OK) return 0;
    size_t sz = f_size(&f);
    if (sz == 0) { f_close(&f); return 0; }
    uint8_t *buf = (uint8_t *)__psram_malloc(sz);
    if (!buf) { f_close(&f); return 0; }
    UINT br = 0;
    f_read(&f, buf, sz, &br);
    f_close(&f);
    if (br != sz) { __psram_free(buf); return 0; }
    g_cas_img = buf;
    return sz;
}

// - - - CoCo audio -> TLV320 DAC over I2S (FRUITJAM-13) ------------------------
// The machine produces 48 kHz mono PCM from its PIA sound tap (coco_machine
// render_audio); this side is the sink: the TLV320DAC3100 configured over I2C0
// exactly as the FRUITJAM-07 bring-up, fed by the earlephilhower PIO-I2S master.
//
// Producer (this core, once per field) and the I2S DMA (hardware) share the I2S
// library's ring: we write each field's samples non-blocking, then top the ring
// back up to its half mark with the last sample (hold-last) so a late field
// can't underrun into a click. The 2048-frame ring (~43 ms) spans >2 fields, so
// the once-per-field feed keeps the DMA fed through the pacing delay.
#define PIN_I2C_SDA    20
#define PIN_I2S_DATA   24
#define PIN_I2S_BCLK   26    // LRCLK = BCLK+1 = 27
#define PIN_PERIPH_RST 22    // shared TLV320 + ESP32-C6 reset (pulse once)

// 0 = headphone jack only (feeding an external amp); 1 = onboard class-D speaker.
#define SPEAKER_OUTPUT 0

static Adafruit_TLV320DAC3100 g_codec;
static I2S                    g_i2s(OUTPUT);

// Ring geometry: setBuffers(16, 128) = 2048 stereo frames = 8192 bytes. Keep the
// ring ~half full: pad up to HALF_BYTES free with hold-last, bounding latency to
// ~21 ms while leaving room for the next field's samples.
static const int I2S_HALF_BYTES = 4096;

static int16_t g_audio_buf[1024];   // one field is ~805 samples
static int16_t g_audio_last = 0;    // hold-last value on underrun
static int     g_audio_peak = 0;    // loudest |sample| since last report (level tuning)

static inline void i2s_put(int16_t s) {
    g_audio_last = s;
    g_i2s.write((int32_t)(((uint32_t)(uint16_t)s << 16) | (uint16_t)s), false);  // L=R, non-blocking
}

static bool configure_codec() {
    if (!g_codec.begin()) { Serial.println("codec.begin() FAILED"); return false; }
    delay(10);
    bool ok = true;
    ok &= g_codec.setCodecInterface(TLV320DAC3100_FORMAT_I2S, TLV320DAC3100_DATA_LEN_16);
    ok &= g_codec.setCodecClockInput(TLV320DAC3100_CODEC_CLKIN_PLL);
    ok &= g_codec.setPLLClockInput(TLV320DAC3100_PLL_CLKIN_BCLK);
    ok &= g_codec.setPLLValues(1, 2, 32, 0);       // P=1 R=2 J=32 D=0
    ok &= g_codec.setNDAC(true, 8);
    ok &= g_codec.setMDAC(true, 2);
    ok &= g_codec.powerPLL(true);
    ok &= g_codec.setDACDataPath(true, true, TLV320_DAC_PATH_NORMAL,
                                 TLV320_DAC_PATH_NORMAL, TLV320_VOLUME_STEP_1SAMPLE);
    ok &= g_codec.configureAnalogInputs(TLV320_DAC_ROUTE_MIXER, TLV320_DAC_ROUTE_MIXER,
                                        false, false, false, false);
    ok &= g_codec.setDACVolumeControl(false, false, TLV320_VOL_INDEPENDENT);
    ok &= g_codec.setChannelVolume(false, 18);     // left  +0 dB
    ok &= g_codec.setChannelVolume(true, 18);      // right +0 dB
    ok &= g_codec.configureHeadphoneDriver(true, true, TLV320_HP_COMMON_1_35V, false);
    ok &= g_codec.configureHPL_PGA(0, true);
    ok &= g_codec.configureHPR_PGA(0, true);
    ok &= g_codec.setHPLVolume(true, 6);
    ok &= g_codec.setHPRVolume(true, 6);
    // Internal speaker DISABLED: output goes to the 3.5mm headphone jack only
    // (feeding an external amp). Set SPEAKER_OUTPUT to 1 to use the onboard
    // speaker instead.
    //
    // Auto-mute-on-insert via the TLV320's headset detection (setHeadsetDetect /
    // getHeadsetStatus, IRQ on GPIO23) was tried and did NOT work for a line
    // cable into an amp: the detector is impedance-based and reads a high-Z amp
    // input as "nothing plugged". Revisit only if real (low-Z) headphone use is
    // wanted AND it's confirmed the Fruit Jam wires the detection usefully.
    ok &= g_codec.enableSpeaker(SPEAKER_OUTPUT);
    if (SPEAKER_OUTPUT) {
        ok &= g_codec.configureSPK_PGA(TLV320_SPK_GAIN_6DB, true);
        ok &= g_codec.setSPKVolume(true, 0);
    }
    return ok;
}

// Bring up the TLV320 + I2S master and prime the ring with silence. Mirrors the
// FRUITJAM-07 sequencing: pulse the shared reset, I2C config, I2S master up with
// BCLK live *before* the DAC PLL tries to lock.
static bool audio_init() {
    pinMode(PIN_PERIPH_RST, OUTPUT);
    digitalWrite(PIN_PERIPH_RST, LOW);  delay(100);
    digitalWrite(PIN_PERIPH_RST, HIGH); delay(100);

    Wire.setSDA(PIN_I2C_SDA);
    Wire.setSCL(PIN_I2C_SDA + 1);       // SCL = 21
    Wire.begin();

    g_i2s.setDATA(PIN_I2S_DATA);
    g_i2s.setBCLK(PIN_I2S_BCLK);        // LRCLK = 27
    g_i2s.setBitsPerSample(16);
    g_i2s.setFrequency(48000);
    g_i2s.setBuffers(16, 128);          // 2048 frames ring
    if (!g_i2s.begin()) { Serial.println("I2S begin FAILED"); return false; }

    // Prime to the half mark with silence so the DMA has a cushion before the
    // first field's samples arrive.
    while (g_i2s.availableForWrite() > I2S_HALF_BYTES) i2s_put(0);

    return configure_codec();
}

// Per-field feed: push this field's PCM, then hold-last-pad back to the half mark.
static void audio_feed() {
    int n = coco_machine_render_audio(g_audio_buf, (int)(sizeof(g_audio_buf) / sizeof(g_audio_buf[0])));
    int field_peak = 0;
    for (int i = 0; i < n; i++) {
        int a = g_audio_buf[i] < 0 ? -g_audio_buf[i] : g_audio_buf[i];
        if (a > field_peak) field_peak = a;
        if (a > g_audio_peak) g_audio_peak = a;
        if (g_i2s.availableForWrite() < 4) break;   // ring full (overrun) — drop rest
        i2s_put(g_audio_buf[i]);
    }
    (void)field_peak;
    // Refill toward half with the last sample so a starved DMA repeats rather
    // than clicking to silence (hold-last on underrun).
    while (g_i2s.availableForWrite() > I2S_HALF_BYTES) i2s_put(g_audio_last);
}

// - - - USB HID keyboard -> CoCo matrix (FRUITJAM-12) --------------------------
// PIO-USB host behind the CH334F hub (proven in FRUITJAM-05). Reports are diffed
// per frame and driven into the machine's PIA0 matrix via coco_machine_press/
// release_key. Implemented fresh; the mapping knowledge is ported from pizero.
#define HOST_PIN_DP 1
#define USB_5V_EN   11

static Adafruit_USBH_Host USBHost;

// HID boot-keyboard usage code -> CoCo DSCAN. 0xFF = no mapping.
static uint8_t g_hid_to_dscan[256];

static void hid_map_init(void) {
    memset(g_hid_to_dscan, 0xFF, sizeof(g_hid_to_dscan));
    for (int i = 0; i < 26; i++) g_hid_to_dscan[0x04 + i] = (uint8_t)(DSCAN_A + i);  // a-z
    static const uint8_t digit[10] = {              // HID 0x1E..0x27 = 1..9,0
        DSCAN_1, DSCAN_2, DSCAN_3, DSCAN_4, DSCAN_5,
        DSCAN_6, DSCAN_7, DSCAN_8, DSCAN_9, DSCAN_0 };
    for (int i = 0; i < 10; i++) g_hid_to_dscan[0x1E + i] = digit[i];
    g_hid_to_dscan[0x28] = DSCAN_ENTER;     // Enter
    g_hid_to_dscan[0x29] = DSCAN_BREAK;     // Esc  -> BREAK
    g_hid_to_dscan[0x2A] = DSCAN_LEFT;      // Backspace -> Left (CoCo rubout)
    g_hid_to_dscan[0x2B] = DSCAN_RIGHT;     // Tab -> Right
    g_hid_to_dscan[0x2C] = DSCAN_SPACE;     // Space
    g_hid_to_dscan[0x2D] = DSCAN_MINUS;     // -
    g_hid_to_dscan[0x33] = DSCAN_SEMICOLON; // ;
    g_hid_to_dscan[0x34] = DSCAN_COLON;     // ' -> : (CoCo has a dedicated colon)
    g_hid_to_dscan[0x36] = DSCAN_COMMA;     // ,
    g_hid_to_dscan[0x37] = DSCAN_FULL_STOP; // .
    g_hid_to_dscan[0x38] = DSCAN_SLASH;     // /
    g_hid_to_dscan[0x2F] = DSCAN_AT;        // [ -> @ (CoCo @ key)
    g_hid_to_dscan[0x4C] = DSCAN_CLEAR;     // Delete -> CLEAR
    g_hid_to_dscan[0x4F] = DSCAN_RIGHT;     // arrows
    g_hid_to_dscan[0x50] = DSCAN_LEFT;
    g_hid_to_dscan[0x51] = DSCAN_DOWN;
    g_hid_to_dscan[0x52] = DSCAN_UP;
    g_hid_to_dscan[0x29] = DSCAN_BREAK;
}

static uint8_t g_prev_codes[6] = {0};
static bool    g_shift_prev = false;

// HID modifier bits: L/R Ctrl 0x01/0x10, L/R Alt 0x04/0x40, L/R Shift 0x02/0x20.
static void hid_keyboard_apply(const uint8_t *report) {
    uint8_t mods = report[0];
    const uint8_t *codes = &report[2];

    // Reset chord: Ctrl+Alt+Delete (HID 0x4C) -> warm reset. Checked before the
    // matrix diff so the chord itself never leaks keys into BASIC.
    bool ctrl = mods & 0x11, alt = mods & 0x44;
    for (int i = 0; i < 6; i++) {
        if (codes[i] == 0x4C && ctrl && alt) {
            coco_machine_reset();
            g_shift_prev = false; memset(g_prev_codes, 0, sizeof(g_prev_codes));
            return;
        }
    }

    bool shift = (mods & 0x22) != 0;
    if (shift && !g_shift_prev) coco_machine_press_key(DSCAN_SHIFT);
    if (!shift && g_shift_prev) coco_machine_release_key(DSCAN_SHIFT);
    g_shift_prev = shift;

    // Releases: was in prev, not in current.
    for (int i = 0; i < 6; i++) {
        uint8_t p = g_prev_codes[i];
        if (!p) continue;
        bool still = false;
        for (int j = 0; j < 6; j++) if (codes[j] == p) { still = true; break; }
        if (!still && g_hid_to_dscan[p] != 0xFF) coco_machine_release_key(g_hid_to_dscan[p]);
    }
    // Presses: in current, not in prev.
    for (int i = 0; i < 6; i++) {
        uint8_t c = codes[i];
        if (!c) continue;
        bool was = false;
        for (int j = 0; j < 6; j++) if (g_prev_codes[j] == c) { was = true; break; }
        if (!was && g_hid_to_dscan[c] != 0xFF) coco_machine_press_key(g_hid_to_dscan[c]);
    }
    memcpy(g_prev_codes, codes, 6);
}

extern "C" void tuh_hid_mount_cb(uint8_t daddr, uint8_t idx,
                                 uint8_t const *desc, uint16_t len) {
    (void)desc; (void)len;
    tuh_hid_receive_report(daddr, idx);
}
extern "C" void tuh_hid_umount_cb(uint8_t daddr, uint8_t idx) {
    (void)daddr; (void)idx;
    coco_machine_release_all_keys();     // drop stuck keys if the keyboard leaves
    memset(g_prev_codes, 0, sizeof(g_prev_codes));
    g_shift_prev = false;
}
extern "C" void tuh_hid_report_received_cb(uint8_t daddr, uint8_t idx,
                                           uint8_t const *report, uint16_t len) {
    // Only the boot-keyboard interface (8-byte reports). Others (consumer/etc.)
    // are ignored but must still be re-armed.
    if (tuh_hid_interface_protocol(daddr, idx) == HID_ITF_PROTOCOL_KEYBOARD && len >= 8)
        hid_keyboard_apply(report);
    tuh_hid_receive_report(daddr, idx);  // re-arm
}

// Deferred HSTX resync (freeze fix). The desync watchdog runs on core 0, but the
// video DMA-IRQ handler and all HSTX/DMA state live on core 1. Calling
// video_output_force_resync() from core 0 tears that state down while core 1's
// IRQ is still firing into it — and its irq_set_enabled(DMA_IRQ_1) only gates
// core 0's NVIC, not core 1's — a cross-core race that hard-locks the board.
// Instead core 0 just requests a resync; core 1 performs it from its background
// task, where disabling DMA_IRQ_1 actually holds off the local handler.
static volatile bool     g_want_resync  = false;
static volatile uint32_t g_resync_count = 0;

static void core1_background(void) {
    video_output_compose_service();
    if (g_want_resync) {
        g_want_resync = false;
        video_output_force_resync();   // safe here: runs on core 1, gates its own IRQ
        g_resync_count++;
    }
}

// Core 1: the pico_hdmi video engine (never returns). Launched manually from
// setup() after USB host init — see the multicore_launch_core1 note there.
static void core1_video_entry() {
    video_output_core1_run();
}

void setup() {
    vreg_set_voltage(VREG_VOLTAGE_1_25);
    delay(2);
    set_sys_clock_khz(252000, true);
    psram_reinit_timing(clock_get_hz(clk_sys));   // retune QMI PSRAM for 252 MHz (FRUITJAM-08)

    // Host 5V on early so the CH334F hub PHY settles before the host starts.
    pinMode(USB_5V_EN, OUTPUT);
    digitalWrite(USB_5V_EN, HIGH);

    Serial.begin(115200);
    const uint32_t start = millis();
    while (!Serial && (millis() - start) < 1500) delay(10);

    Serial.println();
    Serial.println("=== FRUITJAM-25 Boot Color BASIC -> DVI ===");
    Serial.flush(); delay(50);

    // DMA arbitration: pico_hdmi hardcodes DMA channels 0 and 1, but the SD
    // driver grabs channels via dma_claim_unused_channel() at mount and would
    // take 0/1 first, making video_output_init()'s dma_channel_claim(0/1) panic
    // (silent hang). Reserve 0/1 now so SD skips them, then release them just
    // before video init so pico_hdmi can claim them as it expects.
    dma_channel_claim(0);
    dma_channel_claim(1);

    // USB host FIRST — before SD/video. In the standalone test begin() worked
    // when it ran on a clean machine; in the combined build it hung, because SD
    // or pico_hdmi init first grabs a hardware resource PIO-USB needs in begin()
    // (a hardware alarm for alarm_pool_create). Init it up front so it gets that
    // resource. DMA 0/1 are reserved above for pico_hdmi, so PIO-USB's
    // dma_claim_unused takes 2/3; SD then takes 4/5; video reclaims 0/1.
    Serial.print("STAGE usb host... "); Serial.flush(); delay(20);
    hid_map_init();
    pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
    pio_cfg.pin_dp     = HOST_PIN_DP;
    pio_cfg.pio_tx_num = 0;
    pio_cfg.pio_rx_num = 0;
    // PIO-USB's begin() does dma_claim_mask(1 << tx_ch); its default tx_ch is 0,
    // which collides with pico_hdmi's hardcoded DMA 0 (reserved above) and hard-
    // asserts (silent hang). Point it at a free channel: pico_hdmi=0/1, SD=2/3,
    // so USB TX = 4.
    pio_cfg.tx_ch = 4;
    USBHost.configure_pio_usb(1, &pio_cfg);
    tuh_hid_set_default_protocol(HID_PROTOCOL_BOOT);
    USBHost.begin(1);
    Serial.println("ok"); Serial.flush(); delay(20);

    Serial.print("STAGE mount SD... "); Serial.flush(); delay(20);
    if (!mount_sd()) { Serial.println("FATAL: no SD"); while (1) delay(500); }
    Serial.println("ok"); Serial.flush(); delay(20);

    Serial.print("STAGE load ROM... "); Serial.flush(); delay(20);
    size_t got = load_rom("0:/coco/roms/bas12.rom");
    if (got != 8192 && got != 16384) {
        Serial.printf("FATAL: bad ROM size %u\n", (unsigned)got);
        while (1) delay(500);
    }
    Serial.printf("ok (%u bytes)\n", (unsigned)got); Serial.flush(); delay(20);

    Serial.print("STAGE machine init... "); Serial.flush(); delay(20);
    if (!coco_machine_init(g_rom, got)) { Serial.println("FATAL: machine init"); while (1) delay(500); }
    // Optional cassette: drop a .cas at 0:/coco/tapes/AUTO.CAS, then CLOAD in BASIC.
    {
        size_t cas = load_cas("0:/coco/tapes/AUTO.CAS");
        if (cas) { coco_machine_cas_load(g_cas_img, cas);
                   Serial.printf("[cassette AUTO.CAS loaded: %u bytes -> CLOAD to run] ", (unsigned)cas); }
    }
    // Match the resampler cadence to our real-time pacing so the I2S ring neither
    // starves nor overflows (see coco_machine_audio_init).
    coco_machine_audio_init(CYCLES_PER_FRAME, FRAME_US);
    Serial.println("ok"); Serial.flush(); delay(20);

    Serial.print("STAGE audio (TLV320+I2S)... "); Serial.flush(); delay(20);
    if (audio_init()) Serial.println("ok");
    else              Serial.println("FAILED (continuing, video only)");
    Serial.flush(); delay(20);

    Serial.print("STAGE fill fb... "); Serial.flush(); delay(20);
    g_fb.fill(g_pal[8]);   // black border
    Serial.println("ok"); Serial.flush(); delay(20);

    Serial.print("STAGE video init... "); Serial.flush(); delay(20);
    dma_channel_unclaim(0);   // hand 0/1 back for pico_hdmi to claim (SD now on 2/3)
    dma_channel_unclaim(1);
    // pico_hdmi 2.0-beta native/precomposed path (README minimal pattern): init
    // at the NATIVE 320x240 source size; HDMI mode (not DVI — compose_service is
    // a no-op in DVI); compose ring + native pixel mode + a pointer callback that
    // returns the 320-wide RGB565 row (HSTX doubles to 640 in hardware); and the
    // compose service as core-1 background work so per-line ISR cost stays ~1.5us
    // and can't be starved by core-0 emulation load.
    video_output_init(dvi::FB_WIDTH, dvi::FB_HEIGHT);   // 320x240 native source
    video_output_set_dvi_mode(false);                   // HDMI mode (data islands) — required for compose
    video_output_set_compose_ring(g_compose_ring, sizeof(g_compose_ring) / sizeof(g_compose_ring[0]));
    video_output_set_native_pixel_mode(true);
    video_output_set_scanline_pointer_callback(scanline_ptr_cb);
    video_output_set_background_task(core1_background);
    Serial.println("ok"); Serial.flush(); delay(20);

    Serial.printf("clk_sys=%lu clk_hstx=%lu MHz. Starting video on core 1.\n",
                  (unsigned long)(clock_get_hz(clk_sys) / 1000000),
                  (unsigned long)(clock_get_hz(clk_hstx) / 1000000));
    Serial.flush(); delay(20);

    // Launch core 1 LAST — after USB host init on core 0 — so PIO-USB's begin()
    // cross-core setup can't deadlock against a core 1 already running the video
    // DMA-IRQ loop (the pizero ordering). We drive core 1 manually rather than
    // via setup1(), which the framework would auto-launch too early.
    multicore_launch_core1(core1_video_entry);
    Serial.println("STAGE core1 launched — entering loop."); Serial.flush(); delay(20);
}

// Core 0: emulate one field, compose, pace to ~60 Hz (resync-not-debt).
void loop() {
    static uint32_t deadline = 0;
    static uint32_t frames = 0, last_report = 0, run_us_acc = 0;

    USBHost.task();   // service PIO-USB host: fires the HID callbacks -> keys

    uint32_t t0 = micros();
    coco_machine_run_cycles(CYCLES_PER_FRAME);
    coco_machine_render_frame();
    blit_frame();
    audio_feed();     // drain this field's PCM to the TLV320 over I2S
    run_us_acc += micros() - t0;

    // Pace to real time; resync if we fell behind rather than spiral (pizero
    // pattern). delayMicroseconds yields so the USB CDC task still runs.
    if (deadline == 0) deadline = micros();
    deadline += FRAME_US;
    int32_t rem = (int32_t)(deadline - micros());
    if (rem > 0) delayMicroseconds((uint32_t)rem);
    else         deadline = micros();

    if (++frames - last_report >= 60) {
        static uint32_t last_vf = 0, last_ms = 0;
        uint32_t now_ms = millis();
        uint32_t vf = video_frame_count;
        uint32_t fps = (last_ms && now_ms > last_ms) ? (vf - last_vf) * 1000 / (now_ms - last_ms) : 0;

        Serial.printf("emu %lu fields, avg run %lu us/field (%lu.%02lux real-time), video_frames=%lu (%lu fps), audio_peak=%d/32767\n",
                      (unsigned long)frames,
                      (unsigned long)(run_us_acc / (frames - last_report)),
                      (unsigned long)(FRAME_US * (frames - last_report) / run_us_acc),
                      (unsigned long)((uint64_t)FRAME_US * (frames - last_report) * 100 / run_us_acc % 100),
                      (unsigned long)vf, (unsigned long)fps, g_audio_peak);
        if (g_resync_count) Serial.printf("  (resyncs so far: %lu)\n", (unsigned long)g_resync_count);
        g_audio_peak = 0;

        // Desync watchdog: a healthy 60p link advances ~60 fps. If frames race
        // (>90 fps) the HSTX stream has desynced (FIFO underran) — restart it.
        if (last_ms && fps > 90) {
            g_want_resync = true;   // core 1 performs the resync safely (see core1_background)
            Serial.println("  ! HSTX desync detected -> resync requested (core1)");
        }
        last_vf = vf; last_ms = now_ms;
        last_report = frames; run_us_acc = 0;
    }
    delay(1);   // yield so the USB CDC task runs (serial + 1200-baud reset)
}
