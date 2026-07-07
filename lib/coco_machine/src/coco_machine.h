// SPDX-License-Identifier: GPL-3.0-or-later
// coco_machine — hand-written CoCo 1/2 integration glue (FRUITJAM-22).
//
// Wires the vendored XRoar 1.11 chip modules (mc6809 CPU, mc6883 SAM,
// two mc6821 PIAs, mc6847 VDG) into a runnable Tandy Color Computer.
// XRoar's own machine layer (machine.c / dragon.c) was deliberately NOT
// vendored (FRUITJAM-09), so this file IS our machine.
//
// The public surface is intentionally small: init, run-for-N-cycles, and
// read the VDG palette-index frame. Display, keyboard, SD and audio wiring
// all sit ABOVE this layer (FRUITJAM-24/25, -12, -13).
//
// Written fresh for this repo; the CoCo wiring itself is fixed hardware.
#ifndef COCO_MACHINE_H_
#define COCO_MACHINE_H_

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// CoCo VDG active area (the classic 32x16 text grid = 256x192 pixels).
#define COCO_VDG_W 256
#define COCO_VDG_H 192

// Initialise the machine with a Color BASIC ROM image.
//   rom_len == 8192  : Color BASIC only, mapped at $A000-$BFFF ($8000-$9FFF
//                      reads open bus, as on a CoCo with no Extended BASIC).
//   rom_len == 16384 : Extended Color BASIC ($8000-$9FFF) + Color BASIC
//                      ($A000-$BFFF) packed low-to-high.
// The ROM buffer is caller-owned and must outlive the machine. Returns
// true on success.
_Bool coco_machine_init(const uint8_t *rom, size_t rom_len);

// Warm reset: re-assert the 6809 RESET line so it re-reads the reset vector and
// restarts Color BASIC, preserving RAM — the authentic CoCo reset-button
// behavior. Used by the keyboard reset chord (FRUITJAM-12).
void coco_machine_reset(void);

// Run the 6809 for approximately `cycles` CPU cycles (~0.895 MHz authentic).
// The VDG/SAM event queue and the 60 Hz field-sync IRQ are pumped as a side
// effect of the CPU's memory cycles. Frame pacing lives above this call
// (FRUITJAM-23).
void coco_machine_run_cycles(uint32_t cycles);

// Regenerate the VDG palette-index frame from current CoCo state (PIA1 PB
// mode bits + SAM F display base + RAM). FRUITJAM-24 implements the real
// renderer; called once per field by the presentation side. The per-scanline
// render_line path is suppressed, so this is the only producer of the frame.
void coco_machine_render_frame(void);

// Current VDG frame: COCO_VDG_H rows of COCO_VDG_W/2 bytes, nibble-packed
// (two 4-bit palette indices per byte; low nibble = even x). Stable for the
// machine's lifetime; do not free or write.
const uint8_t *coco_machine_get_vdg_buffer(void);

// - - - audio (FRUITJAM-13) ---------------------------------------------------
// The CoCo sound seam. The machine taps PIA1's 6-bit DAC (PA7-2), the sound-mux
// enable (PIA1 CB2), the single-bit sound line (PIA1 PB1) and the mux source
// select (PIA0 CA2/CB2), computes the analog sound-bus level exactly as XRoar's
// dragon.c/sound.c do, and integrate-and-dumps it into 48 kHz mono samples keyed
// to CPU cycle timestamps. The platform pulls PCM and feeds it to the I2S DAC.
//
// coco_machine_audio_init sets the resample cadence: it must match the pacing
// the platform actually runs the emulator at (cycles per field / field period
// in microseconds), so one field's samples equal what a real-time 48 kHz sink
// drains in that field — otherwise the sink ring slowly drifts. Called once,
// after coco_machine_init. Passing 0/0 keeps the authentic-NTSC default.
void coco_machine_audio_init(uint32_t cycles_per_field, uint32_t field_us);

// Pull up to `max` mono signed-16-bit samples generated since the last call,
// integrate-and-dumped over the CPU cycles executed since then. Returns the
// count produced (tracks emulated time; ~805 per 16762 us field). Call once per
// field, after coco_machine_run_cycles.
int coco_machine_render_audio(int16_t *out, int max);

// - - - cassette (.cas) playback (FRUITJAM-28) --------------------------------
// Load a .cas tape image (caller-owned buffer, must outlive playback) and let
// the CoCo read it with CLOAD / CLOADM. Playback is event-driven and auto-gated
// by the emulated cassette motor relay (PIA1 CA2), exactly like real hardware:
// just load the image, then CLOAD in BASIC and the tape "plays" while the motor
// runs. A short 0x55 leader is synthesised ahead of the image so the ROM's tape
// routine can sync even if the file omits one. Returns true if accepted.
_Bool coco_machine_cas_load(const uint8_t *cas, size_t len);
void  coco_machine_cas_eject(void);
_Bool coco_machine_cas_motor(void);   // diagnostic: cassette motor relay state

// Keyboard injection by XRoar DSCAN_* code (see xroar_core dkbd.h). All keys
// released at init. FRUITJAM-12 maps USB HID onto this.
void coco_machine_press_key(uint8_t dscan);
void coco_machine_release_key(uint8_t dscan);
void coco_machine_release_all_keys(void);

// Diagnostics (headless bring-up / smoke tests).
uint16_t coco_machine_get_pc(void);
uint32_t coco_machine_get_total_mem_cycles(void);
uint32_t coco_machine_get_irq_count(void);      // 60 Hz field-sync IRQs taken
const uint8_t *coco_machine_peek_ram(uint16_t addr);

#ifdef __cplusplus
}
#endif

#endif // COCO_MACHINE_H_
