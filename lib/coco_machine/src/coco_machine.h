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
