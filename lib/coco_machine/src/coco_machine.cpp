// coco_machine.cpp — see coco_machine.h. FRUITJAM-22.
//
// Owns: 64 KB RAM (SRAM, per COCO-41 — never PSRAM in the hot path), the ROM
// pointer, the MC6809 / MC6883 / two MC6821 / MC6847 parts (via the XRoar part
// system), the machine event list, and the 256x192 VDG palette-index frame.
//
// Bus model follows the real CoCo: on a READ the SAM address decode selects the
// source (RAM / ROM / PIA); on a WRITE the raw address selects the sink (the
// real SAM asserts RAS/nWE for RAM in parallel with the S output). SAM control
// register writes at $FFC0-$FFDF are handed to the SAM module for their side
// effects (TY all-RAM bit, F display-base bits, VDG address counter).
//
// Scope for FRUITJAM-22 is "boot Color BASIC headless + 60 Hz IRQ": no cart,
// no FDC, no audio tap (those are FRUITJAM-19 / -13). The frame renderer is a
// stub here and is implemented in FRUITJAM-24.

#include <Arduino.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

extern "C" {
#include "coco_machine.h"
#include "hot.h"
#include "part.h"
#include "mc6809/mc6809.h"
#include "mc6883.h"
#include "mc6821.h"
#include "mc6847/mc6847.h"
#include "events.h"
#include "delegate.h"
#include "xroar.h"
}

// 16 x 14.31818 MHz master-clock ticks per 6809 "slow" cycle. We run CoCo 2 at
// the default (slow) rate throughout; SAM fast-cycle accuracy is not modelled.
static const int TICKS_PER_CYCLE = 16;

struct CocoMachine {
    const uint8_t *rom = nullptr;
    size_t rom_len = 0;

    struct MC6809 *cpu  = nullptr;
    struct MC6883 *sam  = nullptr;
    struct MC6821 *pia0 = nullptr;
    struct MC6821 *pia1 = nullptr;
    struct MC6847 *vdg  = nullptr;

    // Shadow of the SAM TY bit (all-RAM mode: $FFDF sets, $FFDE clears) and the
    // SAM F register (VDG display-memory base), kept in sync on the SAM-register
    // write path so the fast bus decode and coco_vdg_fetch never reach into the
    // opaque MC6883 private struct.
    bool     sam_ty = false;
    uint16_t sam_f  = 0;

    // PIA interrupt outputs only change on specific accesses; this defers the
    // IRQ/FIRQ re-evaluation to when something could actually have changed.
    bool pia_irq_dirty = true;

    int32_t  cycles_remaining = 0;   // in master-clock ticks
    uint32_t total_mem_cycles = 0;
    uint32_t field_syncs = 0;        // 60 Hz FS pulses = timer-IRQ source proof

    // Nibble-packed palette-index frame: low nibble = even x, high nibble = odd.
    uint8_t vdg_buffer[COCO_VDG_H * (COCO_VDG_W / 2)];
};
static_assert((COCO_VDG_W & 1) == 0, "COCO_VDG_W must be even for nibble packing");

static CocoMachine g_m;

// 64 KB main RAM, SRAM-resident (COCO-41: CoCo RAM never lives in PSRAM).
static uint8_t g_ram[65536];

// - - - keyboard matrix -------------------------------------------------------
// CoCo PIA0: port A = rows (read by CPU), port B = columns (driven by CPU).
// One byte per column, one bit per row; 1 = released, 0 = pressed.
static uint8_t g_kb_col_row_mask[8] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

// PIA0 port-A pre-read hook: resolve the selected column(s) from port B and pull
// down the matching row bits so the CPU sees pressed keys. With all keys
// released this always returns $FF, which is exactly what keeps Color BASIC
// idling in its keyboard-scan loop — the FRUITJAM-22 headless boot condition.
extern "C" void HOT_FUNC(coco_pia0_preread_a)(void *sptr) {
    (void)sptr;
    if (!g_m.pia0) return;
    uint8_t col_sel = PIA_VALUE_B(g_m.pia0);
    uint8_t rows = 0xFF;
    for (int c = 0; c < 8; c++) {
        if (!(col_sel & (1u << c))) rows &= g_kb_col_row_mask[c];
    }
    g_m.pia0->a.in_sink = rows;
}

// XRoar dkbd row order is rotated relative to the CoCo PIA0 rows: for rows 0-5,
// coco_row = (raw_row + 4) % 6; row 6 (modifier keys) is unchanged. col = low 3.
static inline void dscan_to_row_col(uint8_t dscan, uint8_t *row, uint8_t *col) {
    uint8_t raw_row = (dscan >> 3) & 7;
    *col = dscan & 7;
    *row = (raw_row == 6) ? 6 : (uint8_t)((raw_row + 4) % 6);
}

extern "C" void coco_machine_press_key(uint8_t dscan) {
    if (dscan >= 0x40) return;
    uint8_t row, col; dscan_to_row_col(dscan, &row, &col);
    g_kb_col_row_mask[col] &= ~(1u << row);
}
extern "C" void coco_machine_release_key(uint8_t dscan) {
    if (dscan >= 0x40) return;
    uint8_t row, col; dscan_to_row_col(dscan, &row, &col);
    g_kb_col_row_mask[col] |= (1u << row);
}
extern "C" void coco_machine_release_all_keys(void) {
    memset(g_kb_col_row_mask, 0xFF, sizeof(g_kb_col_row_mask));
}

// - - - VDG mode select -------------------------------------------------------
// PIA1 port B bits 7..3 = VDG mode lines (¬A/G, GM2, GM1, GM0, CSS). Mirror GM0
// into the INT/EXT bit as upstream dragon/coco wiring does, then hand to the VDG.
extern "C" void coco_pia1b_postwrite(void *sptr) {
    (void)sptr;
    if (!g_m.pia1 || !g_m.vdg) return;
    unsigned pb = (g_m.pia1->b.out_source & g_m.pia1->b.out_sink) & 0xF8;
    unsigned vmode = pb | ((pb & 0x10) << 4);   // GM0 -> INT/EXT
    mc6847_set_mode(g_m.vdg, vmode);
}

// - - - bus -------------------------------------------------------------------

// ROM read for the $8000-$BFFF window. 16 KB image: rom[A & 0x3FFF] covers both
// Extended ($8000) and Color ($A000) banks. 8 KB image: only $A000-$BFFF is
// populated; the Extended window reads open bus ($FF), as on a plain CoCo.
static inline uint8_t rom_read(uint16_t A) {
    if (g_m.rom_len == 16384) return g_m.rom[A & 0x3FFF];
    if (A >= 0xA000)          return g_m.rom[A & 0x1FFF];
    return 0xFF;
}

extern "C" void HOT_FUNC(coco_mem_cycle)(void *sptr, _Bool RnW, uint16_t A) {
    (void)sptr;
    g_m.total_mem_cycles++;

    if (RnW) {
        // READ — SAM address decode picks the source.
        if (A < 0x8000 || (g_m.sam_ty && A < 0xFF00)) {
            g_m.cpu->D = g_ram[A];                       // RAM (or all-RAM mode)
        } else if (A < 0xC000) {
            g_m.cpu->D = rom_read(A);                    // Extended / Color BASIC
        } else if (A < 0xFF00) {
            g_m.cpu->D = 0xFF;                           // cart ROM window: absent
        } else if (A < 0xFF20) {
            g_m.cpu->D = mc6821_read(g_m.pia0, A);       // PIA0
            if ((A & 1) == 0) g_m.pia_irq_dirty = true;  // data read may clear IRQ
        } else if (A < 0xFF40) {
            g_m.cpu->D = mc6821_read(g_m.pia1, A);       // PIA1
            if ((A & 1) == 0) g_m.pia_irq_dirty = true;
        } else if (A >= 0xFFE0) {
            g_m.cpu->D = rom_read(A);                    // vectors live in Color BASIC ROM
        } else {
            g_m.cpu->D = 0xFF;                           // FF40-FFDF reads: open bus
        }
    } else {
        // WRITE — raw address picks the sink.
        if ((A & 0xFFE0) == 0xFF00) {
            mc6821_write(g_m.pia0, A, g_m.cpu->D);
            if (A & 1) g_m.pia_irq_dirty = true;         // control write may change IRQ enable
        } else if ((A & 0xFFE0) == 0xFF20) {
            mc6821_write(g_m.pia1, A, g_m.cpu->D);
            if (A & 1) g_m.pia_irq_dirty = true;
        } else if ((A & 0xFFE0) == 0xFFC0) {
            // SAM control register — module handles TY/F/V/M side effects.
            g_m.sam->mem_cycle(g_m.sam, RnW, A);
            if      (A == 0xFFDE) g_m.sam_ty = false;
            else if (A == 0xFFDF) g_m.sam_ty = true;
            if (A >= 0xFFC6 && A <= 0xFFD3) {            // F register: display base bits 9..15
                unsigned bit = ((A >> 1) & 0xF) + 6;
                if (A & 1) g_m.sam_f |=  (1u << bit);
                else       g_m.sam_f &= ~(1u << bit);
            }
        } else if (A < 0x8000) {
            g_ram[A] = g_m.cpu->D;                        // RAM
        } else if (g_m.sam_ty && A < 0xFF00) {
            g_ram[A] = g_m.cpu->D;                        // all-RAM mode write-through
        }
        // ROM writes and other I/O writes are ignored.
    }

    // Advance emulated time; run the event queue only when something is due
    // (the common case is nothing pending — skip the call).
    event_ticks new_tick = event_current_tick + TICKS_PER_CYCLE;
    struct event *head = machine_event_list_global->events;
    if (head && event_tick_delta(new_tick, head->at_tick) >= 0) {
        event_run_queue(machine_event_list_global, TICKS_PER_CYCLE);
        g_m.pia_irq_dirty = true;   // VDG events may have toggled PIA CB1/CA1
    } else {
        event_current_tick = new_tick;
    }

    // Re-drive the CPU interrupt lines from the PIA interrupt outputs.
    if (g_m.pia_irq_dirty) {
        MC6809_IRQ_SET(g_m.cpu,  g_m.pia0->a.irq || g_m.pia0->b.irq);
        MC6809_FIRQ_SET(g_m.cpu, g_m.pia1->a.irq || g_m.pia1->b.irq);
        g_m.pia_irq_dirty = false;
    }

    g_m.cycles_remaining -= TICKS_PER_CYCLE;
    if (g_m.cycles_remaining <= 0) g_m.cpu->running = 0;
}

// - - - VDG sync + fetch ------------------------------------------------------
// VDG sync outputs on the real CoCo drive PIA0 control inputs (and the SAM
// raster counters):
//   FS (field sync, ~60 Hz) -> PIA0 CB1 -> the interrupt Color BASIC times on
//   HS (horizontal sync)    -> PIA0 CA1
extern "C" void HOT_FUNC(coco_vdg_signal_fs)(void *sptr, _Bool level) {
    (void)sptr;
    if (level) g_m.field_syncs++;
    if (g_m.sam)  g_m.sam->vdg_fsync(g_m.sam, level);
    if (g_m.pia0) mc6821_set_cx1(&g_m.pia0->b, level);
}
extern "C" void HOT_FUNC(coco_vdg_signal_hs)(void *sptr, _Bool level) {
    (void)sptr;
    if (g_m.sam)  g_m.sam->vdg_hsync(g_m.sam, level);
    if (g_m.pia0) mc6821_set_cx1(&g_m.pia0->a, level);
}

// VDG video fetch: read `nwords` display bytes from RAM at the SAM F base,
// replicating the two high bits into D9/D8 as the VDG sees them (used for the
// INV/semigraphics attribute lines). Base falls back to $0400 (DECB text page).
extern "C" void HOT_FUNC(coco_vdg_fetch)(void *sptr, uint16_t A, int nwords, uint16_t *dest) {
    (void)sptr;
    const uint16_t base = g_m.sam_f ? g_m.sam_f : 0x0400;
    for (int i = 0; i < nwords; i++) {
        uint8_t v = g_ram[(uint16_t)(base + A + i)];
        dest[i] = (uint16_t)v | (uint16_t)((v & 0xC0) << 2);
    }
}

// Per-scanline render is suppressed (frame-batched renderer, FRUITJAM-24). We
// still register this so mc6847's scanline events run and drive FS/HS timing.
extern "C" void HOT_FUNC(coco_vdg_render)(void *sptr, unsigned burst, unsigned npixels,
                                          const uint8_t *data) {
    (void)sptr; (void)burst; (void)npixels; (void)data;
}

// - - - init / run ------------------------------------------------------------

extern "C" _Bool coco_machine_init(const uint8_t *rom, size_t rom_len) {
    if (rom_len != 8192 && rom_len != 16384) return 0;
    g_m.rom = rom;
    g_m.rom_len = rom_len;

    memset(g_ram, 0, sizeof(g_ram));
    memset(g_m.vdg_buffer, 0, sizeof(g_m.vdg_buffer));

    // Event list must exist before any part_create() that uses MACHINE_EVENT_LIST.
    machine_event_list_global = event_list_new();
    if (!machine_event_list_global) return 0;
    event_current_tick = 0;

    struct part *p;

    p = part_create("MC6809", NULL);
    if (!p) return 0;
    g_m.cpu = (struct MC6809 *)p;
    g_m.cpu->mem_cycle = DELEGATE_AS2(void, bool, uint16, coco_mem_cycle, g_m.cpu);

    p = part_create("SN74LS783", NULL);   // MC6883 / SN74LS783 SAM
    if (!p) return 0;
    g_m.sam = (struct MC6883 *)p;
    g_m.sam->reset(g_m.sam);

    p = part_create("MC6821", NULL);      // PIA0: keyboard + sync IRQ
    if (!p) return 0;
    g_m.pia0 = (struct MC6821 *)p;
    mc6821_reset(g_m.pia0);
    g_m.pia0->a.in_source = 0xFF;
    g_m.pia0->a.in_sink   = 0xFF;
    g_m.pia0->b.in_source = 0xFF;
    g_m.pia0->a.data_preread = DELEGATE_AS0(void, coco_pia0_preread_a, NULL);

    p = part_create("MC6821", NULL);      // PIA1: VDG mode + (later) sound
    if (!p) return 0;
    g_m.pia1 = (struct MC6821 *)p;
    mc6821_reset(g_m.pia1);
    g_m.pia1->b.in_source = 0xFF;
    g_m.pia1->b.data_postwrite = DELEGATE_AS0(void, coco_pia1b_postwrite, NULL);

    // CoCo 2 ships the 6847T1 (lowercase via inverse bit). The non-T1 font path
    // in mc6847.c is then dead at runtime (font_6847[] is a link-time stub).
    p = part_create("MC6847", (void *)"6847T1");
    if (!p) return 0;
    g_m.vdg = (struct MC6847 *)p;
    g_m.vdg->fetch_data  = DELEGATE_AS3(void, uint16, int, uint16p, coco_vdg_fetch, g_m.vdg);
    g_m.vdg->render_line = DELEGATE_AS3(void, unsigned, unsigned, uint8cp, coco_vdg_render, g_m.vdg);
    g_m.vdg->signal_fs   = DELEGATE_AS1(void, bool, coco_vdg_signal_fs, g_m.vdg);
    g_m.vdg->signal_hs   = DELEGATE_AS1(void, bool, coco_vdg_signal_hs, g_m.vdg);
    mc6847_reset(g_m.vdg);
    mc6847_set_mode(g_m.vdg, 0);
    mc6847_unpause(g_m.vdg);

    return 1;
}

extern "C" void coco_machine_run_cycles(uint32_t cycles) {
    if (!g_m.cpu) return;

    // PIZERO-31: keep the 60 Hz field-sync timer interrupt enabled. On real
    // hardware the FS line is wired and Color BASIC leaves PIA0 CB1's interrupt
    // enabled; force the enable bit so nothing (e.g. a later direct-loaded
    // program) can freeze BASIC's TIMER / cursor blink. Only bit 0 is touched.
    if (g_m.pia0) g_m.pia0->b.control_register |= 0x01;

    g_m.cycles_remaining = (int32_t)cycles * TICKS_PER_CYCLE;
    g_m.cpu->running = 1;
    g_m.cpu->run(g_m.cpu);
}

// - - - frame renderer (stub — FRUITJAM-24 implements) ------------------------
extern "C" void coco_machine_render_frame(void) {
    // FRUITJAM-24 will regenerate vdg_buffer from PIA1 PB mode + SAM F + RAM.
    // Until then, leave the buffer as-is (all-black at init).
}

// - - - accessors -------------------------------------------------------------
extern "C" const uint8_t *coco_machine_get_vdg_buffer(void) { return g_m.vdg_buffer; }
extern "C" uint16_t coco_machine_get_pc(void)               { return g_m.cpu ? g_m.cpu->reg_pc : 0; }
extern "C" uint32_t coco_machine_get_total_mem_cycles(void){ return g_m.total_mem_cycles; }
extern "C" uint32_t coco_machine_get_irq_count(void)       { return g_m.field_syncs; }
extern "C" const uint8_t *coco_machine_peek_ram(uint16_t addr) { return &g_ram[addr]; }
