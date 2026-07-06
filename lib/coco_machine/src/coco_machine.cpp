// SPDX-License-Identifier: GPL-3.0-or-later
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

// Forward decl for the audio tap (defined in the audio section below).
static void snd_event(void);

// - - - VDG mode select -------------------------------------------------------
// PIA1 port B bits 7..3 = VDG mode lines (¬A/G, GM2, GM1, GM0, CSS). Mirror GM0
// into the INT/EXT bit as upstream dragon/coco wiring does, then hand to the VDG.
// PB also carries the single-bit sound line (PB1), so an audio event fires here
// too (mirrors dragon_pia1b_data_postwrite: single-bit + VDG mode).
extern "C" void HOT_FUNC(coco_pia1b_postwrite)(void *sptr) {
    (void)sptr;
    if (!g_m.pia1 || !g_m.vdg) return;
    snd_event();                                // single-bit sound (PB1)
    unsigned pb = (g_m.pia1->b.out_source & g_m.pia1->b.out_sink) & 0xF8;
    unsigned vmode = pb | ((pb & 0x10) << 4);   // GM0 -> INT/EXT
    mc6847_set_mode(g_m.vdg, vmode);
}

// PIA1 port A write = 6-bit DAC update.  PIA1 CB2 (control) = sound-mux enable.
// FRUITJAM-15: these fire from the hot coco_mem_cycle on every PIA write (dense
// during SOUND), so keep them and snd_compute() in SRAM (core-set I/O path).
extern "C" void HOT_FUNC(coco_pia1a_postwrite)(void *sptr)      { (void)sptr; snd_event(); }
extern "C" void HOT_FUNC(coco_pia1b_control_postwrite)(void *sptr) { (void)sptr; snd_event(); }
// PIA0 CA2/CB2 (control writes) = sound-mux source select.
extern "C" void HOT_FUNC(coco_pia0_control_postwrite)(void *sptr)  { (void)sptr; snd_event(); }

// - - - audio tap + resampler (FRUITJAM-13) -----------------------------------
// The CoCo sound bus, modelled exactly as XRoar's dragon.c/sound.c:
//   * 6-bit DAC  = PIA1 port A bits 7..2                       (PIA_VALUE_A & 0xFC)
//   * mux enable = PIA1 CB2                                    (PIA_VALUE_CB2(pia1))
//   * single-bit = PIA1 PB1                                    (port B bit 1)
//   * mux source = PIA0 CB2:CA2                                (0 = DAC)
// (Note: the FRUITJAM-13 issue calls the single-bit line CB2; upstream XRoar
// 1.11 — the core we vendored — wires single-bit to PB1 and uses CB2 as the mux
// *enable*. We follow the vendored source, which is self-consistent and covers
// both: SOUND/PLAY via the DAC, and 1-bit audio via either the CB2 enable toggle
// or the PB1 line. Flagged for maintainer reconciliation.)
//
// snd_recompute() collapses those pins into a single analog bus level (0..~1),
// reproducing sound.c's per-source gain/DC-offset tables (MAX_V = 4.70 V). The
// bus is integrate-and-dumped over CPU cycles into 48 kHz mono; a one-pole DC
// blocker removes the (large, note-dependent) DC term so the AC waveform — the
// actual sound — is centered before it reaches the DAC.

// sound.c source_gain_v/source_offset_v, DAC and single-bit rows, /MAX_V.
static const float MAX_V = 4.70f;
static const float DAC_GAIN[3] = { 4.50f / MAX_V, 2.84f / MAX_V, 3.40f / MAX_V };
static const float DAC_OFF[3]  = { 0.20f / MAX_V, 0.18f / MAX_V, 1.30f / MAX_V };
static const float SBS_OFF[3]  = { 0.00f,         0.00f,         3.90f / MAX_V };

// Output scaling. The CoCo SOUND DAC does not swing full-scale, and (per
// sound.c) the single-bit config drops the DAC gain to ~0.6-0.7x, so the steady
// tone AC is well under half scale — 20000 left it quieter than the FRUITJAM-07
// test tone. On-hardware measurement (serial audio_peak) showed the note-edge DC
// steps reaching ~0.82 of full scale; 32000 lifts the steady tone to roughly the
// test-tone level while keeping those transients (0.82 * 32000 ~= 26k) just under
// the 32767 clip. Higher gains clip the edges into clicks.
static const float AUDIO_GAIN = 32000.0f;

// Event log of bus-level CHANGES during the current field: each PIA sound write
// appends {cycle, new bus level}. render_audio (run once after the field) then
// replays the log to integrate each 48 kHz sample over ITS OWN cycle span. This
// separation is essential: the hooks fire mid-field while render runs after it,
// so the two must not share one accumulator (the earlier bug collapsed a whole
// field of toggles into a single clipped sample). A square-wave SOUND is a few
// hundred edges/field; 1024 covers well past any audible tone (overflow just
// drops later edges that field — graceful).
struct SndSeg { uint32_t cyc; float bus; };
static SndSeg   g_snd_log[1024];
static int      g_snd_log_n     = 0;      // entries recorded this field
static int      g_snd_log_rd    = 0;      // render read cursor
static float    g_snd_cur_bus   = 0.0f;   // latest bus level (hook side)
static float    g_snd_seg_bus   = 0.0f;   // bus level of the segment render is in
static uint32_t g_snd_bound_cyc = 0;      // start cycle of the next sample
static uint32_t g_snd_frac_q16  = 0;      // fractional-cycle remainder, 16.16
static uint32_t g_snd_cps_q16   = 0;      // cycles per 48 kHz sample, 16.16
static float    g_snd_dc_x1     = 0.0f;   // DC-blocker state
static float    g_snd_dc_y1     = 0.0f;

// Collapse the PIA sound pins into the analog bus level (sound.c bus_level).
static float HOT_FUNC(snd_compute)(void) {
    if (!g_m.pia1 || !g_m.pia0) return g_snd_cur_bus;
    bool  enabled = PIA_VALUE_CB2(g_m.pia1);
    float dac     = (float)(PIA_VALUE_A(g_m.pia1) & 0xFC) / 252.0f;
    // Single-bit sound on PIA1 PB1 (upstream dragon_pia1b_data_postwrite).
    unsigned bsrc = g_m.pia1->b.out_source, bsnk = g_m.pia1->b.out_sink;
    bool sbs_enabled = !((bsrc ^ bsnk) & (1u << 1));
    bool sbs_level   =  (bsrc & bsnk & (1u << 1)) != 0;
    if (enabled) {
        unsigned src = ((unsigned)PIA_VALUE_CB2(g_m.pia0) << 1) | (unsigned)PIA_VALUE_CA2(g_m.pia0);
        unsigned si  = sbs_enabled ? (sbs_level ? 2u : 1u) : 0u;
        float raw = (src == 0) ? dac : 0.0f;   // only the DAC source is emulated
        return raw * DAC_GAIN[si] + DAC_OFF[si];
    }
    if (sbs_enabled) return SBS_OFF[sbs_level ? 2 : 1];
    return g_snd_cur_bus;   // mux off, single-bit as input -> hold last level
}

// PIA write hook: record the new bus level (with its CPU-cycle timestamp) iff it
// changed. total_mem_cycles is already incremented for the in-progress cycle.
static inline void snd_event(void) {
    float b = snd_compute();
    if (b != g_snd_cur_bus) {
        g_snd_cur_bus = b;
        if (g_snd_log_n < (int)(sizeof(g_snd_log) / sizeof(g_snd_log[0])))
            g_snd_log[g_snd_log_n++] = { g_m.total_mem_cycles, b };
    }
}

extern "C" void coco_machine_audio_init(uint32_t cycles_per_field, uint32_t field_us) {
    if (cycles_per_field == 0 || field_us == 0) {
        cycles_per_field = 14915;   // authentic NTSC field (see coco_main.cpp)
        field_us         = 16762;
    }
    // cycles per 48 kHz sample = cycles_per_field / (field_us * 48000 / 1e6).
    g_snd_cps_q16 = (uint32_t)(((uint64_t)cycles_per_field * 1000000ULL * 65536ULL) /
                               ((uint64_t)field_us * 48000ULL));
    g_snd_frac_q16  = 0;
    g_snd_bound_cyc = g_m.total_mem_cycles;
    g_snd_log_n = g_snd_log_rd = 0;
    g_snd_cur_bus = g_snd_seg_bus = snd_compute();
}

extern "C" int coco_machine_render_audio(int16_t *out, int max) {
    if (g_snd_cps_q16 == 0) coco_machine_audio_init(0, 0);
    const uint32_t end = g_m.total_mem_cycles;
    int n = 0;
    while (n < max) {
        uint32_t step_q16 = g_snd_frac_q16 + g_snd_cps_q16;
        uint32_t step     = step_q16 >> 16;
        uint32_t target   = g_snd_bound_cyc + step;
        if ((int32_t)(target - end) > 0) break;   // not enough emulated time yet
        g_snd_frac_q16 = step_q16 & 0xFFFF;

        // Integrate the piecewise-constant bus over [g_snd_bound_cyc, target),
        // walking any change events that fall inside this sample's span.
        float    integral = 0.0f;
        uint32_t c        = g_snd_bound_cyc;
        float    level    = g_snd_seg_bus;
        while (g_snd_log_rd < g_snd_log_n &&
               (int32_t)(g_snd_log[g_snd_log_rd].cyc - target) < 0) {
            uint32_t ec = g_snd_log[g_snd_log_rd].cyc;
            if ((int32_t)(ec - c) > 0) { integral += level * (int32_t)(ec - c); c = ec; }
            level = g_snd_log[g_snd_log_rd].bus;
            g_snd_log_rd++;
        }
        integral += level * (int32_t)(target - c);
        g_snd_seg_bus   = level;
        g_snd_bound_cyc = target;

        float avg = (step != 0) ? (integral / (int32_t)step) : level;
        // One-pole DC blocker (R = 0.9995, ~4 Hz / ~42 ms tau): removes the
        // note-dependent DC offset while leaving the tone's shape and level.
        float y = avg - g_snd_dc_x1 + 0.9995f * g_snd_dc_y1;
        g_snd_dc_x1 = avg;
        g_snd_dc_y1 = y;
        int32_t s = (int32_t)(y * AUDIO_GAIN);
        if (s > 32767) s = 32767; else if (s < -32768) s = -32768;
        out[n++] = (int16_t)s;
    }

    // Drop consumed events; carry any past `end` (not yet sampled) to next field.
    if (g_snd_log_rd > 0) {
        int rem = g_snd_log_n - g_snd_log_rd;
        for (int i = 0; i < rem; i++) g_snd_log[i] = g_snd_log[g_snd_log_rd + i];
        g_snd_log_n  = rem;
        g_snd_log_rd = 0;
    }
    return n;
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
    // PIA0 CA2/CB2 select the sound-mux source (FRUITJAM-13).
    g_m.pia0->a.control_postwrite = DELEGATE_AS0(void, coco_pia0_control_postwrite, NULL);
    g_m.pia0->b.control_postwrite = DELEGATE_AS0(void, coco_pia0_control_postwrite, NULL);

    p = part_create("MC6821", NULL);      // PIA1: VDG mode + (later) sound
    if (!p) return 0;
    g_m.pia1 = (struct MC6821 *)p;
    mc6821_reset(g_m.pia1);
    g_m.pia1->b.in_source = 0xFF;
    g_m.pia1->b.data_postwrite = DELEGATE_AS0(void, coco_pia1b_postwrite, NULL);
    // Audio tap (FRUITJAM-13): DAC on port A, mux-enable on CB2.
    g_m.pia1->a.data_postwrite    = DELEGATE_AS0(void, coco_pia1a_postwrite, NULL);
    g_m.pia1->b.control_postwrite = DELEGATE_AS0(void, coco_pia1b_control_postwrite, NULL);

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

    // Audio: seat the resampler at the current (zero) cycle and capture the
    // reset bus level so integration starts from a defined state.
    coco_machine_audio_init(0, 0);
    g_snd_dc_x1 = g_snd_dc_y1 = 0.0f;

    return 1;
}

extern "C" void coco_machine_reset(void) {
    if (!g_m.cpu) return;
    // Re-assert RESET: the CPU re-reads the reset vector next run and restarts
    // Color BASIC. RAM is left intact (authentic warm reset). BASIC re-inits the
    // SAM/PIA/VDG itself as it boots. Also release any stuck keys.
    coco_machine_release_all_keys();
    g_m.cpu->reset(g_m.cpu);
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

// - - - frame renderer (FRUITJAM-24) ------------------------------------------
//
// Frame-batched, palette-LUT VDG renderer. Once per field the presentation side
// calls coco_machine_render_frame(), which reads PIA1 PB (mode bits) + the SAM F
// display base + CoCo RAM and regenerates the whole nibble-packed vdg_buffer.
// This replaces XRoar's per-scanline render_line path (which we no-op), the
// ~2x-cheaper approach from COCO-47/AMOLED-57.
//
// Palette indices below are the contract with the presentation layer's RGB565
// table (FRUITJAM-25). Keep the two in lockstep.

extern "C" const uint8_t font_6847t1[];   // 128 glyphs x 12 rows

#define PAL_GREEN       0
#define PAL_YELLOW      1
#define PAL_BLUE        2
#define PAL_RED         3
#define PAL_WHITE       4
#define PAL_CYAN        5
#define PAL_MAGENTA     6
#define PAL_ORANGE      7
#define PAL_BLACK       8
#define PAL_DARK_GREEN  9

// Vertical replication (display scanlines per data row) for graphics GM 0..7.
static const uint8_t GM_nLPR[8] = { 3, 3, 3, 2, 2, 1, 1, 1 };

static inline void put2(uint8_t *dst, int px, uint8_t color) {
    int b = px >> 1;
    if (px & 1) dst[b] = (dst[b] & 0x0F) | (uint8_t)(color << 4);
    else        dst[b] = (dst[b] & 0xF0) | color;
}

// Precomputed expansion of an 8-px font row into the 4 packed vdg_buffer bytes,
// for the two BASIC text colourings (one 32-bit store replaces 8 put2 calls):
//   [0] normal  green-on-black   (ink=GREEN,  paper=BLACK)
//   [1] inverse black-on-green   (ink=BLACK,  paper=GREEN)   <- the iconic look
static uint32_t g_alpha_lut[2][256];
static bool     g_alpha_lut_ready = false;

static void build_alpha_lut(void) {
    const uint8_t combos[2][2] = { { PAL_GREEN, PAL_BLACK }, { PAL_BLACK, PAL_GREEN } };
    for (int c = 0; c < 2; c++) {
        uint8_t ink = combos[c][0], paper = combos[c][1];
        for (int g = 0; g < 256; g++) {
            uint32_t word = 0;
            for (int bit = 0; bit < 8; bit++) {
                uint8_t color = (g & (0x80 >> bit)) ? ink : paper;
                int byte = bit >> 1, shift = (bit & 1) ? 4 : 0;   // low nibble = even px
                word |= (uint32_t)color << (byte * 8 + shift);
            }
            g_alpha_lut[c][g] = word;
        }
    }
    g_alpha_lut_ready = true;
}

// Alphanumeric + SG4 semigraphics. 32 cols x 16 rows, each glyph 8x12.
static void HOT_FUNC(render_alpha_frame)(uint16_t base) {
    if (!g_alpha_lut_ready) build_alpha_lut();
    for (int text_row = 0; text_row < 16; text_row++) {
        for (int sub = 0; sub < 12; sub++) {
            int row = text_row * 12 + sub;
            if (row >= COCO_VDG_H) return;
            uint8_t *dst = &g_m.vdg_buffer[row * (COCO_VDG_W / 2)];
            const uint8_t *cells = &g_ram[(uint16_t)(base + text_row * 32)];
            for (int col = 0; col < 32; col++) {
                uint8_t ch = cells[col];
                if (ch & 0x80) {
                    // SG4: 2x2 colour block, colour = bits 6..4, pattern = bits 3..0.
                    uint8_t color = (ch >> 4) & 7;
                    uint8_t sg = (sub < 6) ? (ch >> 2) : ch;   // top vs bottom half
                    uint8_t left  = (sg & 2) ? color : PAL_BLACK;
                    uint8_t right = (sg & 1) ? color : PAL_BLACK;
                    int bp = col * 8;
                    for (int b = 0; b < 8; b++) put2(dst, bp + b, (b < 4) ? left : right);
                } else {
                    // Alpha: bit 6 = inverse. Glyph index (ch & 0x3F) | 0x40.
                    uint8_t glyph = font_6847t1[(((ch & 0x3F) | 0x40)) * 12 + sub];
                    *(uint32_t *)(dst + col * 4) = g_alpha_lut[(ch >> 6) & 1][glyph];
                }
            }
        }
    }
}

// Colour / resolution graphics, GM 0..6 (RG6/GM7 has its own path below).
// RG modes: 1 bit/pixel fg/bg. CG modes: 2 bits/pixel, colour = cg_base + value.
static void HOT_FUNC(render_graphics_frame)(uint16_t base, uint8_t gm, bool css) {
    const bool is_32       = (gm == 2 || gm == 4 || gm == 6);
    const int  bytes_per_row = is_32 ? 32 : 16;
    const int  nlpr        = GM_nLPR[gm];
    const bool rg          = gm & 1;
    const int  data_rows   = COCO_VDG_H / nlpr;

    const uint8_t cg_base = css ? PAL_WHITE : PAL_GREEN;
    const uint8_t fg = css ? PAL_WHITE : PAL_GREEN;
    const uint8_t bg = css ? PAL_BLACK : PAL_DARK_GREEN;

    const int src_px = rg ? bytes_per_row * 8 : bytes_per_row * 4;
    const int hrep   = COCO_VDG_W / src_px;

    static uint8_t rowbuf[COCO_VDG_W];
    for (int drow = 0; drow < data_rows; drow++) {
        const uint8_t *p = &g_ram[(uint16_t)(base + drow * bytes_per_row)];
        int px = 0;
        for (int byte = 0; byte < bytes_per_row; byte++) {
            uint8_t b = p[byte];
            if (rg) {
                for (int bit = 0; bit < 8; bit++) {
                    uint8_t c = (b & (0x80 >> bit)) ? fg : bg;
                    for (int r = 0; r < hrep; r++) rowbuf[px++] = c;
                }
            } else {
                for (int cell = 0; cell < 4; cell++) {
                    uint8_t c = cg_base + ((b >> 6) & 3);
                    b <<= 2;
                    for (int r = 0; r < hrep; r++) rowbuf[px++] = c;
                }
            }
        }
        for (int rep = 0; rep < nlpr; rep++) {
            int disp = drow * nlpr + rep;
            if (disp >= COCO_VDG_H) break;
            uint8_t *dst = &g_m.vdg_buffer[disp * (COCO_VDG_W / 2)];
            for (int x = 0; x < COCO_VDG_W; x += 2)
                dst[x >> 1] = rowbuf[x] | (rowbuf[x + 1] << 4);
        }
    }
}

// RG6 / PMODE 4: 1 bit/pixel, 256x192 mono. NTSC artifact colour is a later
// refinement (its phase is power-on-arbitrary — a FRUITJAM polish item); render
// plain white-on-black for now.
static void HOT_FUNC(render_rg6_frame)(uint16_t base) {
    for (int row = 0; row < COCO_VDG_H; row++) {
        const uint8_t *src = &g_ram[(uint16_t)(base + row * 32)];
        uint8_t *dst = &g_m.vdg_buffer[row * (COCO_VDG_W / 2)];
        for (int byte = 0; byte < 32; byte++) {
            uint8_t b = src[byte];
            for (int bit = 0; bit < 8; bit++)
                put2(dst, byte * 8 + bit, (b & (0x80 >> bit)) ? PAL_WHITE : PAL_BLACK);
        }
    }
}

extern "C" void HOT_FUNC(coco_machine_render_frame)(void) {
    if (!g_m.pia1) return;
    const uint8_t  pb   = (g_m.pia1->b.out_source & g_m.pia1->b.out_sink) & 0xFF;
    const uint16_t base = g_m.sam_f ? g_m.sam_f : 0x0400;
    const uint8_t  gm   = (pb >> 4) & 7;
    const bool     css  = (pb & 0x08) != 0;
    if (pb & 0x80) {                     // ¬A/G set -> graphics
        if (gm == 7) render_rg6_frame(base);
        else         render_graphics_frame(base, gm, css);
    } else {                             // alpha + SG4
        render_alpha_frame(base);
    }
}

// - - - accessors -------------------------------------------------------------
extern "C" const uint8_t *coco_machine_get_vdg_buffer(void) { return g_m.vdg_buffer; }
extern "C" uint16_t coco_machine_get_pc(void)               { return g_m.cpu ? g_m.cpu->reg_pc : 0; }
extern "C" uint32_t coco_machine_get_total_mem_cycles(void){ return g_m.total_mem_cycles; }
extern "C" uint32_t coco_machine_get_irq_count(void)       { return g_m.field_syncs; }
extern "C" const uint8_t *coco_machine_peek_ram(uint16_t addr) { return &g_ram[addr]; }
