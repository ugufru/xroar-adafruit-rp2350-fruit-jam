/*
 * hot.h — annotate hot inner-loop functions so they live in SRAM instead of
 * flash. AMOLED-29: the RP2350 XIP cache is only ~16 KB; the 6809 emulator's
 * dispatch loop + SAM mem_cycle + event queue together exceed that, so each
 * iteration is paying flash-fetch latency. Wrapping a function definition
 * with HOT_FUNC(name) places it in .time_critical (RAM, single-cycle access).
 */
#ifndef HOT_H_
#define HOT_H_

#ifdef PICO_BUILD
// Inline the .time_critical section attribute rather than pulling in pico.h —
// pico.h drags stdbool.h via a chain that turns `bool` into a macro for
// `_Bool`, which breaks portalib/delegate.h's macro-concatenation of type
// names (`delegate_void_bool_*` becomes `delegate_void__Bool_*`). Same
// section name + same effect as pico-sdk's __not_in_flash_func.
#define HOT_FUNC(name) __attribute__((section(".time_critical." #name))) name
#else
#define HOT_FUNC(name) name
#endif

#endif
