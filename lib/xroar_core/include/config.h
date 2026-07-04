/* Hand-written config.h for the RP2350 port of XRoar (FRUITJAM-09).
 * Replaces the autotools-generated config.h. Keep this minimal — every
 * HAVE_* flipped on here pulls more host code into the build.
 */
#ifndef XROAR_PORT_CONFIG_H
#define XROAR_PORT_CONFIG_H

#define HAVE___BUILTIN_EXPECT 1
#define HAVE_FUNC_ATTRIBUTE_CONST 1
#define HAVE_FUNC_ATTRIBUTE_FORMAT 1
#define HAVE_FUNC_ATTRIBUTE_MALLOC 1
#define HAVE_FUNC_ATTRIBUTE_NONNULL 1
#define HAVE_FUNC_ATTRIBUTE_NORETURN 1
#define HAVE_FUNC_ATTRIBUTE_RETURNS_NONNULL 1
#define HAVE_FUNC_ATTRIBUTE_PURE 1

/* Part-database selection (consumed only by src/part.c's partdb[] array).
 * We register just the chips this port vendors: the MC6809 CPU + MC6821 PIA,
 * the MC6883/SN74LS783 SAM, and the MC6847/MC6847T1 VDG. The upstream part.c
 * lists a few sibling parts under these same WANT_ guards that we do NOT
 * vendor (hd6309, samx8) — plus two unconditional entries (mos6551, ay891x).
 * Those four are satisfied by link-time stub partdb entries in xroar_stubs.c
 * so we never have to touch the frozen part.c. */
#define WANT_PART_MC6809 1
#define WANT_PART_MC6883 1
#define WANT_PART_MC6847 1

/* Intentionally NOT defined:
 *   LOGGING              — logging.h falls back to no-op macros
 *   TRACE                — no disassembly path on the Pico
 *   WANT_MACHINE_ARCH_*  — no full machine parts vendored
 *   WANT_CART_ARCH_*     — no cartridge parts vendored
 *   HAVE_SDL2 etc.       — no host stack
 */

#endif
