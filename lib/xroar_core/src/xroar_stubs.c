/*
 * xroar_stubs.c — definitions for symbols our stub xroar.h / logging.h declare,
 * plus link-time stand-ins for the handful of partdb entries the frozen
 * src/part.c references but that this port does not vendor (FRUITJAM-09).
 *
 * None of these are actively exercised by the smoke harness; they exist purely
 * so unresolved references resolve. The alternative — pruning part.c's partdb[]
 * array — would mean editing a frozen upstream file, which the vendoring policy
 * forbids. Stubs keep part.c byte-for-byte identical to tag 1.11.
 */
#include <stddef.h>

#include "logging.h"
#include "xroar.h"
#include "part.h"

/* Global log/debug flags. All zero — every LOG_* macro is a no-op when
 * LOGGING is not defined (see logging.h), but the struct itself is
 * referenced by mc6809.c (logging.trace_cpu) at compile time. */
struct logging logging;

/* The machine-wide event list. A full machine build (a later issue) will own
 * this; the smoke harness creates only a bare 6809 and never dereferences it. */
struct event_list *machine_event_list_global;

/* - - - partdb stubs - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 *
 * src/part.c's partdb[] array references these four entries:
 *   - hd6309_part, samx8_part : siblings inside the WANT_PART_MC6809 /
 *     WANT_PART_MC6883 guards we enable, but whose .c files we don't vendor.
 *   - mos6551_part, ay891x_part : listed unconditionally in the array.
 * We never create these parts (part_create is only ever called by name for
 * the chips we do vendor), so partdb_find_entry's strcmp simply skips them.
 * They still need valid, non-NULL .funcs so partdb_foreach / partdb_ent_is_a
 * don't dereference NULL if anything iterates the table. */
static struct part *stub_allocate(void) { return NULL; }
static const struct partdb_entry_funcs stub_funcs = {
	.allocate = stub_allocate,
};
const struct partdb_entry hd6309_part  = { .name = "HD6309",  .description = "unvendored (stub)", .funcs = &stub_funcs };
const struct partdb_entry samx8_part   = { .name = "SAMX8",   .description = "unvendored (stub)", .funcs = &stub_funcs };
const struct partdb_entry mos6551_part = { .name = "MOS6551", .description = "unvendored (stub)", .funcs = &stub_funcs };
const struct partdb_entry ay891x_part  = { .name = "AY891X",  .description = "unvendored (stub)", .funcs = &stub_funcs };

/* mc6847.c's non-T1 alpha-mode path indexes font_6847[] even though we always
 * instantiate the T1 variant (font-6847t1.c IS vendored). We exclude the real
 * font-6847.c from the build and provide this zeroed placeholder so the linker
 * is satisfied; reaching it at runtime would indicate a config bug, not data. */
#include <stdint.h>
const uint8_t font_6847[768] = { 0 };

/* - - - fs_* stubs - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 * serialise.c references these for save-state I/O; that .c is excluded from
 * the build and we don't implement save-state on the Pico, so each stub is a
 * no-op returning a sentinel. They exist solely to satisfy the linker in case
 * any kept section still references them. */
#include <stdio.h>
#include <sys/types.h>

off_t   fs_file_size(FILE *fd)                                       { (void)fd; return 0; }
int     fs_truncate(FILE *fd, off_t length)                          { (void)fd; (void)length; return -1; }
uint32_t fs_file_crc32(FILE *fd)                                     { (void)fd; return 0; }
size_t  fs_file_crc32_block(FILE *fd, uint32_t *c, size_t l)         { (void)fd; (void)c; (void)l; return 0; }
int     fs_write_uint8(FILE *fd, int v)                              { (void)fd; (void)v; return -1; }
int     fs_write_uint16(FILE *fd, int v)                             { (void)fd; (void)v; return -1; }
int     fs_write_uint16_le(FILE *fd, int v)                          { (void)fd; (void)v; return -1; }
int     fs_write_uint31(FILE *fd, int v)                             { (void)fd; (void)v; return -1; }
int     fs_read_uint8(FILE *fd)                                      { (void)fd; return -1; }
int     fs_read_uint16(FILE *fd)                                     { (void)fd; return -1; }
int     fs_read_uint16_le(FILE *fd)                                  { (void)fd; return -1; }
int     fs_read_uint31(FILE *fd)                                     { (void)fd; return -1; }
int     fs_sizeof_vuint32(uint32_t v)                                { (void)v; return 0; }
int     fs_write_vuint32(FILE *fd, uint32_t v)                       { (void)fd; (void)v; return -1; }
uint32_t fs_read_vuint32(FILE *fd, int *nread)                       { (void)fd; if (nread) *nread = 0; return 0; }
int     fs_sizeof_vint32(int32_t v)                                  { (void)v; return 0; }
int     fs_write_vint32(FILE *fd, int32_t v)                         { (void)fd; (void)v; return -1; }
int32_t fs_read_vint32(FILE *fd, int *nread)                         { (void)fd; if (nread) *nread = 0; return 0; }
