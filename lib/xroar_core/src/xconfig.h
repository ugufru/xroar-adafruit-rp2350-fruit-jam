/*
 * xconfig.h — minimal stub for the RP2350 port (FRUITJAM-09).
 *
 * Upstream xconfig.h is XRoar's command-line/option-parsing framework. The
 * only vendored core header that includes it is machine.h, and it references
 * exactly one type from it: `struct xconfig_enum`, used solely in a handful of
 * `extern struct xconfig_enum machine_*_list[]` declarations. None of the
 * compiled translation units read or index those arrays, so a lightweight
 * complete definition (matching upstream's field shape) is all that's needed.
 *
 * Kept as glue rather than editing the frozen machine.h, in the same spirit as
 * the stub xroar.h / logging.h headers.
 */
#ifndef XROAR_PORT_XCONFIG_H_
#define XROAR_PORT_XCONFIG_H_

/* Upstream shape: a name/description -> integer-value enumeration entry.
 * Provided complete so any incidental use compiles; the port never populates
 * or reads these lists. */
struct xconfig_enum {
	int value;
	const char *name;
	const char *description;
};

#endif
