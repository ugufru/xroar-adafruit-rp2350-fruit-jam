/*
 * xroar.h — minimal stub for the RP2350 port.
 *
 * Upstream xroar.h is the global app state for the desktop XRoar. We only
 * need it to satisfy `#include "xroar.h"` in part.c / serialise.c. The
 * actual symbols those TUs reference from xroar.h are zero — the include
 * looks vestigial. Keep this header tiny on purpose.
 */
#ifndef XROAR_PORT_XROAR_H_
#define XROAR_PORT_XROAR_H_

#include <stdint.h>
#include <stddef.h>

/* mc6821.c (and other timed modules) initialise events against the global
 * machine event list. Upstream this is `xroar.machine_events` — we expose
 * it as a standalone pointer instead. Definition lives in xroar_stubs.c. */
struct event_list;
extern struct event_list *machine_event_list_global;
#define MACHINE_EVENT_LIST machine_event_list_global

#endif
