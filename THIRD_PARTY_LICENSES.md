# Third-party licenses

This project (GPL-3.0-or-later, matching upstream XRoar) bundles the following
third-party code. See `PROVENANCE.md` for exact versions and commits.

---

## lib/pico_hdmi — fliperama86/pico_hdmi

**The Unlicense** (public domain dedication). Full text in
`lib/pico_hdmi/LICENSE`:

> This is free and unencumbered software released into the public domain.
>
> Anyone is free to copy, modify, publish, use, compile, sell, or distribute
> this software, either in source code form or as a compiled binary, for any
> purpose, commercial or non-commercial, and by any means.
>
> [...] THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND [...]

The Unlicense places the code in the public domain, so it is compatible with
this project's GPL-3.0-or-later license.

---

## lib/xroar_core — XRoar emulation core

**GPL-3.0-or-later** — the same license as this project (this repo is a port of
XRoar). Copyright © Ciaran Anscomb and contributors. Upstream:
https://www.6809.org.uk/xroar/ ; full license text in upstream `COPYING.GPL`.
Vendored from release tag `1.11`; see `PROVENANCE.md` for the exact commit and
the two documented portability edits (`serialise.h` `_Generic`, `mc6847.c`
`SUPPRESS_RENDER_SCANLINE`).

---

## carlk3/no-OS-FatFS-SD-SDIO-SPI-RPi-Pico — SD/FatFs driver (build dependency)

**Apache-2.0.** Pulled at build time via PlatformIO `lib_deps` (not vendored into
this repo, so its sources are not redistributed here). Our `src/sd/hw_config.c`
is the board pin-map config for that driver, adapted from its example template;
it carries this project's GPL-3.0-or-later header. Apache-2.0 is one-way
compatible with GPL-3.0-or-later.
