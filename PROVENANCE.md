# Provenance of vendored third-party code

This file records the exact origin of any third-party source vendored into this
repo, so the lineage stays auditable. See `THIRD_PARTY_LICENSES.md` for licenses.

## lib/pico_hdmi — HSTX-native DVI/HDMI output

- **Upstream:** https://github.com/fliperama86/pico_hdmi
- **Version:** tag `v0.0.7`, commit `f8034c7ae5e4927692229d114b84b6e81cc789d9`
- **Vendored:** 2026-07-02, for FRUITJAM-04 (HSTX DVI bring-up).
- **License:** The Unlicense (public domain) — see `lib/pico_hdmi/LICENSE`.
- **What was taken:** the full `src/` and `include/` trees plus `LICENSE` and
  `README.md`, unmodified.
- **Build selection:** `lib/pico_hdmi/library.json` excludes `src/video_output_rt.c`
  from compilation. `video_output_rt.c` is the newer *runtime-modes* implementation
  of the same public API as `video_output.c` (identical function names) — the two
  are mutually exclusive, exactly as upstream's `CMakeLists.txt` selects one via
  the `PICO_HDMI_RUNTIME_MODES` option (default OFF → `video_output.c`). We mirror
  that default: the standard `video_output.c` scanline-callback path. The file is
  kept on disk for provenance fidelity but is not built.
- **Configuration:** built with `-DMODE_HSTX_CLK_DIV=2` (set in the PlatformIO
  env) so that, with our `clk_sys = 252 MHz`, the library derives
  `clk_hstx = 252/2 = 126 MHz` — the value FRUITJAM-03 measured. Upstream's
  default (`MODE_HSTX_CLK_DIV=1`) assumes a 126 MHz system clock, which we do not
  use (we run 252 MHz for PIO-USB + emulation).
- **Local modifications:** none to the vendored files. All integration lives in
  our own `src/display/`.

## lib/xroar_core — XRoar emulation core (CPU / SAM / PIA / VDG / events)

- **Upstream:** XRoar by Ciaran Anscomb, https://www.6809.org.uk/xroar/ — local
  clone `~/github/xroar`.
- **Version:** release tag **`1.11`**, commit
  `9e929dba135fbdaa7c4e5c58a55729b0b0e272f0`.
- **Vendored:** 2026-07-02, for FRUITJAM-09 (vendor the XRoar 1.11 core).
- **License:** GPL-3.0-or-later (same as this project) — see `src/COPYING.GPL`
  in upstream; also `THIRD_PARTY_LICENSES.md`.
- **Extraction method:** each emulation-core file was pulled **fresh from the
  `1.11` tag** via `git archive 1.11 -- <path>` — deliberately *not* copied from
  the drifted core copies in the prior RP2350 port repos. The prior ports were
  used only as an *insight* into which files form a minimal buildable set (the
  file manifest) and how to write the port glue; none of their (mutated) core
  content was taken. The 58 extracted files under `src/` and `portalib/` are
  **byte-identical to upstream tag 1.11** (verified by diffing every file against
  `git show 1.11:<path>`), with the single exception noted below.
- **Modules:** mc6809 (+mc680x shared ops), mc6847 (VDG + fonts), mc6883 (SAM),
  mc6821 (PIA), events, part, ram, clock, serialise, and a portalib subset
  (delegate, slist, sds, xmalloc, pl-string, etc.).

### The one authorized deviation from verbatim 1.11
- **`src/serialise.h`** — added `default: ser_type_unhandled` to the *inner*
  `_Generic` in the `ser_type_for(m)` macro, with an `RP2350 port:` comment.
  Rationale: the arm-none-eabi/newlib toolchain type-checks every association of
  a `_Generic` (including the unselected `default` branch); `mc6809.c` serialises
  a member of type `unsigned int` (`MC6809.state`) which matches none of the
  inner associations, so upstream 1.11's defaultless inner `_Generic` is
  ill-formed here. `ser_type_unhandled` is already upstream's designed
  "returns control to caller" escape value, so this is semantically safe and
  minimal. This is the **only** content change to any extracted core file.

### Port glue authored for this repo (NOT from upstream core)
These are build scaffolding, modeled on the prior-port pattern — not emulation
core:
- `include/config.h`, `include/top-config.h`, `include/hot.h` — hand-written
  replacements for the autotools-generated config; `hot.h` provides `HOT_FUNC`
  (`.time_critical` SRAM placement under `PICO_BUILD`) without pulling in
  `pico.h` (which would macro-ise `bool` and break `delegate.h` token pasting).
- `src/xroar.h`, `src/logging.h` — minimal stubs replacing upstream's "everything"
  headers (which `#include` unvendored `ui.h`/`xconfig.h`).
- `src/xconfig.h` — minimal stub providing `struct xconfig_enum` so the frozen
  `machine.h` compiles unmodified.
- `src/xroar_stubs.c` — link-time no-op definitions (`logging` global, `fs_*`
  save-state stubs, placeholder `partdb` entries for unvendored parts, VDG font).
- `library.json` — PlatformIO build recipe (which core `.c` compile + include
  paths).
