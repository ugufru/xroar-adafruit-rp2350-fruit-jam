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
