# GPR became QRAW (v0.1, 2026-08-19)

QRAW is the name of this encoder. Everything of ours that said GPR now says
QRAW: source files, types, functions, the static library, environment
variables, redis keys, the shared-memory segment, log messages, on-screen
strings, scripts, directories and this package.

## What was NOT renamed, and why

Three groups keep the GPR name because they are not ours:

| Kept | What it is |
|---|---|
| `.gpr` extension, "GPR container" | the camera writes GoPro VC-5 GPR files; every third-party decoder identifies them that way |
| `benchmark/third_party/gpr/`, `libgpr_sdk.a`, `gpr_params.json`, `gpr_tools` | the GoPro GPR SDK, vendored unmodified apart from the CAQ per-channel quant hook |
| `gpr_dng_export.py`, `gpr2dng.sh`, `gpr_validation.py`, `gpr_view`, `gpr_decode_verify` | tools named after the container they read |

`mjpegPreviewStage.cpp` contains the letters "gPr" and was explicitly protected
from the substitution.

## The renames that matter operationally

| Was | Now |
|---|---|
| `CINEPI_GPR_*` (THREADS, MODE, BITS, STABLE, ZEROCOPY_INFLIGHT, ...) | `CINEPI_QRAW_*` |
| `GPR_MODE`, `GPR_BITS`, `GPR_THREADS` (launcher variables) | `QRAW_MODE`, `QRAW_BITS`, `QRAW_THREADS` |
| `ENCODER=gpr`, codec key value `"gpr"` | `ENCODER=qraw`, `"qraw"` |
| redis `gpr_mode`, `gpr_band`, `gpr_caq`, `gpr_stable`, `gpr_workers` | `qraw_*` |
| `/dev/shm/cinepi_gpr_live`, magic `GPRS` | `/dev/shm/cinepi_qraw_live`, magic `QRAW` |
| `CinepiGprEncoder`, `cinepi_gpr_*()`, `libcinepi_gpr.a` | `CinepiQrawEncoder`, `cinepi_qraw_*()`, `libcinepi_qraw.a` |
| `/opt/cinepi-gpr` and `/home/pi/cinepi-gpr-fixed` (two prefixes) | `/opt/cinepi-qraw` (one) |
| meson option `gpr_prefix` | `qraw_prefix`, defaulting to `/opt/cinepi-qraw` |
| `gpr_encoder.cpp/.hpp`, `gpr_live_stats.hpp` | `qraw_encoder.cpp/.hpp`, `qraw_live_stats.hpp` |
| `cinepi_gpraw_bench/`, `run_gpr_pipeline.sh`, `gpr_stats_cli`, `build_gpr_core.sh` | `cinepi_qraw_bench/`, `run_qraw_pipeline.sh`, `qraw_stats_cli`, `build_qraw_core.sh` |
| package `cinepi-gpr-live-v4.0/` | `cinepi-qraw-live-v4.0/` |

`CINEPI_CAQ` was already CAQ-named and did not change.

## Old spellings still work

`run_live.sh` accepts `ENCODER=gpr` (mapped to `qraw`) and forwards any
`GPR_*` / `CINEPI_GPR_*` variable to its `QRAW_*` name, printing a note on
stderr. Older measurement recipes in `docs/` therefore keep running, but the
`QRAW_*` spelling is the documented one.

## The two prefixes were consolidated

The library used to live at `/home/pi/cinepi-gpr-fixed` while
`meson_options.txt` defaulted to `/opt/cinepi-gpr`, and the two copies had
drifted two days apart -- a fresh `meson setup` with defaults linked the older
encoder (no CAQ) while everything on screen claimed otherwise. Both are now the
single prefix `/opt/cinepi-qraw`, which is also the meson default, so the trap
is gone.

## Verification

* `tools/verify.sh --library` -- **crc32=72f44899 on 2988814 bytes**, byte for
  byte what it was before the rename, across all five encode paths. The rename
  changed no encoder behaviour.
* `benchmark/cinepi_qraw_bench/vc5_bench.cpp` was re-pinned in
  `tools/check_single_encoder.sh`. The change is **provably rename-only**:
  inverting the substitution map reproduces the previous pin
  `89ed1b06...` byte for byte.
* `tools/check_single_encoder.sh` and `tools/check_cinepi_raw_integration.py`
  both PASS.
* Live: camera and GUI launch from the renamed package, record to the renamed
  shm segment, respond to the renamed redis keys, CAQ still engages
  (`settings id: m5-c12-b12-w4-caqX`), and a 30 fps take recorded 359 frames
  with **zero drops and zero missed**.
* `CinePi OS Patch/cinepi_patch.sh` reports 17 steps already correct against
  the renamed prefix and step ids (`qrawcore`, `qrawparams`).
