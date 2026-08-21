# This repository is the encoder half

> New here? **[`ENCODER_REFERENCE.md`](ENCODER_REFERENCE.md)** is the single
> document covering platform targeting, every shipped optimisation with its
> switch, the M1–M10 ladder, precision options and the full option surface.


`cinepi-qraw-live-v4.0` was one package containing both the QRAW encoder and
the CinePi camera application that exercises it. It was split in two on
2026-08-21:

| Repository | Contents |
|---|---|
| **QRAW** (here) | The encoder, its library interface, the static/file benchmark, the quantiser matrix, the correctness gates, and the encoder findings |
| **cinepi-qraw** | The live camera: cinepi-raw and cinepi-gui integration patches, `run_live.sh`, the camera UI, the OS patch, and the live-capture findings |
| **gpr-qraw** | GoPro's GPR SDK with the QRAW modifications. Pulled in here as a submodule. |

## Documents that moved

These are referenced from documents in this repository but now live in
`cinepi-qraw/docs/`:

- `CAMERA_UI.md`
- `FINDING_LIVE_CAPTURE_RATE.md`
- `FINDING_RAW12_60FPS.md`
- `FINDING_CLEARHDR_BLACK_FRAME.md`
- `FINDING_M5_60FPS_SWEEP.md`
- `LIVE_PIPELINE_IMPROVEMENT_MATRIX.md`
- `PROMOTION_STATUS.md`

`FINDING_PLATFORM_TUNING.md` was kept here, because the DRAM-bandwidth ceiling
it documents constrains the encoder directly. It is equally relevant to the
camera repository.

## Scripts that moved

`RUN.sh`, `RUN_ALL_QRAW_OUTPUT.sh`, `RUN_CAPTURE_TEST.sh`,
`RUN_CINEPI_RAW_BENCHMARK.sh`, `RUN_CINEPI_QRAW_BENCHMARK.sh`,
`RUN_MATRIX_TEST.sh`, `RUN_CAMERA_MODE_TEST.sh`, `RUN_LEAN_BISECT.sh`,
`calibrate_wb.sh`, `setup_appliance.sh`, the `cinepi_*ui.py` files, the camera
mode benches, and all `migrate_cinepi_*.py` tools.

`RUN_BENCHMARK.sh --live` now stops with a message pointing at the camera
repository instead of failing obscurely.

## Older documents

Text written before the split may reference moved files by name without
qualification. The content is still accurate; only the location changed.
