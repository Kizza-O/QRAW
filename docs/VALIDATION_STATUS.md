# Validation Status

Validated for this package revision:

- canonical `vc5_bench.cpp` remains v1.16.6 and is SHA-pinned after the quant-constant promotion;
- `tools/check_single_encoder.sh` passes and identifies Universal Standard Quant v3;
- exactly one M1-M10 quant ladder exists: CinePi Universal Standard Quant v3;
- no Quant v2 table, legacy `--mode-ladder` selector or old reference-table file remains;
- the built-in encoder self-test checks all ten production tables exactly and validates every divisor through the packed codec/GPU quantizer path;
- the canonical standalone target builds and completes its self-test on the host validation environment;
- UI selection regression tests pass;
- live encoder library wrapper compiles against the same canonical source and has no fallback encoder path;
- production runners lock VLE prefetch to 128/locality0 and apply IRQ isolation to Core0;
- All QRAW Camera Output enumerates every capture path x effective precision x band setting x
  quality mode, and one leg of each of the four capture paths was run on the live camera with
  every kept frame passing the fail-closed decode gate;
- white balance is measured from the camera rather than inherited: `calibrate_wb.sh` writes
  `cinepi_wb_base.json`, camera runs seed the manual-3A preview gains from it, and the GPR/DNG
  AsShotNeutral is stamped from it instead of from the stock GoPro parameter file. Verified on a
  ClearHDR capture: the frame's AsShotNeutral matches the measured base, and `CAPTURE_CONFIG`
  records which base a run used.

Pi/CM5 performance still must be measured on target hardware; host validation proves source/build integrity, not Pi throughput.
