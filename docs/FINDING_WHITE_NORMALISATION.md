# Settled: the +278,080 byte library/benchmark discrepancy

> **Dated record, not a specification.** The figures below were measured on one
> CM5, one scene and one software state, on the date given. Throughput, file size
> and compression ratio are properties of a scene and a machine: a noisy frame
> compresses less at every quality mode, and a different Pi, kernel, thermal
> state or desktop session moves the numbers again. They are kept so a change can
> be compared against the same conditions — do not read them as what your rig
> will do, and do not quote them as limits.


INTEGRATION_STATUS.md in the v1.11.2 package marked the encoder library
"NOT YET CORRECT": same input, same mode, same machine, the library's m5
output was 3,330,730 bytes against the benchmark's 3,052,650 -- +278,080,
consistently -- with an open hypothesis that the relative
`validated_input/gpr_params.json` path failed to resolve and the container
fell back to different metadata.

## The hypothesis is refuted

Running the documented settling procedure on x86-64 (both binaries built
from the package's own sources):

1. With CWD such that the relative path **does** resolve, the library still
   produces 3,330,730 bytes. Container metadata was never the variable.
2. With CWD such that it does **not** resolve, `gpr_parameters_parse`
   returns -2, `DirectGprEncoder`'s constructor throws, and
   `cinepi_qraw_create` returns NULL. There is no silent fallback to
   different container parameters; the failure mode the hypothesis needed
   does not exist in the code.

## The actual cause

`vc5_bench.cpp`, argument post-processing in `main()`:

    if (!o.white_set) o.white = int((uint32_t(1) << unsigned(o.effective_bits)) - 1u);

With the default `effective_bits = 16`, every benchmark run encodes with
**white = 65535**. The bundled sample's own SAMPLE_INPUT.json records this:
"Both macOS and Pi benchmark this bundled sample with --black 0 --white
65535."

The library face copies `Options` field by field and never runs that rule.
`cfg->white == 0` therefore leaves the `Options` **field default of 4095** --
a value that only ever made sense pre-normalisation. The GP-Log2 LUT
(`make_lut`) then compands input range [black..4095] instead of
[black..65535]: everything above code 4095 clips to `working_max`, the
coefficient statistics change completely, and the entropy coder emits
+278,080 bytes on this sample (ratio 4.98 vs 5.43).

Why the package's own audit missed it: INTEGRATION_STATUS.md verified
"white (4095) -- identical in both". It compared the library against the
`Options` *field default*, which is 4095 on both sides -- but the benchmark
never encodes with the field default; it encodes with the post-parse value.
The audit checked the wrong reference.

## Evidence

    library, cfg.white = 0 (unfixed)   : 3,330,730 bytes   ratio 4.98
    library, cfg.white = 65535         : 3,052,650 bytes   ratio 5.43
    vc5_bench --mode m5 --execution cpu-gpr : gpr_bytes=3052650 ratio=5.43423
                                              (vc5_bytes=3046392)

3,330,730 - 3,052,650 = 278,080 -- the documented delta, exactly.

`encoder_library/white_ab.c` reproduces this in one command and doubles as
the regression test for the fix (against the fixed library both legs must
read 3,052,650).

## The fix

`cinepi_qraw_encoder.cpp`, in `cinepi_qraw_create()`:

    if (cfg->white > 0) o.white = cfg->white;
    else                o.white = (1 << o.effective_bits) - 1;   // main()'s rule

This is a contract repair, not a behaviour change: the header always
documented `white; 0 = (1<<effective_bits)-1`. The fixed file also adds the
`black` field (the LUT subtracts `o.black`, and real sensor frames carry a
pedestal the bundled sample does not -- see qraw_encoder.cpp, which feeds it
from libcamera's `SensorBlackLevels`), and a black/white range guard so an
inverted range fails at `create()` instead of dividing by zero inside
`make_lut`.

## What this does and does not clear

It clears the library for driving the live pipeline and for numbers that
are size-comparable with the campaign's. It does **not** yet prove
byte-identity of the container (see VALIDATION_STATUS.md item 1), and per
the package's own rule the library should not produce archival footage
until that gate and the real-sensor black-level gate are green.
