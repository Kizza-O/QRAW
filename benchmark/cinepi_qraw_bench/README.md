# CinePi Canonical GPR Encoder

`vc5_bench.cpp` is the sole native encoder source and identifies as v1.16.6. The live library includes this exact file rather than maintaining a second encoder copy.

Production execution is CPU-GPR only, with the original winning v1.16.6 settings, VLE128/locality0, and external IRQ isolation to Core0. The runtime wrappers fail closed rather than falling back to another encoder.

Build the canonical static encoder:

```bash
./build_static_winner_fast.sh
```

Run package gate:

```bash
../../tools/check_single_encoder.sh
```
