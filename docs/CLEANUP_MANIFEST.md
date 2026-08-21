# Single Encoder Cleanup Manifest

Canonical encoder source: `benchmark/cinepi_qraw_bench/vc5_bench.cpp` v1.16.6.
Live library: `encoder_library/cinepi_qraw_encoder.cpp` includes that exact source and forces the winning settings.
Runtime standard: IRQs -> Core0 + VLE128/locality0.

Removed as non-production experiment surface:

- CPU wavelet lab and its copied kernel header;
- Vulkan/shader optimisation variants and shader crosschecks;
- M7 factorial and optimisation sweep runners;
- historical v1.7 experimental architecture notes;
- alternate benchmark harness/build paths that were not needed by the winner.

Low-level `third_party/gpr/source/lib/vc5_encoder` remains because it is a required internal library dependency of the canonical winner. Its standalone encoder application is not shipped or built. Decode tooling remains decode-only.

Quant cleanup in this revision:

- Previous M1-M10 production table removed;
- historical v19 ladder removed from source;
- `--mode-ladder` selector removed;
- old Mac reference quant-table JSON removed;
- one canonical `validated_input/canonical_mode_quant_tables.json` remains.
