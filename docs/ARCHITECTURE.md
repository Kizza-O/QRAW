# Architecture

The map, above the terrain. The findings documents in this directory record how
each decision was reached; this one says what the thing actually is.

---

## The pipeline

One frame, start to finish:

```
  sensor
    │  16-bit Bayer, tightly packed, MSB-justified
    ▼
  ┌─────────────────────────────────────────────────────┐
  │  1. SPLIT                                           │
  │     Bayer → four component planes: GS RG BG GD      │
  │     GP-Log2 companding applied in the same pass     │
  └─────────────────────────────────────────────────────┘
    │  four planes, companded, half sensor dimensions each
    ▼
  ┌─────────────────────────────────────────────────────┐
  │  2. WAVELET                                         │
  │     Three-level 2/6 transform, per plane            │
  │     Fused cascade: one pass, not three              │
  └─────────────────────────────────────────────────────┘
    │  ten bands per plane: LL3, then L3/L2/L1 × LH HL HH
    ▼
  ┌─────────────────────────────────────────────────────┐
  │  3. QUANTISE                                        │
  │     M1–M10 ladder, CAQ per component,               │
  │     Pixel Clean and Noise Clean thresholds          │
  │     Fused into the entropy scan, not a separate pass │
  └─────────────────────────────────────────────────────┘
    │
    ▼
  ┌─────────────────────────────────────────────────────┐
  │  4. ENTROPY                                         │
  │     VC-5 Table 17 variable-length coding            │
  └─────────────────────────────────────────────────────┘
    │  VC-5 bitstream
    ▼
  ┌─────────────────────────────────────────────────────┐
  │  5. CONTAINER                                       │
  │     Spliced into a retained DNG/GPR template        │
  └─────────────────────────────────────────────────────┘
    │
    ▼
  .gpr — ordinary GPR, readable by any stock decoder
```

`cinepi_qraw_last_timings()` reports stages 2, 3–4 and 5 separately as
`wavelet_ms`, `entropy_ms` and `dng_wrap_ms`. The third is included in the
second and is the only part serialised across the whole process.

---

## The four components

The split is GPR's own, not something QRAW invented:

| | What it is |
|---|---|
| **GS** | Green sum — the mean of both green phases. The sharpest and least noisy plane; the picture's detail lives here. |
| **RG** | Red minus green, a colour residual |
| **BG** | Blue minus green, a colour residual |
| **GD** | Green difference, between the two green phases |

This ordering is why the quality levers work the way they do. RG and BG carry
colour residual rather than luminance detail, so bits spent there buy less
visible quality than bits spent on GS. CAQ exploits exactly that: it quantises
RG and BG harder, GD more gently, and leaves GS alone until the strongest
setting.

---

## Where the levers act

All three levers change what gets stored, none change the container format:

| Lever | Acts at | Mechanism |
|---|---|---|
| **Mode** (M1–M10) | stage 3 | The quantiser divisor per band |
| **CAQ** | stage 3 | A per-component multiplier on those divisors |
| **Pixel Clean** | stage 3 | A widened zero threshold, resolved to one integer per band |
| **Noise Clean** | stage 3 | A further widened threshold on selected level-1 bands |

Every one of them resolves to an integer comparison inside the quantiser, which
is what makes them nearly free. Because the quantiser is monotone in `|coeff|`,
"dead-zone at 125% of the half-step" and "zero any quantised ±1 on these bands"
collapse into a single `|coeff| <= T` test computed once at setup.

The same widened threshold then feeds the encoder's provable-zero skips: a
coefficient known to quantise to zero does not need its entropy work done. That
is why Pixel Clean and Noise Clean make the encode *cheaper* as well as the file
smaller, instead of trading one against the other.

---

## Threading

```
  camera ──► encode queue ──► N worker threads ──► disk queue ──► writer
```

One encoder object per worker. Each is single-threaded, owns roughly 20 MB of
scratch, and shares nothing with the others except a process-wide mutex inside
the Adobe DNG container writer — which the retained-template splice avoids after
the first few frames.

**Four workers at RAW12**, one per core: 45.5 fps against 40.5 for three, with
fewer drops. The camera thread costs only 6.8% of a core when there is no byte
swap to do, so Core 0 has the headroom. Pair it with a zero-copy inflight limit
of 8, or the queue gate becomes the thing being measured rather than the worker
count.

**Three workers at RAW16.** libcamera byte-swaps 16.8 MB per frame on its camera
thread there, roughly 10 ms of a 33 ms budget, and a fourth encode worker on
Core 0 starves capture.

Movable IRQ affinities are relocated to Core 0 during encode and restored on
exit. That is an external scheduling condition of the measured configuration,
not another encoder path.

---

## What limits it

The encoder is **DRAM-bound, not compute-bound**, on a Pi 5. This is the single
most important fact about its performance and it shaped most of the
optimisation work:

- Non-temporal (`STNP`) coefficient writes, to avoid polluting cache with data
  that will not be read again
- A fused wavelet cascade, so intermediate levels never round-trip to memory
- The GP-Log2 split folded into that same cascade
- In-place container writes, so the VC-5 payload is not copied into the GPR
- Tuned entropy prefetch distance at streaming locality

The clearest demonstration is external: an attached 3440×1440@60 desktop costs
about 7% of encode throughput, because it reads roughly 1.19 GB/s out of the
same DRAM continuously. Nothing about the encoder changed; it simply had less
bandwidth to work with.

This is also why 60 fps is currently out of reach — the ceiling sits near 55 fps
at the heaviest compression — and why the RK3588 is the interesting next
platform, at roughly twice the memory bandwidth.

---

## Source layout

| Path | What lives there |
|---|---|
| `encoder_library/` | The C interface. `cinepi_qraw_encoder.h` documents every configuration field. |
| `benchmark/cinepi_qraw_bench/vc5_bench.cpp` | The canonical encoder implementation, pinned to a SHA-256. The library includes it rather than copying from it, so there is exactly one encoder in the package. |
| `benchmark/quant_matrix/` | How the M1–M10 ladder was derived |
| `benchmark/third_party/gpr` | Submodule: GoPro's SDK with QRAW's changes |
| `tools/` | Verification gates, calibration, test-set generation |
| `docs/` | The campaign record |

The library deliberately `#include`s the benchmark source rather than
duplicating it. A second copy would drift within a week, and the correctness
gates would then be proving something other than what ships.

---

## Where to read next

- `ENCODER_REFERENCE.md` — every option, profile table and measured number
- `BENCHMARK_CONCLUSION.md` — the summary of record for the campaign
- `OPTIMISATIONS.md` — what was tried, what won, what was rejected
- `FINDING_PLATFORM_TUNING.md` — the bandwidth ceiling, measured
- `BUILD_PI5.md` — building, including the 16 KB page size
