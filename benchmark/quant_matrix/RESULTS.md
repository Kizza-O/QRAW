# Quant-table matrix — results

**799 quant tables screened, 23 verified on the live camera. 2026-08-16.**

Goal: the best image quality that still reaches 48 fps on live RAW12/60.

---

## Answer

**Use `1/22/22/13/34/34/22/126/126/175`** (matrix cell `M_0606`).

    live RAW12/60, 3 workers   48.5 fps   (4 workers 48.1)
    static 16-bit              60.8 fps
    rmse                       33.3 codes of 4095   (41.80 dB)
    size                       2.13 MB/frame

The only shipped rung that also clears 48 fps is **m8**, and this table beats it
by **1.21 dB** at the same frame rate. Against the m7 default it costs only
0.15 dB while turning 47.7 fps into 48.5.

If you want margin rather than maximum quality, `1/14/14/16/26/26/36/126/126/32767`
(`M_0766`) runs **51.5 fps** at rmse 35.1 — still 0.76 dB better than m8, and
faster than it too. It prunes HH1 outright, so it changes fine diagonal detail;
judge it on the images before shipping it.

---

## Why the shipped ladder could not do this

m1..m10 move all nine entries together. The matrix mixed the L3 / L2 / L1
triples across rungs independently (10x10x10), and the winners all share a
shape no single rung offers:

    m7        1 / 14 14 8  / 26 26 18 / 126 126 175
    M_0606    1 / 22 22 13 / 34 34 22 / 126 126 175
                  ^^^^^^^^^^^^^^^^^^^^   ^^^^^^^^^^^
                  coarser L3 and L2      m7's L1 kept

L1 holds three quarters of the coefficients, so it dominates both size and
sharpness — keep it fine. L3 and L2 are cheap to coarsen. The ladder cannot
express that because it scales everything at once.

---

## Measured, live RAW12/60, 3 workers

    table                                    rmse    MB   static   live
    1/32767 x9  (everything pruned)         131.5  0.31   110.8   61.1   sensor-capped
    1/29/29/13/45/45/35/272/272/521  (m10)   46.9  1.20    82.8   56.4
    1/14/14/16/26/26/36/126/126/32767        35.1  1.89    68.6   51.5   <- margin pick
    1/18/18/9/34/34/22/155/155/235   (m8)    38.3  1.91    67.4   50.0
    1/22/22/13/34/34/22/126/126/175          33.3  2.13    60.8   48.5   <- BEST AT 48
    1/14/14/8/26/26/18/126/126/175   (m7)    32.7  2.26    62.5   47.7   just misses
    1/18/18/9/26/26/18/73/73/111             21.7  2.71    57.5   44.9
    1/8/8/4/16/16/11/26/26/39                 8.6  4.29    51.6   40.4
    1/4/4/2/8/8/6/26/26/39           (m1)     7.9  4.70    51.0   39.6

Per-cell spread on the finalists was 0.3–1.1 fps, so the 48 fps calls are sound.

---

## Findings worth keeping

* **Entry 0 (LL3) is not a tunable.** `vc5_bench.cpp:2649` computes
  `base = 10 - 3*level` for levels 1..3, so only entries 1..9 are ever read.
  Overriding entry 0 leaves the output byte-identical — measured, not assumed.
* **The shipped ladder is strong at m7 and m10.** Of 799 tables, none beat
  either rung at equal-or-smaller size. m5, m6 and m8 are beatable by
  1.54 / 1.43 / 0.76 dB respectively.
* **3 vs 4 workers is a null result**, reproduced across all 23 verified
  tables (e.g. 48.5/48.1, 51.5/51.7, 47.7/47.7). Independently confirms
  `docs/BENCHMARK_CONCLUSION.md`.
* **The ceiling is the sensor, not the encoder.** The fully pruned table hits
  only 61.1 fps live against 110.8 static — capture-bound at the 60 fps cap.
  Quant tuning has real headroom up to ~60; 48 sits comfortably inside it.
* **fps on this box is not rankable at single-cell granularity.** Repeat
  measurements of an identical config spread up to 8.8 fps. `gpr_bytes` and
  `rmse` are exact functions of the quant table, so the matrix ranks on those
  and measures fps only on the short list, with replication, on a quiet
  machine. Concurrent tooling work on the same four cores was a real
  contaminant during Stage A.

---

## Method

    Stage A   71 runs   move one entry at a time off m7 -> per-entry sensitivity
    matrix   799 tables L3/L2/L1 triples mixed across m1..m10, plus HH-orientation
                        variants and a Lagrangian rate-distortion frontier
    screen   799 x 0.45 s  encode + decode the fixed reference frame; exact
                        gpr_bytes and rmse, no timing involved, 4-way parallel
    Stage C   23 tables  fps verified: static (3 passes) and live RAW12/60 at
                        3 and 4 workers (3 passes)
    Stage D   5 finalists decoded to PNG, 200% crop and amplified diff vs m7

Screening directly through `vc5_bench --save-gpr` instead of the benchmark
harness took the sweep from 2.4 hours to 2.1 minutes, which is what made a
799-table matrix affordable at all.

Quality metric is RMSE in 12-bit code values against
`sample_imx585_3840x2160_gbrg_12bit_gplog2.raw16`, the companded domain the
decoder actually emits. `gpr_decode_verify` reports PSNR against a 65535 peak
(a flat +24.08 dB); the figures here are recomputed with peak 4095.

## Files

    RESULTS.md        this file
    screen.csv        all 799 tables: table, gpr_bytes, rmse, psnr12
    master.csv        every timed run
    matrix.json       the generated tables
    view/             PNG, 200% crop and diff-vs-m7 per finalist
    raw/              decoded 16-bit frames, named by table + rmse + live fps
    screened_gpr/     799 encoded .gpr, one per table

---

# Addendum — the encoder ceiling is scheduling, not quant

## The finding

`ms_wavelet` is flat across all 799 quant tables (35-38 ms even with every
coefficient quantised to zero), so quant cannot touch it. The wavelet is
DRAM-concurrency bound. Measured with the package's own diagnostic
(`--core0-stage-normal-workers`, which varies how many whole-frame workers run
alongside a staged pair):

    concurrent wavelet streams   pair_wavelet_ms   total fps
    1                                 29.02           29.0
    2                                 34.91           37.25
    3                                 50.10           36.75

**The third wavelet stream is worthless**: +43% wavelet cost, -0.5% throughput.
The penalty tracks how far the working set overruns the 2 MB shared L3:

    frame     per worker   x3 workers   3-thread wavelet penalty
    512x272     0.28 MB      0.84 MB (fits L3)     +17.1%
    960x544     1.04 MB      3.1 MB                +32.0%
    1920x1088   4.18 MB     12.5 MB                +45.5%
    3840x2160  16.6 MB      50 MB                  +66.6%

Entropy, by contrast, scales perfectly (24.96 / 25.17 / 26.21 / 23.93 ms at
1/2/3/4 threads) because it is mispredict-bound, not memory-bound.

## The fix already exists and was never enabled

`run_cpu_gpr_adaptive_stage_pipeline` implements exactly the right shape:
two wavelet producers, two entropy consumers, work stealing, ownership handoff
with `copy_bytes=0`, and it already supports the compact direct tile-hybrid slot
so the 15.8 MB int16 coefficient frame is never written.

    ./vc5_bench ... --execution cpu-gpr --cpu-gpr-threads 4 \
        --core0-stage-pipeline adaptive --core0-stage-normal-workers 0 \
        --cpu-direct-hybrid on

Static m7, interleaved A/B, 3 reps:

    shipped frame-parallel x4    36.75 fps  (35.2-37.5)
    adaptive + direct-hybrid     56.33 fps  (55.8-57.2)   +53.3%

core0_wavelet_ms falls 45.4 -> 33.9: the contention saving is real, not a
measurement artefact.

Output is unchanged: `avg_gpr_bytes=2257842`, exactly the shipped path's
byte count, and the pipeline's own gate reports
`CPU_DIRECT_HYBRID_VERIFY bitexact=YES` (128463 int8 tiles, 57 int16 fallbacks).

The plain `dual` mode gets only +2.5% because it hands off through the int16
coefficient frame. The whole win comes from combining the staged schedule with
the compact slot.

## Before this can ship

1. `--save-gpr` and `--frames` are NOT implemented in the adaptive pipeline;
   it is benchmark-only today. Wire them up so the fail-closed decode gate can
   run against it end to end.
2. **The live path cannot use it as configured.** It puts a wavelet producer on
   Core 0, which on the live path belongs to capture/ISP/IPA. The live library
   (`encoder_library/cinepi_qraw_encoder.cpp`, 508 lines) does the wavelet at
   line 451 and the entropy at line 465 in the same worker; splitting those
   across a producer/consumer pool is the live port.
3. With Core 0 reserved for capture only three cores remain, so the live gain
   will be smaller than +53% and must be measured, not extrapolated. A plausible
   split is 2 producers on cores 1-2 with entropy on core 3 plus the existing
   Core 0 SB8x4 assist.
