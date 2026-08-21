# Benchmark Conclusion — v4.0 (final)

> **Dated record, not a specification.** The figures below were measured on one
> CM5, one scene and one software state, on the date given. Throughput, file size
> and compression ratio are properties of a scene and a machine: a noisy frame
> compresses less at every quality mode, and a different Pi, kernel, thermal
> state or desktop session moves the numbers again. They are kept so a change can
> be compared against the same conditions — do not read them as what your rig
> will do, and do not quote them as limits.


**Closed 2026-08-16.** This concludes the CinePi QRAW/VC-5 benchmark campaign.
Every open A/B has been measured and every item in the improvement matrix is
now either shipped, rejected, or blocked on work outside this package. No
benchmark question remains open at v4.0.

Full evidence and derivations live in `../docs/LIVE_PIPELINE_IMPROVEMENT_MATRIX.md`.
This file is the summary of record.

---

## Final measured state

Encoder: **v1.16.6 winning stack + IRQ isolation to Core 0 + VLE128/locality 0**,
Universal Standard Quant v3, direct tile-hybrid + `wav_nzmask` on both paths.
Kernel: **16 KB pages (`kernel_2712.img`)**.

    capture format                   m1        m2        m7
    static reference               49.4        --        --
    RAW12 / 60 fps sensor          34.3      34.2      37.5
    ClearHDR 16-bit linear         30.5      30.4      30.3
    ClearHDR 12-bit gradation      30.5        --        --

Static m1 began this work at 30.7 fps and finished at 49.4. Live RAW12 low-M
modes went 24.4 → 34.3. Both ClearHDR deliveries sit at the 30 fps **sensor**
ceiling with the encoder only 57% busy — capture-limited, not encoder-limited.

Static reference points on the final pinned binary (dae01b5c1789), 4 workers,
16-bit, no disk I/O: **m10 79.7 fps** (ratio 13.88), **m7 60.6 fps**
(ratio 7.39, gpr_bytes 2244822).

---

## What was won

| Change | Measured |
|---|---|
| **Direct tile-hybrid** — the 16.6 MB int16 coefficient frame never exists; ~33 MB/frame of DRAM traffic stops | static +3%, live RAW16 +8%, **live RAW12 +15%** (29.56 → 34.07) |
| **One config wins everywhere** — the nonzero mask falls out of a UMAXV the wavelet already did, so direct-hybrid now beats the sidecar on *both* stages at *every* mode | m1 39.9 → 48.3, m2 40.5 → 49.0, m5 46.1 → 53.4, m7 53.4 → 58.4 |
| **Staging copy moved off the Core 0 event loop** onto encoder workers (RAW12) | copy 22.5 → 7.6 ms; workers 69 → 75% busy |
| **Wavelet rendezvous selectable, default OFF for live** | +4.6% (25.02 → 26.18) |

The largest win, direct tile-hybrid, had been fully implemented and bit-verified
inside `vc5_bench.cpp` for a long time but was **unreachable from the library the
camera actually used** — it looked like only +3% on the static bench, which is
presumably why it was never promoted. It is worth 5× that on the bandwidth-bound
live path. Output is byte-identical throughout: `tools/verify.sh` reports
crc32=72f44899 across all five emit paths.

---

## What was settled and closed

* **The fixed entropy cost is BRANCH MISPREDICTION, not reader bandwidth.** The
  entropy emitter is 27% of cycles, **88% of all branch mispredicts in the
  process**, and only 3.1% of L1D refills; 25–30% of the stage is mispredict
  recovery. Whole-run IPC 1.84, backend stall 37.6%, frontend 1.2%.
* **Streaming / L2-tiled entropy is REJECTED on its own arithmetic** — entropy is
  2/3 *fixed* cost, not proportional to output bytes, so the premise fails.
* **Reader selectors do not help.** pair 12.07 ms / dense 12.04 ms / v20 15.81 ms
  at m10 at matched ratio. The shipped default already wins.
* **Band pruning (E0–E8) is measured.** E1 (HH1) is the winner among the
  quality-changing candidates — best fps per unit of image information destroyed
  (0.46 vs 0.30/0.28/0.24) at every mode. E6 is dead. E3 is a ceiling result, not
  a shipping candidate. E5 already shipped as `wav_nzmask`. **The whole
  entropy-side programme caps at ~+23% because `ms_wavelet` never moves** — which
  is why E4/E8 were not pursued.
  * E1 on the fixed reference frame, the quality number that counts:
    m5 54.00 → 60.00 fps (+11.1%) for −13.7% data, PSNR 43.49 dB, RMS 27.4 codes
    of 4095, 52.8% of pixels changed, diagonal (HH) energy retained 69.2%.
    E1 remains **experimental and off (E0) by default** — it changes the image.
* **Workers 3 vs 4 is a NULL result** across 30 runs (±3%, no consistent sign).
  Earlier claims in *both* directions were noise. The Core 0 SB8x4 assist is
  neither paying nor hurting.
* **Item E, 16 KB pages: NEUTRAL** — see below.

---

## Item E: the 16 KB-page kernel — closed NEUTRAL, and the lesson from it

Three legs, one binary (16 KB → 4 KB → 16 KB), identity re-checked read-only
before each leg, nothing rebuilt:

    leg  kernel        conditions   m10 fps (spread)  m7 fps (spread)
    1    16KB (2712)   long uptime  75.90 (0.60)      58.50 / 57.80 (3.60) / 58.03 (1.10)
    2    4KB  (v8)     fresh boot   79.70 (1.80)      60.50 (0.30)
    3    16KB (2712)   fresh boot   79.73 (1.10)      60.70 (0.60)

Legs 2 vs 3 — different page size, matched conditions — differ by **0.04% at m10**
and 0.33% at m7, far inside the pass spreads. The predicted ≤5% sequential,
10–15% on the 133 MB staging pool, and up to 4× strided **did not materialise at
all**. The box stays on 16 KB because that is what is booted, not because it is
faster.

**The durable lesson is methodological, and it is the most reusable thing in this
campaign:** legs 1 and 3 are the *same kernel* and the *same binary* yet differ by
5.0% at m10. On this box the **between-boot reproducibility (~5%) is worse than
the effect being measured**, even though the within-run pass spread is 0.8–2%.
The spread the runner prints measures noise *within* a boot and systematically
understates the error bar on anything compared *across* boots or across hours of
uptime. Match boot state on every leg — not just thermals and load — and never
skip the return leg. Without leg 3 this would have been published as a real 5%
win for 4 KB and the kernel rolled back for nothing.

This campaign produced **three** ways to get a false result, all of them worth
remembering:

1. **A silent rebuild between legs.** `build_and_verify.sh` fingerprints the
   SOURCES, so an un-benchmarked source edit guarantees a rebuild on the next
   run. This voided the first attempt entirely.
2. **Code layout.** A rebuild proven not to change the output still moved m7
   60.60 → 57.8–58.0. This stage swings ±13 ms/frame on layout alone.
3. **Machine state across boots.** Item E, above.

Any figure quoted across a rebuild or across a boot is suspect unless all three
were controlled.

---

## Not implemented — and why they are not benchmark questions

* **A. RAW16 software endian swap** — needs `src_byteswap` first.
* **B. ISP re-reading the full RAW every frame** (up to 454 MB/s) — preview costs
  this regardless of size.
* **C. Default worker selector 3 for camera sources** — superseded; workers 3 vs 4
  is a null result.
* **D. `g_v2_sidecar_zskip` never set in the library wrapper.**
* **E. 16 KB-page kernel** — **CLOSED, neutral.**
* **F. Unpin the QRAW disk writer from Core 0** — `CINEPI_DISK_STREAM=1` is an
  env-only A/B worth trying first.

**A and B are the only two remaining changes large enough to move the live number
materially, and both require patching libcamera or the sensor driver rather than
this package.** That is the honest reason the campaign concludes here: the
encoder is no longer the limit.

At m7 the live path is at ~55% of static, and the remaining per-frame budget is
DRAM traffic and Core 0, in that order:

    CFE write        454 MB/s   irreducible (that is the capture)
    ISP read         454 MB/s   item B — preview costs this regardless of size
    staging copy     884 MB/s   RAW12 only; now off Core 0, still on the bus
    wavelet          884 MB/s   encoder
    entropy          516 MB/s   encoder, scales with output bytes

The underlying mechanism, reproduced with **no camera running at all**: single-core
NEON streaming hits 97% of the LPDDR4X-4267 ceiling, but adding cores *reduces*
aggregate throughput (16.5 → 11.5 GB/s read from 1 → 4 threads). Three concurrent
streams see ~2.3 GB/s each. The memory controller is healthy; concurrency is the
problem.

Do not re-litigate the rejected hypotheses table in the matrix doc — dma-buf TLB
thrash, non-temporal stores, shallower staging, RAW12 zero-copy via LUT,
CSI2-packed 12-bit, core isolation, worker stagger, hugepages, and removing
`numa=fake=8` are all measured and closed. Two are traps worth restating:
**never `numactl --membind` this pipeline** (3-thread copy 7.05 GB/s interleaved
vs 3.4 GB/s pinned), and **hugepages are impossible here** —
`CONFIG_TRANSPARENT_HUGEPAGE` is unset in both installed kernels.

---

## How to reproduce the reference numbers

From this directory, on a **cold boot**, after waiting for the desktop to go
quiet (packagekitd runs ~2 min post-login and will depress pass 1):

```bash
MODES=m10 PASSES=3 DURATION=10 THREADS=4 INPUT_BITS=16 \
  SAVE_QRAW=off VERIFY=off ./RUN_BENCHMARK.sh
```

Preconditions, all of which must hold or the number is not comparable:

    .static_winner_verified  dae01b5c178995a6c0ab089d8b160a7c94001d2a7fcf6df640378de5c210360c
    md5sum vc5_bench         957d85dc9c22e2a765eca0aa21ac0413
    stat -c%s vc5_bench      2288088
    sha256 vc5_bench.cpp     62079c4220bd009829d019c0b59da2f6c791079b462dd6ce4142f0567840a0a0
    reference frame md5      566f48084010a0fbe301552897ad9425
                             (cinepi_qraw_bench/input/sample_imx585_3840x2160_gbrg_16bit.raw16)

* `PASSES=3`, never 1 — the printed spread is the noise floor for that boot.
* **Weight m10, not m7.** m10 resolves ~1%; m7 only ~2–4%.
* Sanity: ratio **must** be 13.88 at m10 and 7.39 / gpr_bytes 2244822 at m7. The
  reference frame is fixed, so a different ratio means a different input or mode
  and the comparison is void.
* `unset CINEPI_BAND_Q` and confirm no run header prints a `band:` line — every
  number in this file is E0 control, the shipped encoder.
* If a pass shows `timeout_releases` in the hundreds, discard that **pass** and
  re-run it; that is a wavelet-rendezvous stall, not the effect under test.
* If `build_and_verify.sh` says anything other than `REUSE ... dae01b5c1789`,
  **stop** — do not "just rebuild". See failure mode 1 above.

---

## State of this folder at v4.0 — clean

**The raw measurement record has been deleted.** The folder was reset to a clean
benchmarking state when the campaign closed: 707 `results_cinepi_*` text/CSV
files, 161 result bundles in `results/`, 23 `.gpr_scratch_*` directories, 7
`gpr_out_*` directories (992 `.gpr` frames), the pipeline's DNG/GPR validation
clips, and assorted run logs are all gone. 2.9G → 79M.

**This means every number in this document is now a transcription without its
underlying evidence file.** The figures were copied here from the runs while
they existed, and the per-leg detail (pass-by-pass fps, spreads, temperatures,
`timeout_releases`) is preserved above in prose — but the source `.txt`/`.csv`
files behind them no longer exist and cannot be re-examined. Anyone auditing
these numbers must re-measure rather than re-read. The reproduction recipe above
is exact and the binary is unchanged, so re-measuring is possible; note the ~5%
between-boot variance before treating a fresh run as a contradiction.

What is deliberately kept, because the package will not work without it:

    cinepi_qraw_bench/build/vc5_bench          the pinned binary, 957d85dc
    cinepi_qraw_bench/vc5_bench.cpp            canonical source, 62079c42
    cinepi_qraw_bench/input/sample_*.raw16     reference frame, 566f4808
    .static_winner_verified / .build_verified   identity gates
    validated_input/, third_party/, all runners, all docs

Gates re-verified after the cleanup: `tools/check_single_encoder.sh` PASS
(`quant=universal-standard-v3`), all four binary identities unchanged, and the
installed live core fingerprint matches the source tree.

`gpr_out_*/source_uncompressed.raw16` was only ever a `cp` of the canonical
input (`RUN_BENCHMARK.sh:476`, written when `SAVE_QRAW=on`), so no unique image
data was lost; re-run with `SAVE_QRAW=on` to regenerate it.

Note for future runs: the live pipeline bind-mounts this directory as
`/media/RAW` (`run_qraw_pipeline.sh` Step 5), so CinePi clips it records land
back **inside this folder** as `CINEPI_*_DNG` / `CINEPI_*_GPR`. Expect ~2.2G per
12-bit validation run and clear them out afterwards to keep the folder clean.
