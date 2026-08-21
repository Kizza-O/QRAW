# QRAW Encoder Reference

Platform, the quality levers, the mode ladder, and the option surface. For the
shape of the pipeline itself, read `ARCHITECTURE.md` first.
Everything here was read out of the shipped source rather than from earlier
notes.

---

## 1. Platform

QRAW is written for AArch64 (ARMv8-A) with NEON. It is not portable C that
happens to build on ARM: the transform and entropy paths are NEON, and the
scheduling is arranged around the memory system of a specific class of part.

| | |
|---|---|
| Architecture | AArch64 / ARMv8-A with NEON (ASIMD) |
| Reference system | Raspberry Pi 5 and CM5, BCM2712, four Cortex-A76 at 2.4 GHz |
| Build flags | `-mcpu=cortex-a76 -flto=auto` |
| Reference OS | 64-bit Raspberry Pi OS, `kernel_2712.img`, 16 KB pages |
| Reference sensor | Sony IMX585, RAW12 or ClearHDR; any 16-bit Bayer works |

The Pi is the test rig, not a requirement. What the encoder needs is NEON, four
cores and memory bandwidth. The encoder is DRAM-bound rather than compute-bound
at this operating point, which is why most of the optimisation work was about
moving fewer bytes and why the architecture matters at all. The RK3588 is the
interesting next target for exactly that reason.

x86-64 builds and runs, which is useful for checking correctness. No figure in
`docs/` was measured there.

---

## 2. The quality levers

Four independent controls sit on top of the mode ladder. All of them produce
ordinary GPR: no component, dimension, wavelet-topology or entropy-syntax
change, and every band still advertises the divisor it was quantised by, so a
stock VC-5 decoder needs no special knowledge of any of this.

### Mode — `m1` … `m10`

The calibrated quantiser ladder. Per-level and per-band tables mirroring the
GoPro Filmscan-to-Medium ladder, scaled from the SDK's 14-bit domain into the
companded domain. m1 is archival and largest, m10 the smallest. The LL band is
never quantised.

### CAQ — Component-Aware Quantisation

`--caq off|soft|medium|strong`, or `CINEPI_CAQ`. Default in the launcher:
**strong**.

A relative modifier on whichever table `mode` resolves to. It spends fewer bits
on the colour-residual components RG and BG, and more gently on GD, leaving GS
untouched. Scale factors, as coarse/middle/fine per level:

| Profile | RG, BG | GD |
|---|---|---|
| soft | 1.00 / 1.10 / 1.25 | 1.00 / 1.00 / 1.08 |
| medium | 1.00 / 1.25 / 1.55 | 1.00 / 1.08 / 1.18 |
| strong | 1.10 / 1.50 / 2.00 | 1.00 / 1.15 / 1.35 |

`off` is bit-exact against pre-CAQ behaviour by construction: the derivation
returns four copies of the base table and the per-channel path is not taken.

> **Read this before quoting any older benchmark number.** Until 2026-08-21 the
> standalone bench had no `--caq` flag and never read `CINEPI_CAQ`, so every
> figure in this package that describes itself as "CAQ strong" was in fact
> measured with CAQ **off**. The live camera was never affected, because it
> drives the library, which did read the variable.

### Pixel Clean

`--pixel-clean 0|1`, or `CINEPI_PIXEL_CLEAN`. Default in the launcher: **on**.

Two rules that collapse into one integer comparison:

- a dead-zone at 125% of the normal half-step, and
- a post-quant rule forcing quantised RG/BG level-1 coefficients of ±1 to zero.

Because the exact quantiser is monotone in `|coeff|`, both reduce to a single
exact threshold per band, resolved once at encoder setup: `|coeff| <= T`. The
widened threshold also feeds the existing provable-zero compute skips, so this
lever makes the encoder cheaper as well as the file smaller rather than trading
one for the other.

Orthogonal to CAQ and to band pruning; combinable with both. `0` is bit-exact
against the encoder without it.

`tools/pixel_clean_equivalence.cpp` checks the fused implementation
exhaustively against the specification's own pseudocode, across the encoder's
proven coefficient range.

### Noise Clean

`--noise-clean off|soft|medium|strong` and `--noise-clean-strength 0.50..1.50`,
or `CINEPI_NOISE_CLEAN` and `CINEPI_NOISE_CLEAN_STRENGTH`. Default:
**medium**.

**Not a denoise pass.** It widens the encoder's own "this coefficient becomes
zero" threshold on selected level-1 bands, in units of quantised levels, and
changes nothing else: no second pass, no per-frame statistic, no state. The
zero happens inside the quantiser, so every non-zero mask and sidecar is built
from the widened result by construction. As with Pixel Clean, the wider
threshold feeds the provable-zero skips, which is why it is both smaller and
slightly faster.

Depth per component at wavelet level 1, in quantised levels:

| Profile | GS HH1 | GD HH1 | RG/BG HH1 | RG/BG LH1 | RG/BG HL1 |
|---|---|---|---|---|---|
| soft | 0 | 1 | 1 | 1 | 1 |
| medium | 0 | 2 | 2 | 1 | 1 |
| strong | 1 | 2 | 2 | 1 | 1 |

The ±1 level on RG/BG is deliberately *not* what separates the profiles, since
Pixel Clean already zeroes `|q| <= 1` on all three level-1 bands of RG and BG.
Depth on colour is what separates them. An earlier ladder measured soft and
medium byte-identical at every grade for exactly this reason.

**Strong also prunes GS** — the green mean, where the picture's sharpness
lives — so it is offered rather than defaulted. `off` returns the level-1
threshold to Pixel Clean's and is bit-exact against the pre-Noise-Clean
encoder.

Measured on the static reference frame at m5, on the shipping stack (CAQ strong
plus Pixel Clean): soft −3.4% size at no fps change; strong −7.9% size and
+4.6% fps. Medium is the shipped default at −3.5 to −4.4% container size with a
small fps gain, and 50.9 dB against the `off` decode.

Full-frame numbers at 3840×2160:

| | off | medium | strong |
|---|---|---|---|
| m3 | 2,784,854 B @ 39.0 fps | 2,667,618 (−4.2%) @ 40.2 | 2,606,078 (−6.4%) @ 41.4 |
| m5 | 2,050,170 B @ 45.0 fps | 1,975,982 (−3.6%) @ 45.0 | 1,894,454 (−7.6%) @ 46.8 |
| m7 | 1,510,414 B @ 48.0 fps | 1,481,890 (−1.9%) @ 49.2 | 1,420,406 (−6.0%) @ 48.6 |

### Noise Clean, ISO-calibrated (legacy)

Superseded by the profile-based lever above, and reachable only when a run sets
`CINEPI_NOISE_CLEAN_ISO=1`, so the two can never threshold the same frame.

An ISO-calibrated dead-zone in pre-quant wavelet coefficient units: a
coefficient is zeroed when its magnitude is statistically inside the noise the
sensor produces at the active gain. `iso` is required and must be one the
profile was actually measured at — 100, 200, 400, 800, 1600 or 3200 for the
IMX585 RAW12 stacks. There is no interpolation on purpose, and an unknown ISO
makes the lever a no-op rather than an approximation.

`noise_clean_k` scales the threshold against measured sigma, default 0.5. Worth
understanding before using it: the log curve amplifies shadow *signal* by the
same 34–54× it amplifies shadow noise, so in the shadows the two are comparable
and this removes real content along with noise. The result has to be looked at.

Kept because the per-gain measurement behind it is real work, and a calibrated
floor beats an estimated one where one exists.
`tools/qraw_noise_profile.py` produces those profiles.

---

## 3. Precision and colour

| Option | Values | Default | Meaning |
|---|---|---|---|
| `--compand-bits` | 10, 11, 12 | 12 | Stored precision, same curve |
| `--compand-inframe-bits` | 10, 11, 12 | 12 | Effective precision within a frame |
| `--effective-bits` | 12–16 | — | Source precision (16 for a Pi 5 unpacked stream) |
| `--true-12bit` | on/off | off | Stores RG/BG/GD unhalved at 13-bit, losslessly |
| `--working-max` | ≤4095 | 4094 | int16-safe: peak coefficient is 4× this |
| `--log-strength` | — | 599 | Companding strength; 599 is GP-Log2 |
| `--black` / `--white` | — | — | Source range for the log LUT |
| `--bayer` | rggb, gbrg | gbrg | Supported layouts |

GP-Log2 is the Protune-family curve `y = log(1 + S·x) / log(1 + S)` with
S = 599. It does the perceptual bit allocation, which is what lets a 12-bit
container hold a 16-bit sensor's usable range. GPR containers are GP-Log2 by
convention and carry no curve declaration, so a sensor applying its own
gradation curve on chip must be decoded to scene-linear and re-encoded as
GP-Log2: one table, one companding.

---

## 4. The production encoder stack

One pinned implementation, reused by the live library. Retained settings:

- fused NEON wavelet cascade and fused GP-Log2 split
- non-temporal (`STNP`) coefficient band writes
- v2 sidecar with zero-skip; stride split; handoff pool
- entropy sign LUT, 8-wide scan, 64-bit accumulator
- input prefetch; entropy prefetch distance 128 at locality 0 (streaming)
- shared verified in-place output
- 20 ms cyclic wavelet rendezvous, 40 µs release lead for two or more owners
- movable IRQ affinities relocated to Core 0 during encode, restored on exit

### Worker selectors

The selector is not a thread count. `cinepi_winning_stack_env N`:

| N | Configuration |
|---|---|
| 1–3 | that many full-frame owners |
| 4 | three owners plus a Core 0 SB8×4 entropy assistant |
| **5** | **four full-frame owners, one per core — the measured default at RAW12** |

**Four owners is the default for RAW12**, paired with a zero-copy inflight limit
of 8. Measured at 3840×2160, 60 fps requested, m5, three legs each with a fresh
process per leg, where the encoder is saturated so delivered fps is a direct
throughput instrument:

| Configuration | Delivered fps | Drops per 10 s |
|---|---|---|
| selector 3, inflight 6 (the old defaults) | 40.53 | 191–193 |
| selector 5, four owners | 45.47 | 139–145 |
| selector 5, inflight 8 | 46.42 | 126 |

That is +14.5% for two environment variables. The inflight limit goes with the
worker count rather than being independent: with four owners each holding a
camera buffer for a whole encode, the default limit of 6 leaves only two frames
able to queue, so selector 5 at inflight 6 measures the gate as much as the
worker count.

The fourth owner was measured as worth nothing in an earlier campaign, but that
was at m7 with a different entropy path and at 48 fps requested, where the
encoder is not saturated and a 1 fps effect sits inside the noise.

**ClearHDR caps at three owners regardless of the stored preference.** In RAW16
libcamera byte-swaps 16.8 MB per frame on its camera thread, around 10 ms of a
33 ms budget, and a fourth encode worker on Core 0 starves capture. At RAW12
there is no byte swap — the camera thread costs 6.8% of a core against 44.8% —
so Core 0 has real headroom.

Resolution order for the worker count, most explicit first: a caller-stated
`QRAW_THREADS` or `CINEPI_QRAW_THREADS` wins outright, so documented measurement
recipes behave exactly as written; then the stored operator preference from the
camera UI, in full-frame owners, where 4 owners maps to selector 5 and 3 to
selector 3; then the measured default.

There are no A/B encoder variants, no historical shader encoders, no
CPU-wavelet labs and no fallback implementations in this package. Unsupported
fused geometry fails closed rather than silently selecting another path.

### Labelled optimisations

Each carries a switch so it can be measured against the shipped default.

| | Switch | What it does |
|---|---|---|
| C1 | `--cpu-gpr-malloc-tuned` | Recycles the large output block via `mallopt` |
| C2 | `--cpu-gpr-affinity` | Pins each worker to its own core |
| C3 | `--cpu-gpr-hugepages` | THP on per-worker frame buffers |
| C4 | `--cpu-gpr-prefetch` | Prefetches RAW rows in the fused cascade |
| C5 | `--cpu-gpr-dng-splice` | Splices VC-5 into a retained DNG container |
| E1 | `--cpu-gpr-hybrid-entropy` | int8 tile packing plus the runspan scanner |
| E2 | `--vle-prequant-skip` | Eight-wide zero skip in the entropy scan |
| S1 | `--cpu-split-neon` | NEON split arithmetic; LUT gathers stay scalar |
| S2 | `--cpu-gpr-splice-shared` | One splice template across all workers |
| H1 | `--cpu-gpr-helper` | Retires finished buffers on the spare core |
| H2 | `--cpu-gpr-splice-reuse` | Reuses one container buffer per worker |
| Y1 | `--cpu-gpr-rt` | `SCHED_FIFO` workers, with fallback notice |

Others without letter labels: `--cpu-wavelet-kernel` (v53 row-major vertical,
default; v52 column-block retained for comparison), `--cpu-wavelet-vec`,
`--cpu-wavelet-fused`, `--cpu-split-fused`, `--cpu-nontemporal`,
`--cpu-hot-lut`, `--cpu-direct-hybrid`, `--cpu-gpr-local-inplace`,
`--cpu-gpr-shared-inplace`, `--cpu-wavelet-reuse-context`,
`--cpu-vle-prefetch-distance`, `--cpu-vle-prefetch-locality`,
`--cpu-gpr-stagger-us`.

---

## 5. Execution models

`--execution MODE`:

| Mode | Purpose |
|---|---|
| `cpu-gpr` | The production path. All-CPU. `--cpu-gpr-threads` defaults to 3 in the standalone bench; the live default at RAW12 is four full-frame owners. |
| `pipeline` | GPU-assisted transform feeding SDK entropy. Not production. |
| `strict-workers` | Exact end-to-end frame model. Diagnostic, and macOS. |
| `capture` | Recorder simulation: synthetic sensor at `--target-fps`, ring buffer, overflow modelling |

The all-CPU encoder replaced the GPU path and delivered roughly +52%
throughput. The V3D 7.1 QPU occupancy ceiling that motivated the switch is
documented in `OPTIMISATIONS.md`.

---

## 6. Tools

| Tool | What it does |
|---|---|
| `tools/verify.sh --library` | CRC round-trip across every encode path |
| `tools/check_single_encoder.sh` | Encoder source pinned to a SHA-256; exactly one canonical source; the quant ladder rows present; no alternate encoder declaration |
| `tools/pixel_clean_equivalence.cpp` | Exhaustive check of Pixel Clean against the specification pseudocode over the proven coefficient range |
| `tools/qraw_noise_profile.py` | Measures per-sensor, per-gain noise and emits thresholds in pre-quant wavelet coefficient units, so a profile survives a change to the quant ladder |
| `tools/qraw_dark_calibrate.py` | Reports per-CFA black levels, defect pixels, row/column fixed-pattern noise and G1/G2 residual. Deliberately reports rather than writing a correction profile. |
| `tools/make_gpr_test_set.sh` | Generates the full cross product of quality axes from the one reference frame, 640 files, settings readable off the filename |
| `tools/qraw_stats_cli` | Reads the live telemetry block |
| `--self-test` | In-binary consistency check |

On `qraw_dark_calibrate.py`, the rule it exists to serve is worth repeating:
never promote a sensor correction merely because the file gets smaller. It has
to be shown to be a real sensor artefact or statistically justified noise
first, and the tool produces evidence for that decision rather than making it.

---

## 7. Option surface

`vc5_bench.cpp` parses **140 documented options** plus the four quality levers
above. The complete list is in the binary's own help:

```bash
./benchmark/cinepi_qraw_bench/build/vc5_bench --help
```

Most of them exist for measurement, not for use. The production configuration
is a fixed set applied by `winning_stack.sh`, and the switches are how each
element of it earned its place. If you are encoding rather than benchmarking,
use the library interface in `encoder_library/` and set `mode`, `caq`,
`pixel_clean` and `noise_clean_mode`. That is the whole surface you need.

> **Known gap:** `--caq`, `--pixel-clean`, `--noise-clean` and
> `--noise-clean-strength` are parsed but do not appear in the `--help` usage
> block. They are documented here and in `encoder_library/cinepi_qraw_encoder.h`
> until that is fixed.

---

## 8. What the numbers mean

Throughput, file size and compression ratio are properties of a scene and a
machine, not of the encoder. A quality mode fixes the quantiser ladder, not the
ratio: a noisy frame gives bigger files and fewer fps at every mode.

Figures in `docs/` are one rig, one scene, one date, recorded so a change can be
compared against the same conditions. Your own run is the only number that
describes your setup.

What is fixed:

- m1 is larger and better than m10, always
- 12-bit stored precision is finer than 10-bit, always
- LL is never quantised
- CAQ off, Pixel Clean off and Noise Clean off are each bit-exact against the
  encoder before that lever existed
