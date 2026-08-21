<p align="left">
  <img src="docs/qraw-logo.png" alt="QRAW" width="150">
</p>

## QRAW: Optimised VC-5/GPR For ARM

A RAW video encoder for ARM. It takes Bayer frames from a sensor and writes
`.gpr` files fast enough to keep up with a camera, on a Raspberry Pi 5, using
only the CPU.

QRAW started as the GoPro GPR SDK. GPR is built for stills, where one frame per
button press is plenty of time. Video is a different problem: you get about
20 ms per frame and you never get it back. Most of the work here was rewriting
the transform and entropy paths in NEON, and then rearranging everything around
the fact that a Pi 5 runs out of memory bandwidth long before it runs out of
CPU. The result is roughly 52% faster than a straightforward CPU port, and it
holds 4K in real time.

Output is ordinary GPR. Adobe Camera Raw opens it, `gpr_tools` decodes it, and
nothing downstream needs to know the files came from here. That holds for every
quality lever below, including the ones that change what gets stored.

This repository is the encoder and the benchmark that proves it. The CinePi
camera application that drives it lives in **cinepi-qraw-alpha**.

---

## What you get

**Ten quality modes**, M1 through M10, on one calibrated quantiser ladder. M1 is
the largest and best, M10 the smallest, and the low-pass band is never quantised.
The ladder was tuned from a visual-quality against frame-cost matrix rather than
picked by ratio.

Across the full ladder that works out at roughly **3:1 at M1 to 12:1 at M10**
against uncompressed RAW12. A 3840×2160 RAW12 frame is 12.4 MB before encoding,
so the working middle of the ladder lands around 2 MB a frame:

| Mode | Approximate ratio | Frame at 4K |
|---|---|---|
| M1 | ~3:1 | ~4.1 MB |
| M3 | ~4.5:1 | ~2.8 MB |
| M5 | ~6:1 | ~2.0 MB |
| M7 | ~8:1 | ~1.5 MB |
| M10 | ~12:1 | ~1.0 MB |

Those are one scene at one gain. A mode fixes the quantiser ladder, not the
ratio: a noisy or highly detailed frame compresses less at every mode, and a
smooth one more. Treat them as the shape of the ladder rather than as figures
your footage will hit.

Precision is 10, 11 or 12 bit, GP-Log2 companded, with 12 as the reference.

**Three quality levers** sit on top of the M1–M10 ladder, independent of it and of
each other:

- **CAQ**, Component-Aware Quantisation (off / soft / medium / strong), derives a
  separate quantiser table per component instead of one for all four. It spends
  fewer bits on the colour residual channels and, more gently, on GD, leaving the
  green mean alone.
- **Pixel Clean** widens the dead-zone to 125% of the half-step and zeroes ±1
  colour coefficients at level 1. Because the widened threshold also feeds the
  encoder's provable-zero skips, it makes the file smaller *and* the encode
  cheaper rather than trading one against the other.
- **Noise Clean** (off / soft / medium / strong, plus a strength scalar) does the
  same trick deeper, on selected level-1 bands. It is not a denoise pass and adds
  no second pass, no per-frame statistic and no state. Medium is the default at
  roughly 4% smaller with a small speed gain; strong reaches about 8% but starts
  pruning the green mean, which is where sharpness lives, so it is offered rather
  than defaulted.

Turn any lever off and the output is bit-identical to what it was before that
lever existed, which is what makes them safe to A/B.

**Four recommended presets** bundle a mode with the levers, so you pick a working
point rather than assembling one:

| | Mode | Baseline | Ratio | Good for | Up to |
|---|---|---|---|---|---|
| **HQ+** | M2 | CAQ strong + Pixel Clean | ~3.5:1 | archival | 30 fps |
| **HQ** | M5 | CAQ strong + Pixel Clean | ~6:1 | the working default | 48 fps |
| **ST** | M7 | CAQ strong + Pixel Clean | ~8:1 | coarser, more headroom | 50 fps |
| **LT** | M7 + band pruning | CAQ strong + Pixel Clean | ~9:1 | finest diagonal detail dropped | 50 fps |

CAQ strong and Pixel Clean are not optional. Every preset carries them, and so
does every rung of the rate-holding ladder described below. They are the
quantiser the camera was characterised at rather than something an operator
switches on, and every measured figure quoted for the presets or the ladder was
taken with them in force.

The rates are measured on an IMX585 at 3840×2160, ISO 100. Higher gain means more
noise, more noise means less compressible detail, and the encoder works harder for
a bigger file — so the achievable rate falls as ISO rises. These are what each
preset's measured capacity covered on that rig, not limits in the encoder.

**On 60 fps.** The Raspberry Pi 5 cannot currently sustain it through this
pipeline. The ceiling sits around 55 fps at the heaviest compression, and it is a
memory bandwidth limit rather than an encoder one — the encoder runs out of DRAM
throughput before it runs out of CPU. This is the single clearest argument for a
higher-bandwidth AArch64 part; see the platform notes below.

---

### QRAW STABLE

A fixed grade is a promise about the picture. STABLE is a promise about the take:
rather than holding one grade and dropping frames when the scene gets expensive,
it varies the grade between frames to hold the requested rate. A frame that never
arrived cannot be recovered in post; a frame that arrived slightly coarser can be
lived with. STABLE trades the second for the first.

This is the reason the encoder is a ladder rather than a setting. Moving along it
has to be free, or a controller cannot use it.

It runs its own ladder of ten rungs, each one a mode plus the baseline levers.
Listed finest to coarsest:

| Rung | Grade | Ratio | Notes |
|---|---|---|---|
| 1 | M2 | ~3.5:1 | visually lossless |
| 2 | M3 | ~4.5:1 | close to visually lossless |
| 3 | M4 | ~5:1 | close to visually lossless |
| 4 | M5 | ~6:1 | recommended working ratio |
| 5 | M6 | ~7:1 | recommended working ratio |
| 6 | M7 | ~8:1 | recommended for long takes |
| 7 | M7 + band pruning | ~9:1 | Similar to M7 with slightly reduced detail|
| 8 | M8 + band pruning | ~10:1 | **the opening rung** — measured to hold every rate the interface offers |
| 9 | M9 + band pruning | ~11:1 | reserve, entered only under measured pressure |
| 10 | M10 + band pruning | ~12:1 | reserve, coarsest |

Every rung carries CAQ strong and Pixel Clean, exactly as the presets do. This is
not a different quantiser from the one everything else was measured against — it
is the same M1–M10 ladder with the same baseline, walked automatically.

**On band pruning.** Dropping the finest diagonal detail band is most useful at
the coarse end, because those modes are already quantising that band heavily —
there is less left to lose. It costs little visually at M7 and below, and some
viewers prefer the result: removing that band suppresses fine diagonal
quantisation artefacts, which reads as a mild noise reduction. The frame-rate gain
is real either way, which is why the bottom four rungs all carry it.

**Where a take opens, and why it is not the top or the bottom.**

Rung 8 is the starting point. That looks arbitrary until you separate two things
the ladder has to do at once. The opening rung has to be one that is certain to
hold at every frame rate the interface offers, because a take that opens too fine
and immediately falls behind has already lost frames before the controller has
any evidence to act on. M8 with band pruning is that rung, and it was measured
rather than chosen.

The floor is a different question. It has to be as deep as the harshest scene
needs, which is why rungs 9 and 10 sit below the opening rung. While M8 was the
bottom of the ladder those two roles were the same index and the distinction did
not exist. The moment a reserve was added beneath it, they stopped being the
same: opening at the floor would have started every ordinary take two rungs
coarser than before and made it climb back, which is paying for the reserve out
of the takes that never need it.

So rungs 9 and 10 are genuinely a reserve. Nothing descends into them by choice.
The only way in is measured pressure or a lost frame.

**How it moves.**

From the opening rung the controller climbs one rung at a time while the evidence
allows, and coarsens when pressure says to. The climb is deliberate rather than
opportunistic: it is predictive, refusing to move to a rung whose cost it
estimates the rate cannot carry, rather than climbing until something breaks and
backing off afterwards.

That distinction came out of an earlier controller which steered on pipeline
occupancy and climbed until the encode pool was fully subscribed. By Little's law
that ladder was *defined* to climb until the queue was about to back up, which is
a different goal from holding the rate with no dropped frames, and it behaved
accordingly.

**What the encoder contributes.**

The controller itself lives in **cinepi-qraw**. What is here is what makes it
possible: all ten rungs are constructed while the camera is idle, so a grade
change mid-take costs nothing at encode time. An encoder instance takes around
11 ms to create, which is not a cost anyone can pay between frames — so none of
them are created between frames.

Each frame is also tagged with the rung that produced it, which in a take whose
grade varies is the only per-frame record of what each frame actually is.

---

There is one encoder configuration, not a menu of them. It was chosen by
measuring the alternatives and throwing them away, and the reasoning is in
`docs/`. Hand it geometry it cannot encode properly and it stops rather than
quietly falling back to something slower.

`docs/ENCODER_REFERENCE.md` has the profile tables, the measured numbers, every
optimisation with the switch that toggles it, and the full option list.

---

## Using it

The interface is C. One encoder object is single threaded and holds about 20 MB
of scratch, so run one per worker. On a Pi 5 in RAW12 the measured default is
four workers, one per core — worth about 14% over three, since the camera thread
costs under 7% of a core when there is no byte swap to do. In RAW16 it caps at
three: libcamera byte-swaps 16.8 MB a frame there, and a fourth encode worker on
Core 0 starves capture.

```c
CinepiQrawConfig cfg;
cinepi_qraw_config_defaults(&cfg);
cfg.width  = 3840;
cfg.height = 2160;
cfg.bayer  = CINEPI_BAYER_GBRG;
cfg.mode   = "m5";
cfg.caq              = 3;   /* strong */
cfg.pixel_clean      = 1;
cfg.noise_clean_mode = 2;   /* medium */

CinepiQrawEncoder *enc = cinepi_qraw_create(&cfg, NULL);

void *gpr = NULL; size_t size = 0;
if (cinepi_qraw_encode(enc, raw16, &gpr, &size) == 0) {
    write(fd, gpr, size);
    cinepi_qraw_release(enc, gpr);   /* or leak several MB per frame */
}

cinepi_qraw_destroy(enc);
```

Feed it tightly packed 16-bit Bayer, one sample per `uint16_t`, MSB justified.
That is what a Pi 5 hands you when you ask for an unpacked format. Do not pass
`PISP_COMP1` buffers: they are block-delta coded and are not sensor values.
Both dimensions must be even.

Everything else, including every lever and what it does, is documented in
`encoder_library/cinepi_qraw_encoder.h`.

---

## Platform

Written for AArch64 with NEON. Developed and validated on a Raspberry Pi 5 and
CM5 (BCM2712, four Cortex-A76 at 2.4 GHz) running 64-bit Raspberry Pi OS, built
with `-mcpu=cortex-a76 -flto=auto`. The sensor here is an IMX585, but any
16-bit Bayer source works.

The Pi is the test rig, not a requirement. What the encoder actually needs is
NEON, four cores and memory bandwidth. Other AArch64 parts should work, and the
RK3588 is the interesting one, since it has roughly twice the bandwidth and
bandwidth is the ceiling.

AArch64 is currently a hard requirement, not a preference: parts of the
quantiser live inside a NEON guard and the encoder does not compile without it.
There is no x86 build today.

---

## Building

```bash
git clone --recurse-submodules https://github.com/Kizza-O/QRAW.git
cd QRAW
./benchmark/install_pi_dependencies.sh
./encoder_library/build_qraw_core.sh
```

If you already cloned it without the submodule:

```bash
git submodule update --init --recursive
```

`docs/BUILD_PI5.md` covers the Pi 5 specifics, including the 16 KB page size.

---

## Benchmarking

```bash
cd benchmark
./RUN_BENCHMARK.sh
```

The reference frame is attached to the release rather than committed; see
`benchmark/cinepi_qraw_bench/input/README.md`. `tools/make_gpr_test_set.sh`
generates the full cross product of quality settings from it, 640 files, named
so the settings read off the filename.

Some warning about the numbers. Throughput and file size depend on the scene as
much as the encoder. A quality mode fixes the quantiser ladder, not the
compression ratio, so a noisy frame gives you bigger files and fewer fps at
every mode. The figures in `docs/` are one rig, one scene, one date, recorded so
that a change can be compared against the same conditions. Your own run is the
only number that describes your setup.

The ordering does hold: m1 is always larger and better than m10, and 12 bit is
always finer than 10.

One caveat worth knowing before you read older figures: until 2026-08-21 the
standalone benchmark had no `--caq` flag and never read the environment
variable, so anything in this package labelled "CAQ strong" before that date was
in fact measured with CAQ off. The live camera was never affected, because it
drives the library, which did read it.

And before benchmarking anything: an attached display costs about 7% of encode
throughput, because a 3440x1440 desktop at 60 Hz pulls a bit over a gigabyte a
second out of the same DRAM the encoder is competing for. Record with the screen
asleep. `docs/FINDING_PLATFORM_TUNING.md` has the rest.

---

## Correctness

The encoder source is pinned to a hash. `tools/check_single_encoder.sh` fails if
it changes without the pin being moved deliberately, and `tools/verify.sh
--library` round-trips every encode path against a stored CRC.
`tools/pixel_clean_equivalence.cpp` checks Pixel Clean exhaustively against its
specification across the encoder's proven coefficient range. All of it should
pass before anyone believes a performance number, including you.

`CONTRIBUTING.md` explains what a performance change needs to show.

---

## Licence

Apache-2.0, see `LICENSE`.

QRAW derives from the [GoPro GPR SDK](https://github.com/gopro/gpr), which is
Apache-2.0 or MIT at your option, copyright 2018 GoPro, Inc. GPR in turn vendors
Adobe's DNG SDK and XMP Core, both under 3-clause BSD. The fork holding QRAW's
changes to the SDK is [gpr-qraw](https://github.com/Kizza-O/gpr-qraw) on the
`qraw` branch, and `CHANGES-QRAW.md` there lists every file that differs from
upstream.

`THIRD_PARTY_NOTICES.md` has the full inventory.

GoPro, GPR, Protune and GP-Log are trademarks of GoPro, Inc. Adobe and DNG are
trademarks of Adobe Inc. Raspberry Pi is a trademark of Raspberry Pi Ltd. This
project is not affiliated with or endorsed by any of them.

---

## Credits

The VC-5 encoder and the GPR container are GoPro's. The DNG writing is Adobe's.
CinePi is Csaba Nagy's. What is mine is the ARM optimisation work, the quantiser
ladder and the quality levers on top of it, and the interface that lets a camera
call it.
