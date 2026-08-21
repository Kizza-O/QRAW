> **r21 production rule:** the QRAW encoder is the SHA-pinned v1.16.6 winning stack.
> Optimisation, affinity and rendezvous A/B overrides described in older notes are not
> accepted by the normal live/static benchmark path. M1-M10, 10/11/12-bit precision,
> sensor mode and worker selector remain operating parameters, not encoder versions.

# Benchmark variables in the live pipeline

Every tunable the standalone benchmark exposes on its command line is
available to the live encoder as an environment variable, with the same
semantics and the same defaults. An unconfigured live run therefore
reproduces the benchmark's promoted operating point exactly -- the measured
stack (v53 wavelet kernel, x8 vectors, fused cascade, fused split, S1 NEON
split, non-temporal band stores on, spare-core helper off).

`run_live.sh` wraps these in shorter names; the encoder reads the
`CINEPI_QRAW_*` forms directly, so both work.

| benchmark flag | env var | run_live.sh | default |
|---|---|---|---|
| `--mode m1..m10` | `CINEPI_QRAW_MODE` | `QRAW_MODE` | `m5` |
| `--quant N` | `CINEPI_QRAW_QUANT` | `GPR_QUANT` | unset (ladder active) |
| `--threads N` | `CINEPI_QRAW_THREADS` | `QRAW_THREADS` | 3 (selector, max 5) |
| `--compand-bits` | `CINEPI_QRAW_COMPAND_BITS` | `GPR_COMPAND_BITS` | 12 |
| `--compand-inframe-bits` | `CINEPI_COMPAND_BITS` | (not wrapped) | 12 |
| `--compand-quant-scale` | `CINEPI_COMPAND_SCALE` (`0` = off) | (not wrapped) | on |
| saved-frame settings ID suffix | `CINEPI_QRAW_TAG` | (set by the runner) | derived |
| `--true-12bit` | `CINEPI_QRAW_TRUE_12BIT` | `GPR_TRUE_12BIT` | 0 |
| `--log-strength` | `CINEPI_QRAW_LOG_STRENGTH` | `GPR_LOG_STRENGTH` | core default |
| white level | `CINEPI_QRAW_WHITE` | `GPR_WHITE` | 0 = (1<<bits)-1 |
| black level | `CINEPI_QRAW_BLACK` | `GPR_BLACK` | sensor metadata |
| effective bits | `CINEPI_QRAW_BITS` | (via MODE) | 16 |
| `--gpr-params` | `CINEPI_QRAW_PARAMS` | auto | /opt/cinepi-qraw/share/cinepi-qraw/gpr_params.json |
| fused cascade | `CINEPI_QRAW_FUSED_WAVELET` (retired / production-locked) | `GPR_FUSED_WAVELET` | on |
| fused split | `CINEPI_QRAW_FUSED_SPLIT` (retired / production-locked) | `GPR_FUSED_SPLIT` | on |
| NEON split | `CINEPI_QRAW_NEON_SPLIT` (retired / production-locked) | `GPR_NEON_SPLIT` | on |
| non-temporal stores | `CINEPI_QRAW_NONTEMPORAL` (retired / production-locked) | `GPR_NONTEMPORAL` | on |
| DNG splice | `CINEPI_QRAW_DNG_SPLICE` (retired / production-locked) | `GPR_DNG_SPLICE` | on |
| shared splice template | `CINEPI_QRAW_SPLICE_SHARED` (retired / production-locked) | `GPR_SPLICE_SHARED` | on |
| (live only) active-area crop | `CINEPI_QRAW_CROP` | `GPR_CROP` | auto 3840x2160+8+10 |

The resolved values are logged at encoder setup ("config:" and "stack:"
lines) and published in the shared-memory stats block, so the GUI panel
shows what is actually running rather than what was intended.

## Reduced precision: two variants, and which one you get

Both keep the GP-Log2 curve (`y = log1p(k*x)/log1p(k)`, k = 599); they differ
in where the lost precision comes from.

- **Reduced code range** -- `GPR_COMPAND_BITS` / `CINEPI_QRAW_COMPAND_BITS` /
  `--compand-bits`. The curve is scaled to `working_max` 1023 or 2047 instead
  of 4094. The m1..m10 divisors are sized for 12-bit amplitudes, so the ladder
  is scaled by `2^(12-bits)` to keep quantisation constant *relative to
  signal*; `CINEPI_COMPAND_SCALE=0` reproduces the unscaled v1.7.60
  behaviour. **The container still declares saturation 4095**, so these frames
  decode 1 stop (11-bit) or 2 stops (10-bit) dark unless the reader rescales.
- **Reduced in-frame precision** -- `CINEPI_COMPAND_BITS` /
  `--compand-inframe-bits`. The container stays a full 12-bit GPR with the
  white point where it belongs; only the *step* between code values grows
  (multiples of 2 or 4). This is the variant the m5 campaign measured -- it
  gives up a substantial fraction of the bytes for a coarser step, more of it
  at 10 bits than at 11 -- and it is the one to use for footage. How much
  smaller a real frame gets depends on the scene.

`run_live.sh` only wraps the first. The table below is a DETERMINISTIC
reference, not a scene measurement: one fixed bundled sample at m5 (3840x2160,
16-bit linear source), where library and benchmark agree byte for byte. The
crc32 column is a bit-exactness anchor -- these values must not move for this
input, and the build self-test checks them.

| bits | quant ladder | gpr_bytes | crc32 |
|---|---|---|---|
| 12 | 1/8/8/4/16/16/11/73/73/111 | 2,988,814 | 72f44899 |
| 11 | 1/4/4/2/8/8/6/37/37/56 | 2,919,446 | 95e2d7ee |
| 10 | 1/2/2/1/4/4/3/18/18/28 | 2,856,042 | d9b622ff |

With `CINEPI_COMPAND_SCALE=0` the same 10-bit run emits 1,030,286 bytes --
that difference is detail thrown away by over-quantisation, not efficiency.

## White balance

| variable | effect |
|---|---|
| `CINEPI_CG_RB` | `r,b` preview colour gains for this run, overriding the measured base |
| `CINEPI_WB_BASE=off` | ignore `cinepi_wb_base.json` entirely: stock container parameters, no preview correction |

`benchmark/calibrate_wb.sh` measures neutral white balance from a live RAW12/60
frame and stores it in `cinepi_wb_base.json`. Camera runs use that one base
twice: as the preview colour gains for the modes that cannot run auto AWB, and
as the white balance stamped into the GPR and its DNG.

Both matter. The manual-3A modes previously fell back to gains of `1.0,1.0` --
no correction -- which is why every mode except RAW12/60 looked off. And the
white balance in the frames came from the stock `gpr_params.json`, which is a
GoPro template: a different camera under a different light, in every frame
regardless of mode.

RAW12/60 measures it for all modes because neutral white balance is a property
of the sensor and the light, not of the readout mode, and RAW12/60 is the
sensor's native linear readout and the only camera path whose PiSP statistics
are valid. The measurement inverts the GP-Log2 curve before averaging channels;
averaging stored codes instead understates the blue gain badly, because the log
curve compresses a channel that is several times down in light into a code that
is only about twice down.

Colour gains are metadata. They change what a reader renders and never touch
the Bayer samples, so they cannot move a compression or throughput figure.

## Recommended sensor mode

**ClearHDR RAW16 delivered as 12-bit companded** -- `INPUT_BITS=16` with
`CINEPI_CLEARHDR_BITS=12`, which is the default ClearHDR delivery. It is the
only camera path that has the sensor's full ClearHDR highlight range, valid
PiSP statistics so auto 3A works, 12-bit samples rather than 16-bit-wide ones,
and a `.gpr` that renders natively and matches its own DNG (the on-chip
gradation curve is decoded and re-encoded as GP-Log2 rather than stored raw --
`CINEPI_QRAW_NATIVE_GPR=1`, the default).

It reads out at 30 fps. Plain RAW12 reads out at up to 60 fps and is the choice
when frame rate matters more than highlight range; the linear 16-bit ClearHDR
path stays available for reference captures and A/B work, and needs manual
exposure because its statistics are invalid.

## Saved-frame settings ID

Every saved `.gpr` carries the operating point in its filename, after the
frame index, so a frame stays self-describing once it is copied out of its
directory or exported to DNG (the exporter reuses the stem):

```
frame_000000000_m5-c10-b12-w3-p1-fps60.gpr
                m5  QRAW mode (or quantN when GPR_QUANT overrides the ladder)
                c10 compand code range (GPR_COMPAND_BITS)
                b12 source/effective bits (CINEPI_QRAW_BITS)
                w3  encoder workers
```

Also stamped when set: `i10` (in-frame precision), `qsoff` (quant ladder not
scaled), `k<n>` (non-default log strength), `grad` (ClearHDR gradation input),
`t12` (unhalved chroma), then `CINEPI_QRAW_TAG` -- which is where a runner or
the UI adds its own label. `RUN_CINEPI_RAW_BENCHMARK.sh` puts the pass number
and target fps there. The static benchmark's `--save-gpr` stamps the same
fields. The ID is constant within a run and follows the index, so `*.gpr`
globs and name sorts are unaffected.

## Differences from the standalone benchmark, and why

- **Frames come from the sensor, not a file.** There is no `--frames`,
  `--input` or Frame Target simulation: the camera sets the rate, and the
  panel's dropped-frame counter is the measurement that replaces them.
- **CPU path only.** The Vulkan/V3D execution mode is not wired into the
  live encoder; the live stack is the all-CPU pipeline.
- **No `--capture-queue`/`--capture-fps`.** The recorder owns buffering;
  the encoder drops rather than stalls the camera thread, and drops are
  counted in the panel.

## A/B recipes

Quality ladder sweep (same scene, same exposure):
```bash
for m in m1 m3 m5; do QRAW_MODE=$m ./run_live.sh; done   # record ~10 s in each
```

Does the promoted stack still pay off on this silicon?
```bash
./run_live.sh                              # baseline
GPR_FUSED_WAVELET=0 ./run_live.sh          # cascade off
GPR_NONTEMPORAL=0   ./run_live.sh          # STNP band writes off
GPR_DNG_SPLICE=0    ./run_live.sh          # container template off
```

Thread scaling. **The value is a selector, not a worker count:**

| selector | what it starts |
|---|---|
| 1 - 3 | that many full-frame owners, pinned to the highest cores |
| 4 | three owners **plus** the Core0 SB8x4 entropy assistant |
| 5 | **four** full-frame owners, one per core (Core 0 shared with capture) |

```bash
for t in 1 2 3 4 5; do QRAW_THREADS=$t ./run_live.sh; done
```

Selector 4 measured net negative on live input (41 runs), and selector 5 measured
neutral on RAW12 at 48-60 fps -- a third more compute, each frame 17% more
expensive, the two cancelling. Both exist to be re-measured on other hardware, not
as recommendations. See `FINDING_RAW12_60FPS.md` §2-3 for the numbers and for why
the wall is DRAM bandwidth rather than cores.

Compare per-worker wavelet/entropy times, `copy_out_ms`, fps and dropped
frames in the panel between runs. For meaningful comparisons fix exposure
and gain manually -- at 16-bit the PiSP reports that ISP statistics are not
valid, so AGC/AWB will otherwise drift between runs.

## The benchmark tests, in live form

The full-screen UI offers **Encode Benchmark** and **All QRAW Camera Output**;
`RUN.sh --test record` still runs the frame-target model from the command line.
The first two are described below as they map onto the live pipeline, with the
camera replacing the file reader as the frame source.

### 1. Encode Benchmark -- "how fast"

Measures per-frame encode cost and achievable rate. Live equivalent: run
and record; the panel reports per-worker wavelet / entropy / dng_wrap /
copy_out times, encoder fps against the camera's rate for that mode, output
MiB/s and compression ratio. Those are readings, not constants: they move with
the scene and the machine.

Variables: `QRAW_MODE` (or `GPR_QUANT`), `QRAW_THREADS`, the six stack
toggles, `GPR_COMPAND_BITS`, `GPR_TRUE_12BIT`, `GPR_LOG_STRENGTH`.

Benchmark settings with no live equivalent: `passes` (the camera runs
continuously), `save frames` (recording always writes), `seconds` (you
control clip length with the record trigger).

### 2. All QRAW Camera Output -- "what can it write"

Not a measurement at all: coverage. Walks every setting that changes the
CONTENTS of a `.gpr` -- capture path, effective precision, band pruning,
quality mode -- and keeps one decode-verified frame of each, in its own folder,
with a `MANIFEST.tsv` describing every variant. The only input is where the
tree goes.

Each variant is a separate camera session, because `GprEncoder` reads
`CINEPI_QRAW_MODE`, `CINEPI_QRAW_COMPAND_BITS` and `CINEPI_BAND_Q` once when it
is constructed and they cannot be changed inside a running recorder.

    ./RUN.sh --test allgpr --gpr-path /media/RAW
    ./RUN_ALL_QRAW_OUTPUT.sh --dry-run       # the variant list, no camera

### 3. Frame Target / Use Frame Buffers -- "how long"

Measures how long a target rate can be sustained before the RAM buffer
ahead of the disk writer is exhausted. Live equivalent: set the buffer
pool and record until the panel's `buffer_full` flag trips and recording
stops -- the elapsed time is the answer.

Variables: `GPR_OVERFLOW` (MiB; `-1` = auto, 2/3 of MemAvailable, matching
the benchmark's "auto (RAM policy)"; `0` = no pool), plus `FRAMERATE` as
the frame target, and everything from test 1.

```bash
GPR_OVERFLOW=512 FRAMERATE=30 ./run_live.sh    # how long does 512 MiB last at 30fps
GPR_OVERFLOW=-1  ./run_live.sh                 # auto RAM policy
```

## Live system telemetry

The panel shows the same System readings as the benchmark's curses UI,
sampled twice a second from the same sources: per-core CPU bars, total CPU,
CPU clock (MHz), SoC temperature, and RAM used/total. These are sampled in
cinepi-gui, not in the encoder, so the recording path does no telemetry
work and the shared-memory stats ABI is unchanged.

## Not carried over: spare-core helper

The benchmark's "spare-core helper" toggle is not exposed: the encoder
core's library interface (CinepiQrawConfig) has no field for it, so it
cannot be switched from the live pipeline without extending that
interface. It measured as a small but real loss, so off is what runs.

## cinepi_live_ui.py -- the benchmark UI, on the camera

`./cinepi_live_ui.py` is the terminal benchmark: the same F1/F2/F3 layout,
arrow-key selectors and Enter-to-start as the standalone RUN.sh, but each
pass records from the sensor instead of re-encoding a static image.

Selectors: test (Encode Benchmark / Frame Target), encoder (qraw / dng),
modes (m1..m10, or "all" to queue the whole ladder), framerate, seconds,
workers, overflow pool, quant, compand bits, splice reuse.

Live readouts while a pass runs: encode fps vs camera fps, compression
ratio, output MiB/s, frames encoded / submitted / dropped, encode and disk
queue depths, per-worker wavelet / entropy / dng-wrap / copy-out times, and
the System block (per-core CPU bars, total, clock, SoC temperature, RAM).
Selecting "all" queues ten passes and fills the RESULTS table.

The numbers come from the encoder's shared-memory stats block, read with
the same seqlock protocol cinepi-gui's panel uses -- so the UI can also be
started while a recording is already running, and it will attach.

Note the UI drives cinepi-raw itself; do not run run_live.sh at the same
time, or the two will fight over the camera.

## Terminal preview -- and why it is off by default

Press **v** in the Run tab to show the camera under a blue "LIVE CAMERA"
bar, scaled to whatever room is left in the terminal.

It is **not** GPU-rendered. A terminal cannot display a bitmap; each
character cell is painted as a coloured space from the xterm 216-colour
cube. So every displayed frame costs CPU: fetch a JPEG from cinepi-raw's
mjpegPreview stage over localhost, decode it (PIL), downscale it, and write
one attribute per cell. That is real work on the same four cores the
encoder is using.

Consequences, stated plainly:

- The preview is throttled to ~3 fps and off unless you ask for it.
- A pass recorded with the preview on is **not comparable** to one without.
  Passes run with it enabled are tagged `PV` in the RESULTS table so a
  mixed session cannot mislead later.
- For real numbers, leave it off. Use it to frame the shot, then turn it
  off and record.

Requirements: `python3-pil` (`sudo apt install -y python3-pil`) and a
256-colour terminal. If either is missing the bar says so instead of
failing. The frame source is the mjpegPreview post-processing stage, which
the UI enables automatically via `preview_mjpeg.json` (port 8000) -- so the
preview only works while the UI has cinepi-raw running.

## The zero-CPU viewport: cinepi-gui alongside the terminal UI

The terminal preview costs CPU because a terminal cannot show a bitmap.
`cinepi-gui` can: it imports the camera buffer as a **dmabuf**, binds it as
an EGL image and textures it, so the V3D GPU does the scaling and
composition with no copies and no JPEG decode. That is the viewport to use
while benchmarking.

    Terminal 1:   ./cinepi_live_ui.py          # benchmark, drives cinepi-raw
    Terminal 2:   cd /home/pi/cinepi-gui && ./build/cinepi-gui

Order matters: cinepi-gui attaches to the shared context that cinepi-raw
publishes, so start the UI first and press Enter (or just leave the camera
held) before launching the GUI. Starting cinepi-gui with no cinepi-raw
running is what produces the segfault -- it dereferences a peer that is not
there.

### hold camera

Because cinepi-gui dies when its peer goes away, the UI has a **hold
camera** setting (default on). With it on, cinepi-raw keeps streaming
between passes instead of being torn down and relaunched, so the GUI stays
attached for a whole session and passes start faster (no seven-second
warm-up each time).

The camera is still restarted automatically when a setting that is only
read at startup changes -- encoder, workers, framerate, overflow, quant,
compand bits, splice reuse, or the QRAW mode. Queueing "all" therefore
restarts it ten times and cinepi-gui will not survive that; use a single
mode for GUI-alongside sessions, or accept the GUI dropping out during a
ladder sweep.

Turning hold off restores the previous behaviour (camera released after
every pass), which is the cleaner state if you are not using the GUI.
