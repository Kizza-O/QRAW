# CinePi GPR Quick Start

Canonical standard: **v1.16.6 winning stack + IRQ isolation to Core0 + VLE128/locality0**. There is no experimental encoder selector in this package.

## First: white balance

```bash
cd benchmark
./calibrate_wb.sh
```

Once per camera, and again when the lighting changes. It measures neutral white
balance from a live RAW12/60 frame and stores it in `cinepi_wb_base.json`.
Camera runs then use it for the preview gains of the modes that cannot run auto
AWB and for the white balance stamped into the GPR/DNG. Skip it and frames carry
the stock GoPro parameter file's white balance and open with a cast.

## The front door

```bash
cd benchmark
./RUN.sh
```

The full-screen UI. `--no-ui` gives plain prompts; any flag means scripted use.

## How fast does it encode

```bash
./RUN.sh --test encode --mode all --workers 4
```

Use `--workers 3` for live camera input: the Core0 entropy assist that selector
4 adds is a win on static input, where Core0 is idle, and a loss on live input,
where it is not.

## Every kind of GPR file the camera can produce

```bash
./RUN.sh --test allgpr --gpr-path /media/RAW
```

Coverage, not speed. One decode-verified frame of every capture path × effective
precision × band setting × quality mode, each in its own labelled folder with a
`MANIFEST.tsv` saying what it is. The only choice is where the tree goes.
`./RUN_ALL_QRAW_OUTPUT.sh --dry-run` lists the variants without opening the
camera.

## Recommended sensor mode

**ClearHDR RAW16 delivered as 12-bit companded** — full ClearHDR highlight
range, valid statistics so auto 3A works, 12-bit samples, and a `.gpr` that
renders natively and matches its own DNG. 30 fps readout. Use plain RAW12 when
frame rate matters more than highlight range.

## Fixed M7 winner proof

```bash
./RUN_M7_WINNING_STACK.sh
```

## Live CinePi-RAW

From the package root:

```bash
../tools/run_gpr_pipeline.sh
./run_live.sh
```

The production runners apply IRQ isolation for encode work and restore the previous affinities on exit. `setup_appliance.sh` can install narrow passwordless permissions for non-interactive appliance use.

## Verify package policy

```bash
./tools/check_single_encoder.sh
cd benchmark && ./check_pi_environment.sh
python3 benchmark/test_ui_selection.py
```

## Reading the results

Throughput, file size and ratio depend on the scene and the machine, so compare
modes within one of your own runs rather than against a number from a document.
The ordering is what is fixed: m1 highest quality and largest, m10 smallest.
