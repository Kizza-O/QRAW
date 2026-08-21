# CinePi GPR Production Benchmark

All production runners use one encoder implementation: **v1.16.6 winning stack + IRQ isolation to Core0 + VLE128/locality0**. The original winning-stack transform, entropy, handoff, rendezvous and worker settings remain fixed.

## Start here

`./RUN.sh` is the front door. With no arguments it opens the full-screen UI; `--no-ui` gives plain prompts, and any flag means scripted use. It offers:

- **Encode Benchmark** — how fast the pipeline encodes, per mode.
- **All GPR Camera Output** — every KIND of GPR file the camera can produce.

Run `./calibrate_wb.sh` once before shooting anything (see White balance below).

## Runners

- `RUN.sh`: the front door for every test; `--help` lists the flags.
- `RUN_BENCHMARK.sh`: static/file M1-M10 throughput, and the dispatcher the live camera path enters through.
- `RUN_CINEPI_RAW_BENCHMARK.sh`: real CinePi-RAW camera benchmark using the same encoder library.
- `RUN_ALL_QRAW_OUTPUT.sh`: **file coverage, not speed.** Walks every combination that changes what is inside a `.gpr` — the four camera capture paths, 12/11/10-bit effective precision, shipped encoder and experimental band pruning, m1–m10 — and leaves one decode-verified frame of each in its own labelled folder with a `MANIFEST.tsv`. Each variant is a separate camera session, because the encoder reads its mode, precision and band setting once at construction; that is what the run costs. `--dry-run` prints the coverage list without opening the camera, `--only` re-runs individual variants.
- `calibrate_wb.sh`: measures this camera's neutral white balance from a live RAW12/60 frame into `cinepi_wb_base.json`. `--show` prints it, `--force` re-measures.
- `RUN_CAPTURE_TEST.sh`: fixed-rate frame-buffer/record-duration model using the same canonical encoder.
- `RUN_M7_WINNING_STACK.sh`: fixed M7 proof run. It does not search alternatives.
- `RUN_MATRIX_TEST.sh`: integration/build fault diagnostics, not an encoder A/B matrix.

`tools/check_single_encoder.sh` fails if an alternate encoder entry point, old lab, shader encoder bundle, or historical optimisation runner is reintroduced.

## Recommended sensor mode

**ClearHDR RAW16 delivered as 12-bit companded** — the default ClearHDR delivery, `INPUT_BITS=16` with `CINEPI_CLEARHDR_BITS=12`. It is the only camera path with the sensor's full ClearHDR highlight range, valid PiSP statistics so auto 3A works, 12-bit samples rather than 16-bit-wide ones, and a `.gpr` that renders natively and matches its own DNG. It reads out at 30 fps; plain RAW12 reads out at up to 60 fps and is the choice when frame rate matters more than highlight range.

## White balance

```bash
./calibrate_wb.sh          # once per camera, and again when the light changes
./calibrate_wb.sh --show   # what is stored
```

Without a measured base, two things are wrong. The modes that cannot run auto AWB — anything on manual 3A — get no preview correction at all, which is why every mode except RAW12/60 looked off. And every frame carries the white balance of the stock GoPro parameter file, so it opens with a cast in any reader regardless of mode. One measurement fixes both: neutral white balance is a property of the sensor and the light, not of the readout mode.

Colour gains are metadata. They change what a reader renders and never touch the Bayer samples the encoder wrote, so nothing here can move a compression or throughput figure.

## About the numbers

Frames per second, file size and compression ratio describe **a scene on a machine**, not the encoder. A quality mode pins a quantiser ladder, not a ratio: a noisy scene produces larger files and fewer frames per second at every mode than a smooth one, and a different Pi, kernel, thermal state or desktop session moves the throughput again. Figures in these documents are dated records of one rig, kept so a change can be compared against the same conditions — not specifications. Compare modes *within* one of your own runs.

What is fixed, and stated as fact: m1 is the highest quality and largest, m10 the smallest; 12-bit effective precision is the master and 10-bit the coarsest; the ladder is monotonic at every band and the self-test enforces it every run.
