# Building on the Pi 5 / CM5 (r17 CinePi-RAW integration)

## Recommended r17 path

The normal build/install path is now the package-level installer. It preserves
CinePi-RAW as the camera/preview/recording application and installs VC-5/QRAW
only at its existing encoder boundary.

```bash
cd ~/Downloads/cinepi-qraw-live-v4.0
chmod +x run_qraw_pipeline.sh
./tools/run_qraw_pipeline.sh
```

For the IMX585 ClearHDR 16-bit mode:

```bash
SENSOR_BITS=16 ./tools/run_qraw_pipeline.sh
```

For normal live operation after the install:

```bash
./run_live.sh                 # RAW12, 60 fps target
SENSOR_BITS=16 ./run_live.sh  # RAW16 ClearHDR, 30 fps target
```

`run_qraw_pipeline.sh` fingerprints the bundled encoder sources and rebuilds
`/opt/cinepi-qraw` only when those sources change. It then patches/rebuilds the
existing `~/cinepi-raw` checkout idempotently. The normal camera benchmark also
uses this real CinePi-RAW path; the standalone libcamera adapter is diagnostic
only.

## Historical/manual integration notes

The older step-by-step notes below are retained for provenance and debugging.
They predate the r17 one-shot installer and contain old package-version
examples; prefer the commands above.


This is one self-contained zip: the benchmark package lives at `benchmark/`
inside it, so the encoder core builds from the bundled copy by default --
no second download, no separate path to manage.

Every command below was tested before being written down: the three
cinepi-raw patches and the one cinepi-gui patch were verified with
`git apply --check` against fresh clones of the real repos, applied in the
exact order below, and the resulting source compiled clean. The one
exception is noted at step 3 (meson configure/link on real hardware) --
see docs/VALIDATION_STATUS.md.

Order matters: encoder core, then its regression gate, then cinepi-raw,
then cinepi-gui. If step 2 does not print the expected line, stop --
nothing downstream is trustworthy.

## 0. Get the zip onto the Pi and unzip it

```bash
scp cinepi-qraw-live-v0.1.zip pi@cinepi.local:~/
ssh pi@cinepi.local
cd ~
unzip -o cinepi-qraw-live-v0.1.zip
cd cinepi-qraw-live-v0.1
```

Prerequisites:
```bash
sudo apt update
sudo apt install -y cmake build-essential git
```
The CinePI SDK image already carries libcamera (cinepi fork), meson, boost,
spdlog, redis++, jsoncpp, and SDL2/GLES for the GUI.

## 1. Encoder core

```bash
chmod +x encoder_library/build_qraw_core.sh
./encoder_library/build_qraw_core.sh
```
No path argument needed -- it builds from the bundled `benchmark/` folder
in this same zip. (Pass a path explicitly only if you want to build against
a different copy of the benchmark package: `./encoder_library/build_qraw_core.sh /path/to/other/copy`.)

This installs static archives and the fixed header under `/opt/cinepi-qraw`
(pass a second argument to use a different prefix, then add
`-Dqraw_prefix=<that path>` to the `meson setup` command in step 3).
aarch64 builds get `-mcpu=cortex-a76 -flto=auto`, matching the benchmark's
own build flags so the numbers stay comparable. It checks for
`libvulkan.so.1` up front and tells you the exact `apt install` line if
it's missing.

At the end it prints the exact link command for step 2 -- copy it from
your terminal output rather than retyping it, since it depends on which
static archives your CMake build actually produced.

## 2. Regression gate -- do not skip

Step 1 prints the exact commands for this, copy them verbatim from your
own terminal output -- but here's what they look like and why, since both
points below were real bugs caught and fixed by actually running this
end-to-end rather than assuming the obvious command would work:

```bash
gcc -O2 -c encoder_library/white_ab.c -I/opt/cinepi-qraw/include -o /tmp/white_ab.o && \
g++ /tmp/white_ab.o -Wl,--start-group /opt/cinepi-qraw/lib/*.a -Wl,--end-group \
    -lpthread -ldl -lm -o white_ab
```
Two things about this that aren't obvious:
- **`white_ab.c` is C, but the SDK archives are C++** (`dng_sdk` etc. use
  `new`/`delete`, exceptions, typeinfo). Compiling with `gcc` and linking
  with `g++` pulls in `libstdc++` automatically; linking straight from
  `gcc` fails with `undefined reference to operator delete` and similar.
- **`-Wl,--start-group ... -Wl,--end-group`** wrapping the glob makes
  linking order-independent. Which exact archives your CMake build
  produces (and their filesystem order from the glob) can vary machine to
  machine; without this, a plain ordered list can fail to resolve symbols
  depending on what got built.

Then run it -- **the working directory matters and is easy to get wrong**:
```bash
cd benchmark
../white_ab cinepi_qraw_bench/input/sample_imx585_3840x2160_gbrg_16bit.raw16
```
`gpr_params.json` is looked up as a path relative to the process's current
working directory, not relative to the binary or the input file. Running
`white_ab` from anywhere other than the `benchmark/` folder itself fails
with "Cannot parse QRAW metadata JSON" even though every file involved
exists and the binary is fine.

Required output:
```
verdict: FIXED LIBRARY OK -- both legs match the benchmark reference
```
This proves, on this machine and this compiler, that the white-level fix
is in and the encode path lands on 3,052,650 bytes for m5 -- the same
number `vc5_bench` itself produces. If you see anything else, stop and
paste me the full output; nothing past this point should be trusted.

## 3. cinepi-raw

```bash
cd ~/cinepi-raw          # the SDK submodule, branch cinepi-sdk-001
git status               # confirm this is clean before patching
```

Copy the new source files in (adjust the path back to wherever you
unzipped the package):
```bash
PKG=~/cinepi-qraw-live-v0.1
cp $PKG/integration/cinepi-raw/raw_frame_encoder.hpp cinepi/
cp $PKG/integration/cinepi-raw/qraw_encoder.hpp cinepi/
cp $PKG/integration/cinepi-raw/qraw_encoder.cpp cinepi/
cp $PKG/integration/cinepi-raw/qraw_live_stats.hpp cinepi/
```

Apply the three patches, in order, from the repo root:
```bash
git apply $PKG/integration/cinepi-raw/patches/0001-dng-encoder-interface.patch
git apply $PKG/integration/cinepi-raw/patches/0002-recorder-encoder-selection.patch
git apply $PKG/integration/cinepi-raw/patches/0003-meson-qraw.patch
```
Each of these was tested with `git apply --check` against a fresh clone of
this exact branch before being included in the package. If any of them
reports a conflict, your checkout has diverged from upstream
`cinepi-sdk-001` -- run `git diff` first to see how, since that changes
what needs hand-reconciling.

Build:
```bash
meson setup build -Dqraw_prefix=/opt/cinepi-qraw   # omit -Dqraw_prefix if you used the default
meson compile -C build
```

**If you're running a libcamera build with pinned/older symbol names**
(confirmed on the CinePi IMX585 ClearHDR 4K30 patch, which builds
libcamera at commit `bfd68f78`), the build may fail with errors like
`'AeLocked' is not a member of 'libcamera::controls'` or
`'RGGB16_PISP_COMP1' is not a member of 'libcamera::formats'`. These are
not QRAW-integration problems -- they're pre-existing gaps between that
pinned libcamera commit and files `cinepi-raw` synced from a different
point in upstream history. Four targeted patches exist for exactly this:
see `cinepi-raw/patches/clearhdr-compat/README.md` for which patch fixes
which exact error, applied strictly after `0001`-`0003` above and only for
the specific errors you actually hit. **Do not apply these if your build
doesn't show these exact errors** -- on a stock libcamera they'd remove a
working feature or break a correct symbol name instead of fixing anything.
This combination (`0001`-`0007`) has been built and linked successfully on
real Pi 5/CM5 hardware running the actual ClearHDR-patched libcamera.

Before switching to the QRAW path, verify the patched tree still records
DNG exactly as before -- this isolates a refactor regression from a QRAW
problem:
```bash
./build/cinepi/cinepi-raw   # your normal flags; confirm a short DNG clip still records
```

## 4. cinepi-gui

```bash
cd ~/cinepi-gui
git status
```
Copy the new files in (the GUI patch only touches existing files -- these
three are net-new and must be present before the patch is applied):
```bash
PKG=~/cinepi-qraw-live-v0.1
cp $PKG/integration/cinepi-gui/BenchmarkPanel.hpp .
cp $PKG/integration/cinepi-gui/BenchmarkPanel.cpp .
cp $PKG/integration/cinepi-raw/qraw_live_stats.hpp .   # verbatim -- it IS the protocol
```
Apply the patch:
```bash
git apply $PKG/integration/cinepi-gui/patches/0001-benchmark-panel.patch
```
Add the new source file to the build (the patch does not touch
`meson.build`'s source list, since that line's exact position varies by
version):
```bash
grep -n "'Overlays.cpp'" meson.build
```
Add `'BenchmarkPanel.cpp',` on its own line next to that entry, then:
```bash
meson setup build
meson compile -C build
```

## 5. Run

Your confirmed sensor setup: sensor/CFE readout is 3856x2180 RAW12 packed
(used for live-view scaling only, via the ISP); the recording path uses
the sensor's 16-bit mode at the full 3840x2160 active picture area, which
divides cleanly by 16 (240 x 135) for the wavelet transform:

```bash
CINEPI_ENCODER=gpr CINEPI_QRAW_MODE=m5 \
    ./build/cinepi/cinepi-raw --mode 3840:2160:16:U <your usual flags>
```
If the stream comes up PISP_COMP1 instead, the encoder refuses on the
first recorded frame and prints the exact `--mode` string to use --
deliberately loud, because encoding COMP1 would silently measure the
wrong thing.

Then run the GUI as usual:
```bash
./build/cinepi-gui
```
The panel attaches automatically once the QRAW encoder starts; without it
running, the panel says "QRAW encoder not running" and the preview behaves
exactly as stock.

`CINEPI_QRAW_BITS` (default 16) exists if you ever want to record at a
different bit depth than 16 -- leave it unset for the setup above.

## 6. First-light checklist

- [ ] step 2 gate green on this machine (aarch64, not just x86-64)
- [ ] stock DNG record still works on the patched tree (step 3's refactor gate)
- [ ] QRAW record: SGBRG16 accepted, PISP_COMP1 refused with the documented message
- [ ] a recorded `.gpr` opens (gpr_tools / Adobe Camera Raw); blacks sit at 0,
      not lifted (this checks the `SensorBlackLevels` pedestal wiring)
- [ ] a 30 s m5 record: panel's dropped-frame counter stays at 0
      (note: campaign fps numbers were measured on x86-64 CPU-only and do
      not transfer -- compare against Pi 5 numbers you gather yourself)
- [ ] `copy_out_ms` on the panel: if it's a visible slice of the frame
      budget, see qraw_encoder.cpp's header comment for the inline-writer
      alternative
