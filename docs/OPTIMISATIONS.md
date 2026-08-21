
---

# Live camera capture (r36 + camera link)

> **Dated record, not a specification.** The figures below were measured on one
> CM5, one scene and one software state, on the date given. Throughput, file size
> and compression ratio are properties of a scene and a machine: a noisy frame
> compresses less at every quality mode, and a different Pi, kernel, thermal
> state or desktop session moves the numbers again. They are kept so a change can
> be compared against the same conditions — do not read them as what your rig
> will do, and do not quote them as limits.


`benchmark/cinepi_camera_bench.cpp` captures REAL frames from the REAL
camera and encodes them with the PRODUCTION library -- the same
`cinepi_qraw_create` / `cinepi_qraw_encode` pair `cinepi-raw` uses.

    bash benchmark/build_camera_modes.sh      # builds probe + live bench
    build/cinepi_camera_bench --list
    build/cinepi_camera_bench --seconds 10
    build/cinepi_camera_bench --mode 3856x2180:16:U --seconds 20 --workers 3

It reports what the replay benchmarks structurally cannot:

    sensor fps   frames libcamera actually delivered
    encode fps   frames the encoder finished
    dropped      frames delivered while every worker was busy
    recordable   1 only when nothing dropped

## Design notes that matter

* **One encoder handle per worker**, exactly as `cinepi-raw` does. A handle
  owns its coefficient plane and sidecar; sharing one across threads would
  corrupt both.
* **The completion handler never encodes.** libcamera delivers on its own
  thread, so the handler only hands the buffer to a queue. Encoding inline
  would stall the camera and manufacture drops that are not real.
* **Drops are counted, not hidden.** If every worker is busy the frame is
  recycled immediately and counted -- which is exactly what happens on the
  camera, and is the number the test exists to produce.
* **Stride is handled.** libcamera may pad rows; the encoder wants tightly
  packed `width x height` uint16. Repacking happens only when
  `stride != width*2`.
* **12-bit packed is refused, not guessed.** Only unpacked 16-bit is
  encoded. CSI2P unpacking is not written, and inventing the bit layout
  would produce a plausible but wrong frame. The tool says so and exits
  rather than producing a number.

## How far this is verified

Compiled with `-Wall -Wextra` and LINKED: all 18 libcamera symbols confirmed
present in the shipped `libcamera.so.0.5.2`, and all five
`cinepi_qraw_*` symbols confirmed present in the encoder object. **It has not
been run against a camera** -- there is none in this environment. Run
`--list` first and check it against `rpicam-hello --list-cameras` before
trusting any number from it.

---

# Live-camera runs: what was starving the encoder

The 04:52 three-worker run still dropped frames, and reducing workers made
capture WORSE (48.5 -> 33.4 fps). The contention was not the workers.

## Three streams, two of them unused

    configuring streams: (0) 964x544-YUV420  (1) 3856x2180-SRGGB16/RAW
                         (2) 320x240-YUV420

The benchmark consumes (1). Stream (2) was read by nothing, and every
processed stream makes the PiSP run its colour pipeline and the IPA run
AGC/AWB/CCM on the CPU per frame. That run logged **1247 "No lux level
found" and 1247 "no lux value found"** warnings -- the IPA invoked, and
failing, once per frame, for previews the benchmark discards.

Now: no lores stream, and **fixed exposure/gain by default**. Fixed AE is not
only cheaper, it makes frames repeatable -- under auto AE the picture changes
as the loop converges, so the compression ratio drifts inside a single run.

    CINEPI_LORES=1     restore the 320x240 stream
    CINEPI_AE=auto     restore auto exposure/AWB and the per-frame IPA
    CINEPI_SHUTTER_US  fixed shutter, default 8000
    CINEPI_GAIN        fixed gain, default 2.0

## 29% of frames were disappearing silently

    camera event loop      ~2086 frames   60.0 fps  (its own counter)
    submitted to encoder    1304          37.5 fps
    dropped at the queue     170           4.9 fps
    UNACCOUNTED              612          17.6 fps

`frames_submitted`, `frames_dropped` and `camera_capture_fps` are all counted
at the ENCODER HOOK, so none of them can see a frame that never reached it.
612 frames left the camera and were reported as nothing at all -- which is
why `camera_capture_fps` read 33.4 while the camera genuinely ran at 60.

The runner now counts CinePi-RAW's own frame numbers and publishes the
difference as `frames_lost_upstream`, in the log and in the CSV, with a
warning when it is non-zero. A frame lost without being counted is worse
than one counted as dropped.

## ms_copy was a symptom, not a cost

31.5 ms for a 16.9 MB strided read plus a 16.8 MB tight write is 1.07 GB/s
effective. A CM5 memcpy runs several GB/s. The copy thread was being
descheduled mid-copy -- the same starvation showing up in `ms_wavelet`
(59.5 ms against 29.3 static). Worth revisiting the stride copy only after
the ISP work is gone; right now it measures contention, not copying.

## Live view was never running

`GUI_PID` was declared at the top of the runner and killed in the trap, but
nothing ever assigned it: `cinepi-gui` was never started, despite the banner
printing `preview=CINEPI_GUI`. It now starts once the RAW stream is
confirmed up -- so a failed camera cannot orphan a GUI -- and reports whether
it stayed alive.

    CINEPI_LIVEVIEW=0   headless, and cheaper: the preview stream is what
                        makes the ISP run at all

## Running lean: what is left, and what four workers really costs

### Preview: keep it, at shift 1

Live view and "lean" pull against each other -- the preview stream is what
makes the ISP run at all -- but the cost is entirely a resolution choice:

    shift 0   3856x2180   504 MPix/s of ISP output at 60 fps
    shift 1   1928x1090   126 MPix/s      (the default)
    shift 2    964x544     32 MPix/s
    shift 3    482x272      8 MPix/s      (was the default)

**v3.9.1 correction.** Shift 3 was made the default on the MPix/s column
above, and that column is the wrong one to optimise. The ISP re-reads the
whole 16.81 MB raw frame to produce a preview of *any* size, so the dominant
term -- the read -- is already paid at every shift. Only the write scales:

    shift 3    482x272   0.20 MB written per frame
    shift 1   1928x1090  3.15 MB written per frame   (+2.95 MB/frame)

At RAW16/30 that is ~88 MB/s more DRAM write against a ~504 MB/s raw read.
What it bought was a live view that is a 4x upscale of a 482x272 image in the
preview window -- reported from the field as "incredibly pixelated", and
correctly so. Shift 1 is half the sensor's line count and lands 1:1 on a 1080p
display.

The UI exposes this directly as the **live view** row (1928x1090 / 964x544 /
482x272 / off). `CINEPI_SHIFT_ORDER` changes the probe order,
`CINEPI_PREVIEW_SHIFT` pins one without probing, and `CINEPI_LIVEVIEW=0` drops
the viewer entirely for the cheapest possible measurement.

### 2026-08-18: zero-copy is ON by default, and the live rate is fixed

The lifetime question this document left open ("the copy can go only where the
buffer is provably held for the duration of the encode") is answered: the
recorder holds its `CompletedRequestPtr` across `EncodeBuffer2()`, and the
`BufferReadSync` is now held alongside it. `CINEPI_QRAW_ZERO_COPY` defaults to 1,
`CINEPI_QRAW_ZEROCOPY_INFLIGHT` to 6, and run_live.sh passes `--buffer-count 12`
because rpicam-apps' default of 6 is not enough once a buffer is held for a whole
encode. RAW16 m7 E1 went from 26.5 fps -- degrading to 24.2 from the second take
onwards -- to 29.8-30.0 fps sustained. Full write-up, including everything that
was ruled out first, in `docs/FINDING_LIVE_CAPTURE_RATE.md`.

### The 16-bit endian-swap skip is a headless-only optimisation

`LIBCAMERA_RPI_SKIP_16BIT_ENDIAN_SWAP=1` removes libcamera's software
big-endian swap of the full 16.81 MB RAW16 frame and lets the encoder fold it
into its own load. Worth 1.8% at m7/RAW16 (30.24 vs 29.71 fps).

It was defaulted on for all RAW16 runs, and that was wrong, because the encoder
is not the only consumer of that buffer:

* the **PiSP Back End** debayers the same buffer to build the preview and
  cannot undo the swap -- the live view becomes white static (measured hf 36.0
  against 2.1);
* the **staging-copy path** does not fold the swap either (`cfg_.src_byteswap`
  is set inside `if (zero_copy_)` only), so `CINEPI_QRAW_ZERO_COPY=0` silently
  recorded byte-swapped pixels: ratio 1.94 against 6.05, still decoding cleanly.

It is now defaulted on only when the encoder's in-register swap really is the
only thing that interprets the samples: no live view, and zero-copy in use.
An explicit `=1` with a live view is honoured with a warning; with zero-copy off
it is refused. Full write-up in `docs/FINDING_CLEARHDR_BLACK_FRAME.md`.

### Four workers: the earlier verdict is STALE, re-test it

Selector 4 is 3 full-frame workers plus a Core0 SB8x4 entropy assist -- not
a fourth full worker, which is a fair design. It failed on 2026-08-14 because
Core0 was saturated by the ISP and the IPA, not because the assist is wrong.
With the lores stream gone, auto-AE gone and the preview at shift 3, Core0 is
a different place.

**Re-run selector 4 before assuming it cannot work.** Watch `ms_entropy`: the
assist should pull it down, and if it instead pushes `ms_wavelet` up, Core0 is
still oversubscribed.

### Log volume: 256 lines per second

The 04:52 run wrote 8,972 log lines in 35 s. Removing the AGC/CCM spam takes
~2,495 of those, but CinePi-RAW still prints two lines per frame -- the frame
number and an exposure summary -- which is ~120 lines/s of formatted output
and file I/O on the same cores doing the encoding. It is not the dominant
cost, but it is pure overhead during a measurement and it belongs behind a
quiet flag in CinePi-RAW. Not fixed here: that printing is in the CinePi-RAW
event loop, which is not in this package.

### The biggest remaining encoder-side win: delete the copy

In the one clean run -- 16-bit, 30 fps, zero drops -- the per-frame budget was
copy 7.9 ms, wavelet 69.4 ms (starved; ~30 ms unstarved), entropy 17.9 ms.
Once the starvation is gone that is roughly 30 + 18 + 8 = 56 ms, so the copy
is about **14% of the frame budget**, spent moving 16.8 MB that the encoder
could read where it lies.

The encoder already supports it: `v2_fused_frame` takes `src_stride_elems`,
`crop_x` and `crop_y`, and the static benchmark uses exactly that to read a
padded 3856x2180 transport buffer in place. Only the library's C API insists
on tightly packed input. Adding a stride field to `CinepiQrawConfig` would
remove the copy outright.

The counter-argument is in the existing log line -- "workers never touch
camera DMA" -- which is a deliberate ownership boundary. If that is about
DMA buffer lifetime rather than correctness, the copy could at least be
narrowed to the 3840x2160 active window instead of the full padded frame.

## v3.10: the copy is gone

The last structural item. The camera path memcpy'd every frame out of the DMA
buffer -- "one copy to 8 tight DRAM slots" in the recorder log -- purely
because the library's C API demanded tightly packed input. At 3856x2180 that
is 16.8 MB per frame, about **14% of the per-frame budget** once contention is
removed, spent removing padding the cascade can simply skip over.

It never needed to. `v2_fused_frame` has taken a source stride and crop since
the transport work, and the static benchmark has always used them to read a
padded 3856x2180 buffer in place. Only the API insisted.

    cfg.width = 3840; cfg.height = 2160;   /* the active window          */
    cfg.src_stride_elems = 7744 / 2;       /* DMA row pitch, in SAMPLES  */
    cfg.src_crop_x = 8; cfg.src_crop_y = 10;

Zero keeps the old tightly-packed contract, so every existing caller is
unaffected.

### Proven, not asserted

`tools/verify.sh` now runs FOUR paths, and the fourth builds a padded buffer
whose padding is poisoned with 0xDEAD, then encodes it through the stride
path:

    emit=shipped          2988814 bytes  72f44899
    emit=v2               2988814 bytes  72f44899
    emit=library          2988814 bytes  72f44899
    emit=library-strided  2988814 bytes  72f44899

Byte-identical. And the check has teeth: shifting the crop by two pixels
changes the output to 71a9ed20 on 2992834 bytes, so a wrong stride or crop
cannot pass silently.

### The one caveat, which is the caller's to answer

With a stride the encoder reads the caller's buffer DIRECTLY during
`cinepi_qraw_encode()`, so it must stay valid and unmodified for that call.
The recorder's "workers never touch camera DMA" boundary exists for a reason:
if the DMA buffer can be recycled asynchronously underneath a running encode,
keep the copy. `cinepi_camera_bench` uses the stride path by default because
it holds the request until the encode returns; `CINEPI_NO_STRIDE=1` restores
the repack.

Wiring this into CinePi-RAW's recorder is the remaining step, and it is a
lifetime question rather than a performance one: the copy can go only where
the buffer is provably held for the duration of the encode.

### Where the strided read can be used today, and where it cannot

**cinepi_camera_bench: yes, and it is on by default.** It owns its libcamera
requests and does not recycle one until the encode returns, so the buffer
provably outlives the read. It reports which path it took:

    input:  strided read in place -- no per-frame copy

`CINEPI_NO_STRIDE=1` restores the repack for comparison. That comparison IS
the fps test: same camera, same mode, same encoder, one memcpy apart.

**The CinePi-RAW recorder: no, and it now refuses.** Its contract is
explicit in the source:

    "EncodeBuffer2() then returns; CinePiRecorder ends BufferReadSync and
     only after that fires its existing callbacks to recycle the request."

The camera buffer is recycled when `submit()` returns. Reading it during the
encode would not crash -- it would quietly encode whatever the camera wrote
next. `CINEPI_QRAW_ZERO_COPY=1` is wired, checks that `input_shift_ == 0`,
and then **refuses with an explanation** rather than half-enabling.

Half-enabling was the real hazard, and it is worth recording because it was
nearly shipped: setting `cfg.src_stride_elems` while `stage_copy()` still
ran would have handed the encoder a TIGHT staged buffer while telling it the
rows were padded by 32 bytes -- an out-of-bounds read producing a
plausible-looking wrong frame. A flag that looks enabled and is subtly wrong
is worse than one that says no.

Two things must change before the recorder can use it, and neither is in
this package:

1. `CinePiRecorder` must hold `BufferReadSync` until the encode completes,
   not until `submit()` returns.
2. `EncodeItem` currently owns a staging-pool slot returned by
   `stage_give()`; carrying a camera pointer instead is an ownership change.

And note (1) alone is not enough for 12-bit: at `input_shift_ != 0` the
staging pass is not a copy at all, it right-shifts MSB-justified PiSP
samples down to sensor codes. Only 16-bit input can skip it without moving
that shift into the compand LUT.

---

# v3.11: the lean camera settings are reverted to opt-in

The 05:25 run aborted at every preview shift:

    terminate called after throwing an instance of 'std::system_error'
      what():  Invalid argument
    FATAL: CinePi-RAW could not expose unpacked IMX585 RAW12 at 60 fps

That was a regression I introduced. The previous revision made two untested
changes the DEFAULT at once -- dropping the lores stream, and replacing auto
AE with `--shutter/--gain/--awb off/--denoise off` -- against a pipeline that
was working. All three shifts failed identically and immediately after camera
registration, so the probe order was not involved; one of the new arguments
was rejected, and with cinepi-raw's option parser outside this package there
is no way to say which from here.

**The defaults are the known-good invocation again.** Everything lean is
opt-in and separable, so it can be bisected one change at a time on hardware:

    CINEPI_NO_LORES=1    drop the 320x240 stream nothing reads
    CINEPI_AE=fixed      fixed shutter/gain instead of auto
    CINEPI_AWB_ARG=...   only if you want --awb; nothing is passed by default,
                         because "off" is not a documented AWB mode and is the
                         prime suspect
    CINEPI_SHIFT_ORDER="3 2 0"   cheaper preview first
    CINEPI_LEAN=1        the first two together

`--denoise off` is gone entirely: it was in the same untested batch and buys
nothing measurable next to the ISP work.

## The lesson, recorded because it cost a run

The reasoning behind those settings still holds -- 1247 AGC warnings per run
for previews the benchmark discards is real waste, and fixed AE really does
stop the compression ratio drifting mid-run. What was wrong was making them
DEFAULT without a way to test them, and changing two things at once so the
failure could not be attributed. Anything that touches the camera invocation
should land as opt-in first and be promoted only after a run on hardware.

---

# Testing that only required processes run

`benchmark/RUN_LEAN_BISECT.sh` answers two questions the earlier work could
only guess at: which lean setting CinePi-RAW rejects, and whether the
pipeline is running anything it does not need.

    bash benchmark/RUN_LEAN_BISECT.sh          # all five configurations
    bash benchmark/RUN_LEAN_BISECT.sh fixedae  # one of them

Five configurations, each differing from the baseline by EXACTLY ONE setting,
so a failure identifies itself -- which is precisely what the 05:25 abort
could not do, because two things changed at once:

    baseline   shift 2, lores on,  auto AE     (known good)
    shift3     shift 3                          cheaper preview only
    nolores            lores off                drop the unread stream only
    fixedae                       fixed AE      fixed exposure only
    lean       shift 3, lores off, fixed AE     all three

Per configuration it reports whether the RAW stream configured or the process
aborted, how many streams were created, **how many of them nothing reads**,
IPA warnings per frame, log lines per frame, and delivered fps. Then it names
the leanest configuration that worked and prints the env line to adopt it.

## The audit

    PROCESS AUDIT (leanest working configuration: ...)
      FAIL  2 processed stream(s) nothing reads
      FAIL  IPA runs 1.22 times per frame (AGC/CCM warnings)
      INFO  CinePi-RAW writes 4.39 log lines per frame
      AUDIT FAIL: the pipeline is doing work it does not need.

Those numbers are from the real 04:52 and 05:25 logs, not invented -- the
parser was developed against them.

## The parser has its own test

The parser turns a log into a verdict, so a regression in it would quietly
turn "2 unused streams" into "0" and the audit would start passing for the
wrong reason. `benchmark/testdata/test_lean_parser.py` runs it against three
fixtures -- a three-stream log, a raw-only log, and the 05:25 abort -- and
checks every extracted field. Verified by breaking the unused-stream
detection on purpose and confirming the test fails.

It needs no camera: `LEAN_PARSE_ONLY=1` reads a saved log, which is what
makes the parser testable off the Pi.

---

# The 07:26 bisect: what it settled, and a bug in the harness

Five configurations, none aborted. That alone settles the 05:25 failure.

## The abort was --awb off / --denoise off

`fixedae` here passes only `--shutter` and `--gain` and configures fine. The
05:25 version passed `--shutter --gain --awb off --denoise off` and aborted.
`--awb off` is not a documented AWB mode. Those two arguments are gone and
must not come back without being bisected first.

## Promoted to default, verified on hardware

    CINEPI_NO_LORES=1            3 streams -> 2, one unread YUV gone
    CINEPI_SHIFT_ORDER="3 2 0"   preview 964x544 -> 482x272, 4x less ISP
                                 REVERTED in v3.9.1: the default is now
                                 "1 2 3 0" (1928x1090). See "Preview: keep
                                 it, at shift 1" above for why the saving
                                 was smaller than the picture cost.

Both configured cleanly. `CINEPI_NO_LORES=0` restores the old behaviour.

## Fixed AE does NOT stop the IPA -- a negative result

Every configuration, **including fixedae**, reported 2.00 AGC/CCM warnings
per frame. Manual exposure sets the controls but leaves the AGC algorithm
running, and it still fails to find a lux level. The audit used to call this
a FAIL with "with fixed exposure it should be 0.00"; that was wrong and now
says so.

Silencing it needs the tuning file to drop `rpi.agc`, or manual AWB gains --
and `--awb off` is the thing that aborted, so it cannot simply be added back.
Fixed AE is still worth having for repeatability: it stops the compression
ratio drifting as the AE loop converges. It is not promoted to default
because it buys repeatability, not throughput, and it changes the picture.

## The fps column was measuring my own bug

Every row read 30.01 fps against a 60 fps request. Not the pipeline -- the
harness. CinePi's `controller.sync()` reads persisted Redis keys and
**upstream defaults fps to 30**; the real runner seeds them before every
process for exactly this reason, and the bisect did not. The logs said so:

    Mode selection for 3856:2180:12:U(30)
      SRGGB12_CSI2P,3856x2180/60.0024 - Score: 0

The sensor offered 3856x2180 at 60.0024 and the request asked for 30. The
harness now seeds `width`/`height`/`fps` before each launch, warns if
`redis-cli` is missing, and `test_lean_parser.py` asserts the seeding is
still there.

**The stream and IPA columns were unaffected** -- they do not depend on frame
rate -- so the two promotions above stand on valid data. Re-run the bisect to
get meaningful fps numbers.

## The audit was also wrong about the preview

It counted the live-view preview as a stream "nothing reads" and failed the
configuration for showing you the picture you asked for. One processed stream
is now expected when `CINEPI_LIVEVIEW=1`; only extras beyond that are waste.

---

# The 07:38 bisect: fps fixed, and the IPA metric was wrong too

With Redis seeded, every configuration reports **60.00 fps** and the audit
passes. That confirms the sensor does 3856x2180 12-bit at 60, and that the
earlier 30.01 was purely the missing seed.

    config     configured streams unused  fps
    baseline   ok        3        2       60.00
    shift3     ok        3        2       60.00
    nolores    ok        2        1       60.00
    fixedae    ok        3        2       60.00
    lean       ok        2        1       60.00
    AUDIT PASS: nothing unnecessary is running.

## The IPA is a constant cost, not a per-frame one

`ipa/frm` read 2.00 at 30 fps and 1.02 at 60. That is not the IPA halving --
it is a fixed cadence divided by a changing frame rate. Measured directly:

    30 fps run:  30.4 IPA invocations/s
    60 fps run:  30.4 IPA invocations/s

The IPA runs at ~30 Hz regardless, emitting two failed-lux warnings each
time. Reporting it per frame made a constant cost look like it improved when
the frame rate rose, which would have led to exactly the wrong conclusion.
The column is now **ipa/s** and **log/s**, and the parser test pins the
30/s figure against a fixture.

## fixedae confirmed not to help, twice

`fixedae` measured the same IPA rate as `baseline` in both runs.
`--shutter/--gain` sets the controls but leaves the algorithm running.

A new `awbgains` configuration tests `--awbgains R,B` -- a DOCUMENTED option
that puts AWB in manual mode, which is what `--awb off` was reaching for when
it aborted. If it silences the 30 Hz work, that is the setting to promote.
The parser test asserts `--awb off` is never passed in any AE_ARGS branch,
scoped to the argument construction so the comments warning against it do not
trip it.

## Where the pipeline stands

Only the RAW stream and one preview remain, and the preview is wanted. What
is left is not stream configuration:

    ~30/s   IPA invocations for a preview the benchmark discards
    ~190/s  log lines CinePi-RAW writes (3.11 per frame at 60 fps)

Both belong in CinePi-RAW -- the tuning file and a quiet flag -- and neither
is in this package.
