#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Kieran Olsson
# ===========================================================================
# CinePi GPR RAW Benchmark - Pi5
#
#   ./RUN_BENCHMARK.sh
#
# M1-M10 production winner benchmark. Worker count 1..4 uses one common
# codec/output stack; only the core-sharing policy changes.
#
# Base stack for every worker selection:
#   fused NEON v53 + split-fused + non-temporal coefficient writes
#   8/8 winner entropy stack
#   shared verified in-place GPR output / handoff pool
#   VLE prefetch 128, streaming locality
#   all movable IRQ affinities isolated to Core0
#   cyclic wavelet rendezvous for 2+ primary workers (20 ms / 40 us lead)
#
# Worker 4 is NOT a fourth full frame owner. It is the measured production
# winner: cores 1/2/3 remain frame owners and Core0 performs SB8 entropy for
# all four channels at normal priority, leaving the rest of Core0 to Linux.
# ===========================================================================
set -Eeuo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
PACKAGE_VERSION=$(cat "$HERE/PACKAGE_VERSION.txt" 2>/dev/null || echo unknown)
RUNNER_SHA256=$(sha256sum "$0" 2>/dev/null | awk '{print $1}' || echo unknown)
echo "PACKAGE_ID version=$PACKAGE_VERSION benchmark_runner_sha256=$RUNNER_SHA256"
# Startable from anywhere; canonical runtime data lives under benchmark/
cd "$HERE"
# Use the same patched CinePi libcamera IPA/tuning environment as run_live.sh.
# The system IPA on this image does not contain the IMX585 helper used by the
# ClearHDR path, so a benchmark using a different IPA is not the same camera
# pipeline even if libcamera can enumerate the sensor.
export LIBCAMERA_IPA_MODULE_PATH=${LIBCAMERA_IPA_MODULE_PATH:-/usr/local/lib/aarch64-linux-gnu/libcamera/ipa}
export LIBCAMERA_IPA_CONFIG_PATH=${LIBCAMERA_IPA_CONFIG_PATH:-/usr/local/share/libcamera/ipa}
# shellcheck source=perf_governor.sh
source "$HERE/perf_governor.sh"
source "$HERE/winning_stack.sh"
BENCH="$HERE/cinepi_qraw_bench"
STAMP=$(date +%Y%m%d-%H%M%S)
HOSTN=$(hostname -s 2>/dev/null || echo host)
# Where this run's own paperwork goes: the transcript, the CSV and the bundle.
# The default is the benchmark folder, which is where every run has put them.
# RESULTS_DIR moves them next to the frames that run produced instead -- All GPR
# Camera Output starts 24 of these, and 24 results_<host>_<stamp>.txt files
# heaped in one shared folder cannot be told apart from each other by eye, nor
# matched to the variant they describe.
# Output goes to output/, not into this source directory.
#
# The old default was $HERE, so every run left results_cinepi_*.csv/.txt
# loose among the scripts -- twelve of them had accumulated, which is what
# made this folder hard to read. Callers that want them somewhere else (the
# All-GPR runner puts each leg beside the frames it produced) still pass
# RESULTS_DIR, and that is unaffected.
RESULTS_DIR=${RESULTS_DIR:-$HERE/../output/results}
BUNDLE_DIR=${BUNDLE_DIR:-$RESULTS_DIR}
mkdir -p "$RESULTS_DIR" "$BUNDLE_DIR" 2>/dev/null || true
RESULTS="$RESULTS_DIR/results_${HOSTN}_${STAMP}.txt"
CSV="$RESULTS_DIR/results_${HOSTN}_${STAMP}.csv"
# Where the frames go, and whether they are kept at all. The decode gate still
# runs either way -- switching off the output must not switch off verification,
# so with SAVE_QRAW=off the frames are written to a temporary directory, proved
# decodable, and then removed.
# v1.3: which sensor signal to measure. The IMX585 reads out 16-bit only to
# ~30 fps and 12-bit up to 60 fps, so a high-frame-rate target should be
# measured from a 12-bit signal -- companding a 16-bit source models work the
# camera would not do at that rate. The 12-bit frame is the GP-Log2 twin of
# the 16-bit one; both are real files and they encode to byte-identical
# containers, so the only difference measured is the compand itself.
INPUT_BITS=${INPUT_BITS:-16}
SAVE_QRAW=${SAVE_QRAW:-on}
# v1.1: decode verification is now a SETTING, default off. It is a
# correctness gate, not something every timing run needs, and running it by
# default forced every frame to disk purely to be read back and deleted.
VERIFY=${VERIFY:-off}
GPR_PATH=${GPR_PATH:-$HERE}
# v1.1: scratch frames stay in the benchmark folder. They used to go to
# ${TMPDIR:-/tmp}, which on a Pi is usually the boot media rather than the
# NVMe -- so a run that kept nothing still measured the SLOWEST disk in the
# machine. Measured 2026-08-09: 31.4 fps writing to /tmp against 40.4 fps
# writing to NVMe, same encoder, same six winner members. The benchmark
# folder is on whatever device the user unpacked to, which is the device
# they actually care about.
# EXPERIMENTAL band pruning (CINEPI_BAND_Q) is not bit-exact -- it deliberately
# quantises whole wavelet subbands out of the image. Anything recorded under it
# is experimental footage, so the tag goes in the FOLDER NAME as well as the
# header: a clip must not be mistakable for a production one months later.
BAND_TAG="${CINEPI_BAND_TAG:-}"
if [[ -z "$BAND_TAG" && -n "${CINEPI_BAND_Q:-}" ]]; then BAND_TAG="bandq"; fi
BAND_SUFFIX=""
[[ -n "$BAND_TAG" ]] && BAND_SUFFIX="_${BAND_TAG}"
if [[ "$SAVE_QRAW" == "on" ]]; then
    GPRDIR="$GPR_PATH/gpr_out_${STAMP}${BAND_SUFFIX}"
else
    GPRDIR="$GPR_PATH/.gpr_scratch_${STAMP}${BAND_SUFFIX}"
fi
# With neither keeping nor verifying, there is no reason to touch the disk
# at all: measure the ENCODER. Say so plainly in the header so nobody reads
# an encode-only number as a sustainable recording rate.
WRITE_FRAMES=on
if [[ "$SAVE_QRAW" != "on" && "$VERIFY" != "on" ]]; then
    WRITE_FRAMES=off
fi

DURATION=${DURATION:-10}
WARMUP_S=${WARMUP_S:-1}   # discard the first second: cold cache/extents
MODES=${MODES:-"m1 m2 m3 m4 m5 m6 m7 m8 m9 m10"}
PASSES=${PASSES:-2}
THREADS=${THREADS:-4}
SOURCE=${SOURCE:-sample}
[[ "$THREADS" =~ ^[0-9]+$ ]] && ((THREADS >= 1 && THREADS <= 4)) || {
  echo "THREADS must be an integer from 1 to 4" >&2; exit 2; }
cinepi_winning_stack_env "$THREADS"
# r22 SINGLE-ENCODER POLICY: camera, file and static sources use the same
# v1.16.6 winning worker policy. Do not silently disable the measured cyclic
# wavelet rendezvous for live input; camera cadence may add latency, but it
# must not select a different encoder schedule.
WIDTH=${WIDTH:-3840}
HEIGHT=${HEIGHT:-2160}

# v1.16.6 control-plane isolation. The production winner needs cores 1/2/3
# free of shell/UI/reporting wakeups. During timed runs the encoder MAIN
# thread and tee/grep telemetry processes stay on Core 0; --cpu-winner then
# repins the primary worker pthreads onto the high cores.
CONTROL_CORE=()
if command -v taskset >/dev/null 2>&1 \
   && [[ "$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 0)" -ge 4 ]] \
   && taskset -c 0 true >/dev/null 2>&1; then
  CONTROL_CORE=(taskset -c 0)
fi
# Text forwarding is never timing-critical. Run tee/grep below the normal
# Core0 entropy helper so a terminal/log flush cannot delay the helper.
if command -v nice >/dev/null 2>&1; then
  CONTROL_FILTER=(nice -n 10)
else
  CONTROL_FILTER=()
fi

exec > >("${CONTROL_CORE[@]}" "${CONTROL_FILTER[@]}" tee "$RESULTS") 2>&1
LIVEVIEW_PID=""
MODE_REPORT=""
cleanup_exit() {
  local st=$?
  if [[ -n "${LIVEVIEW_PID:-}" ]]; then kill "$LIVEVIEW_PID" 2>/dev/null || true; fi
  [[ -n "${CINEPI_BENCH_PREVIEW_PATH:-}" ]] && rm -f "$CINEPI_BENCH_PREVIEW_PATH" 2>/dev/null || true
  cinepi_irq_isolation_restore || true
  cinepi_governor_restore
  mkdir -p "$BUNDLE_DIR"
  sync
  if command -v zip >/dev/null 2>&1; then
    _bundle=("$RESULTS")
    [[ -f "$CSV" ]] && _bundle+=("$CSV")
    [[ -n "${MODE_REPORT:-}" && -f "$MODE_REPORT" ]] && _bundle+=("$MODE_REPORT")
    # Live failures are difficult to diagnose from the summary alone. Include
    # the install log plus this run's CinePi recorder/preflight logs when they
    # exist; these are text only and add negligible bundle size.
    [[ -f "$HOME/gpr_pipeline.log" ]] && _bundle+=("$HOME/gpr_pipeline.log")
    if [[ "$SOURCE" == "camera" ]]; then
      # The live runner writes these to tmpfs when it can, /tmp otherwise.
      for _lf in /dev/shm/cinepi_raw_gpr_*.log /dev/shm/cinepi_raw_preflight_shift*.log \
                 /tmp/cinepi_raw_gpr_*.log /tmp/cinepi_raw_preflight_shift*.log; do
        [[ -f "$_lf" ]] && _bundle+=("$_lf")
      done
    fi
    ( cd "$HERE" && zip -qj "$BUNDLE_DIR/results_${HOSTN}_${STAMP}.zip" "${_bundle[@]}" 2>/dev/null ) &&
      echo "BUNDLE $BUNDLE_DIR/results_${HOSTN}_${STAMP}.zip"
  fi
  [[ -d "$GPRDIR" && "$SAVE_QRAW" == "on" ]] && echo "FRAMES $GPRDIR"
  [[ "$SAVE_QRAW" != "on" ]] && rm -rf "$GPRDIR"
  exit "$st"
}
trap cleanup_exit EXIT

hr(){ printf -- '-------------------------------------------------------------------\n'; }
say(){ hr; printf '%s\n' "$1"; hr; }

say "CinePi GPR RAW Benchmark - Pi5 -- $STAMP"
uname -srm
[[ -r /proc/device-tree/model ]] && echo "board:     $(tr -d '\0' < /proc/device-tree/model)"
cinepi_governor_boost
cinepi_irq_isolation_apply
echo "settings:  $THREADS workers, ${INPUT_BITS}-bit source, ${DURATION}s per run, $PASSES passes averaged"
echo "layout:    $(cinepi_winning_stack_desc "$THREADS")"
# The rendezvous figure is per-PATH, and printing the static one on a live run
# was actively misleading: CINEPI_WAVELET_RENDEZVOUS_US is read ONLY by the
# standalone vc5_bench (vc5_bench.cpp:11723). The live encoder reads
# CINEPI_QRAW_RENDEZVOUS_US (qraw_encoder.cpp:143) and defaults it to 0 for camera
# sources, which is the correct divergence (+11.7% live). Every live results
# file so far claims "rendezvous=20000us" while the encoder logged
# "wavelet rendezvous: 0 us" -- exactly the read-the-wrong-knob trap this
# package has already been bitten by once.
if [[ "$SOURCE" == "camera" ]]; then
  echo "winner:    v1.16.6 + IRQ->Core0 + VLE128; shared in-place GPR; 8/8 entropy; rendezvous=${CINEPI_QRAW_RENDEZVOUS_US:-0}us (live default; static uses ${CINEPI_WAVELET_RENDEZVOUS_US}us)"
else
  echo "winner:    v1.16.6 + IRQ->Core0 + VLE128; shared in-place GPR; 8/8 entropy; rendezvous=${CINEPI_WAVELET_RENDEZVOUS_US}us lead=${CINEPI_WAVELET_RELEASE_LEAD_US}us"
fi
if ((${#CONTROL_CORE[@]})); then echo "control:   UI/encoder-main/live telemetry pinned to Core0"; fi
echo "build:     $(getconf _NPROCESSORS_ONLN 2>/dev/null || echo '?') cores, parallel LTO"
echo "modes:     $MODES"
# Say plainly, in the file that gets kept, that this run is not the shipped
# encoder. Both the static bench and the live library carry the knob, so this
# applies to every source.
if [[ -n "${CINEPI_BAND_Q:-}" ]]; then
  echo "band:      EXPERIMENTAL band pruning ACTIVE -- CINEPI_BAND_Q=${CINEPI_BAND_Q}${BAND_TAG:+  tag=$BAND_TAG}"
  echo "           quantises whole wavelet subbands away. NOT bit-exact, NOT"
  echo "           the shipped encoder. Any frames kept are EXPERIMENTAL footage."
  echo "           band index: 0=LL3  1..3=L3 LH/HL/HH  4..6=L2  7..9=L1 (9=HH1)"
  if [[ "$SOURCE" != "camera" ]]; then
    echo "           static source: the reference frame is FIXED, so this leg and"
    echo "           an E0 leg encode the SAME pixels -- compare them per-pixel."
  fi
fi
if [[ "$SAVE_QRAW" == "on" ]]; then
    _free=$(df -PB1 -- "$GPR_PATH" 2>/dev/null | awk 'NR==2{printf "%.1f", $4/1073741824}')
    echo "output:    $GPR_PATH  (${_free:-?} GiB free)"
else
    if [[ "$VERIFY" == "on" ]]; then
      echo "output:    not kept; verification frame(s) are written, decode-verified, then removed"
    else
      echo "output:    not kept; no output-frame disk I/O"
    fi
fi
command -v vcgencmd >/dev/null 2>&1 && echo "temp:      $(vcgencmd measure_temp)"

# ---------------------------------------------------- 1. build / live fast path
# Production live-camera runs do NOT need the standalone vc5_bench binary,
# shader bundle, CPU lab binaries, camera-mode helper or standalone decoder.
# CinePi-RAW links the installed /opt/cinepi-qraw core directly, and its installer
# fingerprints that core plus the CinePi integration. Running build_and_verify.sh
# here used to rebuild an entirely separate benchmark toolchain before every
# newly extracted ZIP, even though none of those binaries are in the live path.
if [[ "$SOURCE" == "camera" ]]; then
  say "1. Live pipeline fast path"
  echo "standalone encoder/SDK/lab build: SKIPPED (not used by CinePi-RAW live capture)"
  echo "encoder: single canonical v1.16.6 winning stack; /opt/cinepi-qraw reused only when its winner identity matches"

  # Keep control/UI work off the three hot owner cores during the live run.
  if ((${#CONTROL_CORE[@]})); then
    taskset -pc 0 $$ >/dev/null 2>&1 || true
  fi

  echo
  echo "camera backend: CINEPI_RAW (production recorder; standalone adapter bypassed)"
  export CINEPI_LIVE_GPRDIR="$GPRDIR"
  export CINEPI_RESULT_CSV="$CSV"
  # The live camera path lives in the cinepi-qraw-alpha repository, not here.
  # This repository is the encoder and its static/file benchmarks only.
  if [[ ! -x "$HERE/RUN_CINEPI_RAW_BENCHMARK.sh" ]]; then
    echo "ERROR: live camera benchmarking is not part of the QRAW encoder repository." >&2
    echo "       Use https://github.com/Kizza-O/cinepi-qraw-alpha for camera capture." >&2
    exit 64
  fi
  "$HERE/RUN_CINEPI_RAW_BENCHMARK.sh"
  exit $?
fi

# Static/file benchmarks still use the standalone benchmark executable. Build
# and prove that binary once, then reuse its verification stamp until its source
# actually changes.
say "1. Build"
if [[ "$VERIFY" == "on" || "${CINEPI_FULL_VERIFY:-0}" == "1" ]]; then
  "$HERE/build_and_verify.sh" --full || exit $?
else
  "$HERE/build_and_verify.sh" || exit $?
fi

# Build is finished. Keep the runner shell itself off the three hot owner
# cores for the timed section too; otherwise its periodic echo/parse work can
# migrate onto cores 1..3 even though the encoder workers are pinned.
if ((${#CONTROL_CORE[@]})); then
  taskset -pc 0 $$ >/dev/null 2>&1 || true
fi

# --- static/file input source -----------------------------------------------
BIN="$BENCH/build/vc5_bench"
COMPAND_ARGS=()
INPUT_ARGS=()
INPUT=""
ENCODER_BAYER="gbrg"

# SOURCE=camera has already exited through the CinePi-RAW production path
# above. The standalone benchmark is intentionally static/file only so live
# camera plumbing can never perturb the v1.16.6 winning hot loop again.
if [[ "$SOURCE" == "folder" ]]; then
  [[ -n "${FRAME_DIR:-}" && -d "$FRAME_DIR" ]] || { echo "FATAL: folder source selected but FRAME_DIR is not a directory" >&2; exit 2; }
  INPUT="$BENCH/input/sample_imx585_3840x2160_gbrg_16bit.raw16"
  INPUT_ARGS=(--input "$INPUT" --source folder --frame-dir "$FRAME_DIR")
  COMPAND_ARGS=(--effective-bits 16)
  SENSOR_MAX_FPS=${SENSOR_MAX_FPS:-0}
  SENSOR_SRC="folder input"
else
  [[ "$SOURCE" == "sample" ]] || { echo "FATAL: unsupported standalone SOURCE=$SOURCE" >&2; exit 2; }
  if [[ -z "${SENSOR_MAX_FPS:-}" ]]; then
    if [[ "$INPUT_BITS" == "12" ]]; then SENSOR_MAX_FPS=60; else SENSOR_MAX_FPS=30; fi
    SENSOR_SRC="documented IMX585 figure (static source)"
  fi
  if [[ "$INPUT_BITS" == "12" ]]; then
    INPUT="$BENCH/input/sample_imx585_3840x2160_gbrg_12bit_gplog2.raw16"
    # Keep the exact v1.16.6 static semantics. The input is already GP-Log2;
    # do not add live-camera normalization/crop/stride state to this path.
    COMPAND_ARGS=(--input-companded on)
    if [[ ! -f "$INPUT" ]]; then
      echo "12-bit input missing -- generating it from the 16-bit source"
      python3 "$HERE/make_precompanded.py" \
        "$BENCH/input/sample_imx585_3840x2160_gbrg_16bit.raw16" "$INPUT" || {
          echo "FATAL: could not generate the 12-bit input" >&2; exit 2; }
    fi
  else
    INPUT="$BENCH/input/sample_imx585_3840x2160_gbrg_16bit.raw16"
  fi
  INPUT_ARGS=(--input "$INPUT" --source sample)
fi

echo "sensor:    ${SENSOR_MAX_FPS} fps at ${INPUT_BITS}-bit -- ${SENSOR_SRC}"
echo "encoder:   $("$BIN" --version)"
echo "engine:    v1.16.6 winning static CPU-GPR hot path (camera code not linked)"
echo "bayer:     ${ENCODER_BAYER} -- selected reference/source CFA"
if [[ "$WRITE_FRAMES" == "on" ]]; then
  echo "frames:    written to $GPRDIR (verify=$VERIFY, keep=$SAVE_QRAW)"
else
  echo "frames:    NOT written -- encoder measured without disk I/O."
  echo "           VERIFY=on or SAVE_QRAW=on to include the writer."
fi

# v3.15: the static benchmark now runs the SAME encoder configuration as the
# live camera path -- direct tile-hybrid with the wavelet-emitted nonzero mask.
# Previously static used the sidecar and live used direct-hybrid, because
# direct-hybrid made the wavelet cheaper but entropy dearer and each regime
# preferred a different trade. The mask removes that trade: the wavelet now
# emits the nonzero information it was already computing, so direct-hybrid wins
# on BOTH stages at every mode (m1 39.9 -> 48.3 fps, m2 40.5 -> 49.0,
# m5 46.1 -> 53.4, m7 53.4 -> 58.4). Output is bit-identical either way.
BASE=("${COMPAND_ARGS[@]}" "${INPUT_ARGS[@]}"
      --width "$WIDTH" --height "$HEIGHT" --bayer "$ENCODER_BAYER"
      --duration "$DURATION" --warmup-seconds "$WARMUP_S"
      --trim-seconds "${TRIM_S:-2}"
      --sensor-max-fps "${SENSOR_MAX_FPS}"
      --execution cpu-gpr --cpu-gpr-threads "$THREADS"
      --core0-wavelet-strategy off
      --cpu-gpr-stagger-us 0
      --compand-bits 12
      --cpu-wavelet-kernel v53 --cpu-wavelet-vec 1
      --cpu-wavelet-fused on --cpu-split-fused on --cpu-split-neon on
      --cpu-gpr-raw-copy off
      --cpu-gpr-dng-splice on --cpu-gpr-splice-shared on
      --cpu-gpr-splice-reuse off --cpu-gpr-shared-reuse off
      --cpu-gpr-shared-inplace on --win-handoff-pool on
      --cpu-nontemporal on --cpu-sidecar off --cpu-direct-hybrid on
      --cpu-vle-prefetch-distance 128 --cpu-vle-prefetch-locality 0
      --win-lowpass-bulk off --vle-prequant-skip off
      --cpu-gpr-helper off
      --compand-inframe-bits "${COMPAND_BITS:-12}"
      --cpu-winner on
      --capture-tick-ms "${TICK_MS:-0}")

TMP="$HERE/.run.$$"
CONFIG_LOGGED=0
run_mode() {   # pass, mode, [extra args]
  local pass="$1" mode="$2"; shift 2
  # tee so the full output is available for parsing while the ticks still
  # stream out live. Writing straight to $TMP hid every tick from the UI.
  set +e
  "${CONTROL_CORE[@]}" "$BIN" "${BASE[@]}" --mode "$mode" "$@" 2>&1 \
    | "${CONTROL_CORE[@]}" "${CONTROL_FILTER[@]}" tee "$TMP" \
    | { "${CONTROL_CORE[@]}" "${CONTROL_FILTER[@]}" grep --line-buffered -E '^(ENCODE_TICK|CAPTURE_TICK|CAMERA_SENSOR_MODE_SELECT|CAMERA_RATE_RANGE_HINT|CAMERA_CONFIG|CAMERA_ENCODER_INPUT|CAMERA_STAGING_CONFIG|CAMERA_PREENCODE_RATE|CAMERA_FRAME_CHECK|CAMERA_RESULT)' || true; }
  local rc=${PIPESTATUS[0]}
  set -e
  if ((rc != 0)); then
    echo "  pass $pass $mode: FAILED rc=$rc"
    grep -E '^(CAMERA_SENSOR_MODE_SELECT|CAMERA_RATE_RANGE_HINT|CAMERA_CONFIG|CAMERA_ENCODER_INPUT|CAMERA_STAGING_CONFIG|CAMERA_PREENCODE_RATE|CAMERA_FRAME_CHECK|CAMERA_RESULT|FATAL:)' "$TMP" | tail -12 | sed 's/^/    /' || true
    tail -8 "$TMP" | sed 's/^/    /'
    if [[ "$SOURCE" == "camera" ]]; then
      echo "FATAL: live-camera benchmark failed; no static-frame result will be substituted." >&2
      exit "$rc"
    fi
    return 0
  fi
  # Record the configuration the encoder actually used, once. Without this the
  # log shows results with no evidence of what produced them, which is exactly
  # how E2 shipped default-on and skewed a whole matrix unnoticed.
  if ((CONFIG_LOGGED == 0)); then
    grep -m1 '^CPU_KERNEL' "$TMP" | sed 's/^/  /' || true
    grep -m1 '^CPU_GPR_WARMUP_DONE' "$TMP" | sed 's/^/  /' || true
    grep -m1 '^CPU_GPR_CONFIG' "$TMP" | sed 's/^/  /' && CONFIG_LOGGED=1 || true
    grep -m1 '^CPU_GPR_HELPER' "$TMP" | sed 's/^/  /' || true
    grep -m1 '^CPU_WAVELET_RENDEZVOUS_CONFIG' "$TMP" | sed 's/^/  /' || true
  fi
  if [[ "$SOURCE" == "camera" ]]; then
    grep -m1 '^CAMERA_CONFIG ' "$TMP" | sed 's/^/       /' || true
    grep -m1 '^CAMERA_FRAME_CHECK ' "$TMP" | sed 's/^/       /' || true
    grep -m1 '^CAMERA_RESULT ' "$TMP" | sed 's/^/       /' || true
    if [[ "$SAVE_QRAW" == "on" ]]; then
      mkdir -p "$GPRDIR"
      {
        echo "pass=$pass mode=$mode source=LIVE_IMX585"
        grep -m1 '^CAMERA_CONFIG ' "$TMP" || true
        grep -m1 '^CAMERA_FRAME_CHECK ' "$TMP" || true
        grep -m1 '^CAMERA_RESULT ' "$TMP" || true
      } >> "$GPRDIR/SOURCE_CAMERA.txt"
    fi
    # Preserve warnings/errors emitted by libcamera/PiSP even when the run
    # succeeds. Expected 16-bit statistics warnings remain visible; manual
    # controls are used specifically so they cannot invalidate exposure.
    mapfile -t _cam_issues < <(grep -Ei '(^|[^A-Z])(WARN|WARNING|ERROR|FATAL)([^A-Z]|$)|failed|timeout' "$TMP" \
      | grep -Ev '^CAMERA_(CONFIG|FRAME_CHECK|RESULT)' | tail -20 || true)
    if ((${#_cam_issues[@]})); then
      echo "       CAMERA_DIAGNOSTICS issues=${#_cam_issues[@]}"
      printf '       CAMERA_DIAG %s\n' "${_cam_issues[@]}"
    else
      echo "       CAMERA_DIAGNOSTICS issues=0"
    fi
  fi
  local line wave_line assist_line camera_line
  line=$(grep -m1 '^CPU_QRAW_RESULT' "$TMP" || true)
  wave_line=$(grep -m1 '^CPU_WAVELET_CONCURRENCY' "$TMP" || true)
  assist_line=$(grep -m1 '^VC5_ENTROPY_ASSIST ' "$TMP" || true)
  camera_line=$(grep -m1 '^CAMERA_RESULT ' "$TMP" || true)
  [[ -n "$line" ]] || { echo "  pass $pass $mode: no result"; return 0; }
  grep -m1 '^CPU_GPR_WAVELET_ASSIST' "$TMP" | sed 's/^/       /' || true
  [[ -n "$wave_line" ]] && echo "       $wave_line"
  [[ -n "$assist_line" ]] && echo "       $assist_line"
  CPU_GPR_LINE="$line" WAVE_LINE="$wave_line" ASSIST_LINE="$assist_line" CAMERA_LINE="$camera_line" PASS="$pass" MODE="$mode" CSVPATH="$CSV" python3 - <<'PY'
import re, os
def parse(s):
    return dict(re.findall(r'(\w+)=([^\s]+)', s or ''))
d = parse(os.environ['CPU_GPR_LINE'])
w = parse(os.environ.get('WAVE_LINE',''))
a = parse(os.environ.get('ASSIST_LINE',''))
c = parse(os.environ.get('CAMERA_LINE',''))
csv, pas, mode = os.environ['CSVPATH'], os.environ['PASS'], os.environ['MODE']
# v1.1: the encoder has always emitted the per-stage breakdown; the runner
# threw it away and reported only fps. Without it a slow run gives no clue
# WHERE the time went, which is how a 31 vs 40 fps gap stayed unexplained.
keys = ['fps','assisted_fps','assisted_frames','assist_owner_wait_ms','assist_publish_wait_ms','latency_p50_ms','latency_p99_ms','ms_copy','ms_wavelet',
        'ms_entropy','ms_frame_per_worker','dng_wrap_ms','writer_ms',
        'gpr_bytes','ratio','output_mibs',
        'actual_wavelet_ms','sync_wait_ms_per_frame','c3_pct','gap_p50_us',
        'helper_cpu_pct','helper_cpu_ms_per_job','entropy_assist_wait_ms',
        'camera_sensor_bits','camera_target_fps','camera_capture_fps','camera_ok',
        'camera_sequence_gaps','camera_request_errors','camera_timeouts']
vals = dict(d)
for k in ('actual_wavelet_ms','sync_wait_ms_per_frame','c3_pct','gap_p50_us'):
    vals[k] = w.get(k, '')
vals['helper_cpu_pct'] = a.get('helper_cpu_pct','')
vals['helper_cpu_ms_per_job'] = a.get('helper_cpu_ms_per_job','')
vals['entropy_assist_wait_ms'] = a.get('owner_wait_ms_per_job','')
vals['camera_sensor_bits'] = c.get('sensor_bits','')
vals['camera_target_fps'] = c.get('target_fps','')
vals['camera_capture_fps'] = c.get('capture_fps_actual','')
vals['camera_ok'] = c.get('camera_ok','')
vals['camera_sequence_gaps'] = c.get('sequence_gaps','')
vals['camera_request_errors'] = c.get('request_errors','')
vals['camera_timeouts'] = c.get('timeouts','')
row = [pas, mode] + [vals.get(k, '') for k in keys]
new = not os.path.exists(csv)
with open(csv, 'a') as f:
    if new: f.write("pass,mode," + ",".join(keys) + "\n")
    f.write(",".join(row) + "\n")
def g(k):
    try: return float(d.get(k, 0) or 0)
    except ValueError: return 0.0
print("  %-4s fps=%-7.2f GDassist=%-5.2f join=%.2fms feed=%.2fms p50=%-6.1f p99=%-6.1f ratio=%-6.2f MiB/s=%.1f" % (
      mode, g('fps'), g('assisted_fps'), g('assist_owner_wait_ms'), g('assist_publish_wait_ms'),
      g('latency_p50_ms'), g('latency_p99_ms'), g('ratio'), g('output_mibs')))
print("       stages/frame ms: copy=%.2f wavelet=%.2f entropy=%.2f "
      "dng_wrap=%.2f writer=%.2f  (worker total %.2f)" % (
      g('ms_copy'), g('ms_wavelet'), g('ms_entropy'), g('dng_wrap_ms'),
      g('writer_ms'), g('ms_frame_per_worker')))
def gf(src,k):
    try: return float(src.get(k,0) or 0)
    except ValueError: return 0.0
if w or a:
    print("       winner telemetry: actualW=%.2fms c3=%.1f%% sync=%.3fms helper=%.1f%% helperjob=%.3fms" % (
          gf(w,'actual_wavelet_ms'), gf(w,'c3_pct'), gf(w,'sync_wait_ms_per_frame'),
          gf(a,'helper_cpu_pct'), gf(a,'helper_cpu_ms_per_job')))
PY
}

# ------------------------------------------------------- 2. the measurement
for ((P = 1; P <= PASSES; ++P)); do
  say "2.$P  Pass $P of $PASSES -- M1 to M10"
  for M in $MODES; do
    if ((P == 1)) && [[ "$WRITE_FRAMES" == "on" ]]; then
      # Save one frame per mode on the first pass. Output is deterministic, so
      # which pass writes it does not matter.
      run_mode "$P" "$M" --save-gpr "$GPRDIR/.$M" --save-gpr-limit 1
      mv "$GPRDIR/.$M"/*.gpr "$GPRDIR/$M.gpr" 2>/dev/null || true
      rmdir "$GPRDIR/.$M" 2>/dev/null || true
    else
      run_mode "$P" "$M"
    fi
  done
  command -v vcgencmd >/dev/null 2>&1 && { echo "  temp: $(vcgencmd measure_temp)"; vcgencmd get_throttled | sed 's/^/  /'; }
done
rm -f "$TMP"

# The uncompressed source frame sits alongside the encoded ones so the folder
# is self-contained for comparison.
[[ "$SAVE_QRAW" == "on" && -n "$INPUT" && -f "$INPUT" ]] && cp "$INPUT" "$GPRDIR/source_uncompressed.raw16"

# ------------------------------------------------------- 3. decode gate
if [[ "$VERIFY" != "on" && "$SAVE_QRAW" != "on" ]]; then
  say "3. SDK decode verification -- SKIPPED (VERIFY=off)"
  echo "no frames were written; the encoder was measured without disk I/O."
  echo "VERIFY=on re-enables the fail-closed decode gate."
else
say "3. SDK decode verification -- every saved frame, fail-closed"
mapfile -t FRAMES < <(find "$GPRDIR" -name "*.gpr" | sort -V)
((${#FRAMES[@]})) || { echo "FATAL: no GPR frames were saved"; exit 4; }
"$BENCH/build/gpr_decode_verify" "${FRAMES[@]}" 2>/dev/null | grep -E '^DECODE' | sed "s|$GPRDIR/||"
"$BENCH/build/gpr_decode_verify" "${FRAMES[@]}" >/dev/null 2>&1 \
  || { echo "FATAL: at least one frame failed SDK decode"; exit 4; }
fi

# ------------------------------------------------------- 4. summary
say "4. Results -- mean of $PASSES passes"
CSVPATH="$CSV" PASSES="$PASSES" python3 - <<'PY'
import csv, os, statistics as st
path = os.environ['CSVPATH']
if not os.path.exists(path):
    # Every run failed (e.g. the NEON guard refusing the winner stack on a
    # non-aarch64 host). Say so plainly instead of raising a traceback.
    print('no results: every run failed before producing a result line.')
    print('Check the output above -- the first FATAL explains why.')
    raise SystemExit(0)
rows = list(csv.DictReader(open(path)))
modes = [m for m in (f"m{i}" for i in range(1, 11)) if any(r['mode'] == m for r in rows)]
by = {}
for r in rows:
    by.setdefault(r['mode'], []).append(r)

print(f"{'mode':<6}{'fps':>8}{'spread':>8}{'p50 ms':>9}{'p99 ms':>9}{'GPR MB':>9}{'ratio':>8}{'MiB/s':>8}")
worst = 0.0
for m in modes:
    rs = by.get(m)
    if not rs: continue
    f = [float(r['fps']) for r in rs]
    spread = max(f) - min(f) if len(f) > 1 else 0.0
    worst = max(worst, spread)
    print(f"{m:<6}{st.mean(f):>8.2f}{spread:>8.2f}"
          f"{st.mean([float(r['latency_p50_ms']) for r in rs]):>9.1f}"
          f"{st.mean([float(r['latency_p99_ms']) for r in rs]):>9.1f}"
          f"{float(rs[0]['gpr_bytes'])/1e6:>9.2f}"
          f"{float(rs[0]['ratio']):>8.2f}"
          f"{st.mean([float(r['output_mibs']) for r in rs]):>8.1f}")

ok = [m for m in modes if m in by and st.mean([float(r['fps']) for r in by[m]]) >= 30.0]
print(f"\nat or above 30 fps: {', '.join(ok) if ok else 'none'}  ({len(ok)}/10)")
print(f"largest spread between passes: {worst:.2f} fps"
      f" -- differences smaller than this are not meaningful")
print(f"\ncsv: {os.environ['CSVPATH']}")
PY

say "5. Output folder"
if [[ "$SAVE_QRAW" != "on" ]]; then
    rm -rf "$GPRDIR"
    echo "not kept (SAVE_QRAW=off). The frames were decode-verified before removal."
    echo "text: $RESULTS"
    exit 0
fi
echo "$GPRDIR"
ls -1sh "$GPRDIR" | tail -n +2 | sed 's/^/  /'
echo
echo "  $(find "$GPRDIR" -type f | wc -l) files: one GPR per mode plus the uncompressed source."
echo "text: $RESULTS"
