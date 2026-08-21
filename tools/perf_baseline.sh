#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Kieran Olsson
# Scene-free encoder throughput measurement.
#
# WHY THIS EXISTS: the live camera number is scene-dependent -- the same build
# measured 49.2 fps on one scene and 40.1 on another purely because compressed
# frame size changed (ratio 7.3 vs 4.9). Live fps therefore cannot judge a 1 ms
# change. This runs the encoder over a FIXED input frame, so the only variable
# is the code.
#
# WHY MINIMA: this machine runs a desktop and a browser, so cores and memory
# bandwidth are shared. Contention can only ever make a run SLOWER, never
# faster, so the MINIMUM stage time across N reps is the robust estimator of
# what the code can do. Means are dominated by whatever else woke up. Measured
# noise: entropy time is stable to ~4%, wavelet time swings ~28% because it is
# bandwidth-bound and competes with the desktop.
#
#   tools/perf_baseline.sh [reps] [mode] [threads] [seconds] [label]
set -Eeuo pipefail
# NOTE on pipelines below: this script uses `set -o pipefail`, and a consumer
# that exits early (grep -q, head -1) can SIGPIPE its producer, making the
# pipeline report failure even when it matched. Producers are captured into a
# variable first, so there is no early-exit consumer in a pipeline whose status
# matters. The patch script had a measured 3-in-10 false failure from exactly
# this.

HERE=$(cd "$(dirname "$0")" && pwd); PKG=$(cd "$HERE/.." && pwd)
BENCH="$PKG/benchmark"; BIN="$BENCH/cinepi_qraw_bench/build/vc5_bench"
IN="cinepi_qraw_bench/input/sample_imx585_3840x2160_gbrg_16bit.raw16"
REPS=${1:-7} MODE=${2:-m7} THREADS=${3:-3} WIN=${4:-8} LABEL=${5:-baseline}
# Match the SHIPPED configuration, not the bench defaults. vc5_bench.cpp:275
# defaults cpu_direct_hybrid to false, but the library sets it true
# (cinepi_qraw_encoder.cpp:555), and it forces the sidecar off when it does. The
# split differs sharply -- ~33.9/24.0 wavelet/entropy in production against
# ~42.4/25.5 with the hybrid off -- so measuring the defaults would tune against
# a configuration the camera never runs.
PROD_FLAGS=${PROD_FLAGS:---cpu-direct-hybrid on --cpu-sidecar off}
[[ -x "$BIN" ]] || { echo "FATAL: $BIN not built"; exit 2; }
for p in cinepi-raw cinepi-gui; do
    pgrep -x "$p" >/dev/null && { echo "FATAL: $p is running -- kill it before measuring"; exit 3; }
done
echo "# $LABEL   mode=$MODE threads=$THREADS window=${WIN}s reps=$REPS   $(date '+%F %T')"
echo "# governor=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor) clock=$(vcgencmd measure_clock arm | cut -d= -f2) throttled=$(vcgencmd get_throttled | cut -d= -f2)"
printf "# %-4s %8s %9s %9s %8s %9s %9s\n" rep fps wavelet entropy copy frame_ms lat_p50
cd "$BENCH"
: > /tmp/pb.$$
for i in $(seq 1 "$REPS"); do
    out=$("$BIN" --execution cpu-gpr --mode "$MODE" --seconds "$WIN" --threads "$THREADS" \
            $PROD_FLAGS --input "$IN" 2>/dev/null || true)
    line=$(grep -E "^CPU_QRAW_RESULT" <<<"$out" | sed -n '1p')
    [[ -n "$line" ]] || { echo "  rep $i: NO RESULT"; continue; }
    g(){ sed -n "s/.*[[:space:]]$1=\([^[:space:]]*\).*/\1/p" <<<"$line"; }
    printf "%s %s %s %s %s %s\n" "$(g fps)" "$(g ms_wavelet)" "$(g ms_entropy)" "$(g ms_copy)" "$(g ms_frame_per_worker)" "$(g latency_p50_ms)" >> /tmp/pb.$$
    printf "  %-4s %8.2f %9.2f %9.2f %8.2f %9.2f %9.2f\n" "$i" $(tail -1 /tmp/pb.$$)
done
awk '{ if(NR==1){for(j=1;j<=6;j++){mn[j]=$j; mx[j]=$j}} 
       for(j=1;j<=6;j++){ if($j<mn[j])mn[j]=$j; if($j>mx[j])mx[j]=$j } }
     END{ printf "# BEST   fps %.2f | wavelet %.2f | entropy %.2f | copy %.2f | frame %.2f | p50 %.2f\n", mx[1],mn[2],mn[3],mn[4],mn[5],mn[6]
          printf "# WORST  fps %.2f | wavelet %.2f | entropy %.2f | copy %.2f | frame %.2f | p50 %.2f\n", mn[1],mx[2],mx[3],mx[4],mx[5],mx[6]
          printf "# COMPARE ON THE BEST ROW. Contention only ever makes a rep slower.\n" }' /tmp/pb.$$
rm -f /tmp/pb.$$
