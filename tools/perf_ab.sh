#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Kieran Olsson
# Interleaved A/B of encoder variants on FIXED input.
#
# Variants are compared by their BEST rep (contention only ever slows a rep),
# and reps are INTERLEAVED across variants so slow drift hits every variant
# equally instead of penalising whichever ran last.
#
#   tools/perf_ab.sh <reps> <mode> <threads> <seconds> "<label>=<flags>" ...
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
REPS=$1 MODE=$2 THREADS=$3 WIN=$4; shift 4
for p in cinepi-raw cinepi-gui; do pgrep -x "$p" >/dev/null && { echo "FATAL: $p running"; exit 3; }; done
cd "$BENCH"
declare -A BEST_FPS BEST_WAV BEST_ENT BEST_FRM BYTES
for spec in "$@"; do l="${spec%%=*}"; BEST_FPS[$l]=0; BEST_WAV[$l]=0; BEST_ENT[$l]=0; BEST_FRM[$l]=0; BYTES[$l]="-"; done
for r in $(seq 1 "$REPS"); do
  for spec in "$@"; do
    lbl="${spec%%=*}"; flags="${spec#*=}"
    out=$($BIN --execution cpu-gpr --mode "$MODE" --seconds "$WIN" --threads "$THREADS" \
             $flags --input "$IN" 2>/dev/null || true)
    line=$(grep -E "^CPU_QRAW_RESULT" <<<"$out" | sed -n '1p')
    [[ -n "$line" ]] || { echo "  $lbl rep$r: NO RESULT"; continue; }
    g(){ sed -n "s/.*[[:space:]]$1=\([^[:space:]]*\).*/\1/p" <<<"$line"; }
    f=$(g fps)
    if awk "BEGIN{exit !($f > ${BEST_FPS[$lbl]})}"; then
      BEST_FPS[$lbl]=$f; BEST_WAV[$lbl]=$(g ms_wavelet); BEST_ENT[$lbl]=$(g ms_entropy); BEST_FRM[$lbl]=$(g ms_frame_per_worker)
    fi
    BYTES[$lbl]=$(g gpr_bytes)
    printf "  r%-2s %-22s fps %7.2f  wav %6.2f  ent %6.2f  frame %6.2f  bytes %s\n" \
      "$r" "$lbl" "$f" "$(g ms_wavelet)" "$(g ms_entropy)" "$(g ms_frame_per_worker)" "$(g gpr_bytes)"
  done
done
echo
echo "BEST OF $REPS  (mode=$MODE threads=$THREADS window=${WIN}s)"
for spec in "$@"; do l="${spec%%=*}"
  printf "  %-22s fps %7.2f   wavelet %6.2f   entropy %6.2f   frame %6.2f   gpr_bytes %s\n" \
    "$l" "${BEST_FPS[$l]}" "${BEST_WAV[$l]}" "${BEST_ENT[$l]}" "${BEST_FRM[$l]}" "${BYTES[$l]}"
done
echo "  (gpr_bytes MUST be identical across variants -- a difference means the output changed)"
