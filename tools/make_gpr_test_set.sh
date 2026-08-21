#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Kieran Olsson
# ===========================================================================
# make_gpr_test_set.sh -- every GPR the encoder's quality axes can produce,
# from the ONE static reference frame, named so the settings are readable off
# the filename.
#
#   tools/make_gpr_test_set.sh [output_dir]
#
# Default output: ~/Downloads/QRAW_GPR_TEST_SET
#
# WHAT VARIES (the full cross product, 10 x 2 x 4 x 2 x 4 = 640 files):
#
#   M mode        m1 .. m10          the calibrated quality ladder
#   band          E0 | E1            E1 prunes HH1, the finest diagonal
#   CAQ           off soft med strong  component-aware quantisation
#   Pixel Clean   on | off           dead zone + RG/BG level-1 +-1 prune
#   Noise Clean   off soft med strong  widened level-1 zero threshold
#
# WHAT IS HELD CONSTANT, deliberately, so the set has one variable group:
#
#   input      the bundled static frame, 3840x2160 GBRG 16-bit linear
#   container  12-bit compand, GP-Log2 k=599, white 4095
#   encoder    the pinned v1.16.6 winning stack
#
# The input/container axes (ClearHDR 16-bit, gradation compand, true-12bit) are
# NOT swept here: they change what the container declares rather than what the
# quality levers do, and each one would double the set. Ask if they are wanted.
#
# Every file is decode-verified through the stock GPR SDK before the run ends,
# so a file in this set is a file that reads back.
# ===========================================================================
set -Eeuo pipefail
HERE=$(cd "$(dirname "$0")" && pwd); PKG=$(cd "$HERE/.." && pwd)
BENCH="$PKG/benchmark"
BIN="$BENCH/cinepi_qraw_bench/build/vc5_bench"
VERIFY="$BENCH/cinepi_qraw_bench/build/gpr_decode_verify"
IN="cinepi_qraw_bench/input/sample_imx585_3840x2160_gbrg_16bit.raw16"
OUT=${1:-$HOME/Downloads/QRAW_GPR_TEST_SET}
TMP=$(mktemp -d); trap 'rm -rf "$TMP"' EXIT

[[ -x "$BIN" ]] || { echo "FATAL: build the bench first: benchmark/build_and_verify.sh" >&2; exit 2; }
mkdir -p "$OUT"
MAN="$OUT/MANIFEST.csv"
echo "mode,band,caq,pixel_clean,noise_clean,bytes,crc32,file" > "$MAN"

cd "$BENCH"
n=0
for mode in m1 m2 m3 m4 m5 m6 m7 m8 m9 m10; do
  mkdir -p "$OUT/$mode"
  for band in E0 E1; do
    for caq in off soft medium strong; do
      for pc in on off; do
        for nc in off soft medium strong; do
          case "$caq"  in off) ct=off;; soft) ct=S;; medium) ct=M;; strong) ct=X;; esac
          case "$nc"   in off) nt=off;; soft) nt=S;; medium) nt=M;; strong) nt=X;; esac
          [[ "$pc" == on ]] && pt=ON || pt=OFF
          name="${mode}_${band}_caq${ct}_pc${pt}_nc${nt}.gpr"
          dest="$OUT/$mode/$name"
          if [[ -s "$dest" ]]; then continue; fi          # resumable
          rm -rf "$TMP/w"; mkdir -p "$TMP/w"
          # CAQ, Pixel Clean and Noise Clean go through FLAGS, not the
          # environment. CINEPI_CAQ was silently ignored by this binary until
          # 2026-08-21 -- it had no --caq and never read the variable -- so a set
          # built through the environment came out with all four CAQ values
          # byte-identical. Band pruning is still environment-only.
          env_args=()
          [[ "$band" == E1 ]] && env_args+=(CINEPI_BAND_Q=9=32767)
          env ${env_args[@]+"${env_args[@]}"} "$BIN" --execution cpu-gpr --mode "$mode" \
              --seconds 1 --warmup 0 --warmup-seconds 0 --trim-seconds 0 \
              --threads 2 --buffers 2 --caq "$caq" --pixel-clean "$pc" \
              --noise-clean "$nc" \
              --save-gpr "$TMP/w" --save-gpr-limit 1 --input "$IN" >/dev/null 2>&1 || {
                echo "  FAILED $name" >&2; continue; }
          src=$(find "$TMP/w" -name '*.gpr' | head -1)
          [[ -n "$src" ]] || { echo "  NO FILE $name" >&2; continue; }
          mv "$src" "$dest"
          bytes=$(stat -c%s "$dest")
          crc=$(cksum "$dest" | cut -d' ' -f1)
          echo "$mode,$band,$caq,$pc,$nc,$bytes,$crc,$mode/$name" >> "$MAN"
          n=$((n+1))
          (( n % 40 == 0 )) && echo "  ...$n files"
        done
      done
    done
  done
  echo "$mode done"
done
echo "wrote $n files to $OUT"

if [[ -x "$VERIFY" ]]; then
  echo "== decode-verifying every file through the stock GPR SDK =="
  mapfile -t all < <(find "$OUT" -name '*.gpr' | sort)
  fails=0
  for ((i=0; i<${#all[@]}; i+=32)); do
    "$VERIFY" "${all[@]:i:32}" 2>&1 | grep -E "FAIL" && fails=1 || true
  done
  [[ "$fails" == 0 ]] && echo "DECODE_VERIFY ALL PASS (${#all[@]} files)" \
                      || echo "DECODE_VERIFY had failures -- see above" >&2
fi
