#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Kieran Olsson
# A/B: hp_flat_skip disabled (control) vs enabled (skip).
#
# Both binaries come from the SAME cmake tree with the SAME flags, differing
# only in the two call sites. They are compared against each other and never
# against the pinned binary, because a rebuild moves code layout and this stage
# swings on layout alone (docs/BENCHMARK_CONCLUSION.md, failure mode 2).
#
# Runs are INTERLEAVED control,skip,control,skip so thermal drift hits both
# legs equally rather than accumulating in whichever ran second.
set -uo pipefail
cd "$(dirname "$0")/.."
Q=quant_matrix
IN=cinepi_qraw_bench/input/sample_imx585_3840x2160_gbrg_12bit_gplog2.raw16
DUR=${DUR:-5}
REPS=${REPS:-3}
THREADS=${THREADS:-3}

run() {  # $1=binary $2=mode -> fps
  "$1" --input "$IN" --input-companded on --width 3840 --height 2160 \
       --duration "$DUR" --mode "$2" --execution cpu-gpr \
       --cpu-gpr-threads "$THREADS" 2>/dev/null \
    | grep -oE 'CPU_QRAW_RESULT .*\bfps=[0-9.]+' | grep -oE 'fps=[0-9.]+$' \
    | cut -d= -f2 | tail -1
}

echo "=== bit-exactness gate (output MUST be identical) ==="
for m in m1 m5 m7 m10; do
  a=$(mktemp -d -p /dev/shm); b=$(mktemp -d -p /dev/shm)
  "$Q/vc5_bench.control" --input "$IN" --input-companded on --width 3840 --height 2160 \
      --duration 0 --frames 1 --warmup 1 --mode "$m" --execution cpu-gpr \
      --cpu-gpr-threads 1 --save-gpr "$a" >/dev/null 2>&1
  "$Q/vc5_bench.skip"    --input "$IN" --input-companded on --width 3840 --height 2160 \
      --duration 0 --frames 1 --warmup 1 --mode "$m" --execution cpu-gpr \
      --cpu-gpr-threads 1 --save-gpr "$b" >/dev/null 2>&1
  ha=$(md5sum "$a"/*.gpr 2>/dev/null | awk '{print $1}')
  hb=$(md5sum "$b"/*.gpr 2>/dev/null | awk '{print $1}')
  sa=$(stat -c%s "$a"/*.gpr 2>/dev/null); sb=$(stat -c%s "$b"/*.gpr 2>/dev/null)
  if [[ -n "$ha" && "$ha" == "$hb" ]]; then echo "  $m  IDENTICAL  ($sa bytes, $ha)"
  else echo "  $m  *** DIFFERS *** control=$ha/$sa skip=$hb/$sb"; fi
  rm -rf "$a" "$b"
done

echo
echo "=== throughput, interleaved, ${REPS} reps, ${THREADS} threads, ${DUR}s ==="
printf "%-5s %-9s %-9s %-8s\n" mode control skip delta
for m in m1 m5 m7 m10; do
  cs=(); ss=()
  for _ in $(seq "$REPS"); do
    cs+=("$(run "$Q/vc5_bench.control" "$m")")
    ss+=("$(run "$Q/vc5_bench.skip" "$m")")
  done
  python3 - "$m" "${cs[*]}" "${ss[*]}" <<'PY'
import sys
m, c, s = sys.argv[1], [float(x) for x in sys.argv[2].split()], [float(x) for x in sys.argv[3].split()]
mc, ms = sum(c)/len(c), sum(s)/len(s)
print(f"{m:<5} {mc:<9.2f} {ms:<9.2f} {(ms/mc-1)*100:+.2f}%   control {min(c):.1f}-{max(c):.1f}  skip {min(s):.1f}-{max(s):.1f}")
PY
done
