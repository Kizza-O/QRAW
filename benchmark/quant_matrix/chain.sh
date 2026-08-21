#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Kieran Olsson
# Wait for Stage A, then screen the full quant-table matrix.
# Stops loudly rather than silently falling back, so a broken screening path
# cannot masquerade as a finished matrix.
set -uo pipefail
cd "$(dirname "$0")"

echo "=== waiting for Stage A ==="
while pgrep -f "qm.py A" >/dev/null; do sleep 15; done
echo "Stage A finished at $(date +%H:%M:%S), $(( $(wc -l < master.csv) - 1 )) rows"

echo
echo "=== regenerating matrix with complete Stage A sensitivity ==="
python3 gen_matrix.py --lambdas 60 || { echo "CHAIN FAILED: gen_matrix"; exit 1; }

echo
echo "=== smoke test: 4 tables through the direct screening path ==="
python3 screen.py --limit 4 --jobs 2 --threads 2 || { echo "CHAIN FAILED: screen smoke"; exit 1; }
ok=$(awk -F, 'NR>1 && $12=="yes"' screen.csv 2>/dev/null | wc -l)
echo "smoke test produced $ok/4 good rows"
if [[ "$ok" -lt 3 ]]; then
  echo "CHAIN FAILED: direct vc5_bench --save-gpr screening does not work."
  echo "Falling back would cost 2.4 h for 794 tables; stopping for a decision."
  exit 1
fi

echo
echo "=== full matrix screen ==="
python3 screen.py --jobs 4 --threads 1 --keep-gpr screened_gpr || {
  echo "CHAIN FAILED: full screen"; exit 1; }

echo
echo "=== screening complete at $(date +%H:%M:%S) ==="
awk -F, 'NR>1 && $12=="yes"' screen.csv | wc -l | xargs echo "tables screened:"
