#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Kieran Olsson
# Fast front door for static/file benchmarking.
# Default: restore or build only the proven v1.16.6 CPU-GPR benchmark binary.
# --full: build the same canonical encoder plus the decode correctness gate.
set -Eeuo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
BENCH="$HERE/cinepi_qraw_bench"
BIN="$BENCH/build/vc5_bench"
STAMP="$HERE/.static_winner_verified"
CACHE_ROOT="${XDG_CACHE_HOME:-$HOME/.cache}/cinepi-qraw/static-winning-v166"
FORCE=0 FULL=0 STATUS=0
for a in "$@"; do
  case "$a" in
    --force) FORCE=1 ;;
    --full) FULL=1 ;;
    --status) STATUS=1 ;;
    *) echo "usage: build_and_verify.sh [--force] [--full] [--status]" >&2; exit 2 ;;
  esac
done

if ((FULL)); then
  args=(); ((FORCE)) && args+=(--force); ((STATUS)) && args+=(--status)
  exec "$HERE/build_and_verify_full.sh" "${args[@]}"
fi

# Refuse to benchmark if the proven v1.16.6 hot-path source has drifted.
"$HERE/../tools/check_static_winner_source.sh" >/dev/null

# Hash only code that can change the static winning executable. Live-camera UI,
# CinePi-RAW integration and camera probes deliberately do not invalidate it.
source_hash() {
  {
    printf 'arch=%s\n' "$(uname -m)"
    printf 'compiler=%s\n' "$(g++ --version 2>/dev/null | head -1 || true)"
    printf 'lto=%s\n' "${CINEPI_LTO:-parallel}"
    ( cd "$HERE" && \
      find cinepi_qraw_bench third_party/gpr/source \
        -type f \( -name '*.c' -o -name '*.h' -o -name '*.cpp' -o -name '*.hpp' -o -name '*.inc' -o -name 'CMakeLists.txt' \) \
        -not -path '*/build/*' -not -name 'cinepi_live_camera.cpp' -not -name 'cinepi_live_camera.h' \
        -print0 2>/dev/null | sort -z | xargs -0 sha256sum 2>/dev/null )
    ( cd "$HERE" && sha256sum cinepi_qraw_bench/build_static_winner_fast.sh ) 2>/dev/null || true
  } | sha256sum | cut -d' ' -f1
}

HASH=$(source_hash)
CACHE="$CACHE_ROOT/$HASH"
valid_bin() {
  [[ -x "$BIN" ]] && "$BIN" --version 2>/dev/null | grep -q '^CINEPI_VC5_BENCH_VERSION 1\.16\.6$'
}

if ((STATUS)); then
  if valid_bin && [[ -f "$STAMP" && "$(cat "$STAMP")" == "$HASH" ]]; then
    echo "verified ${HASH:0:12} (local)"
  elif [[ -x "$CACHE/vc5_bench" ]]; then
    echo "verified ${HASH:0:12} (persistent cache)"
  else
    echo "stale"
  fi
  exit 0
fi

if ((FORCE == 0)) && valid_bin && [[ -f "$STAMP" && "$(cat "$STAMP")" == "$HASH" ]]; then
  echo "static winning encoder: REUSE local verified v1.16.6 (${HASH:0:12})"
  exit 0
fi

if ((FORCE == 0)) && [[ -x "$CACHE/vc5_bench" ]]; then
  mkdir -p "$BENCH/build"
  cp -a "$CACHE/." "$BENCH/build/"
  if valid_bin; then
    printf '%s\n' "$HASH" > "$STAMP"
    echo "static winning encoder: RESTORED persistent cache v1.16.6 (${HASH:0:12})"
    exit 0
  fi
fi

rm -f "$STAMP"
echo "static winning encoder: native fast build v1.16.6"
echo "  only vc5_bench + linked libraries; no experimental encoder targets"
"$BENCH/build_static_winner_fast.sh"
valid_bin || { echo "FATAL: fast build did not produce v1.16.6" >&2; exit 3; }
printf '%s\n' "$HASH" > "$STAMP"
mkdir -p "$CACHE"
cp -a "$BENCH/build/vc5_bench" "$CACHE/vc5_bench"
[[ -f "$BENCH/build/self_test.log" ]] && cp -a "$BENCH/build/self_test.log" "$CACHE/self_test.log"
echo "verified and cached (${HASH:0:12}); future extractions can reuse this binary"
