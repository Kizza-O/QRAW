#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Kieran Olsson
# Full correctness verification for the SINGLE canonical encoder.
# Builds the SHA-pinned v1.16.6 winner and the decode verifier only.
set -Eeuo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
BENCH="$HERE/cinepi_qraw_bench"
GPRB="$HERE/third_party/gpr/build"
STAMP="$HERE/.build_verified"
FORCE=0; STATUS_ONLY=0
for a in "$@"; do
  case "$a" in
    --force) FORCE=1 ;;
    --status) STATUS_ONLY=1 ;;
    *) echo "usage: build_and_verify_full.sh [--force] [--status]"; exit 2 ;;
  esac
done

"$HERE/../tools/check_single_encoder.sh" >/dev/null
source_hash(){
  { sha256sum "$BENCH/vc5_bench.cpp" "$BENCH/build_static_winner_fast.sh" \
      "$HERE/../encoder_library/cinepi_qraw_encoder.cpp" "$HERE/../encoder_library/cinepi_qraw_encoder.h" \
      "$HERE/third_party/gpr/source/app/cinepi_vulkan_gpr_bench/CMakeLists.txt" \
      "$HERE/third_party/gpr/source/app/cinepi_gpr_sdk/CMakeLists.txt" 2>/dev/null || true;
    printf 'arch=%s\n' "$(uname -m)"; } | sha256sum | awk '{print $1}'
}
HASH=$(source_hash)
valid(){ [[ -x "$BENCH/build/vc5_bench" && -x "$BENCH/build/gpr_decode_verify" ]] && \
  "$BENCH/build/vc5_bench" --version 2>/dev/null | grep -q '^CINEPI_VC5_BENCH_VERSION 1\.16\.6$'; }
if ((STATUS_ONLY)); then
  valid && [[ -f "$STAMP" && "$(cat "$STAMP")" == "$HASH" ]] && echo "verified ${HASH:0:12}" || echo stale
  exit 0
fi
if ((FORCE==0)) && valid && [[ -f "$STAMP" && "$(cat "$STAMP")" == "$HASH" ]]; then
  echo "single canonical encoder + decoder proof: REUSE (${HASH:0:12})"; exit 0
fi
rm -f "$STAMP"
"$BENCH/build_static_winner_fast.sh"
# build_static_winner_fast configured the SDK build tree. Build decode libraries only.
cmake --build "$GPRB" --target vc5_decoder --parallel "$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 4)"
VERIFY_FLAGS=(-O2 -w -pthread)
[[ "$(uname -m)" == aarch64 ]] && VERIFY_FLAGS+=(-mcpu=cortex-a76 -flto=auto)
mapfile -t GPRLIBS < <(find "$GPRB" -name '*.a')
((${#GPRLIBS[@]})) || { echo 'FATAL: no SDK static libraries found' >&2; exit 3; }
g++ "${VERIFY_FLAGS[@]}" -I"$HERE/third_party/gpr/source/lib/gpr_sdk/public" \
  -I"$HERE/third_party/gpr/source/lib/common/public" \
  "$BENCH/gpr_decode_verify.cpp" -Wl,--start-group "${GPRLIBS[@]}" -Wl,--end-group \
  -o "$BENCH/build/gpr_decode_verify"
"$BENCH/build/vc5_bench" --self-test --log "$BENCH/build/self_test.log" >/dev/null
printf '%s\n' "$HASH" > "$STAMP"
echo "verified single canonical v1.16.6 winner + decode gate (${HASH:0:12})"
