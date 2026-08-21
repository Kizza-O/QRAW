#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Kieran Olsson
# Fast native build for the proven all-CPU v1.16.6 static winning encoder.
# Builds only vc5_bench and the libraries it actually links. No shaders, labs,
# decoder CLI, camera helper, or experimental applications are built here.
set -Eeuo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/.." && pwd)
PKG_ROOT=$(cd "$ROOT/.." && pwd)
"$PKG_ROOT/tools/check_static_winner_source.sh" >/dev/null
SDK="$ROOT/third_party/gpr"
SDK_BUILD="${GPR_BUILD_DIR:-$SDK/build}"
OUT="$HERE/build"
JOBS=${CINEPI_BUILD_JOBS:-$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 4)}
[[ "$JOBS" =~ ^[1-9][0-9]*$ ]] || { echo "FATAL: CINEPI_BUILD_JOBS must be positive" >&2; exit 2; }

for tool in cmake g++; do
  command -v "$tool" >/dev/null 2>&1 || { echo "FATAL: required build tool not found: $tool" >&2; exit 2; }
done

ARCH=$(uname -m)
NEON=OFF
# -DGPR_TIMING=0: the GoPro SDK's TIMESTAMP() macros default to ON and expand to
# printf() calls on the encode path. The static and live encoders must be the
# same code, so both compile them out. See encoder_library/build_qraw_core.sh.
SDK_RELEASE_FLAGS="-O3 -DNDEBUG -DGPR_TIMING=0"
LINK_FLAGS=""
case "$ARCH" in
  aarch64|arm64)
    NEON=ON
    LTO=-flto=auto
    [[ "${CINEPI_LTO:-parallel}" == "serial" ]] && LTO=-flto
    SDK_RELEASE_FLAGS="-O3 -DNDEBUG -DGPR_TIMING=0 -mcpu=cortex-a76 $LTO"
    LINK_FLAGS="-mcpu=cortex-a76 $LTO"
    ;;
  *) echo "NOTE: host validation build on $ARCH; Pi5 target is aarch64." ;;
esac

# A CMake cache records absolute source paths. Drop only a cache copied from a
# different extraction; normal rebuilds retain all object files.
if [[ -f "$SDK_BUILD/CMakeCache.txt" ]] && ! grep -qF "$SDK" "$SDK_BUILD/CMakeCache.txt" 2>/dev/null; then
  rm -rf "$SDK_BUILD"
fi

cmake -S "$SDK" -B "$SDK_BUILD" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCINEPI_ENABLE_NEON="$NEON" \
  -DCMAKE_C_FLAGS_RELEASE="$SDK_RELEASE_FLAGS" \
  -DCMAKE_CXX_FLAGS_RELEASE="$SDK_RELEASE_FLAGS" \
  -DCMAKE_EXE_LINKER_FLAGS_RELEASE="$LINK_FLAGS" >/dev/null

# CMake resolves and builds only this target's actual static-library dependencies.
cmake --build "$SDK_BUILD" --target cinepi_vulkan_gpr_bench --parallel "$JOBS"

mkdir -p "$OUT"
cp -f "$SDK_BUILD/source/app/cinepi_vulkan_gpr_bench/vc5_bench" "$OUT/vc5_bench"
chmod +x "$OUT/vc5_bench"

expected=$(grep -oE 'CINEPI_VERSION = "[0-9]+\.[0-9]+\.[0-9]+"' "$HERE/vc5_bench.cpp" | head -1 | grep -oE '[0-9]+\.[0-9]+\.[0-9]+')
actual=$($OUT/vc5_bench --version)
[[ "$actual" == "CINEPI_VC5_BENCH_VERSION $expected" ]] || {
  echo "FATAL: built '$actual', expected v$expected" >&2; exit 3; }

# The encoder's built-in CPU correctness test is cheap compared with rebuilding
# the SDK/labs and catches a bad native build before timing is allowed.
( cd "$HERE" && "$OUT/vc5_bench" --self-test --log "$OUT/self_test.log" ) >/dev/null
printf 'FAST_BUILD_OK encoder=%s target=%s jobs=%s\n' "$expected" "$ARCH" "$JOBS"
