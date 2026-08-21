#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Kieran Olsson
# ===========================================================================
# build_qraw_core.sh -- build the CinePi GPR encoder core as static archives
# for cinepi-raw to link against.
#
#   ./build_qraw_core.sh [prefix]
#
# The encoder source is NOT selectable. r22 always builds the bundled,
# SHA-pinned v1.16.6 winning stack.
#
# Prefix defaults to /opt/cinepi-qraw (sudo needed to install there).
# Produces:
#   <prefix>/lib/libcinepi_qraw.a   -- the FIXED library face in this folder
#                                     (white normalisation + black field),
#                                     compiled against the benchmark package's
#                                     vc5_bench.cpp, plus gpr_parse_utils
#   <prefix>/lib/lib*.a            -- the GoPro GPR SDK archives
#   <prefix>/include/cinepi_qraw_encoder.h
#
# On aarch64 this uses the same -mcpu=cortex-a76 -flto=auto flags as the
# canonical CMake target, so the linked encoder is the measured
# configuration and its numbers stay comparable with the campaign's.
#
# r19 fast build: the GoPro SDK archives are independently fingerprinted under
# the install prefix. If they already match, only the small CinePi encoder face
# is rebuilt. If they do not match, CMake builds only the static libraries that
# CinePi-RAW links, never the standalone apps/benchmarks.
# ===========================================================================
set -Eeuo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)

# r22 single-encoder policy: the package's bundled winner is the only source.
PKG=$(cd "$HERE/../benchmark" && pwd)
PREFIX=${1:-/opt/cinepi-qraw}
if [[ ! -d "$PKG" ]]; then
    echo "FATAL: bundled benchmark package not found at $PKG"
    exit 2
fi
"$HERE/../tools/check_single_encoder.sh"
SDK="$PKG/third_party/gpr"
BENCH="$PKG/cinepi_qraw_bench"
[[ -f "$BENCH/vc5_bench.cpp" ]] || { echo "FATAL: $BENCH/vc5_bench.cpp not found"; exit 2; }

ARCH=$(uname -m)
# GPR_TIMING defaults to 1 in gpr_platform.h, which compiles the GoPro SDK's
# TIMESTAMP() macros into real printf() calls. gpr_convert_preencoded_vc5_to_gpr()
# carries a TIMESTAMP at level 1, so every encoded frame emitted a [BEG] and an
# [END] line to stdout -- from the ENCODER WORKER THREADS, i.e. on the pinned
# hot cores, through the shared stdio lock, into a log file on the boot/root
# device. Measured in a 45 s live run: 1033 [BEG] + 1034 [END] lines for 1109
# frames. LogPrint() is also not thread safe: it mutates one global TIMER from
# every worker. None of this is wanted in a benchmark, so compile it out.
GPR_QUIET_FLAGS="-DGPR_TIMING=0"
CPU_FLAGS="$GPR_QUIET_FLAGS"
case "$ARCH" in
  aarch64|arm64) CPU_FLAGS="-mcpu=cortex-a76 -flto=auto $GPR_QUIET_FLAGS" ;;
  x86_64|amd64)  echo "NOTE: host-validation build on $ARCH; camera target is aarch64." ;;
esac

BUILD=$(mktemp -d)
trap 'rm -rf "$BUILD"' EXIT

SDK_STAMP="$PREFIX/.sdk_source_sha256"
SDK_REQUIRED=(
  libcJSON.a libcommon.a libvc5_common.a libvc5_decoder.a libvc5_encoder.a
  libdng_sdk.a libgpr_sdk.a libxmp_core.a libexpat_lib.a libmd5_lib.a
  libtiny_jpeg.a
)
# Fingerprint only the private SDK libraries that the winning encoder actually
# links. Historical app/tests must not invalidate these archives. This is what
# lets a cleaned package reuse the exact same already-built dependencies.
SDK_WANT=$(
  cd "$SDK"
  {
    for d in \
      source/lib/common source/lib/vc5_common source/lib/vc5_decoder \
      source/lib/vc5_encoder source/lib/dng_sdk source/lib/gpr_sdk \
      source/lib/xmp_core source/lib/expat_lib source/lib/md5_lib \
      source/lib/tiny_jpeg source/app/common/cJSON; do
      find "$d" -type f \
        \( -name 'CMakeLists.txt' -o -name '*.cmake' -o -name '*.c' -o -name '*.h' \
           -o -name '*.cpp' -o -name '*.hpp' \) -print0
    done | sort -z | xargs -0 sha256sum
    # Hash only the root build-policy section, not app target declarations.
    sed -n '1,/^# add needed subdirectories/p' CMakeLists.txt | sha256sum
    # neon= must be part of the fingerprint: it is a cmake -D, not part of
    # CPU_FLAGS, so without it the stamp would still match and the archives
    # would be REUSED -- leaving the NEON parity fix silently inert.
    printf 'sdk_policy=private-winner-libs-v2 arch=%s flags=%s neon=%s\n' \
           "$ARCH" "$CPU_FLAGS" "ON"
  } | sha256sum | awk '{print $1}'
)
SDK_HAVE=$(cat "$SDK_STAMP" 2>/dev/null || true)
SDK_OK=1
for a in "${SDK_REQUIRED[@]}"; do
  [[ -f "$PREFIX/lib/$a" ]] || { SDK_OK=0; break; }
done
SDK_MIGRATE=0
# r21 used a whole-SDK fingerprint, so removing unused encoder apps would look
# like a dependency change. The actual SDK library sources are unchanged. On
# Pi5 accept that known r21 stamp once and rewrite it to the narrow fingerprint.
R21_SDK_STAMP_AARCH64=a07df0f73df280d073c5d654fae6919d84134ee6073501db6b0e6af8bc0d14ba
if [[ "$SDK_OK" == 1 && ( "$ARCH" == "aarch64" || "$ARCH" == "arm64" ) && \
      "$SDK_HAVE" == "$R21_SDK_STAMP_AARCH64" ]]; then
  SDK_MIGRATE=1
fi

if [[ "$SDK_OK" == 1 && -n "$SDK_HAVE" && ( "$SDK_HAVE" == "$SDK_WANT" || "$SDK_MIGRATE" == 1 ) ]]; then
  if [[ "$SDK_MIGRATE" == 1 ]]; then
    echo "== 1/3 GPR SDK archives: REUSE r21 libraries; migrate cache stamp =="
  else
    echo "== 1/3 GPR SDK archives: REUSE $PREFIX/lib =="
  fi
else
  # The SDK's bench-app CMakeLists probes the Vulkan loader at configure time,
  # even though these static archives do not use it. Only perform that probe
  # when an SDK rebuild is actually required; a reuse-only launch needs no
  # compiler/CMake/Vulkan dependency work at all.
  _ldconfig_cache=$(ldconfig -p 2>/dev/null || true)
  if ! grep -q libvulkan.so.1 <<< "$_ldconfig_cache"; then
    echo "FATAL: libvulkan.so.1 not found. The GPR SDK CMake configure requires it."
    echo "Run the benchmark package's install_pi_dependencies.sh, or:"
    echo "  sudo apt install -y libvulkan1 libvulkan-dev"
    exit 2
  fi
  echo "== 1/3 GPR SDK archives: build required libraries only =="
  # CINEPI_ENABLE_NEON must match the static winner build. It defaults OFF in
  # third_party/gpr/CMakeLists.txt, and build_static_winner_fast.sh:46 passes
  # ON -- so without this the live library and the static reference were
  # DIFFERENT binaries: verified with gcc-nm, 6 *_NEON_* symbols in the bench's
  # libvc5_encoder.a against 0 in /opt/cinepi-qraw/lib/libvc5_encoder.a. The
  # NEON-gated functions sit in forward.c/raw.c, which the pretransformed live
  # path largely bypasses, so this is a parity/identity fix rather than an
  # expected speed-up -- but "one immutable encoder implementation" has to be
  # true at the binary level for any A/B across the two paths to mean anything.
  cmake -S "$SDK" -B "$BUILD/sdk" -DCMAKE_BUILD_TYPE=Release \
        -DCINEPI_ENABLE_NEON=ON \
        -DCMAKE_C_FLAGS="-O3 -DNDEBUG $CPU_FLAGS" \
        -DCMAKE_CXX_FLAGS="-O3 -DNDEBUG $CPU_FLAGS" > "$BUILD/cmake.log" 2>&1 \
        || { tail -30 "$BUILD/cmake.log"; exit 1; }
  cmake --build "$BUILD/sdk" \
        --target common vc5_common vc5_decoder vc5_encoder dng_sdk gpr_sdk \
                 xmp_core expat_lib md5_lib tiny_jpeg cJSON \
        --parallel "$(getconf _NPROCESSORS_ONLN)" \
        >> "$BUILD/cmake.log" 2>&1 || { tail -30 "$BUILD/cmake.log"; exit 1; }
fi

echo "== 2/3 fixed library face =="
S="$SDK/source"
INC=(-I"$HERE" -I"$BENCH" -I"$BENCH/deps/vulkan-headers-1.3.290"
     -I"$S/app/gpr_tools" -I"$S/app/common/cJSON"
     -I"$S/lib/common/private" -I"$S/lib/common/public"
     -I"$S/lib/vc5_common" -I"$S/lib/vc5_encoder" -I"$S/lib/vc5_decoder"
     -I"$S/lib/gpr_sdk/public" -I"$S/lib/gpr_sdk/private"
     -I"$S/lib/dng_sdk" -I"$S/lib/md5_lib")
# -I"$HERE" first: the FIXED cinepi_qraw_encoder.h in this folder must win
# over the unfixed one sitting next to vc5_bench.cpp in the package.
# -DNDEBUG for parity with the SDK archives above and with the static winner
# build; without it these two TUs keep assert() live while every other object
# in the same archive has it compiled out.
g++ -O3 -DNDEBUG -std=c++17 $CPU_FLAGS -DGPR_READING=1 -DGPR_WRITING=1 \
    -Wno-missing-field-initializers \
    -c "$HERE/cinepi_qraw_encoder.cpp" -o "$BUILD/cinepi_qraw_encoder.o" "${INC[@]}"
g++ -O3 -DNDEBUG -std=c++17 $CPU_FLAGS -DGPR_READING=1 -DGPR_WRITING=1 \
    -c "$S/app/gpr_tools/gpr_parse_utils.cpp" -o "$BUILD/gpr_parse_utils.o" "${INC[@]}"
ar rcs "$BUILD/libcinepi_qraw.a" "$BUILD/cinepi_qraw_encoder.o" "$BUILD/gpr_parse_utils.o"

echo "== 3/3 install to $PREFIX =="
SUDO=""; [[ -w "$(dirname "$PREFIX")" ]] || SUDO=sudo
$SUDO mkdir -p "$PREFIX/lib" "$PREFIX/include"
$SUDO cp "$BUILD/libcinepi_qraw.a" "$PREFIX/lib/"
if [[ -d "$BUILD/sdk" ]]; then
  for a in "${SDK_REQUIRED[@]}"; do
    built=$(find "$BUILD/sdk" -name "$a" -print -quit)
    [[ -n "$built" ]] || { echo "FATAL: required SDK archive was not built: $a" >&2; exit 2; }
    $SUDO cp "$built" "$PREFIX/lib/$a"
  done
  printf '%s\n' "$SDK_WANT" | $SUDO tee "$SDK_STAMP" >/dev/null
elif [[ "$SDK_MIGRATE" == 1 ]]; then
  printf '%s\n' "$SDK_WANT" | $SUDO tee "$SDK_STAMP" >/dev/null
fi
$SUDO cp "$HERE/cinepi_qraw_encoder.h" "$PREFIX/include/"
# LIBRARY_SOURCE_SHA256 fingerprints the sources that actually produce the
# encoder archives. Version string, source SHA and build flags all describe the
# INTENT; this describes what was compiled. Without it, changing the library
# face (e.g. routing the wavelet through the direct tile-hybrid path) rebuilds
# /opt/cinepi-qraw but leaves cinepi-raw linked against the previous archives,
# because the installer's coherence check saw an unchanged ENCODER_ID. That is
# the same class of silent staleness the source/binary stamp already guards.
# Covers BOTH halves of the encoder: the CinePi library face / vc5_bench.cpp,
# and the GoPro SDK's VC-5 encoder sources. The entropy coder lives in the
# latter, and an earlier version of this fingerprint omitted it -- so improving
# encoder.c rebuilt nothing and cinepi-raw kept linking the previous archives.
LIBRARY_SOURCE_SHA=$( { cat "$HERE/cinepi_qraw_encoder.cpp" "$HERE/cinepi_qraw_encoder.h" \
                            "$BENCH/vc5_bench.cpp" 2>/dev/null
                        find "$SDK/source/lib/vc5_encoder" "$SDK/source/lib/vc5_common" \
                             -type f \( -name '*.c' -o -name '*.h' \) -print0 2>/dev/null \
                          | sort -z | xargs -0 cat 2>/dev/null
                      } | sha256sum | awk '{print $1}')

# BUILD_FLAGS records the compile-time configuration of the installed archives.
# Encoder identity alone is not enough to decide reuse: a flag such as
# -DGPR_TIMING=0 changes the generated code (it removes per-frame printf() calls
# from the encode path) without changing the encoder version or its source SHA,
# so an installed core built with different flags must rebuild once.
#
# r23: this used to record only $GPR_QUIET_FLAGS, i.e. "-DGPR_TIMING=0", and
# omitted the flags that matter most -- "-mcpu=cortex-a76 -flto=auto". Changing
# the optimisation flags therefore left ENCODER_ID byte-identical, so
# run_qraw_pipeline.sh's staleness check concluded the installed core was current
# and kept linking the OLD archives. Any flag A/B would have compared two
# identical binaries. It now records the full $CPU_FLAGS.
$SUDO tee "$PREFIX/ENCODER_ID" >/dev/null <<EOF
CINEPI_QRAW_ENCODER=1.16.6-winning-stack-irq-vle128
VC5_SOURCE_SHA256=95865cd4c103ae3494905e3788a2483062ae836f293142191a00def35804ddfd
POLICY=single-canonical-source
QUANT_TABLE=CinePi-Universal-Standard-Quant-v2
BUILD_FLAGS=$CPU_FLAGS
LIBRARY_SOURCE_SHA256=$LIBRARY_SOURCE_SHA
EOF

# The encoder core's default GPR metadata path is relative
# ("validated_input/gpr_params.json"), which only works when running from
# the benchmark folder. Install an absolute copy so any application --
# cinepi-raw included -- can find it.
sudo mkdir -p "$PREFIX/share/cinepi-qraw"
if [[ -f "$PKG/validated_input/gpr_params.json" ]]; then
    sudo cp "$PKG/validated_input/gpr_params.json" "$PREFIX/share/cinepi-qraw/gpr_params.json"
    echo "installed GPR metadata: $PREFIX/share/cinepi-qraw/gpr_params.json"
else
    echo "WARNING: $PKG/validated_input/gpr_params.json not found -- GprEncoder will need CINEPI_QRAW_PARAMS"
fi

echo
echo "installed:"
ls -la "$PREFIX/lib" "$PREFIX/include"
echo
if [[ "$ARCH" == "aarch64" || "$ARCH" == "arm64" ]]; then
  echo "regression gate: run tools/verify.sh --library on the Pi before trusting footage"
  echo "expected canonical encoder: 1.16.6-winning-stack"
else
  echo "NOTE: production winner built for host syntax/link validation only."
  echo "      Execution is intentionally refused without AArch64 NEON; there is no fallback encoder."
fi
