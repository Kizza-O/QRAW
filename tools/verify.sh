#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Kieran Olsson
# ===========================================================================
# verify.sh -- prove the optimised encoder still produces byte-identical
# output.
#
#   bash tools/verify.sh              # build and check against the reference
#   CRC=deadbeef bash tools/verify.sh # check against a different reference
#
# The optimisations in this encoder are all bit-exact by construction: they
# remove work or reorder access, never change a value. This is what holds
# that claim to account.
#
# THE REFERENCE NUMBER. 72f44899 as of the r36 "quant-v3" encoder.
#
# It moved from 455e725e, and that was intentional: the r36 quantiser work
# changes the coefficients, so the container legitimately differs (2,988,814
# bytes against the previous 3,052,650 -- smaller, as a quantiser change
# should be). The gate flagged it, which is the gate working.
#
# What made it safe to re-baseline rather than investigate: ALL THREE PATHS
# agreed on the new value. A quantiser change shows up identically in the
# shipped emit, the v2 emit and the production library; a BUG would almost
# certainly not. If you change the quantiser again, expect this number to
# move again -- and if the three paths ever disagree, do not re-baseline,
# because that is a real defect.
#
# Reproducing it needs the pipeline configured the way the live one is, and
# the first version of this gate got that wrong in two ways at once: it read
# the 12-bit ALREADY-COMPANDED sample instead of the 16-bit linear one, so
# the GP-Log2 curve was applied twice, and it skipped the white
# normalisation that main() does after parsing. Both produced a perfectly
# plausible container of the wrong size. If you change the sample, the mode,
# the compand settings or the white point, expect a different number and do
# not assume the encoder broke.
#
# Cross-building? Set CXX to an aarch64 compiler and it will run the result
# under qemu-aarch64-static if one is installed.
# ===========================================================================
set -Eeuo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
B="$ROOT/benchmark"
S="$B/third_party/gpr/source"
SAMPLE="${SAMPLE:-$B/cinepi_qraw_bench/input/sample_imx585_3840x2160_gbrg_16bit.raw16}"
PARAMS="${PARAMS:-$(find "$B" -name gpr_params.json -print -quit)}"
CRC="${CRC:-72f44899}"
OUT="$B/build/verify_crc"

CXX="${CXX:-g++}"
RUN=""
case "$("$CXX" -dumpmachine)" in
  aarch64*) ;;
  *) if command -v aarch64-linux-gnu-g++ >/dev/null; then
       CXX=aarch64-linux-gnu-g++
       command -v qemu-aarch64-static >/dev/null &&
         RUN="qemu-aarch64-static -L /usr/aarch64-linux-gnu"
     fi ;;
esac

[[ -r "$SAMPLE" ]] || { echo "no sample at $SAMPLE" >&2; exit 2; }
mkdir -p "$B/build"

INC=(-I"$B/cinepi_qraw_bench" -I"$ROOT/encoder_library"
     -I"$B/cinepi_qraw_bench/deps/vulkan-headers-1.3.290"
     -I"$S/app/gpr_tools" -I"$S/app/common/cJSON" -I"$S/lib/common/private"
     -I"$S/lib/common/public" -I"$S/lib/vc5_common" -I"$S/lib/vc5_encoder"
     -I"$S/lib/vc5_decoder" -I"$S/lib/gpr_sdk/public" -I"$S/lib/gpr_sdk/private"
     -I"$S/lib/dng_sdk" -I"$S/lib/md5_lib" -I"$S/lib/tiny_jpeg"
     -I"$S/lib/xmp_core" -I"$S/lib/xmp_core/public/include" -I"$S/lib/expat_lib")
DEFS=(-DCINEPI_NO_MAIN=1 -DNEON=1 -DGPR_READING=1 -DGPR_WRITING=1 -DNDEBUG
      -DXML_STATIC=1 -DGPR_TIMING=0 -DGIT_BRANCH='""' -DGIT_COMMIT_HASH='""')

OBJ="$B/build/verify_obj"
rm -rf "$OBJ"; mkdir -p "$OBJ"
echo "== compiling the SDK =="
for d in common/private vc5_common vc5_encoder vc5_decoder gpr_sdk/private \
         dng_sdk xmp_core md5_lib tiny_jpeg expat_lib; do
  while IFS= read -r f; do
    # Unique per PATH: several SDK directories share a basename (sections.c,
    # vc5_encoder.c ...), and a plain basename silently overwrote them, which
    # showed up as undefined references at link time.
    b="$(basename "${f%.*}")_$(echo "$f" | cksum | cut -d' ' -f1)"
    if [[ "$f" == *.cpp ]]; then
      "$CXX" -O2 -std=c++17 -mcpu=cortex-a76 -w "${DEFS[@]}" "${INC[@]}" \
        -c "$f" -o "$OBJ/$b.o"
    else
      "${CXX%++}cc" -O2 -mcpu=cortex-a76 -w "${DEFS[@]}" "${INC[@]}" \
        -c "$f" -o "$OBJ/$b.o" 2>/dev/null ||
      "$CXX" -O2 -x c -mcpu=cortex-a76 -w "${DEFS[@]}" "${INC[@]}" \
        -c "$f" -o "$OBJ/$b.o"
    fi
  done < <(find "$S/lib/$d" -maxdepth 1 \( -name '*.c' -o -name '*.cpp' \) | sort)
done
for f in "$S/app/common/cJSON/cJSON.c" "$S/app/gpr_tools/gpr_parse_utils.cpp"; do
  b="$(basename "${f%.*}")_$(echo "$f" | cksum | cut -d' ' -f1)"
  "$CXX" -O2 -std=c++17 -x c++ -mcpu=cortex-a76 -w "${DEFS[@]}" "${INC[@]}" \
    -c "$f" -o "$OBJ/$b.o"
done

echo "== compiling and linking the gate =="
"$CXX" -O2 -std=c++17 -mcpu=cortex-a76 -w "${DEFS[@]}" "${INC[@]}" \
  -c "$B/cinepi_qraw_bench/verify_crc.cpp" -o "$OBJ/verify_crc.o"
"$CXX" -O2 -mcpu=cortex-a76 "$OBJ"/*.o -o "$OUT" -lpthread -ldl -lm

echo "== running =="
fail=0
for variant in "" "--v2" "--library" "--library --strided" "--library --swapped16"; do
  label=${variant:-"--shipped"}
  $RUN "$OUT" --input "$SAMPLE" --mode m5 --compand-bits 12 \
       ${PARAMS:+--gpr-params "$PARAMS"} $variant --expect "$CRC" || fail=1
done
if [[ "$fail" -ne 0 ]]; then
  echo "GATE FAILED -- the paths must agree; see above" >&2
  exit 1
fi
printf "%s\\n" "all five paths agree: shipped emit, v2 emit, the production library," "  the library reading a PADDED buffer in place (stride 3872, crop 8,10," "  padding poisoned with 0xDEAD so reading it could not go unnoticed)," "  and the library reading the same pixels stored BIG-ENDIAN in a padded" "  buffer with src_byteswap=1 (v3.14 in-register endian fold)"
