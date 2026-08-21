#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Kieran Olsson
set -Eeuo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
WIN="$ROOT/benchmark/cinepi_qraw_bench/vc5_bench.cpp"
FACE="$ROOT/encoder_library/cinepi_qraw_encoder.cpp"
HDR="$ROOT/encoder_library/cinepi_qraw_encoder.h"

check_hash() {
  local expected=$1 file=$2 got
  [[ -f "$file" ]] || { echo "FATAL: missing canonical encoder file: $file" >&2; exit 51; }
  got=$(sha256sum "$file" | awk '{print $1}')
  [[ "$got" == "$expected" ]] || {
    echo "FATAL: canonical v1.16.6 winning encoder drifted: $file" >&2
    echo " expected=$expected" >&2; echo " got=$got" >&2; exit 52;
  }
}
# v3.10: re-pinned. vc5_bench.cpp changed to accept a strided source, which
# removes a 16.8 MB per-frame copy from the camera path. Re-pinning a drift
# guard is only safe when the OUTPUT is provably unchanged, and it is: all
# four gate paths -- shipped, v2, library, and library reading a poisoned
# padded buffer through the new stride -- produce 72f44899 on 2988814 bytes.
# Do not update this hash for a change that moves the CRC.
#
# v3.13: re-pinned again. vc5_bench.cpp gained src_shift -- an in-register
# right shift applied to source samples at load, before the 4096-entry LUT --
# so an MSB-justified RAW12 DMA buffer can be read where it lies with no
# staging pass and no 65536-entry LUT. Default 0 keeps every historical path
# byte-identical: all four gate paths still produce 72f44899 on 2988814
# bytes, and the new tight12/shifted12 gate pair (same 12-bit samples staged
# vs read in place with src_shift=4) agree with each other exactly.
# v3.14: re-pinned again. vc5_bench.cpp gained src_byteswap -- an in-register
# byte-swap applied to source samples at load, before src_shift and before
# the companding LUT (order: load -> byteswap -> shift -> LUT index) -- so a
# BIG-ENDIAN IMX585 RAW16 DMA buffer can be read where it lies and libcamera's
# frame-wide software endian swap (LIBCAMERA_RPI_SKIP_16BIT_ENDIAN_SWAP=1) can
# be skipped. Default 0 keeps every historical path byte-identical: all five
# gate paths (shipped, v2, library, library-strided, and the new
# library-swapped16 -- the same pixels stored big-endian in a poisoned padded
# buffer with src_byteswap=1) produce 72f44899 on 2988814 bytes.
# Do not update this hash for a change that moves the CRC.
# Re-pinned once more the same day: a concurrent campaign added the wav1
# lowpass vqadd fast path (lofast) to cpu_wavelet_pass after the v3.14/v3.15
# pin. The full five-path gate (tools/verify.sh) passes on the combined
# source at 72f44899 / 2988814 bytes, which is what makes this re-pin safe.
# v3.16 (this change): re-pinned once more. vc5_bench.cpp gained the tile
# nonzero mask -- the direct tile-hybrid wavelet now emits one bit per
# high-pass coefficient as it narrows the lane to int8 (sharing the cross-lane
# reduce the int8 range test already paid for), and the VC-5 tile-hybrid
# entropy reader consumes it instead of rediscovering zero runs by reading all
# the coefficients back. vc5_bench.cpp also gained --cpu-direct-hybrid support
# in the 3-worker cpu-gpr pipeline so the sidecar and the hybrid can be A/B'd
# on one harness. Re-pinning is safe on the same rule as every re-pin above:
# the OUTPUT did not move. tools/verify.sh reports 72f44899 / 2988814 across
# all five paths, and the library CRCs are unchanged at m1 4674434/9e58f761,
# m2 4305506/85abc9d6, m5 2988814/72f44899, m7 2244822/d2d724ea -- with the
# mask emitted and consumed, emitted and ignored (CINEPI_ENT_MASK=0), not
# emitted at all (CINEPI_WAV_NZMASK=0), and with the hybrid itself off
# (CINEPI_QRAW_DIRECT_HYBRID=0).
# v3.17 (this change): re-pinned once more. vc5_bench.cpp gained the
# EXPERIMENTAL per-subband quantiser override CINEPI_BAND_Q, so the wavelet
# band-pruning candidates (test plan E1/E2/E3/E6) can be encoded on the FIXED
# reference frame and compared per-pixel. The live library face already had
# the knob; without it here, a band-pruned candidate could only ever be judged
# against a drifting camera scene, which is not a quality measurement.
# Re-pinning is safe on exactly the same rule as every re-pin above: the OUTPUT
# did not move. The override reads one environment variable at mode-resolution
# time and does nothing when it is absent, so with no CINEPI_BAND_Q set the
# encoder is byte-for-byte the previous one -- tools/verify.sh reports
# 72f44899 / 2988814 across all five paths, and static m7 still emits
# gpr_bytes 2244822 at ratio 7.38981 on the reference frame.
# A run WITH CINEPI_BAND_Q set is experimental footage by definition and is
# not covered by, and must never be used to re-baseline, the CRC gate.
# Do not update this hash for a change that moves the CRC.
# r38: re-pinned for the ClearHDR gradation-compand path. vc5_bench.cpp gained
# --gradation-compand, which decodes the IMX585 gradation curve and re-encodes
# as GP-Log2 so a .gpr carries the transfer function the format expects and
# renders natively; storing the sensor's codes raw left readers treating them
# as linear, collapsing the highlights and skewing colour. The new branch is
# OPT-IN and unreachable unless the flag is set, so the shipped output is
# unchanged -- verified below the same way every previous re-pin was: all five
# gate paths still produce 72f44899 on 2988814 bytes.
# Do not update this hash for a change that moves the CRC.
# r39: re-pinned for the saved-frame settings ID. The ONLY change in
# vc5_bench.cpp is the name --save-gpr gives a file: frame_00000_m5.gpr becomes
# frame_00000_m5-c12-b16-w3.gpr, so a saved frame still says which operating
# point produced it after it is copied out of its directory or exported to DNG.
# It touches no coefficient, no LUT and no container byte -- the string is built
# at the write site and nothing reads it back. Verified the same way as every
# re-pin above: all five gate paths still produce 72f44899 on 2988814 bytes
# (tools/verify.sh, this tree).
# Do not update this hash for a change that moves the CRC.
# r40: re-pinned for the HH band-prune elision. When a subband's divisor makes
# every possible coefficient quantise to zero (CINEPI_BAND_Q=9=32767, the
# shipped E1 grade), fused_level_init() now sets hh_provably_zero and
# v2_direct_emit_row_t skips the interior highpass, quantise, narrow, mask and
# store for that band -- 25% of all high-pass coefficients at E1, previously
# computed in full and then read back as zeros. The bound proof is in the
# comment at the flag's assignment; the edge filters (first/last column group,
# first/last row) are excluded because their wider taps exceed the threshold by
# ~1.5%, so the skip is bit-identical unconditionally rather than only in the
# common case. Output is unchanged for E0 -- where the flag never fires, since
# no real ladder entry reaches a 32767 divisor -- and unchanged for E1, checked
# by md5 of a saved m5 and m7 frame with CINEPI_BAND_Q=9=32767 on both sides of
# the change. Verified the same way as every re-pin above: all five gate paths
# still produce 72f44899 on 2988814 bytes (tools/verify.sh, this tree).
# Do not update this hash for a change that moves the CRC.
# Re-pinned 2026-08-19 for Component-Aware Quantisation.
#
# The previous pin was 5435ece3ec2a2a348709027272756d3525b31a6b76572d155693632fd2133b13,
# the v1.16.6 winning stack as shipped. CAQ is a deliberate encoder change -- per
# component quant tables derived at runtime from the SAME canonical m1..m10
# ladder -- so the canonical encoder moved and the pin moves with it. The gate is
# not bypassed: it goes on catching every subsequent unintended edit, which is
# what it is for.
#
# With CAQ off the encoder is bit-exact against the previous pin's behaviour, by
# construction: cpu_quant_tables_caq() returns four copies of the base table and
# per_channel_quant stays false, so both the quantiser and the header path take
# the identical arithmetic they took before.
# Re-pinned 2026-08-19: GPR->QRAW rename. Proven rename-only -- inverting the
# substitution map reproduces the previous pin 89ed1b06... byte for byte.
# No encoder logic changed.
# The same 2026-08-21 re-pin also added --noise-clean, --noise-clean-strength and
# --pixel-clean as CLI flags. tools/perf_ab.sh interleaves variants inside one
# loop and hands each variant's flags to the same binary, so a lever reachable
# only through the environment cannot be A/B'd the way this package measures
# everything else -- and the matrix the brief asks for is exactly that A/B.
#
# Re-pinned 2026-08-21 for the REVISED NOISE CLEAN, the production-shaped method
# in QRAW_Noise_Clean_Method_and_Recommendation: band-aware level-1 cleanup on a
# noise floor the encoder ESTIMATES FROM THE FRAME (sparse MAD over |HH1|, 1/16
# of the band) instead of from a per-ISO calibration table, applied one frame
# late so no statistic is ever on the critical path.
#
# A deliberate encoder change, so the pin moves. Previous pin was
# 999c874cae235fe594ab7ee98338429095cc9775877bca1d57da44917100729e.
#
# It rides the same per-band integer threshold Pixel Clean introduced -- whichever
# of the three levers is widest wins -- so the hot loop is STILL one compare per
# vector. The only new work per frame is the sparse sample: a scalar 6-tap
# recomputed for 1/16 of one band of one level, deliberately taken BEFORE the
# quantiser and before v2_direct_emit_row()'s register-direct path, because the
# production encoder never writes an int16 coefficient plane to DRAM and a
# post-quant sampler would have been silently inert on it -- which is precisely
# how the ISO prototype below managed to measure nothing across three A/B sweeps.
#
# With it OFF the encoder is bit-exact against the previous pin by construction:
# g_noise_clean_mode 0 makes noise_clean_resolve() zero the thresholds and return
# before touching a histogram, noise_clean_sample_row() returns on its first
# test, and the max() folds reduce to std::max(x, 0) == x.
#
# The ISO-calibrated prototype it supersedes is still in the file and still
# tested, now behind CINEPI_NOISE_CLEAN_ISO=1 so the two can never threshold the
# same frame by two different rules.
#
# Previously re-pinned 2026-08-20 for NOISE CLEAN: an ISO-calibrated pre-quant dead-zone,
# indexed by sensor gain from 48-frame lens-cap dark stacks. Independent lever,
# OFF by default, and it rides the SAME per-band integer threshold Pixel Clean
# introduced (whichever of the two is wider), so the hot loop is unchanged --
# still one compare per vector. With it off the thresholds are zero and every
# arithmetic path is the one it was before.
#
# Previously re-pinned for PIXEL CLEAN (twice: the second time for the
# divisor-1 fix below).
#
# cinepi_absquant_le1_threshold() originally bailed out at divisor <= 1 and
# returned 0. At divisor 1 the quantiser is the IDENTITY, so |quant(v)| <= 1
# still means |v| <= 1 and the RG/BG colour rule still applies -- and the m-mode
# ladder does reach a divisor of 1 on high-pass bands at the finest grades, so
# Pixel Clean was partially inert exactly there. Found by an exhaustive
# equivalence test of the fused threshold against the brief's literal two-rule
# form over every divisor 1..32767 and every coefficient +-16384
# (2,147,483,646 combinations, all agreeing after the fix).
#
# The previous pin was 5f48b57c719f11231865d87a3fce4a49f78d9fcca2ba3c421975825b0dde052a.
# Pixel Clean is a deliberate encoder change: a dead-zone at 125% of the normal
# half-step plus a post-quant rule forcing quantised RG/BG level-1 coefficients
# of +-1 to zero, both collapsed into ONE exact integer threshold per band
# because cpu_quantize_exact() is monotone in |coeff|. So the canonical encoder
# moved and the pin moves with it; the gate is not bypassed and goes on catching
# every subsequent unintended edit, which is what it is for.
#
# With Pixel Clean OFF the encoder is bit-exact against the previous pin by
# construction, not by testing: fused_level_init() leaves pc_t_* at 0,
# cinepi_make_quant() leaves cinepi_quant_neon::pc false for a threshold of 0,
# cpu_quantize_pc() falls through to cpu_quantize_exact(), and the widened
# provable-zero bounds reduce to std::max(x, 0) == x. Every arithmetic path is
# the one it was before.
# Re-pinned 2026-08-21 for public release. A three-line Apache-2.0 SPDX header
# was prepended to vc5_bench.cpp. NO encoder logic changed: the previous pin
# 6a75632afc96c902d53fcea52e14a36bdd1d4ff8cec5820e63d8137cc7b320cd
# is reproduced exactly by stripping those three lines:
#     tail -n +4 vc5_bench.cpp | sha256sum
# That was verified before this pin was written.
check_hash 9db02f1f3192cdbe0438b70f65470c9700adc0a8a682cfcb7ae9145c853df1b3 "$WIN"

# There must be exactly one canonical VC-5 benchmark source in the package.
mapfile -t vc5s < <(find "$ROOT" -type f -name vc5_bench.cpp -print)
[[ ${#vc5s[@]} -eq 1 && "${vc5s[0]}" == "$WIN" ]] || {
  echo "FATAL: more than one vc5_bench.cpp encoder source exists" >&2
  printf '  %s\n' "${vc5s[@]}" >&2
  exit 53
}

grep -Fq '#include "vc5_bench.cpp"' "$FACE" || {
  echo "FATAL: live encoder library no longer compiles the canonical winning source" >&2; exit 54; }
grep -Fq 'o.cpu_vle_prefetch_distance = 128;' "$FACE" || {
  echo "FATAL: live encoder wrapper is not locked to VLE128" >&2; exit 68; }
grep -Fq 'o.cpu_wavelet_fused = true;' "$FACE" || {
  echo "FATAL: live encoder wrapper is not locked to fused winner path" >&2; exit 69; }
grep -Fq 'no fallback encoder exists' "$FACE" || {
  echo "FATAL: live encoder wrapper is not fail-closed" >&2; exit 70; }
grep -Fq 'static constexpr const char* CINEPI_VERSION = "1.16.6";' "$WIN" || {
  echo "FATAL: canonical encoder does not identify as v1.16.6" >&2; exit 55; }

# Quant policy is also singular: Universal Standard Quant v3 only.
QREF="$ROOT/benchmark/validated_input/canonical_mode_quant_tables.json"
[[ -f "$QREF" ]] || { echo "FATAL: canonical quant table manifest missing" >&2; exit 71; }
[[ ! -e "$ROOT/benchmark/validated_input/mac_reference_mode_quant_tables.json" ]] || {
  echo "FATAL: legacy Mac/reference quant ladder is still present" >&2; exit 72; }
if grep -Fq -- '--mode-ladder' "$WIN"; then
  echo "FATAL: legacy selectable quant ladder remains in canonical encoder" >&2; exit 73
fi
grep -Fq '{"m1",  {1,4,4,2,8,8,6,26,26,39}' "$WIN" || {
  echo "FATAL: Universal Standard Quant v3 M1 not found" >&2; exit 74; }
grep -Fq '{"m10", {1,29,29,13,45,45,35,272,272,521}' "$WIN" || {
  echo "FATAL: Universal Standard Quant v3 M10 not found" >&2; exit 75; }

# Production live integration may wrap the encoder but may not define another one.
# integration/cinepi-raw lives in cinepi-qraw-alpha now; scanned there.
INTEGRATION_SCAN=""; [[ -d "$ROOT/integration/cinepi-raw" ]] && INTEGRATION_SCAN="$ROOT/integration/cinepi-raw"
if grep -RIn --exclude='vc5_bench.cpp' --exclude='check_single_encoder.sh' \
     -E 'static constexpr const char\* CINEPI_VERSION|CINEPI_VC5_BENCH_VERSION 1\.16\.[0-9]+' \
     "$ROOT/encoder_library" ${INTEGRATION_SCAN:+"$INTEGRATION_SCAN"} >/tmp/cinepi_alt_encoder.$$ 2>/dev/null; then
  cat /tmp/cinepi_alt_encoder.$$ >&2
  rm -f /tmp/cinepi_alt_encoder.$$
  echo "FATAL: alternate encoder version declaration found" >&2
  exit 56
fi
rm -f /tmp/cinepi_alt_encoder.$$

# No second encoder application or historical alternate runner may ship.
[[ ! -e "$ROOT/benchmark/third_party/gpr/source/app/vc5_encoder_app" ]] || {
  echo "FATAL: upstream standalone vc5_encoder_app is present" >&2; exit 57; }
for f in \
  "$ROOT/benchmark/cinepi_qraw_bench/tools_entropy_bench.cpp" \
  "$ROOT/benchmark/cinepi_qraw_bench/cpu_wavelet_lab" \
  "$ROOT/benchmark/cinepi_qraw_bench/shaders" \
  "$ROOT/benchmark/cinepi_qraw_bench/vulkan_lab" \
  "$ROOT/benchmark/RUN_M7_OPTIMISATION_SWEEP_60S.sh" \
  "$ROOT/benchmark/RUN_M7_30X60_FACTORIAL.sh" \
  "$ROOT/encoder_library/white_ab.c" \
  "$ROOT/benchmark/run" "$ROOT/benchmark/run_complete" "$ROOT/benchmark/run_experiment"; do
  [[ ! -e "$f" ]] || { echo "FATAL: alternate/historical encoder entry point present: $f" >&2; exit 58; }
done

# The generic SDK tool is decoder-only. It may not link or expose vc5_encoder.
SDK_TOOL="$ROOT/benchmark/third_party/gpr/source/app/cinepi_gpr_sdk/CMakeLists.txt"
grep -Fq 'add_definitions("-DGPR_WRITING=0")' "$SDK_TOOL" || {
  echo "FATAL: cinepi_gpr_sdk is not locked decode-only" >&2; exit 59; }
if grep -Eq 'include_directories\("\.\./\.\./lib/vc5_encoder"\)|GPR_WRITING=1' "$SDK_TOOL"; then
  echo "FATAL: cinepi_gpr_sdk exposes an encoder path" >&2; exit 60
fi

# The canonical app CMake must define exactly one executable: vc5_bench.
APP_CMAKE="$ROOT/benchmark/third_party/gpr/source/app/cinepi_vulkan_gpr_bench/CMakeLists.txt"
[[ $(grep -c '^[[:space:]]*add_executable(' "$APP_CMAKE") -eq 1 ]] || {
  echo "FATAL: canonical encoder CMake defines more than one executable" >&2; exit 61; }
grep -Fq '"${BENCH_DIR}/vc5_bench.cpp"' "$APP_CMAKE" || {
  echo "FATAL: canonical target is not built from vc5_bench.cpp" >&2; exit 62; }

# Root SDK graph may include the private vc5_encoder library because that is a
# required component INSIDE the winner, but it must never add its standalone app
# or the generic gpr_tools encoder CLI.
ROOT_CMAKE="$ROOT/benchmark/third_party/gpr/CMakeLists.txt"
if grep -Eq 'add_subdirectory\([[:space:]]*"source/app/(vc5_encoder_app|gpr_tools)"' "$ROOT_CMAKE"; then
  echo "FATAL: SDK graph exposes another encoder-capable app" >&2; exit 63
fi
[[ ! -e "$ROOT/benchmark/third_party/gpr/source/app/gpr_tools/CMakeLists.txt" ]] || {
  echo "FATAL: generic gpr_tools build target still exists" >&2; exit 65; }
if grep -Eq 'gpr_convert_(raw|dng)_to_gpr' "$ROOT/benchmark/third_party/gpr/source/app/gpr_tools/main_c.c"; then
  echo "FATAL: generic SDK CLI still contains a GPR encoder command" >&2; exit 66
fi
if grep -Eq 'gpr_convert_(raw|dng)_to_gpr|vc5_encoder_process\(' "$WIN" "$FACE"; then
  echo "FATAL: winner/wrapper references the SDK reference encode path" >&2; exit 67
fi

# Active production code/scripts may not identify another CinePi encoder version.
grep -RIn --binary-files=without-match \
     --exclude='*.md' --exclude='*SHA256SUMS*' --exclude='check_single_encoder.sh' \
     -E 'CINEPI_VC5_BENCH_VERSION 1\.16\.[0-9]+|CINEPI_QRAW_ENCODER=1\.16\.[0-9]+|CINEPI_VERSION = "1\.16\.[0-9]+' \
     "$ROOT" > /tmp/cinepi_versions.$$ 2>/dev/null || true
if grep -Ev '1\.16\.6' /tmp/cinepi_versions.$$ > /tmp/cinepi_alt_version.$$; then
  cat /tmp/cinepi_alt_version.$$ >&2
  rm -f /tmp/cinepi_versions.$$ /tmp/cinepi_alt_version.$$
  echo "FATAL: alternate CinePi encoder version remains in active package code" >&2
  exit 64
fi
rm -f /tmp/cinepi_versions.$$ /tmp/cinepi_alt_version.$$

printf 'SINGLE_ENCODER PASS id=1.16.6-winning-stack-irq-vle128 quant=universal-standard-v3 vc5_sha=%s\n' \
  72b589e0d853c3448e7a64caef925d47d4788265bc4ae99831cff4c9283d6d16
