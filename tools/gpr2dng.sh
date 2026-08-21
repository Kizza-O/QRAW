#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Kieran Olsson
# gpr2dng.sh -- convert recorded .gpr to a DNG that Adobe reads correctly.
#
# WHY THIS EXISTS
#
# Lightroom / Camera Raw DO read .gpr natively (ACR 9.7+): a .gpr is DNG plus
# VC-5. Adobe's vendored DNG SDK 1.4 in this tree cannot -- it has no read case
# for compression code 9 -- but shipping ACR is not that SDK, so do not conclude
# anything about Lightroom from dng_validate refusing a .gpr.
#
# Converting is still needed for COLOUR correctness: the container carries no
# LinearizationTable (GoPro's do not either, and adding one made Lightroom
# render the file distorted), so a .gpr opened directly is viewable but not
# linearised. This produces a DNG that is.
#
# The conversion REGENERATES the LinearizationTable from the environment rather
# than preserving the one in the source file. Convert a gradation-companded
# ClearHDR capture with the environment unset and it silently writes a GP-Log2
# table instead -- verified: Adobe then mapped stored code 1000 to 413 where
# the correct value is 2308. That is a wrong, not merely imprecise, image. This
# wrapper defaults to the correct mode so it cannot be forgotten.
#
# Usage: gpr2dng.sh [--legacy] <in.gpr> [out.dng]
#        gpr2dng.sh [--legacy] <dir>      # every .gpr in the directory
set -Eeuo pipefail

LEGACY=0
if [[ "${1:-}" == "--legacy" ]]; then LEGACY=1; shift; fi

HERE=$(cd "$(dirname "$0")" && pwd)
GPR_TOOLS=${GPR_TOOLS:-$HERE/gpr_tools}
[[ -x "$GPR_TOOLS" ]] || { echo "FATAL: gpr_tools not found at $GPR_TOOLS" >&2
                           echo "       set GPR_TOOLS=/path/to/gpr_tools" >&2; exit 2; }
command -v exiftool >/dev/null || { echo "FATAL: exiftool required" >&2; exit 2; }

convert_one() {
  local in=$1 out=$2 mode

  # The container deliberately carries NO LinearizationTable (see below), so
  # the curve cannot be detected from the file and has to be stated. Default is
  # the current shipping encoding: gradation-companded ClearHDR, compand LUT
  # driven to an identity. Use --legacy for containers written before that, or
  # set CINEPI_GRADATION_LINEARIZATION explicitly.
  if [[ -n "${CINEPI_GRADATION_LINEARIZATION:-}" ]]; then
    mode=$CINEPI_GRADATION_LINEARIZATION
  elif [[ "$LEGACY" == 1 ]]; then
    mode=0
  else
    mode=2
  fi

  CINEPI_GRADATION_LINEARIZATION=$mode "$GPR_TOOLS" -i "$in" -o "$out" >/dev/null
  local blk wht
  blk=$(exiftool -s3 -BlackLevel "$out" 2>/dev/null)
  wht=$(exiftool -s3 -WhiteLevel "$out" 2>/dev/null)
  printf '  %-46s -> %-46s  mode=%s black=%s white=%s\n' \
         "$(basename "$in")" "$(basename "$out")" "$mode" "$blk" "$wht"
}

[[ $# -ge 1 ]] || { echo "usage: $0 <in.gpr|dir> [out.dng]" >&2; exit 2; }

if [[ -d "$1" ]]; then
  shopt -s nullglob
  found=0
  for f in "$1"/*.gpr "$1"/**/*.gpr; do
    convert_one "$f" "${f%.gpr}.dng"; found=1
  done
  (( found )) || { echo "no .gpr files under $1" >&2; exit 1; }
else
  convert_one "$1" "${2:-${1%.gpr}.dng}"
fi
