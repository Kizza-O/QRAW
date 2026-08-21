#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Kieran Olsson
# The package version must agree with itself: directory name,
# PACKAGE_VERSION.txt and (if given) the zip filename and its top-level
# directory. This exists because the zip was twice named for a newer version
# than the directory inside it, and both times the person receiving it
# noticed before I did.
set -Eeuo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
NAME=$(basename "$ROOT")
ZIP=""; [[ "${1:-}" == "--zip" ]] && ZIP="${2:-}"
fail=0
note() { printf '  %-22s %s\n' "$1" "$2"; }
bad()  { printf '  %-22s %s   <-- MISMATCH\n' "$1" "$2"; fail=1; }
[[ "$NAME" =~ ^cinepi-qraw-live-v([0-9]+)\.([0-9]+)$ ]] || {
  echo "  directory '$NAME' is not cinepi-qraw-live-vX.Y" >&2; exit 2; }
MAJ="${BASH_REMATCH[1]}"; MIN="${BASH_REMATCH[2]}"; V="$MAJ.$MIN"
note "directory" "$NAME  -> $V"
PV=$(head -1 "$ROOT/benchmark/PACKAGE_VERSION.txt")
[[ "$PV" == "$V" || "$PV" == "$V-"* ]] && note "PACKAGE_VERSION.txt" "$PV" || bad "PACKAGE_VERSION.txt" "$PV (want $V or $V-...)"
if [[ -n "$ZIP" ]]; then
  ZB=$(basename "$ZIP")
  [[ "$ZB" == "cinepi-qraw-live-v${MAJ}_${MIN}.zip" || "$ZB" == "cinepi-qraw-live-v${MAJ}_${MIN}-"*.zip ]] &&
    note "zip filename" "$ZB" || bad "zip filename" "$ZB"
  if command -v unzip >/dev/null && [[ -f "$ZIP" ]]; then
    ZIP_LIST=$(unzip -Z1 "$ZIP")
    FIRST=${ZIP_LIST%%$'\n'*}
    TOP=${FIRST%%/*}
    [[ "$TOP" == "$NAME" ]] && note "zip top-level dir" "$TOP" || bad "zip top-level dir" "$TOP"
  fi
fi
[[ "$fail" -eq 0 ]] && echo "  version consistent: $V" || { echo "  VERSION MISMATCH" >&2; exit 1; }
