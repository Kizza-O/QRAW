#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Kieran Olsson
set -Eeuo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/.." && pwd)
OUT=${1:-"$ROOT/../cinepi-qraw-live-single-winner.zip"}
"$ROOT/tools/check_single_encoder.sh" >/dev/null
rm -f "$OUT"
( cd "$ROOT/.." && zip -qr "$OUT" "$(basename "$ROOT")" \
    # output/ and archive/ are this machine's run history and superseded copies:
    # neither belongs in a distributable package.
    -x '*/build/*' '*/results/*' '*/output/*' '*/archive/*' \
       '*/.git/*' '*/__pycache__/*' )
echo "$OUT"
