#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Kieran Olsson
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
OUT=${1:-$HERE/qraw_stats_cli}
CXX=${CXX:-g++}
"$CXX" -std=c++17 -O2 -Wall -Wextra -pthread "$HERE/qraw_stats_cli.cpp" -o "$OUT"
echo "$OUT"
