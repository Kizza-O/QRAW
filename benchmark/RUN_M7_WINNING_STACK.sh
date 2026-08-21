#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Kieran Olsson
# Fixed proof run for the canonical production winner. No search or A/B logic.
set -Eeuo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
export MODES=m7
export THREADS=${THREADS:-4}
export DURATION=${DURATION:-60}
export PASSES=${PASSES:-3}
export INPUT_BITS=${INPUT_BITS:-12}
export VERIFY=${VERIFY:-off}
export SAVE_QRAW=${SAVE_QRAW:-off}
echo "CinePi M7 canonical winner proof: v1.16.6 + IRQ->Core0 + VLE128"
exec "$HERE/RUN_BENCHMARK.sh"
