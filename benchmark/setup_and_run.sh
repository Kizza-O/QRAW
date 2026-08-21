#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Kieran Olsson
# One-command setup for the CinePi VC-5 GP-LOG2 benchmark on Raspberry Pi 5.
#
# Usage:
#   ./setup_and_run.sh                 install deps + Mesa, build, standard benchmark
#   ./setup_and_run.sh --build-only    install deps + Mesa and build, do not benchmark
#   ./setup_and_run.sh --no-deps       skip apt packages
#   ./setup_and_run.sh --no-mesa-build skip private Mesa build/check
#   ./setup_and_run.sh --experiment    run optimisation lab after setup
set -Eeuo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
cd "$HERE"

DO_DEPS=1
DO_MESA=1
DO_RUN=1
MODE="candidate"
PASSTHRU=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-deps)       DO_DEPS=0 ;;
    --no-mesa-build) DO_MESA=0 ;;
    --build-only)    DO_RUN=0 ;;
    --experiment)    MODE="experiment" ;;
    --)              shift; PASSTHRU=("$@"); break ;;
    -h|--help)       sed -n '2,12p' "$0"; exit 0 ;;
    *)               PASSTHRU+=("$1") ;;
  esac
  shift
done

ARCH=$(uname -m)
echo "Launching supported single-encoder UI: ./RUN.sh"
# RUN.sh was the combined encoder+camera menu; the camera half now lives in
# the cinepi-qraw-alpha repository.
exec ./RUN_BENCHMARK.sh
