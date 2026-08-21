#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Kieran Olsson
set -Eeuo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
exec "$HERE/check_single_encoder.sh"
