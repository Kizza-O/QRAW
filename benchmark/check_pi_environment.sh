#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Kieran Olsson
# Check only prerequisites used by the canonical v1.16.6 CPU-GPR winner.
set -Eeuo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/.." && pwd)
fail=0
ok(){ echo "OK    $*"; }
bad(){ echo "FAIL  $*"; fail=1; }
for c in cmake g++ make python3 sha256sum; do command -v "$c" >/dev/null 2>&1 && ok "$c" || bad "missing $c"; done
[[ "$(uname -m)" == aarch64 ]] && ok "aarch64 target" || echo "NOTE  host validation on $(uname -m); production target is aarch64"
ldconfig -p 2>/dev/null | grep -q 'libvulkan.so' && ok "Vulkan loader (link dependency)" || bad "libvulkan loader missing"
[[ -f "$HERE/cinepi_qraw_bench/deps/vulkan-headers-1.3.290/vulkan/vulkan.h" ]] && ok "bundled Vulkan headers" || bad "bundled Vulkan headers missing"
"$ROOT/tools/check_single_encoder.sh" && ok "single canonical encoder gate" || bad "single encoder gate"
[[ $fail -eq 0 ]] || exit 1
echo "ENVIRONMENT PASS canonical=v1.16.6+irq-core0+vle128"
