#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Kieran Olsson
set -Eeuo pipefail

if [[ $(uname -s) != Linux ]]; then echo "FATAL: Linux is required." >&2; exit 2; fi
ARCH=$(uname -m)
if [[ "$ARCH" != aarch64 && "$ARCH" != arm64 ]]; then
  echo "WARNING: expected 64-bit Raspberry Pi OS (aarch64); detected $ARCH." >&2
fi

if ! command -v apt-get >/dev/null 2>&1; then
  echo "FATAL: this installer supports Raspberry Pi OS/Debian systems using apt." >&2
  exit 2
fi

SUDO=()
if ((EUID != 0)); then
  command -v sudo >/dev/null 2>&1 || { echo "FATAL: sudo is required when not running as root." >&2; exit 2; }
  SUDO=(sudo)
fi

echo "== Installing CinePi and Mesa build dependencies =="
"${SUDO[@]}" apt-get update
"${SUDO[@]}" apt-get install -y --no-install-recommends \
  build-essential gcc g++ make cmake wget xz-utils pkg-config ninja-build bison flex \
  python3 python3-venv python3-pip python3-numpy \
  libvulkan1 mesa-vulkan-drivers vulkan-tools glslang-tools spirv-tools \
  libdrm-dev libexpat1-dev libudev-dev libzstd-dev libelf-dev libunwind-dev \
  procps coreutils findutils util-linux gawk grep sed

for tool in glslangValidator spirv-val spirv-opt vulkaninfo cmake ninja wget; do
  command -v "$tool" >/dev/null 2>&1 || { echo "FATAL: dependency was not installed: $tool" >&2; exit 2; }
done

if /usr/bin/python3 -c 'import numpy' >/dev/null 2>&1; then
  echo "NumPy: PASS (/usr/bin/python3)"
else
  echo "FATAL: python3-numpy installed but cannot be imported by /usr/bin/python3." >&2
  exit 2
fi

echo "== Dependency installation complete =="
echo "Next: ./check_pi_environment.sh && ./build_and_verify.sh"