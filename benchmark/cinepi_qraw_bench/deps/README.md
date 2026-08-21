# Bundled build dependencies

## vulkan-headers-1.3.290/

Vulkan and Vulkan Video headers from the Khronos Group.

    Copyright 2015-2024 The Khronos Group Inc.
    SPDX-License-Identifier: Apache-2.0

Vendored so the benchmark builds on a Pi with no `libvulkan-dev` installed.
Unmodified. Referenced by `benchmark/build_camera_modes.sh`,
`benchmark/check_pi_environment.sh`, `encoder_library/build_qraw_core.sh` and
`tools/verify.sh`; if you switch to the system headers, update all four.
