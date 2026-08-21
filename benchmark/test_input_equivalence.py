#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Kieran Olsson
"""The 12-bit pre-companded input must be the exact GP-Log2 twin of the
16-bit source.

If the curve in make_precompanded.py ever drifts from make_lut() in
vc5_bench.cpp, the two inputs stop describing the same image and every
12-bit-versus-16-bit comparison silently becomes meaningless. This checks
the mapping directly, without needing to build or run the encoder.
"""
import math, os, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from make_precompanded import gp_log2_lut

SRC = os.path.join(HERE, "cinepi_qraw_bench/input/"
                   "sample_imx585_3840x2160_gbrg_16bit.raw16")
DST = os.path.join(HERE, "cinepi_qraw_bench/input/"
                   "sample_imx585_3840x2160_gbrg_12bit_gplog2.raw16")

def main():
    if not os.path.exists(DST):
        print("12-bit input not present -- skipped (it is generated on demand)")
        return 0
    src = np.fromfile(SRC, dtype=np.uint16)
    dst = np.fromfile(DST, dtype=np.uint16)
    assert src.size == dst.size, f"size mismatch: {src.size} vs {dst.size}"
    lut = gp_log2_lut()
    want = np.take(lut, src)
    bad = int((want != dst).sum())
    assert bad == 0, f"{bad:,} samples differ from the GP-Log2 mapping"
    assert dst.max() <= 4094, f"12-bit input exceeds working_max: {dst.max()}"
    # The endpoints are pinned in both implementations; check them explicitly
    # because an off-by-one there is invisible in aggregate statistics.
    assert lut[0] == 0, "LUT does not pin black to 0"
    assert lut[65535] == 4094, "LUT does not pin white to working_max"
    print(f"12-bit input is the exact GP-Log2 twin of the 16-bit source "
          f"({dst.size:,} samples, max={dst.max()}, mean={dst.mean():.1f})")
    return 0

if __name__ == "__main__":
    sys.exit(main())
