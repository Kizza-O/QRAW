#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Kieran Olsson
"""Generate the 12-bit GP-Log2 pre-companded twin of a 16-bit raw16 frame.

Why this file exists
--------------------
The IMX585 reads out 16-bit only up to ~30 fps; at up to 60 fps it delivers
12-bit. Benchmarking the high-frame-rate case from a 16-bit source therefore
measures a compand step the camera would never perform at that rate. This
produces the 12-bit signal the sensor would actually hand over, so both cases
can be measured against real data instead of simulated.

The curve is GP-Log2 -- log1p(599x)/log(600) -- reproduced EXACTLY as
make_lut() builds it in vc5_bench.cpp, including the endpoint pins. The test
that matters is bit-exactness: encoding this file with --input-companded on
must produce the identical container to encoding the 16-bit source with
companding on. Anything else means the curve drifted.
"""
import argparse, math, sys
import numpy as np

def gp_log2_lut(effective_bits=16, black=0, white=None,
                log_strength=599.0, working_max=4094):
    n = 1 << effective_bits
    if white is None:
        white = n - 1                     # vc5_bench.cpp:1150
    denom = math.log1p(log_strength)
    rng = float(white - black)
    i = np.arange(n, dtype=np.float64)
    x = np.clip((i - black) / rng, 0.0, 1.0)
    y = np.log1p(log_strength * x) / denom
    # llround is half-away-from-zero; numpy rint is half-to-even. Values are
    # positive here, so add a tiny bias the same way llround would.
    lut = np.floor(y * working_max + 0.5).astype(np.uint16)
    lut[0] = 0
    if white < n:
        lut[white] = working_max
    return lut

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src"); ap.add_argument("dst")
    ap.add_argument("--effective-bits", type=int, default=16)
    ap.add_argument("--working-max", type=int, default=4094)
    ap.add_argument("--log-strength", type=float, default=599.0)
    a = ap.parse_args()
    lut = gp_log2_lut(a.effective_bits, 0, None, a.log_strength, a.working_max)
    src = np.fromfile(a.src, dtype=np.uint16)
    if src.size == 0:
        print("FATAL: source is empty", file=sys.stderr); return 2
    np.take(lut, src).astype(np.uint16).tofile(a.dst)
    out = np.fromfile(a.dst, dtype=np.uint16)
    print(f"wrote {a.dst}: {out.size:,} samples, min={out.min()} max={out.max()} "
          f"mean={out.mean():.1f}  (source max={src.max()})")
    return 0

if __name__ == "__main__":
    sys.exit(main())
