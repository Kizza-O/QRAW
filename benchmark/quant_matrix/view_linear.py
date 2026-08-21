#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Kieran Olsson
"""
view_linear.py -- render a decoded ClearHDR RAW16 frame for viewing.

The live 16-bit ClearHDR stream is LINEAR sensor data, unlike the static
reference frame which is GP-Log2 companded. Applying the companded viewer to
it is what makes it look blown out: same bytes, wrong transfer function.

Input is what gpr_decode_verify --dump-raw emits: 3840x2160 GBRG Bayer,
12-bit values in a uint16 container, linear.

Transform: grey-world white balance -> exposure normalise on a high percentile
(the scene is dark, so a fixed /4095 would render nearly black) -> sRGB-ish
gamma. Exposure and WB are computed ONCE from the reference frame and reused
for every frame, so a set stays comparable.
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from PIL import Image

W, H = 3840, 2160
CROP = 512


def load(p: Path) -> np.ndarray:
    a = np.fromfile(p, dtype="<u2")
    if a.size != W * H:
        raise SystemExit(f"{p.name}: expected {W*H} samples, got {a.size}")
    return a.reshape(H, W).astype(np.float32)


def demosaic_gbrg(bayer: np.ndarray) -> np.ndarray:
    from numpy.lib.stride_tricks import sliding_window_view
    g = np.zeros_like(bayer); r = np.zeros_like(bayer); b = np.zeros_like(bayer)
    g[0::2, 0::2] = bayer[0::2, 0::2]
    g[1::2, 1::2] = bayer[1::2, 1::2]
    b[0::2, 1::2] = bayer[0::2, 1::2]
    r[1::2, 0::2] = bayer[1::2, 0::2]

    def fill(ch, mask):
        pv = np.pad(ch, 1, mode="edge")
        pm = np.pad(mask.astype(np.float32), 1, mode="edge")
        num = sliding_window_view(pv, (3, 3)).sum(axis=(-1, -2))
        den = sliding_window_view(pm, (3, 3)).sum(axis=(-1, -2))
        return np.where(mask, ch, num / np.maximum(den, 1e-6))

    gm = np.zeros((H, W), bool); gm[0::2, 0::2] = True; gm[1::2, 1::2] = True
    rm = np.zeros((H, W), bool); rm[1::2, 0::2] = True
    bm = np.zeros((H, W), bool); bm[0::2, 1::2] = True
    return np.stack([fill(r, rm), fill(g, gm), fill(b, bm)], axis=-1)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("outdir")
    ap.add_argument("raws", nargs="+")
    ap.add_argument("--black", type=float, default=0.0)
    args = ap.parse_args()
    out = Path(args.outdir); out.mkdir(parents=True, exist_ok=True)

    ref = demosaic_gbrg(load(Path(args.raws[0])))
    m = ref.reshape(-1, 3).mean(axis=0)
    wb = (m.mean() / np.maximum(m, 1e-6)).astype(np.float32)
    # Exposure from a high percentile of the WB'd reference, so highlights land
    # near white instead of the frame rendering almost black.
    scale = float(np.percentile(ref * wb, 99.5))
    print(f"white balance {wb.round(3).tolist()}   exposure scale {scale:.0f} (linear codes -> 1.0)")

    y0, x0 = (H - CROP) // 2, (W - CROP) // 2
    for p in args.raws:
        path = Path(p)
        rgb = demosaic_gbrg(load(path))
        x = np.clip((rgb - args.black) * wb / max(scale - args.black, 1e-6), 0.0, 1.0)
        x = x ** (1.0 / 2.2)                       # linear -> display gamma
        img = (x * 255.0 + 0.5).astype(np.uint8)
        Image.fromarray(img).save(out / f"{path.stem}.png")
        crop = img[y0:y0 + CROP, x0:x0 + CROP]
        Image.fromarray(crop).resize((CROP * 2, CROP * 2), Image.NEAREST) \
             .save(out / f"{path.stem}_crop.png")
        print(f"  {path.stem}: png + crop")


if __name__ == "__main__":
    main()
