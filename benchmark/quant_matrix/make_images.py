#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Kieran Olsson
"""
make_images.py -- turn decoded GPR frames into viewable PNGs for quality review.

The SDK's own RGB path (gpr_convert_gpr_to_rgb) refuses these files with
"Could not decode input vc5 bitstream. Error number 20", though
gpr_convert_gpr_to_raw decodes them fine -- so we take the raw and demosaic here.

Input is what gpr_decode_verify --dump-raw emits: 3840x2160 GBRG Bayer, 12-bit
values (0..4095) in a uint16 container, still in the GP-Log2 companded domain.

Outputs per frame:
  <name>.png        full-resolution demosaiced RGB
  <name>_crop.png   200% centre crop, where fine-detail loss is actually visible
  <name>_diff.png   amplified absolute difference vs the E0 reference, if given

The crop matters: this matrix prunes the FINEST wavelet detail bands, and a
full-frame view downscaled onto a screen hides exactly the damage being judged.
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from PIL import Image

W, H = 3840, 2160
CROP = 512          # centre crop side, in sensor pixels, before 2x upscale


def load_raw(path: Path) -> np.ndarray:
    a = np.fromfile(path, dtype="<u2")
    if a.size != W * H:
        raise SystemExit(f"{path.name}: expected {W*H} samples, got {a.size}")
    return a.reshape(H, W).astype(np.float32)


def demosaic_gbrg(bayer: np.ndarray) -> np.ndarray:
    """Bilinear demosaic of a GBRG mosaic.

        row 0:  G B G B
        row 1:  R G R G
    """
    g = np.zeros_like(bayer)
    r = np.zeros_like(bayer)
    b = np.zeros_like(bayer)

    g[0::2, 0::2] = bayer[0::2, 0::2]      # G on even row, even col
    g[1::2, 1::2] = bayer[1::2, 1::2]      # G on odd row, odd col
    b[0::2, 1::2] = bayer[0::2, 1::2]      # B on even row, odd col
    r[1::2, 0::2] = bayer[1::2, 0::2]      # R on odd row, even col

    def fill(ch: np.ndarray, mask: np.ndarray) -> np.ndarray:
        """Normalised box interpolation over the known samples."""
        from numpy.lib.stride_tricks import sliding_window_view
        pad_v = np.pad(ch, 1, mode="edge")
        pad_m = np.pad(mask.astype(np.float32), 1, mode="edge")
        num = sliding_window_view(pad_v, (3, 3)).sum(axis=(-1, -2))
        den = sliding_window_view(pad_m, (3, 3)).sum(axis=(-1, -2))
        out = np.where(den > 0, num / np.maximum(den, 1e-6), 0.0)
        return np.where(mask, ch, out)

    gm = np.zeros((H, W), bool); gm[0::2, 0::2] = True; gm[1::2, 1::2] = True
    rm = np.zeros((H, W), bool); rm[1::2, 0::2] = True
    bm = np.zeros((H, W), bool); bm[0::2, 1::2] = True

    return np.stack([fill(r, rm), fill(g, gm), fill(b, bm)], axis=-1)


def to_srgb8(rgb: np.ndarray, wb: np.ndarray) -> np.ndarray:
    """The data is GP-Log2 companded, so it is already perceptually coded.
    A fixed white balance + mild gain and gamma make it a sane screen image.
    wb is computed ONCE from the E0 reference and passed in unchanged for
    every frame, so no candidate is flattered by its own colour statistics."""
    x = np.clip(rgb / 4095.0, 0.0, 1.0) * wb
    x = np.clip(x * 1.35, 0.0, 1.0) ** (1.0 / 1.6)
    return (x * 255.0 + 0.5).astype(np.uint8)


def grey_world(rgb: np.ndarray) -> np.ndarray:
    m = rgb.reshape(-1, 3).mean(axis=0)
    return (m.mean() / np.maximum(m, 1e-6)).astype(np.float32)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("outdir")
    ap.add_argument("raws", nargs="+")
    ap.add_argument("--reference", help="E0 raw16 for the difference maps")
    args = ap.parse_args()

    out = Path(args.outdir)
    out.mkdir(parents=True, exist_ok=True)
    ref = load_raw(Path(args.reference)) if args.reference else None
    y0, x0 = (H - CROP) // 2, (W - CROP) // 2

    # One white balance for the whole set, from the reference when there is one.
    wb = grey_world(demosaic_gbrg(ref if ref is not None else load_raw(Path(args.raws[0]))))
    print(f"white balance (fixed for every frame): {wb.round(3).tolist()}")

    for p in args.raws:
        path = Path(p)
        name = path.stem
        bayer = load_raw(path)
        rgb8 = to_srgb8(demosaic_gbrg(bayer), wb)
        Image.fromarray(rgb8).save(out / f"{name}.png")

        crop = rgb8[y0:y0 + CROP, x0:x0 + CROP]
        Image.fromarray(crop).resize((CROP * 2, CROP * 2), Image.NEAREST) \
             .save(out / f"{name}_crop.png")

        if ref is not None:
            d = np.abs(bayer - ref)
            amp = np.clip(d * 12.0, 0, 255).astype(np.uint8)
            Image.fromarray(amp).save(out / f"{name}_diff.png")
            print(f"{name}: png + crop + diff  (max|d|={d.max():.0f} "
                  f"mean|d|={d.mean():.2f} codes of 4095)")
        else:
            print(f"{name}: png + crop")


if __name__ == "__main__":
    main()
