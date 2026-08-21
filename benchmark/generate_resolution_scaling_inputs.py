#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Kieran Olsson
from pathlib import Path
import argparse, numpy as np
ap=argparse.ArgumentParser(); ap.add_argument('--source',required=True); ap.add_argument('--output-dir',required=True); a=ap.parse_args()
src=np.fromfile(a.source,dtype='<u2')
W,H=3840,2160
if src.size!=W*H: raise SystemExit(f'expected {W*H} samples, got {src.size}')
src=src.reshape(H,W)
out=Path(a.output_dir); out.mkdir(parents=True,exist_ok=True)
for w,h in [(1920,1088),(960,544),(480,272)]:
    # Top-left Bayer-aligned crop. Both dimensions are even and divisible by 16.
    p=out/f'sample_{w}x{h}.raw16'; src[:h,:w].astype('<u2',copy=False).tofile(p); print(f'{w}x{h}|{p}')
