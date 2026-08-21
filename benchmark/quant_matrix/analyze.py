#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Kieran Olsson
"""
analyze.py -- turn the Stage A sweep into a per-band sensitivity table.

Two corrections matter here:

1. Thermal / machine-state drift. The E0 control is re-measured every 10
   cells; fps is normalised against a piecewise-linear fit through those
   controls, so a cell measured during a warm patch is not credited with a
   speedup it did not earn. This is the drift the campaign's own conclusion
   doc calls the most reusable lesson in the project.

2. Quality is reported as RMSE in 12-bit code values (0..4095), the domain
   the decoder actually emits, plus a true 12-bit-peak PSNR. gpr_decode_verify
   prints PSNR against a 65535 peak, which is a flat +24.08 dB.

Efficiency is dFPS per unit of added RMSE: how much speed a band buys per
unit of image error it destroys. Higher is better.
"""
from __future__ import annotations

import csv
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
MASTER = HERE / "master.csv"

BAND_NAME = {1: "L3-LH", 2: "L3-HL", 3: "L3-HH",
             4: "L2-LH", 5: "L2-HL", 6: "L2-HH",
             7: "L1-LH", 8: "L1-HL", 9: "L1-HH"}
BASE_M7 = [1, 14, 14, 8, 26, 26, 18, 126, 126, 175]


def load() -> list[dict]:
    with MASTER.open() as fh:
        rows = [r for r in csv.DictReader(fh) if r.get("fps")]
    for i, r in enumerate(rows):
        r["_i"] = i
        r["_fps"] = float(r["fps"])
        r["_rmse"] = float(r["rmse"]) if r.get("rmse") else None
        r["_bytes"] = int(r["gpr_bytes"]) if r.get("gpr_bytes") else None
    return rows


def control_curve(rows: list[dict]):
    ctl = [(r["_i"], r["_fps"]) for r in rows if r["cell"].startswith("A_ctl_")]
    if not ctl:
        raise SystemExit("no controls in master.csv")
    if len(ctl) == 1:
        return lambda i: ctl[0][1]

    def f(i: int) -> float:
        if i <= ctl[0][0]:
            return ctl[0][1]
        if i >= ctl[-1][0]:
            return ctl[-1][1]
        for (x0, y0), (x1, y1) in zip(ctl, ctl[1:]):
            if x0 <= i <= x1:
                t = (i - x0) / (x1 - x0) if x1 != x0 else 0.0
                return y0 + t * (y1 - y0)
        return ctl[-1][1]
    return f


def main() -> None:
    rows = load()
    curve = control_curve(rows)
    ctl_rows = [r for r in rows if r["cell"].startswith("A_ctl_")]
    ctl_fps = sum(r["_fps"] for r in ctl_rows) / len(ctl_rows)
    ctl_rmse = next((r["_rmse"] for r in ctl_rows if r["_rmse"]), None)
    ctl_bytes = next((r["_bytes"] for r in ctl_rows if r["_bytes"]), None)

    print(f"E0 control: fps {ctl_fps:.2f} (n={len(ctl_rows)}, "
          f"{min(r['_fps'] for r in ctl_rows):.2f}-{max(r['_fps'] for r in ctl_rows):.2f})"
          f"  rmse {ctl_rmse}  bytes {ctl_bytes}")
    print()

    cells = []
    for r in rows:
        if not r["cell"].startswith("A_b"):
            continue
        band = int(r["cell"].split("_")[1][1:])
        q = int(r["cell"].split("_q")[1][:5])
        norm = r["_fps"] * (ctl_fps / curve(r["_i"]))
        d_fps = norm - ctl_fps
        d_rmse = (r["_rmse"] - ctl_rmse) if (r["_rmse"] and ctl_rmse) else None
        d_bytes = (r["_bytes"] - ctl_bytes) / ctl_bytes * 100 if (r["_bytes"] and ctl_bytes) else None
        eff = (d_fps / d_rmse) if (d_rmse and d_rmse > 0.01) else None
        cells.append(dict(band=band, q=q, fps=r["_fps"], norm=norm, d_fps=d_fps,
                          rmse=r["_rmse"], d_rmse=d_rmse, d_bytes=d_bytes,
                          eff=eff, psnr=r.get("psnr12")))

    hdr = (f"{'band':<8}{'name':<8}{'q':>7}{'x':>6}{'fps':>8}{'norm':>8}"
           f"{'dFPS':>8}{'dFPS%':>8}{'rmse':>8}{'dRMSE':>8}{'dBytes%':>9}{'eff':>8}")
    for band in sorted({c["band"] for c in cells}):
        print(f"--- band {band}  {BAND_NAME[band]}  (m7 base {BASE_M7[band]}) ---")
        print(hdr)
        for c in sorted((c for c in cells if c["band"] == band), key=lambda c: c["q"]):
            mult = c["q"] / BASE_M7[band]
            print(f"{c['band']:<8}{BAND_NAME[c['band']]:<8}{c['q']:>7}"
                  f"{mult:>6.1f}{c['fps']:>8.2f}{c['norm']:>8.2f}"
                  f"{c['d_fps']:>8.2f}{c['d_fps']/ctl_fps*100:>8.1f}"
                  f"{(c['rmse'] or 0):>8.2f}"
                  f"{(c['d_rmse'] if c['d_rmse'] is not None else 0):>8.2f}"
                  f"{(c['d_bytes'] if c['d_bytes'] is not None else 0):>9.1f}"
                  f"{(('%.2f' % c['eff']) if c['eff'] else '-'):>8}")
        print()

    print("=== band ranking by best efficiency (dFPS per unit RMSE) ===")
    best = {}
    for c in cells:
        if c["eff"] is None:
            continue
        if c["band"] not in best or c["eff"] > best[c["band"]]["eff"]:
            best[c["band"]] = c
    print(f"{'band':<8}{'name':<8}{'best q':>8}{'x':>6}{'dFPS':>8}{'dRMSE':>8}{'eff':>8}")
    for c in sorted(best.values(), key=lambda c: -c["eff"]):
        print(f"{c['band']:<8}{BAND_NAME[c['band']]:<8}{c['q']:>8}"
              f"{c['q']/BASE_M7[c['band']]:>6.1f}{c['d_fps']:>8.2f}"
              f"{c['d_rmse']:>8.2f}{c['eff']:>8.2f}")

    print()
    print("=== cells reaching a static-fps target ===")
    for target in (60, 63, 65):
        hits = [c for c in cells if c["norm"] >= target]
        hits.sort(key=lambda c: c["d_rmse"] or 1e9)
        tag = f"static >= {target} fps"
        if not hits:
            print(f"{tag}: none from a single band")
        else:
            print(f"{tag}: {len(hits)} cells, cheapest quality cost:")
            for c in hits[:4]:
                print(f"    band {c['band']} {BAND_NAME[c['band']]:<7} q={c['q']:<6} "
                      f"norm={c['norm']:.2f}  dRMSE={c['d_rmse']:.2f}  "
                      f"dBytes={c['d_bytes']:.1f}%")


if __name__ == "__main__":
    main()
