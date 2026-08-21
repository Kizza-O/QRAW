#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Kieran Olsson
"""
stageB.py -- build combined quant candidates from the Stage A gradients.

WHY NOT A GRID. Stage A measured every band independently. Two of the three
quantities that matter are exact functions of the quant table and carry no
measurement noise at all: gpr_bytes and rmse. fps is the noisy one (a single
static cell reproduces to only ~4-8% on this box). So candidates are RANKED on
the noise-free axes and fps is verified later, on a short list, with replication.

THE METHOD is Lagrangian rate-distortion, which is the standard way to pick a
quantiser set and is exactly the "best quality for a given speed" question:

    for a weight L, each band independently picks the q minimising
        bytes(q) + L * mse(q)
    sweeping L traces the optimal frontier. No point off that frontier can be
    better in both bytes and distortion than the point on it.

This works because quantisation error in different wavelet subbands is
approximately additive in MSE (the subbands are near-orthogonal), so total
distortion is the sum of per-band distortions. That assumption is ASSUMED here
and MEASURED in the output: every generated candidate is encoded for real and
its actual rmse compared against the predicted sum. If they diverge the
prediction is wrong and the measured value is what counts.
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
MASTER = HERE / "master.csv"
CAND = HERE / "candidates.json"

BASE_M7 = [1, 14, 14, 8, 26, 26, 18, 126, 126, 175]
BAND_NAME = {1: "L3-LH", 2: "L3-HL", 3: "L3-HH", 4: "L2-LH", 5: "L2-HL",
             6: "L2-HH", 7: "L1-LH", 8: "L1-HL", 9: "L1-HH"}


def load_stage_a() -> tuple[dict[int, list[dict]], float, int]:
    with MASTER.open() as fh:
        rows = [r for r in csv.DictReader(fh) if r.get("fps")]
    ctl = [r for r in rows if r["cell"].startswith("A_ctl_") and r.get("rmse")]
    base_rmse = float(ctl[0]["rmse"])
    base_bytes = int(ctl[0]["gpr_bytes"])

    per: dict[int, list[dict]] = {b: [] for b in range(1, 10)}
    for r in rows:
        if not r["cell"].startswith("A_b") or "_rm" in r["cell"]:
            continue
        if not r.get("rmse") or not r.get("gpr_bytes"):
            continue
        band = int(r["cell"].split("_")[1][1:])
        q = int(r["cell"].split("_q")[1][:5])
        rmse = float(r["rmse"])
        per[band].append({
            "q": q,
            "bytes": int(r["gpr_bytes"]),
            "d_bytes": base_bytes - int(r["gpr_bytes"]),       # saved
            "d_mse": max(0.0, rmse ** 2 - base_rmse ** 2),     # added
        })
    for b in per:
        per[b].insert(0, {"q": BASE_M7[b], "bytes": base_bytes,
                          "d_bytes": 0, "d_mse": 0.0})
        per[b].sort(key=lambda p: p["q"])
    return per, base_rmse, base_bytes


def frontier(per, base_rmse, base_bytes, lambdas) -> list[dict]:
    """For each L, every band picks the q minimising (bytes + L*mse)."""
    out, seen = [], set()
    for L in lambdas:
        table = list(BASE_M7)
        pred_saved, pred_mse = 0, 0.0
        for b in range(1, 10):
            best = min(per[b], key=lambda p: -p["d_bytes"] + L * p["d_mse"])
            table[b] = best["q"]
            pred_saved += best["d_bytes"]
            pred_mse += best["d_mse"]
        key = tuple(table)
        if key in seen:
            continue
        seen.add(key)
        out.append({
            "lambda": L,
            "table": table,
            "overrides": {b: table[b] for b in range(1, 10) if table[b] != BASE_M7[b]},
            "pred_bytes": base_bytes - pred_saved,
            "pred_rmse": (base_rmse ** 2 + pred_mse) ** 0.5,
        })
    return out


def structured() -> list[dict]:
    """Interpretable families, so the frontier has something to be judged against."""
    out = []
    for name, bands in (("L1", (7, 8, 9)), ("L2", (4, 5, 6)), ("L3", (1, 2, 3)),
                        ("HH", (3, 6, 9)), ("LHHL", (1, 2, 4, 5, 7, 8))):
        for mult in (1.5, 2, 3, 4):
            ov = {b: max(1, int(round(BASE_M7[b] * mult))) for b in bands}
            t = list(BASE_M7)
            for b, q in ov.items():
                t[b] = q
            out.append({"lambda": None, "family": f"{name}x{mult}", "table": t,
                        "overrides": ov, "pred_bytes": None, "pred_rmse": None})
    # HH1 pruned (the shipped E1) plus a scaled remainder, the campaign's own
    # best quality-changing candidate, so the frontier is measured against it.
    for mult in (1.0, 1.5, 2.0):
        ov = {9: 32767}
        if mult > 1.0:
            for b in (7, 8):
                ov[b] = int(round(BASE_M7[b] * mult))
        t = list(BASE_M7)
        for b, q in ov.items():
            t[b] = q
        out.append({"lambda": None, "family": f"E1+L1x{mult}", "table": t,
                    "overrides": ov, "pred_bytes": None, "pred_rmse": None})
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--lambdas", type=int, default=24)
    args = ap.parse_args()

    per, base_rmse, base_bytes = load_stage_a()
    print(f"E0 base: bytes={base_bytes} rmse={base_rmse}")
    print(f"Stage A levels per band: "
          f"{ {b: len(per[b]) for b in per} }")

    lam = [10.0 ** (-2.0 + 4.0 * i / (args.lambdas - 1)) for i in range(args.lambdas)]
    front = frontier(per, base_rmse, base_bytes, lam)
    cands = front + structured()
    for i, c in enumerate(cands):
        c["cell"] = f"B_{i:03d}"
    CAND.write_text(json.dumps(cands, indent=1))

    print(f"\n{len(front)} RD-frontier points + {len(cands)-len(front)} structured "
          f"= {len(cands)} candidates -> {CAND.name}")
    print(f"\n{'cell':<7}{'kind':<12}{'pred bytes':>11}{'pred rmse':>10}  table")
    for c in cands:
        kind = c.get("family") or f"L={c['lambda']:.3g}"
        pb = f"{c['pred_bytes']}" if c["pred_bytes"] else "-"
        pr = f"{c['pred_rmse']:.1f}" if c["pred_rmse"] else "-"
        print(f"{c['cell']:<7}{kind:<12}{pb:>11}{pr:>10}  "
              f"{'/'.join(map(str, c['table']))}")


if __name__ == "__main__":
    main()
