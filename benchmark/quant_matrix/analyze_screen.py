#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Kieran Olsson
"""
analyze_screen.py -- find the best quant tables in the screened matrix.

The question is "best quality at 48 fps", which is a constrained optimisation:
among tables that encode to a given size (speed), which has the lowest error?
That is the Pareto frontier of (gpr_bytes, rmse), and everything off it is
strictly dominated -- some other table is both smaller AND cleaner.

The shipped ladder m1..m10 is ten points in this space. The matrix contains
those ten plus 789 others, so the useful output is not "which table is best"
in the abstract but "how much better than the shipped rung of the same speed".

fps is predicted here only to pick a short list. Encode time per frame is
modelled as t = c0 + c1*bytes -- a fixed wavelet cost plus an entropy cost that
scales with output -- fitted on the Stage A runs. The campaign's own profiling
supports that shape (entropy ~2/3 fixed). Predictions are for TRIAGE ONLY;
every short-listed table is then measured for real.
"""
from __future__ import annotations

import argparse
import csv
from pathlib import Path

HERE = Path(__file__).resolve().parent
SCREEN = HERE / "screen.csv"
MASTER = HERE / "master.csv"
SHORTLIST = HERE / "shortlist.json"

LADDER = {
    "m1": [1, 4, 4, 2, 8, 8, 6, 26, 26, 39],
    "m2": [1, 4, 4, 2, 8, 8, 6, 33, 33, 50],
    "m3": [1, 4, 4, 2, 10, 10, 6, 43, 43, 66],
    "m4": [1, 6, 6, 4, 14, 14, 10, 55, 55, 87],
    "m5": [1, 8, 8, 4, 16, 16, 11, 73, 73, 111],
    "m6": [1, 10, 10, 6, 20, 20, 14, 93, 93, 143],
    "m7": [1, 14, 14, 8, 26, 26, 18, 126, 126, 175],
    "m8": [1, 18, 18, 9, 34, 34, 22, 155, 155, 235],
    "m9": [1, 22, 22, 13, 44, 44, 30, 201, 201, 301],
    "m10": [1, 29, 29, 13, 45, 45, 35, 272, 272, 521],
}


def load_screen() -> list[dict]:
    rows = []
    with SCREEN.open() as fh:
        for r in csv.DictReader(fh):
            if r.get("ok") != "yes" or not r.get("rmse"):
                continue
            r["b"] = int(r["gpr_bytes"])
            r["e"] = float(r["rmse"])
            rows.append(r)
    return rows


def fit_time_model() -> tuple[float, float] | None:
    """t = c0 + c1*bytes, least squares on the Stage A runs."""
    if not MASTER.exists():
        return None
    xs, ys = [], []
    with MASTER.open() as fh:
        for r in csv.DictReader(fh):
            if not r.get("fps") or not r.get("gpr_bytes"):
                continue
            try:
                f, b = float(r["fps"]), int(r["gpr_bytes"])
            except ValueError:
                continue
            if f > 0:
                xs.append(b)
                ys.append(1.0 / f)
    n = len(xs)
    if n < 5:
        return None
    mx, my = sum(xs) / n, sum(ys) / n
    den = sum((x - mx) ** 2 for x in xs)
    if den == 0:
        return None
    c1 = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / den
    return my - c1 * mx, c1


def pareto(rows: list[dict]) -> list[dict]:
    """Minimise bytes and rmse together."""
    s = sorted(rows, key=lambda r: (r["b"], r["e"]))
    out, best = [], float("inf")
    for r in s:
        if r["e"] < best - 1e-9:
            out.append(r)
            best = r["e"]
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--shortlist", type=int, default=12)
    args = ap.parse_args()

    rows = load_screen()
    by_table = {r["table"]: r for r in rows}
    print(f"screened tables: {len(rows)}")

    base = by_table.get("/".join(map(str, LADDER["m7"])))
    if not base:
        raise SystemExit("m7 baseline missing from screen.csv")
    b0, e0 = base["b"], base["e"]
    print(f"m7 baseline: bytes={b0} rmse={e0}\n")

    print("=== the shipped ladder, as measured ===")
    print(f"{'mode':<6}{'bytes':>10}{'vs m7':>9}{'rmse':>8}{'psnr12':>9}")
    shipped = {}
    for m, t in LADDER.items():
        r = by_table.get("/".join(map(str, t)))
        if r:
            shipped[m] = r
            print(f"{m:<6}{r['b']:>10}{(r['b']/b0-1)*100:>8.1f}%"
                  f"{r['e']:>8.2f}{float(r['psnr12']):>9.2f}")

    front = pareto(rows)
    print(f"\n=== Pareto frontier: {len(front)} of {len(rows)} tables ===")

    # The headline: for each shipped rung, the best matrix table that is no
    # larger (i.e. at least as fast) -- how much error does it save?
    print("\n=== beating the shipped ladder at equal-or-smaller size ===")
    print(f"{'rung':<6}{'rung rmse':>10}{'best rmse':>10}{'gain dB':>9}  best table")
    for m in ("m5", "m6", "m7", "m8", "m9", "m10"):
        if m not in shipped:
            continue
        tgt = shipped[m]
        cands = [r for r in rows if r["b"] <= tgt["b"]]
        if not cands:
            continue
        best = min(cands, key=lambda r: r["e"])
        import math
        gain = 20 * math.log10(tgt["e"] / best["e"]) if best["e"] > 0 else 0
        flag = " (= the rung itself)" if best["table"] == tgt["table"] else ""
        print(f"{m:<6}{tgt['e']:>10.2f}{best['e']:>10.2f}{gain:>9.2f}  "
              f"{best['table']}{flag}")

    model = fit_time_model()
    if model:
        c0, c1 = model
        print(f"\ntime model: t = {c0:.6g} + {c1:.6g}*bytes  "
              f"(m7 predicts {1/(c0+c1*b0):.1f} fps static)")

    # Short list: spread across the frontier by size, so the live camera run
    # brackets wherever 48 fps actually falls.
    picks, seen = [], set()
    if front:
        lo, hi = front[0]["b"], front[-1]["b"]
        for i in range(args.shortlist):
            target = lo + (hi - lo) * i / max(1, args.shortlist - 1)
            r = min(front, key=lambda x: abs(x["b"] - target))
            if r["table"] not in seen:
                seen.add(r["table"])
                picks.append(r)
    for m in ("m7", "m8", "m9"):          # keep shipped anchors for comparison
        r = shipped.get(m)
        if r and r["table"] not in seen:
            seen.add(r["table"])
            picks.append(r)

    import json
    out = []
    for r in picks:
        t = [int(x) for x in r["table"].split("/")]
        out.append({"cell": r["cell"], "table": t,
                    "overrides": {str(i): t[i] for i in range(1, 10)},
                    "gpr_bytes": r["b"], "rmse": r["e"],
                    "family": r.get("family", "")})
    SHORTLIST.write_text(json.dumps(out, indent=1))
    print(f"\n=== short list for fps verification: {len(out)} tables ===")
    print(f"{'cell':<9}{'family':<10}{'bytes':>10}{'vs m7':>9}{'rmse':>8}  table")
    for o in out:
        print(f"{o['cell']:<9}{o['family']:<10}{o['gpr_bytes']:>10}"
              f"{(o['gpr_bytes']/b0-1)*100:>8.1f}%{o['rmse']:>8.2f}  "
              f"{'/'.join(map(str,o['table']))}")


if __name__ == "__main__":
    main()
