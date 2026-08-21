#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Kieran Olsson
"""
gen_matrix.py -- generate the large quant-table matrix.

The package ships ten hand-tuned quant tables (m1..m10). Each is a 10-entry
vector, and entry 0 (LL3) is never read by the encoder (vc5_bench.cpp:2649
computes base = 10 - 3*level for levels 1..3, so only 1..9 are dereferenced --
verified by measurement: overriding entry 0 leaves the output byte-identical).

The nine live entries fall into three wavelet levels:

    entries 1,2,3  = L3  LH / HL / HH   (coarsest detail)
    entries 4,5,6  = L2  LH / HL / HH
    entries 7,8,9  = L1  LH / HL / HH   (finest detail, most coefficients)

FAMILY 1 -- LADDER MIX (1000 tables). Take the L3 triple from any mode, the
L2 triple from any mode and the L1 triple from any mode: 10 x 10 x 10. This is
the matrix the ladders themselves imply. The shipped modes are the diagonal
(a==b==c); everything off-diagonal is a table nobody has measured, and the
interesting region is high-L1 / low-L3 -- spend the bits where the eye is.

FAMILY 2 -- ORIENTATION. On top of a mix, scale the HH entries (3, 6, 9)
independently of LH/HL. Diagonal detail is the perceptually cheapest, and the
campaign's own E1 result (HH1 pruned) is the degenerate case of this axis.

FAMILY 3 -- RD FRONTIER, from the Stage A per-entry sensitivity. For a weight
L, each entry independently picks the q minimising (bytes + L*mse); sweeping L
traces the optimal frontier. Unlike families 1 and 2 this is not restricted to
values that appear in some shipped ladder.

Every generated table is SCREENED on gpr_bytes and rmse, which are exact
functions of the quant table and carry no measurement noise. fps is verified
afterwards, on the short list only, on a quiet machine.
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
MASTER = HERE / "master.csv"
OUTFILE = HERE / "matrix.json"

LADDER = {
    "m1":  [1, 4, 4, 2, 8, 8, 6, 26, 26, 39],
    "m2":  [1, 4, 4, 2, 8, 8, 6, 33, 33, 50],
    "m3":  [1, 4, 4, 2, 10, 10, 6, 43, 43, 66],
    "m4":  [1, 6, 6, 4, 14, 14, 10, 55, 55, 87],
    "m5":  [1, 8, 8, 4, 16, 16, 11, 73, 73, 111],
    "m6":  [1, 10, 10, 6, 20, 20, 14, 93, 93, 143],
    "m7":  [1, 14, 14, 8, 26, 26, 18, 126, 126, 175],
    "m8":  [1, 18, 18, 9, 34, 34, 22, 155, 155, 235],
    "m9":  [1, 22, 22, 13, 44, 44, 30, 201, 201, 301],
    "m10": [1, 29, 29, 13, 45, 45, 35, 272, 272, 521],
}
MODES = list(LADDER)
L3, L2, L1 = (1, 2, 3), (4, 5, 6), (7, 8, 9)
HH = (3, 6, 9)
BASE = LADDER["m7"]


def mk(l3: str, l2: str, l1: str) -> list[int]:
    t = [1] * 10
    for i in L3:
        t[i] = LADDER[l3][i]
    for i in L2:
        t[i] = LADDER[l2][i]
    for i in L1:
        t[i] = LADDER[l1][i]
    return t


def family_mix() -> list[dict]:
    out = []
    for a in MODES:
        for b in MODES:
            for c in MODES:
                out.append({"family": "mix", "l3": a, "l2": b, "l1": c,
                            "table": mk(a, b, c)})
    return out


def family_orientation(seeds: list[tuple[str, str, str]]) -> list[dict]:
    out = []
    for a, b, c in seeds:
        base = mk(a, b, c)
        for hh in (1.5, 2.0, 3.0, 4.0, 6.0):
            t = list(base)
            for i in HH:
                t[i] = max(1, min(32767, int(round(base[i] * hh))))
            out.append({"family": "hh", "l3": a, "l2": b, "l1": c,
                        "hh_mult": hh, "table": t})
        for hh in (1.5, 2.0, 3.0):          # prune HH1, scale the other two
            t = list(base)
            t[9] = 32767
            for i in (3, 6):
                t[i] = max(1, int(round(base[i] * hh)))
            out.append({"family": "hh1prune", "l3": a, "l2": b, "l1": c,
                        "hh_mult": hh, "table": t})
    return out


def family_rd(n_lambda: int) -> list[dict]:
    """Lagrangian frontier over the Stage A per-entry sensitivity curves."""
    if not MASTER.exists():
        return []
    with MASTER.open() as fh:
        rows = [r for r in csv.DictReader(fh)
                if r.get("rmse") and r.get("gpr_bytes")]
    ctl = [r for r in rows if r["cell"].startswith("A_ctl_")]
    if not ctl:
        return []
    b0, r0 = int(ctl[0]["gpr_bytes"]), float(ctl[0]["rmse"])

    per: dict[int, list[dict]] = {b: [{"q": BASE[b], "d_bytes": 0, "d_mse": 0.0}]
                                  for b in range(1, 10)}
    for r in rows:
        if not r["cell"].startswith("A_b") or "_rm" in r["cell"]:
            continue
        e = int(r["cell"].split("_")[1][1:])
        q = int(r["cell"].split("_q")[1][:5])
        per[e].append({"q": q,
                       "d_bytes": b0 - int(r["gpr_bytes"]),
                       "d_mse": max(0.0, float(r["rmse"]) ** 2 - r0 ** 2)})

    out, seen = [], set()
    for i in range(n_lambda):
        L = 10.0 ** (-3.0 + 6.0 * i / max(1, n_lambda - 1))
        t = list(BASE)
        for e in range(1, 10):
            t[e] = min(per[e], key=lambda p: -p["d_bytes"] + L * p["d_mse"])["q"]
        key = tuple(t)
        if key not in seen:
            seen.add(key)
            out.append({"family": "rd", "lambda": L, "table": t})
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--lambdas", type=int, default=40)
    args = ap.parse_args()

    cands = family_mix()
    # Orientation on a spread of mixes: the shipped diagonal plus the
    # asymmetric corners where L1 is coarse and L3 is fine.
    seeds = [("m1", "m1", "m7"), ("m1", "m3", "m9"), ("m3", "m5", "m10"),
             ("m5", "m5", "m10"), ("m1", "m5", "m8"), ("m7", "m7", "m7"),
             ("m5", "m7", "m9"), ("m3", "m3", "m8")]
    cands += family_orientation(seeds)
    cands += family_rd(args.lambdas)

    seen, uniq = set(), []
    for c in cands:
        key = tuple(c["table"])
        if key in seen:
            continue
        seen.add(key)
        c["cell"] = f"M_{len(uniq):04d}"
        uniq.append(c)

    OUTFILE.write_text(json.dumps(uniq, indent=1))
    fam: dict[str, int] = {}
    for c in uniq:
        fam[c["family"]] = fam.get(c["family"], 0) + 1
    print(f"{len(uniq)} unique quant tables -> {OUTFILE.name}")
    for k, v in sorted(fam.items()):
        print(f"  {k:<10} {v}")


if __name__ == "__main__":
    main()
