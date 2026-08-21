#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Kieran Olsson
"""
screen.py -- screen every generated quant table on gpr_bytes and rmse.

Screening does not need the benchmark harness. gpr_bytes and rmse are exact
functions of the quant table: encode the fixed reference frame once, decode it
once, and both numbers are settled. Running a 5-second timed benchmark per
table would cost 2.4 hours for 794 tables and add nothing, because the fps it
produced would be too noisy to rank on anyway.

That also means screening is free to use the machine however it likes --
nothing here is a timing measurement, so cells run in parallel. fps is measured
later, on the short list only, on a quiet machine.

Per table:
  1. vc5_bench --save-gpr   encode the reference frame with CINEPI_BAND_Q set
  2. gpr_decode_verify --reference   decode and compare against the companded
     source, giving rmse in 12-bit code values
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import shutil
import subprocess
import tempfile
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

HERE = Path(__file__).resolve().parent
BENCH = HERE.parent
BIN = BENCH / "cinepi_qraw_bench" / "build" / "vc5_bench"
DECODER = BENCH / "cinepi_qraw_bench" / "build" / "gpr_decode_verify"
IN12 = BENCH / "cinepi_qraw_bench" / "input" / "sample_imx585_3840x2160_gbrg_12bit_gplog2.raw16"
RESULT = HERE / "screen.csv"
FIELDS = ["cell", "family", "l3", "l2", "l1", "hh_mult", "lambda",
          "table", "gpr_bytes", "rmse", "psnr12", "ok"]


def screen_one(c: dict, threads: int, keep_dir: str | None) -> dict:
    table = c["table"]
    env = dict(os.environ)
    env["CINEPI_BAND_Q"] = ",".join(f"{i}={table[i]}" for i in range(1, 10))
    row = {k: c.get(k, "") for k in ("cell", "family", "l3", "l2", "l1",
                                     "hh_mult", "lambda")}
    row["table"] = "/".join(map(str, table))

    tmp = tempfile.mkdtemp(prefix="scr_", dir="/dev/shm")
    try:
        p = subprocess.run(
            [str(BIN), "--input", str(IN12), "--input-companded", "on",
             "--width", "3840", "--height", "2160",
             "--duration", "0", "--frames", "1", "--warmup", "1",
             "--mode", "m7", "--execution", "cpu-gpr",
             "--cpu-gpr-threads", str(threads), "--save-gpr", tmp],
            env=env, capture_output=True, text=True, timeout=300,
            # vc5_bench resolves validated_input/gpr_params.json relative to
            # the CWD, so it must run from the benchmark dir, not from here.
            cwd=str(BENCH))
        gprs = sorted(Path(tmp).glob("*.gpr"))
        if not gprs:
            row["ok"] = f"no-gpr(rc={p.returncode})"
            return row
        gpr = gprs[-1]
        row["gpr_bytes"] = gpr.stat().st_size

        d = subprocess.run([str(DECODER), "--reference", str(IN12), str(gpr)],
                           capture_output=True, text=True, timeout=300)
        m = re.search(r"rmse=([0-9.]+)", d.stdout)
        if not m:
            row["ok"] = "no-rmse"
            return row
        rmse = float(m.group(1))
        row["rmse"] = f"{rmse:.2f}"
        row["psnr12"] = f"{20 * math.log10(4095.0 / rmse):.2f}" if rmse > 0 else "999"
        row["ok"] = "yes"
        if keep_dir:
            shutil.copy2(gpr, Path(keep_dir) / f"{c['cell']}.gpr")
        return row
    except subprocess.TimeoutExpired:
        row["ok"] = "timeout"
        return row
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--jobs", type=int, default=4)
    ap.add_argument("--threads", type=int, default=1)
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--keep-gpr", default="")
    args = ap.parse_args()

    cands = json.loads((HERE / "matrix.json").read_text())
    done = set()
    if RESULT.exists():
        with RESULT.open() as fh:
            done = {r["cell"] for r in csv.DictReader(fh) if r.get("ok") == "yes"}
    todo = [c for c in cands if c["cell"] not in done]
    if args.limit:
        todo = todo[:args.limit]
    if args.keep_gpr:
        Path(args.keep_gpr).mkdir(parents=True, exist_ok=True)

    print(f"screening {len(todo)} tables ({len(done)} already done), "
          f"{args.jobs} jobs x {args.threads} threads", flush=True)

    new = not RESULT.exists()
    t0 = time.time()
    with RESULT.open("a", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=FIELDS)
        if new:
            w.writeheader()
        with ProcessPoolExecutor(max_workers=args.jobs) as ex:
            futs = {ex.submit(screen_one, c, args.threads,
                              args.keep_gpr or None): c for c in todo}
            for i, f in enumerate(as_completed(futs), 1):
                row = f.result()
                w.writerow(row)
                fh.flush()
                if i % 25 == 0 or i == len(todo):
                    el = time.time() - t0
                    rate = el / i
                    print(f"  {i}/{len(todo)}  {el/60:.1f} min elapsed, "
                          f"~{rate*(len(todo)-i)/60:.1f} min left", flush=True)
    print(f"done in {(time.time()-t0)/60:.1f} min -> {RESULT.name}")


if __name__ == "__main__":
    main()
