#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Kieran Olsson
"""
backfill.py -- recover the live-camera Stage C rows from their logs.

The live path prints "LIVE_CINEPI_RAW_BENCHMARK PASS csv=<path>" while the
static path prints "csv: <path>", and run_cell() only matched the latter, so
30 successful camera runs were recorded as failures. The runs themselves were
fine -- CPU_QRAW_RESULT and CAMERA_RESULT are in every log -- so this reparses
them in place rather than spending another 20 minutes of camera time.
"""
from __future__ import annotations

import csv
import re
from pathlib import Path

HERE = Path(__file__).resolve().parent
LOGS = HERE / "logs"
MASTER = HERE / "master.csv"

FIELDS = ["stage", "cell", "source", "mode", "workers", "band_q", "table",
          "fps", "spread", "ratio", "gpr_bytes", "output_mibs",
          "ms_wavelet", "ms_entropy", "p50_ms", "p99_ms",
          "rmse", "psnr12", "camera_capture_fps", "camera_ok",
          "duration", "passes", "wall_s", "stamp", "gpr_kept"]


def kv(line: str, key: str) -> str:
    m = re.search(rf"\b{re.escape(key)}=([^\s]+)", line)
    return m.group(1) if m else ""


def main() -> None:
    with MASTER.open() as fh:
        rows = list(csv.DictReader(fh))

    fixed = 0
    for r in rows:
        if r["stage"] != "C" or r["fps"] or "cam12" not in r["cell"]:
            continue
        log = LOGS / f"{r['cell']}.log"
        if not log.exists():
            continue
        text = log.read_text()

        res = [l for l in text.splitlines() if l.startswith("CPU_QRAW_RESULT")]
        cam = [l for l in text.splitlines() if l.startswith("CAMERA_RESULT")]
        if not res:
            continue
        # One CPU_QRAW_RESULT per pass; average them, and keep the spread so the
        # noise on each cell stays visible rather than being averaged away.
        fpss = [float(kv(l, "fps")) for l in res if kv(l, "fps")]
        if not fpss:
            continue
        last = res[-1]
        r["fps"] = f"{sum(fpss)/len(fpss):.3f}"
        r["spread"] = f"{max(fpss)-min(fpss):.3f}"
        r["ratio"] = kv(last, "ratio")
        r["gpr_bytes"] = kv(last, "gpr_bytes")
        r["output_mibs"] = kv(last, "output_mibs")
        r["ms_wavelet"] = kv(last, "ms_wavelet")
        r["ms_entropy"] = kv(last, "ms_entropy")
        r["workers"] = kv(last, "workers") or r["workers"]
        if cam:
            r["camera_capture_fps"] = kv(cam[-1], "capture_fps_actual")
            r["camera_ok"] = kv(cam[-1], "camera_ok")
        fixed += 1

    with MASTER.open("w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=FIELDS)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, "") for k in FIELDS})
    print(f"backfilled {fixed} camera rows")


if __name__ == "__main__":
    main()
