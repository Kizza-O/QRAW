#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Kieran Olsson
"""
qm.py -- CinePi VC-5 quant-ladder sensitivity matrix.

Stage A  per-band sensitivity sweep on the fixed static 16-bit frame.
         One band moved at a time off the m7 base ladder, so every cell
         yields a clean dFPS / dRMSE gradient for that band.
Stage B  combined candidates built from the Stage A gradients.
Stage C  survivors re-measured on the live camera RAW12/60 path, 3 and 4
         workers, where the 48 fps gate actually applies.
Stage D  finalists: keep the GPR, decode to raw16 + DNG for visual review.

Every cell runs through RUN_BENCHMARK.sh -- the gated path, pinned binary --
so fps and the saved frame come from the same run and cannot disagree.

Quality metric: the decoder emits the 12-bit companded domain, so PSNR is
computed against sample_..._12bit_gplog2.raw16 with peak 4095. gpr_decode_verify
prints psnr with peak 65535, which is +24.08 dB; we recompute from its rmse.
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
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
BENCH = HERE.parent
OUT = HERE
FRAMES = OUT / "frames"
LOGS = OUT / "logs"
MASTER = OUT / "master.csv"

RUNNER = BENCH / "RUN_BENCHMARK.sh"
DECODER = BENCH / "cinepi_qraw_bench" / "build" / "gpr_decode_verify"
REF12 = BENCH / "cinepi_qraw_bench" / "input" / "sample_imx585_3840x2160_gbrg_12bit_gplog2.raw16"

# Production ladder, vc5_bench.cpp:5017-5028 (universal-standard-v3).
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
BAND_NAME = ["LL3", "L3-LH", "L3-HL", "L3-HH",
             "L2-LH", "L2-HL", "L2-HH",
             "L1-LH", "L1-HL", "L1-HH"]
PRUNE = 32767

FIELDS = [
    "stage", "cell", "source", "mode", "workers", "band_q", "table",
    "fps", "spread", "ratio", "gpr_bytes", "output_mibs",
    "ms_wavelet", "ms_entropy", "p50_ms", "p99_ms",
    "rmse", "psnr12", "camera_capture_fps", "camera_ok",
    "duration", "passes", "wall_s", "stamp", "gpr_kept",
]


def log(msg: str) -> None:
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def load_done() -> dict[str, dict]:
    if not MASTER.exists():
        return {}
    with MASTER.open() as fh:
        return {r["cell"]: r for r in csv.DictReader(fh)}


def append(row: dict) -> None:
    new = not MASTER.exists()
    with MASTER.open("a", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=FIELDS)
        if new:
            w.writeheader()
        w.writerow({k: row.get(k, "") for k in FIELDS})


def band_q_spec(overrides: dict[int, int]) -> str:
    return ",".join(f"{b}={q}" for b, q in sorted(overrides.items()))


def resolved_table(mode: str, overrides: dict[int, int]) -> list[int]:
    t = list(LADDER[mode])
    for b, q in overrides.items():
        t[b] = q
    return t


def run_cell(cell: str, *, stage: str, mode: str, overrides: dict[int, int],
             source: str, workers: int, duration: float, passes: int,
             keep_gpr: bool) -> dict:
    """One benchmark run. Returns the parsed row (already appended)."""
    env = dict(os.environ)
    env.update({
        "MODES": mode,
        "THREADS": str(workers),
        "PASSES": str(passes),
        "DURATION": str(duration),
        "VERIFY": "off",
        "SAVE_QRAW": "on" if keep_gpr else "off",
        "GPR_PATH": str(FRAMES),
        "CINEPI_TTY": "/dev/null",
    })
    if source == "static16":
        env.update({"SOURCE": "sample", "INPUT_BITS": "16"})
    elif source == "static12":
        env.update({"SOURCE": "sample", "INPUT_BITS": "12"})
    elif source == "cam12":
        env.update({"SOURCE": "camera", "INPUT_BITS": "12",
                    "CINEPI_LIVEVIEW": "0", "COMPAND_BITS": "12"})
    elif source == "cam16":
        env.update({"SOURCE": "camera", "INPUT_BITS": "16",
                    "CINEPI_CLEARHDR_BITS": "16", "CINEPI_LIVEVIEW": "0"})
    else:
        raise ValueError(source)

    spec = band_q_spec(overrides)
    if spec:
        env["CINEPI_BAND_Q"] = spec
        env["CINEPI_BAND_TAG"] = cell
    else:
        env.pop("CINEPI_BAND_Q", None)
        env["CINEPI_BAND_TAG"] = cell

    t0 = time.time()
    proc = subprocess.run([str(RUNNER)], cwd=str(BENCH), env=env,
                          capture_output=True, text=True)
    wall = time.time() - t0
    (LOGS / f"{cell}.log").write_text(proc.stdout + proc.stderr)

    m = re.search(r"csv[=:]\s*(\S+)", proc.stdout, re.M)
    if not m:
        log(f"  !! {cell}: no csv emitted (rc={proc.returncode})")
        row = {"stage": stage, "cell": cell, "source": source, "mode": mode,
               "workers": workers, "band_q": spec,
               "table": "/".join(map(str, resolved_table(mode, overrides))),
               "fps": "", "wall_s": f"{wall:.1f}"}
        append(row)
        return row

    csv_path = Path(m.group(1))
    with csv_path.open() as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        log(f"  !! {cell}: empty csv")
        row = {"stage": stage, "cell": cell, "source": source, "mode": mode,
               "workers": workers, "band_q": spec, "fps": "",
               "wall_s": f"{wall:.1f}"}
        append(row)
        return row

    fps = sum(float(r["fps"]) for r in rows) / len(rows)
    vals = [float(r["fps"]) for r in rows]
    last = rows[-1]

    def g(k: str) -> str:
        v = last.get(k, "")
        return v if v not in (None, "") else ""

    stamp = csv_path.stem.replace("results_cinepi_", "")
    row = {
        "stage": stage, "cell": cell, "source": source, "mode": mode,
        "workers": workers, "band_q": spec,
        "table": "/".join(map(str, resolved_table(mode, overrides))),
        "fps": f"{fps:.3f}", "spread": f"{max(vals) - min(vals):.3f}",
        "ratio": g("ratio"), "gpr_bytes": g("gpr_bytes"),
        "output_mibs": g("output_mibs"),
        "ms_wavelet": g("ms_wavelet"), "ms_entropy": g("ms_entropy"),
        "p50_ms": g("latency_p50_ms"), "p99_ms": g("latency_p99_ms"),
        "camera_capture_fps": g("camera_capture_fps"),
        "camera_ok": g("camera_ok"),
        "duration": duration, "passes": passes,
        "wall_s": f"{wall:.1f}", "stamp": stamp, "gpr_kept": "",
    }

    if keep_gpr:
        found = sorted(FRAMES.glob(f"gpr_out_*_{cell}"))
        if found:
            d = found[-1]
            src_copy = d / "source_uncompressed.raw16"
            if src_copy.exists():
                src_copy.unlink()          # identical every run; 16.8 MB each
            gpr = d / f"{mode}.gpr"
            if gpr.exists():
                dest = FRAMES / f"{cell}.gpr"
                shutil.move(str(gpr), dest)
                row["gpr_kept"] = dest.name
                rmse, psnr12 = measure_quality(dest)
                row["rmse"] = "" if rmse is None else f"{rmse:.2f}"
                row["psnr12"] = "" if psnr12 is None else f"{psnr12:.2f}"
            shutil.rmtree(d, ignore_errors=True)

    append(row)
    q = f" rmse={row.get('rmse','-')} psnr12={row.get('psnr12','-')}" if keep_gpr else ""
    log(f"  {cell:28s} fps={fps:7.2f} ratio={row['ratio'][:5]:>5s}{q}  ({wall:.0f}s)")
    return row


def measure_quality(gpr: Path) -> tuple[float | None, float | None]:
    """RMSE vs the companded reference, and a true 12-bit-peak PSNR."""
    p = subprocess.run([str(DECODER), "--reference", str(REF12), str(gpr)],
                       capture_output=True, text=True)
    m = re.search(r"rmse=([0-9.]+)", p.stdout)
    if not m:
        return None, None
    rmse = float(m.group(1))
    psnr12 = 20.0 * math.log10(4095.0 / rmse) if rmse > 0 else 999.0
    return rmse, psnr12


# ---------------------------------------------------------------- stage A
def stage_a(args) -> None:
    mode = args.mode
    base = LADDER[mode]
    done = load_done()
    cells: list[tuple[str, dict]] = []

    # Band 0 (LL3) is deliberately absent. vc5_bench.cpp:2649 computes
    # base = 10 - 3*level for levels 1..3, so the encoder only ever reads
    # indices 1..9. Index 0 is never dereferenced -- the lowpass is not
    # quantised by this table. Measured: overriding it leaves ratio and
    # rmse bit-identical. It is not a tunable.
    for band in range(1, 10):
        b0 = base[band]
        levels = []
        for mult in (1.5, 2, 3, 4, 6):
            v = max(1, min(PRUNE, int(round(b0 * mult))))
            if v != b0 and v not in levels:
                levels.append(v)
        levels.append(PRUNE)
        for q in levels:
            cell = f"A_b{band}_q{q:05d}"
            cells.append((cell, {band: q}))

    log(f"Stage A: {len(cells)} cells, base {mode} = {base}")
    log(f"         + an E0 control every {args.control_every} cells (drift check)")

    if args.limit:
        cells = cells[:args.limit]
    n = 0
    for i, (cell, ov) in enumerate(cells):
        if i % args.control_every == 0:
            ctl = f"A_ctl_{i:03d}"
            if ctl not in done:
                run_cell(ctl, stage="A-control", mode=mode, overrides={},
                         source="static16", workers=args.workers,
                         duration=args.duration, passes=args.passes,
                         keep_gpr=True)
                n += 1
        if cell in done:
            continue
        r = run_cell(cell, stage="A", mode=mode, overrides=ov, source="static16",
                     workers=args.workers, duration=args.duration,
                     passes=args.passes, keep_gpr=True)
        n += 1
        # The measured within-boot noise floor is ~3.2% spread at PASSES=1.
        # Anything wider than that is machine state, not the quant change, so
        # re-measure the cell once rather than publishing the outlier.
        try:
            if r.get("fps") and float(r["spread"]) / float(r["fps"]) > 0.05:
                log(f"  .. {cell}: spread {r['spread']} too wide, re-measuring")
                run_cell(f"{cell}_rm", stage="A-remeasure", mode=mode,
                         overrides=ov, source="static16", workers=args.workers,
                         duration=args.duration, passes=args.passes,
                         keep_gpr=False)
                n += 1
        except (ValueError, KeyError, ZeroDivisionError):
            pass
    log(f"Stage A complete: {n} new cells")


# ---------------------------------------------------------------- stage B
def stage_b(args) -> None:
    """Measure every combined candidate's bytes + rmse. Both are exact
    functions of the quant table, so a short run is enough -- fps from these
    cells is recorded but deliberately NOT trusted."""
    cands = json.loads((OUT / "candidates.json").read_text())
    done = load_done()
    log(f"Stage B: {len(cands)} candidates, screening on bytes+rmse "
        f"(duration={args.duration}, fps not trusted at this length)")
    n = 0
    for c in cands:
        if c["cell"] in done:
            continue
        ov = {int(k): int(v) for k, v in c["overrides"].items()}
        if not ov:
            continue
        run_cell(c["cell"], stage="B", mode=args.mode, overrides=ov,
                 source="static16", workers=args.workers,
                 duration=args.duration, passes=1, keep_gpr=True)
        n += 1
    log(f"Stage B complete: {n} new cells")


# ---------------------------------------------------------------- stage C
def stage_c(args) -> None:
    """fps verification for the short list, where it actually has to hold up:
    static with replication, then the live RAW12/60 camera at 3 and 4 workers.
    48 fps is gated here, not on the screening numbers."""
    shortlist = json.loads((OUT / "shortlist.json").read_text())
    done = load_done()
    log(f"Stage C: {len(shortlist)} candidates x (static + cam12 w3 + cam12 w4)")
    n = 0
    for s in shortlist:
        ov = {int(k): int(v) for k, v in s["overrides"].items()}
        tag = s["cell"]
        plan = [(f"C_{tag}_static", "static16", args.workers, args.passes)]
        for w in (3, 4):
            plan.append((f"C_{tag}_cam12_w{w}", "cam12", w, args.cam_passes))
        for cell, src, w, ps in plan:
            if cell in done:
                continue
            run_cell(cell, stage="C", mode=args.mode, overrides=ov, source=src,
                     workers=w, duration=args.duration, passes=ps,
                     keep_gpr=(src == "static16"))
            n += 1
    log(f"Stage C complete: {n} new cells")


# ---------------------------------------------------------------- stage D
def stage_d(args) -> None:
    """Decode the kept finalist frames to viewable raw16 + DNG, named by config."""
    view = OUT / "view"
    view.mkdir(exist_ok=True)
    rows = load_done()
    finalists = json.loads((OUT / "shortlist.json").read_text())
    want = {f"C_{s['cell']}_static" for s in finalists}
    want.add("A_ctl_000")                      # the E0 reference to compare against
    n = 0
    for cell in sorted(want):
        gpr = FRAMES / f"{cell}.gpr"
        if not gpr.exists():
            log(f"  !! {cell}: no kept gpr")
            continue
        r = rows.get(cell, {})
        table = (r.get("table") or "").replace("/", "-")
        rmse = r.get("rmse", "na")
        name = f"{cell}__q{table}__rmse{rmse}"
        subprocess.run([str(DECODER), "--dump-raw", str(view / name), str(gpr)],
                       capture_output=True, text=True)
        shutil.copy2(gpr, view / f"{name}.gpr")
        n += 1
        log(f"  {name}")
    log(f"Stage D: {n} finalists decoded into {view}")
    log("  DNG export: ./export_gpr_folder_to_dng <dir>  (run from the bench dir)")


def main() -> int:
    for d in (OUT, FRAMES, LOGS):
        d.mkdir(parents=True, exist_ok=True)
    ap = argparse.ArgumentParser()
    ap.add_argument("stage", choices=["A", "B", "C", "D"])
    ap.add_argument("--cam-passes", type=int, default=2)
    ap.add_argument("--mode", default="m7")
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--duration", type=float, default=5)
    ap.add_argument("--passes", type=int, default=1)
    ap.add_argument("--control-every", type=int, default=10)
    ap.add_argument("--limit", type=int, default=0, help="smoke test: first N cells")
    args = ap.parse_args()
    {"A": stage_a, "B": stage_b, "C": stage_c, "D": stage_d}[args.stage](args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
