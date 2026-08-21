#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Kieran Olsson
"""Analyse a lens-cap DARK STACK and report what the sensor actually does.

This is the calibration infrastructure the QRAW optimisation roadmap asks for at
priority 3, and it answers -- from measurement rather than from metadata -- the
questions behind roadmap items 2 (per-CFA black levels), 14 (defect pixels and
row/column fixed-pattern noise) and 18 (G1/G2 residual equalisation).

It deliberately reports and does NOT write a correction profile. The roadmap's own
rule: "Never promote a sensor correction merely because the file is smaller. It
must first be shown to be a real sensor artefact or statistically justified
noise." So the output is evidence for that decision, with the numbers needed to
size the benefit before any encoder change is written.

CAPTURE (the part that needs you and a lens cap):

    # cap the lens, then in the launcher's environment:
    CINEPI_RAW_DUMP=/media/RAW/dark_iso800.raw16 \\
    CINEPI_RAW_DUMP_FRAMES=48 \\
    ISO_LOCK=1 GAIN=8 ./run_live.sh

    The dump lives in EncodeBuffer2(), which cinepi_raw.cpp only calls while
    RECORDING -- it does NOT fill during preview. So start a take, wait for
    "raw dump complete" in the log, then stop. 48 frames is about a second of
    rolling at 48 fps. tools/../cinepi_stable_campaign.sh CAMPAIGN_MODE=dark
    does all of this per gain.

    Repeat per ISO/gain you care about: the roadmap indexes noise by gain,
    and read noise at ISO 100 tells you nothing about ISO 3200.

ANALYSE:

    python3 tools/qraw_dark_calibrate.py /media/RAW/dark_iso800.raw16 \\
        --width 3840 --height 2160 --bits 12 --bayer rggb

Statistics are robust (median / MAD) rather than mean / stdev throughout, because
a single hot pixel moves a mean and a dark stack is full of them -- that is the
roadmap's explicit instruction and the reason a defect map is built BEFORE any
noise threshold is derived from the same data.
"""

import argparse
import os
import sys

try:
    import numpy as np
except ImportError:
    sys.exit("needs numpy: sudo apt install -y python3-numpy")

# MAD -> sigma for a normal distribution. Robust to the outliers a dark stack has.
MAD_TO_SIGMA = 1.4826

# Bayer phase index -> name, for the four 2x2 positions (row%2, col%2).
PHASE_NAMES = {
    "rggb": ["R", "Gr", "Gb", "B"],
    "gbrg": ["Gb", "B", "R", "Gr"],
    "bggr": ["B", "Gb", "Gr", "R"],
    "grbg": ["Gr", "R", "B", "Gb"],
}


def load_stack(path, w, h, limit=None):
    """Memory-map the stack so a 1 GB file does not have to fit in RAM."""
    per = w * h
    total = os.path.getsize(path) // 2
    n = total // per
    if n == 0:
        sys.exit("file holds no complete %dx%d uint16 frame" % (w, h))
    if total % per:
        print("  note: %d trailing samples ignored (partial frame)" % (total % per))
    if limit:
        n = min(n, limit)
    mm = np.memmap(path, dtype=np.uint16, mode="r", shape=(n, h, w))
    return mm, n


def phase_stats(temporal_median, bayer):
    """Per-CFA-phase pedestal. This is roadmap item 2, measured rather than read
    from metadata -- libcamera reporting one value for all four phases does not
    prove the sensor has one."""
    names = PHASE_NAMES[bayer]
    out = []
    for i, (dy, dx) in enumerate([(0, 0), (0, 1), (1, 0), (1, 1)]):
        sub = temporal_median[dy::2, dx::2]
        med = float(np.median(sub))
        mad = float(np.median(np.abs(sub - med)))
        out.append((names[i], med, mad * MAD_TO_SIGMA))
    return out


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("stack", help="raw16 dark stack from CINEPI_RAW_DUMP")
    ap.add_argument("--width", type=int, default=3840,
                    help="the dump writes the CROPPED active area "
                         "(cfg_.width), not the transport width")
    ap.add_argument("--height", type=int, default=2160)
    ap.add_argument("--bits", type=int, default=12,
                    help="sensor bit depth of the samples in the file")
    ap.add_argument("--bayer", default="rggb", choices=sorted(PHASE_NAMES))
    ap.add_argument("--frames", type=int, default=0,
                    help="use at most N frames (0 = all)")
    ap.add_argument("--hot-sigma", type=float, default=8.0,
                    help="a pixel is hot when its temporal median exceeds the "
                         "frame median by this many robust sigma (default 8)")
    args = ap.parse_args()

    mm, n = load_stack(args.stack, args.width, args.height,
                       args.frames or None)
    full = (1 << args.bits) - 1
    print("dark stack: %s" % args.stack)
    print("  %d frames of %dx%d, %d-bit (full scale %d), bayer %s\n"
          % (n, args.width, args.height, args.bits, full, args.bayer))
    if n < 16:
        print("  WARNING: %d frames is thin. The roadmap asks for 32-64; fixed\n"
              "  structure and random noise are not cleanly separable below that,\n"
              "  and a defect map built from too few frames will include\n"
              "  coincidences.\n" % n)

    # Temporal median per pixel separates FIXED structure from random noise.
    # float32 keeps 12-bit values exact and halves the memory against float64.
    print("computing temporal median (separates fixed structure from read noise)...")
    med = np.median(np.asarray(mm, dtype=np.float32), axis=0)

    frame_med = float(np.median(med))
    frame_mad = float(np.median(np.abs(med - frame_med))) * MAD_TO_SIGMA
    print("  frame pedestal: median %.2f codes, robust sigma %.3f\n"
          % (frame_med, frame_mad))

    # ---- roadmap 2 + 18: per-CFA pedestal, and the G1/G2 pair specifically ---
    print("PER-CFA PEDESTAL (roadmap item 2)")
    print("  phase   median      sigma")
    ph = phase_stats(med, args.bayer)
    for name, m, sg in ph:
        print("  %-5s  %8.2f   %8.3f" % (name, m, sg))
    meds = [m for _, m in [(a, b) for a, b, _ in ph]]
    spread = max(meds) - min(meds)
    print("\n  spread across phases: %.2f codes" % spread)
    if spread < 0.5:
        print("  -> BELOW half a code. Per-CFA black correction would remove"
              "\n     nothing measurable; do not implement it for this sensor/mode.")
    else:
        print("  -> a real per-phase pedestal difference. Correcting it BEFORE the"
              "\n     GS/RG/BG/GD transform stops it becoming artificial colour"
              "\n     residual and green-difference energy for the wavelet to code.")

    gr = next((m for nm, m, _ in ph if nm == "Gr"), None)
    gb = next((m for nm, m, _ in ph if nm == "Gb"), None)
    if gr is not None and gb is not None:
        print("\nG1/G2 RESIDUAL (roadmap item 18): Gr - Gb = %+.2f codes" % (gr - gb))
        print("  GD carries G1-G2, so a systematic offset here is energy in every"
              "\n  GD coefficient. %s"
              % ("Worth equalising." if abs(gr - gb) >= 0.5 else
                 "Nothing to equalise."))

    # ---- read noise, which is what a Noise Clean threshold rests on ---------
    print("\nREAD NOISE (input to roadmap item 15, Noise Clean)")
    # Temporal spread per pixel, then a robust summary across the array. This is
    # RANDOM noise: it does not survive the temporal median above.
    sample = np.asarray(mm[:, ::8, ::8], dtype=np.float32)
    tmed = np.median(sample, axis=0)
    tmad = np.median(np.abs(sample - tmed), axis=0) * MAD_TO_SIGMA
    print("  per-pixel temporal sigma: median %.3f codes, p99 %.3f"
          % (float(np.median(tmad)), float(np.percentile(tmad, 99))))
    print("  (sampled every 8th row and column: %d pixels)" % tmad.size)
    print("\n  NOTE: this is DARK noise only. Photon shot noise cannot be measured"
          "\n  with the lens capped -- its variance grows with signal -- so a"
          "\n  lens-cap profile is a Phase 1 model of electronic/dark/structured"
          "\n  noise and nothing more (roadmap items 11 and 17).")
    # ---- roadmap 14: defect pixels -------------------------------------------
    print("\nDEFECT PIXELS (roadmap item 14)")
    # The threshold is k x the READ NOISE, not k x the spatial MAD of the
    # temporal median.
    #
    # The first version used the spatial MAD and produced nonsense: when the
    # sensor is WELL behaved that MAD quantises to 0.000 codes, so the threshold
    # collapsed onto the pedestal and every pixel one code above the median
    # counted as a defect -- 432,756 "hot pixels" at RAW12 ISO 100 against 147 at
    # ISO 800, which is the opposite of physical (defects do not disappear as gain
    # rises). A good sensor made the detector worse, which is the tell.
    #
    # Read noise is the right yardstick because it is what a defect has to stand
    # out FROM to matter: a pixel whose temporal median sits 8 sigma above the
    # pedestal is one a viewer would see in a single frame. The floor of 8 codes
    # keeps the rule sane at low gain, where sigma is ~1.5 codes and 8 sigma would
    # flag ordinary quantisation.
    sigma_read = float(np.median(tmad))
    thresh = frame_med + max(args.hot_sigma * sigma_read, 8.0)
    hot = np.argwhere(med > thresh)
    dead = np.argwhere(med <= 0)
    print("  hot (temporal median > pedestal + max(%.1f x read sigma %.2f, 8) "
          "= %.1f codes): %d" % (args.hot_sigma, sigma_read, thresh, len(hot)))
    print("  stuck at zero: %d" % len(dead))
    if len(hot):
        vals = med[hot[:, 0], hot[:, 1]]
        order = np.argsort(vals)[::-1][:8]
        print("  worst offenders (y, x, median code):")
        for i in order:
            y, x = hot[i]
            print("    %5d %5d   %8.1f" % (y, x, vals[i]))
        frac = 100.0 * len(hot) / float(args.width * args.height)
        print("  that is %.5f%% of the array." % frac)
        print("  -> a defect is expensive in a WAVELET codec: one outlier makes"
              "\n     several high-frequency coefficients across levels. Replacing"
              "\n     it from same-phase neighbours is cheaper than coding it.")

    # ---- roadmap 14: row / column fixed-pattern noise ------------------------
    print("\nROW/COLUMN FIXED-PATTERN NOISE (roadmap item 14)")
    # Per Bayer phase, so a phase pedestal difference is not misread as FPN.
    for i, (dy, dx) in enumerate([(0, 0), (0, 1), (1, 0), (1, 1)]):
        name = PHASE_NAMES[args.bayer][i]
        sub = med[dy::2, dx::2]
        base = float(np.median(sub))
        rows = np.median(sub, axis=1) - base
        cols = np.median(sub, axis=0) - base
        print("  %-3s row offsets: sigma %.4f, peak %+.2f | "
              "col offsets: sigma %.4f, peak %+.2f"
              % (name,
                 float(np.median(np.abs(rows)) * MAD_TO_SIGMA),
                 float(rows[np.argmax(np.abs(rows))]),
                 float(np.median(np.abs(cols)) * MAD_TO_SIGMA),
                 float(cols[np.argmax(np.abs(cols))])))
    print("  -> structured offsets are REPEATABLE, so they are correctable and"
          "\n     must not be lumped in with random noise when a Noise Clean"
          "\n     threshold is derived. Correct first, then measure noise.")

    print("\n  Noise Clean thresholds must be calibrated in PRE-QUANT WAVELET"
          "\n  coefficient units, not pixel codes, so the profile survives a"
          "\n  change to the m-mode quant ladder. That needs the production"
          "\n  wavelet applied to these frames -- a second tool, and the next"
          "\n  thing to build if these numbers justify it.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
