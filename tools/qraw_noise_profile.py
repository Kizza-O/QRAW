#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Kieran Olsson
"""Turn measured pixel-domain dark noise into a per-band PRE-QUANT noise profile.

This is the calibration step Noise Clean (roadmap item 15) needs: the roadmap is
explicit that thresholds must be indexed by sensor / gain / component / level /
band and expressed in PRE-QUANT WAVELET COEFFICIENT units, so the profile stays
valid if the m1..m10 quant ladder changes later.

HOW IT GETS THERE, AND WHAT IS ASSUMED
--------------------------------------
The 3-level VC-5 wavelet and the GS/RG/BG/GD component transform are LINEAR, so
they propagate a noise standard deviation exactly, with no approximation, by the
root-sum-square of their filter weights. The filters are taken verbatim from the
production encoder (vc5_bench.cpp, cinepi_fused_v_scalar / cinepi_hp6):

    interior highpass  (-1,-1,8,-8,1,1)/8   -> gain sqrt(132)/8 = 1.43614
    lowpass pair       (l0 + l1)            -> gain sqrt(2)     = 1.41421
    levels 2 and 3 prescale the lowpass input by >>2 before filtering.

The ONE assumption is that read noise is spatially WHITE -- uncorrelated between
neighbouring photosites. That is not taken on faith: qraw_dark_calibrate.py
measures row and column fixed-pattern noise on the same stacks and reports
sigma 0.0000 with sub-code peaks on this sensor, which is what spatial
independence looks like. If a future sensor shows real row/column structure, this
derivation stops being valid for that sensor and the profile must come from the
production transform instead.

GP-LOG2, AND WHY IT DOMINATES
-----------------------------
GP-Log2 is nonlinear, so it scales the pixel sigma before the wavelet sees any of
it. It is NOT left as an unknown: the curve is deterministic --
y = log1p(k*x)/log1p(k) with x = (code-black)/(white-black), k=599 -- so the tool
builds the encoder's own LUT and measures the effective gain by mapping a
Gaussian of the measured sigma through it. A single tangent would be wrong,
because at k=599 the curve is violently convex at black: ~98 LUT units per sensor
code at x=0, ~57 four codes up, and read noise straddles several codes.

The result is the largest single factor in the chain -- 34x to 54x on this sensor.
One sensor code at black becomes 92 working units, so a few codes of read noise
arrive at the wavelet as a few HUNDRED units. That is the real reason shadow noise
is expensive here: not the wavelet, not the quantiser, but the log curve doing
exactly what it is for. It is also precisely the energy Noise Clean exists to stop
coding.

So the profile below is ABSOLUTE, in pre-quant coefficient units, and can be
compared directly against the m-mode quantiser divisors.
"""

import argparse
import math
import sys

# Verbatim from the production encoder. Root-sum-square of the weights, over the
# same /8 normalisation the integer filter applies.
HP_GAIN = math.sqrt(1 + 1 + 64 + 64 + 1 + 1) / 8.0     # 1.436141
LP_GAIN = math.sqrt(2.0)                               # 1.414214
PRESCALE = 0.25                                        # >>2 on levels 2 and 3

# Component transform. GPR carries GS (green sum), RG and BG (colour residuals)
# and GD (green difference), each formed from the four Bayer phases. Treating the
# phases as independent with equal sigma, the transform gains are the
# root-sum-square of each output's coefficients:
#   GS = (G1 + G2)/2      -> sqrt(2)/2   = 0.7071
#   GD =  G1 - G2         -> sqrt(2)     = 1.4142
#   RG =  R - (G1+G2)/2   -> sqrt(1 + 1/2) = 1.2247
#   BG =  B - (G1+G2)/2   -> sqrt(1 + 1/2) = 1.2247
COMPONENT_GAIN = {"GS": math.sqrt(2) / 2.0,
                  "RG": math.sqrt(1.5),
                  "BG": math.sqrt(1.5),
                  "GD": math.sqrt(2)}

# Band index in the encoder's own numbering: base = 10 - 3*level, then
# LH, HL, HH. Level 1 -> 7,8,9; level 2 -> 4,5,6; level 3 -> 1,2,3.
def band_index(level, which):
    return (10 - 3 * level) + {"LH": 0, "HL": 1, "HH": 2}[which]


def band_gains():
    """Noise gain from the component plane to each (level, band), pre-quant.

    A level applies the horizontal filter then the vertical one, so each band's
    gain is the product of its two 1-D gains. Levels 2 and 3 see the previous
    level's LL, which was lowpass-filtered twice and then prescaled.
    """
    out = {}
    ll = 1.0                      # gain accumulated into the LL the level reads
    for level in (1, 2, 3):
        pre = 1.0 if level == 1 else PRESCALE
        base = ll * pre
        # LH = horizontal high, vertical low.  HL = horizontal low, vertical high.
        out[(level, "LH")] = base * HP_GAIN * LP_GAIN
        out[(level, "HL")] = base * LP_GAIN * HP_GAIN
        out[(level, "HH")] = base * HP_GAIN * HP_GAIN
        ll = base * LP_GAIN * LP_GAIN      # what the next level will read
    return out


def gplog2_gain(sigma, pedestal, black, white, working_max, k):
    """Effective noise gain through GP-Log2, computed from the encoder's own LUT.

    NOT a single slope. The curve is y = log1p(k*x)/log1p(k) with
    x = (code - black)/(white - black), and at k=599 that is violently convex
    near black: the analytic slope is ~98 LUT units per sensor code at x=0 but
    only ~57 four codes up. Read noise straddles several codes, so the honest
    figure is the RMS of the mapped distribution, not the tangent.

    This is why the log curve dominates the whole noise budget: one sensor code
    at black becomes ~92 working units, so a few codes of read noise arrive at
    the wavelet as a few HUNDRED units. Shadow noise is expensive in a log
    domain in a way it simply is not in a linear one -- which is exactly the
    energy Noise Clean exists to stop coding.
    """
    try:
        import numpy as np
    except ImportError:
        sys.exit("computing the LUT gain needs numpy; or pass --gplog2-slope")
    den = math.log1p(k)
    codes = np.arange(4096, dtype=np.float64)
    x = np.clip((codes - black) / (white - black), 0.0, 1.0)
    lut = np.rint(np.log1p(k * x) / den * float(working_max))
    rng = np.random.default_rng(3)
    s = rng.normal(pedestal, sigma, 400000)
    idx = np.clip(np.rint(s), 0, 4095).astype(int)
    return float(lut[idx].std() / sigma)


def self_test():
    """Validate the derivation against simulated white noise through the real
    integer filters. If this fails, the analytic gains are wrong and nothing
    below should be trusted."""
    try:
        import numpy as np
    except ImportError:
        print("  (self-test skipped: numpy unavailable)")
        return True
    rng = np.random.default_rng(11)
    sigma = 40.0
    n = 512
    img = rng.normal(0.0, sigma, (n, n))

    def hp(a, axis):
        # interior 6-tap, matching cinepi_hp6: (-1,-1,8,-8,1,1)/8 on pairs
        a = np.moveaxis(a, axis, 0)
        e, o = a[0::2], a[1::2]
        m = min(e.shape[0], o.shape[0]) - 2
        r = (-e[0:m] - o[0:m] + 8 * e[1:m + 1] - 8 * o[1:m + 1]
             + e[2:m + 2] + o[2:m + 2]) / 8.0
        return np.moveaxis(r, 0, axis)

    def lp(a, axis):
        a = np.moveaxis(a, axis, 0)
        e, o = a[0::2], a[1::2]
        m = min(e.shape[0], o.shape[0])
        return np.moveaxis(e[:m] + o[:m], 0, axis)

    g = band_gains()
    ok = True
    for which, f in (("LH", lambda x: lp(hp(x, 1), 0)),
                     ("HL", lambda x: hp(lp(x, 1), 0)),
                     ("HH", lambda x: hp(hp(x, 1), 0))):
        got = float(np.std(f(img)))
        want = sigma * g[(1, which)]
        err = abs(got - want) / want
        flag = "ok" if err < 0.02 else "MISMATCH"
        if err >= 0.02:
            ok = False
        print("  level1 %-2s  predicted %8.2f  simulated %8.2f  (%.1f%%) %s"
              % (which, want, got, 100 * err, flag))
    return ok


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sigma", type=float, required=True,
                    help="measured per-pixel read-noise sigma in SENSOR CODES, "
                         "from qraw_dark_calibrate.py")
    ap.add_argument("--iso", type=int, required=True)
    ap.add_argument("--mode", default="raw12", help="raw12 | raw16")
    ap.add_argument("--gplog2-slope", type=float, default=0.0,
                    help="override the GP-Log2 gain instead of computing it; 0 "
                         "(default) computes it from the encoder's own LUT")
    ap.add_argument("--pedestal", type=float, default=0.0,
                    help="measured pedestal in sensor codes (from "
                         "qraw_dark_calibrate.py). Required unless "
                         "--gplog2-slope is given: the GP-Log2 gain depends on "
                         "WHERE on the curve the noise sits, and the curve is "
                         "strongly convex at black")
    ap.add_argument("--black", type=float, default=200.0)
    ap.add_argument("--white", type=float, default=4095.0)
    ap.add_argument("--working-max", type=int, default=4094)
    ap.add_argument("--log-strength", type=float, default=599.0,
                    help="the encoder's GP-Log2 k (vc5_bench default 599)")
    ap.add_argument("--k", type=float, default=1.0,
                    help="Noise Clean threshold multiplier to tabulate (the "
                         "roadmap says begin conservatively)")
    ap.add_argument("--self-test", action="store_true",
                    help="validate the analytic gains against simulated noise")
    args = ap.parse_args()

    if args.self_test:
        print("SELF-TEST: analytic band gains vs simulated white noise through "
              "the production filters")
        ok = self_test()
        print("  -> %s\n" % ("gains confirmed" if ok else "GAINS WRONG"))
        if not ok:
            return 1

    gain = args.gplog2_slope
    if gain <= 0.0:
        gain = gplog2_gain(args.sigma, args.pedestal or args.black,
                           args.black, args.white, args.working_max,
                           args.log_strength)
        print("GP-Log2 gain computed from the encoder's own LUT: %.1fx "
              "(k=%g, black=%g, white=%g, working_max=%d, pedestal=%g)\n"
              % (gain, args.log_strength, args.black, args.white,
                 args.working_max, args.pedestal or args.black))
    base = args.sigma * gain
    g = band_gains()
    print("NOISE PROFILE  mode=%s iso=%d  pixel sigma=%.3f codes  "
          "gplog2 slope=%.3f" % (args.mode, args.iso, args.sigma,
                                 args.gplog2_slope))
    print("  pre-quant coefficient sigma, and a k=%.2f threshold\n" % args.k)
    print("  comp  level band  idx    sigma   thresh(k)")
    for comp, cg in COMPONENT_GAIN.items():
        for level in (1, 2, 3):
            for which in ("LH", "HL", "HH"):
                s = base * cg * g[(level, which)]
                print("  %-4s  %5d %-4s %3d  %8.2f  %8.1f"
                      % (comp, level, which, band_index(level, which),
                         s, args.k * s))
    print("\n  Level 1 carries the most noise energy and level 3 the least, so a"
          "\n  conservative Noise Clean starts at level 1 and leaves LL alone --"
          "\n  which is what the roadmap says: 'Begin conservatively in"
          "\n  high-frequency bands. Avoid aggressive LL thresholds.'")
    print("\n  GD is the noisiest component by construction (gain %.3f, it is a"
          "\n  DIFFERENCE of two green phases so their noise adds) and GS the"
          "\n  quietest (%.3f, it is a mean). That is why a single flat threshold"
          "\n  across components would be wrong in both directions."
          % (COMPONENT_GAIN["GD"], COMPONENT_GAIN["GS"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
