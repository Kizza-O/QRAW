/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2026 Kieran Olsson
 */
// Pixel Clean: prove the FUSED threshold is output-equivalent to the LITERAL
// two-rule implementation in the brief (Example 3), over every divisor and every
// coefficient the encoder can produce.
//
// This is the check §20 demands ("Pixel Clean Fast must be output-equivalent to
// the existing Pixel Clean implementation before it replaces it"). The shipped
// encoder went straight to the fused form, so there is no earlier build to diff
// against -- the reference has to be the brief's own pseudocode, reproduced here
// verbatim in behaviour, and compared exhaustively.
//
// Coefficient range: the encoder's own bound proof (fused_level_init) gives
// |v| <= 12798 for interior vertical highpass and <= 6654 for the wider edge
// filters, so +-16384 covers everything with room to spare. Divisors run
// 1..32767 because 32767 is the band-pruning sentinel.
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <algorithm>

// ---- verbatim from the shipped encoder ------------------------------------
static int cpu_quantize_exact(int value, int divisor) {
    if (divisor <= 1) return value;
    int midpoint = divisor >> 1;
    if (midpoint) --midpoint;
    const int multiplier = (1 << 16) / divisor;
    int magnitude = (value < 0 ? -value : value) + midpoint;
    int q = (magnitude * multiplier) >> 16;
    return value < 0 ? -q : q;
}

static int cinepi_deadzone125_threshold(int divisor) {
    if (divisor <= 1) return 0;
    const long long t = (5LL * divisor - 1) / 8LL;
    return int(t > 32766 ? 32766 : t);
}

static int cinepi_absquant_le1_threshold(int divisor) {
    int t = 0;
    while (t < 32766 && std::abs(cpu_quantize_exact(t + 1, divisor)) <= 1) ++t;
    return t;
}

static int cinepi_pixel_clean_threshold(int divisor, bool l1_colour) {
    int t = cinepi_deadzone125_threshold(divisor);
    if (l1_colour) t = std::max(t, cinepi_absquant_le1_threshold(divisor));
    return t;
}

// FUSED, as shipped: one integer compare, then the ordinary quantiser.
static int fused(int v, int q, int pc_t) {
    if (pc_t > 0 && std::abs(v) <= pc_t) return 0;
    return cpu_quantize_exact(v, q);
}

// ---- LITERAL, as the brief writes it (Example 3) ---------------------------
// Part 1 is a PRE-quant dead-zone in coefficient units, using the brief's own
// float threshold 0.625 * q_eff and its strict `<`. Part 2 is the POST-quant
// +-1 -> 0 rule, RG/BG level 1 only.
static int literal(int v, int q, bool l1_colour) {
    const float threshold = 0.625f * float(q);
    if (q > 1 && std::fabs(float(v)) < threshold) return 0;   // part 1
    int qq = cpu_quantize_exact(v, q);
    if (l1_colour && (qq == 1 || qq == -1)) qq = 0;           // part 2
    return qq;
}

int main() {
    const int VMAX = 16384;
    long long checked = 0;
    for (int q = 1; q <= 32767; ++q) {
        for (int l1 = 0; l1 < 2; ++l1) {
            const bool l1c = l1 != 0;
            const int pc_t = cinepi_pixel_clean_threshold(q, l1c);
            for (int v = -VMAX; v <= VMAX; ++v) {
                const int a = fused(v, q, pc_t);
                const int b = literal(v, q, l1c);
                if (a != b) {
                    std::printf("MISMATCH q=%d l1=%d v=%d fused=%d literal=%d "
                                "(pc_t=%d)\n", q, l1, v, a, b, pc_t);
                    return 1;
                }
                ++checked;
            }
        }
    }
    std::printf("EQUIVALENT: %lld (coefficient, divisor, eligibility) "
                "combinations agree exactly\n", checked);
    // Report what the lever actually widens, at the divisors the m-mode ladder
    // uses, so the size/speed result has a mechanism attached.
    std::printf("\n  divisor  plain-zero-T  deadzone-T  RG/BG-L1-T\n");
    for (int q : {1, 2, 3, 4, 6, 8, 12, 16, 24, 32, 48, 64}) {
        int z = 0;
        while (z < 32766 && cpu_quantize_exact(z + 1, q) == 0) ++z;
        std::printf("  %7d  %12d  %10d  %10d\n", q, z,
                    cinepi_deadzone125_threshold(q),
                    cinepi_pixel_clean_threshold(q, true));
    }
    return 0;
}
