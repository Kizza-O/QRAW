/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2026 Kieran Olsson
 */
/* ===========================================================================
 * cinepi_qraw_encoder.cpp -- library face of the benchmark's encoder.
 *
 * This deliberately reuses vc5_bench.cpp rather than copying pieces out of
 * it. That file is what the correctness gates prove and what every number in
 * the campaign was measured on; a second copy would drift from it within a
 * week and the gates would no longer be talking about the shipped code.
 * CINEPI_NO_MAIN drops the benchmark's main() and leaves the pipeline.
 * ========================================================================= */
/* ---------------------------------------------------------------------------
 * ODR ISOLATION (2026-08-18). vc5_bench.cpp defines `struct Options` at global
 * scope. So does rpicam-apps (core/options.hpp), and cinepi-raw links both.
 * Two different types with the same name emit the same weak symbol
 * `_ZN7OptionsD1Ev`, the linker keeps ONE definition for both, and every
 * destruction of one of them then runs the other's destructor. rpicam's
 * Options is polymorphic and vc5's is not, so that reads a vptr that is not
 * there and frees a garbage pointer.
 *
 * It never showed up because nothing destroyed a vc5 Options: the encoder was
 * created once and the process exited without tearing it down. The moment
 * cinepi_qraw_destroy() is called for real -- which live quality-mode switching
 * in the camera UI does on every change -- it segfaults inside free().
 * Reproduced, and traced to these two symbols, with:
 *     nm -C build/cinepi/cinepi-raw | grep 'Options::~Options'
 * showing two weak definitions at different addresses.
 *
 * The fix keeps the benchmark source untouched -- it is the pinned, gate-proven
 * encoder and must not be edited -- and instead gives everything it defines its
 * own namespace here, so the symbol becomes cinepi_vc5::Options::~Options and
 * can no longer be confused with rpicam's.
 *
 * The headers vc5_bench.cpp includes are pulled in FIRST, at global scope. When
 * the file is then included inside the namespace its own #include lines expand
 * to nothing (their include guards are already defined), so no system or SDK
 * declaration ends up namespaced. This list must stay in step with the includes
 * at the top of vc5_bench.cpp.
 * ------------------------------------------------------------------------- */
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cctype>
#include <cerrno>
#include <cstdlib>
#include <ctime>
#include <cstdint>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <numeric>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <vulkan/vulkan.h>
#if defined(__linux__)
#include <malloc.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#endif
#if defined(__APPLE__)
#include <sys/sysctl.h>
#include <mach/mach.h>
#include <mach/mach_host.h>
#include <mach/mach_init.h>
#include <mach/task.h>
#include <mach/task_info.h>
#include <mach/vm_statistics.h>
#endif
extern "C" {
#include "gpr.h"
#include "vc5_encoder.h"
#include "gpr_parse_utils.h"
}

#define CINEPI_NO_MAIN 1
namespace cinepi_vc5 {
#include "vc5_bench.cpp"
}
/* The rest of this file refers to Options, ModeSpec, DirectGprEncoder and the
 * cascade entry points unqualified, exactly as before. Only the mangled names
 * change. */
using namespace cinepi_vc5;

#include "cinepi_qraw_encoder.h"

namespace {

struct EncoderImpl {
    Options o{};
    ModeSpec mode_storage{};
    const ModeSpec* mode = nullptr;
    std::vector<uint16_t> lut;
    std::vector<int16_t> coeff;
    std::vector<int16_t> planes;      // only for the unfused path
    CpuFusedContext ctx;
    std::unique_ptr<DirectGprEncoder> enc;
    /* v3.1: the register-direct (v2) emit and its nonzero-mask sidecar.
       Three optimisations live only in this emit -- ll_vqadd, emit_kind1 and
       the mask-guided entropy prefetch -- so before this the camera ran
       without them while every benchmark number described them. */
    V2Frame v2ctx;
    std::vector<uint8_t> sidecar;
    /* v3.12 direct tile-hybrid output. The wavelet narrows each high-pass row
       to int8 8x8 tiles AS IT LEAVES the vertical filter, so the full 16.6 MB
       int16 coefficient frame is never written and never read back -- the
       single largest block of encoder DRAM traffic. Implemented and bit-verified
       in the benchmark (--cpu-direct-hybrid) but never reachable from this
       library, which is the path the camera actually uses. */
    std::unique_ptr<CpuDirectHybridSlot> hybrid;
    bool direct_hybrid = false;
    size_t row_stride = 0, plane_stride = 0;
    bool fused = false, fused_split = false;
    double wavelet_ms = 0.0, entropy_ms = 0.0, dng_ms = 0.0;
    CinepiLogFn log = nullptr;
    void* log_user = nullptr;

    void say(CinepiLogLevel lvl, const std::string& msg) const {
        if (log) log(lvl, msg.c_str(), log_user);
    }
};

}  // namespace

extern "C" {

void cinepi_qraw_config_defaults(CinepiQrawConfig* cfg) {
    /* v3.10: tightly packed unless the caller says otherwise. */
    cfg->src_stride_elems = 0;
    cfg->src_crop_x = 0;
    cfg->src_crop_y = 0;
    cfg->src_shift = 0;   /* v3.13: no load-time shift by default */
    cfg->src_byteswap = 0; /* v3.14: no load-time byte-swap by default */

    if (!cfg) return;
    *cfg = CinepiQrawConfig{};
    cfg->width = 3840; cfg->height = 2160;
    cfg->bayer = CINEPI_BAYER_GBRG;
    cfg->mode = "m5";
    cfg->caq = 0;                       /* CAQ off: current behaviour exactly */
    cfg->pixel_clean = 0;               /* Pixel Clean off: bit-exact default   */
    cfg->noise_clean_mode = 0;          /* Noise Clean off in the LIBRARY default:
                                         * a caller that has not been updated must
                                         * keep its current output exactly. The
                                         * camera asks for medium explicitly. */
    cfg->noise_clean_strength = 1.0;
    cfg->noise_clean = 0;               /* Noise Clean off: sensor-specific     */
    cfg->noise_clean_k = 0.5;           /* conservative if it is turned on      */
    cfg->iso = 0;                       /* 0 = unknown, makes Noise Clean inert */
    cfg->quant = 1;
    cfg->fused_wavelet = 1;
    cfg->fused_split   = 1;
    cfg->neon_split    = 1;
    cfg->nontemporal   = 1;   // +4.3% measured
    cfg->dng_splice    = 1;   // +12.9% at m1
    cfg->splice_shared = 1;
    cfg->effective_bits = 16; // a Pi 5 unpacked stream is left-justified 16-bit
    cfg->white = 0;
    cfg->black = 0;
    cfg->compand_bits = 12;
    cfg->true_12bit = 0;
    cfg->gradation_compand = 0;
    cfg->log_strength = 0.0;
    cfg->gpr_params_json = nullptr;
}

CinepiQrawEncoder* cinepi_qraw_create(const CinepiQrawConfig* cfg,
                                    CinepiLogFn log, void* log_user) {
    if (!cfg || cfg->width <= 0 || cfg->height <= 0) return nullptr;
    if ((cfg->width & 1) || (cfg->height & 1)) return nullptr;   // Bayer quads
    auto* impl = new (std::nothrow) EncoderImpl();
    if (!impl) return nullptr;
    impl->log = log; impl->log_user = log_user;
    try {
        /* The pinned v1.16.6 winning stack requires the AArch64 NEON
         * register-direct emit to generate its entropy sidecar. Never fall
         * back to another encoder or run with an all-zero sidecar. */
        if (!CINEPI_HAVE_NEON_WAVELET) {
            impl->say(CINEPI_LOG_ERROR,
                      "v1.16.6 winning encoder requires AArch64 NEON; no fallback encoder exists");
            delete impl; return nullptr;
        }
        Options& o = impl->o;
        o.width = cfg->width; o.height = cfg->height;
        o.bayer = (cfg->bayer == CINEPI_BAYER_RGGB) ? "rggb" : "gbrg";
        o.mode = cfg->mode ? cfg->mode : "";
        /* Component-Aware Quantisation. CINEPI_CAQ overrides the caller so a
         * runner or the camera launcher can set it without a rebuild, in the same
         * way CINEPI_BAND_Q reaches band pruning. */
        o.caq = cfg->caq;
        /* Pixel Clean, same override discipline as CAQ: the environment wins so
         * a runner or the camera launcher can flip it without a rebuild. */
        o.pixel_clean = cfg->pixel_clean != 0;
        o.noise_clean = cfg->noise_clean != 0;
        o.noise_clean_k = cfg->noise_clean_k > 0.0 ? cfg->noise_clean_k : 0.5;
        o.iso = cfg->iso;
        /* Revised Noise Clean. Same override discipline as CAQ and Pixel Clean:
         * the environment wins, so a runner or the camera launcher can change the
         * profile without a rebuild, and the matrix in tools/ drives it that way. */
        o.noise_clean_mode = cfg->noise_clean_mode;
        o.noise_clean_strength = cfg->noise_clean_strength > 0.0
                               ? cfg->noise_clean_strength : 1.0;
        if (const char *nm = std::getenv("CINEPI_NOISE_CLEAN")) {
            const std::string v(nm);
            if (v == "off" || v == "0")         o.noise_clean_mode = 0;
            else if (v == "soft" || v == "1")   o.noise_clean_mode = 1;
            else if (v == "medium" || v == "2") o.noise_clean_mode = 2;
            else if (v == "strong" || v == "3") o.noise_clean_mode = 3;
        }
        if (o.noise_clean_mode < 0 || o.noise_clean_mode > 3) o.noise_clean_mode = 0;
        if (const char *ns = std::getenv("CINEPI_NOISE_CLEAN_STRENGTH")) {
            const double v = std::atof(ns);
            if (v >= 0.25 && v <= 4.0) o.noise_clean_strength = v;
        }
        if (const char *nk = std::getenv("CINEPI_NOISE_CLEAN_K")) {
            const double v = std::atof(nk);
            if (v > 0.0 && v < 8.0) o.noise_clean_k = v;
        }
        if (const char *pc = std::getenv("CINEPI_PIXEL_CLEAN")) {
            const std::string v(pc);
            o.pixel_clean = !(v == "off" || v == "0" || v.empty());
        }
        if (const char *cq = std::getenv("CINEPI_CAQ")) {
            const std::string v(cq);
            if (v == "off" || v == "0")      o.caq = CAQ_OFF;
            else if (v == "soft" || v == "1")   o.caq = CAQ_SOFT;
            else if (v == "medium" || v == "2") o.caq = CAQ_MEDIUM;
            else if (v == "strong" || v == "3") o.caq = CAQ_STRONG;
        }
        if (o.caq < CAQ_OFF || o.caq > CAQ_STRONG) o.caq = CAQ_OFF;
        o.quant = cfg->quant > 0 ? cfg->quant : 1;
        o.execution = "cpu-gpr";
        // SINGLE-ENCODER POLICY: optimisation controls are ABI placeholders only.
        // The production library always instantiates the exact v1.16.6 winner.
        o.cpu_wavelet_fused = true;
        o.cpu_split_fused   = true;
        o.cpu_split_neon    = true;
        o.cpu_nontemporal   = true;
        o.cpu_gpr_dng_splice   = true;
        o.cpu_gpr_splice_shared = true;
        o.cpu_gpr_raw_copy = false;      // the caller owns the frame
        o.cpu_gpr_helper = false;        // winner: off
        o.vle_prequant_skip = false;     // winner: off
        o.cpu_gpr_splice_reuse = false; // winner + handoff pool
        o.cpu_gpr_shared_reuse = false; // winner
        o.cpu_gpr_shared_inplace = true;
        o.cpu_vle_prefetch_distance = 128;
        o.cpu_vle_prefetch_locality = 0;
        o.cpu_input_prefetch = true;
        o.effective_bits = cfg->effective_bits > 0 ? cfg->effective_bits : 16;
        o.compand_bits = cfg->compand_bits ? cfg->compand_bits : 12;
        /* vc5_bench REJECTS anything outside 10..12 at parse time
         * (--compand-bits, and --working-max must stay 1..4095 so the int16
         * invariant peak_coeff = working_max*4 < 32767 holds). The library
         * used to take the caller's value blind, so CINEPI_QRAW_COMPAND_BITS=14
         * silently built a working_max of 16383 and overflowed every
         * coefficient. Fail closed here, exactly as the benchmark does. */
        if (o.compand_bits < 10 || o.compand_bits > 12) {
            impl->say(CINEPI_LOG_ERROR,
                      "invalid compand_bits: " + std::to_string(o.compand_bits) +
                      " (must be 10, 11 or 12)");
            delete impl; return nullptr;
        }
        o.true_12bit = cfg->true_12bit != 0;
        o.gradation_compand = cfg->gradation_compand != 0;
        if (cfg->log_strength > 0.0) o.log_strength = cfg->log_strength;
        /* v0.1 integration fix -- the settled +278,080 byte discrepancy.
         * vc5_bench's main() normalises white AFTER parsing:
         *     if (!o.white_set) o.white = (1 << o.effective_bits) - 1;
         * The library skipped that rule, so cfg->white == 0 left the
         * Options field default of 4095 while every validated benchmark
         * ran at 65535 for a 16-bit source. Verified on x86-64: with this
         * rule applied, m5 on the bundled sample converges to the
         * benchmark's 3,052,650 bytes exactly (was 3,330,730). */
        if (cfg->white > 0) o.white = cfg->white;
        else                o.white = (1 << o.effective_bits) - 1;
        if (cfg->black > 0) o.black = cfg->black;
        if (o.black < 0 || o.white <= o.black || o.white > 65535) {
            impl->say(CINEPI_LOG_ERROR,
                      "invalid black/white range: black=" + std::to_string(o.black) +
                      " white=" + std::to_string(o.white));
            delete impl; return nullptr;
        }
        if (cfg->gpr_params_json) o.gpr_params = cfg->gpr_params_json;
        if (o.compand_bits != 12) o.working_max = (1 << o.compand_bits) - 1;
        else if (o.true_12bit)    o.working_max = 4095;

        // These are globals in the benchmark; the library sets them once at
        // create() and they must agree across every encoder in the process.
        g_cpu_wavelet_use_neon   = (CINEPI_HAVE_NEON_WAVELET != 0);
        g_cpu_wavelet_kernel_v53 = true;                 // the promoted kernel
        g_cpu_wavelet_vec_blocks = o.cpu_wavelet_vec_blocks;
        g_cpu_split_neon = o.cpu_split_neon && (CINEPI_HAVE_NEON_WAVELET != 0);
        g_cpu_nontemporal_bands = o.cpu_nontemporal && (CINEPI_HAVE_STNP != 0);
        g_cpu_fused_prefetch = false;                    // measured regression
        g_v2_input_prefetch = true;                      // v1.16.6 winner
        cinepi_vle_prequant_skip = 0;                    // measured regression

        /* v3.0: these are ON by default now.
         *
         * They were compiled in but defaulted OFF "so the camera behaves
         * exactly as before" -- which meant the camera never got them. Every
         * one is bit-identical and CM5-verified, so "as before" was costing
         * throughput for no benefit and no one would have noticed, because
         * the benchmark enabled them and the camera did not.
         *
         * signlut implies scan8 implies acc64, so that single token turns on
         * the whole entropy chain; pool implies inplace for the container.
         * NOT reachable from here: vle_sidecar and the strided-input kernel.
         * Their mask and stride generation live in the v2 emit, and this
         * library uses the shipped emit -- so the camera runs without them
         * even though the benchmark measures with them. That gap is real and
         * is called out in docs/OPTIMISATIONS.md rather than papered over. */
        /* The two emits are bit-identical by construction and the gate
           proves it on both paths (tools/verify.sh, with and without --v2),
           so this changes speed and not output. */
        /* A strided source is only honoured by the v2 path, which is on. */
        o.src_stride_elems = cfg->src_stride_elems;
        o.src_crop_x       = cfg->src_crop_x;
        o.src_crop_y       = cfg->src_crop_y;
        /* v3.13: load-time sample shift, same v2-only rule as the stride.
         * black/white/effective_bits are on the POST-shift scale (see the
         * header); an MSB-justified stream needs
         * src_shift >= 16 - effective_bits or a full-scale sample would
         * index past the LUT, so that is enforced here rather than trusted. */
        if (cfg->src_shift < 0 || cfg->src_shift > 15) {
            impl->say(CINEPI_LOG_ERROR,
                      "invalid src_shift: " + std::to_string(cfg->src_shift));
            delete impl; return nullptr;
        }
        if ((cfg->src_shift > 0 || cfg->src_byteswap) &&
            cfg->src_shift + o.effective_bits < 16) {
            impl->say(CINEPI_LOG_ERROR,
                      "src_shift " + std::to_string(cfg->src_shift) +
                      " + effective_bits " + std::to_string(o.effective_bits) +
                      " < 16: a full-scale 16-bit word would index past the "
                      "companding LUT");
            delete impl; return nullptr;
        }
        o.src_shift        = cfg->src_shift;
        /* v3.14: load-time byte-swap for big-endian RAW16 DMA buffers, same
         * v2-only rule as the stride and the shift. The range guard above
         * also covers it: a swapped word spans the full 16-bit range, so
         * src_shift + effective_bits >= 16 must hold whenever it is set. */
        o.src_byteswap     = cfg->src_byteswap ? 1 : 0;
        o.cpu_v2_kernel     = true;
        o.cpu_sidecar       = true;
        o.cpu_sidecar_zskip = true;

        cinepi_vle_acc64   = 1;   /* winner prerequisite */
        cinepi_vle_scan8   = 1;   /* winner prerequisite */
        cinepi_vle_signlut = 1;   /* winner member */
        cinepi_lowpass_bulk = 0;  /* measured loser: locked OFF */
        cinepi_set_handoff_pool(true);
        g_cpu_splice_inplace = true;
        g_cpu_splice_pool = false;
        cinepi_vle_prefetch_distance = 128;
        cinepi_vle_prefetch_locality = 0;
        /* lowpass_bulk, splice_inplace and splice_pool are deliberately NOT
           enabled. An older experimental branch turned them on without checking the measured winner
           token list, without checking which tokens had actually won --
           lowpass_bulk LOST (-0.499 ms; vc5_bench's own winner resolution
           calls it out as "never won" and forces it off), and neither
           splice_pool nor a bare splice_inplace is a winner member. Enabling
           them made the camera run a configuration measurement rejected, and
           made it disagree with --cpu-winner. r22 removes those rejected
           runtime variants entirely. */
        /* v0.10: reduced compand precision without a rebuild. 12 (default)
         * keeps today's output byte-for-byte. 11 or 10 use the same GP-Log2
         * curve over a smaller code range -- with CINEPI_COMPAND_SCALE=0 the
         * absolute quants act on the smaller signal (far more zeros, far
         * smaller files, lower precision). Changes the recorded image:
         * an on-camera decision, made visible per run in the log. */
        /* --compand-quant-scale, for BOTH reduced-precision paths. It used to
         * be read only inside the CINEPI_COMPAND_BITS branch below, so the
         * cfg->compand_bits path had no way to reach it at all. */
        if (const char *cs = std::getenv("CINEPI_COMPAND_SCALE"))
            o.compand_quant_scale = !(cs[0] == '0');
        if (const char *cb = std::getenv("CINEPI_COMPAND_BITS")) {
            const int bits = std::atoi(cb);
            if (bits == 10 || bits == 11) {
                /* The recorded file stays a full 12-bit GPR -- same frame,
                 * same white point, decodes like any other 12-bit master
                 * (GPR decodes 12-bit only). What changes is the step
                 * between code values: the GP-Log2 curve is stepped to
                 * 2^(12-bits) and the quant ladder absorbs that step, so
                 * the encoder stops coding bits the image does not carry.
                 * Because the steps sit on the log curve they are roughly
                 * perceptually uniform -- unlike dropping bits from a
                 * linear frame, where the shadows pay the entire cost. */
                o.compand_bits = 12;                 // container: 12-bit
                o.compand_inframe_bits = bits;       // effective precision
                /* This path RESTORES the full 12-bit container, so the code
                 * range has to come back with it. Without this, setting both
                 * CINEPI_QRAW_COMPAND_BITS and CINEPI_COMPAND_BITS left a
                 * reduced working_max under a nominally 12-bit frame. */
                o.working_max = o.true_12bit ? 4095 : 4094;
                impl->say(CINEPI_LOG_INFO,
                          (std::string("compand: 12-bit frame at ") + cb +
                           "-bit effective precision, quant step " +
                           (o.compand_quant_scale ? "applied" : "NOT applied "
                            "(control: no size saving expected)")).c_str());
            } else if (bits != 12) {
                impl->say(CINEPI_LOG_WARN,
                          "CINEPI_COMPAND_BITS must be 10, 11 or 12; ignored");
            }
        }

        /* r22 SINGLE-ENCODER POLICY: no runtime optimisation override. */

        {
            std::string on;
            if (cinepi_vle_acc64)      on += "acc64 ";
            if (cinepi_vle_scan8)      on += (cinepi_vle_scan_width == 16
                                              ? "scan16 " : "scan8 ");
            if (cinepi_vle_signlut)    on += "signlut ";
            if (cinepi_lowpass_bulk)   on += "lowpass_bulk ";
            if (g_cpu_splice_inplace)  on += "inplace ";
            if (g_cpu_splice_pool)     on += "pool ";
            if (on.empty()) on = "(none)";
            impl->say(CINEPI_LOG_INFO,
                      ("encoder optimisations: " + on).c_str());
        }

        if (!o.mode.empty()) {
            impl->mode_storage = resolve_mode(o.mode, o.log_strength, o.working_max);
            /* Reduced CODE RANGE (cfg->compand_bits, i.e. GPR_COMPAND_BITS /
             * --compand-bits): the GP-Log2 curve keeps its shape but lands in
             * 0..1023 or 0..2047, so the coefficients are 4x/2x smaller while
             * the m1..m10 divisors are still sized for 12-bit amplitudes.
             * vc5_bench.cpp:15098 divides the ladder by 2^(12-bits) for exactly
             * this reason, and its own comment calls the unscaled behaviour
             * "a bug in effect" (v1.7.60: tiny files that were mostly detail
             * loss, not 10-bit efficiency). This block was missing here, so the
             * camera quantised 4x harder relative to signal at 10 bits than the
             * benchmark that docs/BENCHMARK_VARIABLES.md maps it to. */
            if (o.compand_bits < 12) {
                if (o.compand_quant_scale) {
                    const int sh = 12 - o.compand_bits;
                    for (auto& q : impl->mode_storage.quant_table)
                        q = std::max(1, (q + (1 << sh >> 1)) >> sh);
                }
                /* Reported like the benchmark's own "mode" event, so a run's
                 * log says which of the two it was. */
                std::string qt;
                for (size_t i = 0; i < impl->mode_storage.quant_table.size(); ++i)
                    qt += (i ? "/" : "") +
                          std::to_string(impl->mode_storage.quant_table[i]);
                impl->say(CINEPI_LOG_INFO,
                          (o.mode + (o.compand_quant_scale
                                     ? " quant-scaled-to-" : " UNSCALED-at-") +
                           std::to_string(o.compand_bits) +
                           "-bit, working_max " + std::to_string(o.working_max) +
                           ", quant table=" + qt).c_str());
                /* The container's saturation level is a fixed 4095 (see
                 * gpr_params.json / vc5_bench.cpp:9135), so a reduced code
                 * range decodes 2^(12-bits) times dark unless the reader
                 * rescales. Say so per run rather than let it look like an
                 * exposure fault. */
                impl->say(CINEPI_LOG_WARN,
                          ("compand_bits=" + std::to_string(o.compand_bits) +
                           " keeps the GP-Log2 curve but reduces the code range; "
                           "the .gpr still declares saturation 4095, so it reads "
                           + std::to_string(12 - o.compand_bits) +
                           " stop(s) dark. CINEPI_COMPAND_BITS uses the "
                           "white-point-preserving in-frame variant instead.").c_str());
            }
            if (o.compand_inframe_bits < 12 && o.compand_quant_scale) {
                // Coefficients are multiples of S; the quant ladder absorbs
                // exactly that step, so it is divided out rather than coded.
                const int S = 1 << (12 - o.compand_inframe_bits);
                for (auto& q : impl->mode_storage.quant_table)
                    q = std::min(32767, q * S);
            }
            /* Experimental per-subband quantiser override, for the band-pruning
             * candidates (E1/E2/E3/E6 of the wavelet optimisation test plan).
             *
             *   CINEPI_BAND_Q="9=32767"          E1: zero HH1
             *   CINEPI_BAND_Q="9=32767,6=32767"  E2: zero HH1 + HH2
             *
             * Band indexing follows the ladder itself (vc5_bench.cpp:2950,
             * base = 10 - 3*level, then q_lh/q_hl/q_hh):
             *
             *   0        LL3 lowpass
             *   1,2,3    level 3  LH3, HL3, HH3
             *   4,5,6    level 2  LH2, HL2, HH2
             *   7,8,9    level 1  LH1, HL1, HH1
             *
             * This is a DATA change, not a code change: it moves quantiser
             * values the m1..m10 ladder already varies, so the encoder core is
             * untouched and the bit-exactness gate still holds when unset.
             *
             * NOTE it prices only the ENTROPY half of band pruning -- a zeroed
             * band still gets computed and quantised, it just stops emitting,
             * so the reader's mask skips it 64 coefficients at a time. Any gain
             * measured here is a LOWER BOUND on true wavelet-side skipping.
             * The image change, by contrast, is exact and final. */
            if (const char *bq = std::getenv("CINEPI_BAND_Q")) {
                std::string spec(bq), applied;
                size_t pos = 0;
                while (pos < spec.size()) {
                    const size_t comma = spec.find(',', pos);
                    const std::string item = spec.substr(pos, comma - pos);
                    const size_t eq = item.find('=');
                    if (eq != std::string::npos) {
                        const int band = std::atoi(item.substr(0, eq).c_str());
                        const int qv   = std::atoi(item.substr(eq + 1).c_str());
                        if (band >= 0 && band < 10 && qv >= 1 && qv <= 32767) {
                            impl->mode_storage.quant_table[size_t(band)] = qv;
                            applied += " " + std::to_string(band) + "=" + std::to_string(qv);
                        } else {
                            impl->say(CINEPI_LOG_WARN,
                                      ("CINEPI_BAND_Q: ignored out-of-range item '" +
                                       item + "' (band 0..9, quant 1..32767)").c_str());
                        }
                    }
                    if (comma == std::string::npos) break;
                    pos = comma + 1;
                }
                if (!applied.empty())
                    impl->say(CINEPI_LOG_INFO,
                              ("BAND_QUANT_OVERRIDE (experimental, changes the image):" +
                               applied).c_str());
            }
            if (o.noise_clean_mode > 0)
                impl->say(CINEPI_LOG_INFO,
                          (std::string("NOISE_CLEAN profile=") +
                           noise_clean_mode_name(o.noise_clean_mode) +
                           " strength=" + std::to_string(o.noise_clean_strength) +
                           " (widens the encoder's own zero threshold on level-1 "
                           "detail bands -- not a denoise pass; layered on CAQ and "
                           "Pixel Clean; CHANGES the image)").c_str());
            if (o.caq != CAQ_OFF)
                impl->say(CINEPI_LOG_INFO,
                          (std::string("COMPONENT_AWARE_QUANT profile=") +
                           caq_profile_name(o.caq) +
                           " (relative modifier on the resolved m-mode table; "
                           "GS unchanged, RG/BG and GD coarsened toward fine detail; "
                           "CHANGES the image)").c_str());
            impl->mode = &impl->mode_storage;
        }

        const int pw = o.width / 2, ph = o.height / 2;
        impl->row_stride = size_t(pw);
        impl->plane_stride = impl->row_stride * size_t(ph);
        impl->lut = make_lut(o);
        /* Sized after the direct-hybrid decision below: under the hybrid the
           coefficient frame is never written and never read (encode() takes the
           else branch, and the sidecar is forced off), so the full
           plane_stride*4 int16 frame -- 16.8 MB at 3856x2180, x3 workers -- is
           pure dead RSS and dead page tables on this 4 KB-page/no-THP kernel.
           It cannot simply become empty: set_sidecar()'s coeff_base doubles as
           the enable GATE for the Core0 SB8x4 entropy assist
           (encoder.c:1772-1774 returns NULL when it is NULL), so passing
           nullptr would silently demote selector 4 to selector 3. A minimal
           non-NULL allocation keeps that gate open; nothing dereferences it
           under the hybrid, because every read is behind
           cinepi_sidecar_base != NULL (encoder.c:1811, 2841) and the mask is
           off. */

        impl->fused = cpu_fused_geometry_ok(o);
        impl->fused_split = impl->fused;
        if (!impl->fused_split) {
            impl->say(CINEPI_LOG_ERROR,
                      "v1.16.6 winning encoder requires fused-split geometry; no fallback encoder exists");
            delete impl; return nullptr;
        }

        /* The sidecar only exists on the fused-split path, because only the
           v2 emit writes the mask. Allocating it anywhere else would hand
           the entropy coder a mask describing a frame nobody wrote. */
        /* cpu_v2_kernel is REQUIRED here, not just cpu_sidecar. Only the v2
           emit writes the mask; with the shipped emit the buffer stays all
           zero, and an all-zero mask tells the entropy coder every block is
           empty, so it silently drops real coefficients. Caught by a
           negative control that flipped v2 off and got a different CRC
           instead of the shipped path's. */
        /* Direct tile-hybrid. Measured on the static reference, m7, 3 workers:
               off  36.75 fps   wavelet 42.4  entropy 25.5
               on   37.88 fps   wavelet 33.9  entropy 33.5
           with gpr_bytes IDENTICAL (2244822) -- the output is bit-exact, the
           wavelet simply stops writing a 16.6 MB frame that the entropy stage
           then reads back. Only +3% where bandwidth is plentiful, which is
           presumably why it never got promoted from the static benchmark; the
           live pipeline is bandwidth-bound, so it is worth more there.
           CINEPI_QRAW_DIRECT_HYBRID=0 disables. */
        {
            const char* e = std::getenv("CINEPI_QRAW_DIRECT_HYBRID");
            impl->direct_hybrid = impl->fused_split && o.cpu_v2_kernel &&
                                  !(e && e[0] == '0');
        }
        if (impl->direct_hybrid) {
            /* The hybrid reader does its own zero-run detection, so the sidecar
               mask must be off -- the benchmark enforces the same rule. */
            impl->o.cpu_sidecar = impl->o.cpu_sidecar_zskip = false;
            o.cpu_sidecar = o.cpu_sidecar_zskip = false;
            /* PLANE dimensions, not frame: each of the 4 Bayer planes is
               width/2 x height/2, and that is what the benchmark passes. */
            impl->hybrid.reset(new CpuDirectHybridSlot(uint32_t(o.width / 2),
                                                       uint32_t(o.height / 2)));
        }
        /* CINEPI_QRAW_HYBRID_LEAN_COEFF=0 restores the full allocation, so the
           saving can be A/B'd on ONE binary rather than across a rebuild. */
        {
            const char* e = std::getenv("CINEPI_QRAW_HYBRID_LEAN_COEFF");
            const bool lean = impl->direct_hybrid && !(e && e[0] == '0');
            impl->coeff.assign(lean ? size_t(8) : impl->plane_stride * 4u,
                               int16_t(0));
        }

        const bool want_sidecar =
            impl->fused_split && o.cpu_v2_kernel && o.cpu_sidecar;
        if (want_sidecar)
            impl->sidecar.assign(impl->coeff.size() / 8u + 64u, 0u);
        else
            impl->o.cpu_sidecar = impl->o.cpu_sidecar_zskip = false;

        impl->enc.reset(new DirectGprEncoder(o, impl->mode,
                                             impl->row_stride, impl->plane_stride));
        /* Without this the mask is generated and then ignored: the entropy
           coder never sees it, so neither sidecar_zskip nor the mask-guided
           prefetch can fire. Pointers are stable for the encoder's life --
           both vectors are sized once, above, and never reallocated. */
        impl->enc->set_sidecar(want_sidecar ? impl->sidecar.data() : nullptr,
                               impl->coeff.data());
        impl->enc->splice_enabled = o.cpu_gpr_dng_splice;
        impl->enc->splice_shared  = o.cpu_gpr_dng_splice && o.cpu_gpr_splice_shared;
    } catch (const std::exception& e) {
        impl->say(CINEPI_LOG_ERROR, std::string("create failed: ") + e.what());
        delete impl; return nullptr;
    } catch (...) {
        delete impl; return nullptr;
    }
    return reinterpret_cast<CinepiQrawEncoder*>(impl);
}

int cinepi_qraw_encode(CinepiQrawEncoder* handle, const uint16_t* raw16,
                      void** out_buffer, size_t* out_size) {
    auto* impl = reinterpret_cast<EncoderImpl*>(handle);
    if (!impl || !raw16 || !out_buffer || !out_size) return -1;
    *out_buffer = nullptr; *out_size = 0;
    try {
        const auto t0 = Clock::now();
        V2Sidecar sc{};
        const V2Sidecar* scp = nullptr;
        if (impl->o.cpu_sidecar && !impl->sidecar.empty()) {
            sc.mask_base = impl->sidecar.data();
            sc.coeff_base = impl->coeff.data();
            scp = &sc;
        }
        const size_t src_stride = impl->o.src_stride_elems
                                    ? impl->o.src_stride_elems
                                    : size_t(impl->o.width);
        if (impl->direct_hybrid) {
            v2_fused_frame_compact(impl->o, impl->mode, raw16, src_stride,
                                   impl->o.src_crop_x, impl->o.src_crop_y,
                                   impl->lut, *impl->hybrid, impl->v2ctx);
        } else {
            cpu_fused_frame_from_raw(impl->o, impl->mode, raw16, impl->lut,
                                     impl->coeff.data(), impl->plane_stride,
                                     impl->row_stride, impl->ctx,
                                     scp, &impl->v2ctx);
        }
        const auto t1 = Clock::now();
        const int64_t before = impl->enc->dng_wrap_ns;
        gpr_buffer out{nullptr, 0};
        size_t vc5_bytes = 0;
        if (impl->direct_hybrid)
            impl->enc->encode_hybrid(impl->hybrid->frame(), out, vc5_bytes);
        else
            impl->enc->encode(impl->coeff.data(), out, vc5_bytes);
        const auto t2 = Clock::now();
        if (!out.buffer || out.size == 0) return -2;

        impl->wavelet_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        impl->entropy_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
        impl->dng_ms = double(impl->enc->dng_wrap_ns - before) / 1e6;
        *out_buffer = out.buffer;
        *out_size = out.size;
        return 0;
    } catch (const std::exception& e) {
        impl->say(CINEPI_LOG_ERROR, std::string("encode failed: ") + e.what());
        return -3;
    } catch (...) {
        return -3;
    }
}

void cinepi_qraw_release(CinepiQrawEncoder* handle, void* buffer) {
    auto* impl = reinterpret_cast<EncoderImpl*>(handle);
    if (!impl || !buffer) return;
    gpr_buffer b{buffer, 0};
    impl->enc->free_buffer(b);
}

void cinepi_qraw_last_timings(const CinepiQrawEncoder* handle,
                             double* wavelet_ms, double* entropy_ms,
                             double* dng_wrap_ms) {
    const auto* impl = reinterpret_cast<const EncoderImpl*>(handle);
    if (!impl) return;
    if (wavelet_ms)  *wavelet_ms  = impl->wavelet_ms;
    if (entropy_ms)  *entropy_ms  = impl->entropy_ms;
    if (dng_wrap_ms) *dng_wrap_ms = impl->dng_ms;
}

void cinepi_qraw_destroy(CinepiQrawEncoder* handle) {
    delete reinterpret_cast<EncoderImpl*>(handle);
}

const char* cinepi_qraw_version(void) { return CINEPI_VERSION; }

}  // extern "C"
