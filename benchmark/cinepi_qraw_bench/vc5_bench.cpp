/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2026 Kieran Olsson
 */
// CinePi VC-5 direct GPR benchmark pipeline v1.7.34 / Pi capture pipeline + V19
//
// End-to-end Raspberry Pi 5 benchmark for the single GPRAW profile:
//   vc5-444: SDK GPR components (GS/RG/BG/GD), 3-level 2/6 transform,
//            official VC-5 syntax and TIFF/DNG GPR output for Adobe decode.
// The non-conformant vc5-422 and vc5-420 reduced-chroma profiles were removed
// in this foundation; GPRAW is the main codec and the only build target.
//
// The GPU transform feeds its mapped coefficient buffers directly into the
// vendored GoPro SDK entropy and container writer. Quantisation is fused into
// the official Table 17 entropy scan. No second split, wavelet transform, SDK
// re-encode, coefficient copy, CPR packet, or VC-5 decode is performed.

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
#define _GNU_SOURCE_MEMMEM 1
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
// E2 switch, defined in vc5_encoder/encoder.c
extern int cinepi_vle_prequant_skip;
extern int cinepi_vle_skip_width;
extern int cinepi_vle_acc64;
extern int cinepi_vle_scan8;
extern int cinepi_vle_scan_width;
extern int cinepi_vle_signlut;
extern int cinepi_vle_stream_read;
extern int cinepi_vle_prefetch_distance;
extern int cinepi_vle_prefetch_locality;

extern bool cinepi_handoff_pool_active();   // defined with the pool itself
extern void cinepi_set_handoff_pool(bool on);
extern int cinepi_lowpass_bulk;
extern int cinepi_lowpass_precision_override;
#include "gpr.h"
#include "vc5_encoder.h"
#include "gpr_parse_utils.h"
}

// MoltenVK / macOS portability support. Guarded so older Vulkan headers
// (e.g. the Pi OS packages) still compile; values match the Vulkan spec.
#ifndef VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME
#define VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME "VK_KHR_portability_enumeration"
#endif
#ifndef VK_INSTANCE_CREATE_ENUMERATE_PORTABILITY_BIT_KHR
#define VK_INSTANCE_CREATE_ENUMERATE_PORTABILITY_BIT_KHR ((VkInstanceCreateFlags)0x00000001)
#endif
#define CINEPI_VK_PORTABILITY_SUBSET_NAME "VK_KHR_portability_subset"


using Clock = std::chrono::steady_clock;
using Ns = std::chrono::nanoseconds;
static constexpr const char* CINEPI_VERSION = "1.16.6";

static int64_t elapsed_ns(Clock::time_point start) {
    return std::chrono::duration_cast<Ns>(Clock::now() - start).count();
}
static double to_ms(int64_t ns) { return double(ns) / 1.0e6; }
static std::string now_local() {
    const std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);
    char b[64]{};
    std::strftime(b, sizeof(b), "%Y-%m-%dT%H:%M:%S%z", &tm);
    return b;
}

static std::mutex g_log_mu;
static void log_event(const char* level, const char* component, const std::string& text) {
    std::lock_guard<std::mutex> lock(g_log_mu);
    std::cerr << now_local() << " [" << level << "] [" << component << "] " << text << '\n';
    std::cerr.flush();
}

class TeeBuf final : public std::streambuf {
public:
    TeeBuf(std::streambuf* a, std::streambuf* b) : a_(a), b_(b) {}
protected:
    int overflow(int ch) override {
        if (ch == traits_type::eof()) return traits_type::not_eof(ch);
        const int x = a_ ? a_->sputc(char(ch)) : ch;
        const int y = b_ ? b_->sputc(char(ch)) : ch;
        return (x == traits_type::eof() || y == traits_type::eof()) ? traits_type::eof() : ch;
    }
    int sync() override {
        const int x = a_ ? a_->pubsync() : 0;
        const int y = b_ ? b_->pubsync() : 0;
        return (x == 0 && y == 0) ? 0 : -1;
    }
private:
    std::streambuf* a_;
    std::streambuf* b_;
};

struct Options {
    std::string codec = "vc5-444";
    std::string input;
    int width = 3840;
    int height = 2160;
    int effective_bits = 16;   // IMX585 source is genuine 16-bit linear
    int black = 0;
    int white = 4095;
    bool white_set = false;
    /* v1.3: the input already carries GP-Log2 values (0..working_max), as an
       IMX585 delivers at 12-bit readout for high frame rates. The compand
       LUT becomes an identity so the frame is not companded twice -- the
       exact double-compand bug this campaign already fixed once. */
    bool input_companded = false;
    /* Sensor-companded input (IMX585 ClearHDR 12-bit): decode the gradation
       curve and re-encode as GP-Log2, so the container holds what the GPR
       format expects instead of a curve it cannot declare. See make_lut. */
    bool gradation_compand = false;
    std::string bayer = "rggb";
    double log_strength = 599.0;   // GP-Log2: log(599x+1)/log(600), Protune family base 600
    int working_max = 4094;
    bool working_max_set = false;
    int quant = 4;
    /* Component-Aware Quantisation profile. 0 = off (bit-exact current
     * behaviour), 1 = soft, 2 = medium, 3 = strong. See the CAQ block below. */
    int caq = 0;
    /* Pixel Clean: dead-zone at 125% of the normal half-step, plus a post-quant
     * rule forcing quantised RG/BG level-1 coefficients of +-1 to zero. An
     * independent lever, combinable with CAQ and with band pruning (HH1
     * zeroing), and expressed as ONE exact integer threshold per band -- see
     * cinepi_pixel_clean_threshold(). */
    bool pixel_clean = false;
    /* Noise Clean: an ISO-CALIBRATED dead-zone in pre-quant coefficient units.
     * Independent of Pixel Clean and of band pruning, as the roadmap requires,
     * and OFF by default -- it is sensor-specific and it removes shadow content,
     * so it must be chosen and seen, never inherited. */
    bool noise_clean = false;
    double noise_clean_k = 0.5;
    int    iso = 0;              /* 0 = unknown, disables Noise Clean */
    /* ---- Noise Clean, revised production-shaped method -------------------
     * (QRAW_Noise_Clean_Method_and_Recommendation brief, 2026-08-21.)
     *
     * 0 off, 1 soft, 2 medium, 3 strong. This SUPERSEDES the ISO-calibrated
     * lever above as the product feature: it is band-aware, it estimates its own
     * noise floor from the frame instead of a per-ISO calibration table, and it
     * therefore works on a sensor nobody has characterised. The ISO lever is
     * kept, reachable only with CINEPI_NOISE_CLEAN_ISO=1, because its
     * measurement and its self-test (tools/qraw_noise_profile.py) are real work
     * and a calibrated floor is still the better answer WHEN it exists.
     *
     * strength is the brief's per-profile refinement scalar, 0.50..1.50. */
    int    noise_clean_mode = 0;
    double noise_clean_strength = 1.0;
    std::string mode;              // m1..m10 calibrated GPR quality ladder; empty = legacy --quant rule
    int threads = 1;
    // Pipeline auto keeps two GPU submissions available when processing slots
    // permit it, independently of the entropy-worker count. Strict-worker auto
    // remains one in-flight submission per end-to-end worker.
    int gpu_inflight = 0;
    std::string dump_coeff; // optional first measured transformed coefficient frame
    // Raspberry Pi production uses the asynchronous capture/GPU/entropy/writer
    // pipeline. strict-workers is retained only as a diagnostic/Mac-style mode.
    std::string execution = "pipeline";
    int buffers = 3;
    bool buffers_explicit = false;
    int capture_queue = 0;   // benchmark default: normal 12-frame pipeline only; -1 enables adaptive production overflow
    double capture_fps = 0.0; // 0 = uncapped throughput benchmark; >0 = paced camera simulation
    int warmup = 2;
    int frames = 300;
    double duration = 5.0;
    /* v0.29: seconds of encoding to run and DISCARD before measurement
       starts. Page faults, cold file cache, the writer's first extents and
       a still-ramping clock all land in the first seconds and understate
       the encoder. Default 1 s. */
    double warmup_seconds = 1.0;
    /* v1.9: seconds discarded at EACH end of the averaged window. The
       pipeline keeps running through them, so the counted frames are
       bracketed by steady-state work instead of by a spin-up and a drain.
       --duration still means "seconds actually averaged"; the run is
       2*trim_seconds longer. */
    double trim_seconds = 2.0;
    /* Production affinity stays on the highest cores. One-time staggering is
       now zero by default because the cyclic rendezvous owns worker phase. */
    /* v3.10: let a caller hand us a padded buffer instead of copying it.
       src_stride_elems is in SAMPLES per row; 0 means tightly packed. The
       crop selects the active window inside that buffer. Only the v2 path
       honours these -- the shipped fused_split_transform_frame has no stride
       argument -- so a strided source implies cpu_v2_kernel. */
    size_t src_stride_elems = 0;
    int    src_crop_x = 0;
    int    src_crop_y = 0;
    /* v3.13: right-shift applied IN-REGISTER to every source sample at load
       time, before the companding LUT is indexed. An IMX585 RAW12 stream
       arrives MSB-justified in 16-bit words (low 4 bits exactly zero); with
       src_shift = 4 the encoder reads that buffer where it lies and indexes
       the same 4096-entry LUT, so the staging pass that used to shift every
       sample into a tight buffer (16.8 MB read + 16.6 MB write per frame)
       disappears without growing the LUT. black/white/effective_bits are all
       on the POST-shift scale. 0 = today's behaviour, and like the stride,
       only the v2 load path honours it. */
    int    src_shift = 0;
    /* v3.14: byte-swap applied IN-REGISTER to every source sample at load
       time, before src_shift and before the companding LUT is indexed. The
       IMX585 emits BIG-ENDIAN RAW16 and RP1 has no hardware swab for 16-bit,
       so libcamera byte-swaps the whole 16.81 MB frame on the camera thread
       every frame (~6.4 ms/frame idle, worse under load). With the libcamera
       swap skipped (LIBCAMERA_RPI_SKIP_16BIT_ENDIAN_SWAP=1) the DMA buffer
       holds big-endian samples; src_byteswap = 1 folds one `rev16`-class
       register op into the LUT gather chain, which is already load-latency
       bound, so the swap becomes effectively free and the frame-wide staging
       swap (2 x 16.81 MB of DRAM traffic per frame) disappears. Order is:
       load -> byteswap -> src_shift -> LUT index. Like the stride and the
       shift, only the v2 load path honours it -- which is the path the
       production library always runs. 0 keeps today's behaviour. */
    int    src_byteswap = 0;
    bool cpu_gpr_affinity_high = false;
    int cpu_gpr_stagger_ms = 0;
    int cpu_gpr_stagger_us = -1;  // optional finer one-time phase spacing; -1 uses ms field
    bool cpu_gpr_stagger_explicit = false;
    // Production four-selected policy is now three frame owners on cores 1..3
    // plus the Core0 SB8x4 entropy helper. The fields below are retained only
    // for explicit legacy wavelet-assist diagnostics and default to inactive.
    int cpu_gpr_os_worker_ms = 100;
    // Backward-compatible parser field for legacy wavelet-assist diagnostics.
    int cpu_gpr_os_worker_duty = 50;
    // Legacy Core0 wavelet strategy. Production default is off.
    std::string core0_wavelet_strategy = "off";
    // Row transport/scheduling knobs used by the Core-0 search matrix.
    // A smaller ring has less cache footprint; a larger ring tolerates longer
    // OS interruptions before it back-pressures the frame owner.
    int core0_wavelet_ring_rows = 16;
    std::string core0_wavelet_sched = "normal"; // normal|batch|idle

    // Core-0 staged CPU pipeline experiment. Unlike the row/plane assistant,
    // this moves an entire STAGE, never part of a stage: one core owns RAW ->
    // fused wavelet, a second core owns VC-5 entropy + GPR, and cores 2/3 keep
    // their normal end-to-end workers. Coefficient slots are preallocated and
    // ownership is transferred without copying.
    std::string core0_stage_pipeline = "off"; // off|w0e1|w1e0|dual
    int core0_stage_slots = 2;                 // preallocated coeff handoff slots
    int core0_stage_normal_workers = 2;        // 0..2 complete-frame diagnostic workers on cores 2/3
    std::string core0_stage_sched = "normal"; // normal|batch|idle on physical core 0
    int core0_stage_stagger_ms = 0;            // normal-worker phase spacing / dual target phase
    bool core0_stage_phase_lock = false;       // dual: re-align pair B to pair A each frame
    int core0_stage_phase_lock_us = 1000;      // max per-frame B delay used for phase correction
    bool cpu_gpr_shared_reuse = false;         // shared verified template + retained per-encoder container

    // v1.14 near-60 experiment. The adaptive staged path keeps wavelet producers
    // on cores 0/2 but lets entropy cores 1/3 consume either producer's frames.
    // This removes the fixed-pair bottleneck when ordinary Linux work preempts
    // core 0. The coefficient handoff remains ownership-only and zero-copy.
    std::string dual_queue_policy = "preferred"; // fifo|preferred|core2
    int dual_global_slots = 4;                    // shared coefficient slots, 3..6
    int dual_core0_reserve_slots = 1;             // free slots core0 must leave for core2
    bool dual_core0_soft_reserve = false;          // reserve only while core2 is actually waiting
    bool cpu_wavelet_reuse_context = false;       // do not zero already-sized ring scratch each frame
    bool cpu_gpr_local_inplace = false;            // local splice: VC5 writes directly into final GPR payload
    bool cpu_gpr_local_inplace_pool = false;       // retain the direct-output container between frames
    int dual_core0_nice = 0;                       // 0..19, applied to core0 producer only
    int dual_core0_start_gap_us = 0;                // core0 waits this long after latest core2 wavelet start
    bool cpu_hot_lut = false;                       // exact bucket-base + nibble-delta GP-Log lookup
    bool cpu_coeff_profile = false;                  // one warmup-frame compact/sparsity diagnostic
    bool cpu_direct_hybrid = false;                  // adaptive wavelet emits tile-hybrid directly, no full int16 frame
    int dual_slot_alignment = 64;                   // adaptive coeff/sidecar allocation alignment, 64..4096
    int progress_every = 30;
    std::string shader = "shaders/vc5_forward_26_coalesced.spv";
    std::string output = "none";
    std::string gpr_params = "validated_input/gpr_params.json";
    std::string log = "vc5_bench.log";
    std::string frame_log;
    bool no_crc = true;
    bool self_test = false;
    bool verify_gpu = false;
    bool allow_software_vulkan_validation = false;
    int workgroup = 128;
    std::string barrier_scope = "global";
    std::string command_usage = "simultaneous";
    std::string dispatch_order = "serial";
    enum class ReadbackMode { Auto, Direct, Copy, VulkanCopy };
    ReadbackMode readback_mode = ReadbackMode::Auto;
    bool device_local_input = false;
    bool device_local_coeff = false;
    // Tile-direct hybrid output can either be entropy-coded directly from the
    // Vulkan mapping or copied sequentially into normal cacheable ARM memory.
    // Auto selects the cacheable snapshot in the staged Pi pipeline whenever
    // the Vulkan coefficient allocation is not HOST_CACHED.
    enum class HybridHandoff { Auto, DirectMapped, CacheableSnapshot };
    HybridHandoff hybrid_handoff = HybridHandoff::Auto;
    int snapshot_jobs = 0; // 0 = auto: processing slots + one spare job
    bool tiled_dispatch = false;
    bool fused2d_dispatch = false;  // one horizontal+vertical dispatch per level
    bool require_v3d = false;
    enum class CoeffStorage { Int16, GpuFullPersistent, CpuHybridBand, CpuHybridTile, GpuHybridV15, GpuHybridOnePass, GpuHybridFusedMirror, GpuHybridTileDirect };
    // The raw native binary retains an int16-safe fallback default. The normal
    // macOS launcher/benchmark selects F by explicitly passing tile-direct,
    // scan-linear layout and the band-fast reader. D remains selectable.
    CoeffStorage coeff_storage = CoeffStorage::Int16;
    enum class TileReader { Legacy, RowPlan, Neon, BandFast, Group16x8, Group16x16, RunSpan };
    enum class TileLayout { FixedSlots, ScanLinear };
    enum class TileFlagLayout { BitmapAtomic, U32PerTile };
    // Plane-major is the shipped v1.7.34 order: 3 levels per plane, 8 intervening
    // barriers. Level-major issues all four planes at a level before advancing,
    // which needs only 2 barriers and batches the tiny level-3 dispatches.
    enum class DispatchSchedule { PlaneMajor, LevelMajor };
    DispatchSchedule dispatch_schedule = DispatchSchedule::PlaneMajor;
    TileReader tile_reader = TileReader::Neon;
    TileLayout tile_layout = TileLayout::FixedSlots;
    TileFlagLayout tile_flag_layout = TileFlagLayout::BitmapAtomic;
    std::string int8_range_shader = "shaders/int8_band_range.spv";
    std::string int8_pack_shader = "shaders/int8_band_pack.spv";
    std::string int8_onepass_shader = "aux_shaders/int8_onepass_fixed.spv";
    std::string tile_direct_shader = "shaders/vc5_forward_26_fused2d_tile_direct.spv";
    // Output pairs per workgroup axis for the tile-direct fused shader. 8 is the
    // shipped and v20 geometry; 16 selects the taller v21 16x16 variant.
    int tile_direct_workgroup = 8;
    int tile_direct_workgroup_y = 0; // 0 means square: use X value
    int tile_direct_pairs_x = 1; // output pairs owned by each invocation in X
    int tile_direct_pairs_y = 1; // output pairs owned by each invocation in Y
    // Per-dispatch GPU timestamps. Off by default because writing a timestamp
    // between dispatches can act as a synchronisation point on some drivers,
    // which perturbs exactly the inter-dispatch overlap that level-major aims
    // to create. Use it to find hotspots, not to decide schedules.
    bool gpu_trace = false;
    bool planes_in_z = false;
    bool shader_explicit = false;
    bool tile_direct_shader_explicit = false;
    bool hybrid_verify_baseline = false;
    int gpu_levels = 3; // 0=CPU-only control, 1=GPU L1 + CPU L2/L3, 3=all levels on GPU

    // v1.7.51 wide-separable transform. Replaces the single fused2d dispatch per
    // level with two barrier-free, shared-memory-free dispatches (v101 horizontal,
    // v102 vertical) plus one int8/int16 tile packer (v103). Output ABI is
    // byte-identical to the tile-direct hybrid path, so entropy, handoff, the CRC
    // gates and the fps accounting are all unchanged.
    // See V1_7_51_V3D_SHARED_MEMORY_ROOT_CAUSE.md.
    // scalar | neon | auto (auto = neon where the target has it). The scalar path
    // is the bit-exact reference; self_test() proves neon == scalar before any run.
    std::string cpu_wavelet = "auto";
    // v1.7.53. v53 is the row-major vertical pass; v52 is the shipped
    // column-block sweep, kept so the change can be A/B'd on hardware.
    std::string cpu_wavelet_kernel = "v53";
    // v1.7.54. The fused cascade streams RAW or split planes through all three
    // levels in one pass, keeping every intermediate in cache.
    bool cpu_wavelet_fused = true;
    bool cpu_split_fused = true;
    // v1.7.55. Non-temporal (STNP) coefficient band writes. Off by default:
    // STNP is a hint the core may ignore, so it must be measured, not assumed.
    // PROMOTED. Non-temporal (STNP) coefficient band writes.
    //
    // Rejected in round 5 as a null result -- correctly, on the coefficient
    // bands of the time. The v1.7.67 ladder more than doubled them, and on the
    // CM5 at m5 it then measured +4.30%: five runs spanning 31.76-32.47 fps
    // against a baseline's 30.43-31.70, with no overlap between the two sets.
    // Rotated slots, discarded warm-up, median-spread floor.
    //
    // Bit-exact either way: the fused crosschecks run every case with the hint
    // on and off (84 and 32 cases). STNP remains an architectural HINT the
    // core may ignore, so --cpu-nontemporal off restores the old behaviour.
    bool cpu_nontemporal = true;
    // The RAW copy simulates a DMA the real capture path performs into the
    // buffer directly. It costs ~8% of frame time and does not exist in
    // production, so it can be measured out.
    bool cpu_gpr_raw_copy = true;
    // v1.7.58 optimisation candidates. Each is off by default and tested on its
    // own before any of them are combined, because effects do not always add.
    //
    // C1 malloc-tuned: the GPR output is 0.6-12 MB and is malloc'd and freed
    //    every frame. Above glibc's mmap threshold that is an mmap, a few
    //    thousand first-touch page faults, and an munmap per frame per worker.
    //    Raising the threshold makes glibc recycle the block instead. One
    //    mallopt call, no custom allocator, no ownership risk.
    // C2 affinity: pin each worker to its own core so the kernel stops migrating
    //    it away from the L2 holding its cascade rings.
    // C3 hugepages: the per-worker RAW and coefficient buffers are 16.6 MB each.
    //    4 KiB pages means ~8000 TLB entries per frame; THP means ~16.
    // C4 prefetch: the fused cascade streams RAW rows it can see coming.
    bool cpu_gpr_malloc_tuned = false;
    bool cpu_gpr_affinity = false;
    bool cpu_gpr_hugepages = false;
    bool cpu_gpr_prefetch = false;
    // C5: the DNG/TIFF container around the VC-5 payload is byte-identical
    // frame to frame apart from the payload itself, yet it is rebuilt through
    // the Adobe SDK -- under a PROCESS-WIDE mutex -- every frame (15.4 ms at
    // m1: 37% of wall time serialised). Splice the payload into a retained
    // template instead. Self-verifying: the first frames run BOTH paths and
    // must be byte-identical or the splice disables itself for the whole run.
    // PROMOTED (round 3): +12.9% at m1. Default ON -- it is self-verifying, so
    // if the spliced container ever differs from the SDK's it disables itself
    // for the run. --cpu-gpr-dng-splice off restores the SDK path.
    bool cpu_gpr_dng_splice = true;
    // True 12-bit. The shipped split stores (diff + 4096) >> 1: the >>1 throws
    // one bit of every chroma difference away to fit the GPU's 12-bit range.
    // With no GPU there is no reason to keep it. GS stays (g1+g2)>>1 (12-bit);
    // RG/BG/GD are stored UNHALVED (13-bit). This is exactly invertible:
    // (g1+g2) and (g1-g2) share parity, so sum = 2*GS + (GD & 1), and r,g1,g2,b
    // recover losslessly. int16 stays safe BY CONSTRUCTION: max component 8191,
    // peak level-1 LL = 4x8191 = 32764, and pre_p2's int16 +3 gives 32767 --
    // at the edge, but inside it. int32 planes were considered and rejected:
    // they halve NEON width and double wavelet traffic for zero extra fidelity
    // once 13 bits fit int16. Coefficients grow ~1 bit on three planes, so
    // expect larger files at the same quant table.
    bool true_12bit = false;
    // Write finished GPR frames to disk (empty = off). save_gpr_limit is per
    // run, 0 = every frame.
    std::string save_gpr;
    int save_gpr_limit = 3;
    // Companded working precision. GP-Log2 does the perceptual allocation, so
    // the code range under it can shrink without touching the curve shape:
    // 12 keeps the shipped behaviour; 11 and 10 quantise the companded values
    // to 2047 / 1023. Coefficients shrink with them, so entropy has fewer bits
    // to code: smaller files and higher fps at the same quant divisors. Note
    // the M-mode quant tables are amplitude-relative, so a given mode at 10
    // bits quantises harder RELATIVE to signal than it does at 12.
    int compand_bits = 12;
    /* In-frame effective precision. The container stays a full 12-bit GPR
       (working_max unchanged, white where it belongs, GPR's 12-bit-only
       decode satisfied); what changes is the STEP between code values:
       11-bit => multiples of 2, 10-bit => multiples of 4 spread across the
       whole 0..working_max frame. The GP-Log2 curve is what makes this
       cheap perceptually -- the steps are perceptually uniform, unlike
       dropping bits off a linear frame where the shadows pay everything. */
    int compand_inframe_bits = 12;
    /* v0.18: the promoted winner pipeline. Off by default so existing runs
       are unchanged; --cpu-winner on turns the whole stack on at once. */
    /* v0.27: DEFAULT ON. The CM5-confirmed stack is the baseline for every
       benchmark, at every precision -- "how fast", "how long", and anything
       else that encodes. --cpu-winner off restores the shipped fused path
       for A/B work. */
    /* v1.16.6: DEFAULT ON. The production base is the measured 8/8
       v2/sidecar entropy stack plus shared in-place GPR output. Worker phase
       is handled by the cyclic rendezvous and the UI's 4-core selection adds
       only the proven Core0 SB8x4 entropy assistant. --cpu-winner off remains
       available only for explicit A/B work. */
    bool cpu_winner = true;
    /* v0.28: each late-added member independently switchable, so a
       regression can be bisected instead of argued. -1 = follow the winner
       default, 0 = force off, 1 = force on. */
    int win_lowpass_bulk = -1;
    int win_vle_signlut  = -1;
    int win_vle_acc64    = -1;   // v1.1.1: prerequisite of the winner's entropy
    int win_vle_scan8    = -1;   // v1.1.1: signlut rides on this
    int win_handoff_pool = -1;
    // v1.15 entropy/memory A/B controls. These do not change coefficients or
    // syntax; they only change where the VC-5 payload lands and the cache hint
    // used while the sidecar guides the entropy scan.
    bool cpu_gpr_shared_inplace = true;
    int cpu_vle_prefetch_distance = 128; // coefficients ahead; 0 disables
    int cpu_vle_prefetch_locality = 0;   // __builtin_prefetch locality 0..3
    bool cpu_v2_kernel = false;       // stride-aware split, register-direct emit
    bool cpu_input_prefetch = false;  // v1.0: PLDL1STRM on the raw sensor rows
    bool cpu_sidecar = false;         // nonzero mask consumed by the entropy coder
    bool cpu_sidecar_explicit = false; // explicit off must survive --cpu-winner for direct-hybrid tests
    bool cpu_sidecar_zskip = false;   // skip stores for all-zero blocks
    std::string frame_dir;            // encode every frame in this folder
    std::string source = "sample";    // sample | folder | camera
    // The M-mode quant divisors are sized for 12-bit amplitudes. At 10 bits
    // the signal is 4x smaller but the divisors were not, so v1.7.60 quantised
    // ~4x harder RELATIVE to signal: its tiny files were mostly detail loss,
    // not 10-bit efficiency. Scaling the divisors by 2^(12-bits) keeps the
    // quantisation iso-relative to signal, so a mode means the same thing at
    // every width. On by default because the unscaled behaviour was a bug in
    // effect; off reproduces v1.7.60 exactly for comparison.
    bool compand_quant_scale = true;
    // S1: NEON-assisted split arithmetic (GBRG rows; gathers remain scalar).
    // PROMOTED (round 3): +3.8% at m1. Bit-exact against the scalar split in
    // 32 crosscheck cases covering both Bayer orders. Default ON.
    bool cpu_split_neon = true;
    // Y1: SCHED_FIFO for the workers. Needs CAP_SYS_NICE or an rtprio rlimit;
    // if the kernel refuses, the run says so once and continues at normal
    // priority, which is an honest null rather than a silent one.
    bool cpu_gpr_rt = false;
    // E1: route cpu-gpr entropy through the CPU tile hybrid packer and the
    // runspan scanner (reader_mode 6) instead of the SDK's plain int16 path.
    // Tiles whose coefficients fit int8 are packed to half the bytes before
    // the entropy coder ever reads them, and the runspan reader consumes them
    // with 64-bit accumulation. Self-verifying: the first frames produce BOTH
    // containers and must be byte-identical, or E1 disables itself for the
    // run. The quantised coefficients are identical either way, so the VC-5
    // bitstream must be too -- byte equality is the proof, not an assumption.
    bool cpu_gpr_hybrid_entropy = false;
    // v19 restores the original ladder (payload parity with the macOS
    // validation); default is the corrected ladder with m1 fixed.
    // S2: share one verified splice template across all workers.
    // Round 5: mechanism-proven (dng_wrap distributions do not overlap), fps
    // effect below the noise floor. Self-verifying like C5. Default ON.
    bool cpu_gpr_splice_shared = true;
    // E2: eight-wide zero skip in the entropy scan for prequantized bands.
    //
    // REJECTED as a default (round 6: all ten modes, interleaved, 3 repeats):
    // mean -2.74%, four modes worse beyond the noise floor. The effect tracks
    // data sparsity exactly -- m4 -6.6%, m8 -0.8%, m9 +0.7%, m10 +5.3%. When
    // the eight-group is NOT all zero the vector load, abs and horizontal
    // reduce are thrown away and the scalar loop runs anyway, so at the
    // quality end of the ladder -- where fine quantisation leaves few zero
    // runs -- the check is pure overhead.
    //
    // Default OFF. Bit-identical either way, and still available via
    // --vle-prequant-skip on for m10-style sparse work where it measured +5.3%.
    //
    // This shipped default ON before it was measured, which contaminated the
    // v1.7.73 final matrix: that run read m1 at 28.17 fps, tracking E2-on
    // (28.06) rather than the true baseline (30.00).
    bool vle_prequant_skip = false;
    // ---- capture simulation (--execution capture) --------------------------
    // Models the recorder: a camera producing frames at a fixed rate into a
    // 12-frame ring, an encoder draining it, and a large uncompressed RAM
    // buffer that absorbs the difference when the encoder cannot keep up.
    double target_fps = 30.0;      // capture rate the sensor is delivering
    double capture_seconds = 60.0; // how long to run, if RAM lasts that long
    /* What the SENSOR can actually deliver in the mode being benchmarked.
       0 = unknown/unconstrained. Neither benchmark opens a camera -- both
       replay a bundled frame -- so nothing otherwise stops a run reporting
       a rate the sensor could never produce. The IMX585 reads 16-bit to
       ~30 fps and 12-bit to 60; the runners derive this from the input and
       SENSOR_MAX_FPS overrides it for another sensor. */
    double sensor_max_fps = 0.0;
    int    ring_frames = 12;       // the primary capture ring
    long   overflow_mb = -1;       // -1 = derive from installed RAM
    /* v0.30 DEFAULT OFF. This memcpy charges the encode cores 16.8 MB of
       copying per frame (~1 GB/s of read+write traffic at 30 fps) to model
       a sensor DMA write -- but real DMA lands in DRAM without spending CPU
       cycles, so the simulation was penalising capture for work the camera
       never does. That is why "how long" trailed "how fast" even though
       both run the same encoder. Ring slots are pre-filled with real frame
       data, so the encoder still sees distinct buffers and a realistic
       working set; only the phantom CPU copy is gone. Turn it back on to
       model a source that genuinely needs a CPU copy (e.g. a USB camera). */
    bool   capture_dma_copy = false;
    // Reserve the whole overflow buffer before the take starts, the way a
    // recorder does. With it off, buffers are allocated as frames spill, which
    // charges capture for mmap and first-touch faults that a real camera never
    // pays -- the DMA writes into memory that was committed up front.
    bool   capture_prealloc = true;
    int    capture_tick_ms = 500;  // live telemetry interval, 0 = silent
    // H1: hand finished GPR buffers to a helper thread on the spare core so the
    // worker starts its next frame instead of paying for the release.
    //
    // MEASURED AND REJECTED. The idea was that releasing a multi-megabyte GPR
    // buffer -- an munmap, so a syscall plus a TLB shootdown across every core
    // -- was worth moving off the worker's critical path. It is not: on the
    // CM5 at m5 the helper cost about 2.7%, and turning it off won on every
    // repeat. The thread and its queue lock cost more than the munmap they
    // were meant to hide, and the fourth core is more useful left alone.
    //
    // It shipped ON by default before it had ever been A/B'd, which was the
    // same mistake as E2. Off now, and still available for measurement.
    bool   cpu_gpr_helper = false;
    // H2: reuse one container buffer per worker for the spliced output.
    //
    // The splice currently allocates a fresh container every frame, then
    // copies prefix + payload + suffix into it, and the caller frees it. But
    // the prefix and suffix are byte-identical every frame -- that is the
    // whole premise of the splice -- and the size is fixed, so the allocation,
    // the two constant copies and the free are all avoidable. Write the
    // payload into a buffer the worker already owns and keep it.
    //
    // Removes per frame, per worker: one multi-megabyte malloc, the matching
    // free (an munmap: syscall plus TLB shootdown across all cores), and the
    // prefix/suffix copies. H1 moved that free to a spare core; this deletes
    // it. Output is unchanged -- the same bytes, in a buffer that persists.
    bool   cpu_gpr_splice_reuse = true;
    // Workers for --execution cpu-gpr. CORE 0 is deliberately left to
    // the OS, the camera stack and the writer. On the CM5 at m5 the fourth
    // worker bought +3.2% throughput (41.24 -> 42.58 fps) and cost 29% frame
    // latency (72.0 -> 92.8 ms p50) -- a bad trade on a board that also has to
    // service capture and storage.
    int cpu_gpr_threads = 3;
    int cpu_wavelet_vec_blocks = 1;  // 4K: x8 and x16 within noise; 1080p: x8 wins clearly
    // Frame-parallel dual engine. N of the --threads strict workers run the
    // ENTIRE wavelet on the CPU on their own frames while the remaining
    // workers use the GPU. Both engines pull from one shared frame counter, so
    // aggregate throughput is the sum of what each engine actually achieves
    // under contention, not a modelled sum.
    int cpu_wavelet_workers = 0;
    bool wide_separable = false;
    bool wide_hybrid_pack = true;      // run v103; off leaves everything int16
    int  wide_rows_per_march = 32;     // coefficient rows per v102 invocation, 0 = whole strip
    std::string wide_h_shader = "shaders/vc5_forward_26_v101_wide_horizontal.spv";
    std::string wide_v_shader = "shaders/vc5_forward_26_v102_wide_vertical.spv";
    std::string wide_p_shader = "shaders/vc5_forward_26_v103_tile_hybrid_pack.spv";
};

static void usage() {
    std::cout << R"USAGE(CinePi VC-5 direct GPR benchmark v1.7.56 / all-CPU GPR pipeline

Usage:
  ./vc5_bench --input frame.raw16 --width 3840 --height 2160 --duration 5

Profile (fixed):
  vc5-444   GPR GS/RG/BG/GD component planes, three 2/6 wavelet levels,
            official VC-5 entropy syntax and TIFF/DNG GPR output

Options:
  --input PATH              little-endian uint16 Bayer frame
  --width N --height N      default 3840x2160; both must be even
  --seconds SECONDS         measured submission window, default 5; set 0 to use --frames
  --duration SECONDS        alias for --seconds
  --frames N                fixed measured frames when --duration 0, default 300
  --warmup N                warm-up frames, default 2
  --buffers N               processing pipeline ring depth, default 3
  --capture-queue auto|N    RAW16 queue ceiling; default 0 keeps the normal 12-frame benchmark pipeline
                            the normal processing pipeline remains 12 frames and protection RAM is
                            allocated lazily only when capture outruns encoding.
                            auto caps total queued RAW by memory class, leaving
                            1.5 GiB for the OS: 512 MiB on 2 GB, 2.5 GiB on 4 GB,
                            6.5 GiB on 8 GB and 14.5 GiB on 16 GB. Boards below
                            the 2 GB class get the normal 12-frame queue only.
                            Every figure is an upper bound and is clamped again
                            against MemAvailable at start-up;
                            0 disables overflow but retains the normal 12-frame queue
  --capture-fps FPS         pace capture at a camera rate (for example 24 or 30);
                            default 0 is uncapped for maximum-throughput measurement
  --threads N               strict-workers: exact end-to-end CinePi frame-processing
                            workers (1..N or max), default 1. No capture/split/GPU/writer
                            service threads are created by the encoder application.
                            pipeline: whole-frame VC-5 entropy workers.
  --execution MODE          pipeline|strict-workers|cpu-gpr, default pipeline on Pi.
                            cpu-gpr is the v1.7.55 all-CPU path: N workers, each
                            doing a whole frame RAW->split->wavelet->VC-5->GPR.
                            No Vulkan instance is created at all. Worker count is
                            --cpu-gpr-threads (default 3).
                            strict-workers makes the calling/main thread worker 0 and
                            spawns N-1 additional workers. MoltenVK and macOS may still
                            create internal driver/runtime threads.
  --gpu-inflight N|max|auto maximum submitted GPU frames awaiting completion;
                            pipeline auto uses min(2, processing slots) so GPU work
                            overlaps split/snapshot without depending on entropy workers;
                            strict-workers auto equals the worker count.
  --dump-coeff PATH         write the first measured transformed int16 coefficient
                            frame for the separate hybrid-int8 laboratory.
  --coeff-storage MODE      int16|gpu-full-persistent|cpu-hybrid-band|cpu-hybrid-tile|gpu-hybrid-onepass|gpu-hybrid-fused|gpu-hybrid-tile-direct.
                            v15 uses separate range and per-band pack passes.
                            onepass uses one fixed-layout pack+validate dispatch and one fence.
                            fused writes a duplicate int8 mirror inside the 12 fused wavelet dispatches.
                            tile-direct writes each 8x8 high-pass tile as either int8 or int16,
                            never both, and uses the tile-aware direct entropy reader.
  --int8-range-shader PATH  V15 GPU maximum-absolute-value shader
  --int8-pack-shader PATH   V15 GPU hybrid pack shader
  --int8-onepass-shader PATH V17 one-dispatch pack+validate shader
  --tile-reader MODE       legacy|rowplan|neon|bandfast|group16x8|group16x16|runspan.
                           V19 production default is bandfast with linear layout;
                           mixed bands fall back to the exact tile-aware NEON reader.
                           runspan is the V20 candidate: prequantized bands are
                           scanned 32 coefficients at a time with 64-bit word
                           tests, a uniform scan-linear band is emitted as one
                           contiguous span, and mixed bands merge adjacent
                           same-format tiles instead of losing the fast path.
  --tile-layout MODE       fixed|linear. fixed retains V17 8x8 tile slots; linear
                           writes selected coefficients in band row-scan order.
  --tile-flag-layout MODE  bitmap|u32. bitmap uses the production packed bitmap
                           and global atomicOr. u32 allocates one uint per tile,
                           lets each tile owner write 0/1 without global atomics,
                           and requires a matching no-atomic laboratory shader.
  --gpu-trace              write one GPU timestamp per dispatch instead of two per
                           frame, and report a per-dispatch breakdown keyed by
                           (plane, level). Only for the fused2d schedules. A
                           timestamp between dispatches can serialise them on some
                           drivers, so treat the absolute total under --gpu-trace
                           as an upper bound and compare schedules without it.
  --tile-direct-workgroup N 8 (default) or 16 output pairs on the X axis.
  --tile-direct-workgroup-y N Optional Y-axis override, 8 or 16. Omit for square.
  --tile-direct-pairs-x N   Output pairs owned by each invocation in X, default 1.
  --tile-direct-pairs-y N   Output pairs owned by each invocation in Y, default 1.
                           V21 is 16x16. The V22 laboratory shader is 16x8,
                           reducing local invocations from 256 to 128 and shared
                           memory from about 8.9 KiB to about 4.9 KiB. Both axes
                           must match --tile-direct-shader.
  --dispatch-schedule S    plane-major|level-major, default plane-major. The
                           fused2d schedule is bit-identical either way; level-major
                           issues all four planes at a level before advancing, using
                           2 barriers per frame instead of 8 and merging the small
                           level-2/level-3 grids into single larger dispatches.
  --planes-in-z            laboratory mode: issue four component planes as
                           dispatch Z=4, reducing a fused level-major frame from
                           12 dispatches to 3. Requires a matching planes-z shader.
  --hybrid-verify-baseline  on the first frame, encode both int16 and hybrid and
                            require byte-identical GPR output
  --effective-bits 12..16   source precision
  --black N --white N       source range used by the 12-bit log LUT
  --bayer rggb|gbrg         implemented production Bayer layouts
  --log-strength X          companding strength; default 599 = GP-Log2
                            (Protune family: y=log(1+Sx)/log(1+S); 599=GP-Log2,
                            399=GP-Log, 112=Protune, 15=legacy CinePi)
  --working-max N           default 4094 (12-bit); max 4095, int16-safe (peak coeff = working_max x4)
  --quant N                 benchmark detail divisor base, default 4; final LL stays exact
  --mode m1..m10            calibrated GPR quality ladder (overrides --quant):
                            per-level/per-band tables mirroring the GoPro
                            Filmscan..Medium ladder, scaled from the SDK
                            14-bit domain to the 11-bit companded domain.
                            m1 archival ... m5 medium; L2/L3 fixed Filmscan
  --shader PATH             vc5_forward_26 SPIR-V
  --output PATH             write the final measured frame as a genuine .GPR
  --gpr-params PATH         GPR metadata JSON; default validated_input/gpr_params.json
  --log PATH                session log
  --frame-log PATH          per-frame CSV; default <log>.frames.csv
  --progress-every N
  --crc                     opt-in full GPR CRC for diagnostics; off by default
  --no-crc                  explicitly disable diagnostic CRC
  --verify-gpu              stress all Vulkan ring slots against the CPU schedule
  --allow-software-vulkan-validation
                            validation only; never accepted as production GPU encode
  --workgroup N             int16 shader workgroup specialization: 32,64,128,256
  --barrier-scope MODE      global|buffer, default global
  --command-usage MODE      simultaneous|reusable, default simultaneous
  --dispatch-order MODE     serial|interleaved, default serial; interleaved
                            issues each level's four plane passes together,
                            6 barriers per frame instead of 24
  --readback-copy           copy full int16 coefficients into cacheable ARM memory
  --vulkan-copy-readback    append vkCmdCopyBuffer into a HOST_CACHED staging buffer
  --device-local-input      upload split planes into DEVICE_LOCAL input before GPU timestamps
  --device-local-coeff      store persistent coefficients in DEVICE_LOCAL memory; requires Vulkan copy
  --direct-mapped-read      force direct mapped-buffer entropy reads
                            default is auto: copy only when Vulkan memory is
                            host-visible but not HOST_CACHED
  --hybrid-handoff MODE     auto|direct|snapshot for tile-direct int8/int16 output;
                            snapshot copies GPU output sequentially into cacheable
                            ARM memory and releases the Vulkan slot before entropy
  --snapshot-jobs N         cacheable hybrid job pool, 0=auto (buffers+1), 2..8
  --tiled-dispatch          use line-aligned tiled dispatch required by the
                            shared-memory two-pass shader; forces serial dispatch order
  --cpu-wavelet-kernel K    v53 (default, row-major vertical) or v52 (column-block
                            sweep). Both are bit-identical to the scalar reference;
                            this selects which one runs so the change is measurable.
  --cpu-wavelet-vec N       v53 vertical columns per iteration: 1 = 8, 2 = 16 (default)
  --cpu-wavelet-fused on|off   fused cascaded CPU wavelet (default on). One
                            streaming pass instead of six; identical output.
                            Declines and falls back unless plane dimensions are
                            divisible by 8.
  --cpu-split-fused on|off  fold the GP-Log2 split into the cascade (default on),
                            removing the intermediate plane frame entirely.
  --cpu-nontemporal on|off  non-temporal (STNP) coefficient band writes, default off.
                            STNP is an architectural HINT the core may ignore, so
                            treat this as an A/B and not as an optimisation that
                            is known to work. aarch64 only.
  --cpu-gpr-threads N       workers for --execution cpu-gpr, default 3. The
                            core 0 is left to the OS, capture and writer.
  --cpu-gpr-dng-splice on|off C5: splice the VC-5 payload into a retained DNG
                            container template instead of rebuilding it through
                            the serialised Adobe SDK every frame. Self-verifying
                            byte-for-byte on the first frames; falls back if the
                            container ever differs or the payload size changes.
  --execution capture       recorder simulation: a sensor producing frames at
                            --target-fps into a --ring-frames ring, workers
                            draining it, and an overflow RAM buffer absorbing
                            the shortfall. Ends when the overflow is full or
                            --capture-seconds elapses.
  --target-fps N            capture rate, e.g. 24 25 30 48 50 60
  --capture-seconds N       run length if RAM lasts (default 60)
  --ring-frames N           primary ring, default 12
  --sensor-max-fps N        what the SENSOR can deliver in this mode; 0 (the
                            default) means unconstrained. Neither benchmark
                            opens a camera, so without this a run can report
                            a rate the sensor could never produce
  --overflow-mb N           overflow cap; default derives from installed RAM
                            (1 GB board: none. 2/4/8/16 GB: 1/3/7/15 GB.)
  --capture-dma-copy on|off model the DMA write into the buffer, default on
  --capture-prealloc on|off reserve the overflow buffer up front, default on.
                            Off allocates as frames spill, which charges the
                            take for page faults a real DMA never pays.
  --capture-tick-ms N       live telemetry interval in ms, default 500, 0 off
  --cpu-gpr-splice-reuse on|off  H2: reuse one container buffer per worker for
                            spliced output instead of allocating and freeing a
                            multi-megabyte block every frame. Default on.
  --cpu-gpr-helper on|off   H1: retire finished GPR buffers on the spare core
                            rather than in the worker. Output is unchanged.
  --vle-prequant-skip on|off  E2: eight-wide zero skip in the entropy scan for
                            prequantized bands. Bit-identical by construction;
                            off restores the one-coefficient-at-a-time scan.
  --cpu-gpr-splice-shared on|off  S2: one splice template for all workers, so
                            the SDK verification path runs 3 times per run
                            instead of 3 times per worker.
  --cpu-gpr-shared-reuse on|off  experimental: use the shared verified template
                            with one retained output container per entropy core,
                            avoiding per-frame multi-MB alloc/free and constant copies.
  --cpu-gpr-hybrid-entropy on|off  E1: int8 tile packing + runspan scanner for
                            the entropy stage. First frames byte-compared
                            against the plain path; any mismatch disables it.
  --cpu-split-neon on|off   S1: NEON split arithmetic, LUT gathers stay scalar
  --cpu-gpr-rt on|off       Y1: SCHED_FIFO workers (falls back with a notice)
  --compand-quant-scale on|off  default on: scale mode quant divisors by
                            2^(12-compand_bits) so quantisation stays constant
                            RELATIVE to signal across widths. off reproduces
                            the v1.7.60 over-quantised behaviour.
  --cpu-winner on|off       scheduling policy + sidecar emit (default on).
                            NOT the arithmetic: that is unconditional now.
                            Historic switch text follows:
                            switch: v2 fused kernel (stride-aware split, no
                            repack), register-direct emit, nonzero sidecar,
                            dead-store suppression, pooled container handoff.
                            Individual parts: --cpu-v2-kernel, --cpu-sidecar,
                            --cpu-sidecar-zskip.
  --source sample|folder|camera   where frames come from (default sample).
                            folder needs --frame-dir; camera needs a CinePi
                            capture source and is reported if unavailable.
  --frame-dir DIR           encode every .raw16/.dng frame in DIR in order
                            (implies --source folder)
  --compand-inframe-bits N  10|11|12 (default 12). EFFECTIVE precision inside a
                            full 12-bit GPR: same frame, same white point,
                            ordinary 12-bit decode; the GP-Log2 curve is
                            stepped to 2^(12-N) and the quant ladder absorbs
                            the step. This is the parameter the m5 campaign
                            measured (-33% bytes at 11, -63% at 10).
  --compand-bits N          10|11|12 (default 12). Same GP-Log2 curve, smaller
                            code range: 10-bit or 11-bit precision inside the
                            12-bit VC-5 container. Decoded values sit in the
                            reduced range; the DNG linearization owns the map.
  --true-12bit on|off       store RG/BG/GD unhalved (13-bit): losslessly
                            invertible true 12-bit, int16-safe by construction.
                            NOTE: a standard GPR decoder will reconstruct chroma
                            2x hot; the decode side must apply the new inverse.
  --save-gpr DIR            write finished GPR frames into DIR
  --save-gpr-limit N        frames to save per run (default 3, 0 = all)
  --cpu-gpr-malloc-tuned on|off   C1: recycle the big output block (mallopt)
  --warmup-seconds N        discarded before the window opens, default 1
  --trim-seconds N          discarded at EACH end of the averaged window,
                            default 2. The pipeline keeps running through
                            the trims -- the counted frames are bracketed by
                            steady-state work instead of by a spin-up and a
                            drain. --duration still means seconds AVERAGED,
                            so a run is 2*N seconds longer than it counts
  --cpu-gpr-affinity-high on|off  pin to the TOP cores and leave core 0 to
                            the OS, its timers and its IRQs. Needs
                            --cpu-gpr-affinity on. This is the layout the
                            measurements were taken with
  --cpu-gpr-stagger-ms N    legacy one-time worker start offset
  --cpu-gpr-stagger-us N    finer equivalent in microseconds; overrides stagger-ms.
                            Winner default is 0: cyclic wavelet rendezvous owns
                            phase continuously instead of a one-time stagger.
  --cpu-gpr-os-worker-ms N  legacy compatibility switch for the retired GD
                            wavelet-assist diagnostics. The production 4-core
                            policy is now three frame owners + Core0 SB8x4
                            entropy assist; this value does not control it.
  --cpu-gpr-os-worker-duty N legacy compatibility switch; ignored.
  --core0-wavelet-strategy NAME
                            Legacy diagnostic wavelet helper (default off). The extended
                            matrix tests single planes, plane pairs/triples,
                            shared/opportunistic helpers, and 1/2/3-primary scaling.
                              gd-opportunistic  one shared GD helper (current)
                              gs1/rg1/bg1/gd1   one plane from primary 0
                              gs2..gd3          one plane from 2/3 primaries
                              gs-rg1 etc.       plane combinations from primary 0
                              all1              all 4 wavelet planes from primary 0
                              *-op              one shared helper, whichever owner is ready
                              off               disable helper (4 full frame workers)
  --core0-wavelet-ring-rows N  2..64 row SPSC transport depth (default 16)
  --core0-wavelet-sched NAME    normal|batch|idle scheduling on core 0 (default normal)
                            RUN_CORE0_WAVELET_MATRIX.sh sweeps these automatically.
  --core0-stage-pipeline NAME   off|w0e1|w1e0|dual|adaptive. Dedicated staged CPU path:
                            w0e1 = core0 RAW/wavelet -> core1 entropy/GPR;
                            w1e0 reverses those stages. Cores 2/3 remain normal
                            complete-frame workers. dual = two independent
                            zero-copy pairs. adaptive = core0/core2 wavelet producers
                            feeding a shared zero-copy slot pool consumed by entropy
                            cores1/3, with work stealing so core2 can compensate when
                            Linux preempts core0.
  --core0-stage-slots N         preallocated coefficient ownership slots, 1..4.
                            2 should naturally leave core0 idle when entropy is
                            slower than wavelet; larger values test burst-ahead.
  --core0-stage-normal-workers N  0..2 complete-frame workers on cores 2/3.
                            2 is the target architecture; 0/1 are diagnostics
                            that reveal how much shared-cache contention costs.
  --core0-stage-sched NAME      normal|batch|idle policy on whichever stage is
                            physically on core0.
  --core0-stage-stagger-ms N    phase offset. In w0e1/w1e0 it offsets normal
                            cores 2/3; in dual it is pair B's target phase behind
                            pair A. 0 disables.
  --core0-stage-phase-lock on|off  dual only: correct pair-B phase every frame
                            instead of using only a startup delay.
  --core0-stage-phase-lock-us N  maximum B delay for each phase correction,
                            0..5000 us (default 1000).
  --dual-queue-policy NAME        adaptive only: fifo|preferred|core2. preferred
                            keeps entropy locality but steals when its preferred
                            producer is empty; core2 prioritises the non-OS producer.
  --dual-global-slots N           adaptive shared coefficient slots, 3..6 (default 4).
  --dual-core0-reserve-slots N    adaptive: core0 leaves this many free slots for
                            core2, 0..2. Turns core0 into an opportunistic producer.
  --dual-core0-soft-reserve on|off adaptive: when on, reserve those slots only
                            while core2 is actually waiting for one.
  --dual-core0-nice N             adaptive: nice value 0..19 on core0 producer.
  --dual-core0-start-gap-us N     adaptive: core0 does not begin a wavelet until
                            N microseconds after core2's latest wavelet start. 0..30000.
                            Core2 never waits, so this is an OS-friendly one-way phase guard.
  --dual-slot-alignment N         adaptive coefficient/sidecar alignment: 64,256,4096.
  --cpu-hot-lut on|off            exact cache-compressed GP-Log LUT. Keeps an 8 KB
                            bucket-base + 32 KB nibble-delta representation hot, with
                            exact low-end fallback; verified against every source code.
  --cpu-coeff-profile on|off       adaptive: inspect one warmup coefficient frame
                            for nonzero density and 8x8 int8-tile eligibility. Not timed.
  --cpu-direct-hybrid on|off       cpu-gpr and adaptive: wavelet writes high-pass coefficients
                            directly into int8 tiles with exact int16 fallback; removes
                            the full 15.8 MiB int16 coefficient-frame handoff.
  --cpu-wavelet-reuse-context on|off  reuse already-sized private wavelet rings
                            instead of zero-filling ~scratch every frame.
  --cpu-gpr-local-inplace on|off  write VC-5 directly into the learned local GPR
                            payload position, eliminating the per-frame payload copy.
  --cpu-gpr-local-inplace-pool on|off  retain that direct-output container.
  --cpu-gpr-shared-inplace on|off write VC-5 directly into the learned shared
                            GPR payload slot; with handoff_pool the final container
                            is returned and reused without a payload memcpy.
  --cpu-vle-prefetch-distance N sidecar entropy coefficient prefetch distance in
                            coefficients; 0 disables, default 128.
  --cpu-vle-prefetch-locality N GCC/ARM prefetch locality 0..3; default 0 streaming.
  --cpu-gpr-affinity on|off       C2: pin each worker to its own core
  --cpu-gpr-hugepages on|off      C3: THP on the per-worker frame buffers
  --cpu-gpr-prefetch on|off       C4: prefetch RAW rows in the fused cascade
  --cpu-gpr-raw-copy on|off default on. The copy simulates a capture DMA that in
                            production writes into the buffer directly, so `off`
                            is the production-equivalent measurement, not a cheat.
  --cpu-wavelet-workers N   frame-parallel dual engine. N of the --threads strict
                            workers do the WHOLE wavelet on the CPU on their own
                            frames; the rest use the GPU. Both pull from one frame
                            counter, so the reported aggregate fps is measured, not
                            modelled. Requires --execution strict-workers and a
                            coefficient storage the CPU can produce on its own
                            (int16, gpu-full-persistent, cpu-hybrid-band,
                            cpu-hybrid-tile). Reports DUAL_ENGINE gpu_fps/cpu_fps.
  --gpu-levels N            0 CPU-only control, 1 GPU level 1 plus CPU levels 2/3, or 3 all GPU
  --cpu-wavelet MODE        scalar|neon|auto (default auto). The CPU wavelet used by
                            --gpu-levels 0/1 and by the CPU reference. The scalar path
                            walks DOWN A COLUMN on the vertical pass (stride between
                            successive loads); the NEON path vectorises across
                            contiguous columns instead and is bit-identical -- proved
                            by self_test() on every run. Use scalar to A/B the win.
  --wide-separable          v1.7.51 wide-transaction transform: two barrier-free,
                            shared-memory-free dispatches per level (v101 horizontal +
                            v102 vertical) plus the v103 int8/int16 tile packer, all
                            using 16-byte uvec4 accesses. Output is byte-identical to
                            the fused tile-direct hybrid path, so entropy, handoff and
                            every correctness gate are unchanged. Requires
                            --coeff-storage gpu-hybrid-tile-direct --tile-layout linear
                            --tile-flag-layout bitmap. Ignores --planes-in-z (it always
                            batches the four planes in the dispatch Z axis) and ignores
                            --tile-direct-workgroup: the wide shaders' workgroup size is
                            compiled in with -DWG= and read back from the SPIR-V.
  --wide-rows-per-march N   coefficient rows per v102 invocation (default 32, 0 = whole
                            column strip). Trades invocation count against per-segment
                            recurrence seeding: 8 extra row loads per segment, so ~12.5%
                            overhead at 32 and ~6.25% at 64. Sweep it.
  --no-wide-hybrid-pack     skip v103; every tile stays int16 fallback. Costs the CPU a
                            2x larger snapshot copy and entropy read, saves GPU time and
                            DRAM traffic. Measure both.
  --wide-h-shader PATH      v101 horizontal SPIR-V
  --wide-v-shader PATH      v102 vertical SPIR-V
  --wide-p-shader PATH      v103 tile packer SPIR-V
                            The workgroup size is compiled into these (-DWG=), and the
                            host reads it back from the SPIR-V rather than assuming.
  --fused2d-dispatch         use the experimental one-dispatch 2-D wavelet schedule:
                            12 dispatches/frame, persistent coefficient output, and
                            no full horizontal intermediate frame
  --require-v3d             reject every Vulkan device except Broadcom V3D/V3DV
  --self-test

Important:
  This executable emits genuine SDK-compatible GPR. The mapped Vulkan
  coefficient buffers are entropy-coded directly; no second SDK encode exists.
)USAGE";
}

static std::string lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c){ return char(std::tolower(c)); });
    return s;
}

struct MemoryInfo {
    uint64_t total_bytes = 0;
    uint64_t available_bytes = 0;
};

static MemoryInfo read_memory_info() {
    MemoryInfo info;
#if defined(__APPLE__)
    // macOS has no /proc/meminfo. Total physical RAM comes from sysctl hw.memsize;
    // "available" is approximated as (free + inactive + speculative) pages, the
    // pages the kernel can hand out without swapping -- the closest analogue to
    // Linux MemAvailable.
    uint64_t memsize = 0;
    size_t len = sizeof(memsize);
    if (sysctlbyname("hw.memsize", &memsize, &len, nullptr, 0) == 0) info.total_bytes = memsize;
    vm_size_t page_size = 0;
    if (host_page_size(mach_host_self(), &page_size) != KERN_SUCCESS) page_size = 4096;
    vm_statistics64_data_t vmstat{};
    mach_msg_type_number_t count = HOST_VM_INFO64_COUNT;
    if (host_statistics64(mach_host_self(), HOST_VM_INFO64,
                          reinterpret_cast<host_info64_t>(&vmstat), &count) == KERN_SUCCESS) {
        const uint64_t avail_pages = uint64_t(vmstat.free_count) +
                                     uint64_t(vmstat.inactive_count) +
                                     uint64_t(vmstat.speculative_count);
        info.available_bytes = avail_pages * uint64_t(page_size);
    }
#else
    std::ifstream f("/proc/meminfo");
    std::string key, unit;
    uint64_t value = 0;
    while (f >> key >> value >> unit) {
        const uint64_t bytes = value * 1024ull;
        if (key == "MemTotal:") info.total_bytes = bytes;
        else if (key == "MemAvailable:") info.available_bytes = bytes;
    }
#endif
    return info;
}

struct ProcessMemoryInfo {
    uint64_t rss_bytes = 0;
    uint64_t peak_rss_bytes = 0;
};

static ProcessMemoryInfo read_process_memory_info() {
    ProcessMemoryInfo info;
#if defined(__APPLE__)
    // macOS has no /proc/self/status. mach task info exposes the current resident
    // set (resident_size) and its high-water mark (resident_size_max), the direct
    // analogues of Linux VmRSS and VmHWM.
    mach_task_basic_info_data_t ti{};
    mach_msg_type_number_t count = MACH_TASK_BASIC_INFO_COUNT;
    if (task_info(mach_task_self(), MACH_TASK_BASIC_INFO,
                  reinterpret_cast<task_info_t>(&ti), &count) == KERN_SUCCESS) {
        info.rss_bytes = ti.resident_size;
        info.peak_rss_bytes = ti.resident_size_max;
    }
#else
    std::ifstream f("/proc/self/status");
    std::string key, unit;
    uint64_t value = 0;
    while (f >> key) {
        if (key == "VmRSS:" || key == "VmHWM:") {
            f >> value >> unit;
            if (key == "VmRSS:") info.rss_bytes = value * 1024ull;
            else info.peak_rss_bytes = value * 1024ull;
        } else {
            std::string rest;
            std::getline(f, rest);
        }
    }
#endif
    return info;
}

// RAW16 capture budget policy.
//
// Every figure is an upper bound. The budget for a memory class is the marketed
// size less 1.5 GiB left for the operating system, and it is then clamped again
// against MemAvailable at start-up, so a busy system yields a smaller queue
// rather than pushing the OS into reclaim and stalling the encoder.
//
// Raspberry Pi firmware reserves some RAM before Linux sees it, so MemTotal on a
// board sits a few per cent under the marketed size. Class thresholds are set
// well below each nominal size to absorb that.
struct CaptureBudgetClass {
    uint64_t min_total_mib;   // lowest MemTotal that selects this class
    uint64_t budget_mib;      // upper bound on total queued RAW16
    const char* name;
};

static constexpr CaptureBudgetClass kCaptureBudgetClasses[] = {
    { 14336ull, 14848ull, "16GB" },   // 14.5 GiB
    {  6144ull,  6656ull,  "8GB"  },   // 6.5 GiB
    {  3072ull,  2560ull,  "4GB"  },   // 2.5 GiB
    {  1536ull,   512ull,  "2GB"  },   // 512 MiB
};

// Below the smallest class there is no overflow budget at all: the encoder runs
// on the normal preallocated queue only.
//
// The 1.5 GiB operating-system allowance is expressed once, in the class budget
// above (each entry is the marketed size less 1.5 GiB). It is deliberately NOT
// subtracted a second time from MemAvailable: MemAvailable is the kernel's own
// estimate of what can be allocated without reclaim, so it already excludes what
// the OS is using. Subtracting 1.5 GiB from it as well would double-count the
// allowance and make the advertised class budgets unreachable -- on an 8 GB board
// with 7.4 GiB available it would cap the queue at 5.9 GiB rather than 6.5 GiB.
//
// What is subtracted from MemAvailable is a smaller working margin, covering the
// allocations that are not RAW16 frames (coefficient pools, GPU mirrors, GPR
// output buffers) plus slack for MemAvailable being an estimate.
static constexpr uint64_t kCaptureOsAllowanceMib = 1536ull;   // folded into the class table
static constexpr uint64_t kCaptureWorkingMarginMib = 512ull;  // held back from MemAvailable

struct CaptureBudget {
    uint64_t bytes = 0;
    const char* class_name = "minimal";
    uint64_t class_budget_mib = 0;
    bool available_limited = false;
};

// Pure policy function so it can be exercised by --self-test on any host.
static CaptureBudget capture_budget(uint64_t total_bytes, uint64_t available_bytes) {
    const uint64_t mib = 1024ull * 1024ull;
    CaptureBudget budget;
    const uint64_t total_mib = total_bytes / mib;
    for (const auto& c : kCaptureBudgetClasses) {
        if (total_mib >= c.min_total_mib) {
            budget.class_name = c.name;
            budget.class_budget_mib = c.budget_mib;
            budget.bytes = c.budget_mib * mib;
            break;
        }
    }
    if (budget.bytes == 0) return budget;

    if (available_bytes > 0) {
        const uint64_t reserve = kCaptureWorkingMarginMib * mib;
        const uint64_t headroom = available_bytes > reserve ? available_bytes - reserve : 0ull;
        if (headroom < budget.bytes) {
            budget.bytes = headroom;
            budget.available_limited = true;
        }
    }
    return budget;
}

static int resolve_capture_queue_frames(const Options& o, size_t frame_bytes) {
    if (o.capture_queue >= 0) return o.capture_queue;
    if (frame_bytes == 0) return 0;
    const MemoryInfo mem = read_memory_info();
    const CaptureBudget budget = capture_budget(mem.total_bytes, mem.available_bytes);
    return int(std::min<uint64_t>(1024ull, budget.bytes / uint64_t(frame_bytes)));
}
static Options parse_args(int argc, char** argv) {
    Options o;
    /* CAQ from the environment, so CINEPI_CAQ reaches this binary the way it
     * reaches the library and the camera.
     *
     * IT DID NOT, AND THAT MATTERED. The standalone bench had no --caq flag and
     * never read CINEPI_CAQ: only comments in this file mentioned it. So every
     * bench measurement in this package that describes itself as "CAQ strong"
     * was in fact taken with CAQ OFF -- the variable was set, exported, and
     * silently ignored, exactly like the Noise Clean prototype before it. Caught
     * on 2026-08-21 by building a GPR test set across all the quality axes and
     * finding all four CAQ values byte-identical. The live camera was never
     * affected: it drives the library, which does read it. */
    if (const char *cq = std::getenv("CINEPI_CAQ")) {
        const std::string v(cq);
        /* Literals, not CaqProfile: that enum is declared further down the file
         * than parse_args. 0..3 is its definition (CAQ_OFF..CAQ_STRONG). */
        if (v == "off" || v == "0")          o.caq = 0;
        else if (v == "soft" || v == "1")    o.caq = 1;
        else if (v == "medium" || v == "2")  o.caq = 2;
        else if (v == "strong" || v == "3")  o.caq = 3;
    }
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto need = [&](const char* n) -> std::string {
            if (i + 1 >= argc) throw std::runtime_error(std::string("Missing value for ") + n);
            return argv[++i];
        };
        if (a == "--help" || a == "-h") { usage(); std::exit(0); }
        else if (a == "--codec") o.codec = lower(need("--codec"));
        else if (a == "--input") o.input = need("--input");
        else if (a == "--width") o.width = std::stoi(need("--width"));
        else if (a == "--height") o.height = std::stoi(need("--height"));
        else if (a == "--seconds" || a == "--duration" || a == "--duration-seconds") o.duration = std::stod(need(a.c_str()));
        else if (a == "--frames") o.frames = std::stoi(need("--frames"));
        else if (a == "--warmup") o.warmup = std::stoi(need("--warmup"));
        else if (a == "--warmup-seconds") o.warmup_seconds = std::stod(need("--warmup-seconds"));
        else if (a == "--trim-seconds") o.trim_seconds = std::stod(need("--trim-seconds"));
        else if (a == "--cpu-gpr-affinity-high") o.cpu_gpr_affinity_high = (lower(need("--cpu-gpr-affinity-high")) == "on");
        else if (a == "--cpu-gpr-stagger-ms") { o.cpu_gpr_stagger_ms = std::stoi(need("--cpu-gpr-stagger-ms")); o.cpu_gpr_stagger_us = -1; o.cpu_gpr_stagger_explicit = true; }
        else if (a == "--cpu-gpr-stagger-us") { o.cpu_gpr_stagger_us = std::stoi(need("--cpu-gpr-stagger-us")); o.cpu_gpr_stagger_explicit = true; }
        else if (a == "--cpu-gpr-os-worker-ms") o.cpu_gpr_os_worker_ms = std::stoi(need("--cpu-gpr-os-worker-ms"));
        else if (a == "--cpu-gpr-os-worker-duty") o.cpu_gpr_os_worker_duty = std::stoi(need("--cpu-gpr-os-worker-duty"));
        else if (a == "--core0-wavelet-strategy") o.core0_wavelet_strategy = lower(need("--core0-wavelet-strategy"));
        else if (a == "--core0-wavelet-ring-rows") o.core0_wavelet_ring_rows = std::stoi(need("--core0-wavelet-ring-rows"));
        else if (a == "--core0-wavelet-sched") o.core0_wavelet_sched = lower(need("--core0-wavelet-sched"));
        else if (a == "--core0-stage-pipeline") o.core0_stage_pipeline = lower(need("--core0-stage-pipeline"));
        else if (a == "--core0-stage-slots") o.core0_stage_slots = std::stoi(need("--core0-stage-slots"));
        else if (a == "--core0-stage-normal-workers") o.core0_stage_normal_workers = std::stoi(need("--core0-stage-normal-workers"));
        else if (a == "--core0-stage-sched") o.core0_stage_sched = lower(need("--core0-stage-sched"));
        else if (a == "--core0-stage-stagger-ms") o.core0_stage_stagger_ms = std::stoi(need("--core0-stage-stagger-ms"));
        else if (a == "--core0-stage-phase-lock") o.core0_stage_phase_lock = (lower(need("--core0-stage-phase-lock")) == "on");
        else if (a == "--core0-stage-phase-lock-us") o.core0_stage_phase_lock_us = std::stoi(need("--core0-stage-phase-lock-us"));
        else if (a == "--dual-queue-policy") o.dual_queue_policy = lower(need("--dual-queue-policy"));
        else if (a == "--dual-global-slots") o.dual_global_slots = std::stoi(need("--dual-global-slots"));
        else if (a == "--dual-core0-reserve-slots") o.dual_core0_reserve_slots = std::stoi(need("--dual-core0-reserve-slots"));
        else if (a == "--dual-core0-soft-reserve") o.dual_core0_soft_reserve = (lower(need("--dual-core0-soft-reserve")) == "on");
        else if (a == "--dual-core0-nice") o.dual_core0_nice = std::stoi(need("--dual-core0-nice"));
        else if (a == "--dual-core0-start-gap-us") o.dual_core0_start_gap_us = std::stoi(need("--dual-core0-start-gap-us"));
        else if (a == "--dual-slot-alignment") o.dual_slot_alignment = std::stoi(need("--dual-slot-alignment"));
        else if (a == "--cpu-hot-lut") o.cpu_hot_lut = (lower(need("--cpu-hot-lut")) == "on");
        else if (a == "--cpu-coeff-profile") o.cpu_coeff_profile = (lower(need("--cpu-coeff-profile")) == "on");
        else if (a == "--cpu-direct-hybrid") o.cpu_direct_hybrid = (lower(need("--cpu-direct-hybrid")) == "on");
        else if (a == "--cpu-wavelet-reuse-context") o.cpu_wavelet_reuse_context = (lower(need("--cpu-wavelet-reuse-context")) == "on");
        else if (a == "--cpu-gpr-local-inplace") o.cpu_gpr_local_inplace = (lower(need("--cpu-gpr-local-inplace")) == "on");
        else if (a == "--cpu-gpr-local-inplace-pool") o.cpu_gpr_local_inplace_pool = (lower(need("--cpu-gpr-local-inplace-pool")) == "on");
        else if (a == "--cpu-gpr-shared-inplace") o.cpu_gpr_shared_inplace = (lower(need("--cpu-gpr-shared-inplace")) == "on");
        else if (a == "--cpu-vle-prefetch-distance") o.cpu_vle_prefetch_distance = std::stoi(need("--cpu-vle-prefetch-distance"));
        else if (a == "--cpu-vle-prefetch-locality") o.cpu_vle_prefetch_locality = std::stoi(need("--cpu-vle-prefetch-locality"));
        else if (a == "--buffers") { o.buffers = std::stoi(need("--buffers")); o.buffers_explicit = true; }
        else if (a == "--capture-queue") {
            const std::string v = lower(need("--capture-queue"));
            o.capture_queue = (v == "auto") ? -1 : std::stoi(v);
        }
        else if (a == "--capture-fps") o.capture_fps = std::stod(need("--capture-fps"));
        else if (a == "--threads") {
            const std::string v = lower(need("--threads"));
            if (v == "max") {
                const unsigned hc = std::thread::hardware_concurrency();
                o.threads = hc ? int(hc) : 1;
            } else {
                o.threads = std::stoi(v);
            }
        }
        else if (a == "--gpu-inflight") {
            const std::string v = lower(need("--gpu-inflight"));
            o.gpu_inflight = (v == "max" || v == "auto") ? 0 : std::stoi(v);
        }
        else if (a == "--dump-coeff") o.dump_coeff = need("--dump-coeff");
        else if (a == "--coeff-storage") {
            const std::string v = lower(need("--coeff-storage"));
            if (v == "int16") o.coeff_storage = Options::CoeffStorage::Int16;
            else if (v == "gpu-full-persistent" || v == "persistent-int16" || v == "v51")
                o.coeff_storage = Options::CoeffStorage::GpuFullPersistent;
            else if (v == "cpu-hybrid-band" || v == "cpu-band-hybrid")
                o.coeff_storage = Options::CoeffStorage::CpuHybridBand;
            else if (v == "cpu-hybrid-tile" || v == "cpu-hybrid" || v == "cpu-tile-hybrid")
                o.coeff_storage = Options::CoeffStorage::CpuHybridTile;
            else if (v == "gpu-hybrid-v15" || v == "gpu-hybrid-int8" || v == "hybrid-int8" || v == "int8")
                o.coeff_storage = Options::CoeffStorage::GpuHybridV15;
            else if (v == "gpu-hybrid-onepass" || v == "onepass")
                o.coeff_storage = Options::CoeffStorage::GpuHybridOnePass;
            else if (v == "gpu-hybrid-fused" || v == "fused-mirror")
                o.coeff_storage = Options::CoeffStorage::GpuHybridFusedMirror;
            else if (v == "gpu-hybrid-tile-direct" || v == "tile-direct" || v == "gpu-tile-direct")
                o.coeff_storage = Options::CoeffStorage::GpuHybridTileDirect;
            else throw std::runtime_error("--coeff-storage must be int16, gpu-full-persistent, cpu-hybrid-band, cpu-hybrid-tile, gpu-hybrid-v15, gpu-hybrid-onepass, gpu-hybrid-fused, or gpu-hybrid-tile-direct");
        }
        else if (a == "--int8-range-shader") o.int8_range_shader = need("--int8-range-shader");
        else if (a == "--int8-pack-shader") o.int8_pack_shader = need("--int8-pack-shader");
        else if (a == "--int8-onepass-shader") o.int8_onepass_shader = need("--int8-onepass-shader");
        else if (a == "--tile-direct-shader") { o.tile_direct_shader = need("--tile-direct-shader"); o.tile_direct_shader_explicit = true; }
        else if (a == "--gpu-trace") o.gpu_trace = true;
        else if (a == "--planes-in-z") o.planes_in_z = true;
        else if (a == "--cpu-wavelet") {
            o.cpu_wavelet = lower(need("--cpu-wavelet"));
            if (o.cpu_wavelet != "scalar" && o.cpu_wavelet != "neon" && o.cpu_wavelet != "auto")
                throw std::runtime_error("--cpu-wavelet must be scalar, neon or auto");
        }
        else if (a == "--wide-separable") o.wide_separable = true;
        else if (a == "--no-wide-hybrid-pack") o.wide_hybrid_pack = false;
        else if (a == "--wide-rows-per-march") {
            o.wide_rows_per_march = std::stoi(need("--wide-rows-per-march"));
            if (o.wide_rows_per_march < 0)
                throw std::runtime_error("--wide-rows-per-march must be >= 0 (0 = whole column strip)");
        }
        else if (a == "--wide-h-shader") o.wide_h_shader = need("--wide-h-shader");
        else if (a == "--wide-v-shader") o.wide_v_shader = need("--wide-v-shader");
        else if (a == "--wide-p-shader") o.wide_p_shader = need("--wide-p-shader");
        else if (a == "--tile-direct-workgroup") {
            o.tile_direct_workgroup = std::stoi(need("--tile-direct-workgroup"));
            if (o.tile_direct_workgroup != 8 && o.tile_direct_workgroup != 16)
                throw std::runtime_error("--tile-direct-workgroup must be 8 or 16");
        }
        else if (a == "--tile-direct-workgroup-y") {
            o.tile_direct_workgroup_y = std::stoi(need("--tile-direct-workgroup-y"));
            if (o.tile_direct_workgroup_y != 8 && o.tile_direct_workgroup_y != 16)
                throw std::runtime_error("--tile-direct-workgroup-y must be 8 or 16");
        }
        else if (a == "--tile-direct-pairs-x") {
            o.tile_direct_pairs_x = std::stoi(need("--tile-direct-pairs-x"));
            if (o.tile_direct_pairs_x < 1 || o.tile_direct_pairs_x > 16)
                throw std::runtime_error("--tile-direct-pairs-x must be 1..16");
        }
        else if (a == "--tile-direct-pairs-y") {
            o.tile_direct_pairs_y = std::stoi(need("--tile-direct-pairs-y"));
            if (o.tile_direct_pairs_y < 1 || o.tile_direct_pairs_y > 16)
                throw std::runtime_error("--tile-direct-pairs-y must be 1..16");
        }
        else if (a == "--dispatch-schedule") {
            const std::string v = lower(need("--dispatch-schedule"));
            if (v == "plane-major" || v == "planemajor") o.dispatch_schedule = Options::DispatchSchedule::PlaneMajor;
            else if (v == "level-major" || v == "levelmajor") o.dispatch_schedule = Options::DispatchSchedule::LevelMajor;
            else throw std::runtime_error("--dispatch-schedule must be plane-major or level-major");
        }
        else if (a == "--tile-reader") {
            const std::string v = lower(need("--tile-reader"));
            if (v == "legacy") o.tile_reader = Options::TileReader::Legacy;
            else if (v == "rowplan" || v == "row-plan") o.tile_reader = Options::TileReader::RowPlan;
            else if (v == "neon" || v == "rowplan-neon" || v == "row-plan-neon") o.tile_reader = Options::TileReader::Neon;
            else if (v == "bandfast" || v == "band-fast" || v == "whole-band") o.tile_reader = Options::TileReader::BandFast;
            else if (v == "group16x8" || v == "16x8") o.tile_reader = Options::TileReader::Group16x8;
            else if (v == "group16x16" || v == "16x16") o.tile_reader = Options::TileReader::Group16x16;
            else if (v == "runspan" || v == "run-span") o.tile_reader = Options::TileReader::RunSpan;
            else throw std::runtime_error("--tile-reader must be legacy, rowplan, neon, bandfast, group16x8, group16x16, or runspan");
        }
        else if (a == "--tile-layout") {
            const std::string v = lower(need("--tile-layout"));
            if (v == "fixed" || v == "fixed-slots") o.tile_layout = Options::TileLayout::FixedSlots;
            else if (v == "linear" || v == "scan-linear") o.tile_layout = Options::TileLayout::ScanLinear;
            else throw std::runtime_error("--tile-layout must be fixed or linear");
        }
        else if (a == "--tile-flag-layout") {
            const std::string v = lower(need("--tile-flag-layout"));
            if (v == "bitmap" || v == "bitmap-atomic" || v == "atomic")
                o.tile_flag_layout = Options::TileFlagLayout::BitmapAtomic;
            else if (v == "u32" || v == "u32-per-tile" || v == "per-tile")
                o.tile_flag_layout = Options::TileFlagLayout::U32PerTile;
            else throw std::runtime_error("--tile-flag-layout must be bitmap or u32");
        }
        else if (a == "--hybrid-verify-baseline") o.hybrid_verify_baseline = true;
        else if (a == "--execution") o.execution = lower(need("--execution"));
        else if (a == "--effective-bits") o.effective_bits = std::stoi(need("--effective-bits"));
        else if (a == "--black") o.black = std::stoi(need("--black"));
        else if (a == "--white") { o.white = std::stoi(need("--white")); o.white_set = true; }
        else if (a == "--input-companded") o.input_companded = (lower(need("--input-companded")) == "on");
        else if (a == "--gradation-compand") o.gradation_compand = (lower(need("--gradation-compand")) == "on");
        else if (a == "--src-byteswap") o.src_byteswap = (lower(need("--src-byteswap")) == "on") ? 1 : 0;
        else if (a == "--bayer") o.bayer = lower(need("--bayer"));
        else if (a == "--log-strength") o.log_strength = std::stod(need("--log-strength"));
        else if (a == "--working-max") { o.working_max = std::stoi(need("--working-max")); o.working_max_set = true; }
        else if (a == "--quant") o.quant = std::stoi(need("--quant"));
        else if (a == "--mode") o.mode = lower(need("--mode"));
        /* The three quality levers as FLAGS, not just environment variables.
         * tools/perf_ab.sh interleaves variants inside one loop and passes each
         * variant's flags to the same binary, so a lever that can only be set in
         * the environment cannot be A/B'd against itself in the way this package
         * measures everything else -- which is why the Noise Clean matrix the
         * brief asks for needs these. Spelled as the brief's section 12 spells
         * them. */
        else if (a == "--noise-clean") {
            const std::string v = lower(need("--noise-clean"));
            if (v == "off" || v == "0")         o.noise_clean_mode = 0;
            else if (v == "soft" || v == "1")   o.noise_clean_mode = 1;
            else if (v == "medium" || v == "2") o.noise_clean_mode = 2;
            else if (v == "strong" || v == "3") o.noise_clean_mode = 3;
            else throw std::runtime_error("--noise-clean must be off, soft, medium or strong");
        }
        else if (a == "--noise-clean-strength")
            o.noise_clean_strength = std::stod(need("--noise-clean-strength"));
        else if (a == "--caq") {
            const std::string v = lower(need("--caq"));
            if (v == "off" || v == "0")         o.caq = 0;
            else if (v == "soft" || v == "1")   o.caq = 1;
            else if (v == "medium" || v == "2") o.caq = 2;
            else if (v == "strong" || v == "3") o.caq = 3;
            else throw std::runtime_error("--caq must be off, soft, medium or strong");
        }
        else if (a == "--pixel-clean") {
            const std::string v = lower(need("--pixel-clean"));
            o.pixel_clean = !(v == "off" || v == "0");
        }
        else if (a == "--shader") { o.shader = need("--shader"); o.shader_explicit = true; }
        else if (a == "--output") o.output = need("--output");
        else if (a == "--gpr-params") o.gpr_params = need("--gpr-params");
        else if (a == "--log") o.log = need("--log");
        else if (a == "--frame-log") o.frame_log = need("--frame-log");
        else if (a == "--progress-every") o.progress_every = std::stoi(need("--progress-every"));
        else if (a == "--crc") o.no_crc = false;
        else if (a == "--no-crc") o.no_crc = true;
        else if (a == "--verify-gpu") o.verify_gpu = true;
        else if (a == "--allow-software-vulkan-validation") o.allow_software_vulkan_validation = true;
        else if (a == "--workgroup") o.workgroup = std::stoi(need("--workgroup"));
        else if (a == "--barrier-scope") o.barrier_scope = lower(need("--barrier-scope"));
        else if (a == "--command-usage") o.command_usage = lower(need("--command-usage"));
        else if (a == "--dispatch-order") o.dispatch_order = lower(need("--dispatch-order"));
        else if (a == "--readback-copy") o.readback_mode = Options::ReadbackMode::Copy;
        else if (a == "--vulkan-copy-readback") o.readback_mode = Options::ReadbackMode::VulkanCopy;
        else if (a == "--device-local-input") o.device_local_input = true;
        else if (a == "--device-local-coeff") o.device_local_coeff = true;
        else if (a == "--direct-mapped-read") o.readback_mode = Options::ReadbackMode::Direct;
        else if (a == "--hybrid-handoff") {
            const std::string v = lower(need("--hybrid-handoff"));
            if (v == "auto") o.hybrid_handoff = Options::HybridHandoff::Auto;
            else if (v == "direct" || v == "direct-mapped")
                o.hybrid_handoff = Options::HybridHandoff::DirectMapped;
            else if (v == "snapshot" || v == "cacheable" || v == "cacheable-snapshot")
                o.hybrid_handoff = Options::HybridHandoff::CacheableSnapshot;
            else throw std::runtime_error("--hybrid-handoff must be auto, direct, or snapshot");
        }
        else if (a == "--snapshot-jobs") o.snapshot_jobs = std::stoi(need("--snapshot-jobs"));
        else if (a == "--tiled-dispatch") o.tiled_dispatch = true;
        else if (a == "--cpu-wavelet-kernel") {
            o.cpu_wavelet_kernel = lower(need("--cpu-wavelet-kernel"));
            if (o.cpu_wavelet_kernel != "v52" && o.cpu_wavelet_kernel != "v53")
                throw std::runtime_error("--cpu-wavelet-kernel must be v52 or v53");
        }
        else if (a == "--cpu-wavelet-vec") o.cpu_wavelet_vec_blocks = std::stoi(need("--cpu-wavelet-vec"));
        else if (a == "--cpu-wavelet-fused") o.cpu_wavelet_fused = (lower(need("--cpu-wavelet-fused")) == "on");
        else if (a == "--cpu-split-fused") o.cpu_split_fused = (lower(need("--cpu-split-fused")) == "on");
        else if (a == "--cpu-gpr-dng-splice") o.cpu_gpr_dng_splice = (lower(need("--cpu-gpr-dng-splice")) == "on");
        else if (a == "--target-fps") o.target_fps = std::stod(need("--target-fps"));
        else if (a == "--capture-seconds") o.capture_seconds = std::stod(need("--capture-seconds"));
        else if (a == "--ring-frames") o.ring_frames = std::stoi(need("--ring-frames"));
        else if (a == "--sensor-max-fps") o.sensor_max_fps = std::stod(need("--sensor-max-fps"));
        else if (a == "--overflow-mb") o.overflow_mb = std::stol(need("--overflow-mb"));
        else if (a == "--cpu-gpr-splice-reuse") o.cpu_gpr_splice_reuse = (lower(need("--cpu-gpr-splice-reuse")) == "on");
        else if (a == "--cpu-gpr-helper") o.cpu_gpr_helper = (lower(need("--cpu-gpr-helper")) == "on");
        else if (a == "--capture-prealloc") o.capture_prealloc = (lower(need("--capture-prealloc")) == "on");
        else if (a == "--capture-tick-ms") o.capture_tick_ms = std::stoi(need("--capture-tick-ms"));
        else if (a == "--capture-dma-copy") o.capture_dma_copy = (lower(need("--capture-dma-copy")) == "on");
        else if (a == "--vle-prequant-skip") o.vle_prequant_skip = (lower(need("--vle-prequant-skip")) == "on");
        else if (a == "--cpu-gpr-splice-shared") o.cpu_gpr_splice_shared = (lower(need("--cpu-gpr-splice-shared")) == "on");
        else if (a == "--cpu-gpr-shared-reuse") o.cpu_gpr_shared_reuse = (lower(need("--cpu-gpr-shared-reuse")) == "on");
        else if (a == "--cpu-gpr-hybrid-entropy") o.cpu_gpr_hybrid_entropy = (lower(need("--cpu-gpr-hybrid-entropy")) == "on");
        else if (a == "--cpu-split-neon") o.cpu_split_neon = (lower(need("--cpu-split-neon")) == "on");
        else if (a == "--cpu-gpr-rt") o.cpu_gpr_rt = (lower(need("--cpu-gpr-rt")) == "on");
        else if (a == "--compand-quant-scale") o.compand_quant_scale = (lower(need("--compand-quant-scale")) == "on");
        else if (a == "--compand-bits") o.compand_bits = std::stoi(need("--compand-bits"));
        else if (a == "--cpu-winner") o.cpu_winner = (lower(need("--cpu-winner")) == "on");
        else if (a == "--win-lowpass-bulk") o.win_lowpass_bulk = (lower(need("--win-lowpass-bulk")) == "on");
        else if (a == "--win-vle-signlut")  o.win_vle_signlut  = (lower(need("--win-vle-signlut")) == "on");
        else if (a == "--win-vle-acc64")    o.win_vle_acc64    = (lower(need("--win-vle-acc64")) == "on");
        else if (a == "--win-vle-scan8")    o.win_vle_scan8    = (lower(need("--win-vle-scan8")) == "on");
        else if (a == "--win-handoff-pool") o.win_handoff_pool = (lower(need("--win-handoff-pool")) == "on");
        else if (a == "--cpu-v2-kernel") o.cpu_v2_kernel = (lower(need("--cpu-v2-kernel")) == "on");
        else if (a == "--cpu-input-prefetch") o.cpu_input_prefetch = (lower(need("--cpu-input-prefetch")) == "on");
        else if (a == "--cpu-sidecar") { o.cpu_sidecar = (lower(need("--cpu-sidecar")) == "on"); o.cpu_sidecar_explicit = true; }
        else if (a == "--cpu-sidecar-zskip") o.cpu_sidecar_zskip = (lower(need("--cpu-sidecar-zskip")) == "on");
        else if (a == "--source") o.source = lower(need("--source"));
        else if (a == "--frame-dir") { o.frame_dir = need("--frame-dir"); o.source = "folder"; }
        else if (a == "--compand-inframe-bits")
            o.compand_inframe_bits = std::stoi(need("--compand-inframe-bits"));
        else if (a == "--true-12bit") o.true_12bit = (lower(need("--true-12bit")) == "on");
        else if (a == "--save-gpr") o.save_gpr = need("--save-gpr");
        else if (a == "--save-gpr-limit") o.save_gpr_limit = std::stoi(need("--save-gpr-limit"));
        else if (a == "--cpu-gpr-malloc-tuned") o.cpu_gpr_malloc_tuned = (lower(need("--cpu-gpr-malloc-tuned")) == "on");
        else if (a == "--cpu-gpr-affinity") o.cpu_gpr_affinity = (lower(need("--cpu-gpr-affinity")) == "on");
        else if (a == "--cpu-gpr-hugepages") o.cpu_gpr_hugepages = (lower(need("--cpu-gpr-hugepages")) == "on");
        else if (a == "--cpu-gpr-prefetch") o.cpu_gpr_prefetch = (lower(need("--cpu-gpr-prefetch")) == "on");
        else if (a == "--cpu-gpr-raw-copy") o.cpu_gpr_raw_copy = (lower(need("--cpu-gpr-raw-copy")) == "on");
        else if (a == "--cpu-nontemporal") o.cpu_nontemporal = (lower(need("--cpu-nontemporal")) == "on");
        else if (a == "--cpu-gpr-threads") o.cpu_gpr_threads = std::stoi(need("--cpu-gpr-threads"));
        else if (a == "--cpu-wavelet-workers") o.cpu_wavelet_workers = std::stoi(need("--cpu-wavelet-workers"));
        else if (a == "--gpu-levels") o.gpu_levels = std::stoi(need("--gpu-levels"));
        else if (a == "--fused2d-dispatch") o.fused2d_dispatch = true;
        else if (a == "--require-v3d") o.require_v3d = true;
        else if (a == "--self-test") o.self_test = true;
        else throw std::runtime_error("Unknown option: " + a);
    }
    if (o.codec != "vc5-444")
        throw std::runtime_error("--codec must be vc5-444 (GPRAW is the only codec in this build)");
    if (o.width <= 0 || o.height <= 0 || (o.width & 1) || (o.height & 1)) throw std::runtime_error("width and height must be positive even numbers");
    if (o.width < 64 || o.height < 64) throw std::runtime_error("width and height must be at least 64 for the three-level VC-5 schedule");
    if (o.width % 16 != 0 || o.height % 16 != 0) throw std::runtime_error("width and height must be divisible by 16 for the fixed three-level schedule");
    if (o.effective_bits < 12 || o.effective_bits > 16) throw std::runtime_error("--effective-bits must be 12..16");
    if (!o.white_set) o.white = int((uint32_t(1) << unsigned(o.effective_bits)) - 1u);
    if (o.black < 0 || o.white <= o.black || o.white > 65535) throw std::runtime_error("invalid black/white range");
    if (o.white > ((1 << o.effective_bits) - 1)) throw std::runtime_error("--white exceeds --effective-bits source range");
    if (o.compand_inframe_bits < 10 || o.compand_inframe_bits > 12)
        throw std::runtime_error("--compand-inframe-bits must be 10, 11 or 12");
    if (o.compand_bits < 10 || o.compand_bits > 12)
        throw std::runtime_error("--compand-bits must be 10, 11 or 12");
    if (!o.working_max_set) {
        if (o.compand_bits != 12) o.working_max = (1 << o.compand_bits) - 1;
        else if (o.true_12bit) o.working_max = 4095; // use the full code range
    }
    if (o.working_max < 1 || o.working_max > 4095) throw std::runtime_error("--working-max must be 1..4095 for the int16-safe 12-bit VC-5 path (peak coeff = working_max x4 <= 16380 < 32767)");
    if (!(o.log_strength > 0.0) || !std::isfinite(o.log_strength)) throw std::runtime_error("invalid --log-strength");
    if (o.quant < 1 || o.quant > 1024) throw std::runtime_error("--quant must be 1..1024");
    if (!o.mode.empty() && (o.mode.size() < 2 || o.mode[0] != 'm'))
        throw std::runtime_error("--mode must be of the form mN");
    if (o.execution == "serial-workers") o.execution = "strict-workers";
    /* v0.27: expand the winner stack after parsing, not inside the flag's
       branch -- otherwise the default only applies when the flag is passed,
       which is how a bare invocation still reported 0/8. */
    if (o.cpu_winner) {
        /* v3.0: most of what this used to switch on is now unconditional --
           the wavelet arithmetic, the 6-row ring and the mask-guided entropy
           prefetch are simply the code. What is left here is the part that
           is a POLICY rather than an optimisation: how the encoder shares
           the machine, and the sidecar/emit pair that the shipped kernel can
           still be run without.

           --cpu-winner off is therefore no longer "the slow encoder". It is
           the optimised encoder without the scheduling policy and without
           the sidecar, which is occasionally what you want when comparing
           against something else. */
        o.cpu_v2_kernel = true;
        if (!o.cpu_sidecar_explicit) o.cpu_sidecar = true;
        o.cpu_sidecar_zskip = o.cpu_sidecar;
        o.cpu_input_prefetch = true;
        /* v1.9: ring6 joined the winner on 2026-08-10 (+0.653 ms solo,
           +0.568 in stack, paired, on a CM5). It lived only in the m5
           suite's OptSpec, so --cpu-winner -- which is what RUN_BENCHMARK
           and RUN_CAPTURE_TEST use -- was still running the v1.0 six-member
           stack without it. Two hand-maintained definitions of "the winner"
           is the bug; test_ui_selection.py now fails if they diverge. */
        /* v3.0: ring6, the four arith_all members, entropy_pf_mask and
           stnp_reg are no longer switches -- they are the code. Nothing to
           turn on here. */
        /* pin_high is the only pinning member of the winner, so it has to
           turn pinning ON as well as choose the layout -- exactly as it does
           in the m5 suite, where `if (pin_workers || pin_high)` gates the
           affinity call and pin_high alone is sufficient. */
        o.cpu_gpr_affinity = true;                  /* pin_high: enable */
        o.cpu_gpr_affinity_high = true;             /* pin_high: top cores */
        if (!o.cpu_gpr_stagger_explicit)
            o.cpu_gpr_stagger_ms = 0;               /* rendezvous owns phase */
        /* v0.28: only surrender splice_reuse when handoff_pool is actually
           taking over. Forcing it off unconditionally cost throughput
           wherever handoff_pool does not apply -- splice_reuse is a real
           optimisation and was being removed for nothing. */
        if (o.win_handoff_pool != 0) o.cpu_gpr_splice_reuse = false;
    }
    /* --cpu-direct-hybrid replaces the int16 coefficient frame with the
       compact tile pool, so there is no int16 buffer for the E8 sidecar to
       describe. The tile pool carries its own per-coefficient mask instead
       (see cinepi_wav_nzmask_enabled), which the hybrid reader consumes. An
       explicit `--cpu-sidecar on` alongside it is a contradiction, not a
       default to be silently overridden. */
    if (o.cpu_direct_hybrid) {
        if (o.cpu_sidecar_explicit && o.cpu_sidecar)
            throw std::runtime_error("--cpu-direct-hybrid cannot be combined with --cpu-sidecar on; "
                                     "the tile-hybrid pool has no int16 coefficient frame for the E8 mask "
                                     "(the wavelet emits the per-tile nonzero mask instead)");
        o.cpu_sidecar = false;
        o.cpu_sidecar_zskip = false;
    }
    if ((o.execution == "cpu-gpr" || o.execution == "capture") &&
        (o.cpu_gpr_threads < 1 || o.cpu_gpr_threads > 4))
        throw std::runtime_error("--cpu-gpr-threads must be 1..4 on the Pi encoder path");
    if (o.cpu_gpr_os_worker_ms < 0)
        throw std::runtime_error("--cpu-gpr-os-worker-ms must be >= 0");
    if (o.cpu_gpr_os_worker_duty < 1 || o.cpu_gpr_os_worker_duty > 100)
        throw std::runtime_error("--cpu-gpr-os-worker-duty must be 1..100");
    {
        static const std::array<const char*,35> allowed{{
            "off",
            "gs-op", "rg-op", "bg-op", "gd-opportunistic",
            "gs-rg-op", "gs-bg-op", "gs-gd-op", "rg-bg-op", "rg-gd-op", "bg-gd-op", "all-op",
            "gs1", "gs2", "gs3", "rg1", "rg2", "rg3", "bg1", "bg2", "bg3", "gd1", "gd2", "gd3",
            "gs-rg1", "gs-bg1", "gs-gd1", "rg-bg1", "rg-gd1", "bg-gd1",
            "gs-rg-bg1", "gs-rg-gd1", "gs-bg-gd1", "rg-bg-gd1", "all1"}};
        if (std::find_if(allowed.begin(), allowed.end(), [&](const char* v){
                return o.core0_wavelet_strategy == v; }) == allowed.end())
            throw std::runtime_error("unknown --core0-wavelet-strategy (run --help for the extended matrix names)");
    }
    if (o.core0_wavelet_ring_rows < 2 || o.core0_wavelet_ring_rows > 64)
        throw std::runtime_error("--core0-wavelet-ring-rows must be 2..64");
    if (o.core0_wavelet_sched != "normal" && o.core0_wavelet_sched != "batch" && o.core0_wavelet_sched != "idle")
        throw std::runtime_error("--core0-wavelet-sched must be normal, batch, or idle");
    if (o.core0_stage_pipeline != "off" && o.core0_stage_pipeline != "w0e1" &&
        o.core0_stage_pipeline != "w1e0" && o.core0_stage_pipeline != "dual" &&
        o.core0_stage_pipeline != "adaptive")
        throw std::runtime_error("--core0-stage-pipeline must be off, w0e1, w1e0, dual, or adaptive");
    if (o.dual_queue_policy != "fifo" && o.dual_queue_policy != "preferred" && o.dual_queue_policy != "core2")
        throw std::runtime_error("--dual-queue-policy must be fifo, preferred, or core2");
    if (o.dual_global_slots < 3 || o.dual_global_slots > 6)
        throw std::runtime_error("--dual-global-slots must be 3..6");
    if (o.dual_core0_reserve_slots < 0 || o.dual_core0_reserve_slots > 2)
        throw std::runtime_error("--dual-core0-reserve-slots must be 0..2");
    if (o.dual_core0_nice < 0 || o.dual_core0_nice > 19)
        throw std::runtime_error("--dual-core0-nice must be 0..19");
    if (o.dual_core0_start_gap_us < 0 || o.dual_core0_start_gap_us > 30000)
        throw std::runtime_error("--dual-core0-start-gap-us must be 0..30000");
    if (o.dual_slot_alignment != 64 && o.dual_slot_alignment != 256 && o.dual_slot_alignment != 4096)
        throw std::runtime_error("--dual-slot-alignment must be 64, 256, or 4096");
    if (o.core0_stage_slots < 1 || o.core0_stage_slots > 4)
        throw std::runtime_error("--core0-stage-slots must be 1..4");
    if (o.core0_stage_normal_workers < 0 || o.core0_stage_normal_workers > 2)
        throw std::runtime_error("--core0-stage-normal-workers must be 0..2");
    if (o.core0_stage_sched != "normal" && o.core0_stage_sched != "batch" && o.core0_stage_sched != "idle")
        throw std::runtime_error("--core0-stage-sched must be normal, batch, or idle");
    if (o.core0_stage_stagger_ms < 0 || o.core0_stage_stagger_ms > 100)
        throw std::runtime_error("--core0-stage-stagger-ms must be 0..100");
    if (o.core0_stage_phase_lock_us < 0 || o.core0_stage_phase_lock_us > 5000)
        throw std::runtime_error("--core0-stage-phase-lock-us must be 0..5000");
    if (o.core0_stage_phase_lock && o.core0_stage_pipeline != "dual")
        throw std::runtime_error("--core0-stage-phase-lock requires --core0-stage-pipeline dual");
    if (o.core0_stage_pipeline != "off" && o.cpu_gpr_threads != 4)
        throw std::runtime_error("--core0-stage-pipeline requires --cpu-gpr-threads 4");
    if (o.execution != "strict-workers" && o.execution != "pipeline" &&
        o.execution != "cpu-gpr" && o.execution != "capture")
        throw std::runtime_error("--execution must be strict-workers, pipeline, cpu-gpr or capture");
    if (o.execution == "capture") {
        if (o.target_fps <= 0.0) throw std::runtime_error("--target-fps must be positive");
        if (o.capture_seconds <= 0.0) throw std::runtime_error("--capture-seconds must be positive");
        if (o.ring_frames < 1) throw std::runtime_error("--ring-frames must be at least 1");
    }
    // v1.7.50 permits the separate GPU hybrid packers in the staged pipeline.
    // Queue submissions remain externally synchronised by the pipeline submit mutex.
    if (o.execution == "pipeline") {
        if (o.threads < 1 || o.threads > 4 || o.buffers < 2 || o.buffers > 5 ||
            o.warmup < 0 || o.frames < 1)
            throw std::runtime_error("entropy workers must be 1..4, processing slots 2..5, warmup >= 0, and frames >= 1");
        if (!o.buffers_explicit && o.threads == 4) o.buffers = 4;
        if (o.threads > o.buffers)
            throw std::runtime_error("entropy workers cannot exceed processing slots; select --buffers >= --threads");
    } else {
        if (o.threads < 1 || o.threads > 128 || o.buffers < 1 || o.buffers > 128 ||
            o.warmup < 0 || o.frames < 1)
            throw std::runtime_error("strict-worker threads and buffers must be 1..128, warmup >= 0, and frames >= 1");
        if (o.buffers < o.threads)
            throw std::runtime_error("strict-workers requires --buffers >= --threads");
    }
    if (o.gpu_inflight < 0) throw std::runtime_error("--gpu-inflight must be positive or auto/max");
    if (o.cpu_wavelet_vec_blocks != 1 && o.cpu_wavelet_vec_blocks != 2)
        throw std::runtime_error("--cpu-wavelet-vec must be 1 or 2");
    if (o.cpu_wavelet_workers < 0)
        throw std::runtime_error("--cpu-wavelet-workers must not be negative");
    if (o.cpu_wavelet_workers > 0) {
        if (o.execution != "strict-workers")
            throw std::runtime_error("--cpu-wavelet-workers requires --execution strict-workers");
        if (o.cpu_wavelet_workers >= o.threads)
            throw std::runtime_error("--cpu-wavelet-workers must leave at least one GPU worker; "
                                     "use --gpu-levels 0 for a CPU-only run");
        // The CPU engine has to be able to produce a complete coefficient frame
        // by itself. The GPU-side packers (tile-direct, one-pass, fused-mirror,
        // v15) consume GPU output that a CPU worker never writes, so they cannot
        // service a CPU-engine frame. cpu-hybrid-tile produces the identical
        // hybrid frame format from a host int16 coefficient frame and is the
        // mixed-mode equivalent of tile-direct.
        if (o.coeff_storage != Options::CoeffStorage::Int16 &&
            o.coeff_storage != Options::CoeffStorage::GpuFullPersistent &&
            o.coeff_storage != Options::CoeffStorage::CpuHybridBand &&
            o.coeff_storage != Options::CoeffStorage::CpuHybridTile)
            throw std::runtime_error(
                "--cpu-wavelet-workers needs a host-producible coefficient storage: "
                "int16, gpu-full-persistent, cpu-hybrid-band or cpu-hybrid-tile. "
                "cpu-hybrid-tile is the mixed-mode equivalent of tile-direct.");
        if (o.gpu_levels != 3)
            throw std::runtime_error("--cpu-wavelet-workers expects --gpu-levels 3 for the GPU "
                                     "workers; the CPU workers are already whole-wavelet");
    }
    if (o.gpu_levels != 0 && o.gpu_levels != 1 && o.gpu_levels != 3)
        throw std::runtime_error("--gpu-levels must be 0, 1, or 3");
    if (o.gpu_levels != 3 && o.coeff_storage != Options::CoeffStorage::GpuFullPersistent &&
        o.coeff_storage != Options::CoeffStorage::CpuHybridBand &&
        o.coeff_storage != Options::CoeffStorage::CpuHybridTile)
        throw std::runtime_error("--gpu-levels 0/1 currently requires gpu-full-persistent, cpu-hybrid-band, or cpu-hybrid-tile");
    if (o.device_local_input && o.gpu_levels == 0)
        throw std::runtime_error("--device-local-input requires at least one GPU level");
    if (o.device_local_coeff && o.readback_mode != Options::ReadbackMode::VulkanCopy)
        throw std::runtime_error("--device-local-coeff requires --vulkan-copy-readback");
    if ((o.device_local_input || o.device_local_coeff) && !o.fused2d_dispatch)
        throw std::runtime_error("device-local V51 controls require --fused2d-dispatch");
    if (o.device_local_input && o.coeff_storage != Options::CoeffStorage::GpuFullPersistent &&
        o.coeff_storage != Options::CoeffStorage::CpuHybridBand &&
        o.coeff_storage != Options::CoeffStorage::CpuHybridTile &&
        o.coeff_storage != Options::CoeffStorage::GpuHybridOnePass)
        throw std::runtime_error("--device-local-input currently requires a V51 persistent coefficient mode");
    if (o.device_local_coeff && o.coeff_storage != Options::CoeffStorage::GpuFullPersistent &&
        o.coeff_storage != Options::CoeffStorage::CpuHybridBand &&
        o.coeff_storage != Options::CoeffStorage::CpuHybridTile &&
        o.coeff_storage != Options::CoeffStorage::GpuHybridOnePass)
        throw std::runtime_error("--device-local-coeff requires a persistent full-frame coefficient mode");
    if (o.snapshot_jobs < 0 || o.snapshot_jobs == 1 || o.snapshot_jobs > 8)
        throw std::runtime_error("--snapshot-jobs must be 0 or 2..8");
    if (o.progress_every < 1) throw std::runtime_error("--progress-every must be at least 1");
    if (o.capture_queue < -1 || o.capture_queue > 1024) throw std::runtime_error("--capture-queue must be auto or 0..1024");
    if (!std::isfinite(o.capture_fps) || o.capture_fps < 0.0 || o.capture_fps > 240.0) throw std::runtime_error("--capture-fps must be 0..240");
    if (o.workgroup != 32 && o.workgroup != 64 && o.workgroup != 128 && o.workgroup != 256) throw std::runtime_error("--workgroup must be 32, 64, 128, or 256");
    if (o.barrier_scope != "global" && o.barrier_scope != "buffer") throw std::runtime_error("--barrier-scope must be global or buffer");
    if (o.command_usage != "simultaneous" && o.command_usage != "reusable") throw std::runtime_error("--command-usage must be simultaneous or reusable");
    if (o.dispatch_order != "serial" && o.dispatch_order != "interleaved") throw std::runtime_error("--dispatch-order must be serial or interleaved");
    if (!std::isfinite(o.duration) || o.duration < 0.0) throw std::runtime_error("--duration must be finite and non-negative");
    if (o.bayer != "rggb" && o.bayer != "gbrg")
        throw std::runtime_error("--bayer must be rggb or gbrg; BGGR/GRBG are not implemented by the production component/GPR path");
    if (o.coeff_storage != Options::CoeffStorage::Int16 && !o.fused2d_dispatch)
        throw std::runtime_error("selected persistent/hybrid storage requires --fused2d-dispatch");
    if (o.frame_log.empty()) o.frame_log = o.log + ".frames.csv";
    return o;
}

static std::string resolve_file(const char* argv0, const std::string& requested) {
    namespace fs = std::filesystem;
    std::error_code ec;
    std::vector<fs::path> candidates;
    candidates.emplace_back(requested);
    fs::path exe = fs::absolute(argv0, ec);
    if (!ec) {
        exe = exe.parent_path();
        candidates.push_back(exe / requested);
        candidates.push_back(exe / "shaders" / fs::path(requested).filename());
        candidates.push_back(exe.parent_path() / requested);
    }
    for (const auto& p : candidates) {
        if (fs::is_regular_file(p, ec)) return p.string();
        ec.clear();
    }
    std::ostringstream ss;
    ss << "Cannot locate file '" << requested << "'. Tried:";
    for (const auto& p : candidates) ss << ' ' << p.string();
    throw std::runtime_error(ss.str());
}

static std::vector<uint16_t> read_raw16(const Options& o) {
    if (o.input.empty()) throw std::runtime_error("--input is required");
    std::ifstream f(o.input, std::ios::binary);
    if (!f) throw std::runtime_error("Cannot open input: " + o.input);
    std::vector<uint16_t> pixels(size_t(o.width) * size_t(o.height));
    const auto expected_bytes = std::streamsize(pixels.size() * sizeof(uint16_t));
    f.read(reinterpret_cast<char*>(pixels.data()), expected_bytes);
    if (f.gcount() != expected_bytes) throw std::runtime_error("Input is shorter than width*height*2 bytes");
    char extra = 0;
    if (f.read(&extra, 1)) throw std::runtime_error("Input is larger than width*height*2 bytes; expected exactly one tightly packed RAW16 frame");
    const uint32_t source_limit = (uint32_t(1) << unsigned(o.effective_bits)) - 1u;
    /* v3.14: with --src-byteswap the stored words are big-endian, so their
       little-endian reading is meaningless for a range check; the encoder
       validates the post-swap value against the LUT by construction
       (byteswap of a 16-bit word cannot exceed 65535, and a nonzero
       src_shift already enforces src_shift + effective_bits >= 16). */
    const auto max_it = o.src_byteswap ? pixels.end()
                                       : std::max_element(pixels.begin(), pixels.end());
    if (max_it != pixels.end() && uint32_t(*max_it) > source_limit) {
        throw std::runtime_error("Input sample exceeds the declared --effective-bits range (max=" +
                                 std::to_string(*max_it) + ", limit=" + std::to_string(source_limit) + ")");
    }
    return pixels;
}

static std::vector<uint16_t> make_lut(const Options& o) {
    const size_t n = size_t(1u) << unsigned(o.effective_bits);
    std::vector<uint16_t> lut(n);
    if (o.input_companded) {
        /* Identity: the values are already GP-Log2. Clamped to working_max
           so a malformed input cannot push coefficients out of range, and
           still stepped when a reduced in-frame precision is requested, so
           --compand-inframe-bits keeps its meaning on this path too. */
        const int S = (o.compand_inframe_bits < 12)
                    ? (1 << (12 - o.compand_inframe_bits)) : 1;
        const int top = (o.working_max / S) * S;
        for (size_t i = 0; i < n; ++i) {
            int v = int(i) > o.working_max ? o.working_max : int(i);
            if (S > 1) { v = (v + S / 2) / S * S; if (v > top) v = top; }
            lut[i] = uint16_t(v);
        }
        return lut;
    }
    const double denom = std::log1p(o.log_strength);

    if (o.gradation_compand) {
        /* IMX585 ClearHDR 12-bit delivery. The sensor has already companded
         * the frame with its own piecewise gradation curve, so GP-Log2 must
         * not be applied on top -- that is the double-compand. But storing the
         * gradation codes raw is not viable either: a .gpr has nowhere to
         * declare that curve (GoPro containers are GP-Log2 and carry no
         * LinearizationTable), so any reader treats the codes as linear, the
         * 1/64 top segment collapses the highlights, and the channels skew
         * non-linearly against each other -- a colour error white balance
         * cannot undo.
         *
         * So decode the sensor curve and re-encode with the one the format
         * expects: gradation code -> scene linear -> GP-Log2. One table, one
         * companding. The stored samples are then exactly what a GP-Log2
         * container has always held, and the file renders natively.
         *
         * Pedestal: 200 in the gradation CODE domain, 3200 in the LINEAR
         * domain. It is removed here rather than carried, so the container's
         * BlackLevel 0 stays truthful and the decode side needs no pedestal
         * knowledge. Full scale is G(4095-200) = 52780. */
        const double ped_code = 200.0;
        auto grad_to_linear = [](double sig) {
            return (sig <= 500.0)  ? sig
                 : (sig <= 3250.0) ? 500.0   + (sig - 500.0)   * 4.0
                                   : 11500.0 + (sig - 3250.0) * 64.0;
        };
        /* Normalise by 65535, NOT by the 52780 the gradation curve actually
         * saturates at. It looks wasteful -- the top 19% of the code range
         * goes unused -- but that gap IS the highlight headroom. A .gpr is
         * read with no LinearizationTable, so whatever fraction of full scale
         * the brightest pixel lands on is what a reader sees: 52780/65535 puts
         * it at 80.5%, the same place the DNG puts it, leaving 0.31 stops for
         * highlight recovery. Normalising by saturation would put it at 100%
         * -- hard against white, nothing to recover, which is exactly the
         * "clipping harder, less DR in the highlights" symptom. */
        const double sat = 65535.0;
        for (size_t i = 0; i < n; ++i) {
            const double c   = std::clamp(double(i), 0.0, 4095.0);
            const double sig = (c > ped_code) ? c - ped_code : 0.0;
            const double x   = std::clamp(grad_to_linear(sig) / sat, 0.0, 1.0);
            /* log_strength selects the trade between how the .gpr looks when
             * read as linear and shadow precision in 12 bits. Measured on a
             * real capture: k=599 lifts the frame 8.8x (highlights pile at
             * white), k=1 lifts 1.4x, k->0 is exact. Shadow quantisation at
             * linear 500 runs 0.0026 stops at k=599 to 0.037 at k=0 -- all far
             * below sensor noise, which is why the near-linear end is usable. */
            const double y = (o.log_strength > 1e-9)
                           ? std::log1p(o.log_strength * x) / denom
                           : x;
            lut[i] = uint16_t(std::llround(y * double(o.working_max)));
        }
        return lut;
    }

    const double range = double(o.white - o.black);
    for (size_t i = 0; i < n; ++i) {
        const double x = std::clamp((double(i) - double(o.black)) / range, 0.0, 1.0);
        const double y = std::log1p(o.log_strength * x) / denom;
        lut[i] = uint16_t(std::llround(y * double(o.working_max)));
    }
    if (o.compand_inframe_bits < 12) {
        // Step the curve to 2^(12-bits) code steps INSIDE the same frame.
        const int S = 1 << (12 - o.compand_inframe_bits);
        const int top = (o.working_max / S) * S;   // largest in-frame multiple
        for (size_t i = 0; i < n; ++i) {
            const int v = (int(lut[i]) + S / 2) / S * S;
            lut[i] = uint16_t(v > top ? top : v);
        }
        lut.front() = 0;
        if (size_t(o.white) < lut.size()) lut[size_t(o.white)] = uint16_t(top);
        return lut;
    }
    lut.front() = 0;
    if (size_t(o.white) < lut.size()) lut[size_t(o.white)] = uint16_t(o.working_max);
    return lut;
}

struct PlaneSpec { int width = 0; int height = 0; int levels = 0; };
// v1.14: exact cache-compressed GP-Log lookup. The 16-bit source requires a
// 65,536-entry logical mapping, but the monotonic curve changes by <=15 codes
// within almost every 16-input bucket. Keep one uint16 base per bucket (8 KiB)
// plus two 4-bit deltas per byte (32 KiB). Any steep low-end prefix buckets
// fall back to their tiny region of the original LUT. This changes no code
// values: configure_cpu_hot_lut verifies every source code before enabling it.
static bool g_cpu_hot_lut = false;
static std::vector<uint16_t> g_hot_lut_base;
static std::vector<uint8_t> g_hot_lut_delta;
/* v3.15: the buckets whose 16 outputs span more than a nibble form one
   contiguous RANGE, not necessarily a prefix. With black = 0 the steep part
   of the GP-Log curve starts at code 0 (the original v1.14 prefix); with a
   real sensor pedestal (e.g. black = 3200) the table is flat below the
   pedestal and the steep region sits just above it. [bad_lo, bad_lo+bad_span)
   falls back to the full LUT -- a few hundred entries, which stay resident. */
static uint32_t g_hot_lut_bad_lo = 0;
static uint32_t g_hot_lut_bad_span = 0;
static size_t g_hot_lut_size = 0;

static inline uint16_t cpu_lut_lookup(const uint16_t* full_lut, uint16_t raw) {
    if (!g_cpu_hot_lut) return full_lut[raw];
    const uint32_t b = uint32_t(raw) >> 4u;
    if (b - g_hot_lut_bad_lo < g_hot_lut_bad_span) return full_lut[raw];
    const uint8_t packed = g_hot_lut_delta[size_t(raw) >> 1u];
    const uint8_t delta = (raw & 1u) ? uint8_t(packed >> 4u) : uint8_t(packed & 0x0Fu);
    return uint16_t(g_hot_lut_base[b] + delta);
}

static void configure_cpu_hot_lut(const Options& o, const std::vector<uint16_t>& lut) {
    g_cpu_hot_lut = false;
    g_hot_lut_base.clear(); g_hot_lut_delta.clear();
    g_hot_lut_bad_lo = 0; g_hot_lut_bad_span = 0; g_hot_lut_size = 0;
    if (!o.cpu_hot_lut) return;
    if (lut.empty() || (lut.size() & 15u) != 0u || lut.size() > 65536u)
        throw std::runtime_error("--cpu-hot-lut requires a power-of-two 16-code-bucket LUT up to 65536 entries");
    const size_t buckets = lut.size() / 16u;
    g_hot_lut_base.resize(buckets);
    g_hot_lut_delta.assign(lut.size() / 2u, uint8_t(0));
    std::vector<uint8_t> bad(buckets, 0u);
    for (size_t b = 0; b < buckets; ++b) {
        const uint16_t base = lut[b * 16u];
        g_hot_lut_base[b] = base;
        for (size_t k = 0; k < 16u; ++k) {
            const int d = int(lut[b * 16u + k]) - int(base);
            if (d < 0 || d > 15) bad[b] = 1u;
        }
    }
    /* The bad buckets must form ONE contiguous range (see the globals above).
       For the GP-Log family they always do: flat below black, monotonically
       flattening above it. Anything else is rejected, never approximated. */
    size_t lo = 0;
    while (lo < buckets && !bad[lo]) ++lo;
    size_t hi = lo;
    while (hi < buckets && bad[hi]) ++hi;
    for (size_t b = hi; b < buckets; ++b)
        if (bad[b]) throw std::runtime_error("--cpu-hot-lut exact nibble representation has a non-contiguous fallback bucket range");
    g_hot_lut_bad_lo = uint32_t(lo);
    g_hot_lut_bad_span = uint32_t(hi - lo);
    for (size_t raw = 0; raw < lut.size(); raw += 2u) {
        const size_t b0 = raw >> 4u;
        if (b0 - lo < hi - lo) continue;   /* fallback range: full LUT serves it */
        const int d0 = int(lut[raw]) - int(g_hot_lut_base[b0]);
        const size_t r1 = raw + 1u;
        const size_t b1 = r1 >> 4u;
        const int d1 = int(lut[r1]) - int(g_hot_lut_base[b1]);
        if (d0 < 0 || d0 > 15 || d1 < 0 || d1 > 15)
            throw std::runtime_error("--cpu-hot-lut delta packing overflow");
        g_hot_lut_delta[raw >> 1u] = uint8_t(d0 | (d1 << 4));
    }
    g_hot_lut_size = lut.size();
    // Enable only for the exhaustive verification, then leave it on.
    g_cpu_hot_lut = true;
    for (size_t i = 0; i < lut.size(); ++i) {
        const uint16_t got = cpu_lut_lookup(lut.data(), uint16_t(i));
        if (got != lut[i]) { g_cpu_hot_lut = false; throw std::runtime_error("--cpu-hot-lut exhaustive verification failed"); }
    }
    const size_t hot_bytes = g_hot_lut_base.size() * sizeof(uint16_t) + g_hot_lut_delta.size();
    const size_t fallback_bytes = size_t(g_hot_lut_bad_span) * 16u * sizeof(uint16_t);
    std::cout << "CPU_HOT_LUT_VERIFY bitexact=YES logical_bytes=" << (lut.size()*sizeof(uint16_t))
              << " packed_bytes=" << hot_bytes
              << " fallback_hot_bytes=" << fallback_bytes
              << " bad_bucket_range=[" << g_hot_lut_bad_lo << ","
              << (g_hot_lut_bad_lo + g_hot_lut_bad_span) << ")\n";
}

static std::array<PlaneSpec,4> plane_specs(const Options& o) {
    const int pw = o.width / 2, ph = o.height / 2;
    return {{{pw,ph,3},{pw,ph,3},{pw,ph,3},{pw,ph,3}}};
}

static int dispatches_per_frame(const Options& o) {
    const auto specs = plane_specs(o);
    const int levels = o.fused2d_dispatch ? o.gpu_levels : specs[0].levels;
    if (o.wide_separable) {
        // Two dispatches per level (v101 horizontal, v102 vertical), both already
        // batching the four planes in Z, plus one v103 packer dispatch per
        // (plane, level, band) = 4*levels*3.
        return o.gpu_levels * 2 + (o.wide_hybrid_pack ? o.gpu_levels * 12 : 0);
    }
    if (o.planes_in_z && o.fused2d_dispatch) return levels;
    int dispatches = 0;
    for (const auto& p : specs) dispatches += (o.fused2d_dispatch ? levels : p.levels * 2);
    return dispatches;
}

// Number of GPU timestamp intervals record() will emit. For every existing path
// that is one per dispatch; the wide-separable path times one interval per level
// (both of its dispatches together) plus one for the v103 packer, because a
// timestamp between v101 and v102 would serialise exactly the overlap being
// measured. Used only to size the query pool.
static int trace_intervals_per_frame(const Options& o) {
    if (o.wide_separable) return o.gpu_levels + (o.wide_hybrid_pack ? 1 : 0);
    return dispatches_per_frame(o);
}

static uint32_t codec_profile_id(const Options&) {
    return 444u;
}

static void split_compand(const Options& o, const uint16_t* src_data,
                          const std::vector<uint16_t>& lut, int16_t* dst,
                          size_t plane_stride_elems) {
    const int pw = o.width / 2, ph = o.height / 2;
    int16_t* gs_plane = dst + 0 * plane_stride_elems;
    int16_t* rg_plane = dst + 1 * plane_stride_elems;
    int16_t* bg_plane = dst + 2 * plane_stride_elems;
    int16_t* gd_plane = dst + 3 * plane_stride_elems;
    // MUST stay 4096 regardless of compand_bits. vc5_decoder/raw.c inverts
    // with a hardcoded midpoint of 2048 against a halved difference, which is
    // exactly an encoder-side bias of 4096. Scaling this with the compand
    // width made 11- and 10-bit files decode with a large negative offset that
    // then clamped to black. Only working_max (the LUT output range) changes
    // with compand_bits; the colour-difference bias is fixed by the format.
    const int midpoint_x2 = 4096;

    // The component equations are range-safe for 12-bit inputs:
    // GS is 0..4095 and each signed colour difference, biased by 4096 then
    // divided by two, is 0..4095. No per-pixel clamp is required.
    // true_12bit stores the differences UNHALVED (0..8191): sh below.
    const int sh = o.true_12bit ? 0 : 1;
    if (o.bayer == "gbrg") {
        for (int y = 0; y < ph; ++y) {
            const uint16_t* row0 = src_data + size_t(2*y) * size_t(o.width);
            const uint16_t* row1 = row0 + size_t(o.width);
            int16_t* gs = gs_plane + size_t(y) * size_t(pw);
            int16_t* rg = rg_plane + size_t(y) * size_t(pw);
            int16_t* bg = bg_plane + size_t(y) * size_t(pw);
            int16_t* gd = gd_plane + size_t(y) * size_t(pw);
            for (int x = 0; x < pw; ++x) {
                const int g1 = lut[*row0++];
                const int b  = lut[*row0++];
                const int r  = lut[*row1++];
                const int g2 = lut[*row1++];
                const int green_sum = g1 + g2;
                const int green = green_sum >> 1;
                *gs++ = int16_t(green);
                *rg++ = int16_t((r - green + midpoint_x2) >> sh);
                *bg++ = int16_t((b - green + midpoint_x2) >> sh);
                *gd++ = int16_t((g1 - g2 + midpoint_x2) >> sh);
            }
        }
    } else { // RGGB
        for (int y = 0; y < ph; ++y) {
            const uint16_t* row0 = src_data + size_t(2*y) * size_t(o.width);
            const uint16_t* row1 = row0 + size_t(o.width);
            int16_t* gs = gs_plane + size_t(y) * size_t(pw);
            int16_t* rg = rg_plane + size_t(y) * size_t(pw);
            int16_t* bg = bg_plane + size_t(y) * size_t(pw);
            int16_t* gd = gd_plane + size_t(y) * size_t(pw);
            for (int x = 0; x < pw; ++x) {
                const int r  = lut[*row0++];
                const int g1 = lut[*row0++];
                const int g2 = lut[*row1++];
                const int b  = lut[*row1++];
                const int green_sum = g1 + g2;
                const int green = green_sum >> 1;
                *gs++ = int16_t(green);
                *rg++ = int16_t((r - green + midpoint_x2) >> sh);
                *bg++ = int16_t((b - green + midpoint_x2) >> sh);
                *gd++ = int16_t((g1 - g2 + midpoint_x2) >> sh);
            }
        }
    }
}

// Startup proof that the true-12bit split is losslessly invertible. (g1+g2)
// and (g1-g2) share parity, so the LSB the halved GS discards is GD & 1.
static void self_test_true_12bit_roundtrip(int bits) {
    const uint32_t vmax = (1u << unsigned(bits)) - 1u;
    const int mid = 4096;   // fixed by the container format, not by `bits`
    uint32_t rs = 77u;
    auto rng = [&]{ rs = rs * 1664525u + 1013904223u; return (rs >> 8) & vmax; };
    for (int k = 0; k < 100000; ++k) {
        const int r = int(rng()), g1 = int(rng()), g2 = int(rng()), b = int(rng());
        const int gs = (g1 + g2) >> 1;
        const int rg = r - gs + mid;
        const int bg = b - gs + mid;
        const int gd = g1 - g2 + mid;
        if (rg < 1 || rg > 2 * mid - 1 || bg < 1 || bg > 2 * mid - 1 || gd < 1 || gd > 2 * mid - 1)
            throw std::runtime_error("true-12bit component out of the 13-bit range");
        const int sum = 2 * gs + ((gd - mid) & 1);
        const int rg1 = (sum + (gd - mid)) / 2;
        const int rg2 = (sum - (gd - mid)) / 2;
        const int rr = rg - mid + gs;
        const int rb = bg - mid + gs;
        if (rr != r || rb != b || rg1 != g1 || rg2 != g2)
            throw std::runtime_error("true-12bit split failed to round-trip");
    }
    std::cout << "TRUE_12BIT_ROUNDTRIP PASS cases=100000 bits=" << bits << " midpoint=4096"
              << " max_component=" << (2 * mid - 1)
              << " peak_L1_LL=" << (4 * (2 * mid - 1))
              << " int16_pre_p2_headroom=" << (32767 - 4 * (2 * mid - 1)) << "\n";
}
struct CpuWaveletPush {
    uint32_t axis_len = 0;
    uint32_t count = 0;
    uint32_t stride = 0;
    uint32_t out_stride = 0;
    uint32_t dir = 0;
    int32_t prescale = 0;
    int32_t q_lh = 1, q_hl = 1, q_hh = 1;
};


static int cpu_quantize_exact(int value, int divisor) {
    if (divisor <= 1) return value;
    int midpoint = divisor >> 1;
    if (midpoint) --midpoint;
    const int multiplier = (1 << 16) / divisor;
    int magnitude = (value < 0 ? -value : value) + midpoint;
    int q = (magnitude * multiplier) >> 16;
    return value < 0 ? -q : q;
}

// GPU push constants carry a precomputed exact quantiser rather than the raw
// divisor. Low 16 bits contain floor(65536/divisor), high 16 bits contain the
// codec rounding midpoint. A zero low half is the identity divisor (<= 1).
// The 80-byte Push ABI does not change, but V3D no longer performs millions of
// integer divisions per frame.
static int32_t pack_gpu_quantizer(int divisor) {
    if (divisor <= 1) return 0;
    const uint32_t midpoint = uint32_t((divisor >> 1) - 1);
    const uint32_t multiplier = uint32_t((1u << 16) / uint32_t(divisor));
    if (multiplier == 0u || multiplier > 0xffffu || midpoint > 0xffffu)
        throw std::runtime_error("GPU quantizer divisor is outside packed range: " + std::to_string(divisor));
    return int32_t((midpoint << 16u) | multiplier);
}

static int cpu_quantize_packed(int value, int32_t packed_q) {
    const uint32_t bits = uint32_t(packed_q);
    const uint32_t multiplier = bits & 0xffffu;
    if (multiplier == 0u) return value;
    const int midpoint = int(bits >> 16u);
    const int magnitude = (value < 0 ? -value : value) + midpoint;
    const int q = (magnitude * int(multiplier)) >> 16;
    return value < 0 ? -q : q;
}

static int cpu_pre(int v, int p) {
    return p != 0 ? ((v + ((1 << p) - 1)) >> p) : v;
}

// One output position of one line, exactly as the original scalar loop computed
// it. Both the scalar pass and the NEON remainder path call this, so there is a
// single implementation of the arithmetic and the two can never drift.
static inline void cinepi_scalar_one_output(const int16_t* src, int16_t* dst,
                                           const CpuWaveletPush& pc,
                                           uint32_t line_i, uint32_t i,
                                           uint32_t last, uint32_t outs) {
    const uint32_t c = 2u * i;
    const int pr = (1 << pc.prescale) - 1;
    auto clamp16 = [](int v) { return std::clamp(v, -32768, 32767); };
    auto ld = [&](uint32_t j) {
        return pc.dir == 0u ? int(src[size_t(line_i) * pc.stride + j])
                            : int(src[size_t(j) * pc.stride + line_i]);
    };
    auto st = [&](uint32_t j, int v) {
        if (pc.dir == 0u) dst[size_t(line_i) * pc.out_stride + j] = int16_t(clamp16(v));
        else dst[size_t(j) * pc.out_stride + line_i] = int16_t(clamp16(v));
    };
    const int lo = (ld(c) + ld(c + 1u) + pr) >> pc.prescale;
    int hi = 0;
    if (i == 0u) {
        hi = (5 * cpu_pre(ld(0u), pc.prescale) - 11 * cpu_pre(ld(1u), pc.prescale)
            + 4 * cpu_pre(ld(2u), pc.prescale) + 4 * cpu_pre(ld(3u), pc.prescale)
            - cpu_pre(ld(4u), pc.prescale) - cpu_pre(ld(5u), pc.prescale) + 4) >> 3;
    } else if (c == last) {
        hi = (cpu_pre(ld(c - 4u), pc.prescale) + cpu_pre(ld(c - 3u), pc.prescale)
            - 4 * cpu_pre(ld(c - 2u), pc.prescale) - 4 * cpu_pre(ld(c - 1u), pc.prescale)
            + 11 * cpu_pre(ld(c), pc.prescale) - 5 * cpu_pre(ld(c + 1u), pc.prescale) + 4) >> 3;
    } else {
        hi = (-cpu_pre(ld(c - 2u), pc.prescale) - cpu_pre(ld(c - 1u), pc.prescale)
            + (8 * cpu_pre(ld(c), pc.prescale)) - (8 * cpu_pre(ld(c + 1u), pc.prescale))
            + cpu_pre(ld(c + 2u), pc.prescale) + cpu_pre(ld(c + 3u), pc.prescale) + 4) >> 3;
    }
    int qlo = lo, qhi = hi;
    if (pc.dir == 1u) {
        const bool horizontal_high = line_i >= (pc.count / 2u);
        if (horizontal_high) qlo = cpu_quantize_exact(qlo, pc.q_lh);
        qhi = cpu_quantize_exact(qhi, horizontal_high ? pc.q_hh : pc.q_hl);
    }
    st(i, qlo);
    st(outs + i, qhi);
}

// ---------------------------------------------------------------------------
// v1.7.52: optimised single-thread NEON CPU wavelet.
//
// WHY. The scalar pass below is the project's bit-exact reference, and it is also
// what --gpu-levels 0/1 actually executes. It has two problems as a production
// kernel: it is scalar, and for dir==1 it reads src[j*stride + line_i], i.e. it
// walks DOWN A COLUMN with the row stride between successive loads. That is
// cache-hostile on an A76 and defeats any prefetcher.
//
// The NEON version fixes both by swapping the loop nest for the vertical pass so
// the inner loop runs along CONTIGUOUS columns, 8 int16 at a time, and by using
// vld2q_s16 on the horizontal pass to deinterleave the even/odd sample pairs the
// 2/6 filter needs.
//
// EXACTNESS. Every intermediate is computed the same width as the scalar code:
// the 6-tap highpass sum in int32 (worst case |sum| <= 4095*20 = 81900 overflows
// int16), the arithmetic >>3 as vshrq_n_s32 (floor, matching C's >> on a negative
// int), and the int16 store clamp as vqmovn_s32 (saturating narrow, matching
// std::clamp). Only prescale 0 and 2 are vectorised because those are the only
// values the schedule uses (vc5_bench.cpp prescale = {0,2,2}); anything else
// falls through to scalar. Edge outputs (i == 0 and 2i == last, which use the
// two special coefficient sets) and any tail that is not a whole vector are also
// left to the scalar path -- 2 of ~960 outputs per line, so >98% is vectorised
// while the fiddly cases stay in already-proven code.
//
// self_test() proves NEON == scalar bit-for-bit before anything else runs.
// ---------------------------------------------------------------------------
#if defined(__ARM_NEON) || defined(__ARM_NEON__) || defined(__aarch64__)
#define CINEPI_HAVE_NEON_WAVELET 1
#include <arm_neon.h>
#else
#define CINEPI_HAVE_NEON_WAVELET 0
#endif

// Set once during option parsing, before any worker thread starts.
static bool g_cpu_wavelet_use_neon = true;

// v1.7.53 kernel selection. Declared outside the NEON guard so option parsing
// compiles identically on a target without NEON.
//   true  = v1.7.53 row-major vertical pass (default)
//   false = v1.7.52 column-block sweep, retained as the A/B control
static bool g_cpu_wavelet_kernel_v53 = true;
// Columns per inner iteration of the v1.7.53 vertical pass, in int16x8 vectors.
// 1 = 8 columns, 2 = 16 columns. Two independent chains is the default.
static int g_cpu_wavelet_vec_blocks = 1;

static void cpu_wavelet_pass_scalar(const int16_t* src, int16_t* dst, const CpuWaveletPush& pc);

#if CINEPI_HAVE_NEON_WAVELET

// An int16x8 lane set held unsaturated in two int32x4 halves. The scalar code
// keeps every intermediate in `int` and only clamps at the store, INCLUDING
// through the quantiser -- quantising an already-clamped value gives a different
// answer whenever |highpass| exceeds int16, so the width must be carried all the
// way to the store or the two paths diverge.
struct cinepi_i32x8 { int32x4_t lo, hi; };

static inline int16x8_t cinepi_sat(cinepi_i32x8 v) {
    return vcombine_s16(vqmovn_s32(v.lo), vqmovn_s32(v.hi));
}

// (-a0 - a1 + 8*a2 - 8*a3 + a4 + a5 + 4) >> 3, unsaturated. vshrq_n_s32 is an
// arithmetic shift, matching C's >> on a negative int (floor).
static inline cinepi_i32x8 cinepi_hp6(int16x8_t a0, int16x8_t a1, int16x8_t a2,
                                      int16x8_t a3, int16x8_t a4, int16x8_t a5) {
    const int32x4_t four = vdupq_n_s32(4);
    cinepi_i32x8 r;
    r.lo = vaddq_s32(vmovl_s16(vget_low_s16(a4)), vmovl_s16(vget_low_s16(a5)));
    r.lo = vsubq_s32(r.lo, vmovl_s16(vget_low_s16(a0)));
    r.lo = vsubq_s32(r.lo, vmovl_s16(vget_low_s16(a1)));
    r.lo = vaddq_s32(r.lo, vshlq_n_s32(vsubq_s32(vmovl_s16(vget_low_s16(a2)),
                                                 vmovl_s16(vget_low_s16(a3))), 3));
    r.lo = vshrq_n_s32(vaddq_s32(r.lo, four), 3);
    r.hi = vaddq_s32(vmovl_s16(vget_high_s16(a4)), vmovl_s16(vget_high_s16(a5)));
    r.hi = vsubq_s32(r.hi, vmovl_s16(vget_high_s16(a0)));
    r.hi = vsubq_s32(r.hi, vmovl_s16(vget_high_s16(a1)));
    r.hi = vaddq_s32(r.hi, vshlq_n_s32(vsubq_s32(vmovl_s16(vget_high_s16(a2)),
                                                 vmovl_s16(vget_high_s16(a3))), 3));
    r.hi = vshrq_n_s32(vaddq_s32(r.hi, four), 3);
    return r;
}

// (a + b + ((1<<p)-1)) >> p, unsaturated. Only p == 0 and p == 2 occur.
static inline cinepi_i32x8 cinepi_lp(int16x8_t a, int16x8_t b, int prescale) {
    cinepi_i32x8 r;
    r.lo = vaddq_s32(vmovl_s16(vget_low_s16(a)), vmovl_s16(vget_low_s16(b)));
    r.hi = vaddq_s32(vmovl_s16(vget_high_s16(a)), vmovl_s16(vget_high_s16(b)));
    if (prescale == 2) {
        const int32x4_t three = vdupq_n_s32(3);
        r.lo = vshrq_n_s32(vaddq_s32(r.lo, three), 2);
        r.hi = vshrq_n_s32(vaddq_s32(r.hi, three), 2);
    }
    return r;
}

// cpu_pre(v, 2) = (v + 3) >> 2 per tap. Safe in int16: the host invariant caps
// coefficients at working_max*4 <= 16380, and 16380 + 3 fits.
static inline int16x8_t cinepi_pre_p2(int16x8_t v) {
    return vshrq_n_s16(vaddq_s16(v, vdupq_n_s16(3)), 2);
}

// ===========================================================================
// v1.4 lab: WIDENING-INTRINSIC forms of the same arithmetic (candidate
// `wl_widen`).
//
// WHY. cinepi_hp6/cinepi_lp widen every tap with vmovl_s16 and then operate
// in int32. AArch64 has widen-and-operate as single instructions -- vaddl,
// vsubl, vsubw, vmlal -- so the widen is free. Per 8 lanes the interior
// highpass goes from 28 instructions to 16 and the lowpass from 6 to 2.
//
// WHY IT IS BIT-EXACT BY CONSTRUCTION. vaddl_s16(a,b) is defined as
// (int32)a + (int32)b; vsubw_s16(acc,x) as acc - (int32)x; vmlal_n_s16
// as acc + (int32)x * (int32)n. Each is exactly the widen-then-operate pair
// it replaces, and int16 (+|-|*small) int16 cannot overflow int32, so no
// intermediate is truncated where the old form kept width. Same values, same
// order, fewer instructions. The CRC gate re-proves it end to end anyway.
// ===========================================================================

// (-a0 - a1 + 8*a2 - 8*a3 + a4 + a5 + 4) >> 3, unsaturated.

// (a + b + ((1<<p)-1)) >> p, unsaturated. Only p == 0 and p == 2 occur.

// (5*a0 - 11*a1 + 4*a2 + 4*a3 - a4 - a5 + 4) >> 3. First output row only.

// (a0 + a1 - 4*a2 - 4*a3 + 11*a4 - 5*a5 + 4) >> 3. Last output row only.

// Precomputed exact quantiser state. v1.7.52 moves the divisor-derived
// constants out of the vertical inner loop. A full 4K frame calls the quantiser
// millions of times, while each pass uses only three fixed divisors.
struct cinepi_quant_neon {
    bool enabled = false;
    int32x4_t multiplier{};
    int32x4_t midpoint{};
    /* Pixel Clean: |v| <= pc_t becomes zero. Carried here so every existing
     * cinepi_make_quant() site picks the lever up without its own branch, and
     * so the hot loop pays one compare-and-select per vector instead of a float
     * dead-zone test plus a post-quant +-1 branch per coefficient. */
    bool pc = false;
    int32x4_t pc_t{};
};

static inline cinepi_quant_neon cinepi_make_quant(int divisor, int pc_threshold = 0) {
    cinepi_quant_neon q{};
    q.enabled = divisor > 1;
    int midpoint = divisor >> 1;
    if (midpoint) --midpoint;
    q.multiplier = vdupq_n_s32(q.enabled ? ((1 << 16) / divisor) : 0);
    q.midpoint = vdupq_n_s32(q.enabled ? midpoint : 0);
    /* A threshold of 0 is the OFF state and must stay a no-op: |v| <= 0 is
     * already quantised to zero by any divisor, so leaving pc false there keeps
     * the lever bit-exact when disabled. */
    q.pc = pc_threshold > 0;
    q.pc_t = vdupq_n_s32(pc_threshold);
    return q;
}

// cpu_quantize_exact() on unsaturated int32 lanes, exactly as the scalar does on
// `int`: magnitude = |v| + midpoint, q = (magnitude * multiplier) >> 16, then the
// sign of v is reapplied.
static inline cinepi_i32x8 cinepi_quant(cinepi_i32x8 v,
                                        const cinepi_quant_neon& qp) {
    if (!qp.enabled) return v;
    const int32x4_t zero = vdupq_n_s32(0);
    cinepi_i32x8 r;
    const int32x4_t alo = vabsq_s32(v.lo), ahi = vabsq_s32(v.hi);
    int32x4_t q = vshrq_n_s32(vmulq_s32(vaddq_s32(alo, qp.midpoint), qp.multiplier), 16);
    r.lo = vbslq_s32(vcltq_s32(v.lo, zero), vnegq_s32(q), q);
    q = vshrq_n_s32(vmulq_s32(vaddq_s32(ahi, qp.midpoint), qp.multiplier), 16);
    r.hi = vbslq_s32(vcltq_s32(v.hi, zero), vnegq_s32(q), q);
    if (qp.pc) {
        /* One compare and one select per four lanes, reusing the |v| already
         * computed for the quantiser. vcleq gives |v| <= T; bsl picks zero. */
        r.lo = vbslq_s32(vcleq_s32(alo, qp.pc_t), zero, r.lo);
        r.hi = vbslq_s32(vcleq_s32(ahi, qp.pc_t), zero, r.hi);
    }
    return r;
}

/* Scalar twin of the above, for the edge/tail paths that do not vectorise. */
static inline int cpu_quantize_pc(int value, int divisor, int pc_threshold) {
    if (pc_threshold > 0 && std::abs(value) <= pc_threshold) return 0;
    return cpu_quantize_exact(value, divisor);
}

static inline cinepi_i32x8 cinepi_hp_top(int16x8_t a0, int16x8_t a1,
                                         int16x8_t a2, int16x8_t a3,
                                         int16x8_t a4, int16x8_t a5) {
    const int32x4_t four = vdupq_n_s32(4);
    cinepi_i32x8 r;
    auto half = [&](int16x4_t x0, int16x4_t x1, int16x4_t x2,
                    int16x4_t x3, int16x4_t x4, int16x4_t x5) {
        int32x4_t v = vmulq_n_s32(vmovl_s16(x0), 5);
        v = vmlaq_n_s32(v, vmovl_s16(x1), -11);
        v = vmlaq_n_s32(v, vmovl_s16(x2), 4);
        v = vmlaq_n_s32(v, vmovl_s16(x3), 4);
        v = vsubq_s32(v, vmovl_s16(x4));
        v = vsubq_s32(v, vmovl_s16(x5));
        return vshrq_n_s32(vaddq_s32(v, four), 3);
    };
    r.lo = half(vget_low_s16(a0), vget_low_s16(a1), vget_low_s16(a2),
                vget_low_s16(a3), vget_low_s16(a4), vget_low_s16(a5));
    r.hi = half(vget_high_s16(a0), vget_high_s16(a1), vget_high_s16(a2),
                vget_high_s16(a3), vget_high_s16(a4), vget_high_s16(a5));
    return r;
}

static inline cinepi_i32x8 cinepi_hp_bottom(int16x8_t a0, int16x8_t a1,
                                            int16x8_t a2, int16x8_t a3,
                                            int16x8_t a4, int16x8_t a5) {
    const int32x4_t four = vdupq_n_s32(4);
    cinepi_i32x8 r;
    auto half = [&](int16x4_t x0, int16x4_t x1, int16x4_t x2,
                    int16x4_t x3, int16x4_t x4, int16x4_t x5) {
        int32x4_t v = vmovl_s16(x0);
        v = vaddq_s32(v, vmovl_s16(x1));
        v = vmlaq_n_s32(v, vmovl_s16(x2), -4);
        v = vmlaq_n_s32(v, vmovl_s16(x3), -4);
        v = vmlaq_n_s32(v, vmovl_s16(x4), 11);
        v = vmlaq_n_s32(v, vmovl_s16(x5), -5);
        return vshrq_n_s32(vaddq_s32(v, four), 3);
    };
    r.lo = half(vget_low_s16(a0), vget_low_s16(a1), vget_low_s16(a2),
                vget_low_s16(a3), vget_low_s16(a4), vget_low_s16(a5));
    r.hi = half(vget_high_s16(a0), vget_high_s16(a1), vget_high_s16(a2),
                vget_high_s16(a3), vget_high_s16(a4), vget_high_s16(a5));
    return r;
}

// hpass_vqadd: the same identity as ll_vqadd, applied to the HORIZONTAL pass.
//
// In the fused cascade fused_level_push() leaves q_lh/q_hl/q_hh at 1, so the
// horizontal pass never quantises, and the vectorised branch below does not
// even build a quantiser. At level 1 (prescale 0) its lowpass is therefore
//     lo  = (int32)e1 + (int32)o1        (cinepi_lp, prescale 0)
//     out = clamp(lo, -32768, 32767)     (cinepi_sat -> vqmovn)
// which is vqaddq_s16, exactly as in the vertical emit. Level 1 is the widest
// level, so this is 960 x 1080 x 4 = 4.15M lowpass outputs per frame.
//
// Levels 2 and 3 use prescale 2 -- (a+b+3)>>2 -- and are NOT covered: the
// intermediate sum there can exceed int16, so folding the clamp in early
// would change the result. The flag checks prescale rather than assuming it.

static void cpu_wavelet_pass_neon(const int16_t* src, int16_t* dst, const CpuWaveletPush& pc) {
    if (pc.prescale != 0 && pc.prescale != 2) { cpu_wavelet_pass_scalar(src, dst, pc); return; }
    const uint32_t last = pc.axis_len - 2u;
    const uint32_t outs = last / 2u + 1u;
    if (outs < 3u || pc.count == 0u) { cpu_wavelet_pass_scalar(src, dst, pc); return; }

    if (pc.dir == 0u) {
        // ---- horizontal: vectorise along the output index -------------------
        // Taps for output k of a block starting at i0 are, deinterleaved,
        // e[k], o[k], e[k+1], o[k+1], e[k+2], o[k+2] taken from src[2*i0-2 ...].
        // vextq_s16 supplies the +1 and +2 shifts.
        // Only legal where the horizontal pass does not quantise (the fused
        // cascade) AND prescale is 0 (level 1). Both are checked, not assumed.
        // At prescale 0 with no quantiser the horizontal lowpass is
        // clamp(e1 + o1), which IS vqaddq_s16. Both conditions are still
        // tested rather than assumed: levels 2 and 3 use prescale 2, where
        // the intermediate can exceed int16, and this kernel is shared with
        // the non-fused schedule which does quantise.
        const bool lofast = pc.prescale == 0 &&
                            pc.q_lh <= 1 && pc.q_hl <= 1 && pc.q_hh <= 1;
        for (uint32_t line_i = 0; line_i < pc.count; ++line_i) {
            const int16_t* row = src + size_t(line_i) * pc.stride;
            int16_t* orow = dst + size_t(line_i) * pc.out_stride;
            uint32_t i = 1u;                      // output 0 uses the top set
            for (; i + 8u <= outs - 1u; i += 8u) {
                const uint32_t base = 2u * i - 2u;
                if (base + 24u > pc.axis_len) break;   // keep the over-read in bounds
                const int16x8x2_t A = vld2q_s16(row + base);
                const int16x4x2_t B = vld2_s16(row + base + 16);
                const int16x8_t eB = vcombine_s16(B.val[0], B.val[0]);
                const int16x8_t oB = vcombine_s16(B.val[1], B.val[1]);
                int16x8_t e0 = A.val[0], o0 = A.val[1];
                int16x8_t e1 = vextq_s16(A.val[0], eB, 1), o1 = vextq_s16(A.val[1], oB, 1);
                int16x8_t e2 = vextq_s16(A.val[0], eB, 2), o2 = vextq_s16(A.val[1], oB, 2);
                cinepi_i32x8 hi = (pc.prescale == 0)
                    ? cinepi_hp6(e0, o0, e1, o1, e2, o2)
                    : cinepi_hp6(cinepi_pre_p2(e0), cinepi_pre_p2(o0),
                                 cinepi_pre_p2(e1), cinepi_pre_p2(o1),
                                 cinepi_pre_p2(e2), cinepi_pre_p2(o2));
                // lofast is loop-invariant, so this unswitches.
                if (lofast) {
                    vst1q_s16(orow + i, vqaddq_s16(e1, o1));
                } else {
                    cinepi_i32x8 lo = cinepi_lp(e1, o1, pc.prescale);
                    vst1q_s16(orow + i, cinepi_sat(lo));
                }
                vst1q_s16(orow + outs + i, cinepi_sat(hi));
            }
            for (uint32_t k = 0; k < outs; ++k) {
                if (k >= 1u && k < i) continue;    // already done by the vector loop
                cinepi_scalar_one_output(src, dst, pc, line_i, k, last, outs);
            }
        }
        return;
    }

    // ---- vertical: contiguous 8-column blocks with a sliding six-row window --
    // v1.7.51 reloaded all six source rows for every output row. Adjacent 2/6
    // outputs overlap by four rows, so v1.7.52 keeps those four vectors live and
    // loads only two new rows per output after the seed. This cuts interior source
    // loads from six vectors to two vectors while retaining the cache-friendly
    // across-column layout.
    const uint32_t half = pc.count / 2u;
    const uint32_t low_vector_end = (half / 8u) * 8u;
    const uint32_t high_vector_end =
        half + ((pc.count - half) / 8u) * 8u;
    const cinepi_quant_neon q_lh = cinepi_make_quant(pc.q_lh);
    const cinepi_quant_neon q_hl = cinepi_make_quant(pc.q_hl);
    const cinepi_quant_neon q_hh = cinepi_make_quant(pc.q_hh);

    auto run_columns = [&](uint32_t from, uint32_t to, bool horizontal_high) {
        for (uint32_t col = from; col < to; col += 8u) {
            auto load_row = [&](uint32_t row) {
                return vld1q_s16(src + size_t(row) * pc.stride + col);
            };
            auto pre = [&](int16x8_t value) {
                return pc.prescale == 0 ? value : cinepi_pre_p2(value);
            };
            auto quantise_and_store = [&](uint32_t output_row,
                                          cinepi_i32x8 lo,
                                          cinepi_i32x8 hi) {
                if (horizontal_high) {
                    lo = cinepi_quant(lo, q_lh);
                    hi = cinepi_quant(hi, q_hh);
                } else {
                    hi = cinepi_quant(hi, q_hl);
                }
                vst1q_s16(dst + size_t(output_row) * pc.out_stride + col,
                          cinepi_sat(lo));
                vst1q_s16(dst + size_t(outs + output_row) * pc.out_stride + col,
                          cinepi_sat(hi));
            };

            // Top boundary output, vectorised across columns.
            int16x8_t a0 = load_row(0u);
            int16x8_t a1 = load_row(1u);
            int16x8_t a2 = load_row(2u);
            int16x8_t a3 = load_row(3u);
            int16x8_t a4 = load_row(4u);
            int16x8_t a5 = load_row(5u);
            cinepi_i32x8 top_lo = cinepi_lp(a0, a1, pc.prescale);
            cinepi_i32x8 top_hi = cinepi_hp_top(
                pre(a0), pre(a1), pre(a2), pre(a3), pre(a4), pre(a5));
            quantise_and_store(0u, top_lo, top_hi);

            // Interior outputs. i=1 uses rows 0..5. The next output uses rows
            // 2..7, so retain a2..a5 and fetch only rows 6 and 7.
            for (uint32_t i = 1u; i + 1u < outs; ++i) {
                cinepi_i32x8 lo = cinepi_lp(a2, a3, pc.prescale);
                cinepi_i32x8 hi = cinepi_hp6(
                    pre(a0), pre(a1), pre(a2), pre(a3), pre(a4), pre(a5));
                quantise_and_store(i, lo, hi);
                if (i + 2u < outs) {
                    const uint32_t next_row = 2u * i + 4u;
                    a0 = a2;
                    a1 = a3;
                    a2 = a4;
                    a3 = a5;
                    a4 = load_row(next_row);
                    a5 = load_row(next_row + 1u);
                }
            }

            // Bottom boundary output uses the final six input rows.
            const uint32_t bottom0 = last - 4u;
            a0 = load_row(bottom0 + 0u);
            a1 = load_row(bottom0 + 1u);
            a2 = load_row(bottom0 + 2u);
            a3 = load_row(bottom0 + 3u);
            a4 = load_row(bottom0 + 4u);
            a5 = load_row(bottom0 + 5u);
            cinepi_i32x8 bottom_lo = cinepi_lp(a4, a5, pc.prescale);
            cinepi_i32x8 bottom_hi = cinepi_hp_bottom(
                pre(a0), pre(a1), pre(a2), pre(a3), pre(a4), pre(a5));
            quantise_and_store(outs - 1u, bottom_lo, bottom_hi);
        }
    };

    run_columns(0u, low_vector_end, false);
    run_columns(half, high_vector_end, true);

    // Scalar remainder columns. Unaligned NEON loads are legal on AArch64, so
    // the high-frequency half can still begin exactly at `half`; only the final
    // fewer-than-eight columns of each half remain scalar.
    for (uint32_t line_i = low_vector_end; line_i < half; ++line_i)
        for (uint32_t i = 0; i < outs; ++i)
            cinepi_scalar_one_output(src, dst, pc, line_i, i, last, outs);
    for (uint32_t line_i = high_vector_end; line_i < pc.count; ++line_i)
        for (uint32_t i = 0; i < outs; ++i)
            cinepi_scalar_one_output(src, dst, pc, line_i, i, last, outs);
}
#endif  // CINEPI_HAVE_NEON_WAVELET

// ===========================================================================
// v1.7.53 CPU wavelet kernels.
//
// WHY THIS EXISTS. v1.7.52's NEON vertical pass sweeps eight columns at a time
// down the whole plane, then returns to the top for the next eight columns.
// Each row touch consumes 16 bytes of a 64-byte cache line and the plane is far
// larger than L2, so by the time columns 8..15 are wanted every line holding
// them has been evicted. The vertical pass therefore reads its source about
// four times and writes its destination about four times (write-allocate means
// a partial line store still pulls the line in). Counting one plane traversal
// as a "pass", one wavelet level currently costs roughly:
//
//     horizontal   read src 1 + write scratch 1
//     vertical     read scratch 4 + write dst 4
//     total       ~10 plane passes
//
// v1.7.53 inverts the vertical loop nest: rows outer, columns inner. Output row
// i needs source rows 2i-2 .. 2i+3, so consecutive output rows share four of six
// source rows and those rows stay resident in L1 (six rows of a 1920-wide plane
// is 23 KiB against the A76's 64 KiB L1D). Every access is now a full sequential
// stream, which the hardware prefetcher handles and which uses whole cache
// lines:
//
//     horizontal   read src 1 + write scratch 1
//     vertical     read scratch 1 + write dst 2   (two output rows per iteration)
//     total       ~5 plane passes
//
// That is a DRAM-traffic argument, not a measured speed-up. It predicts the
// vertical pass becomes compute-bound rather than bandwidth-bound; whether the
// A76 realises it is a hardware question and RUN_V1_7_53_CPU_WAVELET_AB.sh is
// the instrument for answering it.
//
// EXACTNESS IS NOT NEGOTIABLE. Every lane still computes exactly what
// cinepi_scalar_one_output() computes: int32 intermediates, arithmetic >>3,
// saturating narrow at the store, quantiser applied to the unsaturated value.
// The loop nest changed; the arithmetic did not. crosscheck proves it.
// ===========================================================================
#if CINEPI_HAVE_NEON_WAVELET

// -- vertical, row-major -----------------------------------------------------
// One output row i of the dir==1 pass, over the column range [from, to).
// horizontal_high selects the quantiser pair, exactly as the scalar's
// `line_i >= pc.count/2` test does.
template <int VEC>
static inline void cinepi_v53_vertical_row(
        const int16_t* s0, const int16_t* s1, const int16_t* s2,
        const int16_t* s3, const int16_t* s4, const int16_t* s5,
        const int16_t* l0, const int16_t* l1,
        int16_t* dlo, int16_t* dhi,
        uint32_t from, uint32_t to, int prescale, int kind,
        const cinepi_quant_neon& q_lo, const cinepi_quant_neon& q_hi,
        uint32_t& vector_end) {
    const uint32_t width = to - from;
    const uint32_t step = 8u * uint32_t(VEC);
    const uint32_t blocks = width / step;
    uint32_t col = from;
    for (uint32_t b = 0; b < blocks; ++b, col += step) {
        for (int v = 0; v < VEC; ++v) {
            const uint32_t c = col + 8u * uint32_t(v);
            const int16x8_t a0 = vld1q_s16(s0 + c);
            const int16x8_t a1 = vld1q_s16(s1 + c);
            const int16x8_t a2 = vld1q_s16(s2 + c);
            const int16x8_t a3 = vld1q_s16(s3 + c);
            const int16x8_t a4 = vld1q_s16(s4 + c);
            const int16x8_t a5 = vld1q_s16(s5 + c);
            const int16x8_t p0 = prescale == 0 ? a0 : cinepi_pre_p2(a0);
            const int16x8_t p1 = prescale == 0 ? a1 : cinepi_pre_p2(a1);
            const int16x8_t p2 = prescale == 0 ? a2 : cinepi_pre_p2(a2);
            const int16x8_t p3 = prescale == 0 ? a3 : cinepi_pre_p2(a3);
            const int16x8_t p4 = prescale == 0 ? a4 : cinepi_pre_p2(a4);
            const int16x8_t p5 = prescale == 0 ? a5 : cinepi_pre_p2(a5);
            cinepi_i32x8 hi;
            if (kind == 0)      hi = cinepi_hp_top(p0, p1, p2, p3, p4, p5);
            else if (kind == 2) hi = cinepi_hp_bottom(p0, p1, p2, p3, p4, p5);
            else                hi = cinepi_hp6(p0, p1, p2, p3, p4, p5);
            // The lowpass pair is always two of the six rows already loaded, so
            // it costs no extra traffic; l0/l1 select which.
            cinepi_i32x8 lo = cinepi_lp(vld1q_s16(l0 + c), vld1q_s16(l1 + c), prescale);
            lo = cinepi_quant(lo, q_lo);
            hi = cinepi_quant(hi, q_hi);
            vst1q_s16(dlo + c, cinepi_sat(lo));
            vst1q_s16(dhi + c, cinepi_sat(hi));
        }
    }
    vector_end = col;
}

static void cpu_wavelet_vertical_rowmajor(const int16_t* src, int16_t* dst,
                                          const CpuWaveletPush& pc) {
    const uint32_t last = pc.axis_len - 2u;
    const uint32_t outs = last / 2u + 1u;
    const uint32_t half = pc.count / 2u;

    const cinepi_quant_neon q_none = cinepi_make_quant(1);
    const cinepi_quant_neon q_lh = cinepi_make_quant(pc.q_lh);
    const cinepi_quant_neon q_hl = cinepi_make_quant(pc.q_hl);
    const cinepi_quant_neon q_hh = cinepi_make_quant(pc.q_hh);

    for (uint32_t i = 0; i < outs; ++i) {
        uint32_t r0;
        int kind;
        if (i == 0u)               { r0 = 0u;        kind = 0; }
        else if (2u * i == last)   { r0 = last - 4u; kind = 2; }
        else                       { r0 = 2u * i - 2u; kind = 1; }

        const int16_t* const s0 = src + size_t(r0 + 0u) * pc.stride;
        const int16_t* const s1 = src + size_t(r0 + 1u) * pc.stride;
        const int16_t* const s2 = src + size_t(r0 + 2u) * pc.stride;
        const int16_t* const s3 = src + size_t(r0 + 3u) * pc.stride;
        const int16_t* const s4 = src + size_t(r0 + 4u) * pc.stride;
        const int16_t* const s5 = src + size_t(r0 + 5u) * pc.stride;
        // Lowpass is src[2i] + src[2i+1]. Relative to r0 that is rows 0/1 at the
        // top, rows 4/5 at the bottom, rows 2/3 in the interior.
        const int16_t* const l0 = (kind == 0) ? s0 : (kind == 2 ? s4 : s2);
        const int16_t* const l1 = (kind == 0) ? s1 : (kind == 2 ? s5 : s3);

        int16_t* const dlo = dst + size_t(i) * pc.out_stride;
        int16_t* const dhi = dst + size_t(outs + i) * pc.out_stride;

        uint32_t low_end = 0u, high_end = half;
        if (g_cpu_wavelet_vec_blocks >= 2) {
            // Wide blocks first, then one narrow pass to pick up a leftover
            // eight-column group, then scalar for anything under a vector.
            cinepi_v53_vertical_row<2>(s0, s1, s2, s3, s4, s5, l0, l1, dlo, dhi,
                                       0u, half, pc.prescale, kind,
                                       q_none, q_hl, low_end);
            cinepi_v53_vertical_row<1>(s0, s1, s2, s3, s4, s5, l0, l1, dlo, dhi,
                                       low_end, half, pc.prescale, kind,
                                       q_none, q_hl, low_end);
            cinepi_v53_vertical_row<2>(s0, s1, s2, s3, s4, s5, l0, l1, dlo, dhi,
                                       half, pc.count, pc.prescale, kind,
                                       q_lh, q_hh, high_end);
            cinepi_v53_vertical_row<1>(s0, s1, s2, s3, s4, s5, l0, l1, dlo, dhi,
                                       high_end, pc.count, pc.prescale, kind,
                                       q_lh, q_hh, high_end);
        } else {
            cinepi_v53_vertical_row<1>(s0, s1, s2, s3, s4, s5, l0, l1, dlo, dhi,
                                       0u, half, pc.prescale, kind,
                                       q_none, q_hl, low_end);
            cinepi_v53_vertical_row<1>(s0, s1, s2, s3, s4, s5, l0, l1, dlo, dhi,
                                       half, pc.count, pc.prescale, kind,
                                       q_lh, q_hh, high_end);
        }

        // Fewer-than-a-vector remainders at the end of each half, in the
        // already-proven scalar code.
        for (uint32_t c = low_end; c < half; ++c)
            cinepi_scalar_one_output(src, dst, pc, c, i, last, outs);
        for (uint32_t c = high_end; c < pc.count; ++c)
            cinepi_scalar_one_output(src, dst, pc, c, i, last, outs);
    }
}

// -- horizontal --------------------------------------------------------------
// Two changes from v1.7.52. First, the deinterleaved block loaded for the next
// iteration is the same block v1.7.52 loaded separately as a half-width vld2_s16
// tail, so carrying it across the iteration removes one load pair per eight
// outputs. Second, v1.7.52's clean-up loop ran over all `outs` outputs and
// skipped the ones the vector loop had done -- about 950 wasted iterations of
// loop overhead per line at level 1. It now visits only the outputs that are
// actually left.
static void cpu_wavelet_horizontal_v53(const int16_t* src, int16_t* dst,
                                       const CpuWaveletPush& pc) {
    const uint32_t last = pc.axis_len - 2u;
    const uint32_t outs = last / 2u + 1u;
    /* wav1 (hpass_vqadd, the identity the file already proves for the
       vertical emit): at prescale 0 with no quantiser the lowpass is
       clamp((int32)e1 + (int32)o1), which IS vqaddq_s16 -- one instruction
       for the nine of cinepi_lp + cinepi_sat. Level 1 is the widest level,
       so this covers 4.15M lowpass outputs per 4K frame. Levels 2/3 use
       prescale 2 and are untouched; the quantiser fields are checked, not
       assumed, exactly as cpu_wavelet_pass_neon's lofast does. */
    const bool lofast = pc.prescale == 0 &&
                        pc.q_lh <= 1 && pc.q_hl <= 1 && pc.q_hh <= 1;
    for (uint32_t line_i = 0; line_i < pc.count; ++line_i) {
        const int16_t* row = src + size_t(line_i) * pc.stride;
        int16_t* orow = dst + size_t(line_i) * pc.out_stride;
        uint32_t i = 1u;
        // Output 0 uses the top coefficient set and output outs-1 the bottom
        // set, so the vector loop covers [1, outs-1).
        if (outs >= 3u && 2u * 1u - 2u + 32u <= pc.axis_len) {
            int16x8x2_t A = vld2q_s16(row + 0);
            for (; i + 8u <= outs - 1u; i += 8u) {
                const uint32_t base = 2u * i - 2u;
                if (base + 32u > pc.axis_len) break;
                const int16x8x2_t B = vld2q_s16(row + base + 16);
                const int16x8_t e0 = A.val[0], o0 = A.val[1];
                const int16x8_t e1 = vextq_s16(A.val[0], B.val[0], 1);
                const int16x8_t o1 = vextq_s16(A.val[1], B.val[1], 1);
                const int16x8_t e2 = vextq_s16(A.val[0], B.val[0], 2);
                const int16x8_t o2 = vextq_s16(A.val[1], B.val[1], 2);
                const cinepi_i32x8 hi = (pc.prescale == 0)
                    ? cinepi_hp6(e0, o0, e1, o1, e2, o2)
                    : cinepi_hp6(cinepi_pre_p2(e0), cinepi_pre_p2(o0),
                                 cinepi_pre_p2(e1), cinepi_pre_p2(o1),
                                 cinepi_pre_p2(e2), cinepi_pre_p2(o2));
                if (lofast) {   // loop-invariant: the compiler unswitches
                    vst1q_s16(orow + i, vqaddq_s16(e1, o1));
                } else {
                    const cinepi_i32x8 lo = cinepi_lp(e1, o1, pc.prescale);
                    vst1q_s16(orow + i, cinepi_sat(lo));
                }
                vst1q_s16(orow + outs + i, cinepi_sat(hi));
                A = B;
            }
        }
        cinepi_scalar_one_output(src, dst, pc, line_i, 0u, last, outs);
        for (uint32_t k = i; k < outs; ++k)
            cinepi_scalar_one_output(src, dst, pc, line_i, k, last, outs);
    }
}

static void cpu_wavelet_pass_v53(const int16_t* src, int16_t* dst,
                                 const CpuWaveletPush& pc) {
    if (pc.prescale != 0 && pc.prescale != 2) { cpu_wavelet_pass_scalar(src, dst, pc); return; }
    const uint32_t last = pc.axis_len - 2u;
    const uint32_t outs = last / 2u + 1u;
    if (outs < 3u || pc.count == 0u) { cpu_wavelet_pass_scalar(src, dst, pc); return; }
    if (pc.dir == 0u) cpu_wavelet_horizontal_v53(src, dst, pc);
    else              cpu_wavelet_vertical_rowmajor(src, dst, pc);
}

#endif  // CINEPI_HAVE_NEON_WAVELET

static void cpu_wavelet_pass(const int16_t* src, int16_t* dst, const CpuWaveletPush& pc) {
#if CINEPI_HAVE_NEON_WAVELET
    if (g_cpu_wavelet_use_neon) {
        if (g_cpu_wavelet_kernel_v53) cpu_wavelet_pass_v53(src, dst, pc);
        else                          cpu_wavelet_pass_neon(src, dst, pc);
        return;
    }
#endif
    cpu_wavelet_pass_scalar(src, dst, pc);
}

// Bit-for-bit host reference for shaders/vc5_forward_26.comp. This verifies
// the full descriptor, push-constant, level and ping-pong schedule.
static void cpu_wavelet_pass_scalar(const int16_t* src, int16_t* dst, const CpuWaveletPush& pc) {
    const uint32_t last = pc.axis_len - 2u;
    const uint32_t outs = last / 2u + 1u;
    for (uint32_t g = 0; g < outs * pc.count; ++g)
        cinepi_scalar_one_output(src, dst, pc, g / outs, g % outs, last, outs);
}

struct ModeSpec { std::string name; std::array<int,10> quant_table{}; };

// One persistent, one-plane workspace per processing thread. The old CPU path
// allocated and zero-filled a four-plane scratch frame for every input frame,
// even though planes are transformed sequentially and only one scratch plane is
// live at a time. At 4K this removes repeated heap work and reduces per-worker
// scratch from about 15.8 MiB to about 4.0 MiB.
static thread_local std::vector<int16_t> g_cpu_wavelet_workspace;

static int16_t* cpu_wavelet_workspace(size_t plane_stride_elems) {
    if (g_cpu_wavelet_workspace.size() < plane_stride_elems)
        g_cpu_wavelet_workspace.resize(plane_stride_elems);
    return g_cpu_wavelet_workspace.data();
}

static void cpu_transform_schedule(const Options& o, const ModeSpec* mode, int16_t* planes,
                                   size_t plane_stride_elems, size_t row_stride) {
    const auto specs = plane_specs(o);
    int16_t* const scratch = cpu_wavelet_workspace(plane_stride_elems);
    for (int p = 0; p < 4; ++p) {
        int16_t* ping = planes + size_t(p) * plane_stride_elems;
        int16_t* pong = scratch;
        int cw = specs[size_t(p)].width;
        int ch = specs[size_t(p)].height;
        for (int level = 1; level <= specs[size_t(p)].levels; ++level) {
            const CpuWaveletPush h{uint32_t(cw), uint32_t(ch), uint32_t(row_stride),
                                   uint32_t(row_stride), 0u, level == 1 ? 0 : 2};
            cpu_wavelet_pass(ping, pong, h);
            const int base = 10 - 3 * level;
            const int q1 = mode ? mode->quant_table[size_t(base)] : std::max(1, o.quant);
            const int q2 = mode ? mode->quant_table[size_t(base + 1)] : std::max(1, o.quant);
            const int q3 = mode ? mode->quant_table[size_t(base + 2)] : std::max(1, o.quant);
            const CpuWaveletPush v{uint32_t(ch), uint32_t(cw), uint32_t(row_stride),
                                   uint32_t(row_stride), 1u, 0, q1, q2, q3};
            cpu_wavelet_pass(pong, ping, v);
            cw /= 2;
            ch /= 2;
        }
    }
}


// Complete the remaining nested wavelet levels in one cacheable coefficient
// frame. Levels already produced by V51 remain outside the active top-left LL
// rectangle and are therefore preserved. With three pipeline entropy workers,
// three different frames can execute this CPU tail concurrently without
// creating per-frame helper threads.
// ===========================================================================
// v1.7.54 fused cascaded CPU wavelet.
//
// WHAT IT REPLACES. cpu_transform_schedule() runs six separable passes per
// plane (H,V at each of three levels), each one a full traversal of DRAM-sized
// buffers. Even with the v1.7.53 row-major vertical pass that is ~4 plane
// traversals per level, and the level-1 and level-2 LL rectangles are written
// out and read straight back in.
//
// WHAT THIS DOES. One pass. Plane rows stream in; each row is horizontally
// transformed into an 8-row ring; whenever six consecutive H rows are present
// the vertical filter emits one output row, whose three highpass bands go
// straight to their final position in the coefficient plane and whose lowpass
// row is pushed directly into the NEXT level's ring. Levels 1 and 2 never
// write their LL to memory at all.
//
// WHY IT IS SAFE NOW. The in-place hazard that blocks fusion -- V writing
// output row ch/2+i before H has read plane row ch/2+i -- only exists when the
// source and destination are the same buffer. Reading from the split output and
// writing to a separate coefficient plane removes it entirely. The cost is one
// extra frame-sized allocation, which disappears again once the split is fused
// into the front (phase 2).
//
// LAYOUT IS UNCHANGED. The coefficient plane comes out in exactly the standard
// quadrant layout cpu_transform_schedule() produces, so the entropy coder, the
// CRC gates and the GPR container are all untouched. This is a drop-in
// replacement, not a new ABI.
//
// EXACTNESS. The horizontal filter is not reimplemented: it calls the shipped
// cpu_wavelet_pass() with count=1, so a single row goes through the same code
// as always. The vertical filter is a pointer-addressed form of the same
// arithmetic -- the ring rows are not contiguous, so cinepi_scalar_one_output()
// cannot address them, but every intermediate keeps the same width and the same
// rounding. crosscheck_fused() proves byte equality against the real schedule.
// ===========================================================================


// Rows held per level. The live window is never more than six (output i needs
// rows 2i-2..2i+3 and the next output needs 2i..2i+5), so eight is one whole
// step of headroom and keeps the modulo cheap.
// Six rows, not eight. Output i is emitted the moment row 2i+3 arrives and
// reads rows 2i-2..2i+3 -- six, the six most recent. The next push lands on
// (2i+4) % 6 == (2i-2) % 6 and overwrites a row output i has finished with,
// while output i+1 needs 2i..2i+5 and row 2i is still resident. Six is
// sufficient and five is not: forcing five changes the container CRC.
// Eight used to be headroom bought to make the wrap a mask; the wrap is now
// a modulo paid once per OUTPUT ROW, never per lane, and the four-plane
// resident ring set drops from ~215 KB to ~161 KB of a 512 KB L2.
static constexpr int CINEPI_FUSED_RING = 6;

// hp_flat_skip / emit_kind1 (v1.5). See patch_v15_skip_kind.py.

// Hit-rate counters. Only touched when the stats switch is on, which the
// suite sets for the single-threaded gate frame and clears before timing --
// so the timed path never pays for them and there is no data race.

// Largest magnitude this divisor sends to zero: cpu_quantize_exact(v, q) == 0
// for every |v| <= T. Monotone in |v|, so a scan from 0 is exact and it runs
// once per level at init, never per frame.
static int cinepi_zero_threshold(int divisor) {
    if (divisor <= 1) return 0;          // identity: only 0 maps to 0
    int t = 0;
    while (t < 32766 && cpu_quantize_exact(t + 1, divisor) == 0) ++t;
    return t;
}

// hi = (numerator) >> 3 with |numerator - 4| <= B. floor((B+4)/8) <= T and
// floor((4-B)/8) >= -T both hold exactly when B <= 8T+3, so that is the
// largest bound this band may show and still be provably quantised to zero.
// Capped at 65534 because a saturated bound reads as 65535 and must fail.
/* ---- Noise Clean: the measured IMX585 dark-noise profile -----------------
 *
 * Roadmap item 15. Pixel Clean is a fixed, general-purpose sparsity rule; this
 * is sensor-specific, and it zeroes a pre-quant coefficient only when its
 * magnitude is statistically inside the noise the sensor produces at the ACTIVE
 * GAIN. So it has to be indexed by ISO, and it is worthless -- actively harmful
 * -- if the profile does not match the sensor and mode in front of it.
 *
 * MEASURED on this rig, 2026-08-20: 48-frame lens-cap dark stacks at every gain,
 * RAW12, 3840x2160 active area, via tools/qraw_dark_calibrate.py. `sigma` is the
 * per-pixel temporal read noise in SENSOR CODES (robust: median of per-pixel
 * MAD x 1.4826). `lut_gain` is the effective gain of GP-Log2 for noise sitting
 * at that ISO's measured pedestal, computed by mapping a Gaussian through the
 * encoder's own LUT -- not a tangent, because at k=599 the curve is violently
 * convex at black (~98 LUT units per code at x=0, ~57 four codes up).
 *
 * That GP-Log2 term is the largest factor in the whole chain, 34x to 54x. One
 * sensor code at black becomes ~92 working units, so a few codes of read noise
 * reach the wavelet as a few HUNDRED units. It is why shadow noise is expensive
 * here, and it is the energy this lever exists to stop coding.
 *
 * MEASURED, AND THE REASON THIS STAYS OFF: on the reference frame at m5 with
 * Pixel Clean already on, adding Noise Clean gives
 *
 *     k=0.25   2,418,586 -> 1,672,034 bytes (-31%),  fps 38.2 -> 43.5
 *     k=0.50   2,418,586 ->   860,082 bytes (-64%),  fps 38.2 -> 58.5
 *     k=1.00   2,418,586 ->   508,205 bytes (-79%),  fps 38.2 -> 66.8
 *
 * A 64% reduction on a normal scene is not noise being removed. It is IMAGE
 * being removed, and the roadmap's own rule applies: never promote something
 * merely because the file got smaller.
 *
 * WHY A FIXED PER-BAND THRESHOLD IS THE WRONG SHAPE HERE -- the finding that
 * matters most from this work. In a LOG domain a fixed number of sensor codes of
 * read noise becomes a LARGE coefficient increment near black and a SMALL one
 * away from it, because that is what the log slope does (98 LUT units per code at
 * the pedestal, ~57 four codes up, and far less further out). Real detail, by
 * contrast, is roughly scale-invariant in log: a given contrast RATIO produces a
 * similar increment wherever it sits. So a single threshold calibrated at the
 * black level is correct only in the very deepest shadows and grossly too large
 * everywhere else -- which is exactly what the numbers above show.
 *
 * Noise Clean therefore has to be SIGNAL-DEPENDENT even for dark noise, which a
 * per-band scalar cannot express. That is roadmap item 17's territory (a
 * calibrated variance model indexed by local signal) with item 19's GS-guided
 * edge protection as the safety net. Until one of those exists this lever is
 * calibrated, wired, verified inert when off -- and not something to ship on.
 *
 * LL is never touched, per the roadmap. Default OFF, default k 0.5. */
struct NoiseCleanPoint { int iso; double sigma; double lut_gain; };
static const NoiseCleanPoint g_imx585_raw12_noise[] = {
    {  100,  1.483, 50.5 },
    {  200,  1.483, 50.5 },
    {  400,  2.965, 54.3 },
    {  800,  3.707, 47.7 },
    { 1600,  5.930, 41.0 },
    { 3200, 10.378, 33.9 },
};

/* Component transform noise gains. GS is a MEAN of the two green phases so it
 * attenuates noise; GD is their DIFFERENCE so it adds. Root-sum-square of each
 * output's coefficients, treating the phases as independent:
 *   GS=(G1+G2)/2 -> sqrt(2)/2   RG,BG=X-(G1+G2)/2 -> sqrt(1.5)   GD=G1-G2 -> sqrt(2)
 * A flat threshold across components would therefore be wrong in both
 * directions at once: GD carries twice the noise of GS in every band. */
static double noise_clean_component_gain(int plane) {
    switch (plane) {
    case 0:  return 0.70710678;   /* GS */
    case 1:
    case 2:  return 1.22474487;   /* RG, BG */
    case 3:  return 1.41421356;   /* GD */
    default: return 1.0;
    }
}

/* Pre-quant noise gain from a component plane to one (level, band). The wavelet
 * is LINEAR, so it propagates a sigma exactly by root-sum-square of its taps:
 * the interior 6-tap highpass (-1,-1,8,-8,1,1)/8 gives sqrt(132)/8 and the
 * lowpass pair gives sqrt(2). Levels 2 and 3 read a twice-lowpassed LL that has
 * been prescaled by >>2. Validated against simulated white noise through the
 * real integer filters (tools/qraw_noise_profile.py --self-test, agreement
 * within 0.6%). Valid because dark read noise is spatially WHITE, which the
 * same stacks confirm: row/column FPN measures sigma 0.0000. */
static double noise_clean_band_gain(int level, int band_of_level) {
    const double HP = 1.43614066;   /* sqrt(132)/8 */
    const double LP = 1.41421356;   /* sqrt(2)     */
    double ll = 1.0;
    for (int l = 1; l < level; ++l)
        ll *= 0.25 * LP * LP;       /* prescale >>2, then two lowpasses */
    const double base = (level == 1) ? ll : ll;
    switch (band_of_level) {
    case 0:  return base * HP * LP;   /* LH: horizontal high, vertical low  */
    case 1:  return base * LP * HP;   /* HL: horizontal low,  vertical high */
    default: return base * HP * HP;   /* HH: high both ways                 */
    }
}

/* Threshold for one band, in pre-quant coefficient units. Zero when the lever is
 * off, the ISO is unknown, or the profile has no entry -- an uncalibrated Noise
 * Clean must do NOTHING rather than guess. */
static int noise_clean_threshold(const Options& o, int plane, int level,
                                 int band_of_level) {
    /* CINEPI_QRAW_ISO lets a bench run state the gain the profile should be read
     * at, the same way CINEPI_CAQ and CINEPI_PIXEL_CLEAN reach their features.
     * The live camera passes it through cfg->iso instead, from the control plane,
     * so it always matches the gain actually on the sensor. */
    static const int env_iso = [] {
        const char *e = std::getenv("CINEPI_QRAW_ISO");
        const int v = (e && *e) ? std::atoi(e) : 0;
        return v > 0 ? v : 0;
    }();
    /* Enable and k also fall back to the environment. The library reads these
     * for the camera path, but --execution cpu-gpr drives Options directly, so
     * without this the lever was silently inert in every bench run -- which is
     * exactly what the first A/B showed: identical bytes at every k. */
    static const bool env_on = [] {
        /* CINEPI_NOISE_CLEAN now selects a PROFILE for the revised self-estimating
         * method, so it can no longer double as this lever's on switch -- both
         * would fire and the frame would be thresholded twice, by two different
         * rules, with only one of them named in the log. The ISO lever therefore
         * has its own explicit opt-in and is off unless a run asks for it. */
        const char *iso_on = std::getenv("CINEPI_NOISE_CLEAN_ISO");
        if (!iso_on || !*iso_on) return false;
        return !(std::strcmp(iso_on, "0") == 0 || std::strcmp(iso_on, "off") == 0);
    }();
    static const double env_k = [] {
        const char *e = std::getenv("CINEPI_NOISE_CLEAN_K");
        const double v = (e && *e) ? std::atof(e) : 0.0;
        return (v > 0.0 && v < 8.0) ? v : 0.0;
    }();
    const int iso = o.iso > 0 ? o.iso : env_iso;
    const bool on = (o.noise_clean && env_on) || env_on;
    const double kmul = env_k > 0.0 ? env_k
                      : (o.noise_clean_k > 0.0 ? o.noise_clean_k : 0.5);
    if (!on || iso <= 0) return 0;
    const NoiseCleanPoint* pt = nullptr;
    for (const auto& p : g_imx585_raw12_noise)
        if (p.iso == iso) { pt = &p; break; }
    if (!pt) return 0;              /* no interpolation: calibrate the gain you use */
    const double sigma = pt->sigma * pt->lut_gain
                       * noise_clean_component_gain(plane)
                       * noise_clean_band_gain(level, band_of_level);
    const double t = kmul * sigma;
    if (t < 1.0) return 0;
    return int(t > 32766.0 ? 32766.0 : t);
}

/* ---- Noise Clean: the existing zero threshold, widened --------------------
 *
 * NOT A DENOISE PASS. Noise Clean does not filter, average or scan anything: it
 * makes the encoder's OWN "this coefficient will become zero" threshold wider on
 * selected level-1 bands, and then the encoder does what it already did. There is
 * no second pass over coefficients, no per-frame statistic, and no state.
 *
 * That is the whole design, and it is why the lever can pay for itself. The
 * threshold is resolved once per band per setup and handed to cinepi_make_quant()
 * exactly as Pixel Clean's is, so:
 *
 *   - the hot loop is unchanged: one vcleq/vbslq per four lanes, reusing the |v|
 *     the quantiser already computed;
 *   - the zero happens INSIDE the quantiser, so every downstream consumer sees
 *     it. The non-zero mask is built by wav_direct_store8() from the value the
 *     quantiser returned, and the sidecar mask by v2_sidecar_store2() from the
 *     same vector, so both are generated AFTER the widening by construction --
 *     there is no ordering to get wrong;
 *   - it feeds cinepi_hp_bound_limit() and hh_provably_zero, so more high-pass
 *     work becomes provably pointless and is skipped outright. This is where the
 *     throughput comes from, and it is the same mechanism that made Pixel Clean
 *     faster as well as smaller rather than a trade against it.
 *
 * THE UNIT IS A QUANTISED LEVEL, which is what makes one profile mean the same
 * thing at every grade: `n` says "coefficients that would have quantised to +-n
 * or less are zero instead", and cinepi_absquant_le_n_threshold() converts that
 * to the exact pre-quant magnitude, monotonically and once. Pixel Clean's colour
 * rule is the n=1 case of the same idea on RG/BG level 1; Noise Clean is that
 * idea made band-aware and available at n>1.
 *
 * (An earlier revision estimated a noise floor per frame -- a sparse MAD over
 * |HH1|, used one frame late -- as the brief describes. It was removed: measured
 * on the static reference frame it produced the same zeros as n=1 at the grades
 * that matter, while adding a sampling pass, per-instance histogram state and a
 * frame of latency to get there. A deterministic widening needs none of it.) */
struct NoiseCleanProfile {
    /* Levels to prune per component, at wavelet level 1. 0 = leave alone. */
    int gs_hh1   = 0;
    int gd_hh1   = 0;
    int rgbg_hh1 = 0;
    int rgbg_lh1 = 0;
    int rgbg_hl1 = 0;
};

/* Coverage follows the brief's table; the depth is this package's own, because
 * the brief's thresholds were expressed for a self-estimated floor that no longer
 * exists. GS -- the mean of both green phases, the sharpest and least noisy
 * plane -- is untouched below Strong, exactly as the brief asks. */
static NoiseCleanProfile make_noise_clean_profile(int mode) {
    switch (mode) {
    /* MEASURED ON THE STATIC REFERENCE FRAME, m5, CAQ strong + Pixel Clean:
     *
     *   soft    -3.4% size   +0.0% fps
     *   medium  see below    (the shipping default)
     *   strong  -7.9% size   +4.6% fps
     *
     * The +-1 level on RG/BG is deliberately NOT what separates these profiles,
     * because Pixel Clean already does it: its colour rule zeroes |q| <= 1 on all
     * three level-1 bands of RG and BG. That is why an earlier ladder measured
     * soft and medium byte-identical at every grade -- everything they asked for
     * beyond GD's diagonal had already happened. The depth on colour is what
     * separates them now. */
    case 1:  /* Soft:   +-1, which past Pixel Clean means GD's diagonal only   */
        return { 0, 1, 1, 1, 1 };
    case 2:  /* Medium: strong's depth on colour and GD, with GS left alone    */
        return { 0, 2, 2, 1, 1 };
    case 3:  /* Strong: deeper on colour and GD, and GS's diagonal joins in    */
        return { 1, 2, 2, 1, 1 };
    default:
        return {};
    }
}

static const char* noise_clean_mode_name(int mode) {
    switch (mode) {
    case 1: return "soft";
    case 2: return "medium";
    case 3: return "strong";
    default: return "off";
    }
}

/* Mode and strength for the active encoder instance. Globals for the same reason
 * g_pixel_clean is one: fused_level_init() is reached from five helper classes and
 * threading Options through all of them to carry two scalars is not worth it.
 * Written at setup, before any encode thread starts; read-only after. */
static int g_noise_clean_mode = [] {
    const char *e = std::getenv("CINEPI_NOISE_CLEAN");
    if (!e || !*e) return 0;
    if (!std::strcmp(e, "0") || !std::strcmp(e, "off"))    return 0;
    if (!std::strcmp(e, "1") || !std::strcmp(e, "soft"))   return 1;
    if (!std::strcmp(e, "2") || !std::strcmp(e, "medium")) return 2;
    if (!std::strcmp(e, "3") || !std::strcmp(e, "strong")) return 3;
    return 0;
}();
/* A continuous trim on the resolved threshold, so a profile can be tightened or
 * loosened without redefining which bands it covers. 1.0 leaves it exact. */
static double g_noise_clean_strength = [] {
    const char *e = std::getenv("CINEPI_NOISE_CLEAN_STRENGTH");
    const double v = (e && *e) ? std::atof(e) : 0.0;
    return (v >= 0.25 && v <= 4.0) ? v : 1.0;
}();

/* ---- Pixel Clean, as one exact integer threshold ------------------------
 *
 * The lever is defined as two rules in two different domains: a PRE-quant
 * dead-zone at 125% of the half-step, and a POST-quant rule that forces
 * quantised RG/BG level-1 coefficients of +-1 to zero. Applied literally that
 * costs a float compare plus a later branch on every coefficient.
 *
 * They collapse into a single threshold, exactly, because cpu_quantize_exact()
 * is monotone non-decreasing in |v|:
 *
 *   rule 1:  zero iff |v| < 0.625*q          <=>  |v| <= floor((5q-1)/8)
 *   rule 2:  zero iff |quant(v)| <= 1        <=>  |v| <= L1(q)
 *
 * so the coefficient is zeroed iff |v| <= max(D, L1) on an eligible band and
 * |v| <= D elsewhere. Integer only, computed once per band at level init, and
 * it lands in the same place the encoder already keeps its exact zero
 * thresholds -- so it can feed the existing provable-zero compute skips too.
 *
 * Deliberately NOT applied to LL (band 0): the brief's dead-zone test is
 * guarded `band != LL`, and a dead-zone on the DC band would be visible. */
static int cinepi_deadzone125_threshold(int divisor) {
    if (divisor <= 1) return 0;      /* identity divisor: nothing to widen */
    const long long t = (5LL * divisor - 1) / 8LL;   /* |v| < 0.625*divisor */
    return int(t > 32766 ? 32766 : t);
}

/* Largest magnitude whose EXACT quantiser yields -1, 0 or +1. Monotone in |v|,
 * so the scan is exact; runs once per band at init, never per frame. */
static int cinepi_absquant_le1_threshold(int divisor) {
    /* NO early return for divisor <= 1. At divisor 1 the quantiser is the
     * IDENTITY, so |quant(v)| <= 1 still means |v| <= 1 and the colour rule
     * still has something to do -- and the m-mode ladder does reach a divisor of
     * 1 on high-pass bands at the finest grades. Bailing out here left Pixel
     * Clean partially inert exactly there; caught by the exhaustive equivalence
     * test against the brief's literal two-rule form (scratchpad/pc_equiv.cpp),
     * which is the reason that test exists. */
    int t = 0;
    while (t < 32766 && std::abs(cpu_quantize_exact(t + 1, divisor)) <= 1) ++t;
    return t;
}

/* Largest magnitude whose EXACT quantiser yields at most n in absolute value.
 * The generalisation of the +-1 helper above, and monotone for the same reason,
 * so the same scan is exact. n <= 0 gives 0, which is the off state.
 *
 * This is what lets a threshold DEFINED in quantised levels be applied as a
 * pre-quant integer compare: the hot loop keeps its single vcleq per four lanes
 * and never learns that the rule it is enforcing was expressed in another
 * domain. Runs once per band per frame, not per coefficient. */
static int cinepi_absquant_le_n_threshold(int divisor, int n) {
    if (n <= 0) return 0;
    int t = 0;
    while (t < 32766 && std::abs(cpu_quantize_exact(t + 1, divisor)) <= n) ++t;
    return t;
}

/* `l1_colour` = this is RG or BG at wavelet level 1, the only place the tested
 * colour-prune rule applies. */
static int cinepi_pixel_clean_threshold(int divisor, bool l1_colour) {
    int t = cinepi_deadzone125_threshold(divisor);
    if (l1_colour)
        t = std::max(t, cinepi_absquant_le1_threshold(divisor));
    return t;
}

static int cinepi_hp_bound_limit(int zero_threshold) {
    const long long lim = 8LL * zero_threshold + 3;
    return int(lim > 65534 ? 65534 : lim);
}

// Slot of row r in a ring of `rows` rows. Called once per OUTPUT ROW, never
// per lane. For the default 8 this stays the mask it always was.
static inline int cinepi_ring_slot(int r, int rows) {
    // The `rows == 8` mask fast path went with the 8-row ring. `rows` is a
    // field rather than a constant only so the emit and the push agree on
    // one value; it is always CINEPI_FUSED_RING. This runs once per OUTPUT
    // ROW, never per lane, so a modulo here costs nothing measurable.
    return r % rows;
}

// v1.7.55 non-temporal band stores.
//
// WHY. The 3-thread all-CPU measurement on a CM5 peaked at 58.1 fps and then
// REGRESSED to 49.1 at four threads, with per-frame cost inflating 2.5x from
// one thread to four. It was not thermal (throttled=0x0, 66C) and not DRAM
// saturation (3.59 GiB/s against an 8.49 GiB/s measured ceiling). What is left
// is L3 capacity: four workers need 4 x ~215 KiB of rings resident in a 2 MiB
// L3 while streaming ~63 MiB/frame each straight through it. The coefficient
// writes are never read again by the producing core, so allocating them evicts
// exactly the rings the cascade depends on.
//
// STNP is the ARM non-temporal store pair: 32 bytes, hinting no-allocate. Two
// adjacent int16x8 results of the same destination row pair naturally.
//
// IMPORTANT: STNP is architecturally a HINT. A76 is permitted to ignore it. So
// this is off unless asked for, and --cpu-nontemporal on|off is an A/B, not an
// assumption. If it does nothing on your silicon, the flag says so honestly.
static bool g_cpu_nontemporal_bands = false;
// C4. Set before any worker starts.
static bool g_cpu_fused_prefetch = false;
// S1: NEON-assisted split. The four LUT gathers per quad cannot vectorise (no
// gather on NEON), but everything after them can: 8 quads of green/difference
// arithmetic collapse from ~64 scalar ops + 32 scalar stores to ~10 vector ops
// + 4 vector stores. The gathers dominate, so the expected win is small -- this
// is a 1-2% candidate, priced accordingly.
static bool g_cpu_split_neon = false;
// split_hadd (v1.5): the three chroma-difference outputs are
//     (x + midpoint_x2) >> sh
// and at sh == 1 that is EXACTLY vhaddq_s16(x, midpoint) -- one instruction
// instead of an add and a variable shift, three times per 8 quads. The green
// output already uses vhadd for (g1+g2)>>1; this extends the same identity to
// the rest of the row.
//
// EXACTNESS. vhadd computes (a+b)>>1 in a wider intermediate, so it cannot
// overflow where the add-then-shift form could. The operands are bounded
// anyway: LUT outputs are 0..working_max <= 4095, so the differences are
// within +/-4095 and x + 4096 is within 1..8191 -- comfortably int16 either
// way. Truncation direction matches: both are arithmetic >> 1.
//
// sh is 1 only when true_12bit is off, which is the shipped configuration.
// At sh == 0 the shift is a no-op and vhadd would HALVE the value, so the
// candidate checks sh and falls through rather than assuming.

// --- v1.4 lab candidates ---------------------------------------------------
// ll_vqadd: the identity-quantiser lowpass IS a saturating int16 add.
//
// For every column below L.half the vertical emit is called with
// q_lo = cinepi_make_quant(1), i.e. the quantiser is DISABLED. What the code
// then computes is
//     lo  = (int32)lp0 + (int32)lp1          (cinepi_lp, prescale 0)
//     out = clamp(lo, -32768, 32767)         (cinepi_sat -> vqmovn)
// and clamping the exact sum of two int16 values to the int16 range is the
// DEFINITION of a signed saturating add. vqaddq_s16 is one instruction where
// the widening form is nine (4x vmovl, 2x vadd, 2x vqmovn, 1x vcombine).
// This is the LL band of every level plus level 3's final LL, so it fires on
// half the columns of the whole cascade.

// strip_rows: plane-STRIP cascade interleave. 0 keeps the shipped row
// interleave. See v2_fused_frame.

#if defined(__aarch64__)
#define CINEPI_HAVE_STNP 1
// stnp q,q writes 32 bytes with a non-temporal hint. GCC has no builtin for it
// (__builtin_nontemporal_store is clang-only), so this is inline asm.
static inline void cinepi_stnp_q2(int16_t* dst, int16x8_t a, int16x8_t b) {
    asm volatile("stnp %q0, %q1, [%2]" :: "w"(a), "w"(b), "r"(dst) : "memory");
}
#else
#define CINEPI_HAVE_STNP 0
#endif

struct V2CompactSink {
    void* ctx = nullptr;
    void (*highpass_row)(void*, int plane, int level, int band, int row, const int16_t* src, int width) = nullptr;
    void (*final_ll_row)(void*, int plane, int row, const int16_t* src, int width) = nullptr;
    /* wav1: when the sink is the production CpuDirectHybridSlot this points at
       it, and v2_level_emit takes the register-direct row path: highpass lanes
       are range-checked and narrowed to int8 IN REGISTERS as they leave the
       vertical filter, so the per-row int16 staging bounce and the second
       fits-scan in write_highpass_row() disappear. NULL keeps the callback
       path byte-for-byte (any other sink, odd geometries, scalar builds). */
    void* direct_slot = nullptr;
};
static thread_local V2CompactSink* g_v2_compact_sink = nullptr;

struct FusedLevel {
    int cw = 0, ch = 0;             // dimensions of THIS level's input
    int outs = 0;                   // ch/2
    int last = 0;                   // ch-2
    int half = 0;                   // cw/2
    int prescale = 0;               // horizontal prescale: 0 at level 1, else 2
    int q_lh = 1, q_hl = 1, q_hh = 1;
    /* Pixel Clean thresholds for this level's three high-pass bands. Zero means
     * the lever is off for that band, which is a no-op by construction. */
    int pc_t_lh = 0, pc_t_hl = 0, pc_t_hh = 0;
    bool final_level = false;

    // hp_flat_skip: the largest difference bound that still guarantees a zero
    // quantised highpass, for the two quantised highpass bands.
    int hp_limit_hl = -1, hp_limit_hh = -1;
    /* Set by fused_level_init() when q_hh guarantees a zero HH band for every
     * possible input, so the interior highpass can be skipped outright. */
    bool hh_provably_zero = false;
    int ring_rows = CINEPI_FUSED_RING;   // 8, or 6 under the ring6 candidate
    std::vector<int16_t> ring;      // ring_rows rows of cw
    std::vector<int16_t> ll_row;    // cw/2 staging for the next level

    int next_src_row = 0;           // how many input rows have been H-filtered
    int next_out = 0;               // next vertical output row to emit

    // Direct hybrid output uses only row-sized staging here. Highpass rows are
    // immediately narrowed to int8 or written to exact int16 fallback storage,
    // so the 15.8 MiB full coefficient frame never exists for this path.
    bool compact_output = false;
    int compact_plane = -1, compact_level = -1;
    std::vector<int16_t> compact_lo_row, compact_hi_row, compact_ll_row;

    int16_t* dst = nullptr;         // the coefficient plane
    size_t dst_stride = 0;
};

struct FusedPlane {
    std::array<FusedLevel, 3> level{};
};

// --- scalar vertical, one column, six explicit tap rows ---------------------
// Mirrors cinepi_scalar_one_output() for dir==1 with prescale==0, which is what
// the vertical pass always is. cpu_pre(v,0) == v, and (a+b+((1<<0)-1))>>0 == a+b,
// so the prescale drops out and only the six-tap highpass and the quantiser
// remain.
static inline void cinepi_fused_v_scalar(
        const int16_t* t0, const int16_t* t1, const int16_t* t2,
        const int16_t* t3, const int16_t* t4, const int16_t* t5,
        const int16_t* l0, const int16_t* l1,
        int col, int kind, bool horizontal_high,
        int q_lh, int q_hl, int q_hh,
        int16_t* lo_dest, int16_t* hi_dest,
        int pc_lh = 0, int pc_hl = 0, int pc_hh = 0) {
    const int a0 = t0[col], a1 = t1[col], a2 = t2[col];
    const int a3 = t3[col], a4 = t4[col], a5 = t5[col];
    int hi;
    if (kind == 0)
        hi = (5 * a0 - 11 * a1 + 4 * a2 + 4 * a3 - a4 - a5 + 4) >> 3;
    else if (kind == 2)
        hi = (a0 + a1 - 4 * a2 - 4 * a3 + 11 * a4 - 5 * a5 + 4) >> 3;
    else
        hi = (-a0 - a1 + 8 * a2 - 8 * a3 + a4 + a5 + 4) >> 3;
    int lo = int(l0[col]) + int(l1[col]);
    if (horizontal_high) {
        lo = cpu_quantize_pc(lo, q_lh, pc_lh);
        hi = cpu_quantize_pc(hi, q_hh, pc_hh);
    } else {
        hi = cpu_quantize_pc(hi, q_hl, pc_hl);
    }
    lo_dest[col] = int16_t(std::clamp(lo, -32768, 32767));
    hi_dest[col] = int16_t(std::clamp(hi, -32768, 32767));
}

#if CINEPI_HAVE_NEON_WAVELET
// Eight columns at a time. Reuses the shipped helpers unchanged, so the highpass
// coefficient sets, the >>3, the quantiser and the saturating store are the same
// instructions the v1.7.53 pass already emits.
static inline void cinepi_fused_v_neon8(
        const int16_t* t0, const int16_t* t1, const int16_t* t2,
        const int16_t* t3, const int16_t* t4, const int16_t* t5,
        const int16_t* l0, const int16_t* l1,
        int col, int out, int kind,
        const cinepi_quant_neon& q_lo, const cinepi_quant_neon& q_hi,
        int16_t* lo_dest, int16_t* hi_dest) {
    const int16x8_t a0 = vld1q_s16(t0 + col);
    const int16x8_t a1 = vld1q_s16(t1 + col);
    const int16x8_t a2 = vld1q_s16(t2 + col);
    const int16x8_t a3 = vld1q_s16(t3 + col);
    const int16x8_t a4 = vld1q_s16(t4 + col);
    const int16x8_t a5 = vld1q_s16(t5 + col);
    cinepi_i32x8 hi;
    if (kind == 0)      hi = cinepi_hp_top(a0, a1, a2, a3, a4, a5);
    else if (kind == 2) hi = cinepi_hp_bottom(a0, a1, a2, a3, a4, a5);
    else                hi = cinepi_hp6(a0, a1, a2, a3, a4, a5);
    // The lowpass pair is always two of the six rows already in registers, and
    // l0/l1 are separate pointers the compiler cannot prove do not alias, so it
    // reloads them: 8 loads per 8 outputs where 6 suffice. Select instead.
    const int16x8_t lp0 = (kind == 0) ? a0 : (kind == 2 ? a4 : a2);
    const int16x8_t lp1 = (kind == 0) ? a1 : (kind == 2 ? a5 : a3);
    (void)l0; (void)l1;
    cinepi_i32x8 lo = cinepi_lp(lp0, lp1, 0);
    lo = cinepi_quant(lo, q_lo);
    hi = cinepi_quant(hi, q_hi);
    vst1q_s16(lo_dest + out, cinepi_sat(lo));
    vst1q_s16(hi_dest + out, cinepi_sat(hi));
}
#endif

static bool g_cpu_wavelet_reuse_context = false;

/* Pixel Clean, as a file-scope switch set once from Options.
 *
 * A global rather than a parameter threaded through every level-init caller,
 * because that is how this file already carries encoder-wide switches
 * (g_cpu_wavelet_reuse_context above) and the alternative is touching eight
 * call sites in five different helper classes just to forward one bool. It is
 * written once, before any encode thread starts, and only ever read after
 * that. */
/* Noise Clean thresholds for the active encoder instance, resolved once at
 * setup: [plane][band 1..9]. Zero everywhere when the lever is off, which makes
 * it a no-op by construction. A global for the same reason g_pixel_clean is one
 * -- fused_level_init() is reached from five different helper classes and
 * threading Options through all of them to carry one table is not worth it. */
static int g_noise_clean_t[4][10] = {};

static bool g_pixel_clean = [] {
    /* CINEPI_PIXEL_CLEAN=1/on enables the lever without a rebuild, the same way
     * CINEPI_CAQ and CINEPI_BAND_Q reach their features. Options::pixel_clean
     * ORs into this at setup (see cpu_quant_tables_caq) so a caller can enable
     * it programmatically too. */
    const char *e = std::getenv("CINEPI_PIXEL_CLEAN");
    if (!e || !*e) return false;
    return !(std::strcmp(e, "0") == 0 || std::strcmp(e, "off") == 0);
}();

/* The widened zero threshold for one level-1 band, or 0 when this profile does
 * not cover it. Pure function of the profile and the band's own divisor: no
 * state, no sampling, no frame history. Called once per band per setup from
 * fused_level_init(), which is where Pixel Clean's threshold is resolved too. */
static int noise_clean_band_threshold(int plane, int band_of_level, int divisor) {
    const int mode = g_noise_clean_mode;
    if (mode <= 0 || plane < 0 || plane > 3) return 0;
    const NoiseCleanProfile p = make_noise_clean_profile(mode);
    const bool colour = (plane == 1 || plane == 2);
    int n = 0;
    switch (band_of_level) {
    case 0: n = colour ? p.rgbg_lh1 : 0; break;                  /* LH1 */
    case 1: n = colour ? p.rgbg_hl1 : 0; break;                  /* HL1 */
    default:                                                     /* HH1 */
        n = (plane == 0) ? p.gs_hh1 : (plane == 3) ? p.gd_hh1
          : p.rgbg_hh1;
        break;
    }
    if (n <= 0) return 0;
    const int t = cinepi_absquant_le_n_threshold(divisor, n);
    if (g_noise_clean_strength == 1.0) return t;
    const double scaled = double(t) * g_noise_clean_strength;
    if (!(scaled >= 1.0)) return 0;
    return int(scaled > 32766.0 ? 32766.0 : std::floor(scaled + 0.5));
}

/* `pixel_clean` enables the lever; `l1_colour_plane` says this plane is RG or
 * BG, the only planes whose LEVEL-1 bands take the tested colour-prune rule.
 * Both default off so an un-updated caller keeps the current output exactly. */
static void fused_level_init(FusedLevel& L, int cw, int ch, int level,
                             const std::array<int, 10>& quant,
                             int16_t* dst, size_t dst_stride,
                             int plane = -1) {
    L.cw = cw; L.ch = ch;
    L.outs = ch / 2;
    L.last = ch - 2;
    L.half = cw / 2;
    L.prescale = (level == 1) ? 0 : 2;
    const int base = 10 - 3 * level;
    L.q_lh = quant[size_t(base)];
    L.q_hl = quant[size_t(base + 1)];
    L.q_hh = quant[size_t(base + 2)];
    L.final_level = (level == 3);
    /* Pixel Clean, resolved here so the hot loop only ever compares against an
     * integer. The colour rule is level 1 only, and only on RG/BG. */
    if (g_pixel_clean) {
        /* Plane order is the encoder's own: 0 = GS, 1 = RG, 2 = BG, 3 = GD.
         * A caller that does not say which plane it is gets the dead-zone only,
         * never the colour rule -- the conservative half of the lever. */
        const bool l1c = (plane == 1 || plane == 2) && level == 1;
        L.pc_t_lh = cinepi_pixel_clean_threshold(L.q_lh, l1c);
        L.pc_t_hl = cinepi_pixel_clean_threshold(L.q_hl, l1c);
        L.pc_t_hh = cinepi_pixel_clean_threshold(L.q_hh, l1c);
    } else {
        L.pc_t_lh = L.pc_t_hl = L.pc_t_hh = 0;
    }
    /* Noise Clean rides the SAME threshold, by taking whichever of the two
     * levers is wider for this band.
     *
     * They are independently switchable and they mean different things -- one is
     * a fixed sparsity rule, the other a sensor-calibrated noise floor -- but
     * both reduce to "zero this coefficient if |v| <= T", so combining them
     * costs nothing at all in the hot loop: still one integer compare per
     * vector. That is the whole payoff of having expressed Pixel Clean as an
     * exact threshold rather than as two rules. */
    if (plane >= 0 && plane < 4) {
        const int base = 10 - 3 * level;
        L.pc_t_lh = std::max(L.pc_t_lh, g_noise_clean_t[plane][base]);
        L.pc_t_hl = std::max(L.pc_t_hl, g_noise_clean_t[plane][base + 1]);
        L.pc_t_hh = std::max(L.pc_t_hh, g_noise_clean_t[plane][base + 2]);
    }
    /* The revised Noise Clean rides the same threshold as the other two levers,
     * for the same reason and at the same price: still one integer compare per
     * vector in the hot loop, and the widened threshold feeds the provable-zero
     * compute skips below exactly as Pixel Clean's does. Level 1 only, because
     * every profile in the brief covers level-1 bands only. */
    if (level == 1 && g_noise_clean_mode > 0) {
        L.pc_t_lh = std::max(L.pc_t_lh, noise_clean_band_threshold(plane, 0, L.q_lh));
        L.pc_t_hl = std::max(L.pc_t_hl, noise_clean_band_threshold(plane, 1, L.q_hl));
        L.pc_t_hh = std::max(L.pc_t_hh, noise_clean_band_threshold(plane, 2, L.q_hh));
    }
    /* The provable-zero compute skips below key off the widest magnitude a band
     * can send to zero. Pixel Clean only ever WIDENS that, so folding it in is
     * safe and is where the compute saving comes from -- more of the high-pass
     * work becomes provably pointless and is skipped outright. */
    const int z_hl = std::max(cinepi_zero_threshold(L.q_hl), L.pc_t_hl);
    const int z_hh = std::max(cinepi_zero_threshold(L.q_hh), L.pc_t_hh);
    L.hp_limit_hl = cinepi_hp_bound_limit(z_hl);
    L.hp_limit_hh = cinepi_hp_bound_limit(z_hh);
    /* ---- HH is provably zero, so stop computing it -----------------------
     * Band pruning (CINEPI_BAND_Q, the E1/E2/E3 candidates) quantises a whole
     * subband away by setting its divisor to 32767. Until now that only
     * stopped the band being EMITTED: the 6-tap highpass, the quantise, the
     * int8 narrow, the mask byte and 2.07 MB/frame of zero stores all still
     * ran. At E1 -- the shipped LT grade -- that is 25% of every high-pass
     * coefficient produced, for a band the reader then skips 64 coefficients
     * at a time.
     *
     * When the divisor's zero threshold covers the widest value the filter can
     * produce, the quantised result is zero for EVERY possible input, so
     * skipping the work cannot change the output. working_max is validated to
     * 1..4095 (see --working-max: "peak coeff = working_max x4 <= 16380"), and
     * levels 2 and 3 prescale back into that range, so plane samples are <=
     * 4095 at every level and these bounds are unconditional:
     *
     *   interior horizontal highpass, coeffs (-1,-1,8,-8,1,1)/8:
     *       |h| <= (10*4095 + 4) >> 3   = 5119
     *   interior vertical highpass over those, sum|w| = 20:
     *       |v| <= (20*5119 + 4) >> 3   = 12798
     *
     * cpu_quantize_exact(t, q) is zero while t <= zero_threshold(q), and the
     * pre-quant magnitude is bounded by 20*T before the >>3, so the guarantee
     * is 20*T <= 8*Z + 3 -- the same form cinepi_hp_bound_limit() uses, but
     * computed here without its 65534 clamp, which would reject the very case
     * being tested for.
     *
     * The EDGE filters are deliberately NOT covered: the first/last horizontal
     * output uses (5,-11,4,4,-1,-1)/8 (|h| <= 6654) and the first/last output
     * ROW uses the same wider set, and 20*6654 and 26*5119 both exceed the
     * threshold by ~1.5%. Those keep the original path -- 2 of 120 column
     * groups and 2 of 540 rows, 0.58% of the band -- which makes the skip
     * bit-identical unconditionally rather than only in the common case.
     *
     * Safety invariant: the skipped region must already be zero, and stays so.
     * CpuDirectHybridSlot zero-initialises i8_/nz_ at construction, reset()
     * touches only the tile-format flags, this emit is the region's only
     * writer, and q_hh is fixed for the life of an encoder instance (changing
     * the grade drains and re-runs setup_encoder(), which builds new slots). */
    {
        const long long Z = std::max(cinepi_zero_threshold(L.q_hh), L.pc_t_hh);
        L.hh_provably_zero = (20LL * 5119LL) <= (8LL * Z + 3LL);
    }
    L.ring_rows = CINEPI_FUSED_RING;
    const size_t ring_need = size_t(L.ring_rows) * size_t(cw);
    const size_t ll_need = size_t(L.half);
    if (L.ring.size() != ring_need) L.ring.assign(ring_need, int16_t(0));
    else if (!g_cpu_wavelet_reuse_context) std::fill(L.ring.begin(), L.ring.end(), int16_t(0));
    if (L.ll_row.size() != ll_need) L.ll_row.assign(ll_need, int16_t(0));
    else if (!g_cpu_wavelet_reuse_context) std::fill(L.ll_row.begin(), L.ll_row.end(), int16_t(0));
    L.next_src_row = 0;
    L.next_out = 0;
    L.compact_output = false;
    L.compact_plane = -1; L.compact_level = -1;
    L.dst = dst;
    L.dst_stride = dst_stride;
}

static void fused_level_push(FusedPlane& plane, int index, const int16_t* src_row);

static void fused_level_emit(FusedPlane& plane, int index) {
    FusedLevel& L = plane.level[size_t(index)];
    const int i = L.next_out;
    int kind, r0;
    if (i == 0)                    { kind = 0; r0 = 0; }
    else if (2 * i == L.last)      { kind = 2; r0 = L.last - 4; }
    else                           { kind = 1; r0 = 2 * i - 2; }

    const int16_t* const base = L.ring.data();
    const int rr = L.ring_rows;
    auto row = [&](int r) {
        return base + size_t(cinepi_ring_slot(r, rr)) * size_t(L.cw);
    };
    const int16_t* const t0 = row(r0 + 0);
    const int16_t* const t1 = row(r0 + 1);
    const int16_t* const t2 = row(r0 + 2);
    const int16_t* const t3 = row(r0 + 3);
    const int16_t* const t4 = row(r0 + 4);
    const int16_t* const t5 = row(r0 + 5);
    // Lowpass pair is src[2i] and src[2i+1]: rows 0/1 at the top, 4/5 at the
    // bottom, 2/3 in the interior, relative to r0.
    const int16_t* const l0 = (kind == 0) ? t0 : (kind == 2 ? t4 : t2);
    const int16_t* const l1 = (kind == 0) ? t1 : (kind == 2 ? t5 : t3);

    int16_t* const dst_lo = L.dst + size_t(i) * L.dst_stride;
    int16_t* const dst_hi = L.dst + size_t(L.outs + i) * L.dst_stride;
    // Levels 1 and 2 never write their LL: the next level overwrites that whole
    // rectangle. Only level 3's LL is part of the final coefficient frame.
    int16_t* const ll_dest = L.final_level ? dst_lo : L.ll_row.data();

#if CINEPI_HAVE_NEON_WAVELET
    if (g_cpu_wavelet_use_neon) {
        const cinepi_quant_neon q_none = cinepi_make_quant(1);
        const cinepi_quant_neon q_lh = cinepi_make_quant(L.q_lh, L.pc_t_lh);
        const cinepi_quant_neon q_hl = cinepi_make_quant(L.q_hl, L.pc_t_hl);
        const cinepi_quant_neon q_hh = cinepi_make_quant(L.q_hh, L.pc_t_hh);
        int c = 0;
#if CINEPI_HAVE_STNP
        // Sixteen columns per iteration so the two highpass vectors form one
        // 32-byte non-temporal pair.
        if (g_cpu_nontemporal_bands) {
            int16_t hi_pair[16];
            for (; c + 16 <= L.half; c += 16) {
                // The LL half is read again by the next level, so it keeps a
                // normal cached store at its true column. Only the highpass
                // band is staged for the non-temporal pair.
                cinepi_fused_v_neon8(t0, t1, t2, t3, t4, t5, l0, l1, c, 0, kind,
                                     q_none, q_hl, ll_dest + c, hi_pair);
                cinepi_fused_v_neon8(t0, t1, t2, t3, t4, t5, l0, l1, c + 8, 8, kind,
                                     q_none, q_hl, ll_dest + c, hi_pair);
                cinepi_stnp_q2(dst_hi + c, vld1q_s16(hi_pair), vld1q_s16(hi_pair + 8));
            }
        }
#endif
        for (; c + 8 <= L.half; c += 8)
            cinepi_fused_v_neon8(t0, t1, t2, t3, t4, t5, l0, l1, c, c, kind,
                                 q_none, q_hl, ll_dest, dst_hi);
        for (; c < L.half; ++c)
            cinepi_fused_v_scalar(t0, t1, t2, t3, t4, t5, l0, l1, c, kind, false,
                                  L.q_lh, L.q_hl, L.q_hh, ll_dest, dst_hi,
                                  L.pc_t_lh, L.pc_t_hl, L.pc_t_hh);
        // The high half writes its lowpass to the HL quadrant of the plane, at
        // the same column index, so dst_lo is addressed directly.
#if CINEPI_HAVE_STNP
        if (g_cpu_nontemporal_bands) {
            int16_t lo_pair[16], hi_pair[16];
            for (; c + 16 <= L.cw; c += 16) {
                cinepi_fused_v_neon8(t0, t1, t2, t3, t4, t5, l0, l1, c, 0, kind,
                                     q_lh, q_hh, lo_pair, hi_pair);
                cinepi_fused_v_neon8(t0, t1, t2, t3, t4, t5, l0, l1, c + 8, 8, kind,
                                     q_lh, q_hh, lo_pair, hi_pair);
                cinepi_stnp_q2(dst_lo + c, vld1q_s16(lo_pair), vld1q_s16(lo_pair + 8));
                cinepi_stnp_q2(dst_hi + c, vld1q_s16(hi_pair), vld1q_s16(hi_pair + 8));
            }
        }
#endif
        for (; c + 8 <= L.cw; c += 8)
            cinepi_fused_v_neon8(t0, t1, t2, t3, t4, t5, l0, l1, c, c, kind,
                                 q_lh, q_hh, dst_lo, dst_hi);
        for (; c < L.cw; ++c)
            cinepi_fused_v_scalar(t0, t1, t2, t3, t4, t5, l0, l1, c, kind, true,
                                  L.q_lh, L.q_hl, L.q_hh, dst_lo, dst_hi,
                                  L.pc_t_lh, L.pc_t_hl, L.pc_t_hh);
    } else
#endif
    {
        for (int c = 0; c < L.half; ++c)
            cinepi_fused_v_scalar(t0, t1, t2, t3, t4, t5, l0, l1, c, kind, false,
                                  L.q_lh, L.q_hl, L.q_hh, ll_dest, dst_hi,
                                  L.pc_t_lh, L.pc_t_hl, L.pc_t_hh);
        for (int c = L.half; c < L.cw; ++c)
            cinepi_fused_v_scalar(t0, t1, t2, t3, t4, t5, l0, l1, c, kind, true,
                                  L.q_lh, L.q_hl, L.q_hh, dst_lo, dst_hi,
                                  L.pc_t_lh, L.pc_t_hl, L.pc_t_hh);
    }

    ++L.next_out;
    if (!L.final_level)
        fused_level_push(plane, index + 1, ll_dest);
}

static void fused_level_push(FusedPlane& plane, int index, const int16_t* src_row) {
    FusedLevel& L = plane.level[size_t(index)];
    // Horizontal filter of exactly one row, through the shipped kernel. count=1
    // means line_i is always 0, so `stride` is never used and the arithmetic is
    // the same code path a full-plane horizontal pass takes.
    CpuWaveletPush h{};
    h.axis_len = uint32_t(L.cw);
    h.count = 1u;
    h.stride = uint32_t(L.cw);
    h.out_stride = uint32_t(L.cw);
    h.dir = 0u;
    h.prescale = L.prescale;
    cpu_wavelet_pass(src_row,
                     L.ring.data() + size_t(cinepi_ring_slot(L.next_src_row, L.ring_rows))
                                   * size_t(L.cw),
                     h);
    ++L.next_src_row;

    // Emit every output row whose six-row window is now complete.
    for (;;) {
        if (L.next_out >= L.outs) break;
        const int i = L.next_out;
        const int need_max = (i == 0) ? 5
                           : (2 * i == L.last) ? (L.ch - 1)
                           : (2 * i + 3);
        if (L.next_src_row <= need_max) break;
        fused_level_emit(plane, index);
    }
}

// One plane, three levels, one streaming pass.
//   src         plane from the split, cw x ch, row stride src_stride
//   dst         coefficient plane, standard quadrant layout, stride dst_stride
// src and dst MUST be different buffers.
static void fused_transform_plane(const int16_t* src, size_t src_stride,
                                  int16_t* dst, size_t dst_stride,
                                  int cw, int ch,
                                  const std::array<int, 10>& quant,
                                  FusedPlane& plane) {
    int w = cw, h = ch;
    for (int level = 1; level <= 3; ++level) {
        fused_level_init(plane.level[size_t(level - 1)], w, h, level, quant,
                         dst, dst_stride);
        w /= 2;
        h /= 2;
    }
    for (int y = 0; y < ch; ++y)
        fused_level_push(plane, 0, src + size_t(y) * src_stride);
}

static void fused_transform_frame(const int16_t* src_planes, int16_t* dst_planes,
                                  size_t plane_stride_elems, size_t row_stride,
                                  int cw, int ch,
                                  const std::array<int, 10>& quant,
                                  std::array<FusedPlane, 4>& planes) {
    for (int p = 0; p < 4; ++p)
        fused_transform_plane(src_planes + size_t(p) * plane_stride_elems, row_stride,
                              dst_planes + size_t(p) * plane_stride_elems, row_stride,
                              cw, ch, quant, planes[size_t(p)]);
}

// ===========================================================================
// Phase 2: the split fused into the front of the cascade.
//
// split_compand() reads two Bayer rows and writes one row of each of the four
// component planes. Those plane rows are the level-1 input, so materialising
// them as a full frame costs a 16.6 MB write and a 16.6 MB read at 4K that
// nothing else needs. Producing them one row at a time straight into the four
// cascades removes both.
//
// The arithmetic below is copied line for line from split_compand(); the only
// change is that it emits one row instead of a frame. split_row_crosscheck()
// proves the two agree.
//
// COST: four cascades now run concurrently instead of sequentially, so the
// resident ring set is 4x (~215 KB at 4K, against 512 KB of A76 L2) and the
// number of concurrent DRAM write streams goes from 4 to 16. Whether that
// trade is worth 33 MB/frame is a memory-controller question, so both drivers
// are kept and selectable.
// ===========================================================================
/* v3.13: SHIFTED threads o.src_shift into the gathers. It is a template
   parameter so the SHIFTED=false instantiation -- every historical caller --
   keeps byte-for-byte the codegen it always had; the SHIFTED=true one folds
   one register `lsr` between the raw load and the LUT load, both of which
   the gather chain already waits on, which is why the shift measures free.
   v3.14: BSWAP threads o.src_byteswap in the same way -- one register
   `rev16`/`bswap16` between the raw load and everything else (order: load ->
   byteswap -> shift -> LUT index), so a big-endian IMX585 RAW16 DMA buffer
   is read where it lies and the frame-wide software swap in libcamera can be
   skipped. Both false instantiations keep the historical codegen exactly. */
template <bool SHIFTED, bool BSWAP, bool HOTLUT>
static inline void fused_split_row_t(const uint16_t* row0, const uint16_t* row1,
                                     const uint16_t* lut, int pw, bool gbrg,
                                     int16_t* gs, int16_t* rg,
                                     int16_t* bg, int16_t* gd, int sh, int midpoint_x2,
                                     unsigned src_shift) {
    const auto idx = [src_shift](uint16_t v) -> unsigned {
        const unsigned s = BSWAP ? unsigned(uint16_t(__builtin_bswap16(v)))
                                 : unsigned(v);
        return SHIFTED ? (s >> src_shift) : s;
    };
    /* v3.15: HOTLUT reroutes each gather through the v1.14 exact compressed
       GP-Log lookup (8 KiB bucket bases + 32 KiB packed nibble deltas
       instead of the 128 KiB full 16-bit table that thrashes the 64 KiB L1).
       configure_cpu_hot_lut() exhaustively verified every one of the 65536
       codes against the full LUT before this instantiation can be selected,
       so the output is bit-identical by construction. The two loads it does
       (delta byte, bucket base) both depend only on the raw sample, so they
       issue in parallel inside the same load-latency-bound gather chain. */
    const auto cl = [&](uint16_t v) -> uint16_t {
        const unsigned i = idx(v);
        if (!HOTLUT) return lut[i];
        const uint32_t b = uint32_t(i) >> 4u;
        if (b - g_hot_lut_bad_lo < g_hot_lut_bad_span) return lut[i];
        const uint8_t packed = g_hot_lut_delta[size_t(i) >> 1u];
        const uint8_t delta = (i & 1u) ? uint8_t(packed >> 4u)
                                       : uint8_t(packed & 0x0Fu);
        return uint16_t(g_hot_lut_base[b] + delta);
    };
    if (gbrg) {
        int x = 0;
#if CINEPI_HAVE_NEON_WAVELET
        if (g_cpu_split_neon) {
            const int16x8_t vmid = vdupq_n_s16(int16_t(midpoint_x2));
            const int16x8_t vsh = vdupq_n_s16(int16_t(-sh));
            // (x + midpoint) >> 1 is exactly vhaddq_s16. sh is still
            // tested: at sh == 0 the shift is a no-op and vhadd would halve.
            const bool hadd = (sh == 1);
            int16_t g1a[8], ba[8], ra[8], g2a[8];
            for (; x + 8 <= pw; x += 8) {
                for (int k = 0; k < 8; ++k) {   // gathers: scalar by necessity
                    g1a[k] = int16_t(cl(row0[0])); ba[k] = int16_t(cl(row0[1])); row0 += 2;
                    ra[k]  = int16_t(cl(row1[0])); g2a[k] = int16_t(cl(row1[1])); row1 += 2;
                }
                const int16x8_t vg1 = vld1q_s16(g1a), vb = vld1q_s16(ba);
                const int16x8_t vr  = vld1q_s16(ra),  vg2 = vld1q_s16(g2a);
                const int16x8_t vgreen = vhaddq_s16(vg1, vg2);   // (g1+g2)>>1 exactly
                vst1q_s16(gs, vgreen);
                vst1q_s16(rg, (hadd ? vhaddq_s16(vsubq_s16(vr, vgreen), vmid) : vshlq_s16(vaddq_s16(vsubq_s16(vr, vgreen), vmid), vsh)));
                vst1q_s16(bg, (hadd ? vhaddq_s16(vsubq_s16(vb, vgreen), vmid) : vshlq_s16(vaddq_s16(vsubq_s16(vb, vgreen), vmid), vsh)));
                vst1q_s16(gd, (hadd ? vhaddq_s16(vsubq_s16(vg1, vg2), vmid) : vshlq_s16(vaddq_s16(vsubq_s16(vg1, vg2), vmid), vsh)));
                gs += 8; rg += 8; bg += 8; gd += 8;
            }
        }
#endif
        for (; x < pw; ++x) {
            const int g1 = cl(*row0++);
            const int b  = cl(*row0++);
            const int r  = cl(*row1++);
            const int g2 = cl(*row1++);
            const int green = (g1 + g2) >> 1;
            *gs++ = int16_t(green);
            *rg++ = int16_t((r - green + midpoint_x2) >> sh);
            *bg++ = int16_t((b - green + midpoint_x2) >> sh);
            *gd++ = int16_t((g1 - g2 + midpoint_x2) >> sh);
        }
    } else {  // RGGB stays scalar: the production sensor is GBRG
        for (int x = 0; x < pw; ++x) {
            const int r  = cl(*row0++);
            const int g1 = cl(*row0++);
            const int g2 = cl(*row1++);
            const int b  = cl(*row1++);
            const int green = (g1 + g2) >> 1;
            *gs++ = int16_t(green);
            *rg++ = int16_t((r - green + midpoint_x2) >> sh);
            *bg++ = int16_t((b - green + midpoint_x2) >> sh);
            *gd++ = int16_t((g1 - g2 + midpoint_x2) >> sh);
        }
    }
}

static inline void fused_split_row(const uint16_t* row0, const uint16_t* row1,
                                   const uint16_t* lut, int pw, bool gbrg,
                                   int16_t* gs, int16_t* rg,
                                   int16_t* bg, int16_t* gd, int sh, int midpoint_x2,
                                   unsigned src_shift = 0,
                                   bool src_byteswap = false) {
    if (g_cpu_hot_lut) {
        if (src_byteswap) {
            if (src_shift)
                fused_split_row_t<true, true, true>(row0, row1, lut, pw, gbrg,
                                                    gs, rg, bg, gd, sh, midpoint_x2, src_shift);
            else
                fused_split_row_t<false, true, true>(row0, row1, lut, pw, gbrg,
                                                     gs, rg, bg, gd, sh, midpoint_x2, 0u);
        } else if (src_shift) {
            fused_split_row_t<true, false, true>(row0, row1, lut, pw, gbrg,
                                                 gs, rg, bg, gd, sh, midpoint_x2, src_shift);
        } else {
            fused_split_row_t<false, false, true>(row0, row1, lut, pw, gbrg,
                                                  gs, rg, bg, gd, sh, midpoint_x2, 0u);
        }
        return;
    }
    if (src_byteswap) {
        if (src_shift)
            fused_split_row_t<true, true, false>(row0, row1, lut, pw, gbrg,
                                          gs, rg, bg, gd, sh, midpoint_x2, src_shift);
        else
            fused_split_row_t<false, true, false>(row0, row1, lut, pw, gbrg,
                                           gs, rg, bg, gd, sh, midpoint_x2, 0u);
    } else if (src_shift) {
        fused_split_row_t<true, false, false>(row0, row1, lut, pw, gbrg,
                                       gs, rg, bg, gd, sh, midpoint_x2, src_shift);
    } else {
        fused_split_row_t<false, false, false>(row0, row1, lut, pw, gbrg,
                                        gs, rg, bg, gd, sh, midpoint_x2, 0u);
    }
}

// RAW16 in, coefficient planes out, nothing else touching DRAM.
static void fused_split_transform_frame(const uint16_t* raw, const uint16_t* lut,
                                        int width, int height, bool gbrg, int split_sh, int split_mid,
                                        int16_t* dst_planes,
                                        size_t plane_stride_elems, size_t row_stride,
                                        const std::array<int, 10>& quant,
                                        std::array<FusedPlane, 4>& planes,
                                        std::vector<int16_t>& split_scratch) {
    const int pw = width / 2, ph = height / 2;
    for (int p = 0; p < 4; ++p) {
        int w = pw, h = ph;
        for (int level = 1; level <= 3; ++level) {
            fused_level_init(planes[size_t(p)].level[size_t(level - 1)], w, h, level,
                             quant,
                             dst_planes + size_t(p) * plane_stride_elems, row_stride,
                             p);
            w /= 2; h /= 2;
        }
    }
    if (split_scratch.size() < size_t(pw) * 4u)
        split_scratch.assign(size_t(pw) * 4u, int16_t(0));
    int16_t* const gs = split_scratch.data();
    int16_t* const rg = gs + pw;
    int16_t* const bg = rg + pw;
    int16_t* const gd = bg + pw;
    for (int y = 0; y < ph; ++y) {
        const uint16_t* row0 = raw + size_t(2 * y) * size_t(width);
        if (g_cpu_fused_prefetch && y + 4 < ph) {
            const uint16_t* ahead = raw + size_t(2 * (y + 4)) * size_t(width);
            for (int b = 0; b < width * 2; b += 64)
                __builtin_prefetch(reinterpret_cast<const char*>(ahead) + b, 0, 0);
        }
        fused_split_row(row0, row0 + size_t(width), lut, pw, gbrg, gs, rg, bg, gd, split_sh, split_mid);
        fused_level_push(planes[0], 0, gs);
        fused_level_push(planes[1], 0, rg);
        fused_level_push(planes[2], 0, bg);
        fused_level_push(planes[3], 0, gd);
    }
}

// Frame entry points. The fused cascade needs every level to have even extents,
// i.e. plane dimensions divisible by 8. 3840x2160 satisfies this; 1920x1080
// does not, and that is exactly the geometry where the six-pass schedule has a
// pre-existing over-read (see V1_7_54_FUSED_CASCADE.md). Rather than reproduce
// a defect, the fused path declines and the caller falls back.
static bool cpu_fused_geometry_ok(const Options& o) {
    const int pw = o.width / 2, ph = o.height / 2;
    return (pw % 8) == 0 && (ph % 8) == 0;
}

// ===========================================================================
// Component-Aware Quantisation (CAQ)
//
// Spends fewer bits on the colour-residual components than on green detail,
// which is where the eye is least able to see the loss. It is a BIT ALLOCATION
// change inside the existing GPR structure: the four standard components
// (GS / RG / BG / GD), their dimensions and their meanings are untouched, so the
// output stays ordinary GPR that a stock GoPro or Adobe decoder reads.
//
// THE RULE THAT MATTERS: these are RELATIVE multipliers on whatever the current
// production M-mode table resolves to. They are not, and must never become, a
// second ladder of absolute band values -- if the m1..m10 tuning changes, CAQ
// follows it automatically. This function is the only place the constants live.
//
//   profile   GS    RG/BG coarse  middle  fine    GD coarse  middle  fine
//   OFF       1.00      1.00       1.00   1.00      1.00      1.00   1.00
//   SOFT      1.00      1.00       1.10   1.25      1.00      1.00   1.08
//   MEDIUM    1.00      1.00       1.25   1.55      1.00      1.08   1.18
//   STRONG    1.00      1.10       1.50   2.00      1.00      1.15   1.35
//
// GS is never touched -- it carries the detail the picture is judged on. GD
// follows RG/BG in direction but on a gentler curve. Band 0 is the LL lowpass
// and keeps the production encoder's own policy: no CAQ, no new value.
//
// Band index -> wavelet level, from the ladder's own layout:
//   0        LL3 lowpass          (untouched)
//   1,2,3    level 3   COARSE     (LH3, HL3, HH3)
//   4,5,6    level 2   MIDDLE
//   7,8,9    level 1   FINE       (the finest detail, quantised hardest)
//
// The compute saving comes after the wavelet, not in it: coarser residual
// quantisation produces more zeros and smaller magnitudes for the entropy stage.
// ===========================================================================
enum CaqProfile { CAQ_OFF = 0, CAQ_SOFT = 1, CAQ_MEDIUM = 2, CAQ_STRONG = 3 };

struct CaqLevelScale { double coarse, middle, fine; };
struct CaqProfileDef { CaqLevelScale rg_bg, gd; };

static CaqProfileDef caq_profile_def(int profile) {
    switch (profile) {
    case CAQ_SOFT:   return { {1.00, 1.10, 1.25}, {1.00, 1.00, 1.08} };
    case CAQ_MEDIUM: return { {1.00, 1.25, 1.55}, {1.00, 1.08, 1.18} };
    case CAQ_STRONG: return { {1.10, 1.50, 2.00}, {1.00, 1.15, 1.35} };
    default:         return { {1.00, 1.00, 1.00}, {1.00, 1.00, 1.00} };
    }
}

static const char* caq_profile_name(int profile) {
    switch (profile) {
    case CAQ_SOFT:   return "soft";
    case CAQ_MEDIUM: return "medium";
    case CAQ_STRONG: return "strong";
    default:         return "off";
    }
}

static std::array<int,10> cpu_quant_table(const Options& o, const ModeSpec* mode) {
    std::array<int,10> q{};
    for (int k = 0; k < 10; ++k)
        q[size_t(k)] = mode ? mode->quant_table[size_t(k)] : std::max(1, o.quant);
    return q;
}

/* One effective table per GPR component, derived from the resolved BASE table.
 *
 * Plane order is the encoder's own: 0 = GS, 1 = RG, 2 = BG, 3 = GD (see the
 * split in v2_fused_frame_compact, which writes gs/rg/bg/gd in that order).
 *
 * With CAQ off this returns four copies of BASE, so the caller's arithmetic is
 * identical to the single-table path and the output is bit-exact. That is the
 * OFF regression requirement, met by construction rather than by testing. */
static std::array<std::array<int,10>,4>
cpu_quant_tables_caq(const Options& o, const ModeSpec* mode) {
    /* The one function every setup path calls with Options in hand, and always
     * before the first fused_level_init(), so it is where the programmatic half
     * of the Pixel Clean switch is folded into the file-scope flag. Monotone:
     * the environment or the option can turn it on, neither turns it off. */
    if (o.pixel_clean) g_pixel_clean = true;
    /* Same rule for the revised Noise Clean: the option can raise it, the
     * environment can raise it, neither silently lowers what the other asked
     * for. --execution cpu-gpr drives Options directly and the library drives
     * them from the control plane, so both have to land in the same place. */
    if (o.noise_clean_mode > g_noise_clean_mode)
        g_noise_clean_mode = std::clamp(o.noise_clean_mode, 0, 3);
    if (o.noise_clean_strength >= 0.25 && o.noise_clean_strength <= 4.0 &&
        std::getenv("CINEPI_NOISE_CLEAN_STRENGTH") == nullptr)
        g_noise_clean_strength = o.noise_clean_strength;
    /* Resolve the Noise Clean table here too: this is the one function every
     * setup path calls with Options in hand, always before the first
     * fused_level_init(). */
    for (int p = 0; p < 4; ++p)
        for (int b = 0; b < 10; ++b)
            g_noise_clean_t[p][b] = 0;
    /* NOT gated on o.noise_clean/o.iso here: those are the LIBRARY's fields, and
     * --execution cpu-gpr drives Options directly, so gating here meant the
     * environment fallbacks inside noise_clean_threshold() were never reached and
     * the lever was silently inert in every bench run -- byte-identical output at
     * every k, which is exactly what the first three A/B sweeps showed. Let the
     * threshold function decide; it returns 0 when the lever is off or the ISO is
     * uncalibrated, which is the same no-op by a shorter route. */
    {
        bool any = false;
        for (int p = 0; p < 4; ++p)
            for (int level = 1; level <= 3; ++level)
                for (int k = 0; k < 3; ++k) {
                    const int t = noise_clean_threshold(o, p, level, k);
                    g_noise_clean_t[p][(10 - 3 * level) + k] = t;
                    if (t > 0) any = true;
                }
        static bool said = false;
        if (any && !said) {
            said = true;
            std::fprintf(stderr,
                "NOISE_CLEAN thresholds (pre-quant coeff units): "
                "GS L1 %d/%d/%d  RG L1 %d/%d/%d  GD L1 %d/%d/%d  GD L3 %d/%d/%d\n",
                g_noise_clean_t[0][7], g_noise_clean_t[0][8], g_noise_clean_t[0][9],
                g_noise_clean_t[1][7], g_noise_clean_t[1][8], g_noise_clean_t[1][9],
                g_noise_clean_t[3][7], g_noise_clean_t[3][8], g_noise_clean_t[3][9],
                g_noise_clean_t[3][1], g_noise_clean_t[3][2], g_noise_clean_t[3][3]);
        }
    }
    const std::array<int,10> base = cpu_quant_table(o, mode);
    std::array<std::array<int,10>,4> q{ base, base, base, base };
    if (o.caq == CAQ_OFF)
        return q;

    const CaqProfileDef d = caq_profile_def(o.caq);
    for (int plane = 1; plane < 4; ++plane) {          /* plane 0 = GS: untouched */
        const CaqLevelScale& s = (plane == 3) ? d.gd : d.rg_bg;
        for (int b = 1; b < 10; ++b) {                 /* band 0 = LL: untouched  */
            const double f = (b >= 7) ? s.fine : (b >= 4) ? s.middle : s.coarse;
            long v = std::lround(double(base[size_t(b)]) * f);
            /* The quantiser is an integer divisor: 1 is lossless-ish and 32767 is
             * the sentinel the band-pruning feature uses to zero a band. Clamping
             * here keeps a strong profile on a coarse mode from accidentally
             * pruning a band outright, which would be a different feature. */
            if (v < 1) v = 1;
            if (v > 32766) v = 32766;
            q[size_t(plane)][size_t(b)] = int(v);
        }
    }
    return q;
}

// ===========================================================================
// The v2 fused kernel, promoted from the m5 CPU optimisation campaign.
//
// Stride/crop-aware split that reads the transport frame in place (no 15.8 MB
// repack), 16-quad grouped LUT gathers, register-direct STNP emit, the
// nonzero-mask sidecar the entropy coder consumes, and dead-store suppression
// for all-zero blocks. CM5-measured at 10 s windows with the rest of the
// winner stack: 37.1 / 46.9 / 59.6 fps at 12 / 11 / 10-bit effective
// precision.
//
// It lives here, not in the m5 suite, so the UI benchmark, the encoder
// library and cinepi-raw all reach it. The m5 suite includes this file, so
// its candidates keep working unchanged and its byte-compare self-check
// still stands guard over every variant.
// ===========================================================================
enum class EmitKind  { Shipped, RegDirect, RegDirectL3LL };
enum class SplitKind { Shipped, Unroll16, Pipelined };

// ---------------------------------------------------------------------------
// v2 fused chain: the shipped fused split + cascade, generalised with
//   * a source row stride and crop offsets (stride_split), and
//   * selectable split and emit variants,
// all byte-compared against the shipped path before any timing run.
// ---------------------------------------------------------------------------

#if CINEPI_HAVE_NEON_WAVELET
// The shipped cinepi_fused_v_neon8 stores its results; this variant returns
// them so an STNP pair can be built from registers without the stack bounce.
//
// v1.4: templated on the two lab candidates so the SELECTION happens once per
// output row and the column loop stays branch-free.
//   W       -- use the widening-intrinsic kernels (wl_widen)
//   LlFast  -- the caller has PROVEN q_lo is the identity quantiser, so the
//              lowpass collapses to one vqaddq_s16 (ll_vqadd)
// <false,false> is the shipped body, character for character.
// The register-direct vertical emit on one 8-lane column group. Used by the
// non-STNP fallback; the STNP path calls fused_v_from_taps directly so it can
// hold the taps in registers across both halves of a 16-lane pair.
//
// v3.0: this used to be templated on W (widening kernels) and LlFast. The
// widening variants measured worse and were deleted, and this call site never
// took the LlFast path, so both parameters are gone with them.
static inline void fused_v_neon8_reg(
        const int16_t* t0, const int16_t* t1, const int16_t* t2,
        const int16_t* t3, const int16_t* t4, const int16_t* t5,
        int col, int kind,
        const cinepi_quant_neon& q_lo, const cinepi_quant_neon& q_hi,
        int16x8_t* lo_out, int16x8_t* hi_out) {
    const int16x8_t a0 = vld1q_s16(t0 + col);
    const int16x8_t a1 = vld1q_s16(t1 + col);
    const int16x8_t a2 = vld1q_s16(t2 + col);
    const int16x8_t a3 = vld1q_s16(t3 + col);
    const int16x8_t a4 = vld1q_s16(t4 + col);
    const int16x8_t a5 = vld1q_s16(t5 + col);
    cinepi_i32x8 hi;
    if (kind == 0)      hi = cinepi_hp_top(a0, a1, a2, a3, a4, a5);
    else if (kind == 2) hi = cinepi_hp_bottom(a0, a1, a2, a3, a4, a5);
    else                hi = cinepi_hp6(a0, a1, a2, a3, a4, a5);
    const int16x8_t lp0 = (kind == 0) ? a0 : (kind == 2 ? a4 : a2);
    const int16x8_t lp1 = (kind == 0) ? a1 : (kind == 2 ? a5 : a3);
    cinepi_i32x8 lo = cinepi_lp(lp0, lp1, 0);
    lo = cinepi_quant(lo, q_lo);
    *lo_out = cinepi_sat(lo);
    hi = cinepi_quant(hi, q_hi);
    *hi_out = cinepi_sat(hi);
}


// v1.5: the same arithmetic as fused_v_neon8_reg but on taps the caller has
// already loaded, so hp_flat_skip can inspect them before deciding to compute.
// Kind < 0 keeps the shipped run-time branch; 0/1/2 fold it away (emit_kind1).
template <bool LlFast, int Kind, bool SkipHi = false>
static inline void fused_v_from_taps(
        int16x8_t a0, int16x8_t a1, int16x8_t a2,
        int16x8_t a3, int16x8_t a4, int16x8_t a5, int kind_rt,
        const cinepi_quant_neon& q_lo, const cinepi_quant_neon& q_hi,
        int16x8_t* lo_out, int16x8_t* hi_out) {
    const int k = (Kind < 0) ? kind_rt : Kind;
    cinepi_i32x8 hi;
    if (!SkipHi) {
        if (k == 0)      hi = cinepi_hp_top(a0, a1, a2, a3, a4, a5);
        else if (k == 2) hi = cinepi_hp_bottom(a0, a1, a2, a3, a4, a5);
        else             hi = cinepi_hp6(a0, a1, a2, a3, a4, a5);
    }
    const int16x8_t lp0 = (k == 0) ? a0 : (k == 2 ? a4 : a2);
    const int16x8_t lp1 = (k == 0) ? a1 : (k == 2 ? a5 : a3);
    if (LlFast) {
        *lo_out = vqaddq_s16(lp0, lp1);
    } else {
        cinepi_i32x8 lo = cinepi_lp(lp0, lp1, 0);
        lo = cinepi_quant(lo, q_lo);
        *lo_out = cinepi_sat(lo);
    }
    if (!SkipHi) {
        hi = cinepi_quant(hi, q_hi);
        *hi_out = cinepi_sat(hi);
    }
}

// v1.5b. The FIRST bound tried here was the six-tap span, and it was correct
// but useless: measured on the bundled frame it fired on 7 blocks out of
// 340,200 (0.002%). |sum w*d| <= Wmax*span is tight only for adversarial
// alternating taps, and image data is nowhere near that -- a smooth ramp has
// a large span and an exactly zero highpass.
//
// This bound is tight on the actual structure instead. Regrouping the
// interior filter into differences,
//
//     -a0 - a1 + 8*a2 - 8*a3 + a4 + a5  ==  (a4-a0) + (a5-a1) + 8*(a2-a3)
//
// gives  |numerator - 4|  <=  |a4-a0| + |a5-a1| + 8*|a2-a3|, which is small
// exactly when the rows are locally smooth -- which is when the highpass is
// actually zero.
//
// Only the interior shape is covered. Kinds 0 and 2 are the first and last
// output row of each level; skipping them would need their own regrouping
// for two rows in a thousand, so they simply fall through to the full path.
//
// EXACTNESS OF THE ARITHMETIC. vabdq_s16 truncates its result to 16 bits,
// and the absolute difference of two int16 values is in [0, 65535], so
// reinterpreting as uint16 recovers it EXACTLY. The sum then uses saturating
// unsigned adds and a saturating shift: any overflow pins at 65535, which is
// above every permitted limit (capped at 65534 below), so saturation can
// only make the test refuse a block it might have skipped -- never the
// reverse.
static inline int fused_hp1_bound16(
        int16x8_t a0l, int16x8_t a1l, int16x8_t a2l,
        int16x8_t a3l, int16x8_t a4l, int16x8_t a5l,
        int16x8_t a0h, int16x8_t a1h, int16x8_t a2h,
        int16x8_t a3h, int16x8_t a4h, int16x8_t a5h) {
    const uint16x8_t l = vqaddq_u16(
        vqaddq_u16(vreinterpretq_u16_s16(vabdq_s16(a4l, a0l)),
                   vreinterpretq_u16_s16(vabdq_s16(a5l, a1l))),
        vqshlq_n_u16(vreinterpretq_u16_s16(vabdq_s16(a2l, a3l)), 3));
    const uint16x8_t h = vqaddq_u16(
        vqaddq_u16(vreinterpretq_u16_s16(vabdq_s16(a4h, a0h)),
                   vreinterpretq_u16_s16(vabdq_s16(a5h, a1h))),
        vqshlq_n_u16(vreinterpretq_u16_s16(vabdq_s16(a2h, a3h)), 3));
    return int(vmaxvq_u16(vmaxq_u16(l, h)));
}
#endif

// The vertical emit, with the emit variant switch. Variant Shipped executes
// exactly the shipped fused_level_emit body; the byte-compare self-check and
// the driver's CRC gate both stand guard over that claim.
// E8: nonzero-mask sidecar. 1 bit per coefficient of the frame's coeff
// buffer, generated here where the quantised vectors are still in registers.
// v0.14 sidecar_zskip: entropy reads band data ONLY where the mask says
// nonzero, so storing an all-zero block writes memory nobody ever reads.
// The coeff buffers are zero-initialised once; skipped regions therefore
// hold zeros on the gate frame (self-check passes) and stale-but-never-read
// values at steady state (bitstream unaffected -- the CRC gates prove it).
static bool g_v2_sidecar_zskip = false;

struct V2Sidecar;
static void v2_level_push(std::array<FusedLevel,3>& lv, int index,
                          const int16_t* src_row, EmitKind emit,
                          const V2Sidecar* sc);

struct V2Sidecar {
    uint8_t* mask_base = nullptr;          // (coeff lanes)/8 bytes
    const int16_t* coeff_base = nullptr;
};

#if CINEPI_HAVE_NEON_WAVELET
static inline uint16_t v2_sidecar_store2(const V2Sidecar* sc, const int16_t* dst,
                                         int16x8_t v0, int16x8_t v1) {
    // Two mask bytes for 16 lanes: bit i of byte j = (lane 8j+i != 0).
    // v0.9: the earlier version used two cross-lane vaddv reductions, which
    // occupy the vector pipes the wavelet math needs on the pole thread.
    // This one narrows the compare to a 0xFF/0x00 byte mask (vshrn, same
    // trick the scanner uses) and packs bytes to bits with one 64-bit
    // multiply on the scalar pipes: (m * 0x0102040810204080) >> 56 puts
    // byte k's LSB at bit k. Identical mask bytes; the CRC gates re-prove
    // it end to end.
    const uint64_t z0 = vget_lane_u64(vreinterpret_u64_u8(
        vshrn_n_u16(vceqzq_s16(v0), 4)), 0);
    const uint64_t z1 = vget_lane_u64(vreinterpret_u64_u8(
        vshrn_n_u16(vceqzq_s16(v1), 4)), 0);
    const uint64_t kPack = 0x0102040810204080ull;
    const uint8_t b0 = uint8_t(((~z0 & 0x0101010101010101ull) * kPack) >> 56);
    const uint8_t b1 = uint8_t(((~z1 & 0x0101010101010101ull) * kPack) >> 56);
    const size_t lane_off = size_t(dst - sc->coeff_base);
    uint8_t* m = sc->mask_base + (lane_off >> 3);
    m[0] = b0;
    m[1] = b1;
    return uint16_t(b0 | (uint16_t(b1) << 8));
}
#endif

#if CINEPI_HAVE_NEON_WAVELET && CINEPI_HAVE_STNP
// v1.4: the two register-direct STNP column loops, lifted out of
// v2_level_emit so the candidate selection is a template argument resolved
// once per output row rather than a branch inside the column loop. The bodies
// are the shipped bodies verbatim apart from the templated call.
// v1.5 register-direct LL half, with emit_kind1 (Kind) and hp_flat_skip
// (Skip) as template arguments so the column loop stays branch-free.
template <bool LlFast, int Kind, bool Skip>
static inline int v2_stnp_ll_half15(const int16_t* t0, const int16_t* t1,
                                    const int16_t* t2, const int16_t* t3,
                                    const int16_t* t4, const int16_t* t5,
                                    int c, int half, int kind,
                                    const cinepi_quant_neon& q_lo,
                                    const cinepi_quant_neon& q_hl,
                                    int16_t* ll_dest, int16_t* dst_hi,
                                    const V2Sidecar* sc, bool stnp_ll,
                                    int span_limit) {
    const int16x8_t zero = vdupq_n_s16(0);
    for (; c + 16 <= half; c += 16) {
        const int16x8_t a0l = vld1q_s16(t0 + c),     a1l = vld1q_s16(t1 + c);
        const int16x8_t a2l = vld1q_s16(t2 + c),     a3l = vld1q_s16(t3 + c);
        const int16x8_t a4l = vld1q_s16(t4 + c),     a5l = vld1q_s16(t5 + c);
        const int16x8_t a0h = vld1q_s16(t0 + c + 8), a1h = vld1q_s16(t1 + c + 8);
        const int16x8_t a2h = vld1q_s16(t2 + c + 8), a3h = vld1q_s16(t3 + c + 8);
        const int16x8_t a4h = vld1q_s16(t4 + c + 8), a5h = vld1q_s16(t5 + c + 8);

        bool flat = false;
        if (Skip && ((Kind < 0) ? (kind == 1) : (Kind == 1))) {
            flat = fused_hp1_bound16(a0l,a1l,a2l,a3l,a4l,a5l,
                                     a0h,a1h,a2h,a3h,a4h,a5h) <= span_limit;
        }

        int16x8_t lo0, hi0, lo1, hi1;
        if (Skip && flat) {
            // The highpass is provably zero here, so it is not computed. The
            // lowpass still is: the next level reads it.
            const int16x8_t lp0l = (Kind == 0) ? a0l : (Kind == 2 ? a4l : a2l);
            const int16x8_t lp1l = (Kind == 0) ? a1l : (Kind == 2 ? a5l : a3l);
            const int16x8_t lp0h = (Kind == 0) ? a0h : (Kind == 2 ? a4h : a2h);
            const int16x8_t lp1h = (Kind == 0) ? a1h : (Kind == 2 ? a5h : a3h);
            const int kk = (Kind < 0) ? kind : Kind;
            if (Kind < 0) {
                const int16x8_t p0l = (kk == 0) ? a0l : (kk == 2 ? a4l : a2l);
                const int16x8_t p1l = (kk == 0) ? a1l : (kk == 2 ? a5l : a3l);
                const int16x8_t p0h = (kk == 0) ? a0h : (kk == 2 ? a4h : a2h);
                const int16x8_t p1h = (kk == 0) ? a1h : (kk == 2 ? a5h : a3h);
                if (LlFast) { lo0 = vqaddq_s16(p0l, p1l); lo1 = vqaddq_s16(p0h, p1h); }
                else {
                    lo0 = cinepi_sat(cinepi_quant(cinepi_lp(p0l, p1l, 0), q_lo));
                    lo1 = cinepi_sat(cinepi_quant(cinepi_lp(p0h, p1h, 0), q_lo));
                }
            } else if (LlFast) {
                lo0 = vqaddq_s16(lp0l, lp1l); lo1 = vqaddq_s16(lp0h, lp1h);
            } else {
                lo0 = cinepi_sat(cinepi_quant(cinepi_lp(lp0l, lp1l, 0), q_lo));
                lo1 = cinepi_sat(cinepi_quant(cinepi_lp(lp0h, lp1h, 0), q_lo));
            }
            hi0 = zero; hi1 = zero;
        } else {
            fused_v_from_taps<LlFast, Kind>(a0l,a1l,a2l,a3l,a4l,a5l, kind,
                                            q_lo, q_hl, &lo0, &hi0);
            fused_v_from_taps<LlFast, Kind>(a0h,a1h,a2h,a3h,a4h,a5h, kind,
                                            q_lo, q_hl, &lo1, &hi1);
        }

        if (stnp_ll) {
            asm volatile("stnp %q0, %q1, [%2]" :: "w"(lo0), "w"(lo1),
                         "r"(ll_dest + c) : "memory");
        } else {
            vst1q_s16(ll_dest + c,     lo0);
            vst1q_s16(ll_dest + c + 8, lo1);
        }
        if (sc) {
            const uint16_t nz = v2_sidecar_store2(sc, dst_hi + c, hi0, hi1);
            if (nz || !g_v2_sidecar_zskip)
                asm volatile("stnp %q0, %q1, [%2]" :: "w"(hi0), "w"(hi1),
                             "r"(dst_hi + c) : "memory");
        } else {
            asm volatile("stnp %q0, %q1, [%2]" :: "w"(hi0), "w"(hi1),
                         "r"(dst_hi + c) : "memory");
        }
    }
    return c;
}

template <int Kind, bool Skip>
static inline int v2_stnp_hl_half15(const int16_t* t0, const int16_t* t1,
                                    const int16_t* t2, const int16_t* t3,
                                    const int16_t* t4, const int16_t* t5,
                                    int c, int cw, int kind,
                                    const cinepi_quant_neon& q_lh,
                                    const cinepi_quant_neon& q_hh,
                                    int16_t* dst_lo, int16_t* dst_hi,
                                    const V2Sidecar* sc, int span_limit) {
    const int16x8_t zero = vdupq_n_s16(0);
    for (; c + 16 <= cw; c += 16) {
        const int16x8_t a0l = vld1q_s16(t0 + c),     a1l = vld1q_s16(t1 + c);
        const int16x8_t a2l = vld1q_s16(t2 + c),     a3l = vld1q_s16(t3 + c);
        const int16x8_t a4l = vld1q_s16(t4 + c),     a5l = vld1q_s16(t5 + c);
        const int16x8_t a0h = vld1q_s16(t0 + c + 8), a1h = vld1q_s16(t1 + c + 8);
        const int16x8_t a2h = vld1q_s16(t2 + c + 8), a3h = vld1q_s16(t3 + c + 8);
        const int16x8_t a4h = vld1q_s16(t4 + c + 8), a5h = vld1q_s16(t5 + c + 8);

        bool flat = false;
        if (Skip && ((Kind < 0) ? (kind == 1) : (Kind == 1))) {
            flat = fused_hp1_bound16(a0l,a1l,a2l,a3l,a4l,a5l,
                                     a0h,a1h,a2h,a3h,a4h,a5h) <= span_limit;
        }

        int16x8_t lo0, hi0, lo1, hi1;
        if (Skip && flat) {
            // Highpass provably zero; the lowpass here IS quantised, so it is
            // computed in full -- only the highpass is skipped.
            const int kk = (Kind < 0) ? kind : Kind;
            const int16x8_t p0l = (kk == 0) ? a0l : (kk == 2 ? a4l : a2l);
            const int16x8_t p1l = (kk == 0) ? a1l : (kk == 2 ? a5l : a3l);
            const int16x8_t p0h = (kk == 0) ? a0h : (kk == 2 ? a4h : a2h);
            const int16x8_t p1h = (kk == 0) ? a1h : (kk == 2 ? a5h : a3h);
            lo0 = cinepi_sat(cinepi_quant(cinepi_lp(p0l, p1l, 0), q_lh));
            lo1 = cinepi_sat(cinepi_quant(cinepi_lp(p0h, p1h, 0), q_lh));
            hi0 = zero; hi1 = zero;
        } else {
            fused_v_from_taps<false, Kind>(a0l,a1l,a2l,a3l,a4l,a5l, kind,
                                           q_lh, q_hh, &lo0, &hi0);
            fused_v_from_taps<false, Kind>(a0h,a1h,a2h,a3h,a4h,a5h, kind,
                                           q_lh, q_hh, &lo1, &hi1);
        }

        if (sc) {
            const uint16_t nzl = v2_sidecar_store2(sc, dst_lo + c, lo0, lo1);
            const uint16_t nzh = v2_sidecar_store2(sc, dst_hi + c, hi0, hi1);
            if (nzl || !g_v2_sidecar_zskip)
                asm volatile("stnp %q0, %q1, [%2]" :: "w"(lo0), "w"(lo1),
                             "r"(dst_lo + c) : "memory");
            if (nzh || !g_v2_sidecar_zskip)
                asm volatile("stnp %q0, %q1, [%2]" :: "w"(hi0), "w"(hi1),
                             "r"(dst_hi + c) : "memory");
        } else {
            asm volatile("stnp %q0, %q1, [%2]" :: "w"(lo0), "w"(lo1),
                         "r"(dst_lo + c) : "memory");
            asm volatile("stnp %q0, %q1, [%2]" :: "w"(hi0), "w"(hi1),
                         "r"(dst_hi + c) : "memory");
        }
    }
    return c;
}

// Resolve Kind and Skip once per output row.
template <bool LlFast, int Kind>
static inline int ll15_skip(bool skip, const int16_t* t0, const int16_t* t1,
        const int16_t* t2, const int16_t* t3, const int16_t* t4,
        const int16_t* t5, int c, int half, int kind,
        const cinepi_quant_neon& q_lo, const cinepi_quant_neon& q_hl,
        int16_t* ll_dest, int16_t* dst_hi, const V2Sidecar* sc,
        bool stnp_ll, int span_limit) {
    return skip
        ? v2_stnp_ll_half15<LlFast, Kind, true>(t0,t1,t2,t3,t4,t5, c, half,
              kind, q_lo, q_hl, ll_dest, dst_hi, sc, stnp_ll, span_limit)
        : v2_stnp_ll_half15<LlFast, Kind, false>(t0,t1,t2,t3,t4,t5, c, half,
              kind, q_lo, q_hl, ll_dest, dst_hi, sc, stnp_ll, span_limit);
}

template <bool LlFast>
static inline int ll15_kind(bool kind_t, int kind, bool skip,
        const int16_t* t0, const int16_t* t1, const int16_t* t2,
        const int16_t* t3, const int16_t* t4, const int16_t* t5,
        int c, int half, const cinepi_quant_neon& q_lo,
        const cinepi_quant_neon& q_hl, int16_t* ll_dest, int16_t* dst_hi,
        const V2Sidecar* sc, bool stnp_ll, int span_limit) {
    if (!kind_t)
        return ll15_skip<LlFast, -1>(skip, t0,t1,t2,t3,t4,t5, c, half, kind,
                   q_lo, q_hl, ll_dest, dst_hi, sc, stnp_ll, span_limit);
    if (kind == 0)
        return ll15_skip<LlFast, 0>(skip, t0,t1,t2,t3,t4,t5, c, half, kind,
                   q_lo, q_hl, ll_dest, dst_hi, sc, stnp_ll, span_limit);
    if (kind == 2)
        return ll15_skip<LlFast, 2>(skip, t0,t1,t2,t3,t4,t5, c, half, kind,
                   q_lo, q_hl, ll_dest, dst_hi, sc, stnp_ll, span_limit);
    return ll15_skip<LlFast, 1>(skip, t0,t1,t2,t3,t4,t5, c, half, kind,
                   q_lo, q_hl, ll_dest, dst_hi, sc, stnp_ll, span_limit);
}

template <int Kind>
static inline int hl15_skip(bool skip, const int16_t* t0, const int16_t* t1,
        const int16_t* t2, const int16_t* t3, const int16_t* t4,
        const int16_t* t5, int c, int cw, int kind,
        const cinepi_quant_neon& q_lh, const cinepi_quant_neon& q_hh,
        int16_t* dst_lo, int16_t* dst_hi, const V2Sidecar* sc,
        int span_limit) {
    return skip
        ? v2_stnp_hl_half15<Kind, true>(t0,t1,t2,t3,t4,t5, c, cw, kind,
              q_lh, q_hh, dst_lo, dst_hi, sc, span_limit)
        : v2_stnp_hl_half15<Kind, false>(t0,t1,t2,t3,t4,t5, c, cw, kind,
              q_lh, q_hh, dst_lo, dst_hi, sc, span_limit);
}

static inline int hl15_kind(bool kind_t, int kind, bool skip,
        const int16_t* t0, const int16_t* t1, const int16_t* t2,
        const int16_t* t3, const int16_t* t4, const int16_t* t5,
        int c, int cw, const cinepi_quant_neon& q_lh,
        const cinepi_quant_neon& q_hh, int16_t* dst_lo, int16_t* dst_hi,
        const V2Sidecar* sc, int span_limit) {
    if (!kind_t)
        return hl15_skip<-1>(skip, t0,t1,t2,t3,t4,t5, c, cw, kind, q_lh, q_hh,
                             dst_lo, dst_hi, sc, span_limit);
    if (kind == 0)
        return hl15_skip<0>(skip, t0,t1,t2,t3,t4,t5, c, cw, kind, q_lh, q_hh,
                            dst_lo, dst_hi, sc, span_limit);
    if (kind == 2)
        return hl15_skip<2>(skip, t0,t1,t2,t3,t4,t5, c, cw, kind, q_lh, q_hh,
                            dst_lo, dst_hi, sc, span_limit);
    return hl15_skip<1>(skip, t0,t1,t2,t3,t4,t5, c, cw, kind, q_lh, q_hh,
                        dst_lo, dst_hi, sc, span_limit);
}

// v3.0: the two W-templated legacy emit loops that lived here are gone.
// They existed only to carry wl_widen, which measured WORSE than the
// kernels it replaced (35 vs 33 instructions per 8 lanes) and was
// deleted; nothing had called them since. Their removal orphaned the
// cinepi_*_w widening kernels, which go with them.
#endif

/* wav1: register-direct emit straight into the CpuDirectHybridSlot tile
   storage. Defined after the class; declared here so v2_level_emit can take
   the fast path. Returns false when the geometry is unsupported, in which
   case the caller falls through to the byte-identical staging+callback path. */
static bool v2_direct_emit_row(void* slot_v, FusedLevel& L, int i, int kind,
                               const int16_t* t0, const int16_t* t1,
                               const int16_t* t2, const int16_t* t3,
                               const int16_t* t4, const int16_t* t5);

static void v2_level_emit(std::array<FusedLevel,3>& lv, int index, EmitKind emit,
                          const V2Sidecar* sc) {
    FusedLevel& L = lv[size_t(index)];
    const int i = L.next_out;
    int kind, r0;
    if (i == 0)                    { kind = 0; r0 = 0; }
    else if (2 * i == L.last)      { kind = 2; r0 = L.last - 4; }
    else                           { kind = 1; r0 = 2 * i - 2; }

    const int16_t* const base = L.ring.data();
    const int rr = L.ring_rows;
    auto row = [&](int r) {
        return base + size_t(cinepi_ring_slot(r, rr)) * size_t(L.cw);
    };
    const int16_t* const t0 = row(r0 + 0);
    const int16_t* const t1 = row(r0 + 1);
    const int16_t* const t2 = row(r0 + 2);
    const int16_t* const t3 = row(r0 + 3);
    const int16_t* const t4 = row(r0 + 4);
    const int16_t* const t5 = row(r0 + 5);
    const int16_t* const l0 = (kind == 0) ? t0 : (kind == 2 ? t4 : t2);
    const int16_t* const l1 = (kind == 0) ? t1 : (kind == 2 ? t5 : t3);

    const bool compact = L.compact_output && g_v2_compact_sink &&
                         g_v2_compact_sink->highpass_row;
#if CINEPI_HAVE_NEON_WAVELET
    /* wav1: production compact path. Same filter arithmetic (the identity-
       quantised LL collapses to vqaddq_s16 exactly as ll_vqadd proved), same
       tile narrowing semantics, no staging round trip. */
    if (compact && g_cpu_wavelet_use_neon && g_v2_compact_sink->direct_slot &&
        v2_direct_emit_row(g_v2_compact_sink->direct_slot, L, i, kind,
                           t0, t1, t2, t3, t4, t5)) {
        ++L.next_out;
        if (!L.final_level)
            v2_level_push(lv, index + 1, L.ll_row.data(), emit, sc);
        return;
    }
#endif
    if (compact) {
        if (L.compact_lo_row.size() != size_t(L.cw)) L.compact_lo_row.resize(size_t(L.cw));
        if (L.compact_hi_row.size() != size_t(L.cw)) L.compact_hi_row.resize(size_t(L.cw));
        if (L.final_level && L.compact_ll_row.size() != size_t(L.half))
            L.compact_ll_row.resize(size_t(L.half));
    }
    int16_t* const dst_lo = compact ? L.compact_lo_row.data()
                                    : L.dst + size_t(i) * L.dst_stride;
    int16_t* const dst_hi = compact ? L.compact_hi_row.data()
                                    : L.dst + size_t(L.outs + i) * L.dst_stride;
    int16_t* const ll_dest = L.final_level
        ? (compact ? L.compact_ll_row.data() : dst_lo)
        : L.ll_row.data();
    const bool nt_bands = g_cpu_nontemporal_bands && !compact;

#if CINEPI_HAVE_NEON_WAVELET
    if (g_cpu_wavelet_use_neon) {
        const cinepi_quant_neon q_none = cinepi_make_quant(1);
        const cinepi_quant_neon q_lh = cinepi_make_quant(L.q_lh, L.pc_t_lh);
        const cinepi_quant_neon q_hl = cinepi_make_quant(L.q_hl, L.pc_t_hl);
        const cinepi_quant_neon q_hh = cinepi_make_quant(L.q_hh, L.pc_t_hh);
        int c = 0;
#if CINEPI_HAVE_STNP
        if (nt_bands && emit != EmitKind::Shipped) {
            // Register-direct STNP. Identical lane arithmetic; the 32-byte
            // pair is built in registers instead of a 16-entry stack array.
            const bool stnp_ll = (emit == EmitKind::RegDirectL3LL) && L.final_level;
            // ll_vqadd is only ever taken when the quantiser really is the
            // identity. q_none is cinepi_make_quant(1), so !enabled is a
            // tautology here -- it is asserted rather than assumed so the
            // fast path cannot outlive the precondition if this call site
            // ever changes.
            // q_none is cinepi_make_quant(1), so !enabled is a tautology
            // here -- asserted rather than assumed so the fast path cannot
            // outlive its precondition if this call site ever changes.
            const bool llf = !q_none.enabled;
            // emit_kind1 always: the filter shape is loop-invariant per
            // output row, so it is a template argument and the two unused
            // highpass bodies are discarded.
            c = llf ? ll15_kind<true>(true, kind, false,
                          t0,t1,t2,t3,t4,t5, c, L.half, q_none, q_hl,
                          ll_dest, dst_hi, sc, stnp_ll, 0)
                    : ll15_kind<false>(true, kind, false,
                          t0,t1,t2,t3,t4,t5, c, L.half, q_none, q_hl,
                          ll_dest, dst_hi, sc, stnp_ll, 0);
        } else if (nt_bands) {
            // Shipped 16-wide staging path, verbatim.
            int16_t hi_pair[16];
            for (; c + 16 <= L.half; c += 16) {
                cinepi_fused_v_neon8(t0, t1, t2, t3, t4, t5, l0, l1, c, 0, kind,
                                     q_none, q_hl, ll_dest + c, hi_pair);
                cinepi_fused_v_neon8(t0, t1, t2, t3, t4, t5, l0, l1, c + 8, 8, kind,
                                     q_none, q_hl, ll_dest + c, hi_pair);
                cinepi_stnp_q2(dst_hi + c, vld1q_s16(hi_pair), vld1q_s16(hi_pair + 8));
            }
        }
#endif
        for (; c + 8 <= L.half; c += 8)
            cinepi_fused_v_neon8(t0, t1, t2, t3, t4, t5, l0, l1, c, c, kind,
                                 q_none, q_hl, ll_dest, dst_hi);
        for (; c < L.half; ++c)
            cinepi_fused_v_scalar(t0, t1, t2, t3, t4, t5, l0, l1, c, kind, false,
                                  L.q_lh, L.q_hl, L.q_hh, ll_dest, dst_hi,
                                  L.pc_t_lh, L.pc_t_hl, L.pc_t_hh);
#if CINEPI_HAVE_STNP
        if (nt_bands && emit != EmitKind::Shipped) {
            c = hl15_kind(true, kind, false,
                          t0,t1,t2,t3,t4,t5, c, L.cw, q_lh, q_hh,
                          dst_lo, dst_hi, sc, 0);
        } else if (nt_bands) {
            int16_t lo_pair[16], hi_pair[16];
            for (; c + 16 <= L.cw; c += 16) {
                cinepi_fused_v_neon8(t0, t1, t2, t3, t4, t5, l0, l1, c, 0, kind,
                                     q_lh, q_hh, lo_pair, hi_pair);
                cinepi_fused_v_neon8(t0, t1, t2, t3, t4, t5, l0, l1, c + 8, 8, kind,
                                     q_lh, q_hh, lo_pair, hi_pair);
                cinepi_stnp_q2(dst_lo + c, vld1q_s16(lo_pair), vld1q_s16(lo_pair + 8));
                cinepi_stnp_q2(dst_hi + c, vld1q_s16(hi_pair), vld1q_s16(hi_pair + 8));
            }
        }
#endif
        for (; c + 8 <= L.cw; c += 8)
            cinepi_fused_v_neon8(t0, t1, t2, t3, t4, t5, l0, l1, c, c, kind,
                                 q_lh, q_hh, dst_lo, dst_hi);
        for (; c < L.cw; ++c)
            cinepi_fused_v_scalar(t0, t1, t2, t3, t4, t5, l0, l1, c, kind, true,
                                  L.q_lh, L.q_hl, L.q_hh, dst_lo, dst_hi,
                                  L.pc_t_lh, L.pc_t_hl, L.pc_t_hh);
    } else
#endif
    {
        (void)emit;
        for (int c = 0; c < L.half; ++c)
            cinepi_fused_v_scalar(t0, t1, t2, t3, t4, t5, l0, l1, c, kind, false,
                                  L.q_lh, L.q_hl, L.q_hh, ll_dest, dst_hi,
                                  L.pc_t_lh, L.pc_t_hl, L.pc_t_hh);
        for (int c = L.half; c < L.cw; ++c)
            cinepi_fused_v_scalar(t0, t1, t2, t3, t4, t5, l0, l1, c, kind, true,
                                  L.q_lh, L.q_hl, L.q_hh, dst_lo, dst_hi,
                                  L.pc_t_lh, L.pc_t_hl, L.pc_t_hh);
    }

    if (compact) {
        // The three high-pass quadrants are complete for this output row.
        // Hand them straight to the tile writer; only row-sized int16 staging
        // exists here, never a full int16 coefficient frame.
        g_v2_compact_sink->highpass_row(g_v2_compact_sink->ctx, L.compact_plane,
                                        L.compact_level, 1, i, dst_lo + L.half, L.half);
        g_v2_compact_sink->highpass_row(g_v2_compact_sink->ctx, L.compact_plane,
                                        L.compact_level, 2, i, dst_hi, L.half);
        g_v2_compact_sink->highpass_row(g_v2_compact_sink->ctx, L.compact_plane,
                                        L.compact_level, 3, i, dst_hi + L.half, L.half);
        if (L.final_level && g_v2_compact_sink->final_ll_row)
            g_v2_compact_sink->final_ll_row(g_v2_compact_sink->ctx, L.compact_plane,
                                            i, ll_dest, L.half);
    }
    ++L.next_out;
    if (!L.final_level)
        v2_level_push(lv, index + 1, ll_dest, emit, sc);
}

static void v2_level_push(std::array<FusedLevel,3>& lv, int index,
                          const int16_t* src_row, EmitKind emit,
                          const V2Sidecar* sc) {
    FusedLevel& L = lv[size_t(index)];
    CpuWaveletPush h{};
    h.axis_len = uint32_t(L.cw);
    h.count = 1u;
    h.stride = uint32_t(L.cw);
    h.out_stride = uint32_t(L.cw);
    h.dir = 0u;
    h.prescale = L.prescale;
    cpu_wavelet_pass(src_row,
                     L.ring.data() + size_t(cinepi_ring_slot(L.next_src_row, L.ring_rows))
                                   * size_t(L.cw),
                     h);
    ++L.next_src_row;
    for (;;) {
        if (L.next_out >= L.outs) break;
        const int i = L.next_out;
        const int need_max = (i == 0) ? 5
                           : (2 * i == L.last) ? (L.ch - 1)
                           : (2 * i + 3);
        if (L.next_src_row <= need_max) break;
        v2_level_emit(lv, index, emit, sc);
    }
}

// The split row, with the split-variant switch. Variant Shipped defers to the
// shipped fused_split_row.
// v0.9 input_prefetch: streaming (transient) prefetch of the raw sensor
// rows. The 16.6 MB/frame of pixel reads flow through L2 and keep evicting
// the 128 KB compand LUT that this very loop gathers from; PLDL1STRM marks
// the incoming lines transient so they are evicted first, protecting the
// LUT's residency. Costs one prefetch instruction per cache line of input.
static bool g_v2_input_prefetch = false;


// v0.21 precompand_sim: the input frame already carries GP-Log2 values, so
// the split does plain loads instead of 8.3M gathers from the 128 KB LUT.
// This is the exact work an external (GPU/ISP) compander would leave the
// CPU -- its measured gain is the UPPER BOUND on what GPU companding could
// ever save the encode cores, before paying the +33 MB/frame DRAM round
// trip and the dispatch/fence stage a real GPU pass would add.
/* Lab-only kernel variants. They exist for the m5 candidate suite and are
   compiled OUT of the production binary (-DCINEPI_M5_LAB is set only by
   the optimisation suite, since removed), so the shipped hot loop carries no branch for a
   candidate that lost its greedy trial. */
#ifdef CINEPI_M5_LAB
/* Lab-only kernel variants. They exist for the m5 candidate suite and are
   compiled OUT of the production binary (-DCINEPI_M5_LAB is set only by
   the optimisation suite, since removed), so the shipped hot loop carries no branch for a
   candidate that lost its greedy trial. */
#ifdef CINEPI_M5_LAB
static bool g_v2_precompanded_input = false;
static bool g_v2_pair_gather = false;   // v0.26: u32 paired raw loads
#endif
#endif

static void v2_split_row(const uint16_t* row0, const uint16_t* row1,
                         const uint16_t* lut, int pw, bool gbrg,
                         int16_t* gs, int16_t* rg, int16_t* bg, int16_t* gd,
                         int sh, int mid, SplitKind split,
                         unsigned src_shift = 0,
                         bool src_byteswap = false) {
#ifdef CINEPI_M5_LAB
    if (g_v2_precompanded_input && gbrg) {
        // Values are final: de-interleave and colour-difference only, the
        // identical arithmetic to the gather path below minus the 8.3M LUT
        // gathers (companded[i] == lut[raw[i]] by construction; the CRC
        // gates prove bit-equality end to end).
        int x = 0;
#if CINEPI_HAVE_NEON_WAVELET
        const int16x8_t vmid = vdupq_n_s16(int16_t(mid));
        const int16x8_t vsh = vdupq_n_s16(int16_t(-sh));
            // (x + midpoint) >> 1 is exactly vhaddq_s16. sh is still
            // tested: at sh == 0 the shift is a no-op and vhadd would halve.
            const bool hadd = (sh == 1);
        for (; x + 8 <= pw; x += 8) {
            const uint16x8x2_t t0 = vld2q_u16(row0 + 2 * x);
            const uint16x8x2_t t1 = vld2q_u16(row1 + 2 * x);
            const int16x8_t vg1 = vreinterpretq_s16_u16(t0.val[0]);
            const int16x8_t vb  = vreinterpretq_s16_u16(t0.val[1]);
            const int16x8_t vr  = vreinterpretq_s16_u16(t1.val[0]);
            const int16x8_t vg2 = vreinterpretq_s16_u16(t1.val[1]);
            const int16x8_t vgreen = vhaddq_s16(vg1, vg2);
            vst1q_s16(gs + x, vgreen);
            vst1q_s16(rg + x, (hadd ? vhaddq_s16(vsubq_s16(vr, vgreen), vmid) : vshlq_s16(vaddq_s16(vsubq_s16(vr, vgreen), vmid), vsh)));
            vst1q_s16(bg + x, (hadd ? vhaddq_s16(vsubq_s16(vb, vgreen), vmid) : vshlq_s16(vaddq_s16(vsubq_s16(vb, vgreen), vmid), vsh)));
            vst1q_s16(gd + x, (hadd ? vhaddq_s16(vsubq_s16(vg1, vg2), vmid) : vshlq_s16(vaddq_s16(vsubq_s16(vg1, vg2), vmid), vsh)));
        }
#endif
        for (; x < pw; ++x) {
            const int g1 = row0[2 * x + 0], b = row0[2 * x + 1];
            const int r  = row1[2 * x + 0], g2 = row1[2 * x + 1];
            const int green = (g1 + g2) >> 1;
            gs[x] = int16_t(green);
            rg[x] = int16_t((r - green + mid) >> sh);
            bg[x] = int16_t((b - green + mid) >> sh);
            gd[x] = int16_t((g1 - g2 + mid) >> sh);
        }
        return;
    }
#endif /* CINEPI_M5_LAB */
#if defined(__aarch64__)
    if (g_v2_input_prefetch) {
        // Both source rows, 4 lines (256 B = 128 pixels) ahead of use.
        const int bytes = pw * 2 * 2;   // samples per source row in bytes
        for (int off = 0; off < bytes; off += 64) {
            asm volatile("prfm pldl1strm, [%0, #256]" ::
                         "r"(reinterpret_cast<const uint8_t*>(row0) + off));
            asm volatile("prfm pldl1strm, [%0, #256]" ::
                         "r"(reinterpret_cast<const uint8_t*>(row1) + off));
        }
    }
#endif
    /* v3.14: a byteswapped source always takes the templated shipped kernel.
       The Unroll16/Pipelined scheduling variants below are lab alternates
       that use a runtime shift; threading a per-sample byteswap branch
       through them would cost the exact latency the fold is meant to hide,
       and no production caller selects them. */
    if (split == SplitKind::Shipped || !gbrg || src_byteswap || g_cpu_hot_lut) {
        fused_split_row(row0, row1, lut, pw, gbrg, gs, rg, bg, gd, sh, mid,
                        src_shift, src_byteswap);
        return;
    }
    int x = 0;
#if CINEPI_HAVE_NEON_WAVELET
    if (g_cpu_split_neon) {
        const int16x8_t vmid = vdupq_n_s16(int16_t(mid));
        const int16x8_t vsh = vdupq_n_s16(int16_t(-sh));
            // (x + midpoint) >> 1 is exactly vhaddq_s16. sh is still
            // tested: at sh == 0 the shift is a no-op and vhadd would halve.
            const bool hadd = (sh == 1);
        if (split == SplitKind::Unroll16) {
            // Two 8-quad blocks per iteration; all 64 LUT gathers grouped so
            // the loads pipeline before the vector arithmetic begins.
            int16_t g1a[16], ba[16], ra[16], g2a[16];
#ifdef CINEPI_M5_LAB
            if (g_v2_pair_gather) {
                // v0.26 pair_gather: g1,b sit in adjacent uint16s, as do
                // r,g2 -- one u32 load fetches the pair, halving raw-load
                // count (8.3M -> 4.15M) and freeing load-port slots for the
                // LUT gathers. Same gathers, same arithmetic, same bytes.
                for (; x + 16 <= pw; x += 16) {
                    for (int k = 0; k < 16; ++k) {
                        uint32_t p0, p1;
                        std::memcpy(&p0, row0, 4); row0 += 2;
                        std::memcpy(&p1, row1, 4); row1 += 2;
                        g1a[k] = int16_t(lut[(p0 & 0xFFFFu) >> src_shift]);
                        ba[k]  = int16_t(lut[(p0 >> 16) >> src_shift]);
                        ra[k]  = int16_t(lut[(p1 & 0xFFFFu) >> src_shift]);
                        g2a[k] = int16_t(lut[(p1 >> 16) >> src_shift]);
                    }
                    for (int h = 0; h < 16; h += 8) {
                        const int16x8_t vg1 = vld1q_s16(g1a + h), vb = vld1q_s16(ba + h);
                        const int16x8_t vr  = vld1q_s16(ra + h),  vg2 = vld1q_s16(g2a + h);
                        const int16x8_t vgreen = vhaddq_s16(vg1, vg2);
                        vst1q_s16(gs + h, vgreen);
                        vst1q_s16(rg + h, (hadd ? vhaddq_s16(vsubq_s16(vr, vgreen), vmid) : vshlq_s16(vaddq_s16(vsubq_s16(vr, vgreen), vmid), vsh)));
                        vst1q_s16(bg + h, (hadd ? vhaddq_s16(vsubq_s16(vb, vgreen), vmid) : vshlq_s16(vaddq_s16(vsubq_s16(vb, vgreen), vmid), vsh)));
                        vst1q_s16(gd + h, (hadd ? vhaddq_s16(vsubq_s16(vg1, vg2), vmid) : vshlq_s16(vaddq_s16(vsubq_s16(vg1, vg2), vmid), vsh)));
                    }
                    gs += 16; rg += 16; bg += 16; gd += 16;
                }
                goto pair_tail;
            }
#endif /* CINEPI_M5_LAB */
            for (; x + 16 <= pw; x += 16) {
                for (int k = 0; k < 16; ++k) {
                    g1a[k] = int16_t(lut[row0[0] >> src_shift]); ba[k] = int16_t(lut[row0[1] >> src_shift]); row0 += 2;
                    ra[k]  = int16_t(lut[row1[0] >> src_shift]); g2a[k] = int16_t(lut[row1[1] >> src_shift]); row1 += 2;
                }
                for (int h = 0; h < 16; h += 8) {
                    const int16x8_t vg1 = vld1q_s16(g1a + h), vb = vld1q_s16(ba + h);
                    const int16x8_t vr  = vld1q_s16(ra + h),  vg2 = vld1q_s16(g2a + h);
                    const int16x8_t vgreen = vhaddq_s16(vg1, vg2);
                    vst1q_s16(gs + h, vgreen);
                    vst1q_s16(rg + h, (hadd ? vhaddq_s16(vsubq_s16(vr, vgreen), vmid) : vshlq_s16(vaddq_s16(vsubq_s16(vr, vgreen), vmid), vsh)));
                    vst1q_s16(bg + h, (hadd ? vhaddq_s16(vsubq_s16(vb, vgreen), vmid) : vshlq_s16(vaddq_s16(vsubq_s16(vb, vgreen), vmid), vsh)));
                    vst1q_s16(gd + h, (hadd ? vhaddq_s16(vsubq_s16(vg1, vg2), vmid) : vshlq_s16(vaddq_s16(vsubq_s16(vg1, vg2), vmid), vsh)));
                }
                gs += 16; rg += 16; bg += 16; gd += 16;
            }
#ifdef CINEPI_M5_LAB
            pair_tail: ;
#endif
        } else { // Pipelined: gather block k+1 while block k computes
            int16_t bufA[4][8], bufB[4][8];
            auto gather = [&](int16_t buf[4][8]) {
                for (int k = 0; k < 8; ++k) {
                    buf[0][k] = int16_t(lut[row0[0] >> src_shift]); buf[1][k] = int16_t(lut[row0[1] >> src_shift]); row0 += 2;
                    buf[2][k] = int16_t(lut[row1[0] >> src_shift]); buf[3][k] = int16_t(lut[row1[1] >> src_shift]); row1 += 2;
                }
            };
            auto compute = [&](const int16_t buf[4][8]) {
                const int16x8_t vg1 = vld1q_s16(buf[0]), vb = vld1q_s16(buf[1]);
                const int16x8_t vr  = vld1q_s16(buf[2]), vg2 = vld1q_s16(buf[3]);
                const int16x8_t vgreen = vhaddq_s16(vg1, vg2);
                vst1q_s16(gs, vgreen);
                vst1q_s16(rg, (hadd ? vhaddq_s16(vsubq_s16(vr, vgreen), vmid) : vshlq_s16(vaddq_s16(vsubq_s16(vr, vgreen), vmid), vsh)));
                vst1q_s16(bg, (hadd ? vhaddq_s16(vsubq_s16(vb, vgreen), vmid) : vshlq_s16(vaddq_s16(vsubq_s16(vb, vgreen), vmid), vsh)));
                vst1q_s16(gd, (hadd ? vhaddq_s16(vsubq_s16(vg1, vg2), vmid) : vshlq_s16(vaddq_s16(vsubq_s16(vg1, vg2), vmid), vsh)));
                gs += 8; rg += 8; bg += 8; gd += 8;
            };
            if (x + 16 <= pw) {
                int16_t (*cur)[8] = bufA, (*nxt)[8] = bufB;
                gather(cur); x += 8;
                for (; x + 8 <= pw; x += 8) {
                    gather(nxt);
                    compute(cur);
                    std::swap(cur, nxt);
                }
                compute(cur);
            }
        }
    }
#endif
    for (; x < pw; ++x) {
        const int g1 = lut[(*row0++) >> src_shift];
        const int b  = lut[(*row0++) >> src_shift];
        const int r  = lut[(*row1++) >> src_shift];
        const int g2 = lut[(*row1++) >> src_shift];
        const int green = (g1 + g2) >> 1;
        *gs++ = int16_t(green);
        *rg++ = int16_t((r - green + mid) >> sh);
        *bg++ = int16_t((b - green + mid) >> sh);
        *gd++ = int16_t((g1 - g2 + mid) >> sh);
    }
}

// The v2 whole-frame driver: shipped fused_split_transform_frame generalised
// with a source stride + crop window and the two variant switches.
struct V2Frame {
    std::array<std::array<FusedLevel,3>, 4> planes{};
    std::vector<int16_t> split_row;
};

/* ---------------------------------------------------------------------------
 * Worker placement, shared by --execution cpu-gpr and --execution capture.
 *
 * This lived only in the cpu-gpr worker, which is why "how long" (capture)
 * ran consistently slower than "how fast" (cpu-gpr) on identical settings:
 * one pinned its workers to dedicated cores and the other let them float, so
 * the capture run paid migration costs the encode run did not. Same encoder,
 * same frame, different scheduling -- and nothing in either output said so.
 *
 * Layout: the workers take the TOP `--cpu-gpr-threads` cores and CORE 0 is
 * left to the OS, its timers, its IRQs, the capture stack and the writer.
 * With 3 workers on a 4-core Pi that is cores 1, 2 and 3.
 * ------------------------------------------------------------------------ */
/* Auxiliary threads -- the capture producer, the monitor, the reserver, the
 * writer -- onto the core the WORKERS are not using.
 *
 * This is why "how long" trailed "how fast" by a few fps even after the
 * workers were pinned. The capture producer runs on the main thread and
 * memcpys a whole frame into the ring every period: at 4K/24 that is ~400
 * MB/s of copy. Unpinned, the scheduler puts it on a worker core, where it
 * steals cycles from an encode thread. "How fast" has no producer at all --
 * its workers read a shared source buffer directly -- so it never paid that
 * cost, and the difference looked like an encoder difference when it was a
 * placement one.
 *
 * The camera has the same shape and the same fix: workers on the top cores,
 * everything else on core 0 with the OS.
 */
static void cpu_gpr_place_aux(const Options& o)
{
#if defined(__linux__)
    if (!o.cpu_gpr_affinity) return;
    const long ncpu = std::max(1L, sysconf(_SC_NPROCESSORS_ONLN));
    const long budget = std::min<long>(std::max(1, o.cpu_gpr_threads), ncpu);
    cpu_set_t set; CPU_ZERO(&set);
    /* With four workers on a four-core Pi there is no fully spare core, but
       core 0 is intentionally the shared OS core. Pin auxiliary work there
       rather than returning unpinned and letting capture/telemetry migrate
       onto the three full-rate encoder cores. */
    if (o.cpu_gpr_affinity_high && ncpu == 4 && budget == 4) {
        CPU_SET(0, &set);
    } else {
        if (budget >= ncpu) return;
        if (o.cpu_gpr_affinity_high) {
            for (long c = 0; c < ncpu - budget; ++c) CPU_SET(size_t(c), &set);
        } else {
            for (long c = budget; c < ncpu; ++c) CPU_SET(size_t(c), &set);
        }
    }
    if (pthread_setaffinity_np(pthread_self(), sizeof(set), &set) != 0) {
        static std::once_flag warned;
        std::call_once(warned, []{
            std::cout << "CPU_GPR_AUX_AFFINITY status=denied\n";
        });
    }
#else
    (void)o;
#endif
}

struct Core0WaveletPlan {
    std::string name = "off";
    bool shared = false;                 // one helper raced by all primaries
    std::array<uint8_t,3> masks{{0,0,0}}; // per-primary component mask, bit 0=GS .. bit 3=GD
};

static Core0WaveletPlan cpu_gpr_core0_wavelet_plan(const Options& o)
{
    Core0WaveletPlan p;
    p.name = o.core0_wavelet_strategy;
    if      (p.name == "gs-op")             { p.shared = true; p.masks = {{1,1,1}}; }
    else if (p.name == "rg-op")             { p.shared = true; p.masks = {{2,2,2}}; }
    else if (p.name == "bg-op")             { p.shared = true; p.masks = {{4,4,4}}; }
    else if (p.name == "gd-opportunistic")  { p.shared = true; p.masks = {{8,8,8}}; }
    else if (p.name == "gs-rg-op")          { p.shared = true; p.masks = {{3,3,3}}; }
    else if (p.name == "gs-bg-op")          { p.shared = true; p.masks = {{5,5,5}}; }
    else if (p.name == "gs-gd-op")          { p.shared = true; p.masks = {{9,9,9}}; }
    else if (p.name == "rg-bg-op")          { p.shared = true; p.masks = {{6,6,6}}; }
    else if (p.name == "rg-gd-op")          { p.shared = true; p.masks = {{10,10,10}}; }
    else if (p.name == "bg-gd-op")          { p.shared = true; p.masks = {{12,12,12}}; }
    else if (p.name == "all-op")            { p.shared = true; p.masks = {{15,15,15}}; }
    else if (p.name == "gs1")               p.masks = {{1,0,0}};
    else if (p.name == "gs2")               p.masks = {{1,1,0}};
    else if (p.name == "gs3")               p.masks = {{1,1,1}};
    else if (p.name == "rg1")               p.masks = {{2,0,0}};
    else if (p.name == "rg2")               p.masks = {{2,2,0}};
    else if (p.name == "rg3")               p.masks = {{2,2,2}};
    else if (p.name == "bg1")               p.masks = {{4,0,0}};
    else if (p.name == "bg2")               p.masks = {{4,4,0}};
    else if (p.name == "bg3")               p.masks = {{4,4,4}};
    else if (p.name == "gd1")               p.masks = {{8,0,0}};
    else if (p.name == "gd2")               p.masks = {{8,8,0}};
    else if (p.name == "gd3")               p.masks = {{8,8,8}};
    else if (p.name == "gs-rg1")            p.masks = {{3,0,0}};
    else if (p.name == "gs-bg1")            p.masks = {{5,0,0}};
    else if (p.name == "gs-gd1")            p.masks = {{9,0,0}};
    else if (p.name == "rg-bg1")            p.masks = {{6,0,0}};
    else if (p.name == "rg-gd1")            p.masks = {{10,0,0}};
    else if (p.name == "bg-gd1")            p.masks = {{12,0,0}};
    else if (p.name == "gs-rg-bg1")         p.masks = {{7,0,0}};
    else if (p.name == "gs-rg-gd1")         p.masks = {{11,0,0}};
    else if (p.name == "gs-bg-gd1")         p.masks = {{13,0,0}};
    else if (p.name == "rg-bg-gd1")         p.masks = {{14,0,0}};
    else if (p.name == "all1")              p.masks = {{15,0,0}};
    else                                      p = Core0WaveletPlan{};
    return p;
}

static std::string cpu_gpr_plane_mask_name(uint8_t mask)
{
    std::string out;
    const char* names[4] = {"GS","RG","BG","GD"};
    for (int p = 0; p < 4; ++p) if (mask & (1u << p)) {
        if (!out.empty()) out += "+";
        out += names[p];
    }
    return out.empty() ? "none" : out;
}

static bool cpu_gpr_wavelet_assist_requested(const Options& o)
{
#if defined(__linux__)
    const long ncpu = std::max(1L, sysconf(_SC_NPROCESSORS_ONLN));
    return o.cpu_gpr_threads == 4 && ncpu == 4 && o.cpu_gpr_os_worker_ms > 0 &&
           o.core0_wavelet_strategy != "off";
#else
    (void)o;
    return false;
#endif
}

static int cpu_gpr_primary_worker_count(const Options& o, bool wavelet_assist_enabled)
{
    /* v1.16.6 production worker policy. UI selection 4 is deliberately not a
       fourth full-frame owner: the measured winner is three owners on cores
       1/2/3 plus the low-bandwidth Core-0 SB8 entropy assistant. Keep the old
       wavelet-assist diagnostic capable of the same three-owner layout, but
       it is no longer the default four-selected policy. */
    if (o.cpu_gpr_threads == 4) return 3;
    return wavelet_assist_enabled ? 3 : o.cpu_gpr_threads;
}

static void cpu_gpr_apply_entropy_assist_defaults(const Options& o)
{
#if defined(__linux__)
    if (o.cpu_gpr_threads != 4) return;
    /* The VC-5 helper reads these lazily on first use. Respect explicit
       developer overrides, but make the UI/CLI four-core selection select the
       proven production helper even when RUN_BENCHMARK.sh is bypassed. */
    if (!std::getenv("CINEPI_ENTROPY_ASSIST_SUBBAND")) setenv("CINEPI_ENTROPY_ASSIST_SUBBAND", "8", 0);
    if (!std::getenv("CINEPI_ENTROPY_ASSIST_CHANNELS")) setenv("CINEPI_ENTROPY_ASSIST_CHANNELS", "4", 0);
    if (!std::getenv("CINEPI_ENTROPY_ASSIST_SCHED")) setenv("CINEPI_ENTROPY_ASSIST_SCHED", "normal", 0);
    if (!std::getenv("CINEPI_ENTROPY_ASSIST_NICE")) setenv("CINEPI_ENTROPY_ASSIST_NICE", "0", 0);
    if (!std::getenv("CINEPI_ENTROPY_ASSIST_VERIFY")) setenv("CINEPI_ENTROPY_ASSIST_VERIFY", "8", 0);
#else
    (void)o;
#endif
}

static void cpu_gpr_place_worker(const Options& o, int id, bool wavelet_assist_enabled)
{
#if defined(__linux__)
    if (o.cpu_gpr_rt) {
        sched_param sp{}; sp.sched_priority = 10;
        if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
            static std::once_flag warned;
            std::call_once(warned, []{
                std::cout << "CPU_GPR_RT status=denied "
                             "reason=needs-CAP_SYS_NICE-or-rtprio-rlimit\n"; });
        }
    }
    if (o.cpu_gpr_affinity) {
        const long ncpu = std::max(1L, sysconf(_SC_NPROCESSORS_ONLN));
        const long budget = std::min<long>(std::max(1, o.cpu_gpr_threads), ncpu);
        long core = 0;
        /* Four-selected Pi policy: the three frame owners stay on the proven
           cores 1,2,3. Core 0 is reserved for the selected wavelet assistant plus
           ordinary OS/capture/writer work. */
        if ((wavelet_assist_enabled || o.cpu_gpr_threads == 4) &&
            o.cpu_gpr_affinity_high && ncpu == 4)
            core = id + 1;
        else {
            const long base = o.cpu_gpr_affinity_high ? (ncpu - budget) : 0;
            core = base + (long(id) % budget);
        }
        cpu_set_t set; CPU_ZERO(&set);
        CPU_SET(size_t(core), &set);
        if (pthread_setaffinity_np(pthread_self(), sizeof(set), &set) != 0) {
            static std::once_flag warned;
            std::call_once(warned, []{
                std::cout << "CPU_GPR_AFFINITY status=denied\n"; });
        }
    }
#endif
    /* Offset each worker's first frame so they do not all enter the wavelet
       at once. Paid once, during warmup. */
    if (id > 0) {
        if (o.cpu_gpr_stagger_us > 0)
            std::this_thread::sleep_for(std::chrono::microseconds(id * o.cpu_gpr_stagger_us));
        else if (o.cpu_gpr_stagger_ms > 0)
            std::this_thread::sleep_for(std::chrono::milliseconds(id * o.cpu_gpr_stagger_ms));
    }
}

static void cpu_gpr_place_wavelet_assistant(const Options& o)
{
#if defined(__linux__)
    /* cpu_gpr_place_aux() maps the four-selected layout to core 0. Keep normal
       SCHED_OTHER priority by default. The search can assign one or several
       component planes, but never another complete frame owner; ordinary
       core-0 work can still preempt it normally. batch/idle are explicit
       matrix variants rather than hidden policy. */
    cpu_gpr_place_aux(o);
    int policy = SCHED_OTHER;
#if defined(SCHED_BATCH)
    if (o.core0_wavelet_sched == "batch") policy = SCHED_BATCH;
#endif
#if defined(SCHED_IDLE)
    if (o.core0_wavelet_sched == "idle") policy = SCHED_IDLE;
#endif
    if (policy != SCHED_OTHER) {
        sched_param sp{}; sp.sched_priority = 0;
        if (pthread_setschedparam(pthread_self(), policy, &sp) != 0) {
            static std::once_flag warned;
            std::call_once(warned, [&]{
                std::cout << "CPU_GPR_WAVELET_ASSIST_SCHED status=denied requested="
                          << o.core0_wavelet_sched << "\n";
            });
        }
    }
#else
    (void)o;
#endif
}

static void v2_fused_frame(const V2Sidecar* sc,
                           const Options& o, const ModeSpec* mode,
                           const uint16_t* src, size_t src_stride_elems,
                           int crop_x, int crop_y,
                           const std::vector<uint16_t>& lut,
                           int16_t* dst_planes,
                           size_t plane_stride_elems, size_t row_stride,
                           SplitKind split, EmitKind emit, V2Frame& ctx) {
    const int pw = o.width / 2, ph = o.height / 2;
    /* Per-component tables: CAQ. Four copies of BASE when it is off. */
    const auto quant_caq = cpu_quant_tables_caq(o, mode);
    for (int p = 0; p < 4; ++p) {
        int w = pw, h = ph;
        for (int level = 1; level <= 3; ++level) {
            fused_level_init(ctx.planes[size_t(p)][size_t(level - 1)], w, h, level,
                             quant_caq[size_t(p)], dst_planes + size_t(p) * plane_stride_elems,
                             row_stride, p);
            w /= 2; h /= 2;
        }
    }
    if (ctx.split_row.size() < size_t(pw) * 4u)
        ctx.split_row.assign(size_t(pw) * 4u, int16_t(0));
    int16_t* const gs = ctx.split_row.data();
    int16_t* const rg = gs + pw;
    int16_t* const bg = rg + pw;
    int16_t* const gd = bg + pw;
    const bool gbrg = (o.bayer == "gbrg");
    const int sh = o.true_12bit ? 0 : 1;
    for (int y = 0; y < ph; ++y) {
        const uint16_t* row0 = src + size_t(2 * y + crop_y) * src_stride_elems
                                   + size_t(crop_x);
        v2_split_row(row0, row0 + src_stride_elems, lut.data(), pw, gbrg,
                     gs, rg, bg, gd, sh, 4096, split, unsigned(o.src_shift),
                     o.src_byteswap != 0);
        v2_level_push(ctx.planes[0], 0, gs, emit, sc);
        v2_level_push(ctx.planes[1], 0, rg, emit, sc);
        v2_level_push(ctx.planes[2], 0, bg, emit, sc);
        v2_level_push(ctx.planes[3], 0, gd, emit, sc);
    }
}

struct CpuFusedContext {
    std::array<FusedPlane,4> planes{};
    std::vector<int16_t> split_row;
};

// ---------------------------------------------------------------------------
// Four-selected wavelet assist, v2.
//
// The first implementation let core 0 reread RAW and independently regenerate
// GD. It was bit-correct, but it duplicated the RAW/LUT walk and could erase
// most of the wavelet saving. The owner already has the GD split row hot in
// L1 while it creates GS/RG/BG, so the assistant now receives only those GD
// rows through a small SPSC ring. Core 0 performs ONLY the GD wavelet state
// machine. No RAW frame, coefficient frame, or entropy state changes owner.
// ---------------------------------------------------------------------------

struct WaveletAssistStats {
    std::atomic<long long> submitted{0};
    std::atomic<long long> busy_misses{0};
    std::atomic<long long> completed{0};
    std::atomic<long long> rows_published{0};
    std::atomic<int64_t> helper_wall_ns{0};
    std::atomic<int64_t> helper_cpu_ns{0};
    std::atomic<int64_t> publish_wait_ns{0};
};

static int64_t cpu_gpr_thread_cpu_now_ns()
{
#if defined(__linux__)
    timespec ts{};
    if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts) == 0)
        return int64_t(ts.tv_sec) * 1000000000ll + int64_t(ts.tv_nsec);
#endif
    return 0;
}

// One assistant thread owns a selectable subset of the four component wavelets.
// The primary performs the Bayer/GP-Log2 split once and publishes only the hot
// component rows selected by plane_mask. The assistant never rereads RAW and
// never touches entropy. For gd3 the benchmark creates THREE instances of this
// class, all pinned to core 0: Linux time-slices the three independent GD row
// streams on that one core, which directly tests whether core 0 has enough
// compute budget to service GD for all three frame owners.
class WaveletPlaneAssistant {
public:
    struct Ticket { uint64_t seq = 0; explicit operator bool() const { return seq != 0; } };

    WaveletPlaneAssistant(const Options& o, const ModeSpec* mode,
                          size_t plane_stride, size_t row_stride,
                          bool use_v2, uint8_t plane_mask = 8, int helper_id = 0)
        : o_(o), mode_(mode), plane_stride_(plane_stride), row_stride_(row_stride),
          use_v2_(use_v2), plane_mask_(uint8_t(plane_mask & 0x0fu)),
          helper_id_(helper_id), pw_(o.width / 2), ph_(o.height / 2),
          ring_rows_(o.core0_wavelet_ring_rows) {
        for (int p = 0; p < 4; ++p) if (plane_mask_ & (1u << p)) planes_.push_back(p);
        if (planes_.empty()) throw std::runtime_error("wavelet assistant needs at least one plane");
        row_ring_.assign(size_t(ring_rows_) * planes_.size() * size_t(pw_), int16_t(0));
        thread_ = std::thread([this]{ run(); });
    }

    ~WaveletPlaneAssistant() { stop(); }

    uint8_t plane_mask() const { return plane_mask_; }
    int helper_id() const { return helper_id_; }

    Ticket try_begin(int16_t* coeff, uint8_t* sidecar, bool use_sidecar) {
        std::lock_guard<std::mutex> lock(mu_);
        if (busy_ || stop_) {
            stats.busy_misses.fetch_add(1, std::memory_order_relaxed);
            return {};
        }
        const uint64_t seq = ++next_seq_;
        produced_.store(0, std::memory_order_relaxed);
        consumed_.store(0, std::memory_order_relaxed);
        job_ = Job{seq, coeff, sidecar, use_sidecar};
        busy_ = true;
        stats.submitted.fetch_add(1, std::memory_order_relaxed);
        cv_job_.notify_one();
        return Ticket{seq};
    }

    // rows[p] points at the primary's hot split row for component p. Only the
    // selected planes are copied. A 16-row SPSC ring absorbs short OS/IRQ
    // interruptions on core 0 without moving a full plane or coefficient frame.
    int64_t publish_row(Ticket ticket, const std::array<const int16_t*,4>& rows) {
        if (!ticket) return 0;
        const auto wait_begin = Clock::now();
        uint64_t prod = produced_.load(std::memory_order_relaxed);
        for (;;) {
            const uint64_t cons = consumed_.load(std::memory_order_acquire);
            if (prod - cons < uint64_t(ring_rows_)) break;
            if (stop_) return elapsed_ns(wait_begin);
            std::this_thread::yield();
            prod = produced_.load(std::memory_order_relaxed);
        }
        const int64_t waited = elapsed_ns(wait_begin);
        int16_t* slot = row_ring_.data() +
            size_t(prod % uint64_t(ring_rows_)) * planes_.size() * size_t(pw_);
        for (size_t i = 0; i < planes_.size(); ++i) {
            const int p = planes_[i];
            std::memcpy(slot + i * size_t(pw_), rows[size_t(p)],
                        size_t(pw_) * sizeof(int16_t));
        }
        produced_.store(prod + 1u, std::memory_order_release);
        stats.rows_published.fetch_add(1, std::memory_order_relaxed);
        stats.publish_wait_ns.fetch_add(waited, std::memory_order_relaxed);
        return waited;
    }

    // Compatibility overload for the original GD-only capture path.
    int64_t publish_row(Ticket ticket, const int16_t* gd) {
        const std::array<const int16_t*,4> rows{{nullptr,nullptr,nullptr,gd}};
        return publish_row(ticket, rows);
    }

    int64_t wait(Ticket ticket) {
        if (!ticket) return 0;
        const auto t0 = Clock::now();
        std::unique_lock<std::mutex> lock(mu_);
        cv_done_.wait(lock, [&]{ return completed_seq_ >= ticket.seq || stop_; });
        return elapsed_ns(t0);
    }

    void stop() {
        {
            std::lock_guard<std::mutex> lock(mu_);
            if (stop_) return;
            stop_ = true;
        }
        cv_job_.notify_all();
        cv_done_.notify_all();
        if (thread_.joinable()) thread_.join();
    }

    WaveletAssistStats stats;

private:
    struct Job {
        uint64_t seq = 0;
        int16_t* coeff = nullptr;
        uint8_t* sidecar = nullptr;
        bool use_sidecar = false;
    };

    void run() {
        cpu_gpr_place_wavelet_assistant(o_);
        CpuFusedContext fused_ctx;
        V2Frame v2ctx;
        const auto quant_caq = cpu_quant_tables_caq(o_, mode_);
        for (;;) {
            Job job;
            {
                std::unique_lock<std::mutex> lock(mu_);
                cv_job_.wait(lock, [&]{ return job_.has_value() || stop_; });
                if (!job_ && stop_) return;
                job = *job_;
                job_.reset();
            }

            // Initialise only the planes this helper owns. All helpers for a
            // given frame write disjoint coefficient/sidecar regions.
            for (int p : planes_) {
                int w = pw_, h = ph_;
                for (int level = 1; level <= 3; ++level) {
                    if (use_v2_)
                        fused_level_init(v2ctx.planes[size_t(p)][size_t(level - 1)],
                                         w, h, level, quant_caq[size_t(p)],
                                         job.coeff + size_t(p) * plane_stride_, row_stride_,
                                         p);
                    else
                        fused_level_init(fused_ctx.planes[size_t(p)].level[size_t(level - 1)],
                                         w, h, level, quant_caq[size_t(p)],
                                         job.coeff + size_t(p) * plane_stride_, row_stride_,
                                         p);
                    w /= 2; h /= 2;
                }
            }

            V2Sidecar sc{};
            if (use_v2_ && job.use_sidecar) {
                sc.mask_base = job.sidecar;
                sc.coeff_base = job.coeff;
            }

            const auto wall0 = Clock::now();
            const int64_t cpu0 = cpu_gpr_thread_cpu_now_ns();
            for (int y = 0; y < ph_; ++y) {
                uint64_t cons = consumed_.load(std::memory_order_relaxed);
                while (produced_.load(std::memory_order_acquire) <= cons) {
                    if (stop_) return;
                    std::this_thread::yield();
                }
                const int16_t* slot = row_ring_.data() +
                    size_t(cons % uint64_t(ring_rows_)) * planes_.size() * size_t(pw_);
                for (size_t i = 0; i < planes_.size(); ++i) {
                    const int p = planes_[i];
                    const int16_t* row = slot + i * size_t(pw_);
                    if (use_v2_)
                        v2_level_push(v2ctx.planes[size_t(p)], 0, row, EmitKind::RegDirect,
                                      job.use_sidecar ? &sc : nullptr);
                    else
                        fused_level_push(fused_ctx.planes[size_t(p)], 0, row);
                }
                consumed_.store(cons + 1u, std::memory_order_release);
            }
            const int64_t cpu1 = cpu_gpr_thread_cpu_now_ns();
            stats.helper_wall_ns.fetch_add(elapsed_ns(wall0), std::memory_order_relaxed);
            if (cpu1 >= cpu0)
                stats.helper_cpu_ns.fetch_add(cpu1 - cpu0, std::memory_order_relaxed);
            stats.completed.fetch_add(1, std::memory_order_relaxed);
            {
                std::lock_guard<std::mutex> lock(mu_);
                completed_seq_ = std::max(completed_seq_, job.seq);
                busy_ = false;
            }
            cv_done_.notify_all();
        }
    }

    const Options& o_;
    const ModeSpec* mode_;
    size_t plane_stride_ = 0, row_stride_ = 0;
    bool use_v2_ = false;
    uint8_t plane_mask_ = 0;
    int helper_id_ = -1;
    int pw_ = 0, ph_ = 0;
    int ring_rows_ = 16;
    std::vector<int> planes_;
    std::vector<int16_t> row_ring_;
    std::atomic<uint64_t> produced_{0}, consumed_{0};
    std::mutex mu_;
    std::condition_variable cv_job_, cv_done_;
    std::optional<Job> job_;
    bool busy_ = false;
    std::atomic<bool> stop_{false};
    uint64_t next_seq_ = 0, completed_seq_ = 0;
    std::thread thread_;
};

using GdWaveletAssistant = WaveletPlaneAssistant;

template <typename PublishRows>
static void fused_split_transform_owner_rows(const uint16_t* raw, const uint16_t* lut,
                                              int width, int height, bool gbrg,
                                              int split_sh, int split_mid,
                                              int16_t* dst_planes,
                                              size_t plane_stride_elems,
                                              size_t row_stride,
                                              const std::array<int,10>& quant,
                                              CpuFusedContext& ctx,
                                              uint8_t offload_mask,
                                              PublishRows&& publish_rows) {
    const int pw = width / 2, ph = height / 2;
    for (int p = 0; p < 4; ++p) if (!(offload_mask & (1u << p))) {
        int w = pw, h = ph;
        for (int level = 1; level <= 3; ++level) {
            fused_level_init(ctx.planes[size_t(p)].level[size_t(level - 1)],
                             w, h, level, quant,
                             dst_planes + size_t(p) * plane_stride_elems,
                             row_stride, p);
            w /= 2; h /= 2;
        }
    }
    if (ctx.split_row.size() < size_t(pw) * 4u)
        ctx.split_row.assign(size_t(pw) * 4u, int16_t(0));
    int16_t* const gs = ctx.split_row.data();
    int16_t* const rg = gs + pw;
    int16_t* const bg = rg + pw;
    int16_t* const gd = bg + pw;
    for (int y = 0; y < ph; ++y) {
        const uint16_t* row0 = raw + size_t(2 * y) * size_t(width);
        if (g_cpu_fused_prefetch && y + 4 < ph) {
            const uint16_t* ahead = raw + size_t(2 * (y + 4)) * size_t(width);
            for (int b = 0; b < width * 2; b += 64)
                __builtin_prefetch(reinterpret_cast<const char*>(ahead) + b, 0, 0);
        }
        fused_split_row(row0, row0 + size_t(width), lut, pw, gbrg,
                        gs, rg, bg, gd, split_sh, split_mid);
        const std::array<const int16_t*,4> rows{{gs,rg,bg,gd}};
        publish_rows(rows);
        if (!(offload_mask & 1u)) fused_level_push(ctx.planes[0], 0, gs);
        if (!(offload_mask & 2u)) fused_level_push(ctx.planes[1], 0, rg);
        if (!(offload_mask & 4u)) fused_level_push(ctx.planes[2], 0, bg);
        if (!(offload_mask & 8u)) fused_level_push(ctx.planes[3], 0, gd);
    }
}

template <typename PublishRows>
static void v2_fused_owner_rows(const V2Sidecar* sc,
                                const Options& o, const ModeSpec* mode,
                                const uint16_t* src, size_t src_stride_elems,
                                int crop_x, int crop_y,
                                const std::vector<uint16_t>& lut,
                                int16_t* dst_planes,
                                size_t plane_stride_elems, size_t row_stride,
                                SplitKind split, EmitKind emit, V2Frame& ctx,
                                uint8_t offload_mask,
                                PublishRows&& publish_rows) {
    const int pw = o.width / 2, ph = o.height / 2;
    const auto quant_caq = cpu_quant_tables_caq(o, mode);
    for (int p = 0; p < 4; ++p) if (!(offload_mask & (1u << p))) {
        int w = pw, h = ph;
        for (int level = 1; level <= 3; ++level) {
            fused_level_init(ctx.planes[size_t(p)][size_t(level - 1)], w, h, level,
                             quant_caq[size_t(p)], dst_planes + size_t(p) * plane_stride_elems,
                             row_stride, p);
            w /= 2; h /= 2;
        }
    }
    if (ctx.split_row.size() < size_t(pw) * 4u)
        ctx.split_row.assign(size_t(pw) * 4u, int16_t(0));
    int16_t* const gs = ctx.split_row.data();
    int16_t* const rg = gs + pw;
    int16_t* const bg = rg + pw;
    int16_t* const gd = bg + pw;
    const bool gbrg = (o.bayer == "gbrg");
    const int sh = o.true_12bit ? 0 : 1;
    for (int y = 0; y < ph; ++y) {
        const uint16_t* row0 = src + size_t(2 * y + crop_y) * src_stride_elems
                                   + size_t(crop_x);
        v2_split_row(row0, row0 + src_stride_elems, lut.data(), pw, gbrg,
                     gs, rg, bg, gd, sh, 4096, split, unsigned(o.src_shift),
                     o.src_byteswap != 0);
        const std::array<const int16_t*,4> rows{{gs,rg,bg,gd}};
        publish_rows(rows);
        if (!(offload_mask & 1u)) v2_level_push(ctx.planes[0], 0, gs, emit, sc);
        if (!(offload_mask & 2u)) v2_level_push(ctx.planes[1], 0, rg, emit, sc);
        if (!(offload_mask & 4u)) v2_level_push(ctx.planes[2], 0, bg, emit, sc);
        if (!(offload_mask & 8u)) v2_level_push(ctx.planes[3], 0, gd, emit, sc);
    }
}


// Compatibility wrappers for the capture path; the Core-0 matrix uses the
// generic mask-aware functions directly.
template <typename PublishGd>
static void fused_split_transform_primary3_rows(const uint16_t* raw, const uint16_t* lut,
                                                 int width, int height, bool gbrg,
                                                 int split_sh, int split_mid,
                                                 int16_t* dst_planes,
                                                 size_t plane_stride_elems,
                                                 size_t row_stride,
                                                 const std::array<int,10>& quant,
                                                 CpuFusedContext& ctx,
                                                 PublishGd&& publish_gd) {
    auto pub = [&](const std::array<const int16_t*,4>& rows){ publish_gd(rows[3]); };
    fused_split_transform_owner_rows(raw, lut, width, height, gbrg, split_sh, split_mid,
                                     dst_planes, plane_stride_elems, row_stride, quant,
                                     ctx, uint8_t(8), pub);
}

template <typename PublishGd>
static void v2_fused_primary3_rows(const V2Sidecar* sc,
                                   const Options& o, const ModeSpec* mode,
                                   const uint16_t* src, size_t src_stride_elems,
                                   int crop_x, int crop_y,
                                   const std::vector<uint16_t>& lut,
                                   int16_t* dst_planes,
                                   size_t plane_stride_elems, size_t row_stride,
                                   SplitKind split, EmitKind emit, V2Frame& ctx,
                                   PublishGd&& publish_gd) {
    auto pub = [&](const std::array<const int16_t*,4>& rows){ publish_gd(rows[3]); };
    v2_fused_owner_rows(sc, o, mode, src, src_stride_elems, crop_x, crop_y, lut,
                        dst_planes, plane_stride_elems, row_stride, split, emit,
                        ctx, uint8_t(8), pub);
}

static void verify_wavelet_assist_mask(const Options& o, const ModeSpec* mode,
                                       const std::vector<uint16_t>& src,
                                       const std::vector<uint16_t>& lut,
                                       size_t plane_stride, size_t row_stride,
                                       bool use_v2, uint8_t plane_mask) {
    std::vector<int16_t> ref(plane_stride * 4u, int16_t(0));
    std::vector<int16_t> got(plane_stride * 4u, int16_t(0));
    std::vector<uint8_t> ref_sc, got_sc;
    if (use_v2 && o.cpu_sidecar) {
        ref_sc.assign(plane_stride * 4u / 8u, 0u);
        got_sc.assign(plane_stride * 4u / 8u, 0u);
    }

    if (use_v2) {
        V2Frame ref_ctx, p_ctx;
        V2Sidecar rsc{}, gsc{};
        if (!ref_sc.empty()) { rsc.mask_base = ref_sc.data(); rsc.coeff_base = ref.data(); }
        if (!got_sc.empty()) { gsc.mask_base = got_sc.data(); gsc.coeff_base = got.data(); }
        v2_fused_frame(ref_sc.empty() ? nullptr : &rsc, o, mode, src.data(), size_t(o.width),
                       0, 0, lut, ref.data(), plane_stride, row_stride,
                       SplitKind::Shipped, EmitKind::RegDirect, ref_ctx);
        WaveletPlaneAssistant assist(o, mode, plane_stride, row_stride, true, plane_mask, 99);
        const auto ticket = assist.try_begin(got.data(), got_sc.empty() ? nullptr : got_sc.data(),
                                             !got_sc.empty());
        if (!ticket) throw std::runtime_error("wavelet assist verify could not reserve helper");
        auto publish = [&](const std::array<const int16_t*,4>& rows){
            (void)assist.publish_row(ticket, rows);
        };
        v2_fused_owner_rows(got_sc.empty() ? nullptr : &gsc, o, mode, src.data(),
                            size_t(o.width), 0, 0, lut, got.data(), plane_stride,
                            row_stride, SplitKind::Shipped, EmitKind::RegDirect,
                            p_ctx, plane_mask, publish);
        (void)assist.wait(ticket);
        assist.stop();
    } else {
        CpuFusedContext ref_ctx, p_ctx;
        fused_split_transform_frame(src.data(), lut.data(), o.width, o.height,
                                    o.bayer == "gbrg", o.true_12bit ? 0 : 1, 4096,
                                    ref.data(), plane_stride, row_stride,
                                    cpu_quant_table(o, mode), ref_ctx.planes, ref_ctx.split_row);
        WaveletPlaneAssistant assist(o, mode, plane_stride, row_stride, false, plane_mask, 99);
        const auto ticket = assist.try_begin(got.data(), nullptr, false);
        if (!ticket) throw std::runtime_error("wavelet assist verify could not reserve helper");
        auto publish = [&](const std::array<const int16_t*,4>& rows){
            (void)assist.publish_row(ticket, rows);
        };
        fused_split_transform_owner_rows(src.data(), lut.data(), o.width, o.height,
                                         o.bayer == "gbrg", o.true_12bit ? 0 : 1, 4096,
                                         got.data(), plane_stride, row_stride,
                                         cpu_quant_table(o, mode), p_ctx, plane_mask, publish);
        (void)assist.wait(ticket);
        assist.stop();
    }
    if (ref != got)
        throw std::runtime_error("wavelet assist coefficient cross-check failed for mask=" +
                                 cpu_gpr_plane_mask_name(plane_mask));
    if (ref_sc != got_sc)
        throw std::runtime_error("wavelet assist sidecar cross-check failed for mask=" +
                                 cpu_gpr_plane_mask_name(plane_mask));
    if (plane_mask == uint8_t(8))
        std::cout << "CPU_GPR_WAVELET_ASSIST_VERIFY bitexact=YES plane=GD planes=GD transport=row-ring raw_reread=NO\n";
    else
        std::cout << "CPU_GPR_WAVELET_ASSIST_VERIFY bitexact=YES planes="
                  << cpu_gpr_plane_mask_name(plane_mask)
                  << " transport=row-ring raw_reread=NO\n";
}

static void verify_wavelet_assist_plan(const Options& o, const ModeSpec* mode,
                                       const std::vector<uint16_t>& src,
                                       const std::vector<uint16_t>& lut,
                                       size_t plane_stride, size_t row_stride,
                                       bool use_v2, const Core0WaveletPlan& plan) {
    std::array<bool,16> seen{};
    for (uint8_t mask : plan.masks) {
        if (!mask || seen[mask]) continue;
        seen[mask] = true;
        verify_wavelet_assist_mask(o, mode, src, lut, plane_stride, row_stride,
                                   use_v2, mask);
    }
}


static void verify_gd_wavelet_assist(const Options& o, const ModeSpec* mode,
                                     const std::vector<uint16_t>& src,
                                     const std::vector<uint16_t>& lut,
                                     size_t plane_stride, size_t row_stride,
                                     bool use_v2) {
    verify_wavelet_assist_mask(o, mode, src, lut, plane_stride, row_stride, use_v2, uint8_t(8));
}

// RAW16 -> coefficient planes, one streaming pass, nothing intermediate in DRAM.
// v3.1: this is the entry point the production library uses, and it used to
// have no v2 branch at all -- so the camera ran the SHIPPED emit while every
// benchmark number described the v2 one. Three optimisations (ll_vqadd,
// emit_kind1 and the mask-guided entropy prefetch, which needs a sidecar)
// live only in the v2 emit and therefore never reached the camera.
//
// The two emits are bit-identical by construction; the m5 suite's kernel
// self-check existed precisely to prove that, comparing v2's coefficient
// planes against the shipped path's on every run. So dispatching here
// changes speed, not output.
//
// `sc` may be null: the sidecar is optional and the caller owns the buffer.
static void cpu_fused_frame_from_raw(const Options& o, const ModeSpec* mode,
                                     const uint16_t* raw_data,
                                     const std::vector<uint16_t>& lut,
                                     int16_t* dst_planes,
                                     size_t plane_stride_elems, size_t row_stride,
                                     CpuFusedContext& ctx,
                                     const V2Sidecar* sc = nullptr,
                                     V2Frame* v2ctx = nullptr) {
    if (o.cpu_v2_kernel && v2ctx != nullptr) {
        // v3.10: read the source WHERE IT LIES.
        //
        // This used to hardcode a stride of o.width and a zero crop, because
        // the library only ever received a tightly packed frame. That forced
        // the camera path to memcpy every frame out of the DMA buffer -- the
        // "one copy to 8 tight DRAM slots" in the recorder log, 16.8 MB per
        // frame, about 14% of the per-frame budget once contention is
        // removed. The cascade never needed it: v2_fused_frame has taken a
        // source stride and crop since the transport work, and the static
        // benchmark has always used them to read a padded 3856x2180 buffer
        // in place.
        //
        // src_stride_elems is in SAMPLES, not bytes. Zero means "tight".
        const size_t src_stride = o.src_stride_elems ? o.src_stride_elems
                                                     : size_t(o.width);
        v2_fused_frame(sc, o, mode, raw_data, src_stride,
                       o.src_crop_x, o.src_crop_y, lut,
                       dst_planes, plane_stride_elems, row_stride,
                       SplitKind::Shipped, EmitKind::RegDirect, *v2ctx);
        return;
    }
    fused_split_transform_frame(raw_data, lut.data(), o.width, o.height,
                                o.bayer == "gbrg", o.true_12bit ? 0 : 1, 4096, dst_planes,
                                plane_stride_elems, row_stride,
                                cpu_quant_table(o, mode), ctx.planes, ctx.split_row);
}

// Split planes -> coefficient planes. src and dst must differ.
static void cpu_fused_frame_from_planes(const Options& o, const ModeSpec* mode,
                                        const int16_t* src_planes, int16_t* dst_planes,
                                        size_t plane_stride_elems, size_t row_stride,
                                        CpuFusedContext& ctx) {
    fused_transform_frame(src_planes, dst_planes, plane_stride_elems, row_stride,
                          o.width / 2, o.height / 2,
                          cpu_quant_table(o, mode), ctx.planes);
}

static void cpu_transform_tail_inplace(const Options& o, const ModeSpec* mode,
                                       int16_t* coefficients,
                                       size_t plane_stride_elems,
                                       size_t row_stride,
                                       int first_level) {
    if (first_level < 1 || first_level > 3)
        throw std::runtime_error("CPU wavelet tail first level must be 1..3");
    const auto specs = plane_specs(o);
    int16_t* const horizontal = cpu_wavelet_workspace(plane_stride_elems);
    for (int plane = 0; plane < 4; ++plane) {
        int16_t* frame = coefficients + size_t(plane) * plane_stride_elems;
        int cw = specs[size_t(plane)].width >> (first_level - 1);
        int ch = specs[size_t(plane)].height >> (first_level - 1);
        for (int level = first_level; level <= specs[size_t(plane)].levels; ++level) {
            const CpuWaveletPush h{uint32_t(cw), uint32_t(ch), uint32_t(row_stride),
                                   uint32_t(row_stride), 0u,
                                   level == 1 ? 0 : 2};
            cpu_wavelet_pass(frame, horizontal, h);
            const int base = 10 - 3 * level;
            const int q1 = mode ? mode->quant_table[size_t(base)] : std::max(1, o.quant);
            const int q2 = mode ? mode->quant_table[size_t(base + 1)] : std::max(1, o.quant);
            const int q3 = mode ? mode->quant_table[size_t(base + 2)] : std::max(1, o.quant);
            const CpuWaveletPush v{uint32_t(ch), uint32_t(cw), uint32_t(row_stride),
                                   uint32_t(row_stride), 1u, 0, q1, q2, q3};
            cpu_wavelet_pass(horizontal, frame, v);
            cw /= 2;
            ch /= 2;
        }
    }
}

static std::vector<int16_t> pack_wavelet_level_buffers(
        const Options& o,
        const std::array<const int16_t*, MAX_WAVELET_COUNT>& levels,
        size_t plane_stride_elems, size_t row_stride) {
    const auto specs = plane_specs(o);
    std::vector<int16_t> packed(plane_stride_elems * 4u, int16_t(0));
    auto copy_rect = [&](const int16_t* source, int16_t* destination,
                         int x, int y, int width, int height) {
        for (int row = 0; row < height; ++row) {
            const size_t offset = size_t(y + row) * row_stride + size_t(x);
            std::memcpy(destination + offset, source + offset,
                        size_t(width) * sizeof(int16_t));
        }
    };
    for (int channel = 0; channel < 4; ++channel) {
        int16_t* destination = packed.data() + size_t(channel) * plane_stride_elems;
        for (int level = 0; level < specs[size_t(channel)].levels; ++level) {
            const int current_width = specs[size_t(channel)].width >> level;
            const int current_height = specs[size_t(channel)].height >> level;
            const int band_width = current_width / 2;
            const int band_height = current_height / 2;
            const int16_t* source = levels[size_t(level)] +
                                    size_t(channel) * plane_stride_elems;
            copy_rect(source, destination, band_width, 0, band_width, band_height);
            copy_rect(source, destination, 0, band_height, band_width, band_height);
            copy_rect(source, destination, band_width, band_height, band_width, band_height);
            if (level + 1 == specs[size_t(channel)].levels)
                copy_rect(source, destination, 0, 0, band_width, band_height);
        }
    }
    return packed;
}

struct ModeProfile {
    const char* name;
    std::array<int,10> base;
    double scale;
};
// Production quantisation ladder: CinePi Universal Standard Quant v3.
//
// This is the only M1-M10 ladder in the production encoder. Historical v19
// and Previous ladders were removed so the UI, benchmark and live library
// cannot select or silently fall back to another quant table.
//
// Band order:
//   LL / L3-LH / L3-HL / L3-HH / L2-LH / L2-HL / L2-HH /
//   L1-LH / L1-HL / L1-HH
//
// These tables were promoted after paired GPR validation on the bundled
// pre-companded IMX585 GP-Log2 sample and a deterministic noisy low-light
// sample. Every mode reduced GPR size and high-pass non-zero count versus the
// Previous ladder while preserving or improving full-frame/shadow fidelity.
static const ModeProfile kModeProfiles[] = {
    {"m1",  {1,4,4,2,8,8,6,26,26,39},          1.00000000},
    {"m2",  {1,4,4,2,8,8,6,33,33,50},          1.00000000},
    {"m3",  {1,4,4,2,10,10,6,43,43,66},          1.00000000},
    {"m4",  {1,6,6,4,14,14,10,55,55,87},        1.00000000},
    {"m5",  {1,8,8,4,16,16,11,73,73,111},      1.00000000},
    {"m6",  {1,10,10,6,20,20,14,93,93,143},    1.00000000},
    {"m7",  {1,14,14,8,26,26,18,126,126,175},  1.00000000},
    {"m8",  {1,18,18,9,34,34,22,155,155,235},  1.00000000},
    {"m9",  {1,22,22,13,44,44,30,201,201,301}, 1.00000000},
    {"m10", {1,29,29,13,45,45,35,272,272,521}, 1.00000000},
};
static uint32_t mode_index_by_name(const std::string& name) {
    for (uint32_t i=0; i<std::size(kModeProfiles); ++i)
        if (name == kModeProfiles[i].name) return i+1u;
    return 0u;
}
static ModeSpec resolve_mode_from(const ModeProfile* table, size_t n,
                                  const std::string& name) {
    for (size_t k = 0; k < n; ++k) {
        const auto& profile = table[k];
        if (name != profile.name) continue;
        ModeSpec m; m.name = name;
        m.quant_table[0] = 1;
        for (size_t i=1; i<m.quant_table.size(); ++i)
            m.quant_table[i] = std::max(1, int(std::llround(double(profile.base[i]) * profile.scale)));
        return m;
    }
    throw std::runtime_error("unknown mode: " + name);
}
static ModeSpec resolve_mode(const std::string& name, double, int) {
    return resolve_mode_from(kModeProfiles, std::size(kModeProfiles), name);
}

static bool should_compute_frame_crc(bool no_crc, bool collect, int frame) noexcept {
    // The first measured frame is always checksummed so the fail-closed gate
    // remains available even in normal --no-crc performance runs.
    return !no_crc || (collect && frame == 0);
}

static uint32_t crc32_bytes(const void* data, size_t size) {
    static const std::array<uint32_t,256> table=[] {
        std::array<uint32_t,256> t{};
        for(uint32_t i=0;i<256;++i){uint32_t c=i;for(int b=0;b<8;++b)c=(c>>1)^(0xedb88320u&(0u-(c&1u)));t[i]=c;}
        return t;
    }();
    uint32_t c=0xffffffffu;
    const uint8_t* p=static_cast<const uint8_t*>(data);
    while(size--) c=(c>>8)^table[(c^*p++)&0xffu];
    return ~c;
}

#define VK_CHECK(expr) do { VkResult _r=(expr); if(_r!=VK_SUCCESS) throw std::runtime_error(std::string(#expr)+" failed with VkResult "+std::to_string(int(_r))); } while(0)

static std::vector<uint32_t> read_spv(const std::string& path) {
    std::ifstream f(path,std::ios::binary|std::ios::ate);
    if (!f) {
        // GPU-lab auxiliary shaders are not shipped in every package (the
        // campaign moved to the all-CPU pipeline). Say so plainly instead of
        // leaving the caller to guess at a bare open failure.
        const bool aux = path.find("aux_shaders/") != std::string::npos;
        throw std::runtime_error(
            "Cannot open shader: " + path +
            (aux ? "  -- this is a GPU-lab auxiliary shader and is not built "
                   "in this package. The CPU pipeline (--execution cpu-gpr, "
                   "which is what --cpu-winner and the UI benchmark use) does "
                   "not need it; only the GPU hybrid paths do."
                 : ""));
    }
    const auto n=f.tellg(); if(n<=0 || (n%4)!=0)throw std::runtime_error("Invalid SPIR-V size");
    std::vector<uint32_t> v(size_t(n)/4); f.seekg(0); f.read(reinterpret_cast<char*>(v.data()),n); if(!f)throw std::runtime_error("Failed reading shader"); return v;
}

// Read the declared workgroup size out of the SPIR-V so the host dispatch grid
// can be checked against the shader rather than assumed. The tile-direct path
// has two geometries (8x8 and 16x16) with an identical push ABI, so a mismatched
// --tile-direct-workgroup would silently dispatch the wrong number of groups and
// write outside the coefficient pools. This is the same class of silent mismatch
// as the v1.7.34 --shader / --tile-direct-shader defect.
struct SpirvLocalSize { uint32_t x = 0, y = 0, z = 0; bool found = false; };

static SpirvLocalSize spirv_local_size(const std::vector<uint32_t>& words) {
    SpirvLocalSize size;
    if (words.size() < 5u || words[0] != 0x07230203u) return size;
    size_t index = 5u;
    while (index < words.size()) {
        const uint32_t count = words[index] >> 16u;
        const uint32_t opcode = words[index] & 0xffffu;
        if (count == 0u || index + count > words.size()) break;
        // OpExecutionMode = 16, ExecutionMode LocalSize = 17.
        if (opcode == 16u && count >= 6u && words[index + 2u] == 17u) {
            size.x = words[index + 3u];
            size.y = words[index + 4u];
            size.z = words[index + 5u];
            size.found = true;
            break;
        }
        index += count;
    }
    return size;
}

struct VulkanPipeline {
    struct Push {
        uint32_t axis_len,count,stride,out_stride,dir;
        int32_t prescale; int32_t q_lh,q_hl,q_hh;
        uint32_t plane,level;
        uint32_t tile_base_lh,tile_base_hl,tile_base_hh,tiles_x,ll_stride;
        uint32_t coeff_base_lh,coeff_base_hl,coeff_base_hh,band_width;
    };
    struct Slot {
        VkBuffer ping=VK_NULL_HANDLE,pong=VK_NULL_HANDLE,mid=VK_NULL_HANDLE,coeff=VK_NULL_HANDLE;
        VkBuffer upload=VK_NULL_HANDLE;
        VkBuffer mirror=VK_NULL_HANDLE,overflow=VK_NULL_HANDLE;
        VkBuffer tile_i8=VK_NULL_HANDLE,tile_i16=VK_NULL_HANDLE,tile_flags=VK_NULL_HANDLE;
        VkBuffer readback_coeff=VK_NULL_HANDLE;
        VkDeviceMemory ping_mem=VK_NULL_HANDLE,pong_mem=VK_NULL_HANDLE,mid_mem=VK_NULL_HANDLE,coeff_mem=VK_NULL_HANDLE;
        VkDeviceMemory upload_mem=VK_NULL_HANDLE;
        VkDeviceMemory mirror_mem=VK_NULL_HANDLE,overflow_mem=VK_NULL_HANDLE;
        VkDeviceMemory tile_i8_mem=VK_NULL_HANDLE,tile_i16_mem=VK_NULL_HANDLE,tile_flags_mem=VK_NULL_HANDLE;
        VkDeviceMemory readback_coeff_mem=VK_NULL_HANDLE;
        int16_t* mapped_ping=nullptr;
        int16_t* mapped_pong=nullptr;
        int16_t* mapped_mid=nullptr;
        int16_t* mapped_coeff=nullptr;
        uint8_t* mapped_mirror=nullptr;
        uint32_t* mapped_overflow=nullptr;
        int8_t* mapped_tile_i8=nullptr;
        int16_t* mapped_tile_i16=nullptr;
        uint32_t* mapped_tile_flags=nullptr;
        int16_t* mapped_readback_coeff=nullptr;
        std::array<VkDescriptorSet,12> sets{};
        VkCommandBuffer cmd=VK_NULL_HANDLE;
        VkFence fence=VK_NULL_HANDLE;
        VkQueryPool query=VK_NULL_HANDLE;
        int frame=-1;
        int entropy_worker=-1;
        Clock::time_point entropy_ready{};
        Clock::time_point entropy_queue_enter{},entropy_finish{};
        Clock::time_point start{},submit{},finish{};
        int64_t split_ns=0,gpu_wall_ns=0,gpu_exec_ns=-1,entropy_ns=0,entropy_queue_wait_ns=0,writer_ns=0;
        int64_t int8_range_ns=0,int8_pack_ns=0,cpu_tail_ns=0,snapshot_ns=0;
        uint64_t hybrid_bytes=0;
        uint32_t int8_bands=0,int16_fallback_bands=0;
        uint32_t int8_tiles=0,int16_fallback_tiles=0,total_tiles=0;
        uint32_t crc=0;
        bool crc_computed=false;
        gpr_buffer gpr{nullptr,0};
        size_t vc5_bytes=0;
        std::vector<int16_t> readback;   // cached bounce buffer (--readback-copy)
        std::vector<int16_t> host_ping;
        // --wide-separable line buffers: v101 writes them, v102 reads them, the CPU
        // never touches them. Never mapped.
        VkBuffer hlow=VK_NULL_HANDLE, hhigh=VK_NULL_HANDLE;
        VkDeviceMemory hlow_mem=VK_NULL_HANDLE, hhigh_mem=VK_NULL_HANDLE;
        // 0,1 = v101 with ping / pong as input;  2,3 = v102 with pong / ping as the
        // next-LL target;  4 = v103. Separate from `sets` so the existing
        // plane*2+dir indexing is untouched.
        std::array<VkDescriptorSet,5> wide_sets{};
    };
    // --wide-separable push blocks. Byte layout must match the GLSL declarations in
    // optimised_shaders/vc5_forward_26_v10{1,2,3}_*.comp exactly.
    struct WidePushH {
        uint32_t width, height, stride, band_stride, plane_stride, band_plane;
        int32_t  prescale;
    };
    struct WidePushV {
        uint32_t width, height, band_stride, band_plane, out_stride, ll_plane, final_level;
        int32_t  q_lh, q_hl, q_hh;
        uint32_t coeff_base_lh, coeff_base_hl, coeff_base_hh, coeff_plane, band_width;
        uint32_t tile_base_lh, tile_base_hl, tile_base_hh, tiles_x, tile_plane;
        uint32_t rows_per_march;
    };
    struct WidePushP {
        uint32_t band_width, band_height, coeff_base, tile_base, tiles_x, tiles_y;
    };
    VkInstance instance=VK_NULL_HANDLE; VkPhysicalDevice physical=VK_NULL_HANDLE; VkDevice device=VK_NULL_HANDLE;
    VkQueue queue=VK_NULL_HANDLE; uint32_t family=UINT32_MAX;
    VkDescriptorSetLayout dsl=VK_NULL_HANDLE; VkPipelineLayout layout=VK_NULL_HANDLE; VkPipeline pipeline=VK_NULL_HANDLE;
    VkDescriptorPool desc_pool=VK_NULL_HANDLE; VkCommandPool cmd_pool=VK_NULL_HANDLE;
    // --wide-separable. Shares `dsl` (the tile-direct layout already has the 6
    // storage-buffer bindings these shaders need) but needs its own pipeline layout
    // because WidePushV is larger than Push.
    bool wide_separable=false, wide_hybrid_pack=true;
    uint32_t wide_rows_per_march=32;
    VkPipelineLayout wide_layout=VK_NULL_HANDLE;
    VkPipeline wide_h_pipeline=VK_NULL_HANDLE, wide_v_pipeline=VK_NULL_HANDLE, wide_p_pipeline=VK_NULL_HANDLE;
    uint32_t wide_h_wg=0, wide_v_wg=0, wide_p_wg=0;   // read back from the SPIR-V
    size_t band_stride=0, band_plane_elems=0, band_buffer_bytes=0;
    std::vector<Slot> slots;
    size_t row_stride=0, plane_elems=0, plane_stride_elems=0, plane_bytes=0, buffer_bytes=0, mirror_plane_bytes=0, mirror_buffer_bytes=0;
    size_t tile_ll_width=0,tile_ll_height=0,tile_ll_plane_elems=0,tile_ll_plane_bytes=0,tile_ll_buffer_bytes=0;
    uint32_t tile_total_count=0,tile_flag_word_count=0;
    size_t tile_i8_buffer_bytes=0,tile_i16_buffer_bytes=0,tile_flags_buffer_bytes=0;
    std::array<std::array<std::array<uint32_t,3>,3>,4> tile_base{};
    std::array<std::array<std::array<uint32_t,3>,3>,4> tile_coeff_base{};
    size_t tile_linear_coefficient_count=0;
    std::array<uint32_t,3> tile_level_tiles_x{{0,0,0}},tile_level_tiles_y{{0,0,0}};
    bool timestamps=false; double timestamp_ns=0.0;
    // No CPU fallback is permitted for the encode transform.
    bool host_cpu_transform=false;
    int workgroup_size=128;
    std::array<int,3> prescale{{0,2,2}}; // GoPro SDK 12-bit schedule for stages 0,1,2
    bool tiled_dispatch=false;   // shared-memory two-pass shader
    bool fused2d_dispatch=false;  // fused H+V shader with persistent coefficient output
    bool retained_int16_levels=false; // Pi fallback: keep level 1/2/3 buffers independently
    bool fused_mirror_output=false; // V18 production default D writes a fixed-layout int8 mirror in the fused shader
    bool persistent_full_output=false; // V51 writes one complete nested int16 coefficient frame
    bool tile_direct_output=false; // V18 laboratory writes either int8 or int16 per 8x8 high-pass tile
    bool tile_linear_layout=false;
    bool tile_flags_u32=false;
    uint8_t tile_reader_mode=2u;
    bool dispatch_schedule_level_major=false;
    bool gpu_trace=false;
    bool planes_in_z=false;
    uint32_t trace_queries=2u;                      // timestamps per frame
    std::vector<std::pair<int,int>> trace_labels;   // (plane, level) per interval
    std::vector<uint64_t> trace_accum_ns;           // summed per interval
    uint64_t trace_frames=0;
    uint32_t fused_pairs_x=8u; // output pairs per workgroup X axis
    uint32_t fused_pairs_y=8u; // output pairs per workgroup Y axis
    uint32_t fused_pairs_per_thread_x=1u;
    uint32_t fused_pairs_per_thread_y=1u;
    std::string barrier_scope="global";
    std::string command_usage="simultaneous";
    std::string dispatch_order="serial";
    Options host_options{};
    const ModeSpec* mode_spec=nullptr;
    std::array<int,10> quant_table{{1,1,1,1,1,1,1,1,1,1}};
    bool mapped_host_cached=false;
    bool use_readback_copy=false;
    bool use_vulkan_readback_copy=false;
    bool device_local_input=false;
    bool device_local_coeff=false;
    VkMemoryPropertyFlags input_memory_flags=0;
    VkMemoryPropertyFlags coeff_memory_flags=0;
    int gpu_levels=3;
    std::array<PlaneSpec,4> specs{};

    ~VulkanPipeline(){cleanup();}
    void cleanup(){
        if(device) vkDeviceWaitIdle(device);
        for(auto& s : slots){
            if(device && s.mapped_ping) vkUnmapMemory(device,s.upload_mem ? s.upload_mem : s.ping_mem);
            if(device && s.mapped_pong && s.pong_mem) vkUnmapMemory(device,s.pong_mem);
            if(device && s.mapped_mid && s.mid_mem) vkUnmapMemory(device,s.mid_mem);
            if(device && s.mapped_coeff && s.coeff_mem) vkUnmapMemory(device,s.coeff_mem);
            if(device && s.mapped_mirror && s.mirror_mem) vkUnmapMemory(device,s.mirror_mem);
            if(device && s.mapped_overflow && s.overflow_mem) vkUnmapMemory(device,s.overflow_mem);
            if(device && s.mapped_tile_i8 && s.tile_i8_mem) vkUnmapMemory(device,s.tile_i8_mem);
            if(device && s.mapped_tile_i16 && s.tile_i16_mem) vkUnmapMemory(device,s.tile_i16_mem);
            if(device && s.mapped_tile_flags && s.tile_flags_mem) vkUnmapMemory(device,s.tile_flags_mem);
            if(device && s.mapped_readback_coeff && s.readback_coeff_mem) vkUnmapMemory(device,s.readback_coeff_mem);
            if(device && s.upload) vkDestroyBuffer(device,s.upload,nullptr);
            if(device && s.ping) vkDestroyBuffer(device,s.ping,nullptr);
            if(device && s.pong) vkDestroyBuffer(device,s.pong,nullptr);
            if(device && s.mid) vkDestroyBuffer(device,s.mid,nullptr);
            if(device && s.coeff) vkDestroyBuffer(device,s.coeff,nullptr);
            if(device && s.mirror) vkDestroyBuffer(device,s.mirror,nullptr);
            if(device && s.overflow) vkDestroyBuffer(device,s.overflow,nullptr);
            if(device && s.tile_i8) vkDestroyBuffer(device,s.tile_i8,nullptr);
            if(device && s.tile_i16) vkDestroyBuffer(device,s.tile_i16,nullptr);
            if(device && s.tile_flags) vkDestroyBuffer(device,s.tile_flags,nullptr);
            if(device && s.readback_coeff) vkDestroyBuffer(device,s.readback_coeff,nullptr);
            if(device && s.upload_mem) vkFreeMemory(device,s.upload_mem,nullptr);
            if(device && s.ping_mem) vkFreeMemory(device,s.ping_mem,nullptr);
            if(device && s.pong_mem) vkFreeMemory(device,s.pong_mem,nullptr);
            if(device && s.mid_mem) vkFreeMemory(device,s.mid_mem,nullptr);
            if(device && s.coeff_mem) vkFreeMemory(device,s.coeff_mem,nullptr);
            if(device && s.mirror_mem) vkFreeMemory(device,s.mirror_mem,nullptr);
            if(device && s.overflow_mem) vkFreeMemory(device,s.overflow_mem,nullptr);
            if(device && s.tile_i8_mem) vkFreeMemory(device,s.tile_i8_mem,nullptr);
            if(device && s.tile_i16_mem) vkFreeMemory(device,s.tile_i16_mem,nullptr);
            if(device && s.tile_flags_mem) vkFreeMemory(device,s.tile_flags_mem,nullptr);
            if(device && s.readback_coeff_mem) vkFreeMemory(device,s.readback_coeff_mem,nullptr);
            if(device && s.hlow) vkDestroyBuffer(device,s.hlow,nullptr);
            if(device && s.hhigh) vkDestroyBuffer(device,s.hhigh,nullptr);
            if(device && s.hlow_mem) vkFreeMemory(device,s.hlow_mem,nullptr);
            if(device && s.hhigh_mem) vkFreeMemory(device,s.hhigh_mem,nullptr);
            if(device && s.query) vkDestroyQueryPool(device,s.query,nullptr);
            if(device && s.fence) vkDestroyFence(device,s.fence,nullptr);
        }
        if(wide_h_pipeline) vkDestroyPipeline(device,wide_h_pipeline,nullptr);
        if(wide_v_pipeline) vkDestroyPipeline(device,wide_v_pipeline,nullptr);
        if(wide_p_pipeline) vkDestroyPipeline(device,wide_p_pipeline,nullptr);
        if(wide_layout) vkDestroyPipelineLayout(device,wide_layout,nullptr);
        if(pipeline) vkDestroyPipeline(device,pipeline,nullptr);
        if(layout) vkDestroyPipelineLayout(device,layout,nullptr);
        if(desc_pool) vkDestroyDescriptorPool(device,desc_pool,nullptr);
        if(dsl) vkDestroyDescriptorSetLayout(device,dsl,nullptr);
        if(cmd_pool) vkDestroyCommandPool(device,cmd_pool,nullptr);
        if(device) vkDestroyDevice(device,nullptr);
        if(instance) vkDestroyInstance(instance,nullptr);
    }
    std::optional<uint32_t> mem_type(uint32_t bits,VkMemoryPropertyFlags required,VkMemoryPropertyFlags preferred=0){
        VkPhysicalDeviceMemoryProperties p{};vkGetPhysicalDeviceMemoryProperties(physical,&p);
        for(int pass=0;pass<2;++pass){const auto want=required|(pass==0?preferred:0);for(uint32_t i=0;i<p.memoryTypeCount;++i)if((bits&(1u<<i))&&((p.memoryTypes[i].propertyFlags&want)==want))return i;}
        return std::nullopt;
    }
    void create_buffer(VkDeviceSize size,VkBufferUsageFlags usage,VkMemoryPropertyFlags req,VkMemoryPropertyFlags pref,VkBuffer&b,VkDeviceMemory&m,bool* host_cached_out=nullptr,VkMemoryPropertyFlags* selected_flags_out=nullptr){
        VkBufferCreateInfo bi{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};bi.size=size;bi.usage=usage;bi.sharingMode=VK_SHARING_MODE_EXCLUSIVE;VK_CHECK(vkCreateBuffer(device,&bi,nullptr,&b));
        VkMemoryRequirements mr{};vkGetBufferMemoryRequirements(device,b,&mr);auto mt=mem_type(mr.memoryTypeBits,req,pref);if(!mt)throw std::runtime_error("No suitable Vulkan memory type");
        VkPhysicalDeviceMemoryProperties mp{};vkGetPhysicalDeviceMemoryProperties(physical,&mp);
        const VkMemoryPropertyFlags selected=mp.memoryTypes[*mt].propertyFlags;
        if(host_cached_out)*host_cached_out=(selected&VK_MEMORY_PROPERTY_HOST_CACHED_BIT)!=0;
        if(selected_flags_out)*selected_flags_out=selected;
        VkMemoryAllocateInfo ai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};ai.allocationSize=mr.size;ai.memoryTypeIndex=*mt;VK_CHECK(vkAllocateMemory(device,&ai,nullptr,&m));VK_CHECK(vkBindBufferMemory(device,b,m,0));
    }
    void init(const Options&o,const ModeSpec* mode,const std::string&shader_path){
        mode_spec = mode;
        if (mode) quant_table = mode->quant_table; else for(size_t i=1;i<quant_table.size();++i) quant_table[i]=std::max(1,o.quant);
        workgroup_size=o.workgroup; barrier_scope=o.barrier_scope; command_usage=o.command_usage; dispatch_order=o.dispatch_order;
        prescale = {{0,2,2}};
        tiled_dispatch = o.tiled_dispatch;
        fused2d_dispatch = o.fused2d_dispatch;
        fused_mirror_output = o.coeff_storage == Options::CoeffStorage::GpuHybridFusedMirror;
        persistent_full_output = o.coeff_storage == Options::CoeffStorage::GpuFullPersistent ||
                                 o.coeff_storage == Options::CoeffStorage::CpuHybridBand ||
                                 o.coeff_storage == Options::CoeffStorage::CpuHybridTile ||
                                 o.coeff_storage == Options::CoeffStorage::GpuHybridV15 ||
                                 o.coeff_storage == Options::CoeffStorage::GpuHybridOnePass;
        tile_direct_output = o.coeff_storage == Options::CoeffStorage::GpuHybridTileDirect;
        retained_int16_levels = fused2d_dispatch &&
            o.coeff_storage == Options::CoeffStorage::Int16;
        // Resolve readback policy before buffers and command buffers are created.
        // record() must see this flag so vkCmdCopyBuffer is actually emitted.
        use_vulkan_readback_copy = persistent_full_output &&
                                   o.readback_mode == Options::ReadbackMode::VulkanCopy;
        use_readback_copy = !tile_direct_output && !fused_mirror_output &&
                            !use_vulkan_readback_copy &&
                            o.readback_mode == Options::ReadbackMode::Copy;
        gpu_levels = o.gpu_levels;
        device_local_input = o.device_local_input;
        device_local_coeff = o.device_local_coeff;
        tile_linear_layout = tile_direct_output && o.tile_layout == Options::TileLayout::ScanLinear;
        tile_flags_u32 = tile_direct_output && o.tile_flag_layout == Options::TileFlagLayout::U32PerTile;
        // Tile-direct shaders may use square 8x8/16x16 geometry or the V22
        // 16x8 occupancy laboratory geometry. Other fused paths remain 8x8.
        // The laboratory workgroup geometry controls every fused 2-D shader.
        // Defaults remain 8x8 for the original full-int16 path, while packed
        // full-frame and tile-direct candidates can request 16x8 or 16x16.
        fused_pairs_x = uint32_t(o.tile_direct_workgroup);
        fused_pairs_y = uint32_t(o.tile_direct_workgroup_y ? o.tile_direct_workgroup_y : o.tile_direct_workgroup);
        fused_pairs_per_thread_x = uint32_t(o.tile_direct_pairs_x);
        fused_pairs_per_thread_y = uint32_t(o.tile_direct_pairs_y);
        gpu_trace = o.gpu_trace && o.fused2d_dispatch;
        planes_in_z = o.planes_in_z;
        if (o.gpu_trace && !o.fused2d_dispatch)
            log_event("WARN","vulkan","--gpu-trace needs --fused2d-dispatch; using frame totals");
        trace_queries = gpu_trace ? uint32_t(trace_intervals_per_frame(o)) + 1u : 2u;
        // Level-major requires every plane to share one level schedule, which
        // plane_specs() guarantees today; fall back rather than assume it.
        {
            const auto schedule_specs = plane_specs(o);
            bool uniform_specs = true;
            for (const auto& ps : schedule_specs)
                uniform_specs = uniform_specs &&
                    ps.width == schedule_specs[0].width &&
                    ps.height == schedule_specs[0].height &&
                    ps.levels == schedule_specs[0].levels;
            dispatch_schedule_level_major =
                o.dispatch_schedule == Options::DispatchSchedule::LevelMajor && uniform_specs;
            if (o.dispatch_schedule == Options::DispatchSchedule::LevelMajor && !uniform_specs)
                log_event("WARN","vulkan","level-major schedule needs uniform plane specs; using plane-major");
        }
        tile_reader_mode = o.tile_reader == Options::TileReader::Legacy ? 0u :
                           (o.tile_reader == Options::TileReader::RowPlan ? 1u :
                           (o.tile_reader == Options::TileReader::Neon ? 2u :
                           (o.tile_reader == Options::TileReader::BandFast ? 3u :
                           (o.tile_reader == Options::TileReader::Group16x8 ? 4u :
                           (o.tile_reader == Options::TileReader::Group16x16 ? 5u : 6u)))));
        if (fused_mirror_output && !fused2d_dispatch)
            throw std::runtime_error("gpu-hybrid-fused requires --fused2d-dispatch");
        if (persistent_full_output && !fused2d_dispatch)
            throw std::runtime_error("persistent full-int16 output requires --fused2d-dispatch");
        if (tile_direct_output && !fused2d_dispatch)
            throw std::runtime_error("gpu-hybrid-tile-direct requires --fused2d-dispatch");
        if (planes_in_z && (!tile_direct_output || !fused2d_dispatch || !dispatch_schedule_level_major))
            throw std::runtime_error("--planes-in-z requires tile-direct fused2d with --dispatch-schedule level-major");
        if (planes_in_z && tile_flags_u32)
            throw std::runtime_error("--planes-in-z and --tile-flag-layout u32 are separate laboratory variables");
        if (tile_direct_output && !tile_linear_layout && tile_reader_mode >= 3u)
            throw std::runtime_error("bandfast/group16x8/group16x16/runspan tile readers require --tile-layout linear");
        // --wide-separable gates. Each of these is an ABI the v101/v102/v103 shaders
        // assume: the tile-direct hybrid output buffers, scan-linear band storage
        // (they index coeff_base + y*band_width + x directly) and the one-bit-per-tile
        // format map. Fail closed rather than corrupting output.
        wide_separable = o.wide_separable;
        wide_hybrid_pack = o.wide_hybrid_pack;
        wide_rows_per_march = uint32_t(o.wide_rows_per_march);
        if (wide_separable) {
            if (!tile_direct_output)
                throw std::runtime_error("--wide-separable requires --coeff-storage gpu-hybrid-tile-direct");
            if (!tile_linear_layout)
                throw std::runtime_error("--wide-separable requires --tile-layout linear");
            if (tile_flags_u32)
                throw std::runtime_error("--wide-separable requires --tile-flag-layout bitmap");
            if (retained_int16_levels || fused_mirror_output || persistent_full_output)
                throw std::runtime_error("--wide-separable is exclusive with the mirror/persistent/retained paths");
            // parse_args already rejects --gpu-levels 0/1 for gpu-hybrid-tile-direct, and
            // --wide-separable requires tile-direct, so 3 is the only reachable value. Assert
            // it rather than silently mis-scheduling if that gate is ever relaxed: with
            // gpu_levels < 3 the final level's LL would be written to the compact final-LL
            // buffer at tile_ll_width, which is far too small to hold an earlier level's LL.
            if (gpu_levels != 3)
                throw std::runtime_error("--wide-separable requires --gpu-levels 3 "
                                         "(the compact final-LL buffer only fits level 3)");
            // v101/v102 batch the four planes in the dispatch Z axis unconditionally,
            // so --planes-in-z is neither needed nor meaningful here. Accept and
            // ignore it rather than rejecting a flag a caller may pass out of habit,
            // but say so, because it changes dispatches_per_frame().
            if (planes_in_z) {
                log_event("INFO","vulkan",
                          "--planes-in-z is implicit in --wide-separable; ignoring the flag");
                planes_in_z = false;
            }
        }
        if (fused2d_dispatch) {
            tiled_dispatch = false;
            if (dispatch_order != "serial") log_event("WARN","vulkan","fused2d dispatch requires serial order; overriding to serial");
            dispatch_order = "serial";
        } else if (tiled_dispatch && dispatch_order == "interleaved") {
            dispatch_order = "serial";
            log_event("WARN","vulkan","tiled dispatch requires serial order; overriding interleaved->serial");
        }
        log_event("INFO","vulkan","init begin");
        specs=plane_specs(o);row_stride=size_t(o.width/2);plane_elems=row_stride*size_t(o.height/2);
        VkApplicationInfo app{VK_STRUCTURE_TYPE_APPLICATION_INFO};app.pApplicationName="cinepi_vc5_bench";app.apiVersion=VK_API_VERSION_1_1;
        VkInstanceCreateInfo ici{VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};ici.pApplicationInfo=&app;
        // On macOS the Vulkan loader (>= 1.3.216) hides portability drivers
        // such as MoltenVK unless the application opts in. Detect the
        // extension at runtime so Linux/V3DV behaviour is unchanged.
        const char* portability_ext = VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME;
        {
            uint32_t ext_n = 0;
            if (vkEnumerateInstanceExtensionProperties(nullptr,&ext_n,nullptr) == VK_SUCCESS && ext_n) {
                std::vector<VkExtensionProperties> exts(ext_n);
                if (vkEnumerateInstanceExtensionProperties(nullptr,&ext_n,exts.data()) == VK_SUCCESS)
                    for (const auto& e : exts)
                        if (std::string(e.extensionName) == portability_ext) {
                            ici.flags |= VK_INSTANCE_CREATE_ENUMERATE_PORTABILITY_BIT_KHR;
                            ici.enabledExtensionCount = 1;
                            ici.ppEnabledExtensionNames = &portability_ext;
                            log_event("INFO","vulkan","portability enumeration enabled (MoltenVK/conversion layer)");
                            break;
                        }
            }
        }
        const VkResult instance_result = vkCreateInstance(&ici,nullptr,&instance);
        if (instance_result != VK_SUCCESS) {
            if (!o.allow_software_vulkan_validation)
                throw std::runtime_error("vkCreateInstance failed with VkResult " + std::to_string(int(instance_result)));
            host_cpu_transform = true; host_options = o; fused2d_dispatch = false; retained_int16_levels = false;
            plane_bytes = plane_elems * sizeof(int16_t); plane_stride_elems = plane_elems; buffer_bytes = plane_bytes * 4u;
            slots.resize(size_t(o.buffers));
            for (auto& slot : slots) { slot.host_ping.assign(plane_stride_elems * 4u, int16_t(0)); slot.mapped_ping = slot.host_ping.data(); }
            mapped_host_cached=true; use_readback_copy=false;
            std::cout << "VULKAN_TRANSFORM backend=host-cpu-validation reason=no-software-icd validation_only=YES\n";
            log_event("WARN","vulkan","no usable software Vulkan ICD; using validation-only host transform");
            return;
        }
        log_event("INFO","vulkan","instance created");
        uint32_t n=0;VK_CHECK(vkEnumeratePhysicalDevices(instance,&n,nullptr));if(!n)throw std::runtime_error("No Vulkan physical device");std::vector<VkPhysicalDevice> devs(n);VK_CHECK(vkEnumeratePhysicalDevices(instance,&n,devs.data()));
        // Vulkan GPU execution is mandatory. Prefer Raspberry Pi V3DV, then
        // another real hardware GPU. CPU/software Vulkan implementations are
        // explicitly rejected because they do not satisfy the CinePi encode
        // requirement and must never be reported as GPU acceleration.
        int best_score = -1;
        bool selected_software = false;
        bool selected_broadcom = false;
        for (auto d : devs) {
            VkPhysicalDeviceProperties candidate{};
            vkGetPhysicalDeviceProperties(d, &candidate);
            std::string name = candidate.deviceName;
            std::string lower = name;
            std::transform(lower.begin(), lower.end(), lower.begin(),
                           [](unsigned char c){ return char(std::tolower(c)); });
            const bool software = candidate.deviceType == VK_PHYSICAL_DEVICE_TYPE_CPU ||
                lower.find("llvmpipe") != std::string::npos ||
                lower.find("lavapipe") != std::string::npos ||
                lower.find("softpipe") != std::string::npos ||
                lower.find("swiftshader") != std::string::npos ||
                lower.find("software") != std::string::npos;
            const bool broadcom = candidate.vendorID == 0x14e4u ||
                lower.find("v3d") != std::string::npos ||
                lower.find("v3dv") != std::string::npos;
            uint32_t qn = 0;
            vkGetPhysicalDeviceQueueFamilyProperties(d, &qn, nullptr);
            std::vector<VkQueueFamilyProperties> qs(qn);
            vkGetPhysicalDeviceQueueFamilyProperties(d, &qn, qs.data());
            for (uint32_t q = 0; q < qn; ++q) {
                if (!(qs[q].queueFlags & VK_QUEUE_COMPUTE_BIT)) continue;
                int score = 0;
                if (broadcom && !software) score = 1000;
                else if (!software && candidate.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU) score = 700;
                else if (!software && candidate.deviceType == VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU) score = 650;
                else if (!software) score = 500;
                else if (software) score = 100;   // Vulkan CPU layer (software ICD, e.g. lavapipe/SwiftShader):
                                                   // lowest priority, so it is selected ONLY when no V3D or other
                                                   // hardware GPU is present -- the automatic fallback. V3D (1000)
                                                   // and any real GPU (>=500) always outrank it.
                else continue;
                if (candidate.vendorID == 0x14e4u) score += 50;
                if (lower.find("v3d") != std::string::npos) score += 25;
                if (score > best_score) {
                    best_score = score;
                    physical = d;
                    family = q;
                    selected_software = software;
                    selected_broadcom = broadcom;
                }
                break;
            }
        }
        if(!physical) throw std::runtime_error("No Vulkan compute device found at all (no V3D, no GPU, and no software Vulkan ICD such as lavapipe). Install a Vulkan driver.");
        if (o.require_v3d && (selected_software || !selected_broadcom))
            throw std::runtime_error("--require-v3d was specified but the selected Vulkan device is not Broadcom V3D/V3DV hardware");
        VkPhysicalDeviceProperties props{};vkGetPhysicalDeviceProperties(physical,&props);
        auto device_type_name = [](VkPhysicalDeviceType t) {
            switch (t) {
                case VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU: return "integrated-gpu";
                case VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU: return "discrete-gpu";
                case VK_PHYSICAL_DEVICE_TYPE_VIRTUAL_GPU: return "virtual-gpu";
                case VK_PHYSICAL_DEVICE_TYPE_CPU: return "cpu";
                default: return "other";
            }
        };
        std::ostringstream selected;
        selected << "VULKAN_DEVICE name=\"" << props.deviceName << "\""
                 << " type=" << device_type_name(props.deviceType)
                 << " vendor=0x" << std::hex << props.vendorID
                 << " device=0x" << props.deviceID << std::dec
                 << " driver_version=" << props.driverVersion
                 << " api=" << VK_VERSION_MAJOR(props.apiVersion) << '.'
                 << VK_VERSION_MINOR(props.apiVersion) << '.'
                 << VK_VERSION_PATCH(props.apiVersion)
                 << " renderer=" << (selected_software ? "software-cpu" : (selected_broadcom ? "v3dv-hardware" : "hardware-gpu"))
                 << " preferred_v3dv=" << (selected_broadcom && !selected_software ? "YES" : "NO")
                 << " fallback=" << (selected_software || !selected_broadcom ? "YES" : "NO")
                 << " validation_only=" << (selected_software ? "YES" : "NO");
        std::cout << selected.str() << '\n';
        std::cout.flush();
        log_event("INFO","vulkan",selected.str());
        {
            std::ostringstream limits;
            limits << "VULKAN_LIMITS"
                   << " max_compute_shared_memory_bytes=" << props.limits.maxComputeSharedMemorySize
                   << " max_compute_workgroup_invocations=" << props.limits.maxComputeWorkGroupInvocations
                   << " max_compute_workgroup_size_x=" << props.limits.maxComputeWorkGroupSize[0]
                   << " max_compute_workgroup_size_y=" << props.limits.maxComputeWorkGroupSize[1]
                   << " max_compute_workgroup_size_z=" << props.limits.maxComputeWorkGroupSize[2]
                   << " max_compute_workgroup_count_x=" << props.limits.maxComputeWorkGroupCount[0]
                   << " max_compute_workgroup_count_y=" << props.limits.maxComputeWorkGroupCount[1]
                   << " max_compute_workgroup_count_z=" << props.limits.maxComputeWorkGroupCount[2]
                   << " max_storage_buffer_range=" << props.limits.maxStorageBufferRange
                   << " min_storage_buffer_offset_alignment=" << props.limits.minStorageBufferOffsetAlignment
                   << " max_push_constants_bytes=" << props.limits.maxPushConstantsSize;
            std::cout << limits.str() << '\n';
            log_event("INFO", "vulkan", limits.str());
        }
        if (sizeof(Push) > props.limits.maxPushConstantsSize)
            throw std::runtime_error("selected shader push constants exceed maxPushConstantsSize");
        if (selected_software) {
            std::cerr << "WARNING: no V3D / hardware Vulkan GPU present -- falling back to the software Vulkan CPU layer (\""
                      << props.deviceName << "\"). This uses the compute shader when required int16 features are present and otherwise reports a host fallback; timings are NOT "
                      << "representative of Pi V3D GPU performance; the software layer exists only as the fallback "
                      << "when V3D Vulkan is absent.\n";
            log_event("WARN","vulkan","V3D absent; using software Vulkan CPU layer fallback (timings not representative of V3D)");
        }
        VkPhysicalDevice16BitStorageFeatures s16{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_16BIT_STORAGE_FEATURES};
        VkPhysicalDeviceFeatures2 f2{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2};f2.pNext=&s16;vkGetPhysicalDeviceFeatures2(physical,&f2);
        log_event("INFO","vulkan","features queried");
        if(!f2.features.shaderInt16 || !s16.storageBuffer16BitAccess) {
            if (!selected_software) throw std::runtime_error(
                "Vulkan device lacks shaderInt16/storageBuffer16BitAccess required by the VC-5 shader. "
                "On Apple Silicon via KosmicKrisp/MoltenVK this is the key feature to confirm: run "
                "`vulkaninfo | grep -E 'shaderInt16|storageBuffer16BitAccess'` and verify both report true. "
                "If they are false the Metal backend cannot run the GPU wavelet path.");
            host_cpu_transform = true;
            host_options = o;
            fused2d_dispatch = false;
            plane_bytes = plane_elems * sizeof(int16_t);
            plane_stride_elems = plane_elems;
            buffer_bytes = plane_bytes * 4u;
            slots.resize(size_t(o.buffers));
            for (auto& slot : slots) {
                slot.host_ping.assign(plane_stride_elems * 4u, int16_t(0));
                slot.mapped_ping = slot.host_ping.data();
            }
            std::cout << "VULKAN_TRANSFORM backend=host-cpu-fallback reason=software-vulkan-lacks-int16\n";
            log_event("WARN","vulkan","software Vulkan lacks shaderInt16/storageBuffer16BitAccess; using host CPU wavelet fallback");
            return;
        }
        uint32_t qn=0;vkGetPhysicalDeviceQueueFamilyProperties(physical,&qn,nullptr);std::vector<VkQueueFamilyProperties> qs(qn);vkGetPhysicalDeviceQueueFamilyProperties(physical,&qn,qs.data());timestamps=qs[family].timestampValidBits>0&&props.limits.timestampPeriod>0;timestamp_ns=props.limits.timestampPeriod;
        const size_t align=std::max<size_t>(2,size_t(props.limits.minStorageBufferOffsetAlignment));
        plane_bytes=((plane_elems*2+align-1)/align)*align;plane_stride_elems=plane_bytes/2;buffer_bytes=plane_bytes*4;
        mirror_plane_bytes=((plane_elems+align-1)/align)*align;mirror_buffer_bytes=mirror_plane_bytes*4;
        if (tile_direct_output) {
            tile_ll_width = row_stride >> 3u;
            tile_ll_height = (size_t(o.height/2)) >> 3u;
            tile_ll_plane_elems = tile_ll_width * tile_ll_height;
            tile_ll_plane_bytes = ((tile_ll_plane_elems * sizeof(int16_t) + align - 1u) / align) * align;
            tile_ll_buffer_bytes = tile_ll_plane_bytes * 4u;
            if (planes_in_z && tile_ll_plane_bytes != tile_ll_plane_elems * sizeof(int16_t))
                throw std::runtime_error("--planes-in-z requires tightly packed final LL planes");
            uint32_t next_tile = 0;
            uint32_t next_coefficient = 0;
            for (uint32_t plane = 0; plane < 4u; ++plane) {
                uint32_t cw = uint32_t(row_stride), ch = uint32_t(o.height/2);
                for (uint32_t level = 0; level < 3u; ++level) {
                    const uint32_t bw = cw >> 1u, bh = ch >> 1u;
                    const uint32_t tx = (bw + 7u) / 8u, ty = (bh + 7u) / 8u;
                    tile_level_tiles_x[level] = tx; tile_level_tiles_y[level] = ty;
                    for (uint32_t band = 0; band < 3u; ++band) {
                        tile_base[plane][level][band] = next_tile;
                        tile_coeff_base[plane][level][band] = next_coefficient;
                        next_tile += tx * ty;
                        next_coefficient += bw * bh;
                    }
                    cw = bw; ch = bh;
                }
            }
            // The shader packs four adjacent int8 coefficients into one uint, so
            // every band row must start 4-aligned and every band width must be a
            // multiple of 4. Otherwise the final partial group of a row writes
            // past the band and, in scan-linear layout, corrupts the next row.
            {
                uint32_t check_coefficient = 0;
                for (uint32_t plane = 0; plane < 4u; ++plane) {
                    uint32_t cw = uint32_t(row_stride), ch = uint32_t(o.height/2);
                    for (uint32_t level = 0; level < 3u; ++level) {
                        const uint32_t bw = cw >> 1u, bh = ch >> 1u;
                        for (uint32_t band = 0; band < 3u; ++band) {
                            if ((bw % 4u) != 0u || (check_coefficient % 4u) != 0u)
                                throw std::runtime_error(
                                    "tile-direct int8 packing needs 4-aligned band rows: plane " +
                                    std::to_string(plane) + " level " + std::to_string(level + 1u) +
                                    " band width " + std::to_string(bw) + " base " +
                                    std::to_string(check_coefficient));
                            check_coefficient += bw * bh;
                        }
                        cw = bw; ch = bh;
                    }
                }
            }
            tile_total_count = next_tile;
            tile_linear_coefficient_count = next_coefficient;
            tile_flag_word_count = (tile_total_count + 31u) / 32u;
            if (tile_linear_layout) {
                tile_i8_buffer_bytes = ((size_t(next_coefficient) + align - 1u) / align) * align;
                tile_i16_buffer_bytes = ((size_t(next_coefficient) * sizeof(int16_t) + align - 1u) / align) * align;
            } else {
                tile_i8_buffer_bytes = size_t(tile_total_count) * 64u;
                tile_i16_buffer_bytes = size_t(tile_total_count) * 128u;
            }
            tile_flags_buffer_bytes = tile_flags_u32
                ? size_t(tile_total_count) * sizeof(uint32_t)
                : size_t(tile_flag_word_count) * sizeof(uint32_t);
        }
        auto require_storage_range = [&](const char* name, size_t bytes) {
            if (bytes > size_t(props.limits.maxStorageBufferRange))
                throw std::runtime_error(std::string(name) + " exceeds maxStorageBufferRange");
        };
        require_storage_range("VC-5 plane descriptor", plane_elems * sizeof(int16_t));
        require_storage_range("VC-5 full coefficient buffer", buffer_bytes);
        if (fused_mirror_output) {
            require_storage_range("Main D int8 mirror", mirror_buffer_bytes);
            require_storage_range("Main D overflow map", 8u);
        }
        if (wide_separable) {
            // v101 writes two line buffers, each one row of width/2 int16 per source
            // row, for all four planes. band_stride is the LEVEL-1 band width and is
            // reused unchanged at every level (levels 2 and 3 simply occupy a prefix of
            // each row), which keeps every row start 16-byte aligned -- a hard
            // requirement for the uvec4 accesses. 960 int16 = 1920 B at production.
            band_stride = row_stride >> 1u;
            band_plane_elems = band_stride * size_t(o.height / 2);
            band_buffer_bytes = band_plane_elems * sizeof(int16_t) * 4u;
            if ((band_stride % 8u) != 0u)
                throw std::runtime_error("--wide-separable needs a band stride that is a "
                                         "multiple of 8 int16 (16 bytes); got " +
                                         std::to_string(band_stride));
            // Every level's band width and every coefficient base must also be
            // 8-element aligned, which is stricter than the 4-element check the
            // int8 packer already enforces below.
            {
                uint32_t check = 0;
                for (uint32_t plane = 0; plane < 4u; ++plane) {
                    uint32_t cw = uint32_t(row_stride), ch = uint32_t(o.height/2);
                    for (uint32_t level = 0; level < 3u; ++level) {
                        const uint32_t bw = cw >> 1u, bh = ch >> 1u;
                        for (uint32_t band = 0; band < 3u; ++band) {
                            if ((bw % 8u) != 0u || (check % 8u) != 0u)
                                throw std::runtime_error(
                                    "--wide-separable needs 8-element-aligned band rows: plane " +
                                    std::to_string(plane) + " level " + std::to_string(level + 1u) +
                                    " band width " + std::to_string(bw) + " base " +
                                    std::to_string(check));
                            check += bw * bh;
                        }
                        cw = bw; ch = bh;
                    }
                }
            }
            if (tile_ll_plane_bytes != tile_ll_plane_elems * sizeof(int16_t))
                throw std::runtime_error("--wide-separable requires tightly packed final LL planes");
            require_storage_range("wide horizontal line buffer", band_buffer_bytes);
        }
        if (tile_direct_output) {
            require_storage_range("Main F compact LL", tile_ll_buffer_bytes);
            require_storage_range("Main F int8 pool", tile_i8_buffer_bytes);
            require_storage_range("Main F int16 pool", tile_i16_buffer_bytes);
            require_storage_range("Main F tile format map", tile_flags_buffer_bytes);
        }
        size_t reserved_slot_bytes = buffer_bytes * 2u;
        if (tile_direct_output) {
            reserved_slot_bytes += tile_ll_buffer_bytes + tile_i8_buffer_bytes +
                                   tile_i16_buffer_bytes + tile_flags_buffer_bytes;
        } else if (fused_mirror_output) {
            reserved_slot_bytes += buffer_bytes + mirror_buffer_bytes + 8u;
        } else if (retained_int16_levels) {
            reserved_slot_bytes = buffer_bytes * 3u;
        } else if (persistent_full_output) {
            reserved_slot_bytes += buffer_bytes;
            if (o.readback_mode == Options::ReadbackMode::VulkanCopy)
                reserved_slot_bytes += buffer_bytes;
        }
        // --wide-separable adds the two GPU-private Hlow/Hhigh line buffers per slot.
        if (wide_separable) reserved_slot_bytes += band_buffer_bytes * 2u;
        std::ostringstream dm;
        dm << "device=" << props.deviceName
           << " api=" << VK_VERSION_MAJOR(props.apiVersion) << '.' << VK_VERSION_MINOR(props.apiVersion)
           << " plane=" << row_stride << 'x' << o.height / 2
           << " plane_stride_bytes=" << plane_bytes
           << " slot_bytes=" << reserved_slot_bytes
           << " slot_mib=" << (double(reserved_slot_bytes) / 1048576.0)
           << " dispatches=" << dispatches_per_frame(o);
        log_event("INFO", "vulkan", dm.str());
        float pri=1;VkDeviceQueueCreateInfo qci{VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};qci.queueFamilyIndex=family;qci.queueCount=1;qci.pQueuePriorities=&pri;
        VkPhysicalDeviceFeatures enabled{};enabled.shaderInt16=VK_TRUE;VkPhysicalDevice16BitStorageFeatures en16{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_16BIT_STORAGE_FEATURES};en16.storageBuffer16BitAccess=VK_TRUE;
        // The Vulkan spec requires enabling VK_KHR_portability_subset on any
        // device that advertises it (MoltenVK does; V3DV does not).
        std::vector<const char*> dev_exts;
        {
            uint32_t dev_ext_n = 0;
            if (vkEnumerateDeviceExtensionProperties(physical,nullptr,&dev_ext_n,nullptr) == VK_SUCCESS && dev_ext_n) {
                std::vector<VkExtensionProperties> exts(dev_ext_n);
                if (vkEnumerateDeviceExtensionProperties(physical,nullptr,&dev_ext_n,exts.data()) == VK_SUCCESS)
                    for (const auto& e : exts)
                        if (std::string(e.extensionName) == CINEPI_VK_PORTABILITY_SUBSET_NAME) {
                            dev_exts.push_back(CINEPI_VK_PORTABILITY_SUBSET_NAME);
                            log_event("INFO","vulkan","enabling VK_KHR_portability_subset");
                            break;
                        }
            }
        }
        VkDeviceCreateInfo dci{VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO};dci.pNext=&en16;dci.queueCreateInfoCount=1;dci.pQueueCreateInfos=&qci;dci.pEnabledFeatures=&enabled;dci.enabledExtensionCount=uint32_t(dev_exts.size());dci.ppEnabledExtensionNames=dev_exts.empty()?nullptr:dev_exts.data();VK_CHECK(vkCreateDevice(physical,&dci,nullptr,&device));vkGetDeviceQueue(device,family,0,&queue);log_event("INFO","vulkan","logical device created");
        const uint32_t binding_count = tile_direct_output ? 6u :
            (fused_mirror_output ? 5u : (persistent_full_output ? 3u : 2u));
        VkDescriptorSetLayoutBinding bind[6]{};
        for(uint32_t i=0;i<binding_count;++i){
            bind[i].binding=i;
            bind[i].descriptorType=VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            bind[i].descriptorCount=1;
            bind[i].stageFlags=VK_SHADER_STAGE_COMPUTE_BIT;
        }
        VkDescriptorSetLayoutCreateInfo dl{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
        dl.bindingCount=binding_count; dl.pBindings=bind;
        VK_CHECK(vkCreateDescriptorSetLayout(device,&dl,nullptr,&dsl));
        VkPushConstantRange pc{}; pc.stageFlags=VK_SHADER_STAGE_COMPUTE_BIT; pc.size=sizeof(Push);
        VkPipelineLayoutCreateInfo pli{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
        pli.setLayoutCount=1; pli.pSetLayouts=&dsl; pli.pushConstantRangeCount=1; pli.pPushConstantRanges=&pc;
        VK_CHECK(vkCreatePipelineLayout(device,&pli,nullptr,&layout));
        log_event("INFO","vulkan","creating compute pipeline");
        auto spv=read_spv(shader_path);
        {
            const SpirvLocalSize declared = spirv_local_size(spv);
            if ((tile_direct_output || retained_int16_levels) && fused2d_dispatch) {
                if (!declared.found)
                    throw std::runtime_error("cannot read workgroup size from " + shader_path);
                if (declared.x * fused_pairs_per_thread_x != fused_pairs_x ||
                    declared.y * fused_pairs_per_thread_y != fused_pairs_y)
                    throw std::runtime_error(
                        "shader " + shader_path + " declares local_size " +
                        std::to_string(declared.x) + "x" + std::to_string(declared.y) +
                        " with " + std::to_string(fused_pairs_per_thread_x) + "x" +
                        std::to_string(fused_pairs_per_thread_y) + " pairs per invocation, " +
                        "which does not cover tile-direct geometry " +
                        std::to_string(fused_pairs_x) + "x" + std::to_string(fused_pairs_y));
                std::cout << "TILE_DIRECT_GEOMETRY workgroup=" << declared.x << 'x' << declared.y
                          << " pairs_per_thread=" << fused_pairs_per_thread_x << 'x'
                          << fused_pairs_per_thread_y
                          << " pairs_per_group=" << fused_pairs_x << 'x' << fused_pairs_y << "\n";
            }
        }
        VkShaderModuleCreateInfo sm{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
        sm.codeSize=spv.size()*4; sm.pCode=spv.data();
        VkShaderModule mod; VK_CHECK(vkCreateShaderModule(device,&sm,nullptr,&mod));
        VkSpecializationMapEntry sme{}; sme.constantID=0; sme.offset=0; sme.size=sizeof(uint32_t);
        uint32_t wg=uint32_t(workgroup_size);
        VkSpecializationInfo si_spec{}; si_spec.mapEntryCount=1; si_spec.pMapEntries=&sme;
        si_spec.dataSize=sizeof(wg); si_spec.pData=&wg;
        VkComputePipelineCreateInfo ci{VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};
        ci.layout=layout; ci.stage.sType=VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        ci.stage.stage=VK_SHADER_STAGE_COMPUTE_BIT; ci.stage.module=mod; ci.stage.pName="main";
        ci.stage.pSpecializationInfo=fused2d_dispatch?nullptr:&si_spec;
        VK_CHECK(vkCreateComputePipelines(device,VK_NULL_HANDLE,1,&ci,nullptr,&pipeline));
        vkDestroyShaderModule(device,mod,nullptr);
        log_event("INFO","vulkan","compute pipeline created");

        if (wide_separable) {
            // The tile-direct descriptor set layout already has exactly the six
            // storage-buffer bindings v101/v102/v103 need, so `dsl` is shared. Only the
            // pipeline layout differs, because WidePushV (84 B) is larger than Push.
            const uint32_t wide_push = uint32_t(sizeof(WidePushV));
            if (wide_push > props.limits.maxPushConstantsSize)
                throw std::runtime_error("--wide-separable push block (" +
                    std::to_string(wide_push) + " B) exceeds maxPushConstantsSize (" +
                    std::to_string(props.limits.maxPushConstantsSize) + " B)");
            VkPushConstantRange wpc{}; wpc.stageFlags=VK_SHADER_STAGE_COMPUTE_BIT; wpc.size=wide_push;
            VkPipelineLayoutCreateInfo wpli{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
            wpli.setLayoutCount=1; wpli.pSetLayouts=&dsl;
            wpli.pushConstantRangeCount=1; wpli.pPushConstantRanges=&wpc;
            VK_CHECK(vkCreatePipelineLayout(device,&wpli,nullptr,&wide_layout));

            // The workgroup size is baked into these shaders at glslang time (-DWG=),
            // so read it back from the SPIR-V rather than assuming: the dispatch count
            // must match it exactly or the shader's own bounds check silently drops work.
            auto make_wide = [&](const std::string& path, const char* what,
                                 VkPipeline& out_pipe, uint32_t& out_wg) {
                auto code = read_spv(path);
                const SpirvLocalSize ls = spirv_local_size(code);
                if (!ls.found || ls.x == 0u)
                    throw std::runtime_error(std::string("cannot read workgroup size from ") + path);
                if (ls.y != 1u || ls.z != 1u)
                    throw std::runtime_error(std::string(what) + " must declare local_size_y/z = 1");
                if ((ls.x % 16u) != 0u)
                    log_event("WARN","vulkan",std::string(what) +
                              " local_size_x is not a multiple of 16; V3D batches are 16 lanes "
                              "so lanes will be wasted");
                out_wg = ls.x;
                VkShaderModuleCreateInfo wsm{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
                wsm.codeSize = code.size()*4; wsm.pCode = code.data();
                VkShaderModule wmod; VK_CHECK(vkCreateShaderModule(device,&wsm,nullptr,&wmod));
                VkComputePipelineCreateInfo wci{VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};
                wci.layout=wide_layout;
                wci.stage.sType=VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
                wci.stage.stage=VK_SHADER_STAGE_COMPUTE_BIT; wci.stage.module=wmod;
                wci.stage.pName="main"; wci.stage.pSpecializationInfo=nullptr;
                VkResult wr = vkCreateComputePipelines(device,VK_NULL_HANDLE,1,&wci,nullptr,&out_pipe);
                vkDestroyShaderModule(device,wmod,nullptr);
                if (wr != VK_SUCCESS)
                    throw std::runtime_error(std::string("could not create ") + what +
                                             " pipeline from " + path +
                                             " VkResult=" + std::to_string(int(wr)));
            };
            make_wide(o.wide_h_shader, "v101 horizontal", wide_h_pipeline, wide_h_wg);
            make_wide(o.wide_v_shader, "v102 vertical",   wide_v_pipeline, wide_v_wg);
            if (wide_hybrid_pack)
                make_wide(o.wide_p_shader, "v103 tile packer", wide_p_pipeline, wide_p_wg);
            std::cout << "WIDE_SEPARABLE h_wg=" << wide_h_wg << " v_wg=" << wide_v_wg
                      << " p_wg=" << wide_p_wg
                      << " rows_per_march=" << wide_rows_per_march
                      << " hybrid_pack=" << (wide_hybrid_pack ? "YES" : "NO")
                      << " band_stride=" << band_stride
                      << " line_buffer_mib=" << (double(band_buffer_bytes)/1048576.0)
                      << "\n";
            log_event("INFO","vulkan","wide-separable pipelines created");
        }

        // The Pi int16 fallback keeps three complete transformed level buffers.
        // Main F/Main D use two LL ping-pong buffers plus their mode-specific
        // persistent coefficient representation.
        const int sets_per_slot = retained_int16_levels ? 12 : (planes_in_z ? 10 : 8);
        // --wide-separable needs 5 more sets per slot, all using the same `dsl`:
        //   0,1 = v101 with ping / pong as input
        //   2,3 = v102 with pong / ping as the next-LL target
        //   4   = v103
        // Two rather than three per shader because level 3 reuses level 1's buffer
        // assignment (input_ping alternates, so odd levels share).
        const int wide_sets_per_slot = wide_separable ? 5 : 0;
        const int total_sets_per_slot = sets_per_slot + wide_sets_per_slot;
        VkDescriptorPoolSize ps{};
        ps.type=VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        ps.descriptorCount=uint32_t(o.buffers*total_sets_per_slot)*binding_count;
        VkDescriptorPoolCreateInfo dpi{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
        dpi.maxSets=uint32_t(o.buffers*total_sets_per_slot); dpi.poolSizeCount=1; dpi.pPoolSizes=&ps;
        VK_CHECK(vkCreateDescriptorPool(device,&dpi,nullptr,&desc_pool));
        VkCommandPoolCreateInfo cpi{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
        cpi.queueFamilyIndex=family; cpi.flags=VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
        VK_CHECK(vkCreateCommandPool(device,&cpi,nullptr,&cmd_pool));
        slots.resize(size_t(o.buffers));
        std::vector<VkDescriptorSetLayout> layouts(size_t(o.buffers*total_sets_per_slot),dsl);
        std::vector<VkDescriptorSet> sets(layouts.size());
        VkDescriptorSetAllocateInfo da{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        da.descriptorPool=desc_pool; da.descriptorSetCount=uint32_t(layouts.size()); da.pSetLayouts=layouts.data();
        VK_CHECK(vkAllocateDescriptorSets(device,&da,sets.data()));
        std::vector<VkCommandBuffer> cmds(size_t(o.buffers));
        VkCommandBufferAllocateInfo ca{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
        ca.commandPool=cmd_pool; ca.level=VK_COMMAND_BUFFER_LEVEL_PRIMARY; ca.commandBufferCount=uint32_t(cmds.size());
        VK_CHECK(vkAllocateCommandBuffers(device,&ca,cmds.data()));

        mapped_host_cached = true;
        for(int si=0;si<o.buffers;++si){
            auto& s=slots[size_t(si)];
            s.cmd=cmds[size_t(si)];
            const size_t set_base = size_t(si*total_sets_per_slot);
            for(int j=0;j<sets_per_slot;++j) s.sets[size_t(j)]=sets[set_base+size_t(j)];
            for(int j=0;j<wide_sets_per_slot;++j)
                s.wide_sets[size_t(j)]=sets[set_base+size_t(sets_per_slot+j)];
            bool ping_cached=false;
            if (device_local_input) {
                create_buffer(buffer_bytes,
                              VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                              VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, 0,
                              s.ping, s.ping_mem, nullptr, &input_memory_flags);
                create_buffer(buffer_bytes, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                              VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                              VK_MEMORY_PROPERTY_HOST_CACHED_BIT,
                              s.upload, s.upload_mem, &ping_cached);
                VK_CHECK(vkMapMemory(device, s.upload_mem, 0, buffer_bytes, 0,
                                     reinterpret_cast<void**>(&s.mapped_ping)));
            } else {
                create_buffer(buffer_bytes,VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                              VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT|VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                              VK_MEMORY_PROPERTY_HOST_CACHED_BIT,s.ping,s.ping_mem,&ping_cached,&input_memory_flags);
                VK_CHECK(vkMapMemory(device,s.ping_mem,0,buffer_bytes,0,reinterpret_cast<void**>(&s.mapped_ping)));
            }

            if(retained_int16_levels){
                bool pong_cached=false,mid_cached=false;
                create_buffer(buffer_bytes,VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                              VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT|VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                              VK_MEMORY_PROPERTY_HOST_CACHED_BIT,s.pong,s.pong_mem,&pong_cached);
                create_buffer(buffer_bytes,VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                              VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT|VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                              VK_MEMORY_PROPERTY_HOST_CACHED_BIT,s.mid,s.mid_mem,&mid_cached);
                VK_CHECK(vkMapMemory(device,s.pong_mem,0,buffer_bytes,0,reinterpret_cast<void**>(&s.mapped_pong)));
                VK_CHECK(vkMapMemory(device,s.mid_mem,0,buffer_bytes,0,reinterpret_cast<void**>(&s.mapped_mid)));
                mapped_host_cached = mapped_host_cached && ping_cached && pong_cached && mid_cached;

                for(int p=0;p<4;++p){
                    for(int step=0;step<3;++step){
                        const int seti=p*3+step;
                        VkBuffer inbuf=step==0?s.ping:(step==1?s.pong:s.mid);
                        VkBuffer outbuf=step==0?s.pong:(step==1?s.mid:s.ping);
                        VkDescriptorBufferInfo bi[2]{};
                        bi[0].buffer=inbuf; bi[1].buffer=outbuf;
                        bi[0].offset=bi[1].offset=VkDeviceSize(size_t(p)*plane_bytes);
                        bi[0].range=bi[1].range=plane_elems*2u;
                        VkWriteDescriptorSet writes[2]{};
                        for(uint32_t b=0;b<2;++b){
                            writes[b].sType=VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
                            writes[b].dstSet=s.sets[size_t(seti)];
                            writes[b].dstBinding=b;
                            writes[b].descriptorCount=1;
                            writes[b].descriptorType=VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
                            writes[b].pBufferInfo=&bi[b];
                        }
                        vkUpdateDescriptorSets(device,2,writes,0,nullptr);
                    }
                }
            } else {
                // Main F/Main D and the ordinary two-pass path use a device-local
                // LL scratch buffer. Their final coefficient storage is allocated
                // separately below when fused2d is active.
                create_buffer(buffer_bytes,VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                              VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,0,s.pong,s.pong_mem);
                bool final_cached=ping_cached;
                if(fused2d_dispatch){
                    const size_t coeff_allocation_bytes = tile_direct_output ? tile_ll_buffer_bytes : buffer_bytes;
                    bool coeff_cached=false;
                    const VkBufferUsageFlags coeff_usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                        ((persistent_full_output && o.readback_mode == Options::ReadbackMode::VulkanCopy)
                            ? VkBufferUsageFlags(VK_BUFFER_USAGE_TRANSFER_SRC_BIT) : VkBufferUsageFlags(0));
                    if (device_local_coeff) {
                        create_buffer(coeff_allocation_bytes, coeff_usage,
                                      VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, 0,
                                      s.coeff, s.coeff_mem, nullptr, &coeff_memory_flags);
                        s.mapped_coeff = nullptr;
                    } else {
                        create_buffer(coeff_allocation_bytes,coeff_usage,
                                      VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT|VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                                      VK_MEMORY_PROPERTY_HOST_CACHED_BIT,s.coeff,s.coeff_mem,&coeff_cached,&coeff_memory_flags);
                        VK_CHECK(vkMapMemory(device,s.coeff_mem,0,coeff_allocation_bytes,0,
                                             reinterpret_cast<void**>(&s.mapped_coeff)));
                    }
                    final_cached=device_local_coeff ? true : coeff_cached;
                    if (persistent_full_output && o.readback_mode == Options::ReadbackMode::VulkanCopy) {
                        bool staging_cached=false;
                        create_buffer(coeff_allocation_bytes, VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                      VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT|VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                                      VK_MEMORY_PROPERTY_HOST_CACHED_BIT,
                                      s.readback_coeff, s.readback_coeff_mem, &staging_cached);
                        VK_CHECK(vkMapMemory(device, s.readback_coeff_mem, 0, coeff_allocation_bytes, 0,
                                             reinterpret_cast<void**>(&s.mapped_readback_coeff)));
                        if (!staging_cached)
                            log_event("WARN", "memory", "Vulkan readback staging allocation is not HOST_CACHED");
                        if (device_local_coeff) final_cached = staging_cached;
                    }
                    if(tile_direct_output){
                        bool i8_cached=false,i16_cached=false,flags_cached=false;
                        create_buffer(tile_i8_buffer_bytes,VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                                      VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT|VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                                      VK_MEMORY_PROPERTY_HOST_CACHED_BIT,s.tile_i8,s.tile_i8_mem,&i8_cached);
                        create_buffer(tile_i16_buffer_bytes,VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                                      VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT|VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                                      VK_MEMORY_PROPERTY_HOST_CACHED_BIT,s.tile_i16,s.tile_i16_mem,&i16_cached);
                        create_buffer(tile_flags_buffer_bytes,VK_BUFFER_USAGE_STORAGE_BUFFER_BIT|VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                      VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT|VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                                      VK_MEMORY_PROPERTY_HOST_CACHED_BIT,s.tile_flags,s.tile_flags_mem,&flags_cached);
                        VK_CHECK(vkMapMemory(device,s.tile_i8_mem,0,tile_i8_buffer_bytes,0,
                                             reinterpret_cast<void**>(&s.mapped_tile_i8)));
                        VK_CHECK(vkMapMemory(device,s.tile_i16_mem,0,tile_i16_buffer_bytes,0,
                                             reinterpret_cast<void**>(&s.mapped_tile_i16)));
                        VK_CHECK(vkMapMemory(device,s.tile_flags_mem,0,tile_flags_buffer_bytes,0,
                                             reinterpret_cast<void**>(&s.mapped_tile_flags)));
                        final_cached=coeff_cached&&i8_cached&&i16_cached&&flags_cached;
                    } else if(fused_mirror_output){
                        bool mirror_cached=false,overflow_cached=false;
                        create_buffer(mirror_buffer_bytes,VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                                      VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT|VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                                      VK_MEMORY_PROPERTY_HOST_CACHED_BIT,s.mirror,s.mirror_mem,&mirror_cached);
                        create_buffer(8u,VK_BUFFER_USAGE_STORAGE_BUFFER_BIT|VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                      VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT|VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                                      VK_MEMORY_PROPERTY_HOST_CACHED_BIT,s.overflow,s.overflow_mem,&overflow_cached);
                        VK_CHECK(vkMapMemory(device,s.mirror_mem,0,mirror_buffer_bytes,0,
                                             reinterpret_cast<void**>(&s.mapped_mirror)));
                        VK_CHECK(vkMapMemory(device,s.overflow_mem,0,8u,0,
                                             reinterpret_cast<void**>(&s.mapped_overflow)));
                        final_cached=coeff_cached&&mirror_cached&&overflow_cached;
                    }
                }
                mapped_host_cached = mapped_host_cached && final_cached;

                for(int p=0;p<4;++p){
                    for(int dir=0;dir<2;++dir){
                        const int seti=p*2+dir;
                        VkDescriptorBufferInfo bi[6]{};
                        bi[0].buffer=dir==0?s.ping:s.pong;
                        bi[1].buffer=dir==0?s.pong:s.ping;
                        for(int b=0;b<2;++b){
                            bi[b].offset=VkDeviceSize(size_t(p)*plane_bytes);
                            bi[b].range=plane_elems*2u;
                        }
                        if(fused2d_dispatch){
                            bi[2].buffer=s.coeff;
                            if(tile_direct_output){
                                bi[2].offset=VkDeviceSize(size_t(p)*tile_ll_plane_bytes);
                                bi[2].range=tile_ll_plane_elems*sizeof(int16_t);
                                bi[3].buffer=s.tile_i8; bi[3].offset=0; bi[3].range=tile_i8_buffer_bytes;
                                bi[4].buffer=s.tile_i16; bi[4].offset=0; bi[4].range=tile_i16_buffer_bytes;
                                bi[5].buffer=s.tile_flags; bi[5].offset=0; bi[5].range=tile_flags_buffer_bytes;
                            } else {
                                bi[2].offset=VkDeviceSize(size_t(p)*plane_bytes);
                                bi[2].range=plane_elems*2u;
                                if(fused_mirror_output){
                                    bi[3].buffer=s.mirror;
                                    bi[3].offset=VkDeviceSize(size_t(p)*mirror_plane_bytes);
                                    bi[3].range=plane_elems;
                                    bi[4].buffer=s.overflow; bi[4].offset=0; bi[4].range=8u;
                                }
                            }
                        }
                        VkWriteDescriptorSet writes[6]{};
                        for(uint32_t b=0;b<binding_count;++b){
                            writes[b].sType=VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
                            writes[b].dstSet=s.sets[size_t(seti)];
                            writes[b].dstBinding=b;
                            writes[b].descriptorCount=1;
                            writes[b].descriptorType=VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
                            writes[b].pBufferInfo=&bi[b];
                        }
                        vkUpdateDescriptorSets(device,binding_count,writes,0,nullptr);
                    }
                }
                if (planes_in_z) {
                    for (int dir=0; dir<2; ++dir) {
                        const int seti=8+dir;
                        VkDescriptorBufferInfo bi[6]{};
                        bi[0].buffer=dir==0?s.ping:s.pong;
                        bi[1].buffer=dir==0?s.pong:s.ping;
                        bi[0].offset=bi[1].offset=0;
                        bi[0].range=bi[1].range=buffer_bytes;
                        bi[2].buffer=s.coeff; bi[2].offset=0; bi[2].range=tile_ll_buffer_bytes;
                        bi[3].buffer=s.tile_i8; bi[3].offset=0; bi[3].range=tile_i8_buffer_bytes;
                        bi[4].buffer=s.tile_i16; bi[4].offset=0; bi[4].range=tile_i16_buffer_bytes;
                        bi[5].buffer=s.tile_flags; bi[5].offset=0; bi[5].range=tile_flags_buffer_bytes;
                        VkWriteDescriptorSet writes[6]{};
                        for(uint32_t b=0;b<binding_count;++b){
                            writes[b].sType=VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
                            writes[b].dstSet=s.sets[size_t(seti)];
                            writes[b].dstBinding=b;
                            writes[b].descriptorCount=1;
                            writes[b].descriptorType=VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
                            writes[b].pBufferInfo=&bi[b];
                        }
                        vkUpdateDescriptorSets(device,binding_count,writes,0,nullptr);
                    }
                }
            }

            if (wide_separable) {
                // The line buffers are GPU-private: v101 writes them, v102 reads them,
                // the CPU never touches them. Ask for DEVICE_LOCAL and do not map.
                // (On V3D that request is a no-op -- V3DV exposes exactly one memory
                // type -- but it is the correct intent and costs nothing.)
                bool ignored=false;
                create_buffer(band_buffer_bytes, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                              VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, 0,
                              s.hlow, s.hlow_mem, &ignored);
                create_buffer(band_buffer_bytes, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                              VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, 0,
                              s.hhigh, s.hhigh_mem, &ignored);

                // Whole-buffer bindings with no per-plane offset: the shaders take the
                // plane index from gl_GlobalInvocationID.z and add plane*stride
                // themselves, exactly as the --planes-in-z sets above do.
                //
                // Unused bindings still need a valid descriptor -- V3DV has no
                // partially-bound descriptor support at Vulkan 1.1 -- so they are
                // pointed at a real buffer the shader simply never declares.
                auto write_wide = [&](size_t seti, std::array<VkBuffer,6> bufs,
                                      std::array<VkDeviceSize,6> ranges) {
                    VkDescriptorBufferInfo bi[6]{};
                    VkWriteDescriptorSet w[6]{};
                    for (uint32_t b=0;b<binding_count;++b) {
                        bi[b].buffer=bufs[b]; bi[b].offset=0; bi[b].range=ranges[b];
                        w[b].sType=VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
                        w[b].dstSet=s.wide_sets[seti];
                        w[b].dstBinding=b;
                        w[b].descriptorCount=1;
                        w[b].descriptorType=VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
                        w[b].pBufferInfo=&bi[b];
                    }
                    vkUpdateDescriptorSets(device,binding_count,w,0,nullptr);
                };
                const VkDeviceSize plane_range = VkDeviceSize(buffer_bytes);
                const VkDeviceSize band_range  = VkDeviceSize(band_buffer_bytes);
                const VkDeviceSize ll_range    = VkDeviceSize(tile_ll_buffer_bytes);
                const VkDeviceSize i8_range    = VkDeviceSize(tile_i8_buffer_bytes);
                const VkDeviceSize i16_range   = VkDeviceSize(tile_i16_buffer_bytes);
                const VkDeviceSize fl_range    = VkDeviceSize(tile_flags_buffer_bytes);

                // v101: 0=src 1=hlow 2=hhigh, 3..5 unused.
                for (int d=0; d<2; ++d)
                    write_wide(size_t(d),
                        {d==0?s.ping:s.pong, s.hlow, s.hhigh, s.hhigh, s.hhigh, s.tile_flags},
                        {plane_range, band_range, band_range, band_range, band_range, fl_range});
                // v102: 0=hlow 1=hhigh 2=next_ll 3=final_ll 4=int16_values 5=format_words.
                // d==0 is the odd-level parity (input ping, next LL to pong).
                for (int d=0; d<2; ++d)
                    write_wide(size_t(2+d),
                        {s.hlow, s.hhigh, d==0?s.pong:s.ping, s.coeff, s.tile_i16, s.tile_flags},
                        {band_range, band_range, plane_range, ll_range, i16_range, fl_range});
                // v103: 0=int16_values 1=int8_words 2=format_words, 3..5 unused.
                write_wide(4u,
                    {s.tile_i16, s.tile_i8, s.tile_flags, s.tile_flags, s.tile_flags, s.tile_flags},
                    {i16_range, i8_range, fl_range, fl_range, fl_range, fl_range});
            }

            VkFenceCreateInfo fi{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
            VK_CHECK(vkCreateFence(device,&fi,nullptr,&s.fence));
            if(timestamps){
                VkQueryPoolCreateInfo qi{VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO};
                qi.queryType=VK_QUERY_TYPE_TIMESTAMP; qi.queryCount=trace_queries;
                VK_CHECK(vkCreateQueryPool(device,&qi,nullptr,&s.query));
            }
            record(s);
        }

        // Hybrid Main F/Main D descriptors point directly at their mode-specific
        // mapped buffers. A full coefficient-copy mode is valid only for int16.
        // Keep --readback-copy as an int16 A/B control and fail closed to direct
        // mapped hybrid reads rather than silently copying the wrong layout.
        if ((tile_direct_output || fused_mirror_output) &&
            (o.readback_mode == Options::ReadbackMode::Copy ||
             o.readback_mode == Options::ReadbackMode::VulkanCopy)) {
            log_event("WARN", "memory",
                      "full cached-copy readback is not applicable to hybrid storage; using direct mapped hybrid buffers");
        }
        log_event("INFO","memory",std::string("mapped_host_cached=")+(mapped_host_cached?"YES":"NO")+
                  " readback="+(use_vulkan_readback_copy?"vulkan-cached-copy":
                    (use_readback_copy?"cpu-cached-copy":"direct-mapped")));
        std::cout << "VULKAN_MEMORY mapped_coefficient_host_cached="
                  << (mapped_host_cached ? "YES" : "NO")
                  << " readback_policy=" << (use_vulkan_readback_copy ? "vulkan-cached-copy" :
                      (use_readback_copy ? "cpu-cached-copy" : "direct-mapped"))
                  << " coefficient_buffer_mib=" << (double(buffer_bytes) / 1048576.0)
                  << " retained_int16_levels=" << (retained_int16_levels ? 3 : 0) << '\n';
    }
    void global_barrier(VkCommandBuffer cmd,VkAccessFlags src,VkAccessFlags dst,VkPipelineStageFlags ss,VkPipelineStageFlags ds){VkMemoryBarrier b{VK_STRUCTURE_TYPE_MEMORY_BARRIER};b.srcAccessMask=src;b.dstAccessMask=dst;vkCmdPipelineBarrier(cmd,ss,ds,0,1,&b,0,nullptr,0,nullptr);}
    void buffer_barrier(VkCommandBuffer cmd,VkBuffer buf,VkDeviceSize off,VkDeviceSize range,VkAccessFlags src,VkAccessFlags dst,VkPipelineStageFlags ss,VkPipelineStageFlags ds){VkBufferMemoryBarrier b{VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER};b.srcAccessMask=src;b.dstAccessMask=dst;b.srcQueueFamilyIndex=VK_QUEUE_FAMILY_IGNORED;b.dstQueueFamilyIndex=VK_QUEUE_FAMILY_IGNORED;b.buffer=buf;b.offset=off;b.size=range;vkCmdPipelineBarrier(cmd,ss,ds,0,0,nullptr,1,&b,0,nullptr);}
    void sync(VkCommandBuffer cmd,VkBuffer buf,VkDeviceSize off,VkDeviceSize range,VkAccessFlags src,VkAccessFlags dst,VkPipelineStageFlags ss,VkPipelineStageFlags ds){if(barrier_scope=="buffer")buffer_barrier(cmd,buf,off,range,src,dst,ss,ds);else global_barrier(cmd,src,dst,ss,ds);}
    void record(Slot& s) {
        VkCommandBufferBeginInfo begin{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
        begin.flags = command_usage == "simultaneous" ? VK_COMMAND_BUFFER_USAGE_SIMULTANEOUS_USE_BIT : 0;
        VK_CHECK(vkBeginCommandBuffer(s.cmd, &begin));
        uint32_t trace_index = 0;
        if (s.query) vkCmdResetQueryPool(s.cmd, s.query, 0, trace_queries);
        if (device_local_input) {
            sync(s.cmd, s.upload, 0, buffer_bytes, VK_ACCESS_HOST_WRITE_BIT,
                 VK_ACCESS_TRANSFER_READ_BIT, VK_PIPELINE_STAGE_HOST_BIT,
                 VK_PIPELINE_STAGE_TRANSFER_BIT);
            VkBufferCopy upload_copy{0, 0, VkDeviceSize(buffer_bytes)};
            vkCmdCopyBuffer(s.cmd, s.upload, s.ping, 1, &upload_copy);
            sync(s.cmd, s.ping, 0, buffer_bytes, VK_ACCESS_TRANSFER_WRITE_BIT,
                 VK_ACCESS_SHADER_READ_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT,
                 VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);
        } else {
            sync(s.cmd, s.ping, 0, buffer_bytes, VK_ACCESS_HOST_WRITE_BIT,
                 VK_ACCESS_SHADER_READ_BIT, VK_PIPELINE_STAGE_HOST_BIT,
                 VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);
        }
        if (fused_mirror_output) {
            vkCmdFillBuffer(s.cmd, s.overflow, 0, 8u, 0u);
            VkMemoryBarrier clear{VK_STRUCTURE_TYPE_MEMORY_BARRIER};
            clear.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
            clear.dstAccessMask = VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT;
            vkCmdPipelineBarrier(s.cmd, VK_PIPELINE_STAGE_TRANSFER_BIT,
                                 VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0,
                                 1, &clear, 0, nullptr, 0, nullptr);
        }
        if (tile_direct_output && !tile_flags_u32) {
            vkCmdFillBuffer(s.cmd, s.tile_flags, 0, tile_flags_buffer_bytes, 0u);
            VkMemoryBarrier clear{VK_STRUCTURE_TYPE_MEMORY_BARRIER};
            clear.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
            clear.dstAccessMask = VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT;
            vkCmdPipelineBarrier(s.cmd, VK_PIPELINE_STAGE_TRANSFER_BIT,
                                 VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0,
                                 1, &clear, 0, nullptr, 0, nullptr);
        }
        if (s.query) vkCmdWriteTimestamp(s.cmd, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, s.query, 0);
        vkCmdBindPipeline(s.cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline);

        const uint32_t wg = uint32_t(workgroup_size);
        auto ngroups = [&](uint32_t axis, uint32_t count) -> uint32_t {
            if (tiled_dispatch) {
                const uint32_t out_pairs = (axis - 2u) / 2u + 1u;
                const uint32_t blocks = (out_pairs + wg - 1u) / wg;
                return count * blocks;
            }
            return uint32_t((size_t(axis / 2u) * size_t(count) + wg - 1u) / wg);
        };

        if (wide_separable) {
            // v1.7.51 wide-separable schedule. Per level: v101 horizontal into the
            // hlow/hhigh line buffers, barrier, v102 vertical into the next LL plus the
            // int16 coefficient frame and the tile-format map. Then, once, v103 packs
            // every non-overflowing 8x8 tile down to int8 and clears its fallback bit.
            //
            // Both wavelet dispatches batch the four colour planes in the Z axis, so
            // there are only 2 dispatches per level and 2 barriers per level, against
            // the fused path's 4 dispatches + 1 barrier (plane-major) or 1 + 1
            // (--planes-in-z). Neither shader uses `shared` memory or barrier(), so
            // V3DV allocates no per-dispatch shared BO at all -- see
            // V1_7_51_V3D_SHARED_MEMORY_ROOT_CAUSE.md sections 1.1 and 1.2.
            //
            // The output is byte-identical to what the fused tile-direct shader
            // produces, which is why nothing downstream of here changes.
            int cw = specs[0].width;
            int ch = specs[0].height;
            const uint32_t coeff_plane = uint32_t(tile_linear_coefficient_count / 4u);
            const uint32_t tile_plane  = tile_total_count / 4u;

            for (int level = 1; level <= gpu_levels; ++level) {
                const uint32_t level_index = uint32_t(level - 1);
                const bool input_ping = (level & 1) != 0;
                const bool final_level = level == gpu_levels;
                const uint32_t out_stride =
                    final_level ? uint32_t(tile_ll_width) : uint32_t(row_stride);
                const uint32_t band_width = uint32_t(cw / 2);
                const uint32_t groups = (band_width + 3u) / 4u;

                // ---- v101 horizontal -------------------------------------------
                WidePushH ph{};
                ph.width = uint32_t(cw);
                ph.height = uint32_t(ch);
                ph.stride = uint32_t(row_stride);
                ph.band_stride = uint32_t(band_stride);
                ph.plane_stride = uint32_t(plane_stride_elems);
                ph.band_plane = uint32_t(band_plane_elems);
                ph.prescale = prescale[size_t(level - 1)];
                vkCmdBindPipeline(s.cmd, VK_PIPELINE_BIND_POINT_COMPUTE, wide_h_pipeline);
                vkCmdBindDescriptorSets(s.cmd, VK_PIPELINE_BIND_POINT_COMPUTE, wide_layout,
                                        0, 1, &s.wide_sets[size_t(input_ping ? 0 : 1)], 0, nullptr);
                vkCmdPushConstants(s.cmd, wide_layout, VK_SHADER_STAGE_COMPUTE_BIT,
                                   0, sizeof(ph), &ph);
                vkCmdDispatch(s.cmd, (groups + wide_h_wg - 1u) / wide_h_wg, uint32_t(ch), 4u);

                // hlow/hhigh: written by v101, read by v102.
                // global_barrier rather than sync() deliberately: sync() takes a single
                // VkBuffer, and two distinct buffers change hands here (and three below),
                // so --barrier-scope buffer has nothing meaningful to narrow to. This is
                // exactly what sync() does in its default "global" scope anyway.
                global_barrier(s.cmd, VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT,
                               VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                               VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);

                // ---- v102 vertical ---------------------------------------------
                WidePushV pv{};
                pv.width = uint32_t(cw);
                pv.height = uint32_t(ch);
                pv.band_stride = uint32_t(band_stride);
                pv.band_plane = uint32_t(band_plane_elems);
                pv.out_stride = out_stride;
                pv.ll_plane = uint32_t(plane_stride_elems);
                pv.final_level = final_level ? 1u : 0u;
                pv.q_lh = pack_gpu_quantizer(quant_table[size_t(10 - 3 * level)]);
                pv.q_hl = pack_gpu_quantizer(quant_table[size_t(11 - 3 * level)]);
                pv.q_hh = pack_gpu_quantizer(quant_table[size_t(12 - 3 * level)]);
                pv.coeff_base_lh = tile_coeff_base[0][level_index][0];
                pv.coeff_base_hl = tile_coeff_base[0][level_index][1];
                pv.coeff_base_hh = tile_coeff_base[0][level_index][2];
                pv.coeff_plane = coeff_plane;
                pv.band_width = band_width;
                pv.tile_base_lh = tile_base[0][level_index][0];
                pv.tile_base_hl = tile_base[0][level_index][1];
                pv.tile_base_hh = tile_base[0][level_index][2];
                pv.tiles_x = tile_level_tiles_x[level_index];
                pv.tile_plane = tile_plane;
                pv.rows_per_march = wide_rows_per_march;
                const uint32_t bh = uint32_t(ch / 2);
                const uint32_t span = wide_rows_per_march ? wide_rows_per_march : bh;
                const uint32_t nseg = (bh + span - 1u) / span;
                vkCmdBindPipeline(s.cmd, VK_PIPELINE_BIND_POINT_COMPUTE, wide_v_pipeline);
                vkCmdBindDescriptorSets(s.cmd, VK_PIPELINE_BIND_POINT_COMPUTE, wide_layout,
                                        0, 1, &s.wide_sets[size_t(input_ping ? 2 : 3)], 0, nullptr);
                vkCmdPushConstants(s.cmd, wide_layout, VK_SHADER_STAGE_COMPUTE_BIT,
                                   0, sizeof(pv), &pv);
                vkCmdDispatch(s.cmd, (groups + wide_v_wg - 1u) / wide_v_wg, nseg, 8u);

                if (gpu_trace && s.query) {
                    const uint32_t slot_index = ++trace_index;
                    if (slot_index < trace_queries) {
                        vkCmdWriteTimestamp(s.cmd, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                                            s.query, slot_index);
                        if (trace_labels.size() < size_t(slot_index))
                            trace_labels.emplace_back(-1, level);
                    }
                }

                // The next level reads this level's LL; the last level's writes are
                // consumed by v103 and then by the host.
                global_barrier(s.cmd, VK_ACCESS_SHADER_WRITE_BIT,
                               VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT,
                               VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                               VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);
                cw /= 2;
                ch /= 2;
            }

            // ---- v103 int8/int16 tile packer -----------------------------------
            // One dispatch per (plane, level, band). They are mutually disjoint in the
            // int8 pool and only ever clear distinct bits of the format map, so no
            // barriers are needed between them.
            if (wide_hybrid_pack) {
                vkCmdBindPipeline(s.cmd, VK_PIPELINE_BIND_POINT_COMPUTE, wide_p_pipeline);
                vkCmdBindDescriptorSets(s.cmd, VK_PIPELINE_BIND_POINT_COMPUTE, wide_layout,
                                        0, 1, &s.wide_sets[4], 0, nullptr);
                for (uint32_t plane = 0; plane < 4u; ++plane) {
                    uint32_t pw = uint32_t(row_stride), phh = uint32_t(specs[0].height);
                    for (int level = 1; level <= gpu_levels; ++level) {
                        const uint32_t level_index = uint32_t(level - 1);
                        const uint32_t bw = pw >> 1u, bhh = phh >> 1u;
                        for (uint32_t band = 0; band < 3u; ++band) {
                            WidePushP pp{};
                            pp.band_width = bw;
                            pp.band_height = bhh;
                            pp.coeff_base = tile_coeff_base[plane][level_index][band];
                            pp.tile_base = tile_base[plane][level_index][band];
                            pp.tiles_x = tile_level_tiles_x[level_index];
                            pp.tiles_y = tile_level_tiles_y[level_index];
                            vkCmdPushConstants(s.cmd, wide_layout, VK_SHADER_STAGE_COMPUTE_BIT,
                                               0, sizeof(pp), &pp);
                            const uint32_t tiles = pp.tiles_x * pp.tiles_y;
                            vkCmdDispatch(s.cmd, (tiles + wide_p_wg - 1u) / wide_p_wg, 1u, 1u);
                        }
                        pw = bw; phh = bhh;
                    }
                }
                if (gpu_trace && s.query) {
                    const uint32_t slot_index = ++trace_index;
                    if (slot_index < trace_queries) {
                        vkCmdWriteTimestamp(s.cmd, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                                            s.query, slot_index);
                        if (trace_labels.size() < size_t(slot_index))
                            trace_labels.emplace_back(-1, 0);   // level 0 == the v103 packer
                    }
                }
                global_barrier(s.cmd, VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT,
                               VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                               VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);
            }
        } else if (retained_int16_levels) {
            // Pi fail-safe fused path. Each wavelet level writes to a distinct
            // mapped full-frame allocation so level-one and level-two high-pass
            // coefficients remain available to the Table-17 encoder.
            //
            // This is a third, independent dispatch loop alongside the two
            // fused2d branches below (tile-direct level-major and plane-major).
            // --gpu-trace was originally wired into only those two, so any
            // configuration landing here -- plain int16 storage with
            // --fused2d-dispatch, i.e. the "fused2d" shader / --int16-fallback --
            // recorded zero timestamps while gpu_trace still expected
            // dispatches_per_frame(o) of them, and the end-of-record count
            // check (correctly) threw. Instrumented here to match.
            for (int plane = 0; plane < 4; ++plane) {
                int cw = specs[size_t(plane)].width;
                int ch = specs[size_t(plane)].height;
                const VkDeviceSize plane_offset = VkDeviceSize(size_t(plane) * plane_bytes);
                const VkDeviceSize plane_range = VkDeviceSize(plane_elems * sizeof(int16_t));
                for (int level = 1; level <= specs[size_t(plane)].levels; ++level) {
                    Push push{uint32_t(cw), uint32_t(ch), uint32_t(row_stride),
                              uint32_t(row_stride), 0u, prescale[size_t(level - 1)],
                              pack_gpu_quantizer(quant_table[size_t(10 - 3 * level)]),
                              pack_gpu_quantizer(quant_table[size_t(11 - 3 * level)]),
                              pack_gpu_quantizer(quant_table[size_t(12 - 3 * level)])};
                    const int set_index = plane * 3 + level - 1;
                    vkCmdBindDescriptorSets(s.cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                            layout, 0, 1,
                                            &s.sets[size_t(set_index)], 0, nullptr);
                    vkCmdPushConstants(s.cmd, layout, VK_SHADER_STAGE_COMPUTE_BIT,
                                       0, sizeof(push), &push);
                    vkCmdDispatch(s.cmd,
                                  uint32_t((uint32_t(cw / 2) + fused_pairs_x - 1u) / fused_pairs_x),
                                  uint32_t((uint32_t(ch / 2) + fused_pairs_y - 1u) / fused_pairs_y), 1);
                    if (gpu_trace && s.query) {
                        const uint32_t slot_index = ++trace_index;
                        if (slot_index < trace_queries) {
                            vkCmdWriteTimestamp(s.cmd, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                                                s.query, slot_index);
                            if (trace_labels.size() < size_t(slot_index))
                                trace_labels.emplace_back(plane, level);
                        }
                    }
                    VkBuffer written = level == 1 ? s.pong : (level == 2 ? s.mid : s.ping);
                    sync(s.cmd, written, plane_offset, plane_range,
                         VK_ACCESS_SHADER_WRITE_BIT,
                         level == 3 ? VK_ACCESS_HOST_READ_BIT : VK_ACCESS_SHADER_READ_BIT,
                         VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                         level == 3 ? VK_PIPELINE_STAGE_HOST_BIT : VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);
                    cw /= 2;
                    ch /= 2;
                }
            }
        } else if (fused2d_dispatch && dispatch_schedule_level_major) {
            // V20 level-major schedule. Planes are independent and the ping/pong
            // parity depends only on the level, so issuing all four planes at a
            // level before advancing is bit-identical to the plane-major order.
            // It drops the barrier count from 8 to 2 per frame and merges the
            // small level-2 and level-3 dispatches into single 4x larger ones,
            // which matters on V3D where a 30x17 workgroup grid cannot fill the
            // device on its own.
            int cw = specs[0].width;
            int ch = specs[0].height;
            const int schedule_levels = gpu_levels;
            for (int level = 1; level <= schedule_levels; ++level) {
                const bool input_ping = (level & 1) != 0;
                const bool final_level = level == schedule_levels;
                const uint32_t level_index = uint32_t(level - 1);
                const uint32_t output_stride =
                    (tile_direct_output && final_level) ? uint32_t(tile_ll_width) : uint32_t(row_stride);
                if (planes_in_z) {
                    Push push{uint32_t(cw), uint32_t(ch), uint32_t(row_stride),
                              output_stride, final_level ? 1u : 0u,
                              prescale[size_t(level - 1)],
                              pack_gpu_quantizer(quant_table[size_t(10 - 3 * level)]),
                              pack_gpu_quantizer(quant_table[size_t(11 - 3 * level)]),
                              pack_gpu_quantizer(quant_table[size_t(12 - 3 * level)]),
                              tile_total_count / 4u, uint32_t(level),
                              tile_base[0][level_index][0],
                              tile_base[0][level_index][1],
                              tile_base[0][level_index][2],
                              tile_level_tiles_x[level_index],
                              uint32_t(plane_stride_elems),
                              tile_coeff_base[0][level_index][0],
                              tile_coeff_base[0][level_index][1],
                              tile_coeff_base[0][level_index][2],
                              uint32_t(cw / 2)};
                    const int set_index = 8 + (input_ping ? 0 : 1);
                    vkCmdBindDescriptorSets(s.cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                            layout, 0, 1,
                                            &s.sets[size_t(set_index)], 0, nullptr);
                    vkCmdPushConstants(s.cmd, layout, VK_SHADER_STAGE_COMPUTE_BIT,
                                       0, sizeof(push), &push);
                    vkCmdDispatch(s.cmd,
                                  uint32_t((uint32_t(cw / 2) + fused_pairs_x - 1u) / fused_pairs_x),
                                  uint32_t((uint32_t(ch / 2) + fused_pairs_y - 1u) / fused_pairs_y),
                                  4u);
                    if (gpu_trace && s.query) {
                        const uint32_t slot_index = ++trace_index;
                        if (slot_index < trace_queries) {
                            vkCmdWriteTimestamp(s.cmd, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                                                s.query, slot_index);
                            if (trace_labels.size() < size_t(slot_index))
                                trace_labels.emplace_back(-1, level);
                        }
                    }
                } else {
                for (int plane = 0; plane < 4; ++plane) {
                    Push push{uint32_t(cw), uint32_t(ch), uint32_t(row_stride),
                              output_stride, final_level ? 1u : 0u,
                              prescale[size_t(level - 1)],
                              pack_gpu_quantizer(quant_table[size_t(10 - 3 * level)]),
                              pack_gpu_quantizer(quant_table[size_t(11 - 3 * level)]),
                              pack_gpu_quantizer(quant_table[size_t(12 - 3 * level)]),
                              uint32_t(plane), uint32_t(level),
                              tile_direct_output ? tile_base[uint32_t(plane)][level_index][0] : 0u,
                              tile_direct_output ? tile_base[uint32_t(plane)][level_index][1] : 0u,
                              tile_direct_output ? tile_base[uint32_t(plane)][level_index][2] : 0u,
                              tile_direct_output ? tile_level_tiles_x[level_index] : 0u,
                              tile_direct_output ? uint32_t(tile_ll_width) : 0u,
                              tile_direct_output ? tile_coeff_base[uint32_t(plane)][level_index][0] : 0u,
                              tile_direct_output ? tile_coeff_base[uint32_t(plane)][level_index][1] : 0u,
                              tile_direct_output ? tile_coeff_base[uint32_t(plane)][level_index][2] : 0u,
                              tile_direct_output ? uint32_t(cw / 2) : 0u};
                    const int set_index = plane * 2 + (input_ping ? 0 : 1);
                    vkCmdBindDescriptorSets(s.cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                            layout, 0, 1,
                                            &s.sets[size_t(set_index)], 0, nullptr);
                    vkCmdPushConstants(s.cmd, layout, VK_SHADER_STAGE_COMPUTE_BIT,
                                       0, sizeof(push), &push);
                    vkCmdDispatch(s.cmd,
                                  uint32_t((uint32_t(cw / 2) + fused_pairs_x - 1u) / fused_pairs_x),
                                  uint32_t((uint32_t(ch / 2) + fused_pairs_y - 1u) / fused_pairs_y), 1);
                    if (gpu_trace && s.query) {
                        const uint32_t slot_index = ++trace_index;
                        if (slot_index < trace_queries) {
                            vkCmdWriteTimestamp(s.cmd, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                                                s.query, slot_index);
                            if (trace_labels.size() < size_t(slot_index))
                                trace_labels.emplace_back(plane, level);
                        }
                    }
                }
                }
                if (!final_level) {
                    // One barrier covers every plane's next-level LL write.
                    const VkBuffer next_ll = input_ping ? s.pong : s.ping;
                    sync(s.cmd, next_ll, 0, VkDeviceSize(buffer_bytes),
                         VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT,
                         VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                         VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);
                }
                cw /= 2;
                ch /= 2;
            }
        } else if (fused2d_dispatch) {
            // V19 Main F/Main D. A persistent host-visible representation keeps
            // all final bands while ping/pong carry only the next-level LL.
            for (int plane = 0; plane < 4; ++plane) {
                int cw = specs[size_t(plane)].width;
                int ch = specs[size_t(plane)].height;
                const VkDeviceSize plane_offset = VkDeviceSize(size_t(plane) * plane_bytes);
                const VkDeviceSize plane_range = VkDeviceSize(plane_elems * sizeof(int16_t));
                for (int level = 1; level <= gpu_levels; ++level) {
                    const bool input_ping = (level & 1) != 0;
                    const bool final_level = level == gpu_levels;
                    const uint32_t level_index = uint32_t(level - 1);
                    const uint32_t output_stride =
                        (tile_direct_output && final_level) ? uint32_t(tile_ll_width) : uint32_t(row_stride);
                    Push push{uint32_t(cw), uint32_t(ch), uint32_t(row_stride),
                              output_stride, final_level ? 1u : 0u,
                              prescale[size_t(level - 1)],
                              pack_gpu_quantizer(quant_table[size_t(10 - 3 * level)]),
                              pack_gpu_quantizer(quant_table[size_t(11 - 3 * level)]),
                              pack_gpu_quantizer(quant_table[size_t(12 - 3 * level)]),
                              uint32_t(plane), uint32_t(level),
                              tile_direct_output ? tile_base[uint32_t(plane)][level_index][0] : 0u,
                              tile_direct_output ? tile_base[uint32_t(plane)][level_index][1] : 0u,
                              tile_direct_output ? tile_base[uint32_t(plane)][level_index][2] : 0u,
                              tile_direct_output ? tile_level_tiles_x[level_index] : 0u,
                              tile_direct_output ? uint32_t(tile_ll_width) : 0u,
                              tile_direct_output ? tile_coeff_base[uint32_t(plane)][level_index][0] : 0u,
                              tile_direct_output ? tile_coeff_base[uint32_t(plane)][level_index][1] : 0u,
                              tile_direct_output ? tile_coeff_base[uint32_t(plane)][level_index][2] : 0u,
                              tile_direct_output ? uint32_t(cw / 2) : 0u};
                    const int set_index = plane * 2 + (input_ping ? 0 : 1);
                    vkCmdBindDescriptorSets(s.cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                            layout, 0, 1,
                                            &s.sets[size_t(set_index)], 0, nullptr);
                    vkCmdPushConstants(s.cmd, layout, VK_SHADER_STAGE_COMPUTE_BIT,
                                       0, sizeof(push), &push);
                    vkCmdDispatch(s.cmd,
                                  uint32_t((uint32_t(cw / 2) + fused_pairs_x - 1u) / fused_pairs_x),
                                  uint32_t((uint32_t(ch / 2) + fused_pairs_y - 1u) / fused_pairs_y), 1);
                    if (gpu_trace && s.query) {
                        const uint32_t slot_index = ++trace_index;
                        if (slot_index < trace_queries) {
                            vkCmdWriteTimestamp(s.cmd, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                                                s.query, slot_index);
                            if (trace_labels.size() < size_t(slot_index))
                                trace_labels.emplace_back(plane, level);
                        }
                    }
                    if (!final_level) {
                        const VkBuffer next_ll = input_ping ? s.pong : s.ping;
                        sync(s.cmd, next_ll, plane_offset, plane_range,
                             VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT,
                             VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                             VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);
                    }
                    cw /= 2;
                    ch /= 2;
                }
            }
        } else if (dispatch_order == "interleaved") {
            int cw[4], ch[4];
            int max_levels = 0;
            for (int plane = 0; plane < 4; ++plane) {
                cw[plane] = specs[size_t(plane)].width;
                ch[plane] = specs[size_t(plane)].height;
                max_levels = std::max(max_levels, specs[size_t(plane)].levels);
            }
            for (int level = 1; level <= max_levels; ++level) {
                for (int plane = 0; plane < 4; ++plane) {
                    if (level > specs[size_t(plane)].levels) continue;
                    Push horizontal{uint32_t(cw[plane]), uint32_t(ch[plane]),
                                    uint32_t(row_stride), uint32_t(row_stride), 0u,
                                    prescale[size_t(level - 1)], 0, 0, 0};
                    vkCmdBindDescriptorSets(s.cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                            layout, 0, 1,
                                            &s.sets[size_t(plane * 2)], 0, nullptr);
                    vkCmdPushConstants(s.cmd, layout, VK_SHADER_STAGE_COMPUTE_BIT,
                                       0, sizeof(horizontal), &horizontal);
                    vkCmdDispatch(s.cmd,
                                  uint32_t((size_t(cw[plane] / 2) * size_t(ch[plane]) + wg - 1) / wg),
                                  1, 1);
                }
                sync(s.cmd, s.pong, 0, buffer_bytes, VK_ACCESS_SHADER_WRITE_BIT,
                     VK_ACCESS_SHADER_READ_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                     VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);
                for (int plane = 0; plane < 4; ++plane) {
                    if (level > specs[size_t(plane)].levels) continue;
                    Push vertical{uint32_t(ch[plane]), uint32_t(cw[plane]),
                                  uint32_t(row_stride), uint32_t(row_stride), 1u, 0,
                                  pack_gpu_quantizer(quant_table[size_t(10 - 3 * level)]),
                                  pack_gpu_quantizer(quant_table[size_t(11 - 3 * level)]),
                                  pack_gpu_quantizer(quant_table[size_t(12 - 3 * level)])};
                    vkCmdBindDescriptorSets(s.cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                            layout, 0, 1,
                                            &s.sets[size_t(plane * 2 + 1)], 0, nullptr);
                    vkCmdPushConstants(s.cmd, layout, VK_SHADER_STAGE_COMPUTE_BIT,
                                       0, sizeof(vertical), &vertical);
                    vkCmdDispatch(s.cmd,
                                  uint32_t((size_t(ch[plane] / 2) * size_t(cw[plane]) + wg - 1) / wg),
                                  1, 1);
                    cw[plane] /= 2;
                    ch[plane] /= 2;
                }
                sync(s.cmd, s.ping, 0, buffer_bytes, VK_ACCESS_SHADER_WRITE_BIT,
                     VK_ACCESS_SHADER_READ_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                     VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);
            }
        } else {
            for (int plane = 0; plane < 4; ++plane) {
                int cw = specs[size_t(plane)].width;
                int ch = specs[size_t(plane)].height;
                const VkDeviceSize plane_offset = VkDeviceSize(size_t(plane) * plane_bytes);
                const VkDeviceSize plane_range = VkDeviceSize(plane_elems * sizeof(int16_t));
                for (int level = 1; level <= specs[size_t(plane)].levels; ++level) {
                    Push horizontal{uint32_t(cw), uint32_t(ch), uint32_t(row_stride),
                                    uint32_t(row_stride), 0u,
                                    prescale[size_t(level - 1)], 0, 0, 0};
                    vkCmdBindDescriptorSets(s.cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                            layout, 0, 1,
                                            &s.sets[size_t(plane * 2)], 0, nullptr);
                    vkCmdPushConstants(s.cmd, layout, VK_SHADER_STAGE_COMPUTE_BIT,
                                       0, sizeof(horizontal), &horizontal);
                    vkCmdDispatch(s.cmd, ngroups(uint32_t(cw), uint32_t(ch)), 1, 1);
                    sync(s.cmd, s.pong, plane_offset, plane_range,
                         VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT,
                         VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                         VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);
                    Push vertical{uint32_t(ch), uint32_t(cw), uint32_t(row_stride),
                                  uint32_t(row_stride), 1u, 0,
                                  pack_gpu_quantizer(quant_table[size_t(10 - 3 * level)]),
                                  pack_gpu_quantizer(quant_table[size_t(11 - 3 * level)]),
                                  pack_gpu_quantizer(quant_table[size_t(12 - 3 * level)])};
                    vkCmdBindDescriptorSets(s.cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                            layout, 0, 1,
                                            &s.sets[size_t(plane * 2 + 1)], 0, nullptr);
                    vkCmdPushConstants(s.cmd, layout, VK_SHADER_STAGE_COMPUTE_BIT,
                                       0, sizeof(vertical), &vertical);
                    vkCmdDispatch(s.cmd, ngroups(uint32_t(ch), uint32_t(cw)), 1, 1);
                    sync(s.cmd, s.ping, plane_offset, plane_range,
                         VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT,
                         VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                         VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);
                    cw /= 2;
                    ch /= 2;
                }
            }
        }

        if (s.query && !gpu_trace)
            vkCmdWriteTimestamp(s.cmd, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, s.query, 1);
        else if (s.query && trace_index + 1u != trace_queries)
            throw std::runtime_error("gpu-trace timestamp count mismatch: wrote " +
                std::to_string(trace_index + 1u) + " of " + std::to_string(trace_queries));
        if (retained_int16_levels) {
            sync(s.cmd, s.pong, 0, buffer_bytes, VK_ACCESS_SHADER_WRITE_BIT,
                 VK_ACCESS_HOST_READ_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                 VK_PIPELINE_STAGE_HOST_BIT);
            sync(s.cmd, s.mid, 0, buffer_bytes, VK_ACCESS_SHADER_WRITE_BIT,
                 VK_ACCESS_HOST_READ_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                 VK_PIPELINE_STAGE_HOST_BIT);
            sync(s.cmd, s.ping, 0, buffer_bytes, VK_ACCESS_SHADER_WRITE_BIT,
                 VK_ACCESS_HOST_READ_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                 VK_PIPELINE_STAGE_HOST_BIT);
        } else {
            const VkBuffer final_coeff = fused2d_dispatch ? s.coeff : s.ping;
            const VkDeviceSize final_coeff_bytes = tile_direct_output ? tile_ll_buffer_bytes : buffer_bytes;
            if (use_vulkan_readback_copy) {
                sync(s.cmd, final_coeff, 0, final_coeff_bytes,
                     VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_TRANSFER_READ_BIT,
                     VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT);
                VkBufferCopy copy{0,0,final_coeff_bytes};
                vkCmdCopyBuffer(s.cmd, final_coeff, s.readback_coeff, 1, &copy);
                sync(s.cmd, s.readback_coeff, 0, final_coeff_bytes,
                     VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_HOST_READ_BIT,
                     VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_HOST_BIT);
            } else {
                sync(s.cmd, final_coeff, 0, final_coeff_bytes,
                     VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_HOST_READ_BIT,
                     VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_HOST_BIT);
            }
        }
        if (fused_mirror_output) {
            sync(s.cmd, s.mirror, 0, mirror_buffer_bytes, VK_ACCESS_SHADER_WRITE_BIT,
                 VK_ACCESS_HOST_READ_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                 VK_PIPELINE_STAGE_HOST_BIT);
            sync(s.cmd, s.overflow, 0, 8u, VK_ACCESS_SHADER_WRITE_BIT,
                 VK_ACCESS_HOST_READ_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                 VK_PIPELINE_STAGE_HOST_BIT);
        }
        if (tile_direct_output) {
            sync(s.cmd, s.tile_i8, 0, tile_i8_buffer_bytes, VK_ACCESS_SHADER_WRITE_BIT,
                 VK_ACCESS_HOST_READ_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                 VK_PIPELINE_STAGE_HOST_BIT);
            sync(s.cmd, s.tile_i16, 0, tile_i16_buffer_bytes, VK_ACCESS_SHADER_WRITE_BIT,
                 VK_ACCESS_HOST_READ_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                 VK_PIPELINE_STAGE_HOST_BIT);
            sync(s.cmd, s.tile_flags, 0, tile_flags_buffer_bytes, VK_ACCESS_SHADER_WRITE_BIT,
                 VK_ACCESS_HOST_READ_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                 VK_PIPELINE_STAGE_HOST_BIT);
        }
        VK_CHECK(vkEndCommandBuffer(s.cmd));
    }

    std::array<const int16_t*, MAX_WAVELET_COUNT> fused_level_planes(const Slot& s) const {
        if (!retained_int16_levels || host_cpu_transform || !s.mapped_pong ||
            !s.mapped_mid || !s.mapped_ping)
            throw std::runtime_error("native retained fused level buffers unavailable");
        return {s.mapped_pong, s.mapped_mid, s.mapped_ping};
    }

    void gather_fused_coefficients(const Slot& s, std::vector<int16_t>& packed) const {
        const auto levels = fused_level_planes(s);
        packed = pack_wavelet_level_buffers(host_options, levels,
                                            plane_stride_elems, row_stride);
    }

    bool has_full_int16_coefficients() const { return !tile_direct_output; }
    int16_t* coefficient_ptr(Slot& s) {
        if (retained_int16_levels)
            throw std::runtime_error("retained fused int16 coefficients require encode_levels/gather_fused_coefficients");
        if (use_vulkan_readback_copy) return s.mapped_readback_coeff;
        return fused2d_dispatch ? s.mapped_coeff : s.mapped_ping;
    }
    const int16_t* coefficient_ptr(const Slot& s) const {
        if (retained_int16_levels)
            throw std::runtime_error("retained fused int16 coefficients require encode_levels/gather_fused_coefficients");
        if (use_vulkan_readback_copy) return s.mapped_readback_coeff;
        return fused2d_dispatch ? s.mapped_coeff : s.mapped_ping;
    }
    void submit(Slot&s){
        s.submit=Clock::now();
        if (gpu_levels == 0) { s.gpu_wall_ns=0; s.gpu_exec_ns=0; return; }
        if(host_cpu_transform) { cpu_transform_schedule(host_options, mode_spec, s.mapped_ping, plane_stride_elems, row_stride); s.gpu_wall_ns=elapsed_ns(s.submit); s.gpu_exec_ns=s.gpu_wall_ns; return; }
        VK_CHECK(vkResetFences(device,1,&s.fence));VkSubmitInfo si{VK_STRUCTURE_TYPE_SUBMIT_INFO};si.commandBufferCount=1;si.pCommandBuffers=&s.cmd;VK_CHECK(vkQueueSubmit(queue,1,&si,s.fence));
    }
    // Per-dispatch breakdown, averaged over every frame with a monotonic
    // timestamp sequence. Intervals cover the dispatch plus any barrier issued
    // immediately after it, which is where the schedule cost shows up.
    void report_trace() const {
        if(!gpu_trace||!trace_frames||trace_accum_ns.empty())return;
        double total=0.0;
        for(uint64_t v:trace_accum_ns) total+=double(v);
        total/=double(trace_frames);
        std::cout<<"GPU_TRACE frames="<<trace_frames<<" dispatches="<<trace_accum_ns.size()
                 <<" total_ms="<<(total/1e6)<<"\n";
        std::map<int,double> by_level; std::map<int,double> by_plane;
        for(size_t i=0;i<trace_accum_ns.size();++i){
            const double ms=double(trace_accum_ns[i])/double(trace_frames)/1e6;
            const int plane=i<trace_labels.size()?trace_labels[i].first:-1;
            const int level=i<trace_labels.size()?trace_labels[i].second:-1;
            by_level[level]+=ms; by_plane[plane]+=ms;
            std::cout<<"GPU_TRACE_DISPATCH index="<<i<<" plane="<<plane<<" level="<<level
                     <<" ms="<<ms<<" share_pct="<<(total>0.0?100.0*ms*1e6/total:0.0)<<"\n";
        }
        for(const auto&kv:by_level)
            std::cout<<"GPU_TRACE_LEVEL level="<<kv.first<<" ms="<<kv.second
                     <<" share_pct="<<(total>0.0?100.0*kv.second*1e6/total:0.0)<<"\n";
        for(const auto&kv:by_plane)
            std::cout<<"GPU_TRACE_PLANE plane="<<kv.first<<" ms="<<kv.second
                     <<" share_pct="<<(total>0.0?100.0*kv.second*1e6/total:0.0)<<"\n";
    }

    void wait(Slot&s){
        if(host_cpu_transform || gpu_levels == 0) return;
        VK_CHECK(vkWaitForFences(device,1,&s.fence,VK_TRUE,UINT64_MAX));s.gpu_wall_ns=elapsed_ns(s.submit);s.gpu_exec_ns=-1;
        if(s.query){
            std::vector<uint64_t> t(trace_queries,0ull);
            const VkResult r=vkGetQueryPoolResults(device,s.query,0,trace_queries,
                trace_queries*sizeof(uint64_t),t.data(),sizeof(uint64_t),VK_QUERY_RESULT_64_BIT);
            if(r==VK_SUCCESS&&t.back()>=t.front())
                s.gpu_exec_ns=int64_t(double(t.back()-t.front())*timestamp_ns);
            if(r==VK_SUCCESS&&gpu_trace){
                if(trace_accum_ns.size()+1u!=trace_queries) trace_accum_ns.assign(trace_queries-1u,0ull);
                bool monotonic=true;
                for(size_t i=1;i<t.size();++i) monotonic=monotonic&&t[i]>=t[i-1];
                if(monotonic){
                    for(size_t i=0;i+1<t.size();++i)
                        trace_accum_ns[i]+=uint64_t(double(t[i+1]-t[i])*timestamp_ns);
                    ++trace_frames;
                }
            }
        }
    }
};

struct HybridBandLayout {
    uint32_t plane=0, level=0, band_id=0;
    uint32_t x=0, y=0, width=0, height=0;
    uint32_t max_abs=0, format_bits=16, word_offset=0;
    uint64_t elements=0, logical_bytes=0, aligned_bytes=0;
};

static std::vector<HybridBandLayout> make_hybrid_bands(uint32_t width, uint32_t height) {
    std::vector<HybridBandLayout> bands;
    bands.reserve(40);
    for (uint32_t plane = 0; plane < 4; ++plane) {
        uint32_t cw = width, ch = height;
        for (uint32_t level = 1; level <= 3; ++level) {
            const uint32_t hw = cw / 2, hh = ch / 2;
            bands.push_back({plane,level,1,hw,0,hw,hh});
            bands.push_back({plane,level,2,0,hh,hw,hh});
            bands.push_back({plane,level,3,hw,hh,hw,hh});
            cw = hw; ch = hh;
        }
        bands.push_back({plane,4,0,0,0,cw,ch});
    }
    for (auto& band : bands) band.elements = uint64_t(band.width) * band.height;
    return bands;
}

static void assign_hybrid_formats(std::vector<HybridBandLayout>& bands) {
    uint64_t offset = 0;
    for (auto& band : bands) {
        band.format_bits = (band.band_id != 0 && band.max_abs <= 127u) ? 8u : 16u;
        band.logical_bytes = band.elements * uint64_t(band.format_bits / 8u);
        band.aligned_bytes = (band.logical_bytes + 3u) & ~uint64_t(3u);
        band.word_offset = uint32_t(offset / 4u);
        offset += band.aligned_bytes;
    }
}

class CpuBandHybridProvider {
public:
    struct Metrics {
        int64_t range_ns=0, pack_ns=0;
        uint64_t aligned_bytes=0;
        uint32_t int8_bands=0, fallback_bands=0;
    };

    CpuBandHybridProvider(size_t slot_count, size_t row_stride, size_t plane_elems)
        : row_stride_(row_stride), plane_elems_(plane_elems) {
        slots_.resize(slot_count);
        const uint32_t height = uint32_t(plane_elems_ / row_stride_);
        for (auto& slot : slots_)
            slot.bands = make_hybrid_bands(uint32_t(row_stride_), height);
    }

    void prepare(size_t slot_index, const int16_t* coefficients) {
        if (!coefficients) throw std::runtime_error("CPU hybrid pack has no coefficient source");
        auto& s = slots_.at(slot_index);
        const auto range_start = Clock::now();
        for (auto& band : s.bands) {
            uint32_t max_abs = 0;
            if (band.band_id != 0) {
                const int16_t* plane = coefficients + size_t(band.plane) * plane_elems_;
                for (uint32_t row = 0; row < band.height; ++row) {
                    const int16_t* src = plane + size_t(band.y + row) * row_stride_ + band.x;
                    for (uint32_t col = 0; col < band.width; ++col) {
                        const int value = int(src[col]);
                        const uint32_t magnitude = uint32_t(value < 0 ? -value : value);
                        if (magnitude > max_abs) max_abs = magnitude;
                    }
                }
            }
            band.max_abs = max_abs;
        }
        s.metrics.range_ns = elapsed_ns(range_start);
        assign_hybrid_formats(s.bands);

        uint64_t total_bytes = 0;
        for (const auto& band : s.bands) total_bytes += band.aligned_bytes;
        s.packed.assign(size_t(total_bytes), uint8_t(0));

        const auto pack_start = Clock::now();
        s.metrics.aligned_bytes = total_bytes;
        s.metrics.int8_bands = 0;
        s.metrics.fallback_bands = 0;
        for (const auto& band : s.bands) {
            uint8_t* dst = s.packed.data() + uint64_t(band.word_offset) * 4u;
            const int16_t* plane = coefficients + size_t(band.plane) * plane_elems_;
            if (band.format_bits == 8) {
                ++s.metrics.int8_bands;
                for (uint32_t row = 0; row < band.height; ++row) {
                    const int16_t* src = plane + size_t(band.y + row) * row_stride_ + band.x;
                    int8_t* out = reinterpret_cast<int8_t*>(dst + size_t(row) * band.width);
                    for (uint32_t col = 0; col < band.width; ++col)
                        out[col] = int8_t(src[col]);
                }
            } else {
                if (band.band_id != 0) ++s.metrics.fallback_bands;
                for (uint32_t row = 0; row < band.height; ++row) {
                    const int16_t* src = plane + size_t(band.y + row) * row_stride_ + band.x;
                    std::memcpy(dst + size_t(row) * band.width * sizeof(int16_t), src,
                                size_t(band.width) * sizeof(int16_t));
                }
            }
        }
        s.metrics.pack_ns = elapsed_ns(pack_start);
        build_frame(s);
    }

    const vc5_pretransformed_hybrid_frame& frame(size_t slot_index) const {
        return slots_.at(slot_index).frame;
    }
    const Metrics& metrics(size_t slot_index) const { return slots_.at(slot_index).metrics; }

private:
    struct SlotState {
        std::vector<HybridBandLayout> bands;
        std::vector<uint8_t> packed;
        vc5_pretransformed_hybrid_frame frame{};
        Metrics metrics{};
    };

    size_t row_stride_=0, plane_elems_=0;
    std::vector<SlotState> slots_;

    void build_frame(SlotState& s) {
        s.frame = {};
        const HybridBandLayout* final_ll[4]{};
        for (const auto& band : s.bands) {
            const uint8_t* data = s.packed.data() + uint64_t(band.word_offset) * 4u;
            if (band.band_id == 0 && band.level == 4) final_ll[band.plane] = &band;
            if (band.band_id != 0) {
                auto& view = s.frame.band[band.plane][band.level - 1u][band.band_id];
                view.data = data;
                view.pitch = size_t(band.width) * size_t(band.format_bits / 8u);
                view.storage_bits = uint8_t(band.format_bits);
            }
        }
        for (uint32_t plane = 0; plane < 4; ++plane) {
            if (!final_ll[plane]) throw std::runtime_error("CPU hybrid frame missing final LL band");
            const auto& band = *final_ll[plane];
            const uint8_t* data = s.packed.data() + uint64_t(band.word_offset) * 4u;
            for (uint32_t level = 0; level < 3; ++level) {
                auto& view = s.frame.band[plane][level][0];
                view.data = data;
                view.pitch = size_t(band.width) * sizeof(int16_t);
                view.storage_bits = 16;
            }
        }
    }
};

static inline bool cpu_tile_row_fits_i8(const int16_t* src, uint32_t columns) {
#if CINEPI_HAVE_NEON_WAVELET && defined(__aarch64__)
    if (columns == 8u) {
        const int16x8_t values = vld1q_s16(src);
        const uint16x8_t below = vcltq_s16(values, vdupq_n_s16(-128));
        const uint16x8_t above = vcgtq_s16(values, vdupq_n_s16(127));
        return vmaxvq_u16(vorrq_u16(below, above)) == 0u;
    }
#endif
    for (uint32_t x = 0; x < columns; ++x)
        if (src[x] < -128 || src[x] > 127) return false;
    return true;
}

static inline void cpu_tile_store_i8(int8_t* dst, const int16_t* src,
                                     uint32_t columns) {
#if CINEPI_HAVE_NEON_WAVELET
    if (columns == 8u) {
        vst1_s8(dst, vqmovn_s16(vld1q_s16(src)));
        return;
    }
#endif
    for (uint32_t x = 0; x < columns; ++x) dst[x] = int8_t(src[x]);
}

class CpuTileHybridProvider {
public:
    struct Metrics {
        int64_t range_ns=0, pack_ns=0;
        uint64_t physical_bytes=0;
        uint32_t int8_tiles=0, fallback_tiles=0, total_tiles=0;
    };
    CpuTileHybridProvider(size_t slot_count, uint32_t width, uint32_t height,
                          size_t row_stride, size_t plane_stride)
        : width_(width), height_(height), row_stride_(row_stride), plane_stride_(plane_stride) {
        slots_.resize(slot_count);
        build_layout();
        for (auto& s : slots_) allocate(s);
    }
    void prepare(size_t slot_index, const int16_t* coefficients) {
        if (!coefficients) throw std::runtime_error("CPU tile hybrid has no coefficient source");
        auto& s=slots_.at(slot_index);
        std::fill(s.flags.begin(),s.flags.end(),0u);
        s.metrics={}; s.metrics.total_tiles=total_tiles_;
        const auto range_start=Clock::now();
        for(uint32_t plane=0;plane<4u;++plane){
            uint32_t cw=width_,ch=height_;
            const int16_t* base=coefficients+size_t(plane)*plane_stride_;
            for(uint32_t level=0;level<3u;++level){
                const uint32_t bw=cw>>1u,bh=ch>>1u;
                const uint32_t xs[3]{bw,0u,bw},ys[3]{0u,bh,bh};
                const uint32_t tx=(bw+7u)/8u,ty=(bh+7u)/8u;
                for(uint32_t band=0;band<3u;++band){
                    const uint32_t tbase=tile_base_[plane][level][band];
                    for(uint32_t t=0;t<tx*ty;++t){
                        const uint32_t tile_x=t%tx,tile_y=t/tx;
                        const uint32_t cols=std::min(8u,bw-tile_x*8u);
                        const uint32_t rows=std::min(8u,bh-tile_y*8u);
                        bool fallback=false;
                        for(uint32_t y=0;y<rows&&!fallback;++y){
                            const int16_t* src=base+size_t(ys[band]+tile_y*8u+y)*row_stride_+xs[band]+tile_x*8u;
                            fallback = !cpu_tile_row_fits_i8(src, cols);
                        }
                        s.fallback[tbase+t] = uint8_t(fallback);
                    }
                }
                cw=bw;ch=bh;
            }
        }
        s.metrics.range_ns=elapsed_ns(range_start);
        const auto pack_start=Clock::now();
        for(uint32_t plane=0;plane<4u;++plane){
            uint32_t cw=width_,ch=height_;
            const int16_t* base=coefficients+size_t(plane)*plane_stride_;
            for(uint32_t level=0;level<3u;++level){
                const uint32_t bw=cw>>1u,bh=ch>>1u;
                const uint32_t xs[3]{bw,0u,bw},ys[3]{0u,bh,bh};
                const uint32_t tx=(bw+7u)/8u,ty=(bh+7u)/8u;
                for(uint32_t band=0;band<3u;++band){
                    const uint32_t cbase=coeff_base_[plane][level][band];
                    for(uint32_t t=0;t<tx*ty;++t){
                        const uint32_t global_tile=tile_base_[plane][level][band]+t;
                        const bool fallback=s.fallback[global_tile]!=0u;
                        if(fallback){s.flags[global_tile>>5u]|=1u<<(global_tile&31u);++s.metrics.fallback_tiles;}
                        else ++s.metrics.int8_tiles;
                        const uint32_t tile_x=t%tx,tile_y=t/tx;
                        const uint32_t cols=std::min(8u,bw-tile_x*8u);
                        const uint32_t rows=std::min(8u,bh-tile_y*8u);
                        for(uint32_t y=0;y<rows;++y){
                            const int16_t* src=base+size_t(ys[band]+tile_y*8u+y)*row_stride_+xs[band]+tile_x*8u;
                            const size_t dst=size_t(cbase)+size_t(tile_y*8u+y)*bw+tile_x*8u;
                            if(fallback){std::memcpy(s.i16.data()+dst,src,size_t(cols)*sizeof(int16_t));s.metrics.physical_bytes+=uint64_t(cols)*2u;}
                            else {cpu_tile_store_i8(s.i8.data()+dst,src,cols);s.metrics.physical_bytes+=cols;}
                        }
                    }
                }
                cw=bw;ch=bh;
            }
            for(uint32_t y=0;y<final_ll_height_;++y)
                std::memcpy(s.final_ll.data()+size_t(plane)*final_ll_plane_elems_+size_t(y)*final_ll_width_,
                            base+size_t(y)*row_stride_,size_t(final_ll_width_)*sizeof(int16_t));
            cw>>=0;
        }
        s.metrics.physical_bytes+=uint64_t(final_ll_plane_elems_)*4u*sizeof(int16_t);
        s.metrics.pack_ns=elapsed_ns(pack_start);
        build_frame(s);
    }
    const vc5_pretransformed_hybrid_frame& frame(size_t i)const{return slots_.at(i).frame;}
    const Metrics& metrics(size_t i)const{return slots_.at(i).metrics;}
private:
    struct SlotState{
        std::vector<int8_t> i8;std::vector<int16_t> i16;std::vector<uint32_t> flags;std::vector<int16_t> final_ll;std::vector<uint8_t> fallback;
        vc5_pretransformed_hybrid_frame frame{};
        vc5_pretransformed_tiled_band_view tiled[4][3][4]{};
        Metrics metrics{};
    };
    uint32_t width_=0,height_=0,final_ll_width_=0,final_ll_height_=0,total_tiles_=0,total_coeffs_=0;
    size_t row_stride_=0,plane_stride_=0,final_ll_plane_elems_=0;
    uint32_t tile_base_[4][3][3]{},coeff_base_[4][3][3]{};
    std::vector<SlotState> slots_;
    void build_layout(){
        uint32_t nt=0,nc=0;
        for(uint32_t p=0;p<4u;++p){uint32_t cw=width_,ch=height_;for(uint32_t l=0;l<3u;++l){uint32_t bw=cw>>1u,bh=ch>>1u;uint32_t count=((bw+7u)/8u)*((bh+7u)/8u);for(uint32_t b=0;b<3u;++b){tile_base_[p][l][b]=nt;coeff_base_[p][l][b]=nc;nt+=count;nc+=bw*bh;}cw=bw;ch=bh;}}
        total_tiles_=nt;total_coeffs_=nc;final_ll_width_=width_>>3u;final_ll_height_=height_>>3u;final_ll_plane_elems_=size_t(final_ll_width_)*final_ll_height_;
    }
    void allocate(SlotState& s){s.i8.resize(total_coeffs_);s.i16.resize(total_coeffs_);s.flags.resize((total_tiles_+31u)/32u);s.final_ll.resize(final_ll_plane_elems_*4u);s.fallback.resize(total_tiles_);build_frame(s);}
    void build_frame(SlotState& s){s.frame={};for(uint32_t p=0;p<4u;++p){uint32_t cw=width_,ch=height_;for(uint32_t l=0;l<3u;++l){uint32_t bw=cw>>1u,bh=ch>>1u;uint32_t tx=(bw+7u)/8u,ty=(bh+7u)/8u;for(uint32_t b=1;b<=3u;++b){auto& t=s.tiled[p][l][b];t.int8_tiles=s.i8.data();t.int16_tiles=s.i16.data();t.format_words=s.flags.data();t.tile_base=tile_base_[p][l][b-1u];t.tiles_x=tx;t.tiles_y=ty;t.tile_width=8;t.tile_height=8;t.int8_tile_stride_bytes=64;t.int16_tile_stride_bytes=128;t.coefficient_base=coeff_base_[p][l][b-1u];t.row_stride=bw;t.layout_mode=1;t.reader_mode=6;t.reserved=0;auto& v=s.frame.band[p][l][b];v.data=&t;v.pitch=1;v.storage_bits=24;}cw=bw;ch=bh;}const void* ll=s.final_ll.data()+size_t(p)*final_ll_plane_elems_;for(uint32_t l=0;l<3u;++l){auto& v=s.frame.band[p][l][0];v.data=ll;v.pitch=size_t(final_ll_width_)*sizeof(int16_t);v.storage_bits=16;}}}
};


// v1.14 direct CPU tile-hybrid output. Unlike CpuTileHybridProvider this does
// not scan/repack a completed int16 coefficient frame. Each 8-value high-pass
// row segment is tested as it leaves the vertical wavelet. The common case is
// narrowed directly to int8. If a later row makes that 8x8 tile require int16,
// the already-written int8 rows are widened once (max 56 values) and the tile
// flips permanently to int16. The M5 sample profiles at >99% int8 tiles, so the
// fallback repair is rare while the 15.8 MiB full-frame write/read disappears.
/* wav2 (V24): the wavelet emits the entropy coder's nonzero mask.
 *
 * The register-direct emit already holds every high-pass lane in a vector when
 * it narrows it to int8, so classifying it as zero/non-zero there costs four
 * instructions and one byte store per eight coefficients. Without it the
 * entropy stage has to rediscover the same information by reading all the
 * coefficients back -- which is precisely the work the non-hybrid path avoids
 * with its E8 sidecar, and precisely why direct-hybrid used to have to choose
 * between a cheap wavelet and a cheap entropy stage.
 *
 * CINEPI_WAV_NZMASK=0 turns the emit off at runtime (the view then carries a
 * NULL mask and the reader self-scans exactly as before), so both legs of the
 * A/B live in one binary. This stage is bimodal with code layout, so a
 * cross-build A/B can measure the layout rather than the change. */
static bool cinepi_wav_nzmask_enabled() {
    static const bool on = []{
        const char* v = std::getenv("CINEPI_WAV_NZMASK");
        return !(v && v[0] == '0');
    }();
    return on;
}

class CpuDirectHybridSlot {
public:
    CpuDirectHybridSlot(uint32_t width, uint32_t height)
        : width_(width), height_(height) {
        build_layout();
        i8_.resize(total_coeffs_);
        i16_.resize(total_coeffs_);
        flags_.resize((total_tiles_ + 31u) / 32u);
        final_ll_.resize(final_ll_plane_elems_ * 4u);
        /* One bit per coefficient. Only meaningful when every band width and
           every band base is a multiple of eight, which direct_geometry_ok()
           already guarantees (width_ % 64 == 0 makes the level-3 band width
           width_/8 a multiple of 8, and each base accumulates bw*bh). Off that
           geometry the mask byte boundaries would not line up with band rows,
           so the mask is simply not offered. */
        nz_enabled_ = direct_geometry_ok() && cinepi_wav_nzmask_enabled();
        if (nz_enabled_) nz_.assign((total_coeffs_ + 7u) / 8u, 0u);
        build_frame();
        reset();
    }
    bool nz_mask_enabled() const { return nz_enabled_; }

    void reset() {
        std::fill(flags_.begin(), flags_.end(), 0u);
        fallback_tiles_ = 0;
        rows_written_ = 0;
    }

    V2CompactSink sink() {
        V2CompactSink s{};
        s.ctx = this;
        s.highpass_row = &CpuDirectHybridSlot::highpass_row_cb;
        s.final_ll_row = &CpuDirectHybridSlot::final_ll_row_cb;
        /* wav1: 4K production geometry (every band width a multiple of 8, so
           8-lane groups map 1:1 onto tile columns) takes the register-direct
           emit; anything else keeps the staging+callback path unchanged. */
        s.direct_slot = direct_geometry_ok() ? this : nullptr;
        return s;
    }

    /* ---- wav1: register-direct emit support ----------------------------- */
    struct BandRow {
        int8_t*  i8_row  = nullptr;   // this row's first coefficient, int8 view
        int16_t* i16_row = nullptr;   // same position, int16 fallback view
        uint32_t* flags  = nullptr;   // whole-frame tile format words
        uint8_t* nz_row  = nullptr;   // one bit per coefficient, byte per 8 (may be null)
        uint32_t tbase_row = 0;       // tile index of this row's first tile
        uint32_t bw = 0;
        CpuDirectHybridSlot* owner = nullptr;
        int plane = 0, level = 0, band = 0, row = 0;
    };
    bool direct_geometry_ok() const {
        /* level-3 band width = width_>>3 must still be a multiple of 8 */
        return (width_ % 64u) == 0u && (height_ % 8u) == 0u;
    }
    void get_band_row(int plane, int level, int band, int row, BandRow& br) {
        const uint32_t bw = width_ >> (level + 1);
        const uint32_t tx = (bw + 7u) / 8u;
        const size_t roff = size_t(coeff_base_[plane][level][band - 1]) +
                            size_t(row) * bw;
        br.i8_row  = i8_.data() + roff;
        br.i16_row = i16_.data() + roff;
        br.flags   = flags_.data();
        br.nz_row  = nz_.empty() ? nullptr : nz_.data() + (roff >> 3);
        br.tbase_row = tile_base_[plane][level][band - 1] +
                       (uint32_t(row) / 8u) * tx;
        br.bw = bw;
        br.owner = this; br.plane = plane; br.level = level;
        br.band = band; br.row = row;
    }
    int16_t* final_ll_row_ptr(int plane, int row) {
        return final_ll_.data() + size_t(plane) * final_ll_plane_elems_ +
               size_t(row) * final_ll_width_;
    }
    /* The rare path of the register-direct emit: this 8-column tile column
       just left the int8 range (or its tile already fell back). EXACTLY the
       flip logic of write_highpass_row(), for one segment. Out of line: it
       runs ~7 times per 128,520 tiles on the reference frame. */
    void direct_fallback8(int plane, int level, int band, int row, uint32_t x,
                          const int16_t* vals) {
        const uint32_t bw = width_ >> (level + 1);
        const uint32_t tx = (bw + 7u) / 8u;
        const uint32_t cbase = coeff_base_[plane][level][band - 1];
        const uint32_t tile  = tile_base_[plane][level][band - 1] +
                               (uint32_t(row) / 8u) * tx + (x / 8u);
        const size_t dst = size_t(cbase) + size_t(row) * bw + x;
        if (!tile_fallback(tile)) {
            set_fallback(tile);
            const uint32_t y0 = (uint32_t(row) / 8u) * 8u;
            for (uint32_t yy = y0; yy < uint32_t(row); ++yy) {
                const size_t prev = size_t(cbase) + size_t(yy) * bw + x;
                for (uint32_t k = 0; k < 8u; ++k)
                    i16_[prev + k] = int16_t(i8_[prev + k]);
            }
        }
        std::memcpy(i16_.data() + dst, vals, 8u * sizeof(int16_t));
    }
    /* --------------------------------------------------------------------- */

    const vc5_pretransformed_hybrid_frame& frame() const { return frame_; }
    uint32_t fallback_tiles() const { return fallback_tiles_; }
    uint32_t total_tiles() const { return total_tiles_; }
    uint32_t int8_tiles() const { return total_tiles_ - fallback_tiles_; }
    uint64_t logical_compact_bytes() const {
        // All high-pass samples occupy one byte unless their tile falls back.
        // Widths are multiples of 8 for the 4K production geometry; calculate
        // exact fallback sample count anyway so the diagnostic stays general.
        uint64_t bytes = uint64_t(total_coeffs_);
        for (uint32_t p=0;p<4u;++p) {
            uint32_t cw=width_, ch=height_;
            for (uint32_t l=0;l<3u;++l) {
                const uint32_t bw=cw>>1u, bh=ch>>1u;
                const uint32_t tx=(bw+7u)/8u, ty=(bh+7u)/8u;
                for (uint32_t b=0;b<3u;++b) {
                    const uint32_t base=tile_base_[p][l][b];
                    for (uint32_t t=0;t<tx*ty;++t) if (tile_fallback(base+t)) {
                        const uint32_t x=(t%tx)*8u, y=(t/tx)*8u;
                        bytes += uint64_t(std::min(8u,bw-x))*std::min(8u,bh-y);
                    }
                }
                cw=bw; ch=bh;
            }
        }
        bytes += uint64_t(final_ll_.size()) * sizeof(int16_t);
        bytes += uint64_t(flags_.size()) * sizeof(uint32_t);
        return bytes;
    }

    bool verify_against(const int16_t* full, size_t row_stride, size_t plane_stride) const {
        if (!full) return false;
        for (uint32_t p=0;p<4u;++p) {
            uint32_t cw=width_, ch=height_;
            const int16_t* plane=full+size_t(p)*plane_stride;
            for (uint32_t l=0;l<3u;++l) {
                const uint32_t bw=cw>>1u, bh=ch>>1u;
                const uint32_t xs[3]{bw,0u,bw}, ys[3]{0u,bh,bh};
                const uint32_t tx=(bw+7u)/8u;
                for (uint32_t b=0;b<3u;++b) {
                    const uint32_t cbase=coeff_base_[p][l][b];
                    const uint32_t tbase=tile_base_[p][l][b];
                    for (uint32_t y=0;y<bh;++y) for (uint32_t x=0;x<bw;++x) {
                        const uint32_t tile=tbase+(y/8u)*tx+(x/8u);
                        const size_t di=size_t(cbase)+size_t(y)*bw+x;
                        const int16_t got=tile_fallback(tile) ? i16_[di] : int16_t(i8_[di]);
                        const int16_t ref=plane[size_t(ys[b]+y)*row_stride+xs[b]+x];
                        if (got != ref) return false;
                        /* wav2: the nonzero mask is what the entropy reader
                           trusts instead of re-reading the coefficients, so it
                           is checked against every coefficient, not sampled. */
                        if (!nz_.empty()) {
                            const bool bit=((nz_[di>>3]>>(di&7u))&1u)!=0u;
                            if (bit != (got != 0)) return false;
                        }
                    }
                }
                cw=bw; ch=bh;
            }
            for (uint32_t y=0;y<final_ll_height_;++y)
                for (uint32_t x=0;x<final_ll_width_;++x) {
                    const int16_t got=final_ll_[size_t(p)*final_ll_plane_elems_+size_t(y)*final_ll_width_+x];
                    const int16_t ref=plane[size_t(y)*row_stride+x];
                    if (got != ref) return false;
                }
        }
        return true;
    }

private:
    uint32_t width_=0,height_=0,final_ll_width_=0,final_ll_height_=0,total_tiles_=0,total_coeffs_=0;
    size_t final_ll_plane_elems_=0;
    uint32_t tile_base_[4][3][3]{}, coeff_base_[4][3][3]{};
    std::vector<int8_t> i8_;
    std::vector<int16_t> i16_;
    std::vector<uint8_t> nz_;
    bool nz_enabled_ = false;
    std::vector<uint32_t> flags_;
    std::vector<int16_t> final_ll_;
    vc5_pretransformed_hybrid_frame frame_{};
    vc5_pretransformed_tiled_band_view tiled_[4][3][4]{};
    uint32_t fallback_tiles_=0;
    uint64_t rows_written_=0;

    bool tile_fallback(uint32_t tile) const {
        return (flags_[tile>>5u] & (1u<<(tile&31u))) != 0u;
    }
    void set_fallback(uint32_t tile) {
        const uint32_t mask=1u<<(tile&31u);
        uint32_t& word=flags_[tile>>5u];
        if ((word&mask)==0u) { word|=mask; ++fallback_tiles_; }
    }

    void build_layout() {
        uint32_t nt=0,nc=0;
        for(uint32_t p=0;p<4u;++p){
            uint32_t cw=width_,ch=height_;
            for(uint32_t l=0;l<3u;++l){
                const uint32_t bw=cw>>1u,bh=ch>>1u;
                const uint32_t count=((bw+7u)/8u)*((bh+7u)/8u);
                for(uint32_t b=0;b<3u;++b){
                    tile_base_[p][l][b]=nt; coeff_base_[p][l][b]=nc;
                    nt+=count; nc+=bw*bh;
                }
                cw=bw; ch=bh;
            }
        }
        total_tiles_=nt; total_coeffs_=nc;
        final_ll_width_=width_>>3u; final_ll_height_=height_>>3u;
        final_ll_plane_elems_=size_t(final_ll_width_)*final_ll_height_;
    }

    void build_frame() {
        frame_={};
        for(uint32_t p=0;p<4u;++p){
            uint32_t cw=width_,ch=height_;
            for(uint32_t l=0;l<3u;++l){
                const uint32_t bw=cw>>1u,bh=ch>>1u;
                const uint32_t tx=(bw+7u)/8u,ty=(bh+7u)/8u;
                for(uint32_t b=1;b<=3u;++b){
                    auto& t=tiled_[p][l][b];
                    t.int8_tiles=i8_.data(); t.int16_tiles=i16_.data(); t.format_words=flags_.data();
                    t.tile_base=tile_base_[p][l][b-1u]; t.tiles_x=tx; t.tiles_y=ty;
                    t.tile_width=8; t.tile_height=8; t.int8_tile_stride_bytes=64; t.int16_tile_stride_bytes=128;
                    t.coefficient_base=coeff_base_[p][l][b-1u]; t.row_stride=bw;
                    t.layout_mode=1; t.reader_mode=6; t.reserved=0;
                    t.nonzero_mask = nz_.empty() ? nullptr : nz_.data();
                    auto& v=frame_.band[p][l][b]; v.data=&t; v.pitch=1; v.storage_bits=24;
                }
                cw=bw; ch=bh;
            }
            const void* ll=final_ll_.data()+size_t(p)*final_ll_plane_elems_;
            for(uint32_t l=0;l<3u;++l){
                auto& v=frame_.band[p][l][0]; v.data=ll;
                v.pitch=size_t(final_ll_width_)*sizeof(int16_t); v.storage_bits=16;
            }
        }
    }

    static void highpass_row_cb(void* ctx,int plane,int level,int band,int row,const int16_t* src,int width) {
        static_cast<CpuDirectHybridSlot*>(ctx)->write_highpass_row(plane,level,band,row,src,width);
    }
    static void final_ll_row_cb(void* ctx,int plane,int row,const int16_t* src,int width) {
        static_cast<CpuDirectHybridSlot*>(ctx)->write_final_ll_row(plane,row,src,width);
    }

    void write_highpass_row(int plane,int level,int band,int row,const int16_t* src,int width) {
        if (plane<0||plane>=4||level<0||level>=3||band<1||band>3||!src) throw std::runtime_error("direct hybrid row metadata invalid");
        const uint32_t bw=uint32_t(width);
        const uint32_t tx=(bw+7u)/8u;
        const uint32_t cbase=coeff_base_[plane][level][band-1];
        const uint32_t tbase=tile_base_[plane][level][band-1];
        for(uint32_t x=0;x<bw;x+=8u){
            const uint32_t cols=std::min(8u,bw-x);
            const uint32_t tile=tbase+(uint32_t(row)/8u)*tx+(x/8u);
            const size_t dst=size_t(cbase)+size_t(row)*bw+x;
            if (!nz_.empty()) {
                uint32_t m=0;
                for(uint32_t k=0;k<cols;++k) if (src[x+k]!=0) m |= (1u<<k);
                nz_[dst>>3]=uint8_t(m);
            }
            if (tile_fallback(tile)) {
                std::memcpy(i16_.data()+dst,src+x,size_t(cols)*sizeof(int16_t));
                continue;
            }
            if (cpu_tile_row_fits_i8(src+x,cols)) {
                cpu_tile_store_i8(i8_.data()+dst,src+x,cols);
                continue;
            }
            // First out-of-int8 row for this tile. Earlier rows were exact int8,
            // so widening them reconstructs their int16 values without keeping
            // any tile-sized int16 scratch in L2.
            set_fallback(tile);
            const uint32_t y0=(uint32_t(row)/8u)*8u;
            for(uint32_t yy=y0; yy<uint32_t(row); ++yy){
                const size_t prev=size_t(cbase)+size_t(yy)*bw+x;
                for(uint32_t k=0;k<cols;++k) i16_[prev+k]=int16_t(i8_[prev+k]);
            }
            std::memcpy(i16_.data()+dst,src+x,size_t(cols)*sizeof(int16_t));
        }
        ++rows_written_;
    }

    void write_final_ll_row(int plane,int row,const int16_t* src,int width) {
        if (plane<0||plane>=4||row<0||!src||uint32_t(width)!=final_ll_width_) throw std::runtime_error("direct hybrid LL row metadata invalid");
        std::memcpy(final_ll_.data()+size_t(plane)*final_ll_plane_elems_+size_t(row)*final_ll_width_,
                    src,size_t(width)*sizeof(int16_t));
    }
};

#if CINEPI_HAVE_NEON_WAVELET
/* ===========================================================================
 * wav1: register-direct vertical emit for the production tile-hybrid output.
 *
 * WHAT IT REPLACES. On the compact (direct tile-hybrid) path v2_level_emit
 * used the plain 8-wide cinepi_fused_v_neon8 loop into per-row int16 staging,
 * then write_highpass_row() re-read that staging, re-checked every 8-lane
 * segment for int8 range and narrowed it -- a full extra pass over every
 * high-pass coefficient of the frame (measured 3.9 ms of the 29 ms single-core
 * wavelet), plus the ~10 instructions per 8 lanes the identity-quantised LL
 * spends in cinepi_lp/cinepi_quant/cinepi_sat where one vqaddq_s16 suffices.
 *
 * WHAT THIS DOES. One pass: the vertical filter's quantised lanes are range-
 * checked and narrowed to int8 while still in registers and stored straight
 * into the slot's tile storage. The filter arithmetic is fused_v_from_taps
 * (the shipped kernels, kind resolved once per row); the LL lowpass uses the
 * ll_vqadd identity this file already proves (clamp((int32)a+(int32)b) IS
 * vqaddq_s16, and the low-half quantiser is hard-coded to 1 here).
 *
 * EXACTNESS. Tile semantics are write_highpass_row()'s, segment for segment,
 * left to right: fallback tile -> int16 store; fits -> int8; first miss ->
 * flip tile, widen this tile column's earlier rows from their exact int8
 * values, int16 store. Only the order of BANDS within one output row differs
 * (HL interleaves with LL, then LH/HH), and bands are disjoint storage, so
 * the final state is identical. tools/verify.sh re-proves it end to end.
 * Measured single-core (core 1, quiet, m1 RAW12 4K): 29.07 -> 24.68 ms,
 * instructions 185.3M -> 144.2M, identical fallback-tile count and CRC.
 * ========================================================================= */
/* wav2: the nonzero mask and the int8 range test share ONE cross-lane reduce.
 *
 * The store already had to answer "does any lane leave the int8 range?", which
 * costs a UMAXV. The mask needs "which lanes are non-zero?", which on its own
 * costs a second reduce (CMTST + AND + ADDV). Weighting the two questions
 * differently lets one ADDV answer both: lane i contributes bit i when it is
 * non-zero (max total 255) and 256 when it is out of range (a multiple of 256).
 * The low byte of the sum IS the mask, and sum >= 256 IS "some lane needs the
 * int16 fallback" -- no lane's non-zero bit can ever carry into the high byte.
 * Net cost of the mask over the shipped store: CMTST, two ANDs, an ORR and a
 * byte store, with the same single reduce and the same branch. */
static inline uint16x8_t wav_nz_bit_weights() {
    static const uint16_t kb[8] = {1u, 2u, 4u, 8u, 16u, 32u, 64u, 128u};
    return vld1q_u16(kb);
}

template <bool NZ>
static inline void wav_direct_store8(CpuDirectHybridSlot::BandRow& br,
                                     uint32_t x, int16x8_t v) {
    const uint32_t tile = br.tbase_row + (x >> 3u);
    if (!NZ) {
        if (__builtin_expect((br.flags[tile >> 5u] >> (tile & 31u)) & 1u, 0)) {
            vst1q_s16(br.i16_row + x, v);
            return;
        }
        const uint16x8_t bad = vorrq_u16(vcltq_s16(v, vdupq_n_s16(-128)),
                                         vcgtq_s16(v, vdupq_n_s16(127)));
        if (__builtin_expect(vmaxvq_u16(bad) == 0u, 1)) {
            vst1_s8(br.i8_row + x, vqmovn_s16(v));
            return;
        }
        int16_t tmp0[8];
        vst1q_s16(tmp0, v);
        br.owner->direct_fallback8(br.plane, br.level, br.band, br.row, x, tmp0);
        return;
    }
    const uint16x8_t bad = vorrq_u16(vcltq_s16(v, vdupq_n_s16(-128)),
                                     vcgtq_s16(v, vdupq_n_s16(127)));
    const uint32_t sum = vaddvq_u16(
        vorrq_u16(vandq_u16(vtstq_s16(v, v), wav_nz_bit_weights()),
                  vandq_u16(bad, vdupq_n_u16(256u))));
    br.nz_row[x >> 3u] = uint8_t(sum);
    if (__builtin_expect((br.flags[tile >> 5u] >> (tile & 31u)) & 1u, 0)) {
        vst1q_s16(br.i16_row + x, v);
        return;
    }
    if (__builtin_expect(sum < 256u, 1)) {
        vst1_s8(br.i8_row + x, vqmovn_s16(v));
        return;
    }
    int16_t tmp[8];
    vst1q_s16(tmp, v);
    br.owner->direct_fallback8(br.plane, br.level, br.band, br.row, x, tmp);
}

template <int Kind, bool NZ>
static void v2_direct_emit_row_t(CpuDirectHybridSlot* slot, FusedLevel& L,
                                 int i,
                                 const int16_t* t0, const int16_t* t1,
                                 const int16_t* t2, const int16_t* t3,
                                 const int16_t* t4, const int16_t* t5,
                                 int16_t* ll_next) {
    const cinepi_quant_neon q_lh = cinepi_make_quant(L.q_lh, L.pc_t_lh);
    const cinepi_quant_neon q_hl = cinepi_make_quant(L.q_hl, L.pc_t_hl);
    const cinepi_quant_neon q_hh = cinepi_make_quant(L.q_hh, L.pc_t_hh);
    CpuDirectHybridSlot::BandRow b1, b2, b3;
    slot->get_band_row(L.compact_plane, L.compact_level, 1, i, b1);
    slot->get_band_row(L.compact_plane, L.compact_level, 2, i, b2);
    slot->get_band_row(L.compact_plane, L.compact_level, 3, i, b3);
    const int half = L.half;
    /* Low half: LL (identity quantiser by construction -> vqadd) to the next
       level's row; HL quantised and narrowed into band 2. */
    for (int c = 0; c + 8 <= half; c += 8) {
        const int16x8_t a0 = vld1q_s16(t0 + c), a1 = vld1q_s16(t1 + c);
        const int16x8_t a2 = vld1q_s16(t2 + c), a3 = vld1q_s16(t3 + c);
        const int16x8_t a4 = vld1q_s16(t4 + c), a5 = vld1q_s16(t5 + c);
        cinepi_i32x8 hi;
        if (Kind == 0)      hi = cinepi_hp_top(a0, a1, a2, a3, a4, a5);
        else if (Kind == 2) hi = cinepi_hp_bottom(a0, a1, a2, a3, a4, a5);
        else                hi = cinepi_hp6(a0, a1, a2, a3, a4, a5);
        const int16x8_t lp0 = (Kind == 0) ? a0 : (Kind == 2 ? a4 : a2);
        const int16x8_t lp1 = (Kind == 0) ? a1 : (Kind == 2 ? a5 : a3);
        vst1q_s16(ll_next + c, vqaddq_s16(lp0, lp1));
        hi = cinepi_quant(hi, q_hl);
        wav_direct_store8<NZ>(b2, uint32_t(c), cinepi_sat(hi));
    }
    /* High half: LH into band 1, HH into band 3, both quantised+narrowed.
     *
     * When the ladder has already quantised HH to nothing, interior groups skip
     * the highpass entirely -- see the bound proof in fused_level_init(). Only
     * interior ROWS (Kind 1) and interior COLUMN groups qualify; the first and
     * last of each use the wider edge filter and keep the original path. */
    const bool skip_hh = L.hh_provably_zero && (Kind == 1);
    for (int c = half; c + 8 <= L.cw; c += 8) {
        const int16x8_t a0 = vld1q_s16(t0 + c), a1 = vld1q_s16(t1 + c);
        const int16x8_t a2 = vld1q_s16(t2 + c), a3 = vld1q_s16(t3 + c);
        const int16x8_t a4 = vld1q_s16(t4 + c), a5 = vld1q_s16(t5 + c);
        int16x8_t lov, hiv;
        if (skip_hh && c != half && c + 8 != L.cw) {
            fused_v_from_taps<false, Kind, true>(a0, a1, a2, a3, a4, a5, Kind,
                                                 q_lh, q_hh, &lov, &hiv);
            wav_direct_store8<NZ>(b1, uint32_t(c - half), lov);
            continue;              /* band 3 stays the zeros it already holds */
        }
        fused_v_from_taps<false, Kind>(a0, a1, a2, a3, a4, a5, Kind,
                                       q_lh, q_hh, &lov, &hiv);
        wav_direct_store8<NZ>(b1, uint32_t(c - half), lov);
        wav_direct_store8<NZ>(b3, uint32_t(c - half), hiv);
    }
}

static bool v2_direct_emit_row(void* slot_v, FusedLevel& L, int i, int kind,
                               const int16_t* t0, const int16_t* t1,
                               const int16_t* t2, const int16_t* t3,
                               const int16_t* t4, const int16_t* t5) {
    if ((L.half & 7) != 0 || (L.cw & 7) != 0) return false;
    auto* slot = static_cast<CpuDirectHybridSlot*>(slot_v);
    int16_t* ll_next = L.final_level
        ? slot->final_ll_row_ptr(L.compact_plane, i)
        : L.ll_row.data();
    if (slot->nz_mask_enabled()) {
        if (kind == 0)      v2_direct_emit_row_t<0,true>(slot, L, i, t0,t1,t2,t3,t4,t5, ll_next);
        else if (kind == 2) v2_direct_emit_row_t<2,true>(slot, L, i, t0,t1,t2,t3,t4,t5, ll_next);
        else                v2_direct_emit_row_t<1,true>(slot, L, i, t0,t1,t2,t3,t4,t5, ll_next);
    } else {
        if (kind == 0)      v2_direct_emit_row_t<0,false>(slot, L, i, t0,t1,t2,t3,t4,t5, ll_next);
        else if (kind == 2) v2_direct_emit_row_t<2,false>(slot, L, i, t0,t1,t2,t3,t4,t5, ll_next);
        else                v2_direct_emit_row_t<1,false>(slot, L, i, t0,t1,t2,t3,t4,t5, ll_next);
    }
    return true;
}
#else
static bool v2_direct_emit_row(void*, FusedLevel&, int, int,
                               const int16_t*, const int16_t*, const int16_t*,
                               const int16_t*, const int16_t*, const int16_t*) {
    return false;
}
#endif

static void v2_fused_frame_compact(const Options& o, const ModeSpec* mode,
                                   const uint16_t* src, size_t src_stride_elems,
                                   int crop_x, int crop_y,
                                   const std::vector<uint16_t>& lut,
                                   CpuDirectHybridSlot& out, V2Frame& ctx) {
    const int pw=o.width/2, ph=o.height/2;
    /* One table per component: CAQ. Identical for all four when it is off. */
    const auto quant_caq=cpu_quant_tables_caq(o,mode);
    out.reset();
    V2CompactSink sink=out.sink();
    V2CompactSink* old_sink=g_v2_compact_sink;
    g_v2_compact_sink=&sink;
    try {
        for(int p=0;p<4;++p){
            int w=pw,h=ph;
            for(int level=1;level<=3;++level){
                auto& L=ctx.planes[size_t(p)][size_t(level-1)];
                fused_level_init(L,w,h,level,quant_caq[size_t(p)],nullptr,size_t(pw),p);
                L.compact_output=true; L.compact_plane=p; L.compact_level=level-1;
                w/=2; h/=2;
            }
        }
        if(ctx.split_row.size()<size_t(pw)*4u) ctx.split_row.resize(size_t(pw)*4u);
        int16_t* gs=ctx.split_row.data(); int16_t* rg=gs+pw; int16_t* bg=rg+pw; int16_t* gd=bg+pw;
        const bool gbrg=(o.bayer=="gbrg"); const int sh=o.true_12bit?0:1;
        for(int y=0;y<ph;++y){
            const uint16_t* row0=src+size_t(2*y+crop_y)*src_stride_elems+size_t(crop_x);
            v2_split_row(row0,row0+src_stride_elems,lut.data(),pw,gbrg,gs,rg,bg,gd,sh,4096,SplitKind::Shipped,unsigned(o.src_shift),o.src_byteswap!=0);
            v2_level_push(ctx.planes[0],0,gs,EmitKind::RegDirect,nullptr);
            v2_level_push(ctx.planes[1],0,rg,EmitKind::RegDirect,nullptr);
            v2_level_push(ctx.planes[2],0,bg,EmitKind::RegDirect,nullptr);
            v2_level_push(ctx.planes[3],0,gd,EmitKind::RegDirect,nullptr);
        }
    } catch (...) { g_v2_compact_sink=old_sink; throw; }
    g_v2_compact_sink=old_sink;
}

class GpuHybridPacker {
public:
    struct Metrics {
        int64_t range_ns=0, pack_ns=0;
        uint64_t aligned_bytes=0;
        uint32_t int8_bands=0, fallback_bands=0;
    };

    GpuHybridPacker(VulkanPipeline& vp, const Options& o) : vp_(vp) {
        if (vp_.host_cpu_transform)
            throw std::runtime_error("gpu-hybrid-int8 requires a hardware Vulkan device");
        if (!vp_.fused2d_dispatch)
            throw std::runtime_error("gpu-hybrid-int8 requires fused2d coefficient output");
        create_layouts_and_pipelines(o.int8_range_shader, o.int8_pack_shader);
        create_slots();
        std::cout << "HYBRID_GPU_CONFIG storage=gpu-hybrid-int8 fallback=whole-band-int16"
                  << " range_shader=" << o.int8_range_shader
                  << " pack_shader=" << o.int8_pack_shader
                  << " slots=" << slots_.size() << "\n";
    }

    ~GpuHybridPacker() { cleanup(); }

    GpuHybridPacker(const GpuHybridPacker&) = delete;
    GpuHybridPacker& operator=(const GpuHybridPacker&) = delete;

    void submit_range(size_t slot_index) {
        auto& s = slots_.at(slot_index);
        s.range_start = Clock::now();
        VK_CHECK(vkResetFences(vp_.device, 1, &s.range_fence));
        VkSubmitInfo si{VK_STRUCTURE_TYPE_SUBMIT_INFO};
        si.commandBufferCount = 1;
        si.pCommandBuffers = &s.range_cmd;
        VK_CHECK(vkQueueSubmit(vp_.queue, 1, &si, s.range_fence));
    }

    void wait_range_and_prepare_pack(size_t slot_index) {
        auto& s = slots_.at(slot_index);
        VK_CHECK(vkWaitForFences(vp_.device, 1, &s.range_fence, VK_TRUE, UINT64_MAX));
        s.metrics.range_ns = elapsed_ns(s.range_start);
        uint32_t range_index = 0;
        for (auto& band : s.bands) {
            band.max_abs = band.band_id == 0 ? 0u : s.mapped_max[range_index++];
        }
        assign_hybrid_formats(s.bands);
        build_frame(s);
        record_pack(s);
    }

    void submit_pack(size_t slot_index) {
        auto& s = slots_.at(slot_index);
        s.pack_start = Clock::now();
        VK_CHECK(vkResetFences(vp_.device, 1, &s.pack_fence));
        VkSubmitInfo si{VK_STRUCTURE_TYPE_SUBMIT_INFO};
        si.commandBufferCount = 1;
        si.pCommandBuffers = &s.pack_cmd;
        VK_CHECK(vkQueueSubmit(vp_.queue, 1, &si, s.pack_fence));
    }

    void wait_pack(size_t slot_index) {
        auto& s = slots_.at(slot_index);
        VK_CHECK(vkWaitForFences(vp_.device, 1, &s.pack_fence, VK_TRUE, UINT64_MAX));
        s.metrics.pack_ns = elapsed_ns(s.pack_start);
    }

    const vc5_pretransformed_hybrid_frame& frame(size_t slot_index) const {
        return slots_.at(slot_index).frame;
    }

    const Metrics& metrics(size_t slot_index) const { return slots_.at(slot_index).metrics; }
    const std::vector<HybridBandLayout>& bands(size_t slot_index) const { return slots_.at(slot_index).bands; }
    const uint8_t* packed_data(size_t slot_index) const { return slots_.at(slot_index).mapped_packed; }

private:
    struct SlotResources {
        VkBuffer max_buffer=VK_NULL_HANDLE, packed_buffer=VK_NULL_HANDLE;
        VkDeviceMemory max_memory=VK_NULL_HANDLE, packed_memory=VK_NULL_HANDLE;
        uint32_t* mapped_max=nullptr;
        uint8_t* mapped_packed=nullptr;
        VkDescriptorSet range_set=VK_NULL_HANDLE, pack_set=VK_NULL_HANDLE;
        VkCommandBuffer range_cmd=VK_NULL_HANDLE, pack_cmd=VK_NULL_HANDLE;
        VkFence range_fence=VK_NULL_HANDLE, pack_fence=VK_NULL_HANDLE;
        Clock::time_point range_start{}, pack_start{};
        std::vector<HybridBandLayout> bands;
        vc5_pretransformed_hybrid_frame frame{};
        Metrics metrics{};
    };

    struct RangePush { uint32_t x,y,width,height,stride,plane_offset,band_index,elements; };
    struct PackPush { uint32_t x,y,width,height,stride,plane_offset,word_offset,format_bits,elements; };

    VulkanPipeline& vp_;
    VkDescriptorSetLayout dsl_=VK_NULL_HANDLE;
    VkPipelineLayout range_layout_=VK_NULL_HANDLE, pack_layout_=VK_NULL_HANDLE;
    VkPipeline range_pipeline_=VK_NULL_HANDLE, pack_pipeline_=VK_NULL_HANDLE;
    VkDescriptorPool descriptor_pool_=VK_NULL_HANDLE;
    VkCommandPool command_pool_=VK_NULL_HANDLE;
    std::vector<SlotResources> slots_;

    VkPipeline create_pipeline(const std::string& path, VkPipelineLayout layout) {
        const auto code = read_spv(path);
        VkShaderModuleCreateInfo sm{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
        sm.codeSize = code.size() * sizeof(uint32_t);
        sm.pCode = code.data();
        VkShaderModule module=VK_NULL_HANDLE;
        VK_CHECK(vkCreateShaderModule(vp_.device, &sm, nullptr, &module));
        VkComputePipelineCreateInfo ci{VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};
        ci.layout = layout;
        ci.stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        ci.stage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
        ci.stage.module = module;
        ci.stage.pName = "main";
        VkPipeline pipeline=VK_NULL_HANDLE;
        const VkResult result = vkCreateComputePipelines(vp_.device, VK_NULL_HANDLE, 1, &ci, nullptr, &pipeline);
        vkDestroyShaderModule(vp_.device, module, nullptr);
        VK_CHECK(result);
        return pipeline;
    }

    void create_layouts_and_pipelines(const std::string& range_shader, const std::string& pack_shader) {
        VkDescriptorSetLayoutBinding bindings[2]{};
        for (uint32_t index = 0; index < 2; ++index) {
            bindings[index].binding = index;
            bindings[index].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            bindings[index].descriptorCount = 1;
            bindings[index].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
        }
        VkDescriptorSetLayoutCreateInfo dsl_info{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
        dsl_info.bindingCount = 2;
        dsl_info.pBindings = bindings;
        VK_CHECK(vkCreateDescriptorSetLayout(vp_.device, &dsl_info, nullptr, &dsl_));

        VkPushConstantRange range_pc{VK_SHADER_STAGE_COMPUTE_BIT,0,uint32_t(sizeof(RangePush))};
        VkPipelineLayoutCreateInfo layout_info{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
        layout_info.setLayoutCount = 1;
        layout_info.pSetLayouts = &dsl_;
        layout_info.pushConstantRangeCount = 1;
        layout_info.pPushConstantRanges = &range_pc;
        VK_CHECK(vkCreatePipelineLayout(vp_.device, &layout_info, nullptr, &range_layout_));

        VkPushConstantRange pack_pc{VK_SHADER_STAGE_COMPUTE_BIT,0,uint32_t(sizeof(PackPush))};
        layout_info.pPushConstantRanges = &pack_pc;
        VK_CHECK(vkCreatePipelineLayout(vp_.device, &layout_info, nullptr, &pack_layout_));
        range_pipeline_ = create_pipeline(range_shader, range_layout_);
        pack_pipeline_ = create_pipeline(pack_shader, pack_layout_);
    }

    void update_set(VkDescriptorSet set, VkBuffer source, VkBuffer destination) {
        VkDescriptorBufferInfo info[2]{{source,0,VK_WHOLE_SIZE},{destination,0,VK_WHOLE_SIZE}};
        VkWriteDescriptorSet writes[2]{};
        for (uint32_t index = 0; index < 2; ++index) {
            writes[index].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            writes[index].dstSet = set;
            writes[index].dstBinding = index;
            writes[index].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            writes[index].descriptorCount = 1;
            writes[index].pBufferInfo = &info[index];
        }
        vkUpdateDescriptorSets(vp_.device, 2, writes, 0, nullptr);
    }

    void create_slots() {
        const uint32_t slot_count = uint32_t(vp_.slots.size());
        VkDescriptorPoolSize pool_size{VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, slot_count * 4u};
        VkDescriptorPoolCreateInfo pool_info{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
        pool_info.maxSets = slot_count * 2u;
        pool_info.poolSizeCount = 1;
        pool_info.pPoolSizes = &pool_size;
        VK_CHECK(vkCreateDescriptorPool(vp_.device, &pool_info, nullptr, &descriptor_pool_));

        VkCommandPoolCreateInfo cmd_pool_info{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
        cmd_pool_info.queueFamilyIndex = vp_.family;
        cmd_pool_info.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
        VK_CHECK(vkCreateCommandPool(vp_.device, &cmd_pool_info, nullptr, &command_pool_));

        slots_.resize(slot_count);
        std::vector<VkDescriptorSetLayout> layouts(size_t(slot_count) * 2u, dsl_);
        std::vector<VkDescriptorSet> sets(layouts.size());
        VkDescriptorSetAllocateInfo set_info{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        set_info.descriptorPool = descriptor_pool_;
        set_info.descriptorSetCount = uint32_t(layouts.size());
        set_info.pSetLayouts = layouts.data();
        VK_CHECK(vkAllocateDescriptorSets(vp_.device, &set_info, sets.data()));

        std::vector<VkCommandBuffer> commands(size_t(slot_count) * 2u);
        VkCommandBufferAllocateInfo cmd_info{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
        cmd_info.commandPool = command_pool_;
        cmd_info.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        cmd_info.commandBufferCount = uint32_t(commands.size());
        VK_CHECK(vkAllocateCommandBuffers(vp_.device, &cmd_info, commands.data()));

        for (uint32_t index = 0; index < slot_count; ++index) {
            auto& s = slots_[index];
            s.range_set = sets[size_t(index) * 2u];
            s.pack_set = sets[size_t(index) * 2u + 1u];
            s.range_cmd = commands[size_t(index) * 2u];
            s.pack_cmd = commands[size_t(index) * 2u + 1u];
            s.bands = make_hybrid_bands(uint32_t(vp_.row_stride), uint32_t(vp_.plane_elems / vp_.row_stride));

            bool ignored_cached = false;
            vp_.create_buffer(64u * sizeof(uint32_t),
                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                VK_MEMORY_PROPERTY_HOST_CACHED_BIT,
                s.max_buffer, s.max_memory, &ignored_cached);
            vp_.create_buffer(vp_.buffer_bytes,
                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                VK_MEMORY_PROPERTY_HOST_CACHED_BIT,
                s.packed_buffer, s.packed_memory, &ignored_cached);
            VK_CHECK(vkMapMemory(vp_.device, s.max_memory, 0, VK_WHOLE_SIZE, 0,
                                 reinterpret_cast<void**>(&s.mapped_max)));
            VK_CHECK(vkMapMemory(vp_.device, s.packed_memory, 0, VK_WHOLE_SIZE, 0,
                                 reinterpret_cast<void**>(&s.mapped_packed)));
            update_set(s.range_set, vp_.slots[index].coeff, s.max_buffer);
            update_set(s.pack_set, vp_.slots[index].coeff, s.packed_buffer);
            VkFenceCreateInfo fence_info{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
            VK_CHECK(vkCreateFence(vp_.device, &fence_info, nullptr, &s.range_fence));
            VK_CHECK(vkCreateFence(vp_.device, &fence_info, nullptr, &s.pack_fence));
            record_range(s);
        }
    }

    void record_range(SlotResources& s) {
        VkCommandBufferBeginInfo begin{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
        begin.flags = VK_COMMAND_BUFFER_USAGE_SIMULTANEOUS_USE_BIT;
        VK_CHECK(vkBeginCommandBuffer(s.range_cmd, &begin));
        // The fused transform is submitted immediately before this command buffer.
        // Make its coefficient writes visible to the range shader before scanning.
        VkMemoryBarrier coeff_barrier{VK_STRUCTURE_TYPE_MEMORY_BARRIER};
        coeff_barrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
        coeff_barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
        vkCmdPipelineBarrier(s.range_cmd, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                             VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0,
                             1, &coeff_barrier, 0, nullptr, 0, nullptr);
        vkCmdFillBuffer(s.range_cmd, s.max_buffer, 0, VK_WHOLE_SIZE, 0u);
        VkMemoryBarrier clear_barrier{VK_STRUCTURE_TYPE_MEMORY_BARRIER};
        clear_barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        clear_barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT;
        vkCmdPipelineBarrier(s.range_cmd, VK_PIPELINE_STAGE_TRANSFER_BIT,
                             VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0,
                             1, &clear_barrier, 0, nullptr, 0, nullptr);
        vkCmdBindPipeline(s.range_cmd, VK_PIPELINE_BIND_POINT_COMPUTE, range_pipeline_);
        vkCmdBindDescriptorSets(s.range_cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                range_layout_, 0, 1, &s.range_set, 0, nullptr);
        uint32_t range_index = 0;
        for (const auto& band : s.bands) {
            if (band.band_id == 0) continue;
            RangePush pc{band.x,band.y,band.width,band.height,uint32_t(vp_.row_stride),
                         uint32_t(uint64_t(band.plane) * vp_.plane_stride_elems),
                         range_index++,uint32_t(band.elements)};
            vkCmdPushConstants(s.range_cmd, range_layout_, VK_SHADER_STAGE_COMPUTE_BIT,
                               0, sizeof(pc), &pc);
            vkCmdDispatch(s.range_cmd, (pc.elements + 255u) / 256u, 1, 1);
        }
        VkMemoryBarrier host_barrier{VK_STRUCTURE_TYPE_MEMORY_BARRIER};
        host_barrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
        host_barrier.dstAccessMask = VK_ACCESS_HOST_READ_BIT;
        vkCmdPipelineBarrier(s.range_cmd, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                             VK_PIPELINE_STAGE_HOST_BIT, 0,
                             1, &host_barrier, 0, nullptr, 0, nullptr);
        VK_CHECK(vkEndCommandBuffer(s.range_cmd));
    }

    void build_frame(SlotResources& s) {
        s.frame = {};
        const HybridBandLayout* final_ll[4]{};
        s.metrics.aligned_bytes = 0;
        s.metrics.int8_bands = 0;
        s.metrics.fallback_bands = 0;
        for (const auto& band : s.bands) {
            s.metrics.aligned_bytes += band.aligned_bytes;
            if (band.format_bits == 8) ++s.metrics.int8_bands;
            else if (band.band_id != 0) ++s.metrics.fallback_bands;
            if (band.band_id == 0 && band.level == 4) final_ll[band.plane] = &band;
            if (band.band_id != 0) {
                auto& view = s.frame.band[band.plane][band.level - 1u][band.band_id];
                view.data = s.mapped_packed + uint64_t(band.word_offset) * 4u;
                view.pitch = size_t(band.width) * size_t(band.format_bits / 8u);
                view.storage_bits = uint8_t(band.format_bits);
            }
        }
        for (uint32_t plane = 0; plane < 4; ++plane) {
            if (!final_ll[plane]) throw std::runtime_error("hybrid frame missing final LL band");
            const auto& band = *final_ll[plane];
            for (uint32_t level = 0; level < 3; ++level) {
                auto& view = s.frame.band[plane][level][0];
                view.data = s.mapped_packed + uint64_t(band.word_offset) * 4u;
                view.pitch = size_t(band.width) * sizeof(int16_t);
                view.storage_bits = 16;
            }
        }
    }

    void record_pack(SlotResources& s) {
        VK_CHECK(vkResetCommandBuffer(s.pack_cmd, 0));
        VkCommandBufferBeginInfo begin{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
        begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
        VK_CHECK(vkBeginCommandBuffer(s.pack_cmd, &begin));
        VkMemoryBarrier coeff_barrier{VK_STRUCTURE_TYPE_MEMORY_BARRIER};
        coeff_barrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
        coeff_barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
        vkCmdPipelineBarrier(s.pack_cmd, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                             VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0,
                             1, &coeff_barrier, 0, nullptr, 0, nullptr);
        vkCmdBindPipeline(s.pack_cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pack_pipeline_);
        vkCmdBindDescriptorSets(s.pack_cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                pack_layout_, 0, 1, &s.pack_set, 0, nullptr);
        for (const auto& band : s.bands) {
            PackPush pc{band.x,band.y,band.width,band.height,uint32_t(vp_.row_stride),
                        uint32_t(uint64_t(band.plane) * vp_.plane_stride_elems),
                        band.word_offset,band.format_bits,uint32_t(band.elements)};
            vkCmdPushConstants(s.pack_cmd, pack_layout_, VK_SHADER_STAGE_COMPUTE_BIT,
                               0, sizeof(pc), &pc);
            const uint32_t per_word = band.format_bits == 8 ? 4u : 2u;
            const uint32_t words = (pc.elements + per_word - 1u) / per_word;
            vkCmdDispatch(s.pack_cmd, (words + 255u) / 256u, 1, 1);
        }
        VkMemoryBarrier host_barrier{VK_STRUCTURE_TYPE_MEMORY_BARRIER};
        host_barrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
        host_barrier.dstAccessMask = VK_ACCESS_HOST_READ_BIT;
        vkCmdPipelineBarrier(s.pack_cmd, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                             VK_PIPELINE_STAGE_HOST_BIT, 0,
                             1, &host_barrier, 0, nullptr, 0, nullptr);
        VK_CHECK(vkEndCommandBuffer(s.pack_cmd));
    }

    void cleanup() {
        if (!vp_.device) return;
        vkDeviceWaitIdle(vp_.device);
        for (auto& s : slots_) {
            if (s.mapped_max && s.max_memory) vkUnmapMemory(vp_.device, s.max_memory);
            if (s.mapped_packed && s.packed_memory) vkUnmapMemory(vp_.device, s.packed_memory);
            if (s.range_fence) vkDestroyFence(vp_.device, s.range_fence, nullptr);
            if (s.pack_fence) vkDestroyFence(vp_.device, s.pack_fence, nullptr);
            if (s.max_buffer) vkDestroyBuffer(vp_.device, s.max_buffer, nullptr);
            if (s.packed_buffer) vkDestroyBuffer(vp_.device, s.packed_buffer, nullptr);
            if (s.max_memory) vkFreeMemory(vp_.device, s.max_memory, nullptr);
            if (s.packed_memory) vkFreeMemory(vp_.device, s.packed_memory, nullptr);
        }
        if (range_pipeline_) vkDestroyPipeline(vp_.device, range_pipeline_, nullptr);
        if (pack_pipeline_) vkDestroyPipeline(vp_.device, pack_pipeline_, nullptr);
        if (range_layout_) vkDestroyPipelineLayout(vp_.device, range_layout_, nullptr);
        if (pack_layout_) vkDestroyPipelineLayout(vp_.device, pack_layout_, nullptr);
        if (descriptor_pool_) vkDestroyDescriptorPool(vp_.device, descriptor_pool_, nullptr);
        if (dsl_) vkDestroyDescriptorSetLayout(vp_.device, dsl_, nullptr);
        if (command_pool_) vkDestroyCommandPool(vp_.device, command_pool_, nullptr);
        slots_.clear();
    }
};


struct FixedHybridMetrics {
    int64_t range_ns=0, pack_ns=0;
    uint64_t aligned_bytes=0;
    uint32_t int8_bands=0, fallback_bands=0;
};

struct FixedHybridState {
    std::vector<HybridBandLayout> bands;
    vc5_pretransformed_hybrid_frame frame{};
    FixedHybridMetrics metrics{};
};

static void prepare_fixed_hybrid_state(FixedHybridState& state, VulkanPipeline& vp,
                                       size_t slot_index, const uint8_t* mirror,
                                       const uint32_t* overflow_words) {
    state.frame = {};
    state.metrics.aligned_bytes = 0;
    state.metrics.int8_bands = 0;
    state.metrics.fallback_bands = 0;
    const auto& slot = vp.slots.at(slot_index);
    const uint8_t* coeff_bytes = reinterpret_cast<const uint8_t*>(vp.coefficient_ptr(slot));
    const HybridBandLayout* final_ll[4]{};
    uint32_t highpass_index = 0;
    for (auto& band : state.bands) {
        if (band.band_id != 0) {
            const bool overflow = (overflow_words[highpass_index >> 5u] &
                                   (1u << (highpass_index & 31u))) != 0u;
            band.format_bits = overflow ? 16u : 8u;
            ++highpass_index;
        } else {
            band.format_bits = 16u;
        }
        band.logical_bytes = band.elements * uint64_t(band.format_bits / 8u);
        band.aligned_bytes = (band.logical_bytes + 3u) & ~uint64_t(3u);
        state.metrics.aligned_bytes += band.logical_bytes;
        if (band.band_id != 0) {
            if (band.format_bits == 8u) ++state.metrics.int8_bands;
            else ++state.metrics.fallback_bands;
            auto& view = state.frame.band[band.plane][band.level - 1u][band.band_id];
            if (band.format_bits == 8u) {
                view.data = mirror + uint64_t(band.plane) * vp.mirror_plane_bytes +
                            uint64_t(band.y) * vp.row_stride + band.x;
                view.pitch = vp.row_stride;
                view.storage_bits = 8;
            } else {
                view.data = coeff_bytes + uint64_t(band.plane) * vp.plane_bytes +
                            (uint64_t(band.y) * vp.row_stride + band.x) * sizeof(int16_t);
                view.pitch = vp.row_stride * sizeof(int16_t);
                view.storage_bits = 16;
            }
        } else if (band.level == 4) {
            final_ll[band.plane] = &band;
        }
    }
    for (uint32_t plane = 0; plane < 4; ++plane) {
        if (!final_ll[plane]) throw std::runtime_error("fixed hybrid frame missing final LL band");
        const auto& band = *final_ll[plane];
        const void* ll_data = coeff_bytes + uint64_t(plane) * vp.plane_bytes +
            (uint64_t(band.y) * vp.row_stride + band.x) * sizeof(int16_t);
        for (uint32_t level = 0; level < 3; ++level) {
            auto& view = state.frame.band[plane][level][0];
            view.data = ll_data;
            view.pitch = vp.row_stride * sizeof(int16_t);
            view.storage_bits = 16;
        }
    }
}

class GpuOnePassHybridPacker {
public:
    GpuOnePassHybridPacker(VulkanPipeline& vp, const Options& o) : vp_(vp) {
        if (vp_.host_cpu_transform || !vp_.fused2d_dispatch)
            throw std::runtime_error("gpu-hybrid-onepass requires fused hardware Vulkan output");
        create_pipeline(o.int8_onepass_shader);
        create_slots();
        std::cout << "HYBRID_GPU_CONFIG storage=gpu-hybrid-onepass fallback=whole-band-int16"
                  << " shader=" << o.int8_onepass_shader
                  << " dispatches=1 submissions=1 fences=1 slots=" << slots_.size() << "\n";
    }
    ~GpuOnePassHybridPacker() { cleanup(); }
    GpuOnePassHybridPacker(const GpuOnePassHybridPacker&) = delete;
    GpuOnePassHybridPacker& operator=(const GpuOnePassHybridPacker&) = delete;

    void submit_with_transform(size_t index) {
        auto& s = slots_.at(index);
        auto& vs = vp_.slots.at(index);
        s.start = Clock::now();
        vs.submit = s.start;
        VK_CHECK(vkResetFences(vp_.device,1,&s.fence));
        VkCommandBuffer cmds[2]{vs.cmd,s.cmd};
        VkSubmitInfo info{VK_STRUCTURE_TYPE_SUBMIT_INFO};
        info.commandBufferCount=2; info.pCommandBuffers=cmds;
        VK_CHECK(vkQueueSubmit(vp_.queue,1,&info,s.fence));
    }
    void wait_and_prepare(size_t index) {
        auto& s=slots_.at(index); auto& vs=vp_.slots.at(index);
        VK_CHECK(vkWaitForFences(vp_.device,1,&s.fence,VK_TRUE,UINT64_MAX));
        vs.gpu_wall_ns=elapsed_ns(vs.submit); vs.gpu_exec_ns=-1;
        if(vs.query){uint64_t t[2]{};const VkResult r=vkGetQueryPoolResults(vp_.device,vs.query,0,2,sizeof(t),t,sizeof(uint64_t),VK_QUERY_RESULT_64_BIT);if(r==VK_SUCCESS&&t[1]>=t[0])vs.gpu_exec_ns=int64_t(double(t[1]-t[0])*vp_.timestamp_ns);}
        s.state.metrics.range_ns=0; s.state.metrics.pack_ns=0;
        if(s.query){uint64_t t[2]{};const VkResult r=vkGetQueryPoolResults(vp_.device,s.query,0,2,sizeof(t),t,sizeof(uint64_t),VK_QUERY_RESULT_64_BIT);if(r==VK_SUCCESS&&t[1]>=t[0])s.state.metrics.pack_ns=int64_t(double(t[1]-t[0])*vp_.timestamp_ns);}
        prepare_fixed_hybrid_state(s.state,vp_,index,s.mapped_mirror,s.mapped_overflow);
    }
    const vc5_pretransformed_hybrid_frame& frame(size_t i) const { return slots_.at(i).state.frame; }
    const FixedHybridMetrics& metrics(size_t i) const { return slots_.at(i).state.metrics; }

private:
    struct Push { uint32_t total_elements,row_stride,plane_elements,source_plane_stride,mirror_plane_words,plane_width,plane_height; };
    struct Slot {
        VkBuffer mirror=VK_NULL_HANDLE,overflow=VK_NULL_HANDLE;
        VkDeviceMemory mirror_mem=VK_NULL_HANDLE,overflow_mem=VK_NULL_HANDLE;
        uint8_t* mapped_mirror=nullptr; uint32_t* mapped_overflow=nullptr;
        VkDescriptorSet set=VK_NULL_HANDLE; VkCommandBuffer cmd=VK_NULL_HANDLE;
        VkFence fence=VK_NULL_HANDLE; VkQueryPool query=VK_NULL_HANDLE;
        Clock::time_point start{}; FixedHybridState state{};
    };
    VulkanPipeline& vp_;
    VkDescriptorSetLayout dsl_=VK_NULL_HANDLE; VkPipelineLayout layout_=VK_NULL_HANDLE;
    VkPipeline pipeline_=VK_NULL_HANDLE; VkDescriptorPool pool_=VK_NULL_HANDLE;
    VkCommandPool command_pool_=VK_NULL_HANDLE; std::vector<Slot> slots_;

    void create_pipeline(const std::string& path) {
        VkDescriptorSetLayoutBinding b[3]{};for(uint32_t i=0;i<3;++i){b[i].binding=i;b[i].descriptorType=VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;b[i].descriptorCount=1;b[i].stageFlags=VK_SHADER_STAGE_COMPUTE_BIT;}
        VkDescriptorSetLayoutCreateInfo di{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};di.bindingCount=3;di.pBindings=b;VK_CHECK(vkCreateDescriptorSetLayout(vp_.device,&di,nullptr,&dsl_));
        VkPushConstantRange pc{VK_SHADER_STAGE_COMPUTE_BIT,0,uint32_t(sizeof(Push))};VkPipelineLayoutCreateInfo li{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};li.setLayoutCount=1;li.pSetLayouts=&dsl_;li.pushConstantRangeCount=1;li.pPushConstantRanges=&pc;VK_CHECK(vkCreatePipelineLayout(vp_.device,&li,nullptr,&layout_));
        auto code=read_spv(path);VkShaderModuleCreateInfo sm{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};sm.codeSize=code.size()*4;sm.pCode=code.data();VkShaderModule module=VK_NULL_HANDLE;VK_CHECK(vkCreateShaderModule(vp_.device,&sm,nullptr,&module));VkComputePipelineCreateInfo ci{VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};ci.layout=layout_;ci.stage.sType=VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;ci.stage.stage=VK_SHADER_STAGE_COMPUTE_BIT;ci.stage.module=module;ci.stage.pName="main";const VkResult r=vkCreateComputePipelines(vp_.device,VK_NULL_HANDLE,1,&ci,nullptr,&pipeline_);vkDestroyShaderModule(vp_.device,module,nullptr);VK_CHECK(r);
    }
    void create_slots() {
        const uint32_t n=uint32_t(vp_.slots.size());VkDescriptorPoolSize ps{VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,n*3u};VkDescriptorPoolCreateInfo pi{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};pi.maxSets=n;pi.poolSizeCount=1;pi.pPoolSizes=&ps;VK_CHECK(vkCreateDescriptorPool(vp_.device,&pi,nullptr,&pool_));VkCommandPoolCreateInfo ci{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};ci.queueFamilyIndex=vp_.family;ci.flags=VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;VK_CHECK(vkCreateCommandPool(vp_.device,&ci,nullptr,&command_pool_));std::vector<VkDescriptorSetLayout> ls(n,dsl_);std::vector<VkDescriptorSet> sets(n);VkDescriptorSetAllocateInfo ai{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};ai.descriptorPool=pool_;ai.descriptorSetCount=n;ai.pSetLayouts=ls.data();VK_CHECK(vkAllocateDescriptorSets(vp_.device,&ai,sets.data()));std::vector<VkCommandBuffer> cmds(n);VkCommandBufferAllocateInfo ca{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};ca.commandPool=command_pool_;ca.level=VK_COMMAND_BUFFER_LEVEL_PRIMARY;ca.commandBufferCount=n;VK_CHECK(vkAllocateCommandBuffers(vp_.device,&ca,cmds.data()));slots_.resize(n);
        for(uint32_t i=0;i<n;++i){auto& s=slots_[i];s.set=sets[i];s.cmd=cmds[i];s.state.bands=make_hybrid_bands(uint32_t(vp_.row_stride),uint32_t(vp_.plane_elems/vp_.row_stride));bool ignored=false;vp_.create_buffer(vp_.mirror_buffer_bytes,VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT|VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,VK_MEMORY_PROPERTY_HOST_CACHED_BIT,s.mirror,s.mirror_mem,&ignored);vp_.create_buffer(8u,VK_BUFFER_USAGE_STORAGE_BUFFER_BIT|VK_BUFFER_USAGE_TRANSFER_DST_BIT,VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT|VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,VK_MEMORY_PROPERTY_HOST_CACHED_BIT,s.overflow,s.overflow_mem,&ignored);VK_CHECK(vkMapMemory(vp_.device,s.mirror_mem,0,vp_.mirror_buffer_bytes,0,reinterpret_cast<void**>(&s.mapped_mirror)));VK_CHECK(vkMapMemory(vp_.device,s.overflow_mem,0,8u,0,reinterpret_cast<void**>(&s.mapped_overflow)));VkDescriptorBufferInfo bi[3]{{vp_.slots[i].coeff,0,VK_WHOLE_SIZE},{s.mirror,0,VK_WHOLE_SIZE},{s.overflow,0,8u}};VkWriteDescriptorSet w[3]{};for(uint32_t j=0;j<3;++j){w[j].sType=VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;w[j].dstSet=s.set;w[j].dstBinding=j;w[j].descriptorType=VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;w[j].descriptorCount=1;w[j].pBufferInfo=&bi[j];}vkUpdateDescriptorSets(vp_.device,3,w,0,nullptr);VkFenceCreateInfo fi{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};VK_CHECK(vkCreateFence(vp_.device,&fi,nullptr,&s.fence));if(vp_.timestamps){VkQueryPoolCreateInfo qi{VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO};qi.queryType=VK_QUERY_TYPE_TIMESTAMP;qi.queryCount=2;VK_CHECK(vkCreateQueryPool(vp_.device,&qi,nullptr,&s.query));}record(s);}
    }
    void record(Slot& s) {
        VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};bi.flags=VK_COMMAND_BUFFER_USAGE_SIMULTANEOUS_USE_BIT;VK_CHECK(vkBeginCommandBuffer(s.cmd,&bi));if(s.query)vkCmdResetQueryPool(s.cmd,s.query,0,2);vkCmdFillBuffer(s.cmd,s.overflow,0,8u,0u);VkMemoryBarrier clear{VK_STRUCTURE_TYPE_MEMORY_BARRIER};clear.srcAccessMask=VK_ACCESS_TRANSFER_WRITE_BIT;clear.dstAccessMask=VK_ACCESS_SHADER_READ_BIT|VK_ACCESS_SHADER_WRITE_BIT;vkCmdPipelineBarrier(s.cmd,VK_PIPELINE_STAGE_TRANSFER_BIT,VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,0,1,&clear,0,nullptr,0,nullptr);VkMemoryBarrier coeff{VK_STRUCTURE_TYPE_MEMORY_BARRIER};coeff.srcAccessMask=VK_ACCESS_SHADER_WRITE_BIT|VK_ACCESS_TRANSFER_READ_BIT;coeff.dstAccessMask=VK_ACCESS_SHADER_READ_BIT;vkCmdPipelineBarrier(s.cmd,VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT|VK_PIPELINE_STAGE_TRANSFER_BIT,VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,0,1,&coeff,0,nullptr,0,nullptr);if(s.query)vkCmdWriteTimestamp(s.cmd,VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,s.query,0);vkCmdBindPipeline(s.cmd,VK_PIPELINE_BIND_POINT_COMPUTE,pipeline_);vkCmdBindDescriptorSets(s.cmd,VK_PIPELINE_BIND_POINT_COMPUTE,layout_,0,1,&s.set,0,nullptr);Push pc{uint32_t(vp_.plane_elems*4u),uint32_t(vp_.row_stride),uint32_t(vp_.plane_elems),uint32_t(vp_.plane_stride_elems),uint32_t(vp_.mirror_plane_bytes/4u),uint32_t(vp_.row_stride),uint32_t(vp_.plane_elems/vp_.row_stride)};vkCmdPushConstants(s.cmd,layout_,VK_SHADER_STAGE_COMPUTE_BIT,0,sizeof(pc),&pc);const uint32_t words=(pc.total_elements+3u)/4u;vkCmdDispatch(s.cmd,(words+255u)/256u,1,1);if(s.query)vkCmdWriteTimestamp(s.cmd,VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,s.query,1);VkMemoryBarrier host{VK_STRUCTURE_TYPE_MEMORY_BARRIER};host.srcAccessMask=VK_ACCESS_SHADER_WRITE_BIT;host.dstAccessMask=VK_ACCESS_HOST_READ_BIT;vkCmdPipelineBarrier(s.cmd,VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,VK_PIPELINE_STAGE_HOST_BIT,0,1,&host,0,nullptr,0,nullptr);VK_CHECK(vkEndCommandBuffer(s.cmd));
    }
    void cleanup(){if(!vp_.device)return;vkDeviceWaitIdle(vp_.device);for(auto&s:slots_){if(s.mapped_mirror)vkUnmapMemory(vp_.device,s.mirror_mem);if(s.mapped_overflow)vkUnmapMemory(vp_.device,s.overflow_mem);if(s.query)vkDestroyQueryPool(vp_.device,s.query,nullptr);if(s.fence)vkDestroyFence(vp_.device,s.fence,nullptr);if(s.mirror)vkDestroyBuffer(vp_.device,s.mirror,nullptr);if(s.overflow)vkDestroyBuffer(vp_.device,s.overflow,nullptr);if(s.mirror_mem)vkFreeMemory(vp_.device,s.mirror_mem,nullptr);if(s.overflow_mem)vkFreeMemory(vp_.device,s.overflow_mem,nullptr);}if(pipeline_)vkDestroyPipeline(vp_.device,pipeline_,nullptr);if(layout_)vkDestroyPipelineLayout(vp_.device,layout_,nullptr);if(pool_)vkDestroyDescriptorPool(vp_.device,pool_,nullptr);if(dsl_)vkDestroyDescriptorSetLayout(vp_.device,dsl_,nullptr);if(command_pool_)vkDestroyCommandPool(vp_.device,command_pool_,nullptr);}
};

class FusedMirrorHybridProvider {
public:
    explicit FusedMirrorHybridProvider(VulkanPipeline& vp) : vp_(vp) {
        if (!vp_.fused_mirror_output)
            throw std::runtime_error("fused mirror provider requires fused mirror Vulkan shader");
        states_.resize(vp_.slots.size());
        for (auto& state : states_)
            state.bands = make_hybrid_bands(uint32_t(vp_.row_stride),
                                            uint32_t(vp_.plane_elems / vp_.row_stride));
        std::cout << "HYBRID_GPU_CONFIG storage=gpu-hybrid-fused fallback=whole-band-int16"
                  << " dispatches=0 extra_submissions=0 extra_fences=0 slots="
                  << states_.size() << "\n";
    }
    void prepare(size_t i){auto& slot=vp_.slots.at(i);auto& st=states_.at(i);st.metrics.range_ns=0;st.metrics.pack_ns=0;prepare_fixed_hybrid_state(st,vp_,i,slot.mapped_mirror,slot.mapped_overflow);}
    const vc5_pretransformed_hybrid_frame& frame(size_t i)const{return states_.at(i).frame;}
    const FixedHybridMetrics& metrics(size_t i)const{return states_.at(i).metrics;}
private: VulkanPipeline& vp_;std::vector<FixedHybridState> states_;
};


struct TileDirectMetrics {
    uint64_t physical_bytes=0;
    uint32_t int8_tiles=0;
    uint32_t fallback_tiles=0;
    uint32_t total_tiles=0;
};

struct TileDirectState {
    vc5_pretransformed_hybrid_frame frame{};
    vc5_pretransformed_tiled_band_view tiled[4][3][4]{};
    TileDirectMetrics metrics{};
};

static bool tile_fallback_flag(const uint32_t* flags, uint32_t tile, bool u32_per_tile) {
    return u32_per_tile ? flags[tile] != 0u
                        : (flags[tile >> 5u] & (1u << (tile & 31u))) != 0u;
}

static int16_t tiled_band_value(const vc5_pretransformed_tiled_band_view& view,
                                uint32_t x, uint32_t y) {
    const uint32_t tile_x = x / view.tile_width;
    const uint32_t tile_y = y / view.tile_height;
    const uint32_t local_x = x % view.tile_width;
    const uint32_t local_y = y % view.tile_height;
    const uint32_t tile = view.tile_base + tile_y * view.tiles_x + tile_x;
    const uint32_t offset = local_y * view.tile_width + local_x;
    const bool fallback = tile_fallback_flag(view.format_words, tile, view.reserved != 0u);
    if (view.layout_mode == 1u) {
        const size_t linear = size_t(view.coefficient_base) + size_t(y) * view.row_stride + x;
        return fallback ? view.int16_tiles[linear] : int16_t(view.int8_tiles[linear]);
    }
    if (fallback) {
        const auto* base = reinterpret_cast<const uint8_t*>(view.int16_tiles) +
            size_t(tile) * view.int16_tile_stride_bytes;
        int16_t value = 0;
        std::memcpy(&value, base + size_t(offset) * sizeof(int16_t), sizeof(value));
        return value;
    }
    const auto* base = reinterpret_cast<const uint8_t*>(view.int8_tiles) +
        size_t(tile) * view.int8_tile_stride_bytes;
    return int16_t(reinterpret_cast<const int8_t*>(base)[offset]);
}

struct TileDirectSnapshot {
    std::vector<int8_t> int8_values;
    // The fallback pool retains the GPU's dense address space so the existing
    // tile-aware entropy ABI stays unchanged, but it is intentionally allocated
    // without value-initialising every page. Only flagged fallback rows are
    // touched by the sequential snapshot copy, so Linux can keep the unused
    // virtual pages uncommitted instead of consuming roughly 16 MiB per job.
    std::unique_ptr<int16_t[]> int16_values;
    size_t int16_count = 0;
    std::vector<uint32_t> flags;
    std::vector<int16_t> final_ll;
    TileDirectState state{};
    uint64_t copied_bytes = 0;
    uint64_t allocated_bytes = 0;

    void allocate(const VulkanPipeline& vp) {
        int8_values.resize(vp.tile_i8_buffer_bytes);
        const size_t required_int16 =
            (vp.tile_i16_buffer_bytes + sizeof(int16_t) - 1u) / sizeof(int16_t);
        if (required_int16 != int16_count) {
            int16_values.reset(required_int16 ? new int16_t[required_int16] : nullptr);
            int16_count = required_int16;
        }
        flags.resize((vp.tile_flags_buffer_bytes + sizeof(uint32_t) - 1u) /
                     sizeof(uint32_t));
        final_ll.resize((vp.tile_ll_buffer_bytes + sizeof(int16_t) - 1u) /
                        sizeof(int16_t));
        allocated_bytes = uint64_t(vp.tile_i8_buffer_bytes) +
                          uint64_t(vp.tile_i16_buffer_bytes) +
                          uint64_t(vp.tile_flags_buffer_bytes) +
                          uint64_t(vp.tile_ll_buffer_bytes);
        copied_bytes = 0;
    }
};

static uint64_t copy_tile_direct_fallback_int16(
        const VulkanPipeline& vp, const uint32_t* flags,
        const int16_t* source, int16_t* destination) {
    uint64_t copied = 0;
    for (uint32_t plane = 0; plane < 4u; ++plane) {
        uint32_t cw = uint32_t(vp.row_stride);
        uint32_t ch = uint32_t(vp.plane_elems / vp.row_stride);
        for (uint32_t level = 0; level < 3u; ++level) {
            const uint32_t bw = cw >> 1u;
            const uint32_t bh = ch >> 1u;
            const uint32_t tiles_x = (bw + 7u) / 8u;
            const uint32_t tiles_y = (bh + 7u) / 8u;
            const uint32_t tile_count = tiles_x * tiles_y;
            for (uint32_t band = 0; band < 3u; ++band) {
                const uint32_t tile_base = vp.tile_base[plane][level][band];
                const uint32_t coefficient_base = vp.tile_coeff_base[plane][level][band];
                for (uint32_t t = 0; t < tile_count; ++t) {
                    const uint32_t tile = tile_base + t;
                    if (!tile_fallback_flag(flags, tile, vp.tile_flags_u32)) continue;
                    if (!vp.tile_linear_layout) {
                        const size_t byte_offset = size_t(tile) * 128u;
                        std::memcpy(reinterpret_cast<uint8_t*>(destination) + byte_offset,
                                    reinterpret_cast<const uint8_t*>(source) + byte_offset, 128u);
                        copied += 128u;
                        continue;
                    }
                    const uint32_t tx = t % tiles_x;
                    const uint32_t ty = t / tiles_x;
                    const uint32_t x0 = tx * 8u;
                    const uint32_t y0 = ty * 8u;
                    const uint32_t columns = std::min(8u, bw - x0);
                    const uint32_t rows = std::min(8u, bh - y0);
                    for (uint32_t row = 0; row < rows; ++row) {
                        const size_t offset = size_t(coefficient_base) +
                            size_t(y0 + row) * bw + x0;
                        const size_t bytes = size_t(columns) * sizeof(int16_t);
                        std::memcpy(destination + offset, source + offset, bytes);
                        copied += bytes;
                    }
                }
            }
            cw = bw;
            ch = bh;
        }
    }
    return copied;
}

static void prepare_tile_direct_state(const VulkanPipeline& vp,
                                      TileDirectState& state,
                                      const int8_t* int8_values,
                                      const int16_t* int16_values,
                                      const uint32_t* flags,
                                      const int16_t* final_ll) {
    state.frame = {};
    state.metrics = {};
    state.metrics.total_tiles = vp.tile_total_count;
    state.metrics.physical_bytes = uint64_t(vp.tile_ll_plane_elems) *
                                   4u * sizeof(int16_t);

    for (uint32_t plane = 0; plane < 4u; ++plane) {
        uint32_t cw = uint32_t(vp.row_stride);
        uint32_t ch = uint32_t(vp.plane_elems / vp.row_stride);
        for (uint32_t level = 0; level < 3u; ++level) {
            const uint32_t bw = cw >> 1u;
            const uint32_t bh = ch >> 1u;
            const uint32_t tiles_x = (bw + 7u) / 8u;
            const uint32_t tiles_y = (bh + 7u) / 8u;
            const uint32_t tile_count = tiles_x * tiles_y;
            for (uint32_t band = 1u; band <= 3u; ++band) {
                auto& tiled = state.tiled[plane][level][band];
                tiled.int8_tiles = int8_values;
                tiled.int16_tiles = int16_values;
                tiled.format_words = flags;
                tiled.tile_base = vp.tile_base[plane][level][band - 1u];
                tiled.tiles_x = tiles_x;
                tiled.tiles_y = tiles_y;
                tiled.tile_width = 8u;
                tiled.tile_height = 8u;
                tiled.int8_tile_stride_bytes = 64u;
                tiled.int16_tile_stride_bytes = 128u;
                tiled.coefficient_base = vp.tile_coeff_base[plane][level][band - 1u];
                tiled.row_stride = bw;
                tiled.layout_mode = vp.tile_linear_layout ? 1u : 0u;
                tiled.reader_mode = vp.tile_reader_mode;
                tiled.reserved = vp.tile_flags_u32 ? 1u : 0u;
                auto& view = state.frame.band[plane][level][band];
                view.data = &tiled;
                view.pitch = 1u;
                view.storage_bits = 24u;
                for (uint32_t t = 0; t < tile_count; ++t) {
                    const uint32_t tile = tiled.tile_base + t;
                    const bool fallback = tile_fallback_flag(flags, tile, vp.tile_flags_u32);
                    const uint32_t tx = t % tiles_x;
                    const uint32_t ty = t / tiles_x;
                    const uint32_t samples = vp.tile_linear_layout
                        ? std::min(8u, bw - tx * 8u) * std::min(8u, bh - ty * 8u)
                        : 64u;
                    if (fallback) {
                        ++state.metrics.fallback_tiles;
                        state.metrics.physical_bytes += uint64_t(samples) * 2u;
                    } else {
                        ++state.metrics.int8_tiles;
                        state.metrics.physical_bytes += samples;
                    }
                }
            }
            cw = bw;
            ch = bh;
        }
        const uint8_t* ll = reinterpret_cast<const uint8_t*>(final_ll) +
            uint64_t(plane) * vp.tile_ll_plane_bytes;
        for (uint32_t level = 0; level < 3u; ++level) {
            auto& view = state.frame.band[plane][level][0];
            view.data = ll;
            view.pitch = vp.tile_ll_width * sizeof(int16_t);
            view.storage_bits = 16u;
        }
    }
}

static std::vector<int16_t> reconstruct_tile_direct_state(const VulkanPipeline& vp,
                                                           const TileDirectState& state,
                                                           const int16_t* final_ll) {
    std::vector<int16_t> full(vp.plane_stride_elems * 4u, int16_t(0));
    const auto bands = make_hybrid_bands(uint32_t(vp.row_stride),
                                         uint32_t(vp.plane_elems / vp.row_stride));
    for (const auto& band : bands) {
        int16_t* plane = full.data() + size_t(band.plane) * vp.plane_stride_elems;
        if (band.band_id != 0u) {
            const auto& tiled = state.tiled[band.plane][band.level - 1u][band.band_id];
            for (uint32_t y = 0; y < band.height; ++y)
                for (uint32_t x = 0; x < band.width; ++x)
                    plane[(size_t(band.y + y) * vp.row_stride) + band.x + x] =
                        tiled_band_value(tiled, x, y);
        } else if (band.level == 4u) {
            const auto* ll = reinterpret_cast<const int16_t*>(
                reinterpret_cast<const uint8_t*>(final_ll) +
                uint64_t(band.plane) * vp.tile_ll_plane_bytes);
            for (uint32_t y = 0; y < band.height; ++y)
                std::memcpy(plane + size_t(y) * vp.row_stride,
                            ll + size_t(y) * vp.tile_ll_width,
                            size_t(band.width) * sizeof(int16_t));
        }
    }
    return full;
}

class TileDirectHybridProvider {
public:
    explicit TileDirectHybridProvider(VulkanPipeline& vp) : vp_(vp) {
        if (!vp_.tile_direct_output)
            throw std::runtime_error("tile-direct provider requires tile-direct Vulkan shader");
        states_.resize(vp_.slots.size());
        std::cout << "HYBRID_GPU_CONFIG storage=gpu-hybrid-tile-direct fallback=tile-int16"
                  << " tile=8x8 layout=" << (vp_.tile_linear_layout ? "scan-linear" : "fixed-slots")
                  << " flags=" << (vp_.tile_flags_u32 ? "u32-per-tile-no-global-atomic" : "bitmap-atomic")
                  << " reader=" << (vp_.tile_reader_mode == 0u ? "legacy" :
                                      (vp_.tile_reader_mode == 1u ? "rowplan" :
                                      (vp_.tile_reader_mode == 2u ? "neon" :
                                      (vp_.tile_reader_mode == 3u ? "bandfast" :
                                      (vp_.tile_reader_mode == 4u ? "group16x8" :
                                      (vp_.tile_reader_mode == 5u ? "group16x16" : "runspan"))))))
                  << " duplicate_highpass_writes=0 extra_dispatches=0"
                  << " extra_submissions=0 extra_fences=0 total_tiles=" << vp_.tile_total_count
                  << " slots=" << states_.size() << "\n";
    }

    void prepare(size_t index) {
        const auto& slot = vp_.slots.at(index);
        prepare_tile_direct_state(vp_, states_.at(index), slot.mapped_tile_i8,
                                  slot.mapped_tile_i16, slot.mapped_tile_flags,
                                  slot.mapped_coeff);
    }

    void allocate_snapshot(TileDirectSnapshot& snapshot) const {
        snapshot.allocate(vp_);
    }

    void snapshot(size_t index, TileDirectSnapshot& snapshot) const {
        const auto& slot = vp_.slots.at(index);
        if (snapshot.int8_values.size() != vp_.tile_i8_buffer_bytes ||
            snapshot.int16_count * sizeof(int16_t) != vp_.tile_i16_buffer_bytes)
            snapshot.allocate(vp_);
        std::memcpy(snapshot.int8_values.data(), slot.mapped_tile_i8,
                    vp_.tile_i8_buffer_bytes);
        std::memcpy(snapshot.flags.data(), slot.mapped_tile_flags,
                    vp_.tile_flags_buffer_bytes);
        std::memcpy(snapshot.final_ll.data(), slot.mapped_coeff,
                    vp_.tile_ll_buffer_bytes);
        const uint64_t fallback_bytes = copy_tile_direct_fallback_int16(
            vp_, snapshot.flags.data(), slot.mapped_tile_i16,
            snapshot.int16_values.get());
        snapshot.copied_bytes = uint64_t(vp_.tile_i8_buffer_bytes) +
                                uint64_t(vp_.tile_flags_buffer_bytes) +
                                uint64_t(vp_.tile_ll_buffer_bytes) + fallback_bytes;
        prepare_tile_direct_state(vp_, snapshot.state, snapshot.int8_values.data(),
                                  snapshot.int16_values.get(), snapshot.flags.data(),
                                  snapshot.final_ll.data());
    }

    const vc5_pretransformed_hybrid_frame& frame(size_t index) const {
        return states_.at(index).frame;
    }
    const TileDirectMetrics& metrics(size_t index) const { return states_.at(index).metrics; }
    const vc5_pretransformed_hybrid_frame& frame(const TileDirectSnapshot& snapshot) const {
        return snapshot.state.frame;
    }
    const TileDirectMetrics& metrics(const TileDirectSnapshot& snapshot) const {
        return snapshot.state.metrics;
    }

    std::vector<int16_t> reconstruct_full_int16(size_t index) const {
        const auto& slot = vp_.slots.at(index);
        return reconstruct_tile_direct_state(vp_, states_.at(index), slot.mapped_coeff);
    }
    std::vector<int16_t> reconstruct_full_int16(const TileDirectSnapshot& snapshot) const {
        return reconstruct_tile_direct_state(vp_, snapshot.state, snapshot.final_ll.data());
    }

private:
    VulkanPipeline& vp_;
    std::vector<TileDirectState> states_;
};

static int16_t hybrid_value(const vc5_pretransformed_band_view& view,
                            uint32_t x, uint32_t y) {
    if (view.storage_bits == 24)
        return tiled_band_value(*static_cast<const vc5_pretransformed_tiled_band_view*>(view.data), x, y);
    const auto* row = static_cast<const uint8_t*>(view.data) + size_t(y) * view.pitch;
    if (view.storage_bits == 8) return int16_t(reinterpret_cast<const int8_t*>(row)[x]);
    int16_t value = 0;
    std::memcpy(&value, row + size_t(x) * sizeof(int16_t), sizeof(value));
    return value;
}

static void verify_hybrid_gpu_packer(GpuHybridPacker& packer, VulkanPipeline& vp) {
    size_t mismatches = 0;
    for (size_t slot_index = 0; slot_index < vp.slots.size(); ++slot_index) {
        packer.submit_range(slot_index);
        packer.wait_range_and_prepare_pack(slot_index);
        packer.submit_pack(slot_index);
        packer.wait_pack(slot_index);
        const int16_t* source = vp.coefficient_ptr(vp.slots[slot_index]);
        const auto& frame = packer.frame(slot_index);
        for (const auto& band : packer.bands(slot_index)) {
            const uint32_t level_index = band.band_id == 0 ? 0u : band.level - 1u;
            const auto& view = frame.band[band.plane][level_index][band.band_id];
            for (uint32_t y = 0; y < band.height; ++y) {
                for (uint32_t x = 0; x < band.width; ++x) {
                    const int16_t expected = source[uint64_t(band.plane) * vp.plane_stride_elems +
                        uint64_t(band.y + y) * vp.row_stride + band.x + x];
                    const int16_t actual = hybrid_value(view, x, y);
                    if (actual != expected) {
                        if (++mismatches <= 8)
                            log_event("ERROR", "hybrid-verify", "coefficient mismatch slot=" +
                                std::to_string(slot_index) + " plane=" + std::to_string(band.plane) +
                                " level=" + std::to_string(band.level) + " band=" +
                                std::to_string(band.band_id));
                    }
                }
            }
        }
    }
    if (mismatches) throw std::runtime_error("GPU hybrid pack verification failed: mismatches=" + std::to_string(mismatches));
    std::cout << "HYBRID_GPU_VERIFY exact_coefficients=YES slots=" << vp.slots.size() << "\n";
}

static void verify_tile_direct_gpu_transform(VulkanPipeline& vp,
                                             TileDirectHybridProvider& provider,
                                             const Options& o,
                                             const std::vector<uint16_t>& src,
                                             const std::vector<uint16_t>& lut) {
    constexpr int rounds = 3;
    log_event("INFO", "verification", "stress-testing tile-direct GPU output across all Vulkan ring slots for " +
              std::to_string(rounds) + " rounds against the CPU reference");
    const size_t total_elems = vp.plane_stride_elems * 4u;
    const auto specs = plane_specs(o);
    std::vector<std::vector<int16_t>> expected(vp.slots.size());
    int overall_max_abs = 0;

    for (int round = 0; round < rounds; ++round) {
        for (size_t si = 0; si < vp.slots.size(); ++si) {
            auto& slot = vp.slots[si];
            std::fill(slot.mapped_ping, slot.mapped_ping + total_elems, int16_t(0));
            split_compand(o, src.data(), lut, slot.mapped_ping, vp.plane_stride_elems);
            expected[si].assign(slot.mapped_ping, slot.mapped_ping + total_elems);
            cpu_transform_schedule(o, vp.mode_spec, expected[si].data(),
                                   vp.plane_stride_elems, vp.row_stride);
            vp.submit(slot);
        }

        for (size_t si = 0; si < vp.slots.size(); ++si) {
            auto& slot = vp.slots[si];
            vp.wait(slot);
            provider.prepare(si);
            const auto got = provider.reconstruct_full_int16(si);
            size_t mismatches = 0;
            int max_abs = 0;
            std::ostringstream first;
            for (int plane = 0; plane < 4; ++plane) {
                const int16_t* actual = got.data() + size_t(plane) * vp.plane_stride_elems;
                const int16_t* reference = expected[si].data() + size_t(plane) * vp.plane_stride_elems;
                for (int y = 0; y < specs[size_t(plane)].height; ++y) {
                    for (int x = 0; x < specs[size_t(plane)].width; ++x) {
                        const size_t offset = size_t(y) * vp.row_stride + size_t(x);
                        max_abs = std::max(max_abs, std::abs(int(actual[offset])));
                        if (actual[offset] != reference[offset]) {
                            if (mismatches < 8) {
                                first << " round=" << round << " slot=" << si
                                      << " plane=" << plane << " x=" << x << " y=" << y
                                      << " gpu=" << actual[offset]
                                      << " cpu=" << reference[offset];
                            }
                            ++mismatches;
                        }
                    }
                }
            }
            if (mismatches != 0) {
                throw std::runtime_error("VC-5 tile-direct GPU verification failed: mismatches=" +
                                         std::to_string(mismatches) + first.str());
            }
            if (max_abs >= 32767) {
                throw std::runtime_error("VC-5 tile-direct GPU verification reached int16 saturation: max_abs=" +
                                         std::to_string(max_abs));
            }
            overall_max_abs = std::max(overall_max_abs, max_abs);
        }
    }

    std::cout << "GPU_VERIFY PASS rounds=" << rounds
              << " slots=" << vp.slots.size()
              << " submissions=" << (vp.slots.size() * size_t(rounds))
              << " peak_abs_coeff=" << overall_max_abs
              << " int16_limit=32767 storage=gpu-hybrid-tile-direct\n";
    std::cout << "INT16_SAFETY working_max=" << o.working_max
              << " peak_abs_coeff=" << overall_max_abs
              << " int16_limit=32767 gpu_matches_cpu=YES clamp_activations=0"
              << " tile_direct_reconstruction=EXACT"
              << "  (12-bit int16-safe: " << overall_max_abs << " < 32767)\n";
    std::cout << "HYBRID_GPU_VERIFY exact_coefficients=YES storage=gpu-hybrid-tile-direct"
              << " slots=" << vp.slots.size() << " rounds=" << rounds << "\n";
    log_event("INFO", "verification", "tile-direct GPU output matched CPU reference exactly (peak_abs_coeff=" +
              std::to_string(overall_max_abs) + "): slots=" +
              std::to_string(vp.slots.size()) + " submissions=" +
              std::to_string(vp.slots.size() * size_t(rounds)));
}

static void verify_gpu_transform(VulkanPipeline& vp, const Options& o,
                                 const std::vector<uint16_t>& src,
                                 const std::vector<uint16_t>& lut) {
    constexpr int rounds = 3;
    log_event("INFO", "verification", "stress-testing all Vulkan ring slots for " +
              std::to_string(rounds) + " rounds against the CPU reference");
    const size_t total_elems = vp.plane_stride_elems * 4u;
    const auto specs = plane_specs(o);
    std::vector<std::vector<int16_t>> expected(vp.slots.size());
    int overall_max_abs = 0;

    for (int round = 0; round < rounds; ++round) {
        for (size_t si = 0; si < vp.slots.size(); ++si) {
            auto& slot = vp.slots[si];
            std::fill(slot.mapped_ping, slot.mapped_ping + total_elems, int16_t(0));
            split_compand(o, src.data(), lut, slot.mapped_ping, vp.plane_stride_elems);
            expected[si].assign(slot.mapped_ping, slot.mapped_ping + total_elems);
            cpu_transform_schedule(o, vp.mode_spec, expected[si].data(), vp.plane_stride_elems, vp.row_stride);
            vp.submit(slot);
        }

        for (size_t si = 0; si < vp.slots.size(); ++si) {
            auto& slot = vp.slots[si];
            vp.wait(slot);
            size_t mismatches = 0;
            int max_abs = 0;
            std::ostringstream first;
            std::vector<int16_t> retained_packed;
            const int16_t* got_all = nullptr;
            if (o.gpu_levels == 0) {
                retained_packed.assign(slot.mapped_ping, slot.mapped_ping + total_elems);
                cpu_transform_tail_inplace(o, vp.mode_spec, retained_packed.data(),
                                           vp.plane_stride_elems, vp.row_stride, 1);
                got_all = retained_packed.data();
            } else if (vp.retained_int16_levels) {
                vp.gather_fused_coefficients(slot, retained_packed);
                got_all = retained_packed.data();
            } else {
                got_all = vp.coefficient_ptr(slot);
                if (o.gpu_levels == 1) {
                    retained_packed.assign(got_all, got_all + total_elems);
                    cpu_transform_tail_inplace(o, vp.mode_spec, retained_packed.data(),
                                               vp.plane_stride_elems, vp.row_stride, 2);
                    got_all = retained_packed.data();
                }
            }
            for (int p = 0; p < 4; ++p) {
                const int16_t* got = got_all + size_t(p) * vp.plane_stride_elems;
                const int16_t* ref = expected[si].data() + size_t(p) * vp.plane_stride_elems;
                for (int y = 0; y < specs[size_t(p)].height; ++y) {
                    for (int x = 0; x < specs[size_t(p)].width; ++x) {
                        const size_t i = size_t(y) * vp.row_stride + size_t(x);
                        max_abs = std::max(max_abs, std::abs(int(got[i])));
                        if (got[i] != ref[i]) {
                            if (mismatches < 8) {
                                first << " round=" << round << " slot=" << si
                                      << " plane=" << p << " x=" << x << " y=" << y
                                      << " gpu=" << got[i] << " cpu=" << ref[i];
                            }
                            ++mismatches;
                        }
                    }
                }
            }
            if (mismatches != 0) {
                throw std::runtime_error("VC-5 GPU verification failed: mismatches=" +
                                         std::to_string(mismatches) + first.str());
            }
            if (max_abs >= 32767) {
                throw std::runtime_error("VC-5 GPU verification reached int16 saturation: max_abs=" +
                                         std::to_string(max_abs));
            }
            overall_max_abs = std::max(overall_max_abs, max_abs);
        }
    }
    // Machine-readable pass marker. verify_gpu_transform only reaches this point
    // when every ring slot matched the CPU reference for all rounds without int16
    // saturation; any mismatch throws above. Downstream gates (optimisation_lab.py,
    // benchmark_m5_optimisations.py) require this exact token to treat a run as
    // GPU-verified rather than inferring it from a clean exit code alone.
    std::cout << "GPU_VERIFY PASS rounds=" << rounds
              << " slots=" << vp.slots.size()
              << " submissions=" << (vp.slots.size() * size_t(rounds))
              << " peak_abs_coeff=" << overall_max_abs
              << " int16_limit=32767\n";
    std::cout << "INT16_SAFETY working_max=" << o.working_max
              << " peak_abs_coeff=" << overall_max_abs
              << " int16_limit=32767 gpu_matches_cpu=YES clamp_activations=0"
              << "  (12-bit int16-safe: " << overall_max_abs << " < 32767)\n";
    log_event("INFO", "verification", "GPU ring stress verification passed (peak_abs_coeff=" +
              std::to_string(overall_max_abs) + ", int16-safe): slots=" +
              std::to_string(vp.slots.size()) + " submissions=" +
              std::to_string(vp.slots.size() * rounds));
}

static uint64_t coefficient_int16_bytes(const Options& o);

static double modelled_dram_mb_per_frame(const Options& o) {
    const auto specs = plane_specs(o);
    // Includes the GPU read of the component-plane input. The traditional
    // two-pass schedule reads+writes each level twice (8 bytes/sample); fused2d reads the current
    // LL once and writes LL/high-pass coefficients once (4 bytes/sample).
    double bytes = double(o.width) * double(o.height) * 2.0;
    // --wide-separable materialises the horizontally-filtered lowpass and highpass
    // to memory between the two dispatches, so on top of fused2d's read-once /
    // write-once (4 B/sample) it writes and re-reads one int16 per sample per band
    // pair: +4 B/sample. The v103 packer then reads the int16 high-pass frame and
    // writes the int8 pool, added separately below.
    const double wavelet_bytes_per_sample =
        o.wide_separable ? 8.0 : (o.fused2d_dispatch ? 4.0 : 8.0);
    for (const auto& p : specs) {
        int cw = p.width;
        int ch = p.height;
        for (int level = 0; level < p.levels; ++level) {
            bytes += wavelet_bytes_per_sample * double(cw) * double(ch);
            cw /= 2;
            ch /= 2;
        }
    }
    if (o.wide_separable && o.wide_hybrid_pack) {
        // v103 reads every high-pass int16 coefficient and writes the int8 mirror.
        const double high_pass = double(coefficient_int16_bytes(o));  // 3 bands x all levels
        bytes += high_pass + high_pass / 2.0;
    }
    return bytes / 1048576.0;
}

static uint64_t coefficient_int16_bytes(const Options& o) {
    return uint64_t(o.width) * uint64_t(o.height) * sizeof(int16_t);
}

static uint64_t final_ll_int16_bytes(const Options& o) {
    const uint64_t ll_width = uint64_t(o.width / 16);
    const uint64_t ll_height = uint64_t(o.height / 16);
    return ll_width * ll_height * 4u * sizeof(int16_t);
}

// Logical software-traffic accounting, not a physical memory-controller
// measurement. Main F adjusts the fused transform model for its selected
// int8/int16 coefficient writes; Main D adds its duplicate int8 high-pass
// mirror. Caches, write allocation, coherency and driver behaviour can change
// external LPDDR traffic, so the report labels every value as a model.
struct LogicalTrafficModel {
    double csi_raw_write_mib = 0.0;
    double cpu_raw_read_mib = 0.0;
    double cpu_component_write_mib = 0.0;
    double gpu_input_upload_mib = 0.0;
    double gpu_transform_base_mib = 0.0;
    double hybrid_write_adjustment_mib = 0.0;
    double gpu_transform_mib = 0.0;
    double readback_copy_mib = 0.0;
    double entropy_coefficient_read_mib = 0.0;
    double gpr_output_mib = 0.0;
    double encoder_excluding_csi_mib = 0.0;
    double complete_including_csi_mib = 0.0;
};

static LogicalTrafficModel logical_traffic_model(const Options& o,
                                                  bool readback_copy,
                                                  size_t gpr_packet_bytes,
                                                  uint64_t measured_hybrid_bytes) {
    LogicalTrafficModel m;
    constexpr double mib = 1048576.0;
    const double raw_bytes = double(o.width) * double(o.height) * 2.0;
    const uint64_t full_coeff_bytes = coefficient_int16_bytes(o);
    const uint64_t entropy_bytes = measured_hybrid_bytes != 0u
        ? measured_hybrid_bytes : full_coeff_bytes;

    m.csi_raw_write_mib = raw_bytes / mib;
    m.cpu_raw_read_mib = raw_bytes / mib;
    m.cpu_component_write_mib = double(full_coeff_bytes) / mib;
    m.gpu_input_upload_mib = o.device_local_input ? (2.0 * double(full_coeff_bytes) / mib) : 0.0;
    m.gpu_transform_base_mib = modelled_dram_mb_per_frame(o);
    if (o.coeff_storage == Options::CoeffStorage::GpuHybridTileDirect &&
        measured_hybrid_bytes != 0u && measured_hybrid_bytes < full_coeff_bytes) {
        m.hybrid_write_adjustment_mib =
            -double(full_coeff_bytes - measured_hybrid_bytes) / mib;
    } else if (o.coeff_storage == Options::CoeffStorage::GpuHybridFusedMirror) {
        const uint64_t final_ll = final_ll_int16_bytes(o);
        const uint64_t highpass_int8_mirror =
            full_coeff_bytes > final_ll ? (full_coeff_bytes - final_ll) / 2u : 0u;
        m.hybrid_write_adjustment_mib = double(highpass_int8_mirror) / mib;
    }
    m.gpu_transform_mib = m.gpu_transform_base_mib + m.hybrid_write_adjustment_mib;
    m.readback_copy_mib = readback_copy ? (2.0 * double(full_coeff_bytes) / mib) : 0.0;
    m.entropy_coefficient_read_mib = double(entropy_bytes) / mib;
    m.gpr_output_mib = double(gpr_packet_bytes) / mib;
    m.encoder_excluding_csi_mib =
        m.cpu_raw_read_mib + m.cpu_component_write_mib + m.gpu_input_upload_mib + m.gpu_transform_mib +
        m.readback_copy_mib + m.entropy_coefficient_read_mib + m.gpr_output_mib;
    m.complete_including_csi_mib = m.encoder_excluding_csi_mib + m.csi_raw_write_mib;
    return m;
}

static double traffic_gib_per_second(double mib_per_frame, double fps) {
    return mib_per_frame * fps / 1024.0;
}

static int hybrid_extra_dispatches(const Options& o) {
    if (o.coeff_storage == Options::CoeffStorage::GpuHybridV15) return 76;
    if (o.coeff_storage == Options::CoeffStorage::GpuHybridOnePass) return 1;
    return 0;
}

static std::string memory_flags_name(VkMemoryPropertyFlags flags) {
    std::string out;
    auto add=[&](const char* name){if(!out.empty())out+='|';out+=name;};
    if(flags&VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT)add("DEVICE_LOCAL");
    if(flags&VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT)add("HOST_VISIBLE");
    if(flags&VK_MEMORY_PROPERTY_HOST_COHERENT_BIT)add("HOST_COHERENT");
    if(flags&VK_MEMORY_PROPERTY_HOST_CACHED_BIT)add("HOST_CACHED");
    return out.empty()?"NONE":out;
}

static const char* readback_mode_name(const VulkanPipeline& vp) {
    if (vp.use_vulkan_readback_copy) return "vulkan-copy-host-cached";
    if (vp.use_readback_copy) return "cpu-copy-cacheable";
    return "direct-mapped";
}

static const char* coeff_storage_name(const Options& o) {
    switch (o.coeff_storage) {
        case Options::CoeffStorage::Int16: return "int16";
        case Options::CoeffStorage::GpuFullPersistent: return "gpu-full-persistent";
        case Options::CoeffStorage::CpuHybridBand: return "cpu-hybrid-band";
        case Options::CoeffStorage::CpuHybridTile: return "cpu-hybrid-tile";
        case Options::CoeffStorage::GpuHybridV15: return "gpu-hybrid-v15";
        case Options::CoeffStorage::GpuHybridOnePass: return "gpu-hybrid-onepass";
        case Options::CoeffStorage::GpuHybridFusedMirror: return "gpu-hybrid-fused";
        case Options::CoeffStorage::GpuHybridTileDirect: return "gpu-hybrid-tile-direct";
    }
    return "unknown";
}

template<class T>
class Queue {
public:
    explicit Queue(size_t capacity) : capacity_(capacity) {}

    bool push(T value) {
        std::unique_lock<std::mutex> lock(mu_);
        cv_write_.wait(lock, [&] { return closed_ || queue_.size() < capacity_; });
        if (closed_) return false;
        queue_.push_back(std::move(value));
        high_water_ = std::max(high_water_, queue_.size());
        cv_read_.notify_one();
        return true;
    }

    bool pop(T& value) {
        std::unique_lock<std::mutex> lock(mu_);
        cv_read_.wait(lock, [&] { return closed_ || !queue_.empty(); });
        if (queue_.empty()) return false;
        value = std::move(queue_.front());
        queue_.pop_front();
        cv_write_.notify_one();
        return true;
    }

    bool try_pop(T& value) {
        std::lock_guard<std::mutex> lock(mu_);
        if (closed_ || queue_.empty()) return false;
        value = std::move(queue_.front());
        queue_.pop_front();
        cv_write_.notify_one();
        return true;
    }

    bool pop_until(T& value, Clock::time_point deadline) {
        std::unique_lock<std::mutex> lock(mu_);
        if (!cv_read_.wait_until(lock, deadline,
                                 [&] { return closed_ || !queue_.empty(); })) {
            return false;
        }
        if (queue_.empty()) return false;
        value = std::move(queue_.front());
        queue_.pop_front();
        cv_write_.notify_one();
        return true;
    }

    void close() {
        std::lock_guard<std::mutex> lock(mu_);
        closed_ = true;
        cv_read_.notify_all();
        cv_write_.notify_all();
    }

    size_t size() const {
        std::lock_guard<std::mutex> lock(mu_);
        return queue_.size();
    }

    size_t high_water() const {
        std::lock_guard<std::mutex> lock(mu_);
        return high_water_;
    }

    size_t capacity() const { return capacity_; }

private:
    size_t capacity_;
    mutable std::mutex mu_;
    std::condition_variable cv_read_, cv_write_;
    std::deque<T> queue_;
    size_t high_water_ = 0;
    bool closed_ = false;
};

static int effective_gpu_inflight_limit(const Options& o, int worker_count) {
    const int maximum = o.execution == "pipeline" ? o.buffers : worker_count;
    const int automatic = o.execution == "pipeline"
        ? std::min(2, o.buffers)
        : worker_count;
    return std::max(1, std::min(o.gpu_inflight > 0 ? o.gpu_inflight : automatic,
                                maximum));
}

static size_t nonnegative_frame_difference(uint64_t newer, uint64_t older) {
    if (newer <= older) return 0;
    const uint64_t difference = newer - older;
    return difference > uint64_t(std::numeric_limits<size_t>::max())
        ? std::numeric_limits<size_t>::max()
        : size_t(difference);
}

struct FrameStat {
    int frame = -1;
    int engine = 0;          // 0 = GPU wavelet, 1 = whole-wavelet CPU engine
    double latency = 0.0;
    double split = 0.0;
    double gpu = 0.0;
    double gpu_exec = -1.0;
    double snapshot = 0.0;
    uint64_t snapshot_bytes = 0;
    double gpu_slot_hold = 0.0;
    double int8_range = 0.0;
    double int8_pack = 0.0;
    double cpu_tail = 0.0;
    double entropy = 0.0;
    double entropy_queue_wait = 0.0;
    uint64_t hybrid_bytes = 0;
    uint32_t int8_bands = 0;
    uint32_t int16_fallback_bands = 0;
    uint32_t int8_tiles = 0;
    uint32_t int16_fallback_tiles = 0;
    uint32_t total_tiles = 0;
    double writer = 0.0;
    int entropy_worker = -1;
    size_t payload = 0;
    size_t packet = 0;
    uint32_t crc = 0;
    bool crc_computed = false;
    Clock::time_point finish{};
};

struct CompletionTiming {
    std::vector<double> gaps_ms;
    double span_seconds = 0.0;
};

static CompletionTiming completion_timing(const std::vector<FrameStat>& stats) {
    CompletionTiming timing;
    if (stats.empty()) return timing;
    std::vector<Clock::time_point> finishes;
    finishes.reserve(stats.size());
    for (const auto& stat : stats) finishes.push_back(stat.finish);
    std::sort(finishes.begin(), finishes.end());
    timing.gaps_ms.reserve(finishes.size() > 1 ? finishes.size() - 1 : 0);
    for (size_t i = 1; i < finishes.size(); ++i) {
        timing.gaps_ms.push_back(std::chrono::duration<double, std::milli>(
            finishes[i] - finishes[i - 1]).count());
    }
    if (finishes.size() > 1) {
        timing.span_seconds = std::chrono::duration<double>(
            finishes.back() - finishes.front()).count();
    }
    return timing;
}

struct Result {
    int frames = 0;
    double fps = 0.0;
    // v1.7.53 dual engine. gpu_fps + cpu_fps == fps by construction: both are
    // that engine's share of the frames actually completed, over the same
    // measured window. They are a measurement of how the two engines split the
    // work under real contention, not two independent runs added together.
    int dual_cpu_workers = 0;
    int gpu_frames = 0;
    int cpu_frames = 0;
    double gpu_fps = 0.0;
    double cpu_fps = 0.0;
    double gpu_avg_latency = 0.0;
    double cpu_avg_latency = 0.0;
    double avg_latency = 0.0;
    double p99_latency = 0.0;
    double avg_split = 0.0;
    double avg_gpu = 0.0;
    double avg_gpu_exec = -1.0;
    double avg_snapshot = 0.0;
    double avg_gpu_slot_hold = 0.0;
    uint64_t snapshot_bytes = 0;
    bool cacheable_hybrid_handoff = false;
    double avg_int8_range = 0.0;
    double avg_int8_pack = 0.0;
    double avg_cpu_tail = 0.0;
    double avg_entropy = 0.0;
    uint64_t avg_hybrid_bytes = 0;
    uint32_t avg_int8_bands = 0;
    uint32_t avg_int16_fallback_bands = 0;
    uint32_t avg_int8_tiles = 0;
    uint32_t avg_int16_fallback_tiles = 0;
    uint32_t avg_total_tiles = 0;
    double avg_writer = 0.0;
    double p99_gap = 0.0;
    size_t payload = 0;
    size_t packet = 0;
    double payload_ratio = 0.0;
    double packet_ratio = 0.0;
    double output_mibs = 0.0;
    double submission_seconds = 0.0;
    double total_seconds = 0.0;
    size_t capture_queue_capacity = 0;
    size_t capture_queue_high_water = 0;
    size_t capture_protection_high_water = 0;
    uint64_t capture_frames_enqueued = 0;
    uint64_t capture_frames_dequeued = 0;
    uint64_t capture_backpressure_events = 0;
    size_t capture_dynamic_allocated = 0;
    size_t capture_pending_at_stop = 0;
    double capture_drain_seconds = 0.0;
    double capture_longest_wait_ms = 0.0;
    size_t capture_max_encoder_deficit = 0;
    uint64_t process_rss_bytes = 0;
    uint64_t process_peak_rss_bytes = 0;
    uint32_t first_frame_crc = 0;
    bool first_frame_crc_computed = false;
    size_t reorder_high_water = 0;
    uint64_t out_of_order_entropy_finishes = 0;
    std::vector<uint64_t> worker_frames;
    std::vector<double> worker_avg_entropy_ms;
    std::vector<double> worker_avg_queue_wait_ms;
    std::vector<double> worker_utilisation_pct;
};

// S2. The C5 splice learns its container template per ENCODER, so with N
// workers the SDK path runs 3*N times before any splicing starts. On the
// v1.7.67 ladder an m1 container is ~5 MB and the SDK wrap is expensive, so
// those verification frames now show up as 4-6.5 ms of average dng_wrap.
// The container is identical across workers, so one template suffices: learn
// and byte-verify it once under a lock, publish it, and every worker splices
// from then on. Verification frames drop from 3*N to 3.
struct SharedSpliceTemplate {
    std::mutex mu;
    std::atomic<bool> ready{false};
    std::atomic<bool> failed{false};
    std::vector<uint8_t> prefix, suffix;
    size_t payload_size = 0;
    int verified = 0;
};
static SharedSpliceTemplate g_shared_splice;

/* E9 (v0.3 encoder work): when the shared splice template is ready, encode
   the VC-5 bitstream directly into the container's payload slot instead of
   into the context scratch, and skip the per-frame payload memcpy entirely
   (~3 MB read + ~3 MB write per frame gone). The container is allocated
   with the encoder's full worst-case capacity so an oversized frame cannot
   overrun; a frame whose payload does not match the learned size falls back
   to the SDK wrap, exactly as the copying splice does. Ships identical
   bytes: prefix + payload + suffix, same values, same order. */
static bool g_cpu_splice_inplace = false;

/* C5 (v0.5): retain the in-place container across frames instead of
   malloc/free-ing ~17 MB of virtual space per frame. Only meaningful with
   the Copy writer (the buffer must come back after the copy-out); with
   Handoff the buffer leaves for the writer thread and the pool degrades
   gracefully to per-frame allocation. Ships identical bytes. */
static bool g_cpu_splice_pool = false;

/* v0.9 handoff_pool: the two ideas that individually lost or cost, fixed by
 * each other. writer_handoff lost on the CM5 (-1.78%) because every frame
 * paid a fresh container malloc/free; splice_pool keeps the container but
 * needs the Copy writer (and its ~3 MB copy-out) to get it back. Combined:
 * the pooled container itself is handed to the writer thread, and after the
 * write the writer RETURNS it to the owning encoder's pool. No copy-out, no
 * allocation, at steady state. Identical bytes. */
static bool g_cpu_handoff_pool = false;
bool cinepi_handoff_pool_active() { return g_cpu_handoff_pool; }
void cinepi_set_handoff_pool(bool on) { g_cpu_handoff_pool = on; }

class DirectGprEncoder {
public:
    DirectGprEncoder(const Options& o, const ModeSpec* mode, size_t row_stride, size_t plane_stride) {
        allocator_.Alloc = malloc;
        allocator_.Free = free;
        gpr_parameters_set_defaults(&gpr_params_);
        if (gpr_parameters_parse(&gpr_params_, o.gpr_params.c_str()) != 0)
            throw std::runtime_error("Cannot parse GPR metadata JSON: " + o.gpr_params);
        gpr_params_.input_width = unsigned(o.width);
        gpr_params_.input_height = unsigned(o.height);
        gpr_params_.input_pitch = unsigned(o.width * int(sizeof(uint16_t)));
        gpr_params_.tuning_info.pixel_format = o.bayer == "rggb" ? PIXEL_FORMAT_RGGB_12 : PIXEL_FORMAT_GBRG_12;
        gpr_params_.tuning_info.dgain_saturation_level.level_red = 4095;
        gpr_params_.tuning_info.dgain_saturation_level.level_green_even = 4095;
        gpr_params_.tuning_info.dgain_saturation_level.level_green_odd = 4095;
        gpr_params_.tuning_info.dgain_saturation_level.level_blue = 4095;
        gpr_params_.enable_preview = false;
        gpr_params_.compute_md5sum = false;

        vc5_pretransformed_parameters_set_default(&vc5_params_);
        vc5_params_.input_width = unsigned(o.width);
        vc5_params_.input_height = unsigned(o.height);
        vc5_params_.channel_width = unsigned(o.width / 2);
        vc5_params_.channel_height = unsigned(o.height / 2);
        vc5_params_.row_stride = row_stride;
        vc5_params_.plane_stride = plane_stride;
        vc5_params_.prescale[0] = 0;
        vc5_params_.prescale[1] = 2;
        vc5_params_.prescale[2] = 2;
        vc5_params_.bands_prequantized = true;
        if (mode) {
            for (size_t i=0;i<mode->quant_table.size();++i)
                vc5_params_.quant_table[i] = QUANT(mode->quant_table[i]);
        }
        /* Component-Aware Quantisation: the header must advertise, per channel,
         * exactly what the wavelet path quantised that channel's coefficients by.
         * Both come from the same generator, so they cannot drift apart -- which
         * is the whole compatibility argument for the feature. */
        {
            const auto caq = cpu_quant_tables_caq(o, mode);
            vc5_params_.per_channel_quant = (o.caq != CAQ_OFF);
            for (int ch = 0; ch < 4; ++ch)
                for (size_t i = 0; i < caq[size_t(ch)].size(); ++i)
                    vc5_params_.quant_table_channel[ch][i] = QUANT(caq[size_t(ch)][i]);
        }
        local_inplace = o.cpu_gpr_local_inplace;
        local_inplace_pool = o.cpu_gpr_local_inplace_pool;
        const CODEC_ERROR error = vc5_pretransformed_encoder_create(&vc5_params_, &context_);
        if (error != CODEC_ERROR_OKAY)
            throw std::runtime_error("vc5_pretransformed_encoder_create failed: " + std::to_string(int(error)));
    }

    ~DirectGprEncoder() {
        if (inplace_buf_) allocator_.Free(inplace_buf_);
        if (pool_buf_) allocator_.Free(pool_buf_);
        vc5_pretransformed_encoder_destroy(context_);
        gpr_parameters_destroy(&gpr_params_, allocator_.Free);
    }

    // Accumulated time inside the serialised DNG wrap, including lock wait.
    int64_t dng_wrap_ns = 0;
    bool splice_enabled = false;
    bool splice_shared = false;
    bool splice_active = false;
    bool splice_failed = false;
    int  splice_verified_frames = 0;
    long splice_hits = 0, splice_fallbacks = 0;
    std::vector<uint8_t> tmpl_prefix, tmpl_suffix;
    size_t tmpl_payload_size = 0;
    // E9: in-place splice state. When armed, the VC-5 encode below wrote its
    // bitstream directly into inplace_buf_ at the prefix offset.
    void*  inplace_buf_ = nullptr;
    bool   inplace_pending_ = false;
    bool   inplace_shared_pending_ = false;
    bool   inplace_pool_pending_ = false;
    bool   local_inplace = false;
    bool   local_inplace_pool = false;
    int    local_inplace_verified_ = 0;
    // C5: the retained container and the last output shipped from it.
    // v0.9: pool_mu_ guards pool_buf_/pool_cap_ because with handoff_pool
    // the WRITER thread returns containers while the encoder thread takes.
    std::mutex pool_mu_;
    void*  pool_buf_ = nullptr;
    size_t pool_cap_ = 0;
    void*  pool_last_out_ = nullptr;
    size_t pool_last_cap_ = 0;

public:
    size_t last_container_capacity() const { return pool_last_cap_; }

    // v0.9: writer-thread return path for handoff_pool.
    void give_container(void* buf, size_t capacity) {
        if (!buf) return;
        void* to_free = nullptr;
        {
            std::lock_guard<std::mutex> lock(pool_mu_);
            if (pool_buf_) to_free = buf;      // one retained is enough
            else { pool_buf_ = buf; pool_cap_ = capacity; }
        }
        if (to_free) allocator_.Free(to_free);
    }
    // H2: the container the worker keeps and rewrites in place.
    bool splice_reuse = false;
    bool shared_reuse = false;
    int shared_reuse_verified_ = 0;
    std::vector<uint8_t> container_;
    bool container_primed_ = false;
    static constexpr int SPLICE_VERIFY_FRAMES = 3;

    // S2 fast path: publish-once, then lock-free reads. Only called when the
    // shared template is already marked ready, so prefix/suffix are immutable.
    bool shared_splice_try(const gpr_buffer& vc5, gpr_buffer& out) {
        if (!g_shared_splice.ready.load(std::memory_order_acquire)) return false;
        if (vc5.size != g_shared_splice.payload_size) return false;
        const auto& pre = g_shared_splice.prefix;
        const auto& suf = g_shared_splice.suffix;
        const size_t total = pre.size() + vc5.size + suf.size();
        out.buffer = allocator_.Alloc(total);
        if (!out.buffer) throw std::runtime_error("shared splice allocation failed");
        out.size = total;
        uint8_t* d = static_cast<uint8_t*>(out.buffer);
        std::memcpy(d, pre.data(), pre.size());
        std::memcpy(d + pre.size(), vc5.buffer, vc5.size);
        std::memcpy(d + pre.size() + vc5.size, suf.data(), suf.size());
        return true;
    }

    bool shared_splice_reuse_try(const gpr_buffer& vc5, gpr_buffer& out) {
        if (!shared_reuse || !g_shared_splice.ready.load(std::memory_order_acquire)) return false;
        if (vc5.size != g_shared_splice.payload_size) return false;
        const auto& pre = g_shared_splice.prefix;
        const auto& suf = g_shared_splice.suffix;
        const size_t total = pre.size() + vc5.size + suf.size();
        if (container_.size() != total) { container_.assign(total, 0); container_primed_ = false; }
        uint8_t* d = container_.data();
        if (!container_primed_) {
            std::memcpy(d, pre.data(), pre.size());
            std::memcpy(d + pre.size() + vc5.size, suf.data(), suf.size());
            container_primed_ = true;
        }
        std::memcpy(d + pre.size(), vc5.buffer, vc5.size);
        out.buffer = d;
        out.size = total;
        if (shared_reuse_verified_ < SPLICE_VERIFY_FRAMES) {
            gpr_buffer trial{nullptr, 0};
            if (!shared_splice_try(vc5, trial)) return false;
            const bool same = trial.size == out.size && std::memcmp(trial.buffer, out.buffer, out.size) == 0;
            if (trial.buffer) allocator_.Free(trial.buffer);
            if (!same) throw std::runtime_error("shared retained GPR splice verification failed");
            ++shared_reuse_verified_;
        }
        return true;
    }

    bool splice_learn(const gpr_buffer& vc5, const gpr_buffer& container) {
        const uint8_t* hay = static_cast<const uint8_t*>(container.buffer);
        const uint8_t* pay = static_cast<const uint8_t*>(vc5.buffer);
        if (!hay || !pay || vc5.size < 64 || container.size <= vc5.size) return false;
        const void* first = memmem(hay, container.size, pay, vc5.size);
        if (!first) return false;
        const size_t off = size_t(static_cast<const uint8_t*>(first) - hay);
        if (memmem(hay + off + 1, container.size - off - 1, pay, vc5.size)) return false;
        tmpl_prefix.assign(hay, hay + off);
        tmpl_suffix.assign(hay + off + vc5.size, hay + container.size);
        tmpl_payload_size = vc5.size;
        return true;
    }

    // Same-size payloads only. A recorder pads VC-5 payloads to bucket sizes
    // (legal in the bitstream) to make every frame spliceable; that policy is
    // the camera side's call. Different size here = SDK path for that frame.
    void splice_build(const gpr_buffer& vc5, gpr_buffer& out) {
        const size_t total = tmpl_prefix.size() + vc5.size + tmpl_suffix.size();
        if (splice_reuse) {
            // Prefix and suffix are identical on every frame, so they are
            // written once and only the payload is refreshed after that.
            if (container_.size() != total) { container_.assign(total, 0); container_primed_ = false; }
            uint8_t* d = container_.data();
            if (!container_primed_) {
                std::memcpy(d, tmpl_prefix.data(), tmpl_prefix.size());
                std::memcpy(d + tmpl_prefix.size() + vc5.size,
                            tmpl_suffix.data(), tmpl_suffix.size());
                container_primed_ = true;
            }
            std::memcpy(d + tmpl_prefix.size(), vc5.buffer, vc5.size);
            out.buffer = d;
            out.size = total;
            return;
        }
        out.buffer = allocator_.Alloc(total);
        if (!out.buffer) throw std::runtime_error("splice allocation failed");
        out.size = total;
        uint8_t* d = static_cast<uint8_t*>(out.buffer);
        std::memcpy(d, tmpl_prefix.data(), tmpl_prefix.size());
        std::memcpy(d + tmpl_prefix.size(), vc5.buffer, vc5.size);
        std::memcpy(d + tmpl_prefix.size() + vc5.size, tmpl_suffix.data(), tmpl_suffix.size());
    }

    /* True when the last output points into the encoder's own container, so
       the caller must not free it. Checked by pointer, which cannot be got
       wrong by a caller that forgets. */
    bool owns_output(const gpr_buffer& b) const {
        return !container_.empty() && b.buffer == container_.data();
    }

    bool arm_inplace_output(const std::vector<uint8_t>& pre, const std::vector<uint8_t>& suf,
                            bool shared_template) {
        const size_t capacity = size_t(vc5_params_.channel_width)
                              * size_t(vc5_params_.channel_height) * 4u * 2u + (1u << 20);
        const size_t total = pre.size() + capacity + suf.size();
        const bool pool_enabled = shared_template
            ? (g_cpu_splice_pool || g_cpu_handoff_pool)
            : local_inplace_pool;
        if (pool_enabled) {
            std::lock_guard<std::mutex> lock(pool_mu_);
            if (pool_buf_ && pool_cap_ >= total) {
                inplace_buf_ = pool_buf_;
                pool_buf_ = nullptr;
            }
        }
        if (!inplace_buf_) inplace_buf_ = allocator_.Alloc(total);
        if (!inplace_buf_) return false;
        pool_last_cap_ = total;
        vc5_pretransformed_encoder_set_output_storage(
            context_, static_cast<uint8_t*>(inplace_buf_) + pre.size(), capacity);
        inplace_pending_ = true;
        inplace_shared_pending_ = shared_template;
        inplace_pool_pending_ = pool_enabled;
        return true;
    }

    void encode(const int16_t* coefficients,
                gpr_buffer& out_gpr, size_t& vc5_bytes) {
        if (out_gpr.buffer) {
            // Ownership-aware: with H2 splice_reuse the previous output
            // points into the retained container_, which the allocator does
            // not own. A raw Free here was a latent double-free for any
            // caller that re-passes the same gpr_buffer across frames.
            if (!owns_output(out_gpr)) allocator_.Free(out_gpr.buffer);
            out_gpr = {nullptr,0};
        }
        gpr_buffer vc5{nullptr,0};
        if (g_cpu_splice_inplace && splice_shared && !splice_reuse &&
            g_shared_splice.ready.load(std::memory_order_acquire)) {
            arm_inplace_output(g_shared_splice.prefix, g_shared_splice.suffix, true);
        } else if (local_inplace && splice_enabled && !splice_shared && !splice_reuse &&
                   splice_active && !tmpl_prefix.empty()) {
            // v1.14: same E9 idea, but with the per-entropy-core local template
            // that won the dual-stage matrix. The VC-5 writer lands directly
            // in the final payload slot; only prefix/suffix remain to write.
            arm_inplace_output(tmpl_prefix, tmpl_suffix, false);
        }
        const CODEC_ERROR error = vc5_pretransformed_encoder_encode(context_, &vc5_params_, coefficients, &vc5);
        if (error != CODEC_ERROR_OKAY)
            throw std::runtime_error("direct VC-5 encode failed: " + std::to_string(int(error)));
        vc5_bytes = vc5.size;
        // The DNG wrapper is a very small part of the frame cost, but the
        // legacy Adobe DNG SDK uses shared global allocator state. Serialize
        // only this sub-millisecond wrapper while VC-5 entropy remains fully
        // parallel across frames.
        {
            const auto wrap_start = Clock::now();
            if (inplace_pending_) {
                // The payload is already inside the final container. Select
                // either the publish-once shared template or this encoder's
                // verified local template; both were byte-checked against SDK.
                inplace_pending_ = false;
                vc5_pretransformed_encoder_set_output_storage(context_, nullptr, 0);
                const bool shared_ip = inplace_shared_pending_;
                inplace_shared_pending_ = false;
                const auto& ip_pre = shared_ip ? g_shared_splice.prefix : tmpl_prefix;
                const auto& ip_suf = shared_ip ? g_shared_splice.suffix : tmpl_suffix;
                const size_t ip_payload = shared_ip ? g_shared_splice.payload_size : tmpl_payload_size;
                if (vc5.size == ip_payload) {
                    uint8_t* d = static_cast<uint8_t*>(inplace_buf_);
                    std::memcpy(d, ip_pre.data(), ip_pre.size());
                    std::memcpy(d + ip_pre.size() + vc5.size, ip_suf.data(), ip_suf.size());
                    out_gpr.buffer = inplace_buf_;
                    out_gpr.size = ip_pre.size() + vc5.size + ip_suf.size();
                    pool_last_out_ = inplace_pool_pending_ ? inplace_buf_ : nullptr;
                    inplace_pool_pending_ = false;
                    // Local in-place is equivalent to the already verified
                    // copying splice. Re-check the first three direct frames
                    // against that local construction before trusting it.
                    if (!shared_ip && local_inplace_verified_ < SPLICE_VERIFY_FRAMES) {
                        gpr_buffer trial{nullptr,0};
                        splice_build(vc5, trial);
                        const bool same = trial.size == out_gpr.size &&
                            std::memcmp(trial.buffer, out_gpr.buffer, out_gpr.size) == 0;
                        free_buffer(trial);
                        if (!same) throw std::runtime_error("local in-place GPR splice verification failed");
                        ++local_inplace_verified_;
                        if (local_inplace_verified_ == SPLICE_VERIFY_FRAMES)
                            std::cout << "CPU_GPR_LOCAL_INPLACE_VERIFY bitexact=YES frames=" << SPLICE_VERIFY_FRAMES << " path=int16\n";
                    }
                    inplace_buf_ = nullptr;
                    ++splice_hits;
                    dng_wrap_ns += std::chrono::duration_cast<Ns>(Clock::now() - wrap_start).count();
                    return;
                }
                // Size drifted from the template: wrap through the SDK using
                // the payload where it lies, then drop our container.
                ++splice_fallbacks;
                inplace_pool_pending_ = false;
                static std::mutex dng_wrap_mutex_ip;
                std::lock_guard<std::mutex> lock(dng_wrap_mutex_ip);
                const bool ok = gpr_convert_preencoded_vc5_to_gpr(&allocator_, &gpr_params_, &vc5, &out_gpr);
                allocator_.Free(inplace_buf_);
                inplace_buf_ = nullptr;
                if (!ok)
                    throw std::runtime_error("direct pre-encoded VC-5 to GPR container write failed");
                dng_wrap_ns += std::chrono::duration_cast<Ns>(Clock::now() - wrap_start).count();
                return;
            }
            if (splice_shared && shared_splice_reuse_try(vc5, out_gpr)) {
                ++splice_hits;
            } else if (splice_shared && shared_splice_try(vc5, out_gpr)) {
                ++splice_hits;
            } else if (splice_shared) {
                // Learn/verify the one shared template under the lock.
                std::lock_guard<std::mutex> lock(g_shared_splice.mu);
                if (!gpr_convert_preencoded_vc5_to_gpr(&allocator_, &gpr_params_, &vc5, &out_gpr))
                    throw std::runtime_error("direct pre-encoded VC-5 to GPR container write failed");
                if (!g_shared_splice.failed.load(std::memory_order_relaxed) &&
                    !g_shared_splice.ready.load(std::memory_order_relaxed)) {
                    if (g_shared_splice.prefix.empty()) {
                        if (splice_learn(vc5, out_gpr)) {
                            g_shared_splice.prefix = tmpl_prefix;
                            g_shared_splice.suffix = tmpl_suffix;
                            g_shared_splice.payload_size = tmpl_payload_size;
                            g_shared_splice.verified = 1;
                        } else g_shared_splice.failed.store(true, std::memory_order_relaxed);
                    } else if (vc5.size == g_shared_splice.payload_size) {
                        gpr_buffer trial{nullptr, 0};
                        const size_t total = g_shared_splice.prefix.size() + vc5.size
                                           + g_shared_splice.suffix.size();
                        trial.buffer = allocator_.Alloc(total); trial.size = total;
                        uint8_t* d = static_cast<uint8_t*>(trial.buffer);
                        std::memcpy(d, g_shared_splice.prefix.data(), g_shared_splice.prefix.size());
                        std::memcpy(d + g_shared_splice.prefix.size(), vc5.buffer, vc5.size);
                        std::memcpy(d + g_shared_splice.prefix.size() + vc5.size,
                                    g_shared_splice.suffix.data(), g_shared_splice.suffix.size());
                        const bool same = trial.size == out_gpr.size &&
                            std::memcmp(trial.buffer, out_gpr.buffer, trial.size) == 0;
                        free_buffer(trial);
                        if (!same) g_shared_splice.failed.store(true, std::memory_order_relaxed);
                        else if (++g_shared_splice.verified >= SPLICE_VERIFY_FRAMES)
                            g_shared_splice.ready.store(true, std::memory_order_release);
                    }
                }
            } else if (splice_active && vc5.size == tmpl_payload_size) {
                splice_build(vc5, out_gpr);   // no SDK, no global state, NO MUTEX
                ++splice_hits;
            } else {
                if (splice_active) ++splice_fallbacks;
                static std::mutex dng_wrap_mutex;
                std::lock_guard<std::mutex> lock(dng_wrap_mutex);
                if (!gpr_convert_preencoded_vc5_to_gpr(&allocator_, &gpr_params_, &vc5, &out_gpr))
                    throw std::runtime_error("direct pre-encoded VC-5 to GPR container write failed");
                if (splice_enabled && !splice_failed && !splice_active) {
                    if (tmpl_prefix.empty()) {
                        if (!splice_learn(vc5, out_gpr)) splice_failed = true;
                        else splice_verified_frames = 1;
                    } else if (vc5.size == tmpl_payload_size) {
                        // Byte-for-byte proof against the SDK's own output, or
                        // the splice is dead for the whole run. No partial trust.
                        gpr_buffer trial{nullptr, 0};
                        splice_build(vc5, trial);
                        const bool same = trial.size == out_gpr.size &&
                            std::memcmp(trial.buffer, out_gpr.buffer, trial.size) == 0;
                        free_buffer(trial);
                        if (!same) splice_failed = true;
                        else if (++splice_verified_frames >= SPLICE_VERIFY_FRAMES)
                            splice_active = true;
                    }
                }
            }
            dng_wrap_ns += elapsed_ns(wrap_start);
        }
        if (!out_gpr.buffer || out_gpr.size < 8)
            throw std::runtime_error("direct GPR writer returned an empty output");
        const uint8_t* h=static_cast<const uint8_t*>(out_gpr.buffer);
        const bool tiff=(h[0]=='I'&&h[1]=='I'&&h[2]==42&&h[3]==0) ||
                        (h[0]=='M'&&h[1]=='M'&&h[2]==0&&h[3]==42);
        if (!tiff) throw std::runtime_error("direct output is not a TIFF/DNG GPR container");
    }

    void encode_levels(const std::array<const int16_t*, MAX_WAVELET_COUNT>& levels,
                       gpr_buffer& out_gpr, size_t& vc5_bytes) {
        for (const int16_t* level : levels)
            if (!level) throw std::runtime_error("null fused coefficient level buffer");
        if (out_gpr.buffer) {
            if (!owns_output(out_gpr)) allocator_.Free(out_gpr.buffer);   // H2-safe
            out_gpr = {nullptr, 0};
        }
        gpr_buffer vc5{nullptr, 0};
        const CODEC_ERROR error = vc5_pretransformed_encoder_encode_levels(
            context_, &vc5_params_, levels.data(), &vc5);
        if (error != CODEC_ERROR_OKAY)
            throw std::runtime_error("direct fused-level VC-5 encode failed: " +
                                     std::to_string(int(error)));
        vc5_bytes = vc5.size;
        {
            static std::mutex dng_wrap_mutex;
            std::lock_guard<std::mutex> lock(dng_wrap_mutex);
            if (!gpr_convert_preencoded_vc5_to_gpr(&allocator_, &gpr_params_, &vc5, &out_gpr))
                throw std::runtime_error("direct fused-level VC-5 to GPR container write failed");
        }
        if (!out_gpr.buffer || out_gpr.size < 8)
            throw std::runtime_error("direct fused-level GPR writer returned an empty output");
        const uint8_t* h = static_cast<const uint8_t*>(out_gpr.buffer);
        const bool tiff = (h[0]=='I' && h[1]=='I' && h[2]==42 && h[3]==0) ||
                          (h[0]=='M' && h[1]=='M' && h[2]==0 && h[3]==42);
        if (!tiff) throw std::runtime_error("direct fused-level output is not a TIFF/DNG GPR container");
    }

    void encode_hybrid(const vc5_pretransformed_hybrid_frame& frame,
                       gpr_buffer& out_gpr, size_t& vc5_bytes) {
        if (out_gpr.buffer) {
            if (!owns_output(out_gpr)) allocator_.Free(out_gpr.buffer);   // H2-safe
            out_gpr = {nullptr,0};
        }
        gpr_buffer vc5{nullptr,0};
        if (g_cpu_splice_inplace && splice_shared && !splice_reuse &&
            g_shared_splice.ready.load(std::memory_order_acquire)) {
            arm_inplace_output(g_shared_splice.prefix, g_shared_splice.suffix, true);
        } else if (local_inplace && splice_enabled && !splice_shared && !splice_reuse &&
                   splice_active && !tmpl_prefix.empty()) {
            arm_inplace_output(tmpl_prefix, tmpl_suffix, false);
        }
        const CODEC_ERROR error = vc5_pretransformed_encoder_encode_hybrid(
            context_, &vc5_params_, &frame, &vc5);
        if (error != CODEC_ERROR_OKAY)
            throw std::runtime_error("direct hybrid VC-5 encode failed: " + std::to_string(int(error)));
        vc5_bytes = vc5.size;
        {
            // Same splice-aware wrap as encode(): the container template does
            // not care which entropy path produced the payload bytes.
            const auto wrap_start = Clock::now();
            if (inplace_pending_) {
                inplace_pending_ = false;
                vc5_pretransformed_encoder_set_output_storage(context_, nullptr, 0);
                const bool shared_ip = inplace_shared_pending_;
                inplace_shared_pending_ = false;
                const auto& ip_pre = shared_ip ? g_shared_splice.prefix : tmpl_prefix;
                const auto& ip_suf = shared_ip ? g_shared_splice.suffix : tmpl_suffix;
                const size_t ip_payload = shared_ip ? g_shared_splice.payload_size : tmpl_payload_size;
                if (vc5.size == ip_payload) {
                    uint8_t* d = static_cast<uint8_t*>(inplace_buf_);
                    std::memcpy(d, ip_pre.data(), ip_pre.size());
                    std::memcpy(d + ip_pre.size() + vc5.size, ip_suf.data(), ip_suf.size());
                    out_gpr.buffer = inplace_buf_;
                    out_gpr.size = ip_pre.size() + vc5.size + ip_suf.size();
                    pool_last_out_ = inplace_pool_pending_ ? inplace_buf_ : nullptr;
                    inplace_pool_pending_ = false;
                    if (!shared_ip && local_inplace_verified_ < SPLICE_VERIFY_FRAMES) {
                        gpr_buffer trial{nullptr,0};
                        splice_build(vc5, trial);
                        const bool same = trial.size == out_gpr.size &&
                            std::memcmp(trial.buffer, out_gpr.buffer, out_gpr.size) == 0;
                        free_buffer(trial);
                        if (!same) throw std::runtime_error("local in-place hybrid GPR splice verification failed");
                        ++local_inplace_verified_;
                        if (local_inplace_verified_ == SPLICE_VERIFY_FRAMES)
                            std::cout << "CPU_GPR_LOCAL_INPLACE_VERIFY bitexact=YES frames=" << SPLICE_VERIFY_FRAMES << " path=hybrid\n";
                    }
                    inplace_buf_ = nullptr;
                    ++splice_hits;
                    dng_wrap_ns += std::chrono::duration_cast<Ns>(Clock::now() - wrap_start).count();
                    return;
                }
                ++splice_fallbacks;
                inplace_pool_pending_ = false;
                static std::mutex dng_wrap_mutex_ip_hybrid;
                std::lock_guard<std::mutex> lock(dng_wrap_mutex_ip_hybrid);
                const bool ok = gpr_convert_preencoded_vc5_to_gpr(&allocator_, &gpr_params_, &vc5, &out_gpr);
                allocator_.Free(inplace_buf_);
                inplace_buf_ = nullptr;
                if (!ok)
                    throw std::runtime_error("direct hybrid pre-encoded VC-5 to GPR container write failed");
                dng_wrap_ns += std::chrono::duration_cast<Ns>(Clock::now() - wrap_start).count();
                return;
            }
            if (splice_shared && shared_splice_reuse_try(vc5, out_gpr)) {
                ++splice_hits;
            } else if (splice_shared && shared_splice_try(vc5, out_gpr)) {
                ++splice_hits;
            } else if (splice_shared) {
                // Learn/verify the one shared template under the lock.
                std::lock_guard<std::mutex> lock(g_shared_splice.mu);
                if (!gpr_convert_preencoded_vc5_to_gpr(&allocator_, &gpr_params_, &vc5, &out_gpr))
                    throw std::runtime_error("direct hybrid pre-encoded VC-5 to GPR container write failed");
                if (!g_shared_splice.failed.load(std::memory_order_relaxed) &&
                    !g_shared_splice.ready.load(std::memory_order_relaxed)) {
                    if (g_shared_splice.prefix.empty()) {
                        if (splice_learn(vc5, out_gpr)) {
                            g_shared_splice.prefix = tmpl_prefix;
                            g_shared_splice.suffix = tmpl_suffix;
                            g_shared_splice.payload_size = tmpl_payload_size;
                            g_shared_splice.verified = 1;
                        } else g_shared_splice.failed.store(true, std::memory_order_relaxed);
                    } else if (vc5.size == g_shared_splice.payload_size) {
                        gpr_buffer trial{nullptr, 0};
                        const size_t total = g_shared_splice.prefix.size() + vc5.size
                                           + g_shared_splice.suffix.size();
                        trial.buffer = allocator_.Alloc(total); trial.size = total;
                        uint8_t* d = static_cast<uint8_t*>(trial.buffer);
                        std::memcpy(d, g_shared_splice.prefix.data(), g_shared_splice.prefix.size());
                        std::memcpy(d + g_shared_splice.prefix.size(), vc5.buffer, vc5.size);
                        std::memcpy(d + g_shared_splice.prefix.size() + vc5.size,
                                    g_shared_splice.suffix.data(), g_shared_splice.suffix.size());
                        const bool same = trial.size == out_gpr.size &&
                            std::memcmp(trial.buffer, out_gpr.buffer, trial.size) == 0;
                        free_buffer(trial);
                        if (!same) g_shared_splice.failed.store(true, std::memory_order_relaxed);
                        else if (++g_shared_splice.verified >= SPLICE_VERIFY_FRAMES)
                            g_shared_splice.ready.store(true, std::memory_order_release);
                    }
                }
            } else if (splice_active && vc5.size == tmpl_payload_size) {
                splice_build(vc5, out_gpr);
                ++splice_hits;
            } else {
                if (splice_active) ++splice_fallbacks;
                static std::mutex dng_wrap_mutex;
                std::lock_guard<std::mutex> lock(dng_wrap_mutex);
                if (!gpr_convert_preencoded_vc5_to_gpr(&allocator_, &gpr_params_, &vc5, &out_gpr))
                    throw std::runtime_error("direct hybrid pre-encoded VC-5 to GPR container write failed");
                if (splice_enabled && !splice_failed && !splice_active) {
                    if (tmpl_prefix.empty()) {
                        if (!splice_learn(vc5, out_gpr)) splice_failed = true;
                        else splice_verified_frames = 1;
                    } else if (vc5.size == tmpl_payload_size) {
                        gpr_buffer trial{nullptr, 0};
                        splice_build(vc5, trial);
                        const bool same = trial.size == out_gpr.size &&
                            std::memcmp(trial.buffer, out_gpr.buffer, trial.size) == 0;
                        free_buffer(trial);
                        if (!same) splice_failed = true;
                        else if (++splice_verified_frames >= SPLICE_VERIFY_FRAMES)
                            splice_active = true;
                    }
                }
            }
            dng_wrap_ns += elapsed_ns(wrap_start);
        }
        if (!out_gpr.buffer || out_gpr.size < 8)
            throw std::runtime_error("direct hybrid GPR writer returned an empty output");
        const uint8_t* h=static_cast<const uint8_t*>(out_gpr.buffer);
        const bool tiff=(h[0]=='I'&&h[1]=='I'&&h[2]==42&&h[3]==0) ||
                        (h[0]=='M'&&h[1]=='M'&&h[2]==0&&h[3]==42);
        if (!tiff) throw std::runtime_error("direct hybrid output is not a TIFF/DNG GPR container");
    }

    // E8: register (or clear) the nonzero-mask sidecar for the coefficient
    // buffer the next encode call will read.
    void set_sidecar(const uint8_t* mask_base, const void* coeff_base) {
        vc5_pretransformed_encoder_set_sidecar(context_, mask_base, coeff_base);
    }

    // Return a direct-output container to this encoder's private pool without
    // touching ordinary allocator-owned outputs.  The adaptive staged path
    // already called free_buffer() directly, but the normal 3-worker path
    // routes outputs through retire_buffer().  Exposing this narrow recycle
    // hook makes shared in-place GPR output equally zero-allocation in both
    // architectures instead of silently free/malloc cycling on 3 workers.
    bool recycle_pooled_buffer(gpr_buffer& buffer) {
        if ((local_inplace_pool || g_cpu_splice_pool || g_cpu_handoff_pool) &&
            buffer.buffer && buffer.buffer == pool_last_out_) {
            give_container(buffer.buffer, pool_last_cap_);
            pool_last_out_ = nullptr;
            buffer = {nullptr, 0};
            return true;
        }
        return false;
    }

    void free_buffer(gpr_buffer& buffer) {
        if (recycle_pooled_buffer(buffer)) return;
        if (buffer.buffer && !owns_output(buffer)) allocator_.Free(buffer.buffer);
        buffer = {nullptr,0};
    }

private:
    gpr_allocator allocator_{};
    gpr_parameters gpr_params_{};
    vc5_pretransformed_parameters vc5_params_{};
    vc5_pretransformed_encoder* context_=nullptr;
};

// ===========================================================================
// v1.7.55 --execution cpu-gpr : the whole GPR encode, on the CPU, no Vulkan.
//
// Measured on a CM5 at 3840x2160, the fused cascade does RAW copy + split +
// three wavelet levels at 30.3 fps on ONE A76 core and 58.1 fps on three. The
// wavelet is no longer the constraint, so the useful question stopped being
// "can the CPU do the transform" and became "can the CPU do the whole frame".
//
// This path answers that directly. N workers, each owning one complete frame
// end to end:
//
//     RAW16 copy -> GP-Log2 split -> fused 3-level wavelet -> VC-5 entropy
//     -> GPR/DNG container
//
// Frame-parallel, not stage-parallel: each worker keeps its own ~215 KiB of
// cascade rings hot in its private 512 KiB L2 and never hands a buffer to
// another core. Stage-parallel would push every intermediate through L3.
//
// No Vulkan instance is created. That is the point -- it removes the driver,
// the queue, the fences and, on a thermally shared SoC, the GPU's power draw.
// ===========================================================================

struct CpuGprWorkerStats {
    long frames = 0;
    int64_t copy_ns = 0, wavelet_ns = 0, entropy_ns = 0, writer_ns = 0;
    long splice_hits_out = 0, splice_fb_out = 0;
    bool splice_active_out = false, splice_failed_out = false;
    long hyb_frames = 0;
    int  hyb_verified = 0;
    bool hyb_failed = false;
    uint64_t hyb_int8_tiles = 0, hyb_total_tiles = 0;
    uint64_t payload_bytes = 0, packet_bytes = 0;
    int64_t dng_ns = 0;
    long wavelet_assist_frames = 0;
    int64_t wavelet_assist_wait_ns = 0;
    int64_t wavelet_assist_publish_wait_ns = 0;
    // v1.16.3: winning-stack wavelet burst telemetry. Each worker owns this
    // vector, so recording intervals has no cross-core lock.
    std::vector<std::pair<int64_t,int64_t>> wave_intervals_ns;
    int64_t wavelet_sync_wait_ns = 0;
    int64_t wavelet_exec_ns = 0;
    // Complete-frame wall time: claim to finished GPR container. This is the
    // "start to finish" number; fps is the aggregate the pipeline sustains.
    std::vector<double> latency_ms;
};


// ===========================================================================
// --execution capture : the recorder, not the encoder.
//
// Every other mode answers "how fast can this encode". A camera asks a
// different question: the sensor delivers frames at a FIXED rate whether the
// encoder is ready or not, so what matters is whether the buffers survive.
//
//   sensor --> 12-frame ring --> N encoder workers --> GPR
//                  |
//                  +--> overflow RAM buffer, when the ring is full
//
// If the encoder sustains the rate the ring never fills and the overflow is
// never touched. If it cannot, the shortfall accumulates in the overflow at
// (target_fps - encode_fps) x 16.6 MB per second, and the run ends when that
// buffer is exhausted. THAT TIME IS THE ANSWER: the maximum clip length at
// that mode and frame rate on that board.
//
// Overflow sizing follows the product policy -- installed RAM less 1 GB for
// the system: 1 GB board gets no overflow, 2/4/8/16 GB get 1/3/7/15 GB. The
// benchmark additionally clamps to what is actually available right now and
// says so, because a 7 GB allocation on a running 8 GB board will not stand.
// ===========================================================================
static long capture_policy_overflow_mb() {
    long mem_total_kb = 0, mem_avail_kb = 0;
    if (std::ifstream mi("/proc/meminfo"); mi) {
        std::string key; long value; std::string unit;
        while (mi >> key >> value >> unit) {
            if (key == "MemTotal:") mem_total_kb = value;
            else if (key == "MemAvailable:") mem_avail_kb = value;
        }
    }
    if (mem_total_kb <= 0) return 0;
    // Round the installed size to the nearest power-of-two GiB the board is
    // sold as (a "4 GB" Pi reports ~3.9 GiB), then keep 1 GiB for the system.
    const double gib = double(mem_total_kb) / (1024.0 * 1024.0);
    long rounded = 1;
    for (const long candidate : {1L, 2L, 4L, 8L, 16L, 32L})
        if (gib >= double(candidate) * 0.9) rounded = candidate;
    long policy_mb = (rounded - 1) * 1024;
    if (policy_mb < 0) policy_mb = 0;
    // Clamp to what is genuinely free, less a 512 MiB working margin.
    if (mem_avail_kb > 0) {
        const long avail_mb = mem_avail_kb / 1024 - 512;
        if (avail_mb < policy_mb) policy_mb = avail_mb > 0 ? avail_mb : 0;
    }
    return policy_mb;
}

static void announce_cpu_kernel(const Options& o, const char* pipeline) {
    // v1.0: membership is DERIVED from live state via a table, never
    // hardcoded. The previous version asserted "stride_split,split16,
    // stnp_reg" unconditionally, so after the E-mode search dropped
    // split16 and stnp_reg the binary reported members it was not running.
    // A partial stack must be visible in the first line of output rather
    // than hiding in the fps.
    const bool v2 = o.cpu_v2_kernel;
    const std::pair<const char*, bool> members[] = {
        { "sidecar_zskip",  o.cpu_sidecar_zskip },
        { "stride_split",   v2 },                     // inherent to the v2 kernel
        { "handoff_pool",   cinepi_handoff_pool_active() },
        { "vle_signlut",    cinepi_vle_signlut != 0 },
        { "vle_scan8",      cinepi_vle_scan8 != 0 },
        // v1.2: acc64 was enabled but not reported. It is what actually
        // gates the sidecar (encoder.c:2333), so leaving it out of this
        // list is the same invisibility that hid the scan8 gap for weeks.
        { "vle_acc64",      cinepi_vle_acc64 != 0 },
        { "input_prefetch", o.cpu_input_prefetch },
        { "vle_sidecar",    o.cpu_sidecar },
    };
    const int total = int(sizeof(members) / sizeof(members[0]));
    std::string list;
    int count = 0;
    for (const auto& m : members) {
        if (!v2 || !m.second) continue;
        if (!list.empty()) list += ",";
        list += m.first;
        ++count;
    }
    std::cout << "CPU_KERNEL pipeline=" << pipeline
              << " kernel=" << (v2 ? "v2-winner" : "shipped-fused")
              << " members=" << (list.empty() ? "-" : list)
              << " count=" << count << "/" << total
              << (v2 && cinepi_lowpass_bulk ? " +lowpass_bulk(not-in-winner)" : "")
              << " inframe_bits=" << o.compand_inframe_bits << "\n";
}

static int run_capture_simulation(const Options& o, const ModeSpec* mode,
                                  const std::vector<uint16_t>& src,
                                  const std::vector<uint16_t>& lut) {
    cpu_gpr_apply_entropy_assist_defaults(o);
    announce_cpu_kernel(o, "capture");
    const int pw = o.width / 2, ph = o.height / 2;
    const size_t row_stride = size_t(pw);
    const size_t plane_stride = row_stride * size_t(ph);
    const size_t frame_elems = src.size();
    const size_t frame_bytes = frame_elems * sizeof(uint16_t);
    const int requested_workers = o.cpu_gpr_threads;
    const bool fused = o.cpu_wavelet_fused && cpu_fused_geometry_ok(o);
    const bool wavelet_assist_enabled = cpu_gpr_wavelet_assist_requested(o) &&
        cpu_fused_geometry_ok(o) && (o.cpu_v2_kernel || (fused && o.cpu_split_fused));
    const int workers = cpu_gpr_primary_worker_count(o, wavelet_assist_enabled);

    long policy_mb = capture_policy_overflow_mb();
    const bool user_capped = o.overflow_mb >= 0;
    const long cap_mb = user_capped ? o.overflow_mb : policy_mb;
    const size_t overflow_cap_frames =
        frame_bytes ? size_t((uint64_t(cap_mb) * 1024ull * 1024ull) / frame_bytes) : 0;

    if (o.sensor_max_fps > 0.0 && o.target_fps > o.sensor_max_fps + 0.01)
        std::cout << "CAPTURE_OVER_SENSOR target_fps=" << o.target_fps
                  << " sensor_max_fps=" << o.sensor_max_fps
                  << " note=this-rate-models-a-sensor-that-cannot-deliver-it"
                     "-the-encoder-result-is-real-the-capture-scenario-is-not\n";
    std::cout << "CAPTURE_CONFIG target_fps=" << o.target_fps
              << " seconds=" << o.capture_seconds
              << " workers=" << requested_workers
              << " primary_workers=" << workers
              << " wavelet_assist=" << (wavelet_assist_enabled ? "legacy-diagnostic" : "off")
              << " core0_entropy_assist=" << (requested_workers == 4 ? "SB8x4" : "off")
              << " mode=" << (o.mode.empty() ? "base-q" : o.mode)
              << " ring_frames=" << o.ring_frames
              << " frame_MiB=" << (double(frame_bytes) / 1048576.0)
              << " overflow_cap_MiB=" << cap_mb
              << " overflow_cap_frames=" << overflow_cap_frames
              << (user_capped ? " source=user" : " source=ram-policy")
              << " dma_copy=" << (o.capture_dma_copy ? "on" : "off") << "\n";
    if (!user_capped && policy_mb == 0)
        std::cout << "CAPTURE_NOTE overflow is zero: either a 1 GB board, or too "
                     "little free RAM right now. Only the ring absorbs jitter.\n";

    // ---- buffer pool -------------------------------------------------------
    std::mutex mu;
    std::condition_variable cv_work, cv_free;
    std::vector<std::unique_ptr<uint16_t[]>> owned;      // every allocation
    std::vector<uint16_t*> free_list;                    // ready to fill
    std::deque<uint16_t*> pending;                       // captured, not encoded
    std::atomic<uint64_t> cap_bytes{0};                  // total encoded bytes
    owned.reserve(size_t(o.ring_frames) + overflow_cap_frames);
    for (int i = 0; i < o.ring_frames; ++i) {
        owned.emplace_back(new uint16_t[frame_elems]);
        /* v0.30: pre-fill every slot with the source frame instead of
           zeroing it. This faults the pages in exactly as before, but it
           also means the encoder always sees REAL image data even when the
           per-frame producer copy is off -- encoding zeroed buffers would
           be trivially compressible and would flatter the result. */
        std::memcpy(owned.back().get(), src.data(), frame_bytes);
        free_list.push_back(owned.back().get());
    }
    // overflow_used counts buffers ever DRAWN from the overflow pool. It is a
    // high-water mark and never falls, because once a worker finishes with a
    // buffer it goes back on the free list and is indistinguishable from a
    // ring slot. Reporting it as the live occupancy made a buffer that had
    // long since drained look permanently full. Live occupancy is the queue
    // depth beyond the ring; this stays for the peak.
    size_t overflow_used = 0, peak_queue = 0, peak_overflow = 0;
    std::vector<uint16_t*> ready_pool;
    std::condition_variable pool_cv;
    size_t pool_reserved = 0, pool_misses = 0;
    constexpr size_t READY_TARGET = 4;   // only used by the lazy path

    // Reserve the take's buffer BEFORE the clock starts, exactly as a recorder
    // does when you hit record. Once committed, a frame spilling into the
    // overflow costs the same as one landing in the ring: a single DMA write
    // into memory that is already faulted in. Allocating per spilled frame
    // instead made overflow mode look like extra compute when the only real
    // difference is which address the sensor writes to.
    double reserve_s = 0.0;
    if (o.capture_prealloc && overflow_cap_frames > 0) {
        const auto t_res = Clock::now();
        for (size_t i = 0; i < overflow_cap_frames; ++i) {
            uint16_t* p = nullptr;
            try { p = new uint16_t[frame_elems]; }
            catch (const std::bad_alloc&) { break; }   // take what we can get
            std::memcpy(p, src.data(), frame_bytes);   // commit pages, real data
            owned.emplace_back(p);
            ready_pool.push_back(p);
            ++pool_reserved;
        }
        reserve_s = std::chrono::duration<double>(Clock::now() - t_res).count();
        std::cout << "CAPTURE_RESERVE frames=" << pool_reserved
                  << " MiB=" << (double(pool_reserved) * double(frame_bytes) / 1048576.0)
                  << " seconds=" << reserve_s
                  << " note=committed-before-the-clock-starts-and-not-charged-to-the-run\n";
        if (pool_reserved < overflow_cap_frames)
            std::cout << "CAPTURE_RESERVE_SHORT wanted=" << overflow_cap_frames
                      << " got=" << pool_reserved
                      << " note=the-cap-is-now-what-was-actually-reserved\n";
    }
    bool stop_full = false;
    std::atomic<bool> stop{false};
    long long captured = 0, encoded = 0;
    std::exception_ptr worker_error;

    if (wavelet_assist_enabled)
        verify_gd_wavelet_assist(o, mode, src, lut, plane_stride, row_stride,
                                 o.cpu_v2_kernel);

    struct CaptureCoeffBuffers {
        std::vector<int16_t> coeff;
        std::vector<uint8_t> sidecar;
    };
    std::vector<std::unique_ptr<CaptureCoeffBuffers>> capture_coeffs;
    capture_coeffs.reserve(size_t(workers));
    for (int w = 0; w < workers; ++w) {
        auto b = std::make_unique<CaptureCoeffBuffers>();
        b->coeff.assign(plane_stride * 4u, int16_t(0));
        if (o.cpu_v2_kernel && o.cpu_sidecar)
            b->sidecar.assign(plane_stride * 4u / 8u, 0u);
        capture_coeffs.emplace_back(std::move(b));
    }

    std::unique_ptr<GdWaveletAssistant> wavelet_assistant;
    if (wavelet_assist_enabled)
        wavelet_assistant = std::make_unique<GdWaveletAssistant>(
            o, mode, plane_stride, row_stride, o.cpu_v2_kernel);
    std::atomic<int64_t> wavelet_assist_wait_ns{0};
    std::atomic<int64_t> wavelet_assist_publish_wait_ns{0};
    std::atomic<long long> wavelet_assist_frames{0};

    const auto start = Clock::now();
    const auto deadline = start + std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(o.capture_seconds));
    double fill_time_s = -1.0;

    // ---- encoder workers ---------------------------------------------------
    auto worker = [&](int id) {
        cpu_gpr_place_worker(o, id, wavelet_assist_enabled);
        try {
            auto& db = *capture_coeffs[size_t(id)];
            CpuFusedContext ctx;
            DirectGprEncoder enc(o, mode, row_stride, plane_stride);
            enc.splice_enabled = o.cpu_gpr_dng_splice;
            enc.splice_shared = o.cpu_gpr_dng_splice && o.cpu_gpr_splice_shared;
            std::vector<int16_t> planes;
            if (!(fused && o.cpu_split_fused)) planes.assign(plane_stride * 4u, int16_t(0));
            V2Frame v2ctx;
            g_v2_sidecar_zskip = o.cpu_sidecar_zskip;
            g_v2_input_prefetch = o.cpu_input_prefetch;
            for (;;) {
                uint16_t* buf = nullptr;
                {
                    std::unique_lock<std::mutex> lock(mu);
                    cv_work.wait(lock, [&]{ return !pending.empty() || stop.load(); });
                    if (pending.empty()) break;
                    buf = pending.front(); pending.pop_front();
                }
                auto& coeff = db.coeff;
                auto& sidecar = db.sidecar;
                const bool use_sc = o.cpu_v2_kernel && o.cpu_sidecar && !sidecar.empty();
                GdWaveletAssistant::Ticket assist_ticket{};
                int64_t assist_publish_wait_ns = 0;
                if (wavelet_assistant)
                    assist_ticket = wavelet_assistant->try_begin(
                        coeff.data(), use_sc ? sidecar.data() : nullptr, use_sc);
                auto publish_gd = [&](const int16_t* gd) {
                    assist_publish_wait_ns += wavelet_assistant->publish_row(assist_ticket, gd);
                };

                if (o.cpu_v2_kernel) {
                    V2Sidecar sc{};
                    if (use_sc) { sc.mask_base = sidecar.data(); sc.coeff_base = coeff.data(); }
                    if (assist_ticket)
                        v2_fused_primary3_rows(use_sc ? &sc : nullptr, o, mode, buf,
                                               size_t(o.width), 0, 0, lut, coeff.data(),
                                               plane_stride, row_stride,
                                               SplitKind::Shipped, EmitKind::RegDirect, v2ctx,
                                               publish_gd);
                    else
                        v2_fused_frame(use_sc ? &sc : nullptr, o, mode, buf,
                                       size_t(o.width), 0, 0, lut, coeff.data(),
                                       plane_stride, row_stride,
                                       SplitKind::Shipped, EmitKind::RegDirect, v2ctx);
                }
                else if (fused && o.cpu_split_fused) {
                    if (assist_ticket)
                        fused_split_transform_primary3_rows(buf, lut.data(), o.width, o.height,
                                                            o.bayer == "gbrg",
                                                            o.true_12bit ? 0 : 1, 4096,
                                                            coeff.data(), plane_stride, row_stride,
                                                            cpu_quant_table(o, mode), ctx,
                                                            publish_gd);
                    else
                        cpu_fused_frame_from_raw(o, mode, buf, lut, coeff.data(),
                                                 plane_stride, row_stride, ctx);
                } else {
                    split_compand(o, buf, lut, planes.data(), plane_stride);
                    if (fused) cpu_fused_frame_from_planes(o, mode, planes.data(), coeff.data(),
                                                           plane_stride, row_stride, ctx);
                    else {
                        std::memcpy(coeff.data(), planes.data(), coeff.size() * sizeof(int16_t));
                        cpu_transform_tail_inplace(o, mode, coeff.data(), plane_stride, row_stride, 1);
                    }
                }

                if (assist_ticket) {
                    wavelet_assist_wait_ns.fetch_add(wavelet_assistant->wait(assist_ticket),
                                                     std::memory_order_relaxed);
                    wavelet_assist_publish_wait_ns.fetch_add(assist_publish_wait_ns,
                                                             std::memory_order_relaxed);
                    wavelet_assist_frames.fetch_add(1, std::memory_order_relaxed);
                }

                enc.set_sidecar(use_sc ? sidecar.data() : nullptr, coeff.data());
                gpr_buffer out{nullptr, 0}; size_t vc5_bytes = 0;
                enc.encode(coeff.data(), out, vc5_bytes);
                cap_bytes.fetch_add(out.size, std::memory_order_relaxed);
                enc.free_buffer(out);
                {
                    std::lock_guard<std::mutex> lock(mu);
                    free_list.push_back(buf);
                    ++encoded;
                }
                cv_free.notify_one();
            }
        } catch (...) {
            std::lock_guard<std::mutex> lock(mu);
            if (!worker_error) worker_error = std::current_exception();
            stop.store(true);
            cv_work.notify_all();
        }
    };
    std::vector<std::thread> pool;
    for (int w = 0; w < workers; ++w) pool.emplace_back(worker, w);

    // ---- buffer reserver ---------------------------------------------------
    // Only run the reserver if there is a core going spare for it.
    //
    // It was added to keep 16.6 MB allocations off the producer's critical
    // path, and it does -- but it is another thread, and with N workers plus
    // the producer plus the monitor there may be nothing left to run it on.
    // On a 4-core CM5 with 3 workers at 48 fps it starved the producer down to
    // 46.6 of 48 fps, which tripped the sensor-validity gate and cost about
    // 5% of encode throughput. A reserver that steals from the sensor is worse
    // than the inline allocation it replaces.
    // Only needed when the buffer was NOT reserved up front.
    const unsigned hw = std::max(1u, std::thread::hardware_concurrency());
    const bool reserver_fits = !o.capture_prealloc && (unsigned(workers) + 2u) <= hw;
    std::thread reserver;
    if (overflow_cap_frames > 0 && reserver_fits) {
        reserver = std::thread([&]{
            cpu_gpr_place_aux(o);
            for (;;) {
                {
                    std::unique_lock<std::mutex> lock(mu);
                    pool_cv.wait(lock, [&]{
                        return stop.load() || (ready_pool.size() < READY_TARGET &&
                               pool_reserved + overflow_used < overflow_cap_frames);
                    });
                    if (stop.load()) return;
                }
                uint16_t* p = nullptr;
                try { p = new uint16_t[frame_elems]; }
                catch (const std::bad_alloc&) { return; }
                std::memset(p, 0, frame_bytes);      // fault the pages now
                {
                    std::lock_guard<std::mutex> lock(mu);
                    owned.emplace_back(p);
                    ready_pool.push_back(p);
                    ++pool_reserved;
                }
            }
        });
    }

    // ---- live telemetry ----------------------------------------------------
    // A recorder is watched while it runs, not read afterwards, so the state
    // that decides whether the take survives -- ring occupancy and how fast
    // the overflow is filling -- is emitted continuously.
    std::thread monitor;
    if (o.capture_tick_ms > 0) {
        monitor = std::thread([&]{
            cpu_gpr_place_aux(o);
            const auto tick = std::chrono::milliseconds(o.capture_tick_ms);
            // One tick can be shorter than one frame, so a per-tick rate
            // reads 0, N, 0, N. Average over a ~2 s window instead.
            struct Sample { double t; long long enc; size_t over; };
            std::deque<Sample> window;
            constexpr double WINDOW_S = 2.0;
            while (!stop.load(std::memory_order_relaxed)) {
                std::this_thread::sleep_for(tick);
                long long cap_n, enc_n; size_t queued, over;
                {
                    std::lock_guard<std::mutex> lock(mu);
                    cap_n = captured; enc_n = encoded;
                    queued = pending.size();
                    // What is actually sitting in the overflow right now.
                    over = queued > size_t(o.ring_frames)
                         ? queued - size_t(o.ring_frames) : 0u;
                }
                const auto now = Clock::now();
                const double elapsed = std::chrono::duration<double>(now - start).count();
                window.push_back({elapsed, enc_n, over});
                while (window.size() > 2 && elapsed - window.front().t > WINDOW_S)
                    window.pop_front();
                const double dt = elapsed - window.front().t;
                const double fps_now = dt > 0.0
                    ? double(enc_n - window.front().enc) / dt
                    : (elapsed > 0.0 ? double(enc_n) / elapsed : 0.0);
                const double fps_avg = elapsed > 0.0 ? double(enc_n) / elapsed : 0.0;
                const double fill_rate = dt > 0.0
                    ? double(over - window.front().over) / dt : 0.0;
                double eta = -1.0;
                if (fill_rate > 0.0 && overflow_cap_frames > over)
                    eta = double(overflow_cap_frames - over) / fill_rate;
                std::cout << "CAPTURE_TICK t=" << elapsed
                          << " captured=" << cap_n
                          << " encoded=" << enc_n
                          << " fps_now=" << fps_now
                          << " fps_avg=" << fps_avg
                          << " ring=" << std::min<size_t>(queued, size_t(o.ring_frames))
                          << "/" << o.ring_frames
                          << " queued=" << queued
                          << " overflow_MiB=" << (double(over) * double(frame_bytes) / 1048576.0)
                          << " overflow_cap_MiB=" << cap_mb
                          << " overflow_pct=" << (overflow_cap_frames
                                 ? 100.0 * double(over) / double(overflow_cap_frames) : 0.0)
                          << " overflow_peak_MiB="
                          << (double(peak_overflow) * double(frame_bytes) / 1048576.0)
                          << " eta_s=" << eta
                          << std::endl;   // flushed: a live reader needs it now

            }
        });
    }

    // ---- the sensor --------------------------------------------------------
    // Absolute deadlines, so a slow iteration does not shift the whole clock.
    // A sensor DMA is hardware: it delivers on schedule whether or not a CPU
    // thread was scheduled to notice. An earlier version enqueued one frame
    // per wake, so when the encoder workers saturated the cores the producer
    // was starved and the modelled capture rate silently sagged below target
    // -- which flatters the result, because the encoder was then chasing a
    // slower sensor than the one configured. The producer now catches up to
    // wall-clock: on every wake it emits every frame that has come due.
    /* The producer is this thread. Keep it off the worker cores. */
    cpu_gpr_place_aux(o);
    const auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(1.0 / o.target_fps));
    long long emitted = 0;
    while (!stop.load()) {
        const auto now = Clock::now();
        if (now >= deadline) break;
        const double elapsed_now = std::chrono::duration<double>(now - start).count();
        const long long due_count = (long long)(elapsed_now * o.target_fps) + 1;
        bool out_of_room = false;
        while (emitted < due_count && !out_of_room) {
        uint16_t* buf = nullptr;
        {
            std::lock_guard<std::mutex> lock(mu);
            if (!free_list.empty()) { buf = free_list.back(); free_list.pop_back(); }
            else if (overflow_used < overflow_cap_frames) {
                // The ring is full: the encoder is behind. Grow into RAM.
                //
                // Allocating and first-touching 16.6 MB inline here made the
                // producer expensive exactly while the buffer was filling,
                // which depressed the measured encode rate and so made the
                // clip-length estimate pessimistic. A real recorder reserves
                // its buffer ahead of the take; this takes from a pool the
                // reserver thread keeps stocked, and only allocates inline if
                // the pool has run dry.
                if (!ready_pool.empty()) {
                    buf = ready_pool.back(); ready_pool.pop_back();
                    ++overflow_used;
                    peak_overflow = std::max(peak_overflow, overflow_used);
                    pool_cv.notify_one();
                } else {
                    try {
                        owned.emplace_back(new uint16_t[frame_elems]);
                        buf = owned.back().get();
                        std::memset(buf, 0, frame_bytes);   // fault it in now
                        ++overflow_used;
                        ++pool_misses;
                        peak_overflow = std::max(peak_overflow, overflow_used);
                    } catch (const std::bad_alloc&) {
                        stop_full = true;
                    }
                }
            } else {
                stop_full = true;    // nowhere left to put this frame
            }
            if (stop_full) {
                fill_time_s = std::chrono::duration<double>(Clock::now() - start).count();
                stop.store(true);
                cv_work.notify_all();
            }
        }
        if (!buf) { out_of_room = true; break; }
        if (o.capture_dma_copy) std::memcpy(buf, src.data(), frame_bytes);
        {
            std::lock_guard<std::mutex> lock(mu);
            pending.push_back(buf);
            peak_queue = std::max(peak_queue, pending.size());
            ++captured;
        }
        ++emitted;
        cv_work.notify_one();
        }
        if (out_of_room) break;
        // Sleep until the next frame is genuinely due.
        std::this_thread::sleep_until(start + period * emitted);
    }
    stop.store(true);
    cv_work.notify_all();
    for (auto& t : pool) t.join();
    if (wavelet_assistant) wavelet_assistant->stop();
    pool_cv.notify_all();
    if (reserver.joinable()) reserver.join();
    if (monitor.joinable()) monitor.join();
    if (worker_error) std::rethrow_exception(worker_error);

    const double elapsed = std::chrono::duration<double>(Clock::now() - start).count();
    const double encode_fps = elapsed > 0.0 ? double(encoded) / elapsed : 0.0;
    // How long the sensor was actually producing: it stops at the run deadline
    // or when the buffer filled, whichever came first.
    const double capture_window = stop_full ? fill_time_s : std::min(elapsed, o.capture_seconds);
    const double capture_fps = capture_window > 0.0 ? double(captured) / capture_window : 0.0;
    // A real sensor DMA cannot be descheduled. This one is a thread, and if
    // the workers saturate every core it can be, at which point the encoder is
    // chasing a slower sensor than the one configured and the result flatters
    // itself. Below 98% of target the cell is not a valid measurement.
    const bool capture_ok = capture_fps >= o.target_fps * 0.98;
    const bool sustained = (peak_overflow == 0) && !stop_full;
    // Did the encoder catch up and empty the overflow again before the end?
    const bool recovered = peak_overflow > 0 && !stop_full && pending.empty();
    const double deficit = o.target_fps - encode_fps;

    const uint64_t cap_total = cap_bytes.load();
    std::cout << "CAPTURE_RESULT mode=" << (o.mode.empty() ? "base-q" : o.mode)
              << " gpr_bytes_mean=" << (encoded > 0 ? cap_total / uint64_t(encoded) : 0)
              << " target_fps=" << o.target_fps
              << " sensor_max_fps=" << o.sensor_max_fps
              << " over_sensor=" << ((o.sensor_max_fps > 0.0 &&
                                      o.target_fps > o.sensor_max_fps + 0.01) ? 1 : 0)
              << " encode_fps=" << encode_fps
              << " captured=" << captured
              << " capture_fps_actual=" << capture_fps
              << " capture_ok=" << (capture_ok ? "YES" : "NO")
              << " encoded=" << encoded
              << " elapsed_s=" << elapsed
              << " sustained=" << (sustained ? "YES" : "NO")
              << " overflow_peak_frames=" << peak_overflow
              << " overflow_drained=" << (recovered ? "YES" : "NO")
              << " overflow_MiB_used=" << (double(peak_overflow) * double(frame_bytes) / 1048576.0)
              << " overflow_pct=" << (overflow_cap_frames
                     ? 100.0 * double(peak_overflow) / double(overflow_cap_frames) : 0.0)
              << " peak_queue=" << peak_queue
              << " reserved_ahead=" << pool_reserved
              << " reserve_misses=" << pool_misses
              << " prealloc=" << (o.capture_prealloc ? "on" : "off")
              << " reserve_s=" << reserve_s
              << " buffer_exhausted=" << (stop_full ? "YES" : "NO")
              << " max_clip_s=" << (stop_full ? fill_time_s : o.capture_seconds)
              << " clip_limited_by=" << (stop_full ? "RAM" : "run-length")
              << "\n";
    if (wavelet_assistant) {
        const long long af = wavelet_assist_frames.load(std::memory_order_relaxed);
        const long long completed = wavelet_assistant->stats.completed.load(std::memory_order_relaxed);
        const double compute_ms = completed
            ? double(wavelet_assistant->stats.helper_wall_ns.load(std::memory_order_relaxed)) / double(completed) / 1e6
            : 0.0;
        const double wait_ms = af
            ? double(wavelet_assist_wait_ns.load(std::memory_order_relaxed)) / double(af) / 1e6
            : 0.0;
        const double publish_wait_ms = af
            ? double(wavelet_assist_publish_wait_ns.load(std::memory_order_relaxed)) / double(af) / 1e6
            : 0.0;
        std::cout << "CAPTURE_WAVELET_ASSIST plane=GD"
                  << " assisted_frames=" << af
                  << " assisted_fps=" << (elapsed > 0.0 ? double(af) / elapsed : 0.0)
                  << " gd_compute_ms=" << compute_ms
                  << " owner_wait_ms=" << wait_ms
                  << " publish_wait_ms=" << publish_wait_ms
                  << " submitted=" << wavelet_assistant->stats.submitted.load(std::memory_order_relaxed)
                  << " busy_misses=" << wavelet_assistant->stats.busy_misses.load(std::memory_order_relaxed)
                  << " rows=" << wavelet_assistant->stats.rows_published.load(std::memory_order_relaxed)
                  << " transport=row-ring ring_rows=" << o.core0_wavelet_ring_rows
                  << " sched=" << o.core0_wavelet_sched
                  << " stagger_ms=" << o.cpu_gpr_stagger_ms
                  << " stagger_us=" << o.cpu_gpr_stagger_us
                  << " raw_reread=NO coefficient_handoff_bytes=0\n";
    }
    if (!capture_ok) {
        std::cout << "CAPTURE_VERDICT INVALID -- the modelled sensor only reached "
                  << capture_fps << " of " << o.target_fps << " fps ("
                  << (100.0 * capture_fps / o.target_fps) << "% of target), so the "
                     "encoder was chasing a slow sensor and this cell means nothing.\n"
                  << "  " << workers << " workers plus a producer thread on "
                  << std::max(1u, std::thread::hardware_concurrency()) << " cores. Try:\n"
                  << "    --cpu-gpr-threads " << std::max(1, workers - 1)
                  << "        one fewer worker, leaving the producer a core\n"
                     "    --capture-dma-copy off   drop the modelled DMA write "
                     "(loses the bandwidth term)\n";
    } else if (sustained) {
        std::cout << "CAPTURE_VERDICT SUSTAINED -- the ring never overflowed, so this "
                     "mode records indefinitely at " << o.target_fps << " fps.\n";
    } else if (recovered) {
        std::cout << "CAPTURE_VERDICT RECOVERED -- the ring overflowed to a peak of "
                  << (double(peak_overflow) * double(frame_bytes) / 1048576.0)
                  << " MiB, then the encoder caught up and drained it. The buffer did "
                     "its job: this rate is survivable through bursts, though not with "
                     "margin.\n";
    } else if (!stop_full) {
        const double rate = elapsed > 0.0 ? double(peak_overflow) / elapsed : 0.0;
        const double projected = rate > 0.0
            ? double(overflow_cap_frames) / rate : 0.0;
        std::cout << "CAPTURE_VERDICT NOT SUSTAINED -- short by "
                  << deficit << " fps, overflow filling at " << rate
                  << " frames/s. Projected time to exhaust the buffer: "
                  << projected << " s (run ended first).\n";
    } else {
        std::cout << "CAPTURE_VERDICT BUFFER EXHAUSTED after " << fill_time_s
                  << " s -- that is the maximum clip length at " << o.target_fps
                  << " fps in this mode. Short by " << deficit << " fps.\n";
    }
    return 0;
}

// ===========================================================================
// Core-0 staged CPU experiment.
//
// Two normal workers stay end-to-end on cores 2 and 3. The remaining stream is
// stage-parallel across cores 0 and 1:
//
//   W0E1: core0 RAW/split/fused-wavelet -> ownership slot -> core1 entropy/GPR
//   W1E0: core1 RAW/split/fused-wavelet -> ownership slot -> core0 entropy/GPR
//
// The handoff is ownership-only. Coefficients and sidecar are preallocated in
// bounded slots and are never copied. With two slots and wavelet faster than
// entropy, the wavelet core naturally blocks on a returned free slot for the
// difference between the two stages, leaving the OS real scheduling headroom.
// ===========================================================================

static void cpu_stage_pin_exact_core(const Options& o, int core, const std::string& sched_name)
{
#if defined(__linux__)
    if (o.cpu_gpr_affinity) {
        cpu_set_t set; CPU_ZERO(&set); CPU_SET(size_t(core), &set);
        if (pthread_setaffinity_np(pthread_self(), sizeof(set), &set) != 0) {
            static std::once_flag warned;
            std::call_once(warned, []{ std::cout << "CPU_STAGE_AFFINITY status=denied\n"; });
        }
    }
    if (core == 0) {
        int policy = SCHED_OTHER;
#if defined(SCHED_BATCH)
        if (sched_name == "batch") policy = SCHED_BATCH;
#endif
#if defined(SCHED_IDLE)
        if (sched_name == "idle") policy = SCHED_IDLE;
#endif
        if (policy != SCHED_OTHER) {
            sched_param sp{}; sp.sched_priority = 0;
            if (pthread_setschedparam(pthread_self(), policy, &sp) != 0) {
                static std::once_flag warned_sched;
                std::call_once(warned_sched, [&]{
                    std::cout << "CPU_STAGE_SCHED status=denied requested=" << sched_name << "\n";
                });
            }
        }
    }
#else
    (void)o; (void)core; (void)sched_name;
#endif
}

struct CpuStageThreadStats {
    long frames = 0;
    int64_t copy_ns = 0;
    int64_t wavelet_ns = 0;
    int64_t entropy_ns = 0;
    int64_t dng_wrap_ns = 0;
    int64_t wait_ns = 0;
    int64_t thread_cpu_ns = 0;
    uint64_t payload_bytes = 0;
    uint64_t packet_bytes = 0;
    std::vector<double> latency_ms;
};

static double cpu_stage_percentile(std::vector<double> v, double p)
{
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    const double x = p * double(v.size() - 1);
    const size_t i = size_t(x);
    const size_t j = std::min(v.size() - 1, i + 1);
    const double f = x - double(i);
    return v[i] * (1.0 - f) + v[j] * f;
}


static int run_cpu_gpr_dual_stage_pipeline(const Options& o, const ModeSpec* mode,
                                           const std::vector<uint16_t>& src,
                                           const std::vector<uint16_t>& lut)
{
    announce_cpu_kernel(o, "cpu-dual-stage");
    if (!o.cpu_v2_kernel)
        throw std::runtime_error("dual staged pipeline requires --cpu-v2-kernel on / --cpu-winner on");
    if (o.cpu_gpr_hybrid_entropy)
        throw std::runtime_error("dual staged pipeline requires --cpu-gpr-hybrid-entropy off");
    if (o.core0_stage_normal_workers != 0)
        throw std::runtime_error("dual staged pipeline uses all four cores as two pairs; --core0-stage-normal-workers must be 0");
    // RegDirect's nonzero sidecar is emitted by the STNP path on Pi. The earlier
    // stage matrix proved that forcing NT off can create a tiny/invalid GPR by
    // leaving the sidecar incomplete, so fail closed instead of benchmarking it.
    if (o.cpu_sidecar && !o.cpu_nontemporal)
        throw std::runtime_error("dual staged pipeline requires --cpu-nontemporal on while the winner sidecar is enabled");

    const int pw = o.width / 2, ph = o.height / 2;
    const size_t row_stride = size_t(pw);
    const size_t plane_stride = row_stride * size_t(ph);
    const bool use_sidecar = o.cpu_sidecar;

    std::cout << "CPU_DUAL_STAGE_CONFIG pairA=core0-wavelet->core1-entropy"
              << " pairB=core2-wavelet->core3-entropy"
              << " slots_per_pair=" << o.core0_stage_slots
              << " pairB_phase_ms=" << o.core0_stage_stagger_ms
              << " phase_lock=" << (o.core0_stage_phase_lock ? "on" : "off")
              << " phase_lock_us=" << o.core0_stage_phase_lock_us
              << " core0_sched=" << o.core0_stage_sched
              << " shared_reuse=" << (o.cpu_gpr_shared_reuse ? "on" : "off")
              << " nontemporal=" << (g_cpu_nontemporal_bands ? "on" : "off")
              << " sidecar=" << (use_sidecar ? "on" : "off")
              << " handoff=ownership-no-copy"
              << " geometry=" << o.width << "x" << o.height << "\n";

    struct DualSlot {
        std::vector<int16_t> coeff;
        std::vector<uint8_t> sidecar;
        Clock::time_point t0{}, t1{}, t2{};
        int64_t producer_wait_ns = 0;
        int state = 0; // 0 free, 1 producer owns, 2 ready, 3 entropy owns
    };
    struct DualPair {
        const char* name = nullptr;
        int wave_core = -1;
        int entropy_core = -1;
        int phase_ms = 0;
        std::vector<std::unique_ptr<DualSlot>> slots;
        std::mutex mu;
        std::condition_variable cv;
        std::deque<int> ready;
        bool producer_done = false;
        CpuStageThreadStats producer_stats;
        CpuStageThreadStats entropy_stats;
    };

    auto init_pair = [&](DualPair& p, const char* name, int wc, int ec, int phase) {
        p.name = name; p.wave_core = wc; p.entropy_core = ec; p.phase_ms = phase;
        p.slots.reserve(size_t(o.core0_stage_slots));
        for (int i = 0; i < o.core0_stage_slots; ++i) {
            auto sl = std::make_unique<DualSlot>();
            sl->coeff.assign(plane_stride * 4u, int16_t(0));
            if (use_sidecar) sl->sidecar.assign(plane_stride * 4u / 8u, 0u);
#if defined(__linux__) && defined(MADV_HUGEPAGE)
            if (o.cpu_gpr_hugepages) {
                madvise(sl->coeff.data(), sl->coeff.size() * sizeof(int16_t), MADV_HUGEPAGE);
                if (!sl->sidecar.empty()) madvise(sl->sidecar.data(), sl->sidecar.size(), MADV_HUGEPAGE);
            }
#endif
            p.slots.emplace_back(std::move(sl));
        }
    };

    DualPair a, b;
    init_pair(a, "A", 0, 1, 0);
    init_pair(b, "B", 2, 3, o.core0_stage_stagger_ms);

    const auto wall_begin = Clock::now();
    const auto measure_from = wall_begin + std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(o.warmup_seconds + o.trim_seconds));
    const auto measure_to = measure_from + std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(o.duration));
    const auto deadline = measure_to + std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(o.trim_seconds));
    const bool timed = o.duration > 0.0;
    std::atomic<int> claimed{0};
    auto claim_work = [&]() -> bool {
        if (timed) return Clock::now() < deadline;
        const int n = claimed.fetch_add(1, std::memory_order_relaxed);
        return n < o.frames;
    };

    std::mutex err_mu;
    std::exception_ptr error;
    std::atomic<bool> abort{false};

    auto find_free_locked = [&](DualPair& p) -> int {
        for (int i = 0; i < int(p.slots.size()); ++i)
            if (p.slots[size_t(i)]->state == 0) return i;
        return -1;
    };

    auto transform_into = [&](DualSlot& slot, const std::vector<uint16_t>& raw_in,
                              V2Frame& v2ctx) {
        V2Sidecar sc{};
        if (use_sidecar) { sc.mask_base = slot.sidecar.data(); sc.coeff_base = slot.coeff.data(); }
        v2_fused_frame(use_sidecar ? &sc : nullptr, o, mode, raw_in.data(), size_t(o.width),
                       0, 0, lut, slot.coeff.data(), plane_stride, row_stride,
                       SplitKind::Shipped, EmitKind::RegDirect, v2ctx);
    };

    std::atomic<int64_t> a_last_start_ns{0};
    std::atomic<int64_t> a_period_ns{0};
    std::mutex phase_stats_mu;
    std::vector<double> phase_error_ms;
    int64_t phase_delay_ns = 0;
    long phase_delay_events = 0;
    long phase_missed = 0;

    auto phase_clock_ns = []() -> int64_t {
        return std::chrono::duration_cast<Ns>(Clock::now().time_since_epoch()).count();
    };

    auto producer_body = [&](DualPair& p) {
        try {
            const std::string sched = p.wave_core == 0 ? o.core0_stage_sched : "normal";
            cpu_stage_pin_exact_core(o, p.wave_core, sched);
            if (p.phase_ms > 0)
                std::this_thread::sleep_for(std::chrono::milliseconds(p.phase_ms));
            const int64_t cpu0 = cpu_gpr_thread_cpu_now_ns();
            std::vector<uint16_t> captured(src.size());
            V2Frame v2ctx;
            g_v2_sidecar_zskip = o.cpu_sidecar_zskip;
            g_v2_input_prefetch = o.cpu_input_prefetch;
            while (!abort.load(std::memory_order_relaxed) && claim_work()) {
                const auto wait0 = Clock::now();
                int idx = -1;
                {
                    std::unique_lock<std::mutex> lock(p.mu);
                    p.cv.wait(lock, [&]{ return abort.load() || (idx = find_free_locked(p)) >= 0; });
                    if (abort.load()) break;
                    p.slots[size_t(idx)]->state = 1;
                }
                auto& slot = *p.slots[size_t(idx)];
                slot.producer_wait_ns = std::chrono::duration_cast<Ns>(Clock::now() - wait0).count();
                if (p.wave_core == 2 && o.core0_stage_phase_lock && o.core0_stage_stagger_ms > 0) {
                    const int64_t target = int64_t(o.core0_stage_stagger_ms) * 1000000LL;
                    const int64_t astart = a_last_start_ns.load(std::memory_order_acquire);
                    const int64_t period = a_period_ns.load(std::memory_order_relaxed);
                    const int64_t now = phase_clock_ns();
                    // Only lock when the measured A cadence is longer than the
                    // requested phase. On Pi/m5 this is ~38 ms vs 14 ms.
                    if (astart > 0 && period > target + 1000000LL && now >= astart) {
                        const int64_t phase = now - astart;
                        if (phase < target) {
                            const int64_t cap = int64_t(o.core0_stage_phase_lock_us) * 1000LL;
                            const int64_t delay = std::min(target - phase, cap);
                            if (delay > 0) {
                                std::this_thread::sleep_for(Ns(delay));
                                std::lock_guard<std::mutex> lk(phase_stats_mu);
                                phase_delay_ns += delay; ++phase_delay_events;
                            }
                        }
                        const auto after = Clock::now();
                        if (after >= measure_from && after < measure_to) {
                            const int64_t phase2 = phase_clock_ns() - astart;
                            const double err = double(phase2 - target) / 1e6;
                            std::lock_guard<std::mutex> lk(phase_stats_mu);
                            phase_error_ms.push_back(err);
                        }
                    } else {
                        std::lock_guard<std::mutex> lk(phase_stats_mu); ++phase_missed;
                    }
                }
                slot.t0 = Clock::now();
                if (p.wave_core == 0) {
                    const int64_t ns = std::chrono::duration_cast<Ns>(slot.t0.time_since_epoch()).count();
                    const int64_t prev = a_last_start_ns.exchange(ns, std::memory_order_acq_rel);
                    if (prev > 0 && ns > prev) a_period_ns.store(ns - prev, std::memory_order_relaxed);
                }
                const std::vector<uint16_t>& raw_in = o.cpu_gpr_raw_copy ? captured : src;
                if (o.cpu_gpr_raw_copy)
                    std::memcpy(captured.data(), src.data(), src.size() * sizeof(uint16_t));
                slot.t1 = Clock::now();
                transform_into(slot, raw_in, v2ctx);
                slot.t2 = Clock::now();
                {
                    std::lock_guard<std::mutex> lock(p.mu);
                    slot.state = 2;
                    p.ready.push_back(idx);
                }
                p.cv.notify_all();
            }
            {
                std::lock_guard<std::mutex> lock(p.mu);
                p.producer_done = true;
            }
            p.cv.notify_all();
            p.producer_stats.thread_cpu_ns = cpu_gpr_thread_cpu_now_ns() - cpu0;
        } catch (...) {
            { std::lock_guard<std::mutex> lock(err_mu); if (!error) error = std::current_exception(); }
            abort.store(true);
            { std::lock_guard<std::mutex> lock(p.mu); p.producer_done = true; }
            a.cv.notify_all(); b.cv.notify_all();
        }
    };

    auto entropy_body = [&](DualPair& p) {
        try {
            cpu_stage_pin_exact_core(o, p.entropy_core, "normal");
            const int64_t cpu0 = cpu_gpr_thread_cpu_now_ns();
            DirectGprEncoder enc(o, mode, row_stride, plane_stride);
            enc.splice_enabled = o.cpu_gpr_dng_splice;
            enc.splice_shared = o.cpu_gpr_dng_splice && o.cpu_gpr_splice_shared;
            enc.splice_reuse = o.cpu_gpr_dng_splice && o.cpu_gpr_splice_reuse;
            enc.shared_reuse = o.cpu_gpr_dng_splice && o.cpu_gpr_splice_shared && o.cpu_gpr_shared_reuse;
            for (;;) {
                int idx = -1;
                const auto wait0 = Clock::now();
                {
                    std::unique_lock<std::mutex> lock(p.mu);
                    p.cv.wait(lock, [&]{ return abort.load() || !p.ready.empty() || p.producer_done; });
                    if (abort.load()) break;
                    if (p.ready.empty()) { if (p.producer_done) break; else continue; }
                    idx = p.ready.front(); p.ready.pop_front();
                    p.slots[size_t(idx)]->state = 3;
                }
                const int64_t wait_ready_ns = std::chrono::duration_cast<Ns>(Clock::now() - wait0).count();
                auto& slot = *p.slots[size_t(idx)];
                enc.set_sidecar(use_sidecar ? slot.sidecar.data() : nullptr, slot.coeff.data());
                gpr_buffer out{nullptr, 0}; size_t vc5_bytes = 0;
                const auto e0 = Clock::now();
                const int64_t dng0 = enc.dng_wrap_ns;
                enc.encode(slot.coeff.data(), out, vc5_bytes);
                const auto t3 = Clock::now();
                const int64_t dng_delta = enc.dng_wrap_ns - dng0;
                if (t3 >= measure_from && t3 < measure_to) {
                    ++p.entropy_stats.frames;
                    p.entropy_stats.copy_ns += std::chrono::duration_cast<Ns>(slot.t1 - slot.t0).count();
                    p.entropy_stats.wavelet_ns += std::chrono::duration_cast<Ns>(slot.t2 - slot.t1).count();
                    p.entropy_stats.entropy_ns += std::chrono::duration_cast<Ns>(t3 - e0).count();
                    p.entropy_stats.dng_wrap_ns += dng_delta;
                    p.producer_stats.wait_ns += slot.producer_wait_ns;
                    p.entropy_stats.wait_ns += wait_ready_ns;
                    p.entropy_stats.payload_bytes += vc5_bytes;
                    p.entropy_stats.packet_bytes += out.size;
                    p.entropy_stats.latency_ms.push_back(std::chrono::duration<double,std::milli>(t3 - slot.t0).count());
                }
                enc.free_buffer(out);
                {
                    std::lock_guard<std::mutex> lock(p.mu);
                    slot.state = 0;
                }
                p.cv.notify_all();
            }
            p.entropy_stats.thread_cpu_ns = cpu_gpr_thread_cpu_now_ns() - cpu0;
        } catch (...) {
            { std::lock_guard<std::mutex> lock(err_mu); if (!error) error = std::current_exception(); }
            abort.store(true); a.cv.notify_all(); b.cv.notify_all();
        }
    };

    // Start consumers first so the phase control affects only wavelet activity,
    // not entropy-thread startup latency.
    std::thread ae(entropy_body, std::ref(a));
    std::thread be(entropy_body, std::ref(b));
    std::thread ap(producer_body, std::ref(a));
    std::thread bp(producer_body, std::ref(b));
    ap.join(); bp.join(); ae.join(); be.join();
    if (error) std::rethrow_exception(error);

    const auto wall_end = Clock::now();
    const double runtime_s = std::max(1e-9, std::chrono::duration<double>(wall_end - wall_begin).count());
    const long af = a.entropy_stats.frames, bf = b.entropy_stats.frames;
    const long frames = af + bf;
    const double fps = o.duration > 0.0 ? double(frames) / o.duration : 0.0;
    const double afps = o.duration > 0.0 ? double(af) / o.duration : 0.0;
    const double bfps = o.duration > 0.0 ? double(bf) / o.duration : 0.0;
    auto per_ms = [](int64_t ns, long f){ return f ? double(ns) / double(f) / 1e6 : 0.0; };
    const double aw = per_ms(a.entropy_stats.wavelet_ns, af);
    const double ae_ms = per_ms(a.entropy_stats.entropy_ns, af);
    const double adng = per_ms(a.entropy_stats.dng_wrap_ns, af);
    const double bw = per_ms(b.entropy_stats.wavelet_ns, bf);
    const double be_ms = per_ms(b.entropy_stats.entropy_ns, bf);
    const double bdng = per_ms(b.entropy_stats.dng_wrap_ns, bf);
    const double afree = per_ms(a.producer_stats.wait_ns, af);
    const double astarve = per_ms(a.entropy_stats.wait_ns, af);
    const double bfree = per_ms(b.producer_stats.wait_ns, bf);
    const double bstarve = per_ms(b.entropy_stats.wait_ns, bf);
    const double c0 = 100.0 * double(a.producer_stats.thread_cpu_ns) / (runtime_s * 1e9);
    const double c1 = 100.0 * double(a.entropy_stats.thread_cpu_ns) / (runtime_s * 1e9);
    const double c2 = 100.0 * double(b.producer_stats.thread_cpu_ns) / (runtime_s * 1e9);
    const double c3 = 100.0 * double(b.entropy_stats.thread_cpu_ns) / (runtime_s * 1e9);
    std::vector<double> lat = a.entropy_stats.latency_ms;
    lat.insert(lat.end(), b.entropy_stats.latency_ms.begin(), b.entropy_stats.latency_ms.end());
    const uint64_t payload = a.entropy_stats.payload_bytes + b.entropy_stats.payload_bytes;
    const uint64_t packets = a.entropy_stats.packet_bytes + b.entropy_stats.packet_bytes;
    double phase_mean = 0.0, phase_abs = 0.0, phase_p95 = 0.0;
    {
        std::lock_guard<std::mutex> lk(phase_stats_mu);
        if (!phase_error_ms.empty()) {
            for (double x : phase_error_ms) { phase_mean += x; phase_abs += std::abs(x); }
            phase_mean /= double(phase_error_ms.size());
            phase_abs /= double(phase_error_ms.size());
            std::vector<double> av; av.reserve(phase_error_ms.size());
            for (double x : phase_error_ms) av.push_back(std::abs(x));
            phase_p95 = cpu_stage_percentile(av, 0.95);
        }
    }

    std::cout << "CPU_DUAL_STAGE_RESULT frames=" << frames
              << " seconds=" << o.duration
              << " fps=" << fps
              << " pairA_fps=" << afps
              << " pairB_fps=" << bfps
              << " slots=" << o.core0_stage_slots
              << " pairB_phase_ms=" << o.core0_stage_stagger_ms
              << " core0_sched=" << o.core0_stage_sched
              << " core0_cpu_pct=" << c0
              << " core1_cpu_pct=" << c1
              << " core2_cpu_pct=" << c2
              << " core3_cpu_pct=" << c3
              << " pairA_wavelet_ms=" << aw
              << " pairA_entropy_ms=" << ae_ms
              << " pairA_dng_ms=" << adng
              << " pairA_vc5_ms=" << std::max(0.0, ae_ms - adng)
              << " pairA_wait_free_ms=" << afree
              << " pairA_starve_ms=" << astarve
              << " pairB_wavelet_ms=" << bw
              << " pairB_entropy_ms=" << be_ms
              << " pairB_dng_ms=" << bdng
              << " pairB_vc5_ms=" << std::max(0.0, be_ms - bdng)
              << " pairB_wait_free_ms=" << bfree
              << " pairB_starve_ms=" << bstarve
              << " latency_p50_ms=" << cpu_stage_percentile(lat, 0.50)
              << " latency_p99_ms=" << cpu_stage_percentile(lat, 0.99)
              << " phase_lock=" << (o.core0_stage_phase_lock ? "on" : "off")
              << " phase_error_mean_ms=" << phase_mean
              << " phase_error_abs_ms=" << phase_abs
              << " phase_error_p95_ms=" << phase_p95
              << " phase_delay_ms=" << (frames ? double(phase_delay_ns) / double(frames) / 1e6 : 0.0)
              << " phase_delay_events=" << phase_delay_events
              << " phase_missed=" << phase_missed
              << " shared_reuse=" << (o.cpu_gpr_shared_reuse ? "on" : "off")
              << " avg_gpr_bytes=" << (frames ? packets / uint64_t(frames) : 0)
              << " avg_vc5_bytes=" << (frames ? payload / uint64_t(frames) : 0)
              << " handoff_copy_bytes=0\n";
    std::cout << "CPU_DUAL_STAGE_PAIR id=A wave_core=0 entropy_core=1 fps=" << afps
              << " wavelet_ms=" << aw << " entropy_ms=" << ae_ms << " dng_ms=" << adng
              << " wait_free_ms=" << afree << " starve_ms=" << astarve << "\n";
    std::cout << "CPU_DUAL_STAGE_PAIR id=B wave_core=2 entropy_core=3 fps=" << bfps
              << " wavelet_ms=" << bw << " entropy_ms=" << be_ms << " dng_ms=" << bdng
              << " wait_free_ms=" << bfree << " starve_ms=" << bstarve << "\n";
    std::cout << "CPU_DUAL_STAGE_VERIFY coefficient_handoff=ownership-only copy_bytes=0 slots_per_pair="
              << o.core0_stage_slots << " sidecar_handoff=" << (use_sidecar ? "YES" : "NO")
              << " max_simultaneous_wavelets=2\n";
    return 0;
}


// ===========================================================================
// v1.14 adaptive dual-stage pipeline.
//
// Two wavelet producers (cores 0 and 2) feed a SHARED coefficient-slot pool.
// Two entropy consumers (cores 1 and 3) can consume either producer's frames.
// Core 0 can therefore be SCHED_IDLE / nice and lose cycles to Linux without
// stranding core 1: the entropy cores steal ready frames from core 2 instead.
// Core 0 can also be forced to leave one or two free slots for the core-2
// producer, making the OS core explicitly opportunistic rather than symmetric.
// ===========================================================================
static void verify_v2_context_reuse_once(const Options& o, const ModeSpec* mode,
                                         const std::vector<uint16_t>& src,
                                         const std::vector<uint16_t>& lut,
                                         size_t plane_stride, size_t row_stride)
{
    if (!o.cpu_wavelet_reuse_context) return;
    const bool old = g_cpu_wavelet_reuse_context;
    const size_t n = plane_stride * 4u;
    std::vector<int16_t> ref(n), got(n);
    std::vector<uint8_t> ref_sc(n / 8u), got_sc(n / 8u);
    V2Frame a, b;
    V2Sidecar rsa{ref_sc.data(), ref.data()};
    V2Sidecar gsa{got_sc.data(), got.data()};
    g_cpu_wavelet_reuse_context = false;
    v2_fused_frame(&rsa, o, mode, src.data(), size_t(o.width), 0, 0, lut,
                   ref.data(), plane_stride, row_stride,
                   SplitKind::Shipped, EmitKind::RegDirect, a);
    g_cpu_wavelet_reuse_context = true;
    // First pass sizes/fills the scratch; second pass is the real stale-data test.
    v2_fused_frame(&gsa, o, mode, src.data(), size_t(o.width), 0, 0, lut,
                   got.data(), plane_stride, row_stride,
                   SplitKind::Shipped, EmitKind::RegDirect, b);
    v2_fused_frame(&gsa, o, mode, src.data(), size_t(o.width), 0, 0, lut,
                   got.data(), plane_stride, row_stride,
                   SplitKind::Shipped, EmitKind::RegDirect, b);
    g_cpu_wavelet_reuse_context = old;
    if (ref != got || ref_sc != got_sc)
        throw std::runtime_error("wavelet context reuse verification failed");
    std::cout << "CPU_WAVELET_CONTEXT_REUSE_VERIFY bitexact=YES repeated_frame=YES\n";
}

static int run_cpu_gpr_adaptive_stage_pipeline(const Options& o, const ModeSpec* mode,
                                               const std::vector<uint16_t>& src,
                                               const std::vector<uint16_t>& lut)
{
    announce_cpu_kernel(o, "cpu-adaptive-stage");
    if (!o.cpu_v2_kernel)
        throw std::runtime_error("adaptive staged pipeline requires --cpu-v2-kernel on / --cpu-winner on");
    if (o.cpu_gpr_hybrid_entropy)
        throw std::runtime_error("adaptive staged pipeline requires --cpu-gpr-hybrid-entropy off");
    if (o.core0_stage_normal_workers != 0)
        throw std::runtime_error("adaptive staged pipeline uses all four cores; --core0-stage-normal-workers must be 0");
    /* --cpu-direct-hybrid and the E8 int16-frame sidecar are resolved against
       each other in parse_options(): the tile pool has no int16 frame, and it
       now carries its OWN per-coefficient nonzero mask, so the hybrid reader
       no longer has to perform its own zero-run detection. */
    if (o.cpu_direct_hybrid && o.cpu_sidecar)
        throw std::runtime_error("internal: --cpu-direct-hybrid reached the adaptive pipeline with the E8 sidecar still on");
    if (o.cpu_sidecar && !o.cpu_nontemporal)
        throw std::runtime_error("adaptive staged pipeline requires --cpu-nontemporal on while the winner sidecar is enabled");

    const int pw = o.width / 2, ph = o.height / 2;
    const size_t row_stride = size_t(pw);
    const size_t plane_stride = row_stride * size_t(ph);
    const bool use_sidecar = o.cpu_sidecar;
    // These controls are process-wide constants for the whole adaptive run.
    // Set them before launching producer threads rather than writing the same
    // globals concurrently from core0/core2.
    g_v2_sidecar_zskip = o.cpu_sidecar_zskip;
    g_v2_input_prefetch = o.cpu_input_prefetch;
    verify_v2_context_reuse_once(o, mode, src, lut, plane_stride, row_stride);

    std::cout << "CPU_ADAPTIVE_STAGE_CONFIG producers=core0+core2 consumers=core1+core3"
              << " global_slots=" << o.dual_global_slots
              << " queue=" << o.dual_queue_policy
              << " core0_reserve=" << o.dual_core0_reserve_slots
              << " core0_reserve_mode=" << (o.dual_core0_soft_reserve ? "soft" : "hard")
              << " core0_sched=" << o.core0_stage_sched
              << " core0_nice=" << o.dual_core0_nice
              << " core0_phase_ms=" << o.core0_stage_stagger_ms
              << " core0_start_gap_us=" << o.dual_core0_start_gap_us
              << " context_reuse=" << (o.cpu_wavelet_reuse_context ? "on" : "off")
              << " hot_lut=" << (o.cpu_hot_lut ? "on" : "off")
              << " direct_hybrid=" << (o.cpu_direct_hybrid ? "on" : "off")
              << " local_inplace=" << (o.cpu_gpr_local_inplace ? "on" : "off")
              << " local_inplace_pool=" << (o.cpu_gpr_local_inplace_pool ? "on" : "off")
              << " shared_inplace=" << (o.cpu_gpr_shared_inplace ? "on" : "off")
              << " vle_prefetch_distance=" << o.cpu_vle_prefetch_distance
              << " vle_prefetch_locality=" << o.cpu_vle_prefetch_locality
              << " nontemporal=" << (g_cpu_nontemporal_bands ? "on" : "off")
              << " handoff=ownership-no-copy geometry=" << o.width << "x" << o.height << "\n";

    struct Slot {
        int16_t* coeff = nullptr;
        uint8_t* sidecar = nullptr;
        std::unique_ptr<CpuDirectHybridSlot> hybrid;
        size_t coeff_elems = 0, sidecar_bytes = 0;
        Clock::time_point t0{}, t1{}, t2{};
        int64_t producer_wait_ns = 0;
        int producer = -1; // 0=core0, 1=core2
        int state = 0;     // 0 free, 1 wavelet owns, 2 ready, 3 entropy owns
        ~Slot() { std::free(coeff); std::free(sidecar); }
    };
    struct Agg {
        long frames = 0;
        int64_t wavelet_ns = 0, entropy_ns = 0, dng_ns = 0;
        int64_t free_wait_ns = 0, ready_wait_ns = 0;
        uint64_t payload = 0, packet = 0;
        std::vector<double> latency;
    };
    struct Worker {
        int core = -1;
        int64_t cpu_ns = 0;
        int64_t gap_wait_ns = 0;
        long jobs = 0, steals = 0;
    };

    auto alloc_aligned = [&](size_t bytes) -> void* {
        void* mem = nullptr;
        const size_t align = size_t(o.dual_slot_alignment);
#if defined(__linux__) || defined(__APPLE__)
        if (::posix_memalign(&mem, align, bytes) != 0) mem = nullptr;
#else
        const size_t rounded = (bytes + align - 1u) & ~(align - 1u);
        mem = std::aligned_alloc(align, rounded);
#endif
        if (!mem) throw std::runtime_error("adaptive aligned slot allocation failed");
        std::memset(mem, 0, bytes);
        return mem;
    };
    std::vector<std::unique_ptr<Slot>> slots;
    slots.reserve(size_t(o.dual_global_slots));
    for (int i = 0; i < o.dual_global_slots; ++i) {
        auto sl = std::make_unique<Slot>();
        if (o.cpu_direct_hybrid) {
            sl->hybrid = std::make_unique<CpuDirectHybridSlot>(uint32_t(pw), uint32_t(ph));
        } else {
            sl->coeff_elems = plane_stride * 4u;
            sl->coeff = static_cast<int16_t*>(alloc_aligned(sl->coeff_elems * sizeof(int16_t)));
            if (use_sidecar) {
                sl->sidecar_bytes = plane_stride * 4u / 8u;
                sl->sidecar = static_cast<uint8_t*>(alloc_aligned(sl->sidecar_bytes));
            }
        }
#if defined(__linux__) && defined(MADV_HUGEPAGE)
        if (o.cpu_gpr_hugepages && sl->coeff) {
            madvise(sl->coeff, sl->coeff_elems * sizeof(int16_t), MADV_HUGEPAGE);
            if (sl->sidecar) madvise(sl->sidecar, sl->sidecar_bytes, MADV_HUGEPAGE);
        }
#endif
        slots.emplace_back(std::move(sl));
    }
    const uintptr_t coeff_mod64 = (slots.empty() || !slots[0]->coeff) ? 0u : (reinterpret_cast<uintptr_t>(slots[0]->coeff) & 63u);
    const uintptr_t side_mod64 = (slots.empty() || !slots[0]->sidecar) ? 0u : (reinterpret_cast<uintptr_t>(slots[0]->sidecar) & 63u);
    const double direct_slot_mib = (slots.empty() || !slots[0]->hybrid) ? 0.0 : double(slots[0]->hybrid->logical_compact_bytes())/1048576.0;
    std::cout << "CPU_ADAPTIVE_MEMORY coeff_base_mod64=" << coeff_mod64
              << " sidecar_base_mod64=" << side_mod64
              << " coeff_slot_mib=" << (double(plane_stride * 4u * sizeof(int16_t)) / 1048576.0)
              << " total_coeff_mib=" << (o.cpu_direct_hybrid ? direct_slot_mib*double(o.dual_global_slots) : double(plane_stride * 4u * sizeof(int16_t) * size_t(o.dual_global_slots)) / 1048576.0)
              << " direct_compact_slot_mib=" << direct_slot_mib
              << " requested_alignment=" << o.dual_slot_alignment
              << "\n";

    std::mutex mu, err_mu, stats_mu, intervals_mu;
    std::condition_variable cv;
    std::deque<int> ready;
    bool producer_done[2]{false,false};
    std::atomic<bool> abort{false};
    std::exception_ptr error;
    Agg agg[2];
    Worker prod[2]{{0,0,0,0,0},{2,0,0,0,0}};
    Worker ent[2]{{1,0,0,0,0},{3,0,0,0,0}};
    std::atomic<int64_t> last_core2_start_ns{0};
    std::atomic<bool> core2_waiting_for_slot{false};
    std::atomic<bool> coeff_profile_done{false};
    std::atomic<bool> direct_hybrid_verified{false};
    std::vector<std::pair<Clock::time_point,Clock::time_point>> intervals[2];

    const auto wall_begin = Clock::now();
    const auto measure_from = wall_begin + std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(o.warmup_seconds + o.trim_seconds));
    const auto measure_to = measure_from + std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(o.duration));
    const auto deadline = measure_to + std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(o.trim_seconds));
    const bool timed = o.duration > 0.0;
    std::atomic<int> claimed{0};
    auto claim_work = [&]() -> bool {
        if (timed) return Clock::now() < deadline;
        return claimed.fetch_add(1, std::memory_order_relaxed) < o.frames;
    };
    auto free_count_locked = [&]() {
        int n=0; for (auto& sl: slots) if (sl->state==0) ++n; return n;
    };
    auto free_index_locked = [&]() {
        for (int i=0;i<int(slots.size());++i) if (slots[size_t(i)]->state==0) return i;
        return -1;
    };
    auto all_done_locked = [&]() { return producer_done[0] && producer_done[1]; };

    auto producer_body = [&](int pid) {
        try {
            const int core = pid==0 ? 0 : 2;
            cpu_stage_pin_exact_core(o, core, pid==0 ? o.core0_stage_sched : "normal");
#if defined(__linux__)
            if (pid==0 && o.dual_core0_nice > 0) {
                errno = 0;
                const int r = ::nice(o.dual_core0_nice);
                if (r == -1 && errno != 0)
                    std::cout << "CPU_ADAPTIVE_NICE status=denied requested=" << o.dual_core0_nice << "\n";
                else
                    std::cout << "CPU_ADAPTIVE_NICE status=ok value=" << r << "\n";
            }
#endif
            // Core2 is the primary producer. Core0 is the optional follower.
            if (pid==0 && o.core0_stage_stagger_ms>0)
                std::this_thread::sleep_for(std::chrono::milliseconds(o.core0_stage_stagger_ms));
            const int64_t cpu0 = cpu_gpr_thread_cpu_now_ns();
            V2Frame ctx;
            while (!abort.load(std::memory_order_relaxed) && claim_work()) {
                // One-way phase guard: only the OS core waits. Core2 remains
                // the primary producer and is never delayed by core0. This
                // preserves a minimum start separation without the unstable
                // closed-loop phase controller used by the previous matrix.
                if (pid == 0 && o.dual_core0_start_gap_us > 0) {
                    const int64_t other = last_core2_start_ns.load(std::memory_order_acquire);
                    if (other > 0) {
                        const int64_t target = other + int64_t(o.dual_core0_start_gap_us) * 1000ll;
                        const int64_t now_ns = std::chrono::duration_cast<Ns>(Clock::now().time_since_epoch()).count();
                        if (target > now_ns) {
                            const auto gw0 = Clock::now();
                            std::this_thread::sleep_until(Clock::time_point(Ns(target)));
                            prod[0].gap_wait_ns += std::chrono::duration_cast<Ns>(Clock::now() - gw0).count();
                        }
                    }
                }
                const auto w0 = Clock::now();
                int idx=-1;
                {
                    std::unique_lock<std::mutex> lk(mu);
                    if (pid == 1) core2_waiting_for_slot.store(true, std::memory_order_release);
                    cv.wait(lk,[&]{
                        if (abort.load()) return true;
                        const int free = free_count_locked();
                        if (pid != 0) return free > 0;
                        if (!o.dual_core0_soft_reserve) return free > o.dual_core0_reserve_slots;
                        const int reserve = core2_waiting_for_slot.load(std::memory_order_acquire)
                                          ? o.dual_core0_reserve_slots : 0;
                        return free > reserve;
                    });
                    if (pid == 1) core2_waiting_for_slot.store(false, std::memory_order_release);
                    if (abort.load()) break;
                    idx = free_index_locked();
                    if (idx < 0) continue;
                    slots[size_t(idx)]->state=1;
                    slots[size_t(idx)]->producer=pid;
                }
                auto& sl=*slots[size_t(idx)];
                sl.producer_wait_ns=std::chrono::duration_cast<Ns>(Clock::now()-w0).count();
                sl.t0=Clock::now();
                sl.t1=sl.t0;
                if (pid == 1)
                    last_core2_start_ns.store(std::chrono::duration_cast<Ns>(sl.t1.time_since_epoch()).count(),
                                              std::memory_order_release);
                V2Sidecar sc{};
                if (o.cpu_direct_hybrid) {
                    v2_fused_frame_compact(o, mode, src.data(), size_t(o.width), 0, 0, lut, *sl.hybrid, ctx);
                } else {
                    if (use_sidecar) { sc.mask_base=sl.sidecar; sc.coeff_base=sl.coeff; }
                    v2_fused_frame(use_sidecar ? &sc : nullptr, o, mode, src.data(), size_t(o.width),
                                   0,0,lut,sl.coeff,plane_stride,row_stride,
                                   SplitKind::Shipped,EmitKind::RegDirect,ctx);
                }
                sl.t2=Clock::now();
                if (o.cpu_direct_hybrid && !direct_hybrid_verified.exchange(true, std::memory_order_acq_rel)) {
                    std::vector<int16_t> ref(plane_stride * 4u);
                    V2Frame refctx;
                    v2_fused_frame(nullptr, o, mode, src.data(), size_t(o.width), 0, 0, lut,
                                   ref.data(), plane_stride, row_stride,
                                   SplitKind::Shipped, EmitKind::RegDirect, refctx);
                    if (!sl.hybrid->verify_against(ref.data(), row_stride, plane_stride))
                        throw std::runtime_error("direct hybrid coefficient verification failed");
                    std::cout << "CPU_DIRECT_HYBRID_VERIFY bitexact=YES int8_tiles=" << sl.hybrid->int8_tiles()
                              << " fallback_tiles=" << sl.hybrid->fallback_tiles()
                              << " total_tiles=" << sl.hybrid->total_tiles()
                              << " compact_mib=" << (double(sl.hybrid->logical_compact_bytes())/1048576.0)
                              << " full_int16_mib=" << (double(plane_stride*4u*sizeof(int16_t))/1048576.0) << "\n";
                }
                if (o.cpu_coeff_profile && !o.cpu_direct_hybrid && !coeff_profile_done.exchange(true, std::memory_order_acq_rel)) {
                    // Warmup-only diagnostic. This intentionally performs the
                    // existing post-pack once so we can quantify whether a future
                    // direct compact wavelet target is worth engineering.
                    CpuTileHybridProvider hp(1, uint32_t(row_stride), uint32_t(ph),
                                             row_stride, plane_stride);
                    hp.prepare(0, sl.coeff);
                    const auto& hm = hp.metrics(0);
                    uint64_t nz = 0, total = plane_stride * 4u;
                    if (use_sidecar && sl.sidecar) {
                        for (size_t bi = 0; bi < sl.sidecar_bytes; ++bi)
                            nz += uint64_t(__builtin_popcount(unsigned(sl.sidecar[bi])));
                    } else {
                        for (size_t ci = 0; ci < total; ++ci) nz += sl.coeff[ci] != 0;
                    }
                    std::cout << "CPU_COEFF_PROFILE nonzero_pct=" << (100.0*double(nz)/double(total))
                              << " int8_tiles=" << hm.int8_tiles
                              << " fallback_tiles=" << hm.fallback_tiles
                              << " total_tiles=" << hm.total_tiles
                              << " int8_tile_pct=" << (hm.total_tiles ? 100.0*double(hm.int8_tiles)/double(hm.total_tiles) : 0.0)
                              << " compact_physical_mib=" << (double(hm.physical_bytes)/1048576.0)
                              << " full_int16_mib=" << (double(total*sizeof(int16_t))/1048576.0)
                              << " note=profile_not_timed\n";
                }
                if (sl.t2 >= measure_from && sl.t1 < measure_to) {
                    std::lock_guard<std::mutex> il(intervals_mu);
                    intervals[pid].push_back({std::max(sl.t1,measure_from),std::min(sl.t2,measure_to)});
                }
                {
                    std::lock_guard<std::mutex> lk(mu);
                    sl.state=2; ready.push_back(idx);
                }
                cv.notify_all();
            }
            {
                std::lock_guard<std::mutex> lk(mu); producer_done[pid]=true;
            }
            cv.notify_all();
            prod[pid].cpu_ns=cpu_gpr_thread_cpu_now_ns()-cpu0;
        } catch (...) {
            { std::lock_guard<std::mutex> lk(err_mu); if (!error) error=std::current_exception(); }
            abort.store(true); cv.notify_all();
        }
    };

    auto pick_ready_locked = [&](int eid, bool& stolen) -> int {
        stolen=false;
        if (ready.empty()) return -1;
        if (o.dual_queue_policy=="fifo") { int x=ready.front(); ready.pop_front(); return x; }
        int preferred = (o.dual_queue_policy=="core2") ? 1 : eid; // e0 prefers p0, e1 prefers p1
        auto it=std::find_if(ready.begin(),ready.end(),[&](int x){return slots[size_t(x)]->producer==preferred;});
        if (it!=ready.end()) { int x=*it; ready.erase(it); return x; }
        int x=ready.front(); ready.pop_front();
        stolen = slots[size_t(x)]->producer != preferred;
        return x;
    };

    auto entropy_body = [&](int eid) {
        try {
            const int core=eid==0?1:3;
            cpu_stage_pin_exact_core(o,core,"normal");
            const int64_t cpu0=cpu_gpr_thread_cpu_now_ns();
            DirectGprEncoder enc(o,mode,row_stride,plane_stride);
            enc.splice_enabled=o.cpu_gpr_dng_splice;
            enc.splice_shared=o.cpu_gpr_dng_splice && o.cpu_gpr_splice_shared;
            enc.splice_reuse=o.cpu_gpr_dng_splice && o.cpu_gpr_splice_reuse;
            enc.shared_reuse=o.cpu_gpr_dng_splice && o.cpu_gpr_splice_shared && o.cpu_gpr_shared_reuse;
            for (;;) {
                int idx=-1; bool stolen=false;
                const auto rw0=Clock::now();
                {
                    std::unique_lock<std::mutex> lk(mu);
                    cv.wait(lk,[&]{ return abort.load() || !ready.empty() || all_done_locked(); });
                    if (abort.load()) break;
                    if (ready.empty()) { if (all_done_locked()) break; else continue; }
                    idx=pick_ready_locked(eid,stolen);
                    if (idx<0) continue;
                    slots[size_t(idx)]->state=3;
                }
                const int64_t ready_wait=std::chrono::duration_cast<Ns>(Clock::now()-rw0).count();
                auto& sl=*slots[size_t(idx)];
                const int pid=sl.producer;
                if (!o.cpu_direct_hybrid) enc.set_sidecar(use_sidecar ? sl.sidecar : nullptr, sl.coeff);
                gpr_buffer out{nullptr,0}; size_t vc5_bytes=0;
                const int64_t d0=enc.dng_wrap_ns;
                const auto e0=Clock::now();
                if (o.cpu_direct_hybrid) enc.encode_hybrid(sl.hybrid->frame(),out,vc5_bytes);
                else enc.encode(sl.coeff,out,vc5_bytes);
                const auto e1=Clock::now();
                const int64_t dng=enc.dng_wrap_ns-d0;
                if (e1>=measure_from && e1<measure_to) {
                    std::lock_guard<std::mutex> sk(stats_mu);
                    ++agg[pid].frames;
                    agg[pid].wavelet_ns += std::chrono::duration_cast<Ns>(sl.t2-sl.t1).count();
                    agg[pid].entropy_ns += std::chrono::duration_cast<Ns>(e1-e0).count();
                    agg[pid].dng_ns += dng;
                    agg[pid].free_wait_ns += sl.producer_wait_ns;
                    agg[pid].ready_wait_ns += ready_wait;
                    agg[pid].payload += vc5_bytes; agg[pid].packet += out.size;
                    agg[pid].latency.push_back(std::chrono::duration<double,std::milli>(e1-sl.t0).count());
                    ++ent[eid].jobs; if (stolen) ++ent[eid].steals;
                }
                enc.free_buffer(out);
                {
                    std::lock_guard<std::mutex> lk(mu); sl.state=0; sl.producer=-1;
                }
                cv.notify_all();
            }
            ent[eid].cpu_ns=cpu_gpr_thread_cpu_now_ns()-cpu0;
        } catch (...) {
            { std::lock_guard<std::mutex> lk(err_mu); if (!error) error=std::current_exception(); }
            abort.store(true); cv.notify_all();
        }
    };

    std::thread e0(entropy_body,0), e1(entropy_body,1);
    std::thread p2(producer_body,1), p0(producer_body,0);
    p0.join(); p2.join(); e0.join(); e1.join();
    if (error) std::rethrow_exception(error);

    const auto wall_end=Clock::now();
    const double runtime_s=std::max(1e-9,std::chrono::duration<double>(wall_end-wall_begin).count());
    auto per_ms=[](int64_t ns,long f){return f?double(ns)/double(f)/1e6:0.0;};
    const long f0=agg[0].frames,f2=agg[1].frames,frames=f0+f2;
    const double fps=o.duration>0?double(frames)/o.duration:0.0;
    const double f0ps=o.duration>0?double(f0)/o.duration:0.0;
    const double f2ps=o.duration>0?double(f2)/o.duration:0.0;
    const double c0=100.0*double(prod[0].cpu_ns)/(runtime_s*1e9);
    const double c2=100.0*double(prod[1].cpu_ns)/(runtime_s*1e9);
    const double c1=100.0*double(ent[0].cpu_ns)/(runtime_s*1e9);
    const double c3=100.0*double(ent[1].cpu_ns)/(runtime_s*1e9);
    std::vector<double> lat=agg[0].latency; lat.insert(lat.end(),agg[1].latency.begin(),agg[1].latency.end());
    const uint64_t payload=agg[0].payload+agg[1].payload, packets=agg[0].packet+agg[1].packet;

    auto interval_sum=[](const std::vector<std::pair<Clock::time_point,Clock::time_point>>& v){
        double x=0; for(auto& q:v) x+=std::chrono::duration<double>(q.second-q.first).count(); return x; };
    auto overlap_sum=[](const auto& a,const auto& b){
        size_t i=0,j=0; double x=0;
        while(i<a.size()&&j<b.size()){
            auto lo=std::max(a[i].first,b[j].first), hi=std::min(a[i].second,b[j].second);
            if(hi>lo) x+=std::chrono::duration<double>(hi-lo).count();
            if(a[i].second<b[j].second) ++i; else ++j;
        }
        return x;
    };
    const double aactive=interval_sum(intervals[0]), bactive=interval_sum(intervals[1]);
    const double overlap=overlap_sum(intervals[0],intervals[1]);
    const double denom=std::max(1e-9,o.duration);

    std::cout << "CPU_ADAPTIVE_STAGE_RESULT frames=" << frames
              << " seconds=" << o.duration << " fps=" << fps
              << " core0_frames=" << f0 << " core0_fps=" << f0ps
              << " core2_frames=" << f2 << " core2_fps=" << f2ps
              << " core0_cpu_pct=" << c0 << " core1_cpu_pct=" << c1
              << " core2_cpu_pct=" << c2 << " core3_cpu_pct=" << c3
              << " core0_wavelet_ms=" << per_ms(agg[0].wavelet_ns,f0)
              << " core2_wavelet_ms=" << per_ms(agg[1].wavelet_ns,f2)
              << " core0_entropy_ms=" << per_ms(agg[0].entropy_ns,f0)
              << " core2_entropy_ms=" << per_ms(agg[1].entropy_ns,f2)
              << " core0_dng_ms=" << per_ms(agg[0].dng_ns,f0)
              << " core2_dng_ms=" << per_ms(agg[1].dng_ns,f2)
              << " core0_free_wait_ms=" << per_ms(agg[0].free_wait_ns,f0)
              << " core0_gap_wait_ms=" << per_ms(prod[0].gap_wait_ns,f0)
              << " core2_free_wait_ms=" << per_ms(agg[1].free_wait_ns,f2)
              << " entropy1_jobs=" << ent[0].jobs << " entropy1_steals=" << ent[0].steals
              << " entropy3_jobs=" << ent[1].jobs << " entropy3_steals=" << ent[1].steals
              << " wavelet_core0_active_pct=" << (100.0*aactive/denom)
              << " wavelet_core2_active_pct=" << (100.0*bactive/denom)
              << " wavelet_overlap_pct=" << (100.0*overlap/denom)
              << " latency_p50_ms=" << cpu_stage_percentile(lat,0.50)
              << " latency_p99_ms=" << cpu_stage_percentile(lat,0.99)
              << " avg_gpr_bytes=" << (frames?packets/uint64_t(frames):0)
              << " avg_vc5_bytes=" << (frames?payload/uint64_t(frames):0)
              << " handoff_copy_bytes=0\n";
    std::cout << "CPU_ADAPTIVE_VERIFY shared_slots=" << o.dual_global_slots
              << " reserve_mode=" << (o.dual_core0_soft_reserve ? "soft" : "hard")
              << " work_steal=" << (o.dual_queue_policy=="fifo"?"YES":"YES")
              << " coefficient_handoff=ownership-only copy_bytes=0 context_reuse="
              << (o.cpu_wavelet_reuse_context?"YES":"NO")
              << " direct_hybrid=" << (o.cpu_direct_hybrid?"YES":"NO") << "\n";
    return 0;
}

static int run_cpu_gpr_stage_pipeline(const Options& o, const ModeSpec* mode,
                                      const std::vector<uint16_t>& src,
                                      const std::vector<uint16_t>& lut)
{
    if (o.core0_stage_pipeline == "adaptive")
        return run_cpu_gpr_adaptive_stage_pipeline(o, mode, src, lut);
    if (o.core0_stage_pipeline == "dual")
        return run_cpu_gpr_dual_stage_pipeline(o, mode, src, lut);
    announce_cpu_kernel(o, "cpu-stage");
    if (o.core0_stage_pipeline == "off")
        throw std::runtime_error("internal: staged pipeline called with strategy off");
    if (!o.cpu_v2_kernel)
        throw std::runtime_error("Core-0 stage pipeline currently requires --cpu-v2-kernel on / --cpu-winner on");
    if (o.cpu_gpr_hybrid_entropy)
        throw std::runtime_error("Core-0 stage matrix requires --cpu-gpr-hybrid-entropy off for a clean handoff test");

    const int pw = o.width / 2, ph = o.height / 2;
    const size_t row_stride = size_t(pw);
    const size_t plane_stride = row_stride * size_t(ph);
    const bool use_sidecar = o.cpu_sidecar;
    if (use_sidecar && !o.cpu_nontemporal)
        throw std::runtime_error("staged winner pipeline requires --cpu-nontemporal on; NT-off does not emit the RegDirect sidecar correctly");
    const int wave_core = o.core0_stage_pipeline == "w0e1" ? 0 : 1;
    const int entropy_core = o.core0_stage_pipeline == "w0e1" ? 1 : 0;
    const int normal_cores[2] = {2, 3};

    std::cout << "CPU_STAGE_CONFIG direction=" << o.core0_stage_pipeline
              << " wave_core=" << wave_core
              << " entropy_core=" << entropy_core
              << " normal_cores=" << o.core0_stage_normal_workers
              << " slots=" << o.core0_stage_slots
              << " core0_sched=" << o.core0_stage_sched
              << " stagger_ms=" << o.core0_stage_stagger_ms
              << " nontemporal=" << (g_cpu_nontemporal_bands ? "on" : "off")
              << " sidecar=" << (use_sidecar ? "on" : "off")
              << " handoff=ownership-no-copy"
              << " geometry=" << o.width << "x" << o.height << "\n";

    struct StageSlot {
        std::vector<int16_t> coeff;
        std::vector<uint8_t> sidecar;
        Clock::time_point t0{}, t1{}, t2{};
        int64_t producer_wait_ns = 0;
        int state = 0; // 0 free, 1 producer owns, 2 ready, 3 entropy owns
    };
    std::vector<std::unique_ptr<StageSlot>> slots;
    slots.reserve(size_t(o.core0_stage_slots));
    for (int i = 0; i < o.core0_stage_slots; ++i) {
        auto s = std::make_unique<StageSlot>();
        s->coeff.assign(plane_stride * 4u, int16_t(0));
        if (use_sidecar) s->sidecar.assign(plane_stride * 4u / 8u, 0u);
#if defined(__linux__) && defined(MADV_HUGEPAGE)
        if (o.cpu_gpr_hugepages) {
            madvise(s->coeff.data(), s->coeff.size() * sizeof(int16_t), MADV_HUGEPAGE);
            if (!s->sidecar.empty()) madvise(s->sidecar.data(), s->sidecar.size(), MADV_HUGEPAGE);
        }
#endif
        slots.emplace_back(std::move(s));
    }

    const auto wall_begin = Clock::now();
    const auto measure_from = wall_begin + std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(o.warmup_seconds + o.trim_seconds));
    const auto measure_to = measure_from + std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(o.duration));
    const auto deadline = measure_to + std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(o.trim_seconds));
    const bool timed = o.duration > 0.0;
    std::atomic<int> claimed{0};
    auto claim_work = [&]() -> bool {
        if (timed) return Clock::now() < deadline;
        const int n = claimed.fetch_add(1, std::memory_order_relaxed);
        return n < o.frames;
    };

    std::mutex q_mu, err_mu;
    std::condition_variable q_cv;
    std::deque<int> ready_q;
    bool producer_done = false;
    std::exception_ptr error;
    std::atomic<bool> abort{false};
    CpuStageThreadStats producer_stats, entropy_stats;
    std::array<CpuStageThreadStats,2> normal_stats;
    std::atomic<long> ticker_frames{0};
    std::atomic<uint64_t> ticker_bytes{0};

    auto find_free_locked = [&]() -> int {
        for (int i = 0; i < int(slots.size()); ++i) if (slots[size_t(i)]->state == 0) return i;
        return -1;
    };

    auto transform_into = [&](StageSlot& slot, const std::vector<uint16_t>& raw_in,
                              V2Frame& v2ctx) {
        V2Sidecar sc{};
        if (use_sidecar) { sc.mask_base = slot.sidecar.data(); sc.coeff_base = slot.coeff.data(); }
        v2_fused_frame(use_sidecar ? &sc : nullptr, o, mode, raw_in.data(), size_t(o.width),
                       0, 0, lut, slot.coeff.data(), plane_stride, row_stride,
                       SplitKind::Shipped, EmitKind::RegDirect, v2ctx);
    };

    std::thread producer([&]{
        try {
            cpu_stage_pin_exact_core(o, wave_core, o.core0_stage_sched);
            const int64_t cpu0 = cpu_gpr_thread_cpu_now_ns();
            std::vector<uint16_t> captured(src.size());
            V2Frame v2ctx;
            g_v2_sidecar_zskip = o.cpu_sidecar_zskip;
            g_v2_input_prefetch = o.cpu_input_prefetch;
            while (!abort.load(std::memory_order_relaxed) && claim_work()) {
                const auto wait0 = Clock::now();
                int idx = -1;
                {
                    std::unique_lock<std::mutex> lock(q_mu);
                    q_cv.wait(lock, [&]{ return abort.load() || (idx = find_free_locked()) >= 0; });
                    if (abort.load()) break;
                    slots[size_t(idx)]->state = 1;
                }
                auto& slot = *slots[size_t(idx)];
                slot.producer_wait_ns = std::chrono::duration_cast<Ns>(Clock::now() - wait0).count();
                slot.t0 = Clock::now();
                const std::vector<uint16_t>& raw_in = o.cpu_gpr_raw_copy ? captured : src;
                if (o.cpu_gpr_raw_copy)
                    std::memcpy(captured.data(), src.data(), src.size() * sizeof(uint16_t));
                slot.t1 = Clock::now();
                transform_into(slot, raw_in, v2ctx);
                slot.t2 = Clock::now();
                {
                    std::lock_guard<std::mutex> lock(q_mu);
                    slot.state = 2;
                    ready_q.push_back(idx);
                }
                q_cv.notify_all();
            }
            {
                std::lock_guard<std::mutex> lock(q_mu);
                producer_done = true;
            }
            q_cv.notify_all();
            producer_stats.thread_cpu_ns = cpu_gpr_thread_cpu_now_ns() - cpu0;
        } catch (...) {
            std::lock_guard<std::mutex> lock(err_mu); if (!error) error = std::current_exception();
            abort.store(true); { std::lock_guard<std::mutex> ql(q_mu); producer_done = true; } q_cv.notify_all();
        }
    });

    std::thread entropy([&]{
        try {
            cpu_stage_pin_exact_core(o, entropy_core, o.core0_stage_sched);
            const int64_t cpu0 = cpu_gpr_thread_cpu_now_ns();
            DirectGprEncoder enc(o, mode, row_stride, plane_stride);
            enc.splice_enabled = o.cpu_gpr_dng_splice;
            enc.splice_shared = o.cpu_gpr_dng_splice && o.cpu_gpr_splice_shared;
            enc.splice_reuse = o.cpu_gpr_dng_splice && o.cpu_gpr_splice_reuse;
            for (;;) {
                int idx = -1;
                const auto wait0 = Clock::now();
                {
                    std::unique_lock<std::mutex> lock(q_mu);
                    q_cv.wait(lock, [&]{ return abort.load() || !ready_q.empty() || producer_done; });
                    if (abort.load()) break;
                    if (ready_q.empty()) { if (producer_done) break; else continue; }
                    idx = ready_q.front(); ready_q.pop_front();
                    slots[size_t(idx)]->state = 3;
                }
                const int64_t wait_ready_ns = std::chrono::duration_cast<Ns>(Clock::now() - wait0).count();
                auto& slot = *slots[size_t(idx)];
                enc.set_sidecar(use_sidecar ? slot.sidecar.data() : nullptr, slot.coeff.data());
                gpr_buffer out{nullptr, 0}; size_t vc5_bytes = 0;
                const auto e0 = Clock::now();
                enc.encode(slot.coeff.data(), out, vc5_bytes);
                const auto t3 = Clock::now();
                ticker_frames.fetch_add(1, std::memory_order_relaxed);
                ticker_bytes.fetch_add(out.size, std::memory_order_relaxed);
                if (t3 >= measure_from && t3 < measure_to) {
                    ++entropy_stats.frames;
                    entropy_stats.copy_ns += std::chrono::duration_cast<Ns>(slot.t1 - slot.t0).count();
                    entropy_stats.wavelet_ns += std::chrono::duration_cast<Ns>(slot.t2 - slot.t1).count();
                    entropy_stats.entropy_ns += std::chrono::duration_cast<Ns>(t3 - e0).count();
                    producer_stats.wait_ns += slot.producer_wait_ns;
                    entropy_stats.wait_ns += wait_ready_ns;
                    entropy_stats.payload_bytes += vc5_bytes;
                    entropy_stats.packet_bytes += out.size;
                    entropy_stats.latency_ms.push_back(std::chrono::duration<double,std::milli>(t3 - slot.t0).count());
                }
                enc.free_buffer(out);
                {
                    std::lock_guard<std::mutex> lock(q_mu);
                    slot.state = 0;
                }
                q_cv.notify_all();
            }
            entropy_stats.thread_cpu_ns = cpu_gpr_thread_cpu_now_ns() - cpu0;
        } catch (...) {
            std::lock_guard<std::mutex> lock(err_mu); if (!error) error = std::current_exception();
            abort.store(true); q_cv.notify_all();
        }
    });

    auto normal_body = [&](int ni) {
        try {
            const int core = normal_cores[ni];
            cpu_stage_pin_exact_core(o, core, "normal");
            if (o.core0_stage_stagger_ms > 0)
                std::this_thread::sleep_for(std::chrono::milliseconds((ni + 1) * o.core0_stage_stagger_ms));
            const int64_t cpu0 = cpu_gpr_thread_cpu_now_ns();
            CpuStageThreadStats& st = normal_stats[size_t(ni)];
            std::vector<uint16_t> captured(src.size());
            StageSlot local;
            local.coeff.assign(plane_stride * 4u, int16_t(0));
            if (use_sidecar) local.sidecar.assign(plane_stride * 4u / 8u, 0u);
            V2Frame v2ctx;
            g_v2_sidecar_zskip = o.cpu_sidecar_zskip;
            g_v2_input_prefetch = o.cpu_input_prefetch;
            DirectGprEncoder enc(o, mode, row_stride, plane_stride);
            enc.splice_enabled = o.cpu_gpr_dng_splice;
            enc.splice_shared = o.cpu_gpr_dng_splice && o.cpu_gpr_splice_shared;
            enc.splice_reuse = o.cpu_gpr_dng_splice && o.cpu_gpr_splice_reuse;
            while (!abort.load(std::memory_order_relaxed) && claim_work()) {
                const auto t0 = Clock::now();
                const std::vector<uint16_t>& raw_in = o.cpu_gpr_raw_copy ? captured : src;
                if (o.cpu_gpr_raw_copy)
                    std::memcpy(captured.data(), src.data(), src.size() * sizeof(uint16_t));
                const auto t1 = Clock::now();
                transform_into(local, raw_in, v2ctx);
                const auto t2 = Clock::now();
                enc.set_sidecar(use_sidecar ? local.sidecar.data() : nullptr, local.coeff.data());
                gpr_buffer out{nullptr, 0}; size_t vc5_bytes = 0;
                enc.encode(local.coeff.data(), out, vc5_bytes);
                const auto t3 = Clock::now();
                ticker_frames.fetch_add(1, std::memory_order_relaxed);
                ticker_bytes.fetch_add(out.size, std::memory_order_relaxed);
                if (t3 >= measure_from && t3 < measure_to) {
                    ++st.frames;
                    st.copy_ns += std::chrono::duration_cast<Ns>(t1 - t0).count();
                    st.wavelet_ns += std::chrono::duration_cast<Ns>(t2 - t1).count();
                    st.entropy_ns += std::chrono::duration_cast<Ns>(t3 - t2).count();
                    st.payload_bytes += vc5_bytes;
                    st.packet_bytes += out.size;
                    st.latency_ms.push_back(std::chrono::duration<double,std::milli>(t3 - t0).count());
                }
                enc.free_buffer(out);
            }
            st.thread_cpu_ns = cpu_gpr_thread_cpu_now_ns() - cpu0;
        } catch (...) {
            std::lock_guard<std::mutex> lock(err_mu); if (!error) error = std::current_exception();
            abort.store(true); q_cv.notify_all();
        }
    };

    std::vector<std::thread> normal_threads;
    for (int i = 0; i < o.core0_stage_normal_workers; ++i) normal_threads.emplace_back(normal_body, i);
    producer.join(); entropy.join();
    for (auto& t : normal_threads) t.join();
    if (error) std::rethrow_exception(error);

    const auto wall_end = Clock::now();
    const double runtime_s = std::max(1e-9, std::chrono::duration<double>(wall_end - wall_begin).count());
    const long pair_frames = entropy_stats.frames;
    long normal_frames = 0;
    for (int i = 0; i < o.core0_stage_normal_workers; ++i) normal_frames += normal_stats[size_t(i)].frames;
    const long frames = pair_frames + normal_frames;
    const double fps = o.duration > 0.0 ? double(frames) / o.duration : 0.0;
    const double pair_fps = o.duration > 0.0 ? double(pair_frames) / o.duration : 0.0;
    const double normal_fps = o.duration > 0.0 ? double(normal_frames) / o.duration : 0.0;
    std::vector<double> lat = entropy_stats.latency_ms;
    lat.insert(lat.end(), normal_stats[0].latency_ms.begin(), normal_stats[0].latency_ms.end());
    lat.insert(lat.end(), normal_stats[1].latency_ms.begin(), normal_stats[1].latency_ms.end());
    const double stage_wav_ms = pair_frames ? double(entropy_stats.wavelet_ns) / double(pair_frames) / 1e6 : 0.0;
    const double stage_ent_ms = pair_frames ? double(entropy_stats.entropy_ns) / double(pair_frames) / 1e6 : 0.0;
    int64_t normal_wav_ns = 0, normal_ent_ns = 0;
    for (int i = 0; i < o.core0_stage_normal_workers; ++i) {
        normal_wav_ns += normal_stats[size_t(i)].wavelet_ns;
        normal_ent_ns += normal_stats[size_t(i)].entropy_ns;
    }
    const double normal_wav_ms = normal_frames ? double(normal_wav_ns) / double(normal_frames) / 1e6 : 0.0;
    const double normal_ent_ms = normal_frames ? double(normal_ent_ns) / double(normal_frames) / 1e6 : 0.0;
    const int64_t core0_cpu_ns = wave_core == 0 ? producer_stats.thread_cpu_ns : entropy_stats.thread_cpu_ns;
    const double core0_cpu_pct = 100.0 * double(core0_cpu_ns) / (runtime_s * 1e9);
    const double producer_wait_ms = pair_frames ? double(producer_stats.wait_ns) / double(pair_frames) / 1e6 : 0.0;
    const double entropy_wait_ms = pair_frames ? double(entropy_stats.wait_ns) / double(pair_frames) / 1e6 : 0.0;

    uint64_t payload = entropy_stats.payload_bytes, packets = entropy_stats.packet_bytes;
    for (int i = 0; i < o.core0_stage_normal_workers; ++i) {
        payload += normal_stats[size_t(i)].payload_bytes;
        packets += normal_stats[size_t(i)].packet_bytes;
    }
    std::cout << "CPU_STAGE_RESULT frames=" << frames
              << " seconds=" << o.duration
              << " fps=" << fps
              << " pair_fps=" << pair_fps
              << " normal_fps=" << normal_fps
              << " normal_workers=" << o.core0_stage_normal_workers
              << " direction=" << o.core0_stage_pipeline
              << " slots=" << o.core0_stage_slots
              << " core0_sched=" << o.core0_stage_sched
              << " stagger_ms=" << o.core0_stage_stagger_ms
              << " core0_cpu_pct=" << core0_cpu_pct
              << " pair_wavelet_ms=" << stage_wav_ms
              << " pair_entropy_ms=" << stage_ent_ms
              << " producer_wait_free_ms=" << producer_wait_ms
              << " entropy_wait_ready_ms=" << entropy_wait_ms
              << " normal_wavelet_ms=" << normal_wav_ms
              << " normal_entropy_ms=" << normal_ent_ms
              << " latency_p50_ms=" << cpu_stage_percentile(lat, 0.50)
              << " latency_p99_ms=" << cpu_stage_percentile(lat, 0.99)
              << " avg_gpr_bytes=" << (frames ? packets / uint64_t(frames) : 0)
              << " avg_vc5_bytes=" << (frames ? payload / uint64_t(frames) : 0)
              << " handoff_copy_bytes=0\n";
    std::cout << "CPU_STAGE_CORE core=" << wave_core << " role=wavelet frames=" << pair_frames
              << " cpu_pct=" << (100.0 * double(producer_stats.thread_cpu_ns) / (runtime_s * 1e9))
              << " wait_free_ms=" << producer_wait_ms << "\n";
    std::cout << "CPU_STAGE_CORE core=" << entropy_core << " role=entropy frames=" << pair_frames
              << " cpu_pct=" << (100.0 * double(entropy_stats.thread_cpu_ns) / (runtime_s * 1e9))
              << " wait_ready_ms=" << entropy_wait_ms << "\n";
    for (int i = 0; i < o.core0_stage_normal_workers; ++i)
        std::cout << "CPU_STAGE_CORE core=" << normal_cores[i] << " role=normal frames=" << normal_stats[size_t(i)].frames
                  << " fps=" << (o.duration > 0.0 ? double(normal_stats[size_t(i)].frames) / o.duration : 0.0)
                  << " cpu_pct=" << (100.0 * double(normal_stats[size_t(i)].thread_cpu_ns) / (runtime_s * 1e9)) << "\n";
    std::cout << "CPU_STAGE_VERIFY coefficient_handoff=ownership-only copy_bytes=0 slots="
              << o.core0_stage_slots << " sidecar_handoff=" << (use_sidecar ? "YES" : "NO") << "\n";
    return 0;
}

static int run_cpu_gpr_pipeline(const Options& o, const ModeSpec* mode,
                                const std::vector<uint16_t>& src,
                                const std::vector<uint16_t>& lut) {
    cpu_gpr_apply_entropy_assist_defaults(o);
    announce_cpu_kernel(o, "cpu-gpr");
    const int pw = o.width / 2, ph = o.height / 2;
    const size_t row_stride = size_t(pw);
    const size_t plane_stride = row_stride * size_t(ph);
    const int requested_workers = o.cpu_gpr_threads;
    const bool fused = o.cpu_wavelet_fused && cpu_fused_geometry_ok(o);
    const bool fused_split = fused && o.cpu_split_fused;
    const bool wavelet_assist_enabled = cpu_gpr_wavelet_assist_requested(o) &&
        cpu_fused_geometry_ok(o) && (o.cpu_v2_kernel || fused_split);
    const Core0WaveletPlan core0_plan = wavelet_assist_enabled
        ? cpu_gpr_core0_wavelet_plan(o) : Core0WaveletPlan{};
    const int workers = cpu_gpr_primary_worker_count(o, wavelet_assist_enabled);

    std::cout << "CPU_GPR_CONFIG workers=" << requested_workers
              << " primary_workers=" << workers
              << " wavelet_assist=" << (wavelet_assist_enabled ? "on" : "off")
              << " core0_strategy=" << core0_plan.name
              << " core0_entropy_assist=" << (requested_workers == 4 ? "SB8x4" : "off")
              << " shared_inplace=" << (o.cpu_gpr_shared_inplace ? "on" : "off")
              << " assist_masks=" << cpu_gpr_plane_mask_name(core0_plan.masks[0]) << ","
              << cpu_gpr_plane_mask_name(core0_plan.masks[1]) << ","
              << cpu_gpr_plane_mask_name(core0_plan.masks[2])
              << " geometry=" << o.width << "x" << o.height
              << " fused=" << (fused ? "YES" : "NO")
              << " split_fused=" << (fused_split ? "YES" : "NO")
              << " kernel=" << (g_cpu_wavelet_use_neon ? o.cpu_wavelet_kernel : std::string("scalar"))
              << " vec_blocks=" << g_cpu_wavelet_vec_blocks
              << " nontemporal=" << (g_cpu_nontemporal_bands ? "on" : "off")
              << " raw_copy=" << (o.cpu_gpr_raw_copy ? "on" : "off")
              << " C1malloc=" << (o.cpu_gpr_malloc_tuned ? "on" : "off")
              << " C2affinity=" << (o.cpu_gpr_affinity ? "on" : "off")
              << " C3hugepages=" << (o.cpu_gpr_hugepages ? "on" : "off")
              << " C4prefetch=" << (o.cpu_gpr_prefetch ? "on" : "off")
              << " C5dngsplice=" << (o.cpu_gpr_dng_splice ? "on" : "off")
              << " S2spliceshared=" << (o.cpu_gpr_splice_shared ? "on" : "off")
              << " E2vleskip=" << (o.vle_prequant_skip ? "on" : "off")
              << " H1helper=" << (o.cpu_gpr_helper ? "on" : "off")
              << " H2splicereuse=" << (o.cpu_gpr_splice_reuse ? "on" : "off")
              << " S1splitneon=" << (g_cpu_split_neon ? "on" : "off")
              << " E1hybridentropy=" << (o.cpu_gpr_hybrid_entropy ? "on" : "off")
              << " direct_hybrid=" << (o.cpu_direct_hybrid ? "on" : "off")
              << " wav_nzmask=" << (cinepi_wav_nzmask_enabled() ? "on" : "off")
              << " Y1rt=" << (o.cpu_gpr_rt ? "on" : "off")
              << " true12bit=" << (o.true_12bit ? "on" : "off")
              << " ladder=universal-standard-v3"
              << " compand_bits=" << o.compand_bits
              << " quant_scale=" << ((o.compand_bits < 12) ? (o.compand_quant_scale ? "on" : "OFF") : "n/a")
              << " working_max=" << o.working_max
              << " vulkan=NONE\n";
    if (o.cpu_wavelet_fused && !fused)
        std::cout << "CPU_GPR_WARN fused declined: plane dims not divisible by 8; "
                     "falling back to the six-pass schedule, which over-reads on "
                     "odd level heights\n";

    std::mutex assign_mu, error_mu;
    int next_frame = 0;
    std::atomic<bool> stop{false};
    std::exception_ptr error;
    std::vector<CpuGprWorkerStats> stats(static_cast<size_t>(workers));
    // H1. Three workers leave one core idle. Releasing a multi-megabyte GPR
    // buffer is pure overhead on the worker's critical path, so it can be
    // handed over instead. The queue is bounded: if the helper ever fell
    // behind, unbounded retirement would grow memory without limit, so the
    // worker frees inline rather than queue without end.
    std::mutex retire_mu;
    std::condition_variable retire_cv;
    std::deque<void*> retire_q;
    std::atomic<bool> retire_stop{false};
    std::atomic<long long> retired{0}, retired_inline{0};
    constexpr size_t RETIRE_MAX = 24;
    std::thread helper;
    if (o.cpu_gpr_helper) {
        helper = std::thread([&]{
            cpu_gpr_place_aux(o);
            for (;;) {
                void* p = nullptr;
                {
                    std::unique_lock<std::mutex> lock(retire_mu);
                    retire_cv.wait(lock, [&]{ return !retire_q.empty() || retire_stop.load(); });
                    if (retire_q.empty()) return;
                    p = retire_q.front(); retire_q.pop_front();
                }
                std::free(p);
                retired.fetch_add(1, std::memory_order_relaxed);
                retire_cv.notify_all();
            }
        });
    }
    auto retire_buffer = [&](gpr_buffer& b, DirectGprEncoder& e) {
        if (!b.buffer) { b = {nullptr, 0}; return; }
        // v1.15.1: shared/local in-place outputs may belong to the encoder's
        // handoff pool.  Recycle them before the generic free/helper path so
        // 3-worker mode gets the same retained-container behaviour as the
        // adaptive staged path.
        if (e.recycle_pooled_buffer(b)) return;
        if (e.owns_output(b)) { b = {nullptr, 0}; return; }   // H2: nothing to free
        if (o.cpu_gpr_helper) {
            std::unique_lock<std::mutex> lock(retire_mu);
            if (retire_q.size() < RETIRE_MAX) {
                retire_q.push_back(b.buffer);
                lock.unlock();
                retire_cv.notify_one();
                b = {nullptr, 0};
                return;
            }
            lock.unlock();
            retired_inline.fetch_add(1, std::memory_order_relaxed);
        }
        std::free(b.buffer);
        b = {nullptr, 0};
    };
    std::atomic<long> saved_count{0};
    std::atomic<long> warm_frames{0};   // discarded warmup frames, reported
    if (!o.save_gpr.empty()) std::filesystem::create_directories(o.save_gpr);
    const bool timed = o.duration > 0.0;

#if defined(__linux__) && defined(M_MMAP_THRESHOLD)
    if (o.cpu_gpr_malloc_tuned) {
        // Keep the multi-megabyte GPR output on the heap so it is reused rather
        // than mmap'd and faulted in fresh every frame.
        mallopt(M_MMAP_THRESHOLD, 64 * 1024 * 1024);
        mallopt(M_TRIM_THRESHOLD, 64 * 1024 * 1024);
    }
#endif
    if (wavelet_assist_enabled)
        verify_wavelet_assist_plan(o, mode, src, lut, plane_stride, row_stride,
                                   o.cpu_v2_kernel, core0_plan);

    std::unique_ptr<WaveletPlaneAssistant> shared_wavelet_assistant;
    std::array<std::unique_ptr<WaveletPlaneAssistant>,3> dedicated_wavelet_assistants;
    if (wavelet_assist_enabled) {
        if (core0_plan.shared) {
            /* A shared strategy uses one identical plane mask for all three
               owners. Do NOT hard-code GD here: the search matrix deliberately
               compares GS/RG/BG/GD and multi-plane shared masks. A previous
               build always constructed the shared helper with mask 8 (GD),
               which made labelled GS/RG/BG/pair tests silently benchmark GD. */
            const uint8_t shared_mask = core0_plan.masks[0];
            if (!shared_mask || core0_plan.masks[1] != shared_mask ||
                core0_plan.masks[2] != shared_mask)
                throw std::runtime_error("invalid shared Core-0 wavelet plan: masks must match and be non-zero");
            shared_wavelet_assistant = std::make_unique<WaveletPlaneAssistant>(
                o, mode, plane_stride, row_stride, o.cpu_v2_kernel, shared_mask, 0);
        } else {
            for (int w = 0; w < 3; ++w) if (core0_plan.masks[size_t(w)])
                dedicated_wavelet_assistants[size_t(w)] = std::make_unique<WaveletPlaneAssistant>(
                    o, mode, plane_stride, row_stride, o.cpu_v2_kernel,
                    core0_plan.masks[size_t(w)], w);
        }
    }
    const auto assist_runtime_begin = Clock::now();

    /* v0.29 warmup: encode for warmup_seconds, discard those frames, then
       measure. `start` is the measurement origin, so fps and the per-stage
       ms are computed over the steady-state window only. */
    const auto wall_begin = Clock::now();
    /* v1.9 edge trim. measure_from/measure_to bracket the AVERAGED window;
       the deadline sits a further trim_seconds out so the pipeline is still
       fully loaded when counting stops. */
    const auto measure_from = wall_begin + std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(o.warmup_seconds + o.trim_seconds));
    const auto start = measure_from;
    const auto measure_to = start + std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(o.duration));
    const auto deadline = measure_to + std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(o.trim_seconds));
    if (o.warmup_seconds > 0.0)
        std::cout << "CPU_GPR_WARMUP seconds=" << o.warmup_seconds
                  << " (frames before the window are discarded)\n";

    // v1.16.3 production-candidate phase lock. Diagnostics showed the fast M7
    // state is a BURST: all three primary wavelets begin almost together, then
    // the memory system gets a quiet entropy interval. The bad state is a
    // near-continuous two-wavelet conveyor. This cyclic rendezvous tries to
    // recreate the measured 62.52-fps burst without imposing a concurrency cap.
    // A short timeout is fail-open: an OS-delayed worker can never stall the
    // other two indefinitely. A common release timestamp removes CV wake-order
    // jitter while adding no work to Core0. Both controls are disabled at zero.
    auto env_int = [](const char* name, int defv) {
        if (const char* e = std::getenv(name)) {
            try { return std::stoi(e); } catch (...) {}
        }
        return defv;
    };
    const int default_rendezvous_us = workers >= 2 ? 20000 : 0;
    const int wavelet_rendezvous_us = std::max(0,
        env_int("CINEPI_WAVELET_RENDEZVOUS_US", default_rendezvous_us));
    const int wavelet_release_lead_us = std::max(0, std::min(500,
        env_int("CINEPI_WAVELET_RELEASE_LEAD_US", 40)));
    std::mutex wave_rv_mu;
    std::condition_variable wave_rv_cv;
    int wave_rv_generation = 0;
    int wave_rv_arrived = 0;
    Clock::time_point wave_rv_release_at = wall_begin;
    std::atomic<long> wave_rv_all3_releases{0};
    std::atomic<long> wave_rv_timeout_releases{0};

    auto wait_until_precise = [&](Clock::time_point target) {
        if (wavelet_release_lead_us <= 0) return;
        for (;;) {
            const auto now = Clock::now();
            if (now >= target) break;
            const auto left = std::chrono::duration_cast<std::chrono::microseconds>(target - now).count();
            if (left > 80) std::this_thread::sleep_until(target - std::chrono::microseconds(50));
            else std::this_thread::yield();
        }
    };

    auto wave_rendezvous_enter = [&]() -> int64_t {
        if (wavelet_rendezvous_us <= 0) return 0;
        const auto wait_begin = Clock::now();
        Clock::time_point release_at;
        {
            std::unique_lock<std::mutex> lk(wave_rv_mu);
            const int my_generation = wave_rv_generation;
            ++wave_rv_arrived;
            if (wave_rv_arrived == workers) {
                wave_rv_arrived = 0;
                ++wave_rv_generation;
                wave_rv_release_at = Clock::now() + std::chrono::microseconds(wavelet_release_lead_us);
                release_at = wave_rv_release_at;
                wave_rv_all3_releases.fetch_add(1, std::memory_order_relaxed);
                lk.unlock();
                wave_rv_cv.notify_all();
            } else {
                const auto deadline_rv = wait_begin + std::chrono::microseconds(wavelet_rendezvous_us);
                while (wave_rv_generation == my_generation) {
                    if (wave_rv_cv.wait_until(lk, deadline_rv) == std::cv_status::timeout &&
                        wave_rv_generation == my_generation) {
                        // Fail open. All waiters from this generation are
                        // released together; a late worker joins the next one.
                        wave_rv_arrived = 0;
                        ++wave_rv_generation;
                        wave_rv_release_at = Clock::now() + std::chrono::microseconds(wavelet_release_lead_us);
                        wave_rv_timeout_releases.fetch_add(1, std::memory_order_relaxed);
                        lk.unlock();
                        wave_rv_cv.notify_all();
                        lk.lock();
                        break;
                    }
                }
                release_at = wave_rv_release_at;
                lk.unlock();
            }
        }
        wait_until_precise(release_at);
        return std::chrono::duration_cast<Ns>(Clock::now() - wait_begin).count();
    };

    std::cout << "CPU_WAVELET_RENDEZVOUS_CONFIG timeout_us=" << wavelet_rendezvous_us
              << " release_lead_us=" << (wavelet_rendezvous_us > 0 ? wavelet_release_lead_us : 0) << "\n";

    struct CpuCoeffBuffers {
        std::vector<int16_t> coeff;
        std::vector<uint8_t> sidecar;
        /* v3.16: the 3-worker static bench can now run the direct tile-hybrid
           wavelet as well, so the sidecar and the hybrid can be A/B'd against
           each other on exactly the harness that produced the shipped static
           numbers, instead of only on the 4-core adaptive staged path. */
        std::unique_ptr<CpuDirectHybridSlot> hybrid;
    };
    const bool direct_hybrid = o.cpu_direct_hybrid && o.cpu_v2_kernel;
    if (o.cpu_direct_hybrid && !o.cpu_v2_kernel)
        throw std::runtime_error("--cpu-direct-hybrid requires --cpu-v2-kernel on / --cpu-winner on");
    /* Fail closed rather than silently ignore. The Core-0 wavelet assistant
       publishes rows into the int16 coefficient frame, which the direct
       hybrid does not allocate, and E1's repack provider reads that same
       frame back. Neither has a compact-pool equivalent. */
    if (direct_hybrid && wavelet_assist_enabled)
        throw std::runtime_error("--cpu-direct-hybrid cannot be combined with the Core-0 wavelet assistant: "
                                 "the assistant publishes into the int16 coefficient frame the hybrid never writes");
    if (direct_hybrid && o.cpu_gpr_hybrid_entropy)
        throw std::runtime_error("--cpu-direct-hybrid cannot be combined with --cpu-gpr-hybrid-entropy: "
                                 "E1 repacks a completed int16 frame, which the direct hybrid never produces");
    std::vector<std::unique_ptr<CpuCoeffBuffers>> cpu_coeffs;
    cpu_coeffs.reserve(size_t(workers));
    for (int w = 0; w < workers; ++w) {
        auto b = std::make_unique<CpuCoeffBuffers>();
        if (direct_hybrid)
            b->hybrid = std::make_unique<CpuDirectHybridSlot>(uint32_t(pw), uint32_t(ph));
        else
            b->coeff.assign(plane_stride * 4u, int16_t(0));
        if (o.cpu_v2_kernel && o.cpu_sidecar)
            b->sidecar.assign(plane_stride * 4u / 8u, 0u);
        cpu_coeffs.emplace_back(std::move(b));
    }
    if (direct_hybrid) {
        /* One-shot proof, before the timing window, that the compact pool
           holds exactly the coefficients the full int16 emit would have
           written AND that its nonzero mask agrees with them bit for bit. */
        auto probe = std::make_unique<CpuDirectHybridSlot>(uint32_t(pw), uint32_t(ph));
        V2Frame pctx;
        v2_fused_frame_compact(o, mode, src.data(), size_t(o.width), 0, 0, lut, *probe, pctx);
        std::vector<int16_t> ref(plane_stride * 4u);
        V2Frame rctx;
        v2_fused_frame(nullptr, o, mode, src.data(), size_t(o.width), 0, 0, lut,
                       ref.data(), plane_stride, row_stride,
                       SplitKind::Shipped, EmitKind::RegDirect, rctx);
        if (!probe->verify_against(ref.data(), row_stride, plane_stride))
            throw std::runtime_error("direct hybrid coefficient/mask verification failed");
        std::cout << "CPU_DIRECT_HYBRID_VERIFY bitexact=YES nzmask="
                  << (probe->nz_mask_enabled() ? "on" : "off")
                  << " int8_tiles=" << probe->int8_tiles()
                  << " fallback_tiles=" << probe->fallback_tiles()
                  << " total_tiles=" << probe->total_tiles()
                  << " compact_mib=" << (double(probe->logical_compact_bytes())/1048576.0)
                  << " full_int16_mib=" << (double(plane_stride*4u*sizeof(int16_t))/1048576.0) << "\n";
    }

    // Live telemetry, same contract as the capture path so one UI reads both.
    std::atomic<long> ticker_frames{0};
    std::atomic<uint64_t> ticker_bytes{0};
    std::atomic<bool> ticker_stop{false};
    std::mutex ticker_mu;
    std::condition_variable ticker_cv;
    std::thread ticker;
    if (o.capture_tick_ms > 0) {
        ticker = std::thread([&]{
            cpu_gpr_place_aux(o);
#if defined(__linux__)
            // UI telemetry is advisory. Keep it below the normal-priority
            // Core0 SB8 helper so printing a status line cannot delay entropy
            // into the next synchronized wavelet burst. Nice is per-thread on
            // Linux; encoder workers/helper remain unchanged at normal priority.
            (void)setpriority(PRIO_PROCESS, 0, 10);
#endif
            const auto tick = std::chrono::milliseconds(o.capture_tick_ms);
            struct Sample { double t; long n; };
            std::deque<Sample> window;
            long avg_base_n = -1;
            double avg_base_t = 0.0;
            while (!ticker_stop.load(std::memory_order_relaxed)) {
                {
                    std::unique_lock<std::mutex> lk(ticker_mu);
                    if (ticker_cv.wait_for(lk, tick, [&]{ return ticker_stop.load(std::memory_order_relaxed); }))
                        break;
                }
                const double el = std::chrono::duration<double>(Clock::now() - start).count();
                if (el < 0.0) continue; // do not perturb/report warmup + leading trim
                const long n = ticker_frames.load(std::memory_order_relaxed);
                const uint64_t b = ticker_bytes.load(std::memory_order_relaxed);
                if (avg_base_n < 0) { avg_base_n = n; avg_base_t = el; }
                window.push_back({el, n});
                // Cohort-aware live rate. Three synchronized owners complete
                // frames in groups of three, so a 1-second two-sample rate
                // aliases a smooth ~59 fps stream into artificial 57/60 fps
                // steps. Keep three samples (two tick intervals) instead.
                while (window.size() > 3) window.pop_front();
                const double dt = el - window.front().t;
                const double fps_now = dt > 0.0 ? double(n - window.front().n) / dt : 0.0;
                const double fps_avg = (el > avg_base_t)
                    ? double(n - avg_base_n) / (el - avg_base_t) : 0.0;
                const double avg_bytes = n > 0 ? double(b) / double(n) : 0.0;
                std::cout << "ENCODE_TICK t=" << el
                          << " duration=" << o.duration
                          << " frames=" << n
                          << " fps_now=" << fps_now
                          << " fps_avg=" << fps_avg
                          << " gpr_bytes=" << uint64_t(avg_bytes)
                          << " ratio=" << (avg_bytes > 0.0
                                 ? double(size_t(o.width) * size_t(o.height) * 2u) / avg_bytes : 0.0)
                          << std::endl;
            }
        });
    }

    // Core-0 wavelet assistant is created before the timing window. It owns no
    // frame: primaries opportunistically submit only the GD wavelet plane and
    // keep entropy/container work local.

    auto body = [&](int id) {
        try {
            cpu_gpr_place_worker(o, id, wavelet_assist_enabled);
            std::vector<uint16_t> captured(src.size());
            auto& db = *cpu_coeffs[size_t(id)];
#if defined(__linux__) && defined(MADV_HUGEPAGE)
            if (o.cpu_gpr_hugepages) {
                madvise(captured.data(), captured.size() * sizeof(uint16_t), MADV_HUGEPAGE);
                madvise(db.coeff.data(), db.coeff.size() * sizeof(int16_t), MADV_HUGEPAGE);
            }
#endif
            std::vector<int16_t> planes;
            if (!fused_split) planes.assign(plane_stride * 4u, int16_t(0));
            V2Frame v2ctx;
            g_v2_sidecar_zskip = o.cpu_sidecar_zskip;
            g_v2_input_prefetch = o.cpu_input_prefetch;
            CpuFusedContext ctx;
            DirectGprEncoder enc(o, mode, row_stride, plane_stride);
            enc.splice_enabled = o.cpu_gpr_dng_splice;
            enc.splice_shared = o.cpu_gpr_dng_splice && o.cpu_gpr_splice_shared;
            enc.splice_reuse = o.cpu_gpr_dng_splice && o.cpu_gpr_splice_reuse;
            std::unique_ptr<CpuTileHybridProvider> hyb;
            if (o.cpu_gpr_hybrid_entropy)
                hyb = std::make_unique<CpuTileHybridProvider>(
                    1, uint32_t(row_stride), uint32_t(plane_stride / row_stride),
                    row_stride, plane_stride);
            constexpr int HYBRID_VERIFY_FRAMES = 3;
            int hyb_ok = 0; bool hyb_dead = false;
            CpuGprWorkerStats& s = stats[size_t(id)];

            for (;;) {
                {
                    std::lock_guard<std::mutex> lock(assign_mu);
                    if (stop.load(std::memory_order_relaxed)) break;
                    if (timed) {
                        if (next_frame > 0 && Clock::now() >= deadline) break;
                    } else if (next_frame >= o.frames) break;
                    ++next_frame;
                }

                const auto t0 = Clock::now();
                auto& coeff = db.coeff;
                auto& sidecar = db.sidecar;
                const std::vector<uint16_t>& raw_in =
                    o.cpu_gpr_raw_copy ? captured : src;
                if (o.cpu_gpr_raw_copy)
                    std::memcpy(captured.data(), src.data(), src.size() * sizeof(uint16_t));
                const auto t1 = Clock::now();
                const int64_t sync_wait_ns = wave_rendezvous_enter();
                const auto wave_exec_begin = Clock::now();

                const bool use_sc = o.cpu_v2_kernel && o.cpu_sidecar && !sidecar.empty();
                WaveletPlaneAssistant* frame_assistant = nullptr;
                if (shared_wavelet_assistant) frame_assistant = shared_wavelet_assistant.get();
                else if (id >= 0 && id < 3 && dedicated_wavelet_assistants[size_t(id)])
                    frame_assistant = dedicated_wavelet_assistants[size_t(id)].get();
                const uint8_t assist_mask = frame_assistant ? frame_assistant->plane_mask() : uint8_t(0);
                WaveletPlaneAssistant::Ticket assist_ticket{};
                int64_t assist_publish_wait_ns = 0;
                if (frame_assistant)
                    assist_ticket = frame_assistant->try_begin(
                        coeff.data(), use_sc ? sidecar.data() : nullptr, use_sc);
                auto publish_rows = [&](const std::array<const int16_t*,4>& rows) {
                    assist_publish_wait_ns += frame_assistant->publish_row(assist_ticket, rows);
                };

                if (direct_hybrid) {
                    /* The wavelet writes int8 8x8 tiles (plus the exact int16
                       fallback for the rare out-of-range tile) and the
                       per-coefficient nonzero mask, straight out of the
                       vertical filter. No 16.6 MB int16 frame is written and
                       none is read back. */
                    v2_fused_frame_compact(o, mode, raw_in.data(), size_t(o.width), 0, 0,
                                           lut, *db.hybrid, v2ctx);
                } else if (o.cpu_v2_kernel) {
                    V2Sidecar sc{};
                    if (use_sc) { sc.mask_base = sidecar.data(); sc.coeff_base = coeff.data(); }
                    if (assist_ticket)
                        v2_fused_owner_rows(use_sc ? &sc : nullptr, o, mode, raw_in.data(),
                                            size_t(o.width), 0, 0, lut, coeff.data(),
                                            plane_stride, row_stride,
                                            SplitKind::Shipped, EmitKind::RegDirect, v2ctx,
                                            assist_mask, publish_rows);
                    else
                        v2_fused_frame(use_sc ? &sc : nullptr, o, mode, raw_in.data(),
                                       size_t(o.width), 0, 0, lut, coeff.data(),
                                       plane_stride, row_stride,
                                       SplitKind::Shipped, EmitKind::RegDirect, v2ctx);
                } else if (fused_split) {
                    if (assist_ticket)
                        fused_split_transform_owner_rows(raw_in.data(), lut.data(), o.width, o.height,
                                                         o.bayer == "gbrg",
                                                         o.true_12bit ? 0 : 1, 4096,
                                                         coeff.data(), plane_stride, row_stride,
                                                         cpu_quant_table(o, mode), ctx,
                                                         assist_mask, publish_rows);
                    else
                        cpu_fused_frame_from_raw(o, mode, raw_in.data(), lut, coeff.data(),
                                                 plane_stride, row_stride, ctx);
                } else {
                    split_compand(o, raw_in.data(), lut, planes.data(), plane_stride);
                    if (fused)
                        cpu_fused_frame_from_planes(o, mode, planes.data(), coeff.data(),
                                                    plane_stride, row_stride, ctx);
                    else {
                        std::memcpy(coeff.data(), planes.data(), coeff.size() * sizeof(int16_t));
                        cpu_transform_tail_inplace(o, mode, coeff.data(),
                                                   plane_stride, row_stride, 1);
                    }
                }
                int64_t assist_wait_ns = 0;
                if (assist_ticket)
                    assist_wait_ns = frame_assistant->wait(assist_ticket);
                const auto wave_exec_end = Clock::now();
                const auto t2 = wave_exec_end;
                // Record the true wavelet execution interval, excluding any
                // admission wait, clipped to the measurement window.
                const auto clip_beg = std::max(wave_exec_begin, measure_from);
                const auto clip_end = std::min(wave_exec_end, measure_to);
                if (clip_end > clip_beg) {
                    s.wave_intervals_ns.emplace_back(
                        std::chrono::duration_cast<Ns>(clip_beg - measure_from).count(),
                        std::chrono::duration_cast<Ns>(clip_end - measure_from).count());
                    s.wavelet_exec_ns += std::chrono::duration_cast<Ns>(clip_end - clip_beg).count();
                }
                if (t2 >= measure_from && t2 < measure_to) s.wavelet_sync_wait_ns += sync_wait_ns;
                if (!direct_hybrid)
                    enc.set_sidecar(use_sc ? sidecar.data() : nullptr, coeff.data());

                gpr_buffer out{nullptr, 0};
                size_t vc5_bytes = 0;
                if (direct_hybrid) {
                    enc.encode_hybrid(db.hybrid->frame(), out, vc5_bytes);
                } else if (hyb && !hyb_dead) {
                    if (hyb_ok < HYBRID_VERIFY_FRAMES) {
                        // Both paths, byte-compared. The quantised coefficients
                        // are identical, so the containers must be too.
                        gpr_buffer ref{nullptr, 0}; size_t ref_bytes = 0;
                        enc.encode(coeff.data(), ref, ref_bytes);
                        hyb->prepare(0, coeff.data());
                        enc.encode_hybrid(hyb->frame(0), out, vc5_bytes);
                        const bool same = ref.size == out.size && ref.buffer && out.buffer &&
                            std::memcmp(ref.buffer, out.buffer, ref.size) == 0;
                        if (same) { enc.free_buffer(ref); ++hyb_ok; ++s.hyb_frames; }
                        else {
                            // Keep the reference output; never trust E1 again.
                            enc.free_buffer(out); out = ref; vc5_bytes = ref_bytes;
                            hyb_dead = true; s.hyb_failed = true;
                        }
                    } else {
                        hyb->prepare(0, coeff.data());
                        enc.encode_hybrid(hyb->frame(0), out, vc5_bytes);
                        ++s.hyb_frames;
                    }
                    if (!hyb_dead) {
                        const auto& hm = hyb->metrics(0);
                        s.hyb_int8_tiles += hm.int8_tiles;
                        s.hyb_total_tiles += hm.total_tiles;
                    }
                    s.hyb_verified = hyb_ok;
                } else {
                    enc.encode(coeff.data(), out, vc5_bytes);
                }
                const auto t3 = Clock::now();

                if (!o.save_gpr.empty()) {
                    const long ticket = saved_count.fetch_add(1, std::memory_order_relaxed);
                    if (o.save_gpr_limit == 0 || ticket < o.save_gpr_limit) {
                        /* Settings ID in the filename, same fields and order
                         * the live encoder stamps (cinepi-raw/qraw_encoder.cpp),
                         * so a saved frame stays self-describing once it leaves
                         * its directory and a bench frame can be compared with
                         * a camera frame by name. Constant within a run, and it
                         * follows the frame index, so *.gpr globs and name
                         * sorts are unaffected. */
                        static const std::string settings_id = [&o] {
                            std::ostringstream t;
                            t << (o.mode.empty() ? "baseq" : o.mode)
                              << "-c" << o.compand_bits;
                            if (o.compand_inframe_bits < 12)
                                t << "-i" << o.compand_inframe_bits;
                            if ((o.compand_bits < 12 || o.compand_inframe_bits < 12)
                                && !o.compand_quant_scale)
                                t << "-qsoff";
                            t << "-b" << o.effective_bits
                              << "-w" << o.cpu_gpr_threads;
                            if (o.log_strength != 599.0)
                                t << "-k" << std::llround(o.log_strength);
                            if (o.gradation_compand) t << "-grad";
                            if (o.true_12bit)        t << "-t12";
                            if (const char *bt = std::getenv("CINEPI_BAND_TAG");
                                bt && *bt) {
                                std::string s;
                                for (char c : std::string(bt)) {
                                    if (std::isalnum(static_cast<unsigned char>(c))
                                        || c == '-' || c == '.') s += c;
                                    else s += '-';
                                    if (s.size() >= 48) break;
                                }
                                while (!s.empty() && s.back() == '-') s.pop_back();
                                if (!s.empty()) t << '-' << s;
                            }
                            return t.str();
                        }();
                        char name[192];
                        std::snprintf(name, sizeof(name), "frame_%05ld_%s.gpr", ticket,
                                      settings_id.c_str());
                        std::ofstream gf(std::filesystem::path(o.save_gpr) / name,
                                         std::ios::binary | std::ios::trunc);
                        gf.write(static_cast<const char*>(out.buffer),
                                 std::streamsize(out.size));
                        if (!gf) throw std::runtime_error("GPR write failed in " + o.save_gpr);
                    } else {
                        saved_count.fetch_sub(1, std::memory_order_relaxed);
                    }
                }
                if (t3 >= measure_from && t3 < measure_to) s.writer_ns += elapsed_ns(t3);

                s.dng_ns = enc.dng_wrap_ns;
                s.splice_hits_out = enc.splice_hits;
                s.splice_fb_out = enc.splice_fallbacks;
                s.splice_active_out = enc.splice_active;
                s.splice_failed_out = enc.splice_failed;
                if (t3 >= measure_from && t3 < measure_to) {
                    s.payload_bytes += vc5_bytes;
                    s.packet_bytes += out.size;
                }
                ticker_frames.fetch_add(1, std::memory_order_relaxed);
                ticker_bytes.fetch_add(out.size, std::memory_order_relaxed);
                retire_buffer(out, enc);

                /* v0.29: frames finished before the measurement window are
                   warmup -- the work is done for real (caches, file extents
                   and the allocator all reach steady state) but nothing is
                   counted, so fps and the stage breakdown describe the
                   steady state only. */
                if (t3 < measure_from || t3 >= measure_to) {
                    /* Outside the averaged window: real work, still done, so
                       the machine stays in steady state -- but not counted.
                       Before the window it is warmup; after it, the tail. */
                    ++warm_frames;
                } else {
                    s.latency_ms.push_back(
                        std::chrono::duration<double, std::milli>(t3 - t0).count());
                    s.copy_ns += std::chrono::duration_cast<Ns>(t1 - t0).count();
                    s.wavelet_ns += std::chrono::duration_cast<Ns>(t2 - t1).count();
                    s.entropy_ns += std::chrono::duration_cast<Ns>(t3 - t2).count();
                    if (assist_ticket) {
                        ++s.wavelet_assist_frames;
                        s.wavelet_assist_wait_ns += assist_wait_ns;
                        s.wavelet_assist_publish_wait_ns += assist_publish_wait_ns;
                    }
                    ++s.frames;
                }
            }
        } catch (...) {
            std::lock_guard<std::mutex> lock(error_mu);
            if (!error) error = std::current_exception();
            stop.store(true, std::memory_order_relaxed);
        }
    };

    std::vector<std::thread> extra;
    for (int w = 1; w < workers; ++w) extra.emplace_back(body, w);
    body(0);
    for (auto& t : extra) t.join();
    if (shared_wavelet_assistant) shared_wavelet_assistant->stop();
    for (auto& a : dedicated_wavelet_assistants) if (a) a->stop();
    const auto assist_runtime_end = Clock::now();
    int64_t assist_helper_cpu_ns = 0, assist_helper_wall_ns = 0;
    long long assist_submitted = 0, assist_completed = 0, assist_busy_misses = 0, assist_rows = 0;
    auto gather_assist = [&](const std::unique_ptr<WaveletPlaneAssistant>& a) {
        if (!a) return;
        assist_helper_cpu_ns += a->stats.helper_cpu_ns.load(std::memory_order_relaxed);
        assist_helper_wall_ns += a->stats.helper_wall_ns.load(std::memory_order_relaxed);
        assist_submitted += a->stats.submitted.load(std::memory_order_relaxed);
        assist_completed += a->stats.completed.load(std::memory_order_relaxed);
        assist_busy_misses += a->stats.busy_misses.load(std::memory_order_relaxed);
        assist_rows += a->stats.rows_published.load(std::memory_order_relaxed);
    };
    gather_assist(shared_wavelet_assistant);
    for (const auto& a : dedicated_wavelet_assistants) gather_assist(a);
    const double assist_runtime_s = std::chrono::duration<double>(
        assist_runtime_end - assist_runtime_begin).count();
    const double assist_core0_cpu_pct = assist_runtime_s > 0.0
        ? 100.0 * double(assist_helper_cpu_ns) / (assist_runtime_s * 1e9) : 0.0;
    if (helper.joinable()) {
        retire_stop.store(true);
        retire_cv.notify_all();
        helper.join();
    }
    ticker_stop.store(true);
    ticker_cv.notify_all();
    if (ticker.joinable()) ticker.join();
    if (error) std::rethrow_exception(error);

    /* The averaged window, not wall time to here: everything after
       measure_to is trim and its frames were not counted. */
    const double seconds =
        std::chrono::duration<double>(measure_to - start).count();
    if (o.warmup_seconds > 0.0)
        std::cout << "CPU_GPR_WARMUP_DONE discarded_frames=" << warm_frames.load()
                  << " measured_seconds=" << seconds << "\n";
    long frames = 0;
    int64_t copy = 0, wav = 0, ent = 0, dng = 0, wrt = 0;
    long sp_hits = 0, sp_fb = 0; bool sp_on = false, sp_dead = false;
    long hy_frames = 0; int hy_ver = 0; bool hy_dead = false;
    uint64_t hy_i8 = 0, hy_tot = 0;
    uint64_t payload = 0, packet = 0;
    long assisted_frames = 0;
    int64_t assist_wait = 0;
    int64_t assist_publish_wait = 0;
    std::vector<double> latencies;
    std::vector<std::pair<int64_t,int>> wave_events;
    int64_t wave_sync_wait_total = 0, wave_exec_total = 0;
    long wave_interval_count = 0;
    for (const auto& s : stats) {
        frames += s.frames; copy += s.copy_ns; wav += s.wavelet_ns; ent += s.entropy_ns;
        payload += s.payload_bytes; packet += s.packet_bytes; dng += s.dng_ns; wrt += s.writer_ns;
        sp_hits += s.splice_hits_out; sp_fb += s.splice_fb_out;
        sp_on = sp_on || s.splice_active_out; sp_dead = sp_dead || s.splice_failed_out;
        hy_frames += s.hyb_frames; hy_ver = std::max(hy_ver, s.hyb_verified);
        hy_dead = hy_dead || s.hyb_failed; hy_i8 += s.hyb_int8_tiles; hy_tot += s.hyb_total_tiles;
        assisted_frames += s.wavelet_assist_frames;
        assist_wait += s.wavelet_assist_wait_ns;
        assist_publish_wait += s.wavelet_assist_publish_wait_ns;
        wave_sync_wait_total += s.wavelet_sync_wait_ns;
        wave_exec_total += s.wavelet_exec_ns;
        wave_interval_count += long(s.wave_intervals_ns.size());
        for (const auto& iv : s.wave_intervals_ns) {
            wave_events.emplace_back(iv.first, +1);
            wave_events.emplace_back(iv.second, -1);
        }
        latencies.insert(latencies.end(), s.latency_ms.begin(), s.latency_ms.end());
    }
    std::sort(latencies.begin(), latencies.end());
    auto pct = [&](double q) {
        if (latencies.empty()) return 0.0;
        size_t i = size_t(q * double(latencies.size() - 1) + 0.5);
        return latencies[std::min(i, latencies.size() - 1)];
    };
    if (!frames) throw std::runtime_error("cpu-gpr pipeline completed no frames");
    const double f = double(frames);
    const double fps = seconds > 0.0 ? f / seconds : 0.0;
    const size_t input_bytes = size_t(o.width) * size_t(o.height) * 2u;

    // Reconstruct concurrency from per-worker intervals without perturbing
    // the timed hot path. This is the direct evidence for/against phase
    // collapse into three simultaneous wavelets.
    std::array<int64_t,4> wave_conc_ns{{0,0,0,0}};
    std::vector<int64_t> wave_starts;
    for (size_t i=0; i+1<wave_events.size(); i+=2) (void)i;
    for (const auto& st : stats) for (const auto& iv : st.wave_intervals_ns) wave_starts.push_back(iv.first);
    std::sort(wave_starts.begin(), wave_starts.end());
    std::sort(wave_events.begin(), wave_events.end(), [](const auto& a, const auto& b){
        if (a.first != b.first) return a.first < b.first;
        return a.second < b.second; // end before start at identical timestamp
    });
    int cur_wave = 0; int64_t prev_ns = 0;
    const int64_t measure_ns = std::chrono::duration_cast<Ns>(measure_to - measure_from).count();
    for (const auto& ev : wave_events) {
        const int64_t t = std::max<int64_t>(0, std::min<int64_t>(measure_ns, ev.first));
        if (t > prev_ns && cur_wave >= 0 && cur_wave <= 3) wave_conc_ns[size_t(cur_wave)] += t - prev_ns;
        cur_wave += ev.second; prev_ns = t;
    }
    if (prev_ns < measure_ns && cur_wave >= 0 && cur_wave <= 3) wave_conc_ns[size_t(cur_wave)] += measure_ns - prev_ns;
    std::vector<int64_t> start_gaps;
    for (size_t i=1;i<wave_starts.size();++i) start_gaps.push_back(wave_starts[i]-wave_starts[i-1]);
    auto pctile_ns = [](std::vector<int64_t> v, double q)->double {
        if (v.empty()) return 0.0; std::sort(v.begin(), v.end());
        const size_t idx = std::min(v.size()-1, size_t(q * double(v.size()-1)));
        return double(v[idx]);
    };
    const double wave_exec_ms = wave_interval_count > 0 ? double(wave_exec_total)/1e6/double(wave_interval_count) : 0.0;
    const double sync_wait_ms = frames > 0 ? double(wave_sync_wait_total)/1e6/double(frames) : 0.0;
    std::cout << "CPU_WAVELET_CONCURRENCY rendezvous_us=" << wavelet_rendezvous_us
              << " release_lead_us=" << (wavelet_rendezvous_us > 0 ? wavelet_release_lead_us : 0)
              << " actual_wavelet_ms=" << wave_exec_ms
              << " sync_wait_ms_per_frame=" << sync_wait_ms
              << " all3_releases=" << wave_rv_all3_releases.load(std::memory_order_relaxed)
              << " timeout_releases=" << wave_rv_timeout_releases.load(std::memory_order_relaxed)
              << " c0_pct=" << (measure_ns?100.0*double(wave_conc_ns[0])/double(measure_ns):0.0)
              << " c1_pct=" << (measure_ns?100.0*double(wave_conc_ns[1])/double(measure_ns):0.0)
              << " c2_pct=" << (measure_ns?100.0*double(wave_conc_ns[2])/double(measure_ns):0.0)
              << " c3_pct=" << (measure_ns?100.0*double(wave_conc_ns[3])/double(measure_ns):0.0)
              << " starts=" << wave_starts.size()
              << " gap_p10_us=" << pctile_ns(start_gaps,0.10)/1000.0
              << " gap_p50_us=" << pctile_ns(start_gaps,0.50)/1000.0
              << " gap_p90_us=" << pctile_ns(start_gaps,0.90)/1000.0
              << "\n";

    std::cout << "CPU_QRAW_RESULT frames=" << frames
              << " seconds=" << seconds
              << " fps=" << fps
              << " workers=" << requested_workers
              << " primary_workers=" << workers
              << " wavelet_assist=" << (wavelet_assist_enabled ? "on" : "off")
              << " core0_strategy=" << core0_plan.name
              << " core0_assist_cpu_pct=" << assist_core0_cpu_pct
              << " assisted_frames=" << assisted_frames
              << " assisted_fps=" << (seconds > 0.0 ? double(assisted_frames) / seconds : 0.0)
              << " assist_owner_wait_ms=" << (assisted_frames ? double(assist_wait) / double(assisted_frames) / 1e6 : 0.0)
              << " assist_publish_wait_ms=" << (assisted_frames ? double(assist_publish_wait) / double(assisted_frames) / 1e6 : 0.0)
              << " ms_copy=" << (double(copy) / f / 1e6)
              << " ms_wavelet=" << (double(wav) / f / 1e6)
              << " ms_entropy=" << (double(ent) / f / 1e6)
              << " ms_frame_per_worker=" << (double(copy + wav + ent) / f / 1e6)
              << " dng_wrap_ms=" << (double(dng) / f / 1e6)
              << " writer_ms=" << (double(wrt) / f / 1e6)
              << " latency_p50_ms=" << pct(0.50)
              << " latency_p99_ms=" << pct(0.99)
              << " mode=" << (o.mode.empty() ? std::string("base-q") + std::to_string(o.quant) : o.mode)
              << " vc5_bytes=" << (payload / uint64_t(frames))
              << " gpr_bytes=" << (packet / uint64_t(frames))
              << " ratio=" << (double(input_bytes) / (double(packet) / f))
              << " sensor_max_fps=" << o.sensor_max_fps
              << " over_sensor=" << ((o.sensor_max_fps > 0.0 &&
                                      fps > o.sensor_max_fps + 0.01) ? 1 : 0)
              << " output_mibs=" << (double(packet) / f * fps / 1048576.0)
              << "\n";
    /* "How fast" measures the ENCODER, replaying one frame from RAM. It is
       not a capture and may legitimately exceed any sensor -- say so, so the
       number is not read as a recordable frame rate. */
    if (o.sensor_max_fps > 0.0 && fps > o.sensor_max_fps + 0.01)
        std::cout << "CPU_GPR_OVER_SENSOR encoder_fps=" << fps
                  << " sensor_max_fps=" << o.sensor_max_fps
                  << " note=encoder-headroom-not-a-recordable-rate\n";
    for (int w = 0; w < workers; ++w) {
        const auto& ws = stats[size_t(w)];
        const double wfps = seconds > 0.0 ? double(ws.frames) / seconds : 0.0;
        std::cout << "CPU_GPR_WORKER id=" << w
                  << " role=primary"
                  << " frames=" << ws.frames
                  << " fps=" << wfps
                  << " assisted_frames=" << ws.wavelet_assist_frames
                  << " assist_wait_ms=" << (ws.wavelet_assist_frames
                         ? double(ws.wavelet_assist_wait_ns) / double(ws.wavelet_assist_frames) / 1e6 : 0.0)
                  << " publish_wait_ms=" << (ws.wavelet_assist_frames
                         ? double(ws.wavelet_assist_publish_wait_ns) / double(ws.wavelet_assist_frames) / 1e6 : 0.0)
                  << "\n";
    }
    if (wavelet_assist_enabled) {
        const double helper_job_wall_ms = assist_completed
            ? double(assist_helper_wall_ns) / double(assist_completed) / 1e6 : 0.0;
        std::cout << "CPU_GPR_WAVELET_ASSIST strategy=" << core0_plan.name
                  << " masks=" << cpu_gpr_plane_mask_name(core0_plan.masks[0]) << ","
                  << cpu_gpr_plane_mask_name(core0_plan.masks[1]) << ","
                  << cpu_gpr_plane_mask_name(core0_plan.masks[2])
                  << " assisted_frames=" << assisted_frames
                  << " assisted_fps=" << (seconds > 0.0 ? double(assisted_frames) / seconds : 0.0)
                  << " helper_job_wall_ms=" << helper_job_wall_ms
                  << " core0_assist_cpu_pct=" << assist_core0_cpu_pct
                  << " owner_wait_ms=" << (assisted_frames ? double(assist_wait) / double(assisted_frames) / 1e6 : 0.0)
                  << " publish_wait_ms=" << (assisted_frames ? double(assist_publish_wait) / double(assisted_frames) / 1e6 : 0.0)
                  << " submitted=" << assist_submitted
                  << " completed=" << assist_completed
                  << " busy_misses=" << assist_busy_misses
                  << " rows=" << assist_rows
                  << " transport=row-ring"
                  << " ring_rows=" << o.core0_wavelet_ring_rows
                  << " sched=" << o.core0_wavelet_sched
                  << " stagger_ms=" << o.cpu_gpr_stagger_ms
                  << " stagger_us=" << o.cpu_gpr_stagger_us
                  << " raw_reread=NO coefficient_handoff_bytes=0 entropy_handoff=NO\n";
        if (shared_wavelet_assistant) {
            std::cout << "CPU_GPR_WAVELET_HELPER id=0 shared=YES planes="
                      << cpu_gpr_plane_mask_name(shared_wavelet_assistant->plane_mask())
                      << " completed=" << shared_wavelet_assistant->stats.completed.load(std::memory_order_relaxed)
                      << " busy_misses=" << shared_wavelet_assistant->stats.busy_misses.load(std::memory_order_relaxed) << "\n";
        }
        for (int i = 0; i < 3; ++i) if (dedicated_wavelet_assistants[size_t(i)]) {
            const auto& a = dedicated_wavelet_assistants[size_t(i)];
            std::cout << "CPU_GPR_WAVELET_HELPER id=" << i << " shared=NO planes="
                      << cpu_gpr_plane_mask_name(a->plane_mask())
                      << " completed=" << a->stats.completed.load(std::memory_order_relaxed)
                      << " busy_misses=" << a->stats.busy_misses.load(std::memory_order_relaxed) << "\n";
        }
    }
    if (o.cpu_gpr_helper)
        std::cout << "CPU_GPR_HELPER retired=" << retired.load()
                  << " freed_inline=" << retired_inline.load()
                  << " note=inline-means-the-helper-fell-behind-and-the-queue-was-full\n";
    if (o.cpu_gpr_hybrid_entropy)
        std::cout << "CPU_GPR_HYBRID verified=" << (hy_ver >= 3 ? "YES" : "NO")
                  << " failed=" << (hy_dead ? "YES" : "NO")
                  << " hybrid_frames=" << hy_frames
                  << " int8_tile_pct=" << (hy_tot ? 100.0 * double(hy_i8) / double(hy_tot) : 0.0)
                  << " note=first-3-frames-per-worker-byte-compared-against-plain-path\n";
    if (o.cpu_gpr_dng_splice) {
        // In shared mode the template lives in g_shared_splice, not per encoder.
        const bool on = o.cpu_gpr_splice_shared
            ? g_shared_splice.ready.load(std::memory_order_acquire) : sp_on;
        const bool dead = o.cpu_gpr_splice_shared
            ? g_shared_splice.failed.load(std::memory_order_acquire) : sp_dead;
        std::cout << "CPU_GPR_SPLICE shared=" << (o.cpu_gpr_splice_shared ? "YES" : "NO")
                  << " verified=" << (on ? "YES" : "NO")
                  << " failed=" << (dead ? "YES" : "NO")
                  << " spliced_frames=" << sp_hits
                  << " size_fallbacks=" << sp_fb
                  << " note=first-frames-byte-compared-against-SDK-output\n";
    }
    // Stage shares tell you what to optimise next. They are per-worker sums, so
    // they add to ms_frame_per_worker, not to the wall clock.
    const double total = double(copy + wav + ent);
    if (total > 0.0)
        std::cout << "CPU_GPR_SHARE copy_pct=" << (100.0 * double(copy) / total)
                  << " wavelet_pct=" << (100.0 * double(wav) / total)
                  << " entropy_pct=" << (100.0 * double(ent) / total)
                  << " dng_wrap_ms=" << (double(dng) / f / 1e6)
                  << " dng_wrap_pct=" << (100.0 * double(dng) / total)
                  << " note=dng_wrap_is_inside_entropy_and_is_process-wide-serialised\n";
    return 0;
}



// Strict end-to-end worker benchmark. The encoder application creates no
// capture, split, GPU-completion, entropy-service, or writer service threads.
// Worker 0 runs on the calling/main thread. With --threads N, exactly N-1 extra
// std::threads are created. Each worker owns one Vulkan slot, one RAW
// copy buffer, and one DirectGprEncoder, and performs the complete frame path:
// RAW copy -> GP-Log2/component split -> Vulkan submit/wait -> VC-5 entropy ->
// GPR wrap/accounting. The shared Vulkan queue is externally synchronised as
// required by Vulkan. Driver/runtime threads created internally by MoltenVK or
// macOS are outside the application worker count and are reported separately.
static Result run_strict_workers(VulkanPipeline& vp, GpuHybridPacker* hybrid_packer,
                                 GpuOnePassHybridPacker* onepass_packer,
                                 CpuBandHybridProvider* cpu_hybrid_provider,
                                 CpuTileHybridProvider* cpu_tile_provider,
                                 FusedMirrorHybridProvider* fused_provider,
                                 TileDirectHybridProvider* tile_provider,
                                 std::vector<std::unique_ptr<DirectGprEncoder>>& direct_encoders,
                                 const ModeSpec* run_mode, const Options& o,
                                 const std::vector<uint16_t>& src,
                                 const std::vector<uint16_t>& lut,
                                 int fixed_frames, double duration, bool collect,
                                 std::ostream* csv, gpr_buffer* final_gpr) {
    (void)run_mode;
    const int worker_count = int(direct_encoders.size());
    if (worker_count < 1)
        throw std::runtime_error("strict-workers requires at least one DirectGprEncoder instance");
    if (int(vp.slots.size()) < worker_count)
        throw std::runtime_error("strict-workers has fewer Vulkan slots than processing workers");

    std::vector<FrameStat> stats;
    if (collect && duration <= 0.0) stats.reserve(size_t(fixed_frames));

    const auto start = Clock::now();
    const auto deadline = start + std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(duration));
    Clock::time_point submission_end = start;

    std::mutex assign_mu;
    std::mutex gpu_submit_mu;
    std::mutex gpu_inflight_mu;
    std::condition_variable gpu_inflight_cv;
    const int gpu_inflight_limit = effective_gpu_inflight_limit(o, worker_count);
    int gpu_inflight_active = 0;
    std::mutex stats_mu;
    std::mutex final_mu;
    std::mutex dump_mu;
    bool coefficient_dumped = false;
    std::mutex error_mu;
    std::exception_ptr error;
    // v1.0.2: strict-workers had the v2 kernel wired since v0.23 but never
    // announced it, so this was the one pipeline where you could not see
    // which encoder ran. All three now report identically.
    if (o.cpu_wavelet_workers > 0) announce_cpu_kernel(o, "strict-workers");
    std::atomic<bool> stop{false};
    std::atomic<int> completed{0};
    int next_frame = 0;
    int retained_final_frame = -1;

    auto set_error = [&](std::exception_ptr ep) {
        std::lock_guard<std::mutex> lock(error_mu);
        if (!error) error = ep;
        stop.store(true, std::memory_order_relaxed);
        gpu_inflight_cv.notify_all();
    };

    auto worker_body = [&](int worker) {
        const std::string stage_name = "strict-worker-" + std::to_string(worker);
        try {
            log_event("INFO", stage_name.c_str(), "stage started");
            std::vector<uint16_t> captured(src.size());
            auto& slot = vp.slots[size_t(worker)];
            DirectGprEncoder& direct = *direct_encoders[size_t(worker)];

            // ---- v1.7.53 frame-parallel dual engine ------------------------
            // The last --cpu-wavelet-workers workers run the ENTIRE wavelet on
            // the CPU on their own frames. They never enqueue Vulkan work and
            // never occupy a GPU in-flight slot, so the only things the two
            // engines share are memory bandwidth and CPU cores. Both draw from
            // the same next_frame counter, which is what makes the aggregate
            // rate self-balancing: whichever engine is faster simply takes more
            // frames, and no static ratio has to be guessed.
            const bool cpu_engine = worker >= worker_count - o.cpu_wavelet_workers;
            // A CPU-engine worker must not run five plane passes of wavelet
            // inside a Vulkan HOST_VISIBLE mapping; on V3D that allocation is
            // not guaranteed to be cached, and an uncached read-modify-write
            // would dominate everything else. It gets ordinary heap memory.
            // v1.7.54: the fused cascade writes to a separate coefficient
            // buffer, which is what makes fusion safe at all. With the split
            // folded in too, the intermediate plane frame is not allocated.
            const bool fused_ok = cpu_engine && o.cpu_wavelet_fused &&
                                  cpu_fused_geometry_ok(o);
            const bool fused_split = fused_ok && o.cpu_split_fused;
            std::vector<int16_t> cpu_plane, cpu_coeff;
            V2Frame cpu_v2ctx;
            std::vector<uint8_t> cpu_sidecar;
            CpuFusedContext cpu_fused_ctx;
            if (cpu_engine) {
                if (fused_ok) cpu_coeff.assign(vp.plane_stride_elems * 4u, int16_t(0));
                if (o.cpu_v2_kernel) {
                    cpu_coeff.assign(vp.plane_stride_elems * 4u, int16_t(0));
                    if (o.cpu_sidecar)
                        cpu_sidecar.assign(vp.plane_stride_elems * 4u / 8u, 0u);
                    g_v2_sidecar_zskip = o.cpu_sidecar_zskip;
                    g_v2_input_prefetch = o.cpu_input_prefetch;
                }
                if (!fused_split) cpu_plane.assign(vp.plane_stride_elems * 4u, int16_t(0));
            }
            int16_t* const cpu_result = cpu_engine
                ? (fused_ok ? cpu_coeff.data() : cpu_plane.data()) : nullptr;
            int16_t* const split_target = cpu_engine ? cpu_plane.data() : slot.mapped_ping;

            for (;;) {
                int frame = -1;
                {
                    std::lock_guard<std::mutex> lock(assign_mu);
                    if (stop.load(std::memory_order_relaxed)) break;
                    if (duration <= 0.0) {
                        if (next_frame >= fixed_frames) break;
                    } else if (next_frame > 0 && Clock::now() >= deadline) {
                        break;
                    }
                    frame = next_frame++;
                }

                if (o.capture_fps > 0.0) {
                    const auto release_time = start + std::chrono::duration_cast<Clock::duration>(
                        std::chrono::duration<double>(double(frame) / o.capture_fps));
                    std::this_thread::sleep_until(release_time);
                }

                slot.frame = frame;
                slot.start = Clock::now();
                slot.vc5_bytes = 0;
                slot.crc = 0;
                slot.int8_range_ns = 0;
                slot.int8_pack_ns = 0;
                slot.cpu_tail_ns = 0;
                slot.snapshot_ns = 0;
                slot.hybrid_bytes = 0;
                slot.int8_bands = 0;
                slot.int16_fallback_bands = 0;
                slot.int8_tiles = 0;
                slot.int16_fallback_tiles = 0;
                slot.total_tiles = 0;

                // Include the simulated RAW DMA/capture copy in the same worker's
                // CPU input-stage timing. There is no separate capture thread.
                const auto split_start = Clock::now();
                std::memcpy(captured.data(), src.data(), src.size() * sizeof(uint16_t));
                if (!fused_split)
                    split_compand(o, captured.data(), lut, split_target, vp.plane_stride_elems);
                slot.split_ns = elapsed_ns(split_start);

                // The CPU engine's transform stands where the GPU submit/wait
                // stands for a GPU worker, so the stage boundaries and the
                // per-frame latency stay comparable between the two. With the
                // split fused, split_ns is ~0 and its cost appears here instead.
                if (cpu_engine) {
                    const auto cpu_wavelet_start = Clock::now();
                    if (o.cpu_v2_kernel) {
                        // strict-workers' CPU engine uses the same promoted
                        // kernel as the other two pipelines.
                        V2Sidecar sc{};
                        const bool use_sc = o.cpu_sidecar && !cpu_sidecar.empty();
                        if (use_sc) {
                            sc.mask_base = cpu_sidecar.data();
                            sc.coeff_base = cpu_coeff.data();
                        }
                        v2_fused_frame(use_sc ? &sc : nullptr, o, run_mode,
                                       captured.data(), size_t(o.width), 0, 0, lut,
                                       cpu_coeff.data(), vp.plane_stride_elems,
                                       vp.row_stride, SplitKind::Shipped,
                                       EmitKind::RegDirect, cpu_v2ctx);
                        direct.set_sidecar(use_sc ? cpu_sidecar.data() : nullptr,
                                           cpu_coeff.data());
                    }
                    else if (fused_split)
                        cpu_fused_frame_from_raw(o, run_mode, captured.data(), lut, cpu_coeff.data(),
                                                 vp.plane_stride_elems, vp.row_stride,
                                                 cpu_fused_ctx);
                    else if (fused_ok)
                        cpu_fused_frame_from_planes(o, run_mode, cpu_plane.data(),
                                                    cpu_coeff.data(), vp.plane_stride_elems,
                                                    vp.row_stride, cpu_fused_ctx);
                    else
                        cpu_transform_tail_inplace(o, run_mode, cpu_plane.data(),
                                                   vp.plane_stride_elems, vp.row_stride, 1);
                    slot.cpu_tail_ns = elapsed_ns(cpu_wavelet_start);
                    slot.gpu_wall_ns = 0;
                    slot.gpu_exec_ns = 0;
                }

                // VkQueue submission requires external synchronisation, but fence
                // waiting does not. A separate in-flight limiter allows multiple
                // worker-owned command buffers to be queued without inventing
                // service threads.
                if (!cpu_engine) {
                    std::unique_lock<std::mutex> lock(gpu_inflight_mu);
                    gpu_inflight_cv.wait(lock, [&]{
                        return stop.load(std::memory_order_relaxed) ||
                               gpu_inflight_active < gpu_inflight_limit;
                    });
                    if (stop.load(std::memory_order_relaxed)) break;
                    ++gpu_inflight_active;
                }
                if (!cpu_engine) try {
                    if (onepass_packer) {
                        {
                            std::lock_guard<std::mutex> lock(gpu_submit_mu);
                            onepass_packer->submit_with_transform(size_t(worker));
                            submission_end = std::max(submission_end, slot.submit);
                        }
                        onepass_packer->wait_and_prepare(size_t(worker));
                        const auto& hm = onepass_packer->metrics(size_t(worker));
                        slot.int8_range_ns = hm.range_ns;
                        slot.int8_pack_ns = hm.pack_ns;
                        slot.hybrid_bytes = hm.aligned_bytes;
                        slot.int8_bands = hm.int8_bands;
                        slot.int16_fallback_bands = hm.fallback_bands;
                    } else {
                        {
                            std::lock_guard<std::mutex> lock(gpu_submit_mu);
                            vp.submit(slot);
                            submission_end = std::max(submission_end, slot.submit);
                        }
                        vp.wait(slot);
                        if (hybrid_packer) {
                            {
                                std::lock_guard<std::mutex> lock(gpu_submit_mu);
                                hybrid_packer->submit_range(size_t(worker));
                            }
                            hybrid_packer->wait_range_and_prepare_pack(size_t(worker));
                            {
                                std::lock_guard<std::mutex> lock(gpu_submit_mu);
                                hybrid_packer->submit_pack(size_t(worker));
                            }
                            hybrid_packer->wait_pack(size_t(worker));
                            const auto& hm = hybrid_packer->metrics(size_t(worker));
                            slot.int8_range_ns = hm.range_ns;
                            slot.int8_pack_ns = hm.pack_ns;
                            slot.hybrid_bytes = hm.aligned_bytes;
                            slot.int8_bands = hm.int8_bands;
                            slot.int16_fallback_bands = hm.fallback_bands;
                        } else if (fused_provider) {
                            fused_provider->prepare(size_t(worker));
                            const auto& hm = fused_provider->metrics(size_t(worker));
                            slot.int8_range_ns = hm.range_ns;
                            slot.int8_pack_ns = hm.pack_ns;
                            slot.hybrid_bytes = hm.aligned_bytes;
                            slot.int8_bands = hm.int8_bands;
                            slot.int16_fallback_bands = hm.fallback_bands;
                        } else if (tile_provider) {
                            tile_provider->prepare(size_t(worker));
                            const auto& hm = tile_provider->metrics(size_t(worker));
                            slot.int8_range_ns = 0;
                            slot.int8_pack_ns = 0;
                            slot.hybrid_bytes = hm.physical_bytes;
                            slot.int8_tiles = hm.int8_tiles;
                            slot.int16_fallback_tiles = hm.fallback_tiles;
                            slot.total_tiles = hm.total_tiles;
                        }
                    }
                } catch (...) {
                    {
                        std::lock_guard<std::mutex> lock(gpu_inflight_mu);
                        --gpu_inflight_active;
                    }
                    gpu_inflight_cv.notify_all();
                    throw;
                }
                if (!cpu_engine) {
                    {
                        std::lock_guard<std::mutex> lock(gpu_inflight_mu);
                        --gpu_inflight_active;
                    }
                    gpu_inflight_cv.notify_one();
                }

                const auto entropy_start = Clock::now();
                std::vector<int16_t> tile_reconstructed;
                const int16_t* coeff_src = nullptr;
                if (cpu_engine) {
                    // Already a complete int16 coefficient frame, already in
                    // cacheable host memory. No readback, no snapshot.
                    coeff_src = cpu_result;
                } else if (tile_provider) {
                    if ((o.hybrid_verify_baseline && slot.frame == 0) ||
                        (collect && !o.dump_coeff.empty())) {
                        tile_reconstructed = tile_provider->reconstruct_full_int16(size_t(worker));
                        coeff_src = tile_reconstructed.data();
                    }
                } else if (vp.retained_int16_levels) {
                    if (vp.use_readback_copy || (collect && !o.dump_coeff.empty())) {
                        vp.gather_fused_coefficients(slot, slot.readback);
                        coeff_src = slot.readback.data();
                    }
                } else {
                    const size_t total = vp.plane_stride_elems * 4u;
                    if (o.gpu_levels == 0) {
                        slot.readback.resize(total);
                        std::memcpy(slot.readback.data(), slot.mapped_ping,
                                    total * sizeof(int16_t));
                        coeff_src = slot.readback.data();
                    } else {
                        coeff_src = vp.coefficient_ptr(slot);
                        if (vp.use_readback_copy) {
                            const auto copy_start = Clock::now();
                            slot.readback.resize(total);
                            std::memcpy(slot.readback.data(), coeff_src,
                                        total * sizeof(int16_t));
                            slot.snapshot_ns = elapsed_ns(copy_start);
                            coeff_src = slot.readback.data();
                        }
                    }
                }
                if (!cpu_engine && o.gpu_levels < 3) {
                    if (!coeff_src) throw std::runtime_error("CPU wavelet tail has no coefficient source");
                    const auto tail_start = Clock::now();
                    cpu_transform_tail_inplace(o, run_mode, const_cast<int16_t*>(coeff_src),
                                               vp.plane_stride_elems, vp.row_stride,
                                               o.gpu_levels == 0 ? 1 : 2);
                    slot.cpu_tail_ns = elapsed_ns(tail_start);
                }
                if (cpu_hybrid_provider) {
                    cpu_hybrid_provider->prepare(size_t(worker), coeff_src);
                    const auto& hm = cpu_hybrid_provider->metrics(size_t(worker));
                    slot.int8_range_ns = hm.range_ns;
                    slot.int8_pack_ns = hm.pack_ns;
                    slot.hybrid_bytes = hm.aligned_bytes;
                    slot.int8_bands = hm.int8_bands;
                    slot.int16_fallback_bands = hm.fallback_bands;
                }
                if (cpu_tile_provider) {
                    cpu_tile_provider->prepare(size_t(worker), coeff_src);
                    const auto& hm = cpu_tile_provider->metrics(size_t(worker));
                    slot.int8_range_ns = hm.range_ns;
                    slot.int8_pack_ns = hm.pack_ns;
                    slot.hybrid_bytes = hm.physical_bytes;
                    slot.int8_tiles = hm.int8_tiles;
                    slot.int16_fallback_tiles = hm.fallback_tiles;
                    slot.total_tiles = hm.total_tiles;
                }
                if (collect && !o.dump_coeff.empty()) {
                    std::lock_guard<std::mutex> lock(dump_mu);
                    if (!coefficient_dumped) {
                        if (!coeff_src) {
                            if (tile_provider) {
                                tile_reconstructed = tile_provider->reconstruct_full_int16(size_t(worker));
                                coeff_src = tile_reconstructed.data();
                            } else if (vp.retained_int16_levels) {
                                vp.gather_fused_coefficients(slot, slot.readback);
                                coeff_src = slot.readback.data();
                            }
                        }
                        if (!coeff_src)
                            throw std::runtime_error("coefficient dump has no int16 reconstruction");
                        const std::filesystem::path dump_path(o.dump_coeff);
                        if (dump_path.has_parent_path())
                            std::filesystem::create_directories(dump_path.parent_path());
                        std::ofstream dump(dump_path, std::ios::binary | std::ios::trunc);
                        if (!dump)
                            throw std::runtime_error("cannot create coefficient dump: " + dump_path.string());
                        const size_t total = vp.plane_stride_elems * 4u;
                        dump.write(reinterpret_cast<const char*>(coeff_src),
                                   std::streamsize(total * sizeof(int16_t)));
                        if (!dump)
                            throw std::runtime_error("coefficient dump write failed: " + dump_path.string());
                        coefficient_dumped = true;
                        log_event("INFO", "coeff-dump", "wrote " + dump_path.string() +
                                  " bytes=" + std::to_string(total * sizeof(int16_t)));
                    }
                }
                const vc5_pretransformed_hybrid_frame* hybrid_frame = nullptr;
                if (cpu_tile_provider) hybrid_frame = &cpu_tile_provider->frame(size_t(worker));
                else if (cpu_hybrid_provider) hybrid_frame = &cpu_hybrid_provider->frame(size_t(worker));
                else if (hybrid_packer) hybrid_frame = &hybrid_packer->frame(size_t(worker));
                else if (onepass_packer) hybrid_frame = &onepass_packer->frame(size_t(worker));
                else if (fused_provider) hybrid_frame = &fused_provider->frame(size_t(worker));
                else if (tile_provider) hybrid_frame = &tile_provider->frame(size_t(worker));
                if (hybrid_frame) {
                    if (o.hybrid_verify_baseline && slot.frame == 0) {
                        if (!coeff_src) {
                            tile_reconstructed = tile_provider->reconstruct_full_int16(size_t(worker));
                            coeff_src = tile_reconstructed.data();
                        }
                        gpr_buffer baseline{nullptr,0};
                        size_t baseline_vc5_bytes = 0;
                        direct.encode(coeff_src, baseline, baseline_vc5_bytes);
                        direct.encode_hybrid(*hybrid_frame, slot.gpr, slot.vc5_bytes);
                        const bool exact = baseline.size == slot.gpr.size &&
                            baseline_vc5_bytes == slot.vc5_bytes &&
                            std::memcmp(baseline.buffer, slot.gpr.buffer, baseline.size) == 0;
                        direct.free_buffer(baseline);
                        if (!exact) throw std::runtime_error("hybrid GPR differs from int16 baseline");
                        std::lock_guard<std::mutex> output_lock(g_log_mu);
                        std::cout << "HYBRID_GPR_VERIFY exact_gpr=YES frame=0 mode="
                                  << (o.mode.empty()?std::string("base-q")+std::to_string(o.quant):o.mode)
                                  << " storage=" << coeff_storage_name(o) << "\n";
                    } else {
                        direct.encode_hybrid(*hybrid_frame, slot.gpr, slot.vc5_bytes);
                    }
                } else if (!cpu_engine && vp.retained_int16_levels && !vp.use_readback_copy) {
                    direct.encode_levels(vp.fused_level_planes(slot), slot.gpr, slot.vc5_bytes);
                } else {
                    if (!coeff_src) throw std::runtime_error("strict entropy stage has no coefficient source");
                    direct.encode(coeff_src, slot.gpr, slot.vc5_bytes);
                }
                slot.entropy_ns = elapsed_ns(entropy_start);

                const auto writer_start = Clock::now();
                slot.crc_computed = should_compute_frame_crc(o.no_crc, collect, slot.frame);
                slot.crc = slot.crc_computed ? crc32_bytes(slot.gpr.buffer, slot.gpr.size) : 0u;
                const size_t packet = slot.gpr.size;
                slot.writer_ns = elapsed_ns(writer_start);
                slot.finish = Clock::now();

                const int completed_now = completed.fetch_add(1, std::memory_order_relaxed) + 1;
                if (collect) {
                    FrameStat stat;
                    stat.frame = slot.frame;
                    stat.engine = cpu_engine ? 1 : 0;
                    stat.latency = to_ms(std::chrono::duration_cast<Ns>(
                        slot.finish - slot.start).count());
                    stat.split = to_ms(slot.split_ns);
                    stat.gpu = to_ms(slot.gpu_wall_ns);
                    stat.gpu_exec = slot.gpu_exec_ns < 0 ? -1.0 : to_ms(slot.gpu_exec_ns);
                    stat.snapshot = to_ms(slot.snapshot_ns);
                    stat.snapshot_bytes = (slot.snapshot_ns > 0 || vp.use_vulkan_readback_copy) ? coefficient_int16_bytes(o) : 0u;
                    stat.cpu_tail = to_ms(slot.cpu_tail_ns);
                    stat.int8_range = to_ms(slot.int8_range_ns);
                    stat.int8_pack = to_ms(slot.int8_pack_ns);
                    stat.hybrid_bytes = slot.hybrid_bytes;
                    stat.int8_bands = slot.int8_bands;
                    stat.int16_fallback_bands = slot.int16_fallback_bands;
                    stat.int8_tiles = slot.int8_tiles;
                    stat.int16_fallback_tiles = slot.int16_fallback_tiles;
                    stat.total_tiles = slot.total_tiles;
                    stat.entropy = to_ms(slot.entropy_ns);
                    stat.writer = to_ms(slot.writer_ns);
                    stat.payload = slot.vc5_bytes;
                    stat.packet = packet;
                    stat.crc = slot.crc;
                    stat.crc_computed = slot.crc_computed;
                    stat.finish = slot.finish;

                    {
                        std::lock_guard<std::mutex> lock(stats_mu);
                        stats.push_back(stat);
                        if (csv) {
                            (*csv) << slot.frame << ',' << worker << ','
                                   << std::fixed << std::setprecision(3)
                                   << stat.latency << ',' << stat.split << ',' << stat.gpu << ','
                                   << stat.gpu_exec << ',' << stat.snapshot << ',' << stat.snapshot_bytes << ','
                                   << stat.cpu_tail << ',' << stat.int8_range << ',' << stat.int8_pack << ','
                                   << stat.hybrid_bytes << ',' << stat.int8_bands << ','
                                   << stat.int16_fallback_bands << ',' << stat.int8_tiles << ','
                                   << stat.int16_fallback_tiles << ',' << stat.total_tiles << ','
                                   << stat.entropy << ',' << stat.writer << ','
                                   << stat.payload << ',' << stat.packet << ',' << stat.crc << ','
                                   << (stat.crc_computed ? "YES" : "NO") << ','
                                   << (stat.engine ? "cpu" : "gpu") << '\n';
                            csv->flush();
                            if (!*csv) throw std::runtime_error("per-frame CSV write failed");
                        }
                    }

                    if (completed_now == 1 || completed_now % o.progress_every == 0 ||
                        (duration <= 0.0 && completed_now == fixed_frames)) {
                        log_event("INFO", "progress",
                                  "completed=" + std::to_string(completed_now) +
                                  " worker=" + std::to_string(worker) +
                                  " latency_ms=" + std::to_string(stat.latency) +
                                  " vc5_bytes=" + std::to_string(stat.payload) +
                                  " gpr_bytes=" + std::to_string(stat.packet));
                    }
                }

                if (final_gpr && collect) {
                    std::lock_guard<std::mutex> lock(final_mu);
                    if (slot.frame > retained_final_frame) {
                        direct.free_buffer(*final_gpr);
                        *final_gpr = slot.gpr;
                        slot.gpr = {nullptr, 0};
                        retained_final_frame = slot.frame;
                    }
                }
                direct.free_buffer(slot.gpr);
            }
            log_event("INFO", stage_name.c_str(), "stage finished");
        } catch (...) {
            const std::exception_ptr ep = std::current_exception();
            try {
                std::rethrow_exception(ep);
            } catch (const std::exception& e) {
                log_event("ERROR", stage_name.c_str(), e.what());
            } catch (...) {
                log_event("ERROR", stage_name.c_str(), "unknown non-standard exception");
            }
            set_error(ep);
        }
    };

    std::vector<std::thread> extra_workers;
    for (int worker = 1; worker < worker_count; ++worker)
        extra_workers.emplace_back(worker_body, worker);
    worker_body(0);
    for (auto& thread : extra_workers) thread.join();
    const auto end = Clock::now();

    for (size_t index = 0; index < vp.slots.size(); ++index) {
        const size_t owner = std::min(index, direct_encoders.size() - 1u);
        direct_encoders[owner]->free_buffer(vp.slots[index].gpr);
    }
    if (error) std::rethrow_exception(error);

    Result result;
    result.frames = next_frame;
    result.submission_seconds = std::chrono::duration<double>(submission_end - start).count();
    result.total_seconds = std::chrono::duration<double>(end - start).count();
    // No simulated producer queue exists in strict-workers mode. Each processing
    // worker owns exactly one local RAW copy and completes it end-to-end.
    result.capture_queue_capacity = 0;
    result.capture_queue_high_water = 0;
    result.capture_protection_high_water = 0;
    result.capture_frames_enqueued = uint64_t(result.frames);
    result.capture_frames_dequeued = uint64_t(result.frames);
    result.capture_backpressure_events = 0;
    result.capture_dynamic_allocated = 0;
    result.capture_pending_at_stop = 0;
    result.capture_drain_seconds = 0.0;
    result.capture_longest_wait_ms = 0.0;
    result.capture_max_encoder_deficit = 0;
    const ProcessMemoryInfo process_mem = read_process_memory_info();
    result.process_rss_bytes = process_mem.rss_bytes;
    result.process_peak_rss_bytes = process_mem.peak_rss_bytes;

    if (!collect || stats.empty()) return result;
    std::sort(stats.begin(), stats.end(), [](const FrameStat& a, const FrameStat& b) {
        return a.frame < b.frame;
    });
    result.first_frame_crc = stats.front().crc;
    result.first_frame_crc_computed = stats.front().crc_computed;
    if (stats.size() != size_t(result.frames)) {
        throw std::runtime_error("strict worker frame count mismatch: assigned=" +
                                 std::to_string(result.frames) + " completed=" +
                                 std::to_string(stats.size()));
    }

    const double frame_count = static_cast<double>(stats.size());
    std::vector<double> latencies;
    std::vector<Clock::time_point> finishes;
    std::vector<double> gaps;
    latencies.reserve(stats.size());
    finishes.reserve(stats.size());
    double sum_split = 0.0, sum_gpu = 0.0, sum_gpu_exec = 0.0;
    double sum_snapshot = 0.0, sum_cpu_tail = 0.0;
    uint64_t sum_snapshot_bytes = 0;
    double sum_int8_range = 0.0, sum_int8_pack = 0.0;
    double sum_entropy = 0.0, sum_writer = 0.0;
    uint64_t sum_hybrid_bytes = 0, sum_int8_bands = 0, sum_fallback_bands = 0;
    uint64_t sum_int8_tiles = 0, sum_fallback_tiles = 0, sum_total_tiles = 0;
    size_t gpu_exec_frames = 0;
    uint64_t payload_bytes = 0, packet_bytes = 0;
    for (const auto& stat : stats) {
        latencies.push_back(stat.latency);
        finishes.push_back(stat.finish);
        sum_split += stat.split;
        sum_gpu += stat.gpu;
        sum_snapshot += stat.snapshot;
        sum_snapshot_bytes += stat.snapshot_bytes;
        sum_cpu_tail += stat.cpu_tail;
        sum_int8_range += stat.int8_range;
        sum_int8_pack += stat.int8_pack;
        sum_hybrid_bytes += stat.hybrid_bytes;
        sum_int8_bands += stat.int8_bands;
        sum_fallback_bands += stat.int16_fallback_bands;
        sum_int8_tiles += stat.int8_tiles;
        sum_fallback_tiles += stat.int16_fallback_tiles;
        sum_total_tiles += stat.total_tiles;
        sum_entropy += stat.entropy;
        sum_writer += stat.writer;
        payload_bytes += stat.payload;
        packet_bytes += stat.packet;
        if (stat.gpu_exec >= 0.0) {
            sum_gpu_exec += stat.gpu_exec;
            ++gpu_exec_frames;
        }
    }
    std::sort(finishes.begin(), finishes.end());
    for (size_t i = 1; i < finishes.size(); ++i) {
        gaps.push_back(std::chrono::duration<double, std::milli>(
            finishes[i] - finishes[i - 1]).count());
    }
    auto percentile = [](std::vector<double> values, double p) {
        if (values.empty()) return 0.0;
        std::sort(values.begin(), values.end());
        const size_t index = std::min(
            values.size() - 1,
            size_t(std::ceil(p * static_cast<double>(values.size())) - 1.0));
        return values[index];
    };

    result.avg_latency = std::accumulate(latencies.begin(), latencies.end(), 0.0) / frame_count;
    result.p99_latency = percentile(latencies, 0.99);
    result.p99_gap = percentile(gaps, 0.99);
    result.avg_split = sum_split / frame_count;
    result.avg_gpu = sum_gpu / frame_count;
    result.avg_gpu_exec = gpu_exec_frames ? sum_gpu_exec / double(gpu_exec_frames) : -1.0;
    result.avg_snapshot = sum_snapshot / frame_count;
    result.snapshot_bytes = uint64_t(sum_snapshot_bytes / uint64_t(result.frames));
    result.cacheable_hybrid_handoff = vp.use_readback_copy || vp.use_vulkan_readback_copy ||
        o.coeff_storage == Options::CoeffStorage::CpuHybridBand ||
        o.coeff_storage == Options::CoeffStorage::CpuHybridTile ||
        o.coeff_storage == Options::CoeffStorage::GpuHybridOnePass;
    result.avg_cpu_tail = sum_cpu_tail / frame_count;
    result.avg_int8_range = sum_int8_range / frame_count;
    result.avg_int8_pack = sum_int8_pack / frame_count;
    result.avg_hybrid_bytes = uint64_t(sum_hybrid_bytes / uint64_t(result.frames));
    result.avg_int8_bands = uint32_t(sum_int8_bands / uint64_t(result.frames));
    result.avg_int16_fallback_bands = uint32_t(sum_fallback_bands / uint64_t(result.frames));
    result.avg_int8_tiles = uint32_t(sum_int8_tiles / uint64_t(result.frames));
    result.avg_int16_fallback_tiles = uint32_t(sum_fallback_tiles / uint64_t(result.frames));
    result.avg_total_tiles = uint32_t(sum_total_tiles / uint64_t(result.frames));
    result.avg_entropy = sum_entropy / frame_count;
    result.avg_writer = sum_writer / frame_count;
    result.payload = size_t(payload_bytes / uint64_t(result.frames));
    result.packet = size_t(packet_bytes / uint64_t(result.frames));
    const size_t input = size_t(o.width) * size_t(o.height) * 2u;
    result.payload_ratio = double(input) / double(std::max<size_t>(1u, result.payload));
    result.packet_ratio = double(input) / double(std::max<size_t>(1u, result.packet));
    if (result.frames > 1) {
        const double seconds = std::chrono::duration<double>(
            finishes.back() - finishes.front()).count();
        result.fps = seconds > 0.0 ? double(result.frames - 1) / seconds : 0.0;
    } else {
        result.fps = result.avg_latency > 0.0 ? 1000.0 / result.avg_latency : 0.0;
    }
    result.output_mibs = double(result.packet) * result.fps / 1048576.0;

    // Per-engine split. Both shares use the same measured window, so
    // gpu_fps + cpu_fps == fps exactly. This is a decomposition of one measured
    // aggregate, NOT two separately measured rates that have been added: the
    // engines were contending for memory bandwidth the whole time, and that
    // contention is already inside these numbers.
    result.dual_cpu_workers = o.cpu_wavelet_workers;
    if (!stats.empty()) {
        double gpu_latency = 0.0, cpu_latency = 0.0;
        for (const auto& s : stats) {
            if (s.engine) { ++result.cpu_frames; cpu_latency += s.latency; }
            else          { ++result.gpu_frames; gpu_latency += s.latency; }
        }
        const double counted = double(result.gpu_frames + result.cpu_frames);
        if (counted > 0.0) {
            result.gpu_fps = result.fps * double(result.gpu_frames) / counted;
            result.cpu_fps = result.fps * double(result.cpu_frames) / counted;
        }
        if (result.gpu_frames) result.gpu_avg_latency = gpu_latency / double(result.gpu_frames);
        if (result.cpu_frames) result.cpu_avg_latency = cpu_latency / double(result.cpu_frames);
    }
    return result;
}



static bool use_cacheable_hybrid_snapshot(const VulkanPipeline& vp,
                                          const TileDirectHybridProvider* tile_provider,
                                          const Options& o) {
    if (tile_provider == nullptr || o.execution != "pipeline") return false;
    if (o.hybrid_handoff == Options::HybridHandoff::CacheableSnapshot) return true;
    if (o.hybrid_handoff == Options::HybridHandoff::DirectMapped) return false;
    return !vp.mapped_host_cached;
}

static Result run_pass_cacheable_snapshot(
                       VulkanPipeline& vp,
                       TileDirectHybridProvider& tile_provider,
                       std::vector<std::unique_ptr<DirectGprEncoder>>& direct_encoders,
                       const ModeSpec* run_mode, const Options& o,
                       const std::vector<uint16_t>& src,
                       const std::vector<uint16_t>& lut,
                       int fixed_frames, double duration, bool collect,
                       std::ostream* csv, gpr_buffer* final_gpr) {
    constexpr int END = -1;
    constexpr int NORMAL_CAPTURE_DEPTH = 12;
    const int entropy_worker_count = int(direct_encoders.size());
    if (entropy_worker_count < 1) throw std::runtime_error("no direct GPR entropy encoders");
    (void)run_mode;

    const int snapshot_job_count = o.snapshot_jobs > 0
        ? o.snapshot_jobs : std::min(8, std::max(2, o.buffers + 1));
    if (snapshot_job_count < entropy_worker_count)
        throw std::runtime_error("cacheable hybrid snapshot requires snapshot jobs >= entropy workers");

    struct SnapshotJob {
        TileDirectSnapshot snapshot;
        int frame = -1;
        int source_slot = -1;
        int entropy_worker = -1;
        Clock::time_point start{}, entropy_queue_enter{}, entropy_finish{}, finish{};
        int64_t split_ns = 0;
        int64_t gpu_wall_ns = 0;
        int64_t gpu_exec_ns = -1;
        int64_t snapshot_ns = 0;
        int64_t gpu_slot_hold_ns = 0;
        int64_t entropy_ns = 0;
        int64_t entropy_queue_wait_ns = 0;
        int64_t writer_ns = 0;
        uint64_t snapshot_bytes = 0;
        uint64_t hybrid_bytes = 0;
        uint32_t int8_tiles = 0;
        uint32_t int16_fallback_tiles = 0;
        uint32_t total_tiles = 0;
        uint32_t crc = 0;
        bool crc_computed = false;
        gpr_buffer gpr{nullptr, 0};
        size_t vc5_bytes = 0;
    };

    const int gpu_inflight_limit = effective_gpu_inflight_limit(o, entropy_worker_count);
    Queue<int> freeq(size_t(o.buffers));
    Queue<int> gpu_tokens{size_t(gpu_inflight_limit)};
    Queue<int> gpuq(size_t(o.buffers) + 1u);
    Queue<int> job_freeq{size_t(snapshot_job_count)};
    Queue<int> entq(size_t(snapshot_job_count) + 1u);
    Queue<int> writeq(size_t(snapshot_job_count) + 1u);
    for (int i = 0; i < o.buffers; ++i)
        if (!freeq.push(i)) throw std::runtime_error("failed to initialise Vulkan slot queue");
    for (int i = 0; i < gpu_inflight_limit; ++i)
        if (!gpu_tokens.push(i)) throw std::runtime_error("failed to initialise GPU in-flight limiter");

    std::vector<SnapshotJob> jobs{size_t(snapshot_job_count)};
    for (int i = 0; i < snapshot_job_count; ++i) {
        tile_provider.allocate_snapshot(jobs[size_t(i)].snapshot);
        if (!job_freeq.push(i)) throw std::runtime_error("failed to initialise snapshot job queue");
    }

    std::cout << "HYBRID_HANDOFF policy=cacheable-snapshot"
              << " snapshot_jobs=" << snapshot_job_count
              << " snapshot_allocated_bytes=" << jobs.front().snapshot.allocated_bytes
              << " compact_fallback_copy=YES"
              << " release_slot_after_snapshot=YES"
              << " gpu_inflight_limit=" << gpu_inflight_limit
              << " entropy_workers=" << entropy_worker_count << "\n";
    log_event("INFO", "handoff",
              "cacheable tile-direct snapshot enabled; Vulkan slot is released before entropy");

    std::vector<FrameStat> stats;
    if (collect && duration <= 0.0) stats.reserve(size_t(fixed_frames));
    std::atomic<int> produced{0};
    std::array<std::atomic<uint64_t>,4> worker_frames{};
    std::array<std::atomic<int64_t>,4> worker_entropy_ns{};
    std::array<std::atomic<int64_t>,4> worker_queue_wait_ns{};
    for (auto& v : worker_frames) v.store(0);
    for (auto& v : worker_entropy_ns) v.store(0);
    for (auto& v : worker_queue_wait_ns) v.store(0);
    std::atomic<size_t> reorder_high_water{0};
    std::atomic<uint64_t> out_of_order_entropy_finishes{0};
    std::exception_ptr error;
    std::mutex error_mu;
    std::mutex dump_mu;
    bool coefficient_dumped = false;
    auto cancel = [&] {
        freeq.close();
        gpu_tokens.close();
        gpuq.close();
        job_freeq.close();
        entq.close();
        writeq.close();
    };
    auto guard = [&](const char* stage, auto&& fn) {
        try {
            log_event("INFO", stage, "stage started");
            fn();
            log_event("INFO", stage, "stage finished");
        } catch (const std::exception& e) {
            log_event("ERROR", stage, e.what());
            {
                std::lock_guard<std::mutex> lock(error_mu);
                if (!error) error = std::current_exception();
            }
            cancel();
        } catch (...) {
            log_event("ERROR", stage, "unknown non-standard exception");
            {
                std::lock_guard<std::mutex> lock(error_mu);
                if (!error) error = std::current_exception();
            }
            cancel();
        }
    };

    const auto start = Clock::now();
    const auto deadline = start + std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(duration));
    Clock::time_point submission_end = start;

    const int capture_total_capacity = std::max(NORMAL_CAPTURE_DEPTH, o.capture_queue);
    std::vector<std::unique_ptr<std::vector<uint16_t>>> capture_buffers{
        size_t(capture_total_capacity)};
    std::vector<Clock::time_point> capture_enqueue_time{size_t(capture_total_capacity)};
    auto capture_free = std::make_unique<Queue<int>>(size_t(capture_total_capacity));
    auto capture_ready = std::make_unique<Queue<int>>(size_t(capture_total_capacity) + 1u);
    for (int i = 0; i < NORMAL_CAPTURE_DEPTH; ++i) {
        capture_buffers[size_t(i)] = std::make_unique<std::vector<uint16_t>>(src.size());
        capture_free->push(i);
    }
    std::mutex capture_alloc_mu;
    int next_dynamic_slot = NORMAL_CAPTURE_DEPTH;
    std::atomic<uint64_t> capture_frames_enqueued{0};
    std::atomic<uint64_t> capture_frames_dequeued{0};
    std::atomic<uint64_t> capture_backpressure_events{0};
    std::atomic<uint64_t> completed_frames{0};
    std::atomic<size_t> capture_dynamic_allocated{0};
    std::atomic<size_t> capture_pending_at_stop{0};
    std::atomic<size_t> capture_max_encoder_deficit{0};
    std::atomic<int64_t> capture_longest_wait_ns{0};
    Clock::time_point capture_stop_time = start;
    size_t capture_data_high_water = 0;

    std::thread capture_thread([&] { guard("capture-queue", [&] {
        int frame = 0;
        for (;;) {
            if (duration <= 0.0) {
                if (frame >= fixed_frames) break;
            } else if (frame > 0 && Clock::now() >= deadline) {
                break;
            }
            if (o.capture_fps > 0.0) {
                const auto release_time = start + std::chrono::duration_cast<Clock::duration>(
                    std::chrono::duration<double>(double(frame) / o.capture_fps));
                std::this_thread::sleep_until(release_time);
                if (duration > 0.0 && frame > 0 && Clock::now() >= deadline) break;
            }

            int ci = -1;
            if (!capture_free->try_pop(ci)) {
                std::lock_guard<std::mutex> lock(capture_alloc_mu);
                if (next_dynamic_slot < capture_total_capacity) {
                    ci = next_dynamic_slot++;
                    capture_buffers[size_t(ci)] =
                        std::make_unique<std::vector<uint16_t>>(src.size());
                    capture_dynamic_allocated.fetch_add(1, std::memory_order_relaxed);
                }
            }
            if (ci < 0) {
                capture_backpressure_events.fetch_add(1, std::memory_order_relaxed);
                if (!capture_free->pop(ci)) return;
            }

            std::memcpy(capture_buffers[size_t(ci)]->data(), src.data(),
                        src.size() * sizeof(uint16_t));
            capture_enqueue_time[size_t(ci)] = Clock::now();
            const uint64_t enqueued_now =
                capture_frames_enqueued.fetch_add(1, std::memory_order_release) + 1u;
            if (!capture_ready->push(ci)) {
                capture_frames_enqueued.fetch_sub(1, std::memory_order_relaxed);
                return;
            }
            const uint64_t completed_now = completed_frames.load(std::memory_order_acquire);
            const size_t deficit_now = nonnegative_frame_difference(enqueued_now, completed_now);
            size_t prior = capture_max_encoder_deficit.load(std::memory_order_relaxed);
            while (deficit_now > prior && !capture_max_encoder_deficit.compare_exchange_weak(
                       prior, deficit_now, std::memory_order_relaxed)) {}
            ++frame;
        }
        capture_stop_time = Clock::now();
        const size_t pending = nonnegative_frame_difference(
            capture_frames_enqueued.load(std::memory_order_acquire),
            completed_frames.load(std::memory_order_acquire));
        capture_pending_at_stop.store(pending, std::memory_order_relaxed);
        capture_data_high_water = capture_ready->high_water();
        (void)capture_ready->push(END);
    }); });

    std::thread producer([&] { guard("split", [&] {
        int frame = 0;
        for (;;) {
            int capture_index = -1;
            if (!capture_ready->pop(capture_index)) return;
            if (capture_index == END) break;
            capture_frames_dequeued.fetch_add(1, std::memory_order_relaxed);
            const auto queue_wait_ns = std::chrono::duration_cast<Ns>(
                Clock::now() - capture_enqueue_time[size_t(capture_index)]).count();
            int64_t longest = capture_longest_wait_ns.load(std::memory_order_relaxed);
            while (queue_wait_ns > longest && !capture_longest_wait_ns.compare_exchange_weak(
                       longest, queue_wait_ns, std::memory_order_relaxed)) {}

            int gpu_token = -1;
            if (!gpu_tokens.pop(gpu_token)) return;
            int slot_index = -1;
            if (!freeq.pop(slot_index)) {
                (void)gpu_tokens.push(gpu_token);
                return;
            }
            auto& slot = vp.slots[size_t(slot_index)];
            slot.frame = frame;
            slot.start = Clock::now();
            slot.vc5_bytes = 0;
            slot.crc = 0;
            slot.crc_computed = false;
            slot.entropy_worker = -1;
            slot.int8_range_ns = 0;
            slot.int8_pack_ns = 0;
            slot.hybrid_bytes = 0;
            slot.int8_bands = 0;
            slot.int16_fallback_bands = 0;
            slot.int8_tiles = 0;
            slot.int16_fallback_tiles = 0;
            slot.total_tiles = 0;
            const auto split_start = Clock::now();
            split_compand(o, capture_buffers[size_t(capture_index)]->data(), lut,
                          slot.mapped_ping, vp.plane_stride_elems);
            slot.split_ns = elapsed_ns(split_start);
            if (!capture_free->push(capture_index)) return;
            vp.submit(slot);
            ++frame;
            produced.store(frame, std::memory_order_relaxed);
            if (!gpuq.push(slot_index)) return;
        }
        submission_end = Clock::now();
        (void)gpuq.push(END);
    }); });

    std::thread completion([&] { guard("gpu-snapshot", [&] {
        for (;;) {
            int slot_index = -1;
            if (!gpuq.pop(slot_index)) return;
            if (slot_index == END) break;
            auto& slot = vp.slots[size_t(slot_index)];
            vp.wait(slot);
            if (!gpu_tokens.push(0)) return;

            int job_index = -1;
            if (!job_freeq.pop(job_index)) return;
            auto& job = jobs[size_t(job_index)];
            job.frame = slot.frame;
            job.source_slot = slot_index;
            job.entropy_worker = -1;
            job.start = slot.start;
            job.split_ns = slot.split_ns;
            job.gpu_wall_ns = slot.gpu_wall_ns;
            job.gpu_exec_ns = slot.gpu_exec_ns;
            job.vc5_bytes = 0;
            job.crc = 0;
            job.crc_computed = false;
            const auto snapshot_start = Clock::now();
            tile_provider.snapshot(size_t(slot_index), job.snapshot);
            job.snapshot_bytes = job.snapshot.copied_bytes;
            job.snapshot_ns = elapsed_ns(snapshot_start);
            job.gpu_slot_hold_ns = std::chrono::duration_cast<Ns>(
                Clock::now() - slot.submit).count();
            const auto& metrics = tile_provider.metrics(job.snapshot);
            job.hybrid_bytes = metrics.physical_bytes;
            job.int8_tiles = metrics.int8_tiles;
            job.int16_fallback_tiles = metrics.fallback_tiles;
            job.total_tiles = metrics.total_tiles;

            // The GPU slot no longer owns any data needed by entropy or the writer.
            // Return it before publishing the cacheable job downstream.
            if (!freeq.push(slot_index)) return;
            job.entropy_queue_enter = Clock::now();
            if (!entq.push(job_index)) return;
        }
        for (int worker = 0; worker < entropy_worker_count; ++worker)
            if (!entq.push(END)) return;
    }); });

    std::vector<std::thread> entropy_workers;
    entropy_workers.reserve(size_t(entropy_worker_count));
    for (int worker = 0; worker < entropy_worker_count; ++worker) {
        entropy_workers.emplace_back([&, worker] {
            const std::string stage_name = "entropy-" + std::to_string(worker);
            guard(stage_name.c_str(), [&] {
                DirectGprEncoder& direct = *direct_encoders[size_t(worker)];
                for (;;) {
                    int job_index = -1;
                    if (!entq.pop(job_index)) return;
                    if (job_index == END) break;
                    auto& job = jobs[size_t(job_index)];
                    const auto entropy_start = Clock::now();
                    job.entropy_worker = worker;
                    job.entropy_queue_wait_ns = std::chrono::duration_cast<Ns>(
                        entropy_start - job.entropy_queue_enter).count();

                    const auto& hybrid_frame = tile_provider.frame(job.snapshot);
                    std::vector<int16_t> reconstructed;
                    const int16_t* baseline_coefficients = nullptr;
                    if ((o.hybrid_verify_baseline && job.frame == 0) ||
                        (collect && !o.dump_coeff.empty())) {
                        reconstructed = tile_provider.reconstruct_full_int16(job.snapshot);
                        baseline_coefficients = reconstructed.data();
                    }

                    if (collect && !o.dump_coeff.empty()) {
                        std::lock_guard<std::mutex> lock(dump_mu);
                        if (!coefficient_dumped) {
                            if (!baseline_coefficients) {
                                reconstructed = tile_provider.reconstruct_full_int16(job.snapshot);
                                baseline_coefficients = reconstructed.data();
                            }
                            const std::filesystem::path dump_path(o.dump_coeff);
                            if (dump_path.has_parent_path())
                                std::filesystem::create_directories(dump_path.parent_path());
                            std::ofstream dump(dump_path, std::ios::binary | std::ios::trunc);
                            if (!dump) throw std::runtime_error(
                                "cannot create coefficient dump: " + dump_path.string());
                            const size_t total = vp.plane_stride_elems * 4u;
                            dump.write(reinterpret_cast<const char*>(baseline_coefficients),
                                       std::streamsize(total * sizeof(int16_t)));
                            if (!dump) throw std::runtime_error(
                                "coefficient dump write failed: " + dump_path.string());
                            coefficient_dumped = true;
                        }
                    }

                    if (o.hybrid_verify_baseline && job.frame == 0) {
                        if (!baseline_coefficients) {
                            reconstructed = tile_provider.reconstruct_full_int16(job.snapshot);
                            baseline_coefficients = reconstructed.data();
                        }
                        gpr_buffer baseline{nullptr, 0};
                        size_t baseline_vc5_bytes = 0;
                        direct.encode(baseline_coefficients, baseline, baseline_vc5_bytes);
                        direct.encode_hybrid(hybrid_frame, job.gpr, job.vc5_bytes);
                        const bool exact = baseline.size == job.gpr.size &&
                            baseline_vc5_bytes == job.vc5_bytes &&
                            std::memcmp(baseline.buffer, job.gpr.buffer, baseline.size) == 0;
                        direct.free_buffer(baseline);
                        if (!exact)
                            throw std::runtime_error("snapshot hybrid GPR differs from int16 baseline");
                        std::lock_guard<std::mutex> output_lock(g_log_mu);
                        std::cout << "HYBRID_GPR_VERIFY exact_gpr=YES frame=0 mode="
                                  << (o.mode.empty() ? std::string("base-q") +
                                      std::to_string(o.quant) : o.mode)
                                  << " storage=" << coeff_storage_name(o)
                                  << " handoff=cacheable-snapshot\n";
                    } else {
                        direct.encode_hybrid(hybrid_frame, job.gpr, job.vc5_bytes);
                    }
                    job.entropy_ns = elapsed_ns(entropy_start);
                    job.entropy_finish = Clock::now();
                    worker_frames[size_t(worker)].fetch_add(1, std::memory_order_relaxed);
                    worker_entropy_ns[size_t(worker)].fetch_add(
                        job.entropy_ns, std::memory_order_relaxed);
                    worker_queue_wait_ns[size_t(worker)].fetch_add(
                        job.entropy_queue_wait_ns, std::memory_order_relaxed);
                    if (!writeq.push(job_index)) return;
                }
                (void)writeq.push(END);
            });
        });
    }

    std::thread writer([&] { guard("writer", [&] {
        int completed = 0;
        int ended_workers = 0;
        int next_expected_frame = 0;
        int retained_final_frame = -1;
        int retained_final_worker = -1;
        std::map<int,int> reorder;

        auto commit_job = [&](int job_index) {
            auto& job = jobs[size_t(job_index)];
            if (job.frame != next_expected_frame)
                throw std::runtime_error("ordered writer snapshot sequence mismatch");
            const auto writer_start = Clock::now();
            job.crc_computed = should_compute_frame_crc(o.no_crc, collect, job.frame);
            job.crc = job.crc_computed ? crc32_bytes(job.gpr.buffer, job.gpr.size) : 0u;
            const size_t packet = job.gpr.size;
            job.writer_ns = elapsed_ns(writer_start);
            job.finish = Clock::now();

            ++completed;
            ++next_expected_frame;
            completed_frames.store(uint64_t(completed), std::memory_order_release);
            if (collect) {
                FrameStat stat;
                stat.frame = job.frame;
                stat.latency = to_ms(std::chrono::duration_cast<Ns>(
                    job.finish - job.start).count());
                stat.split = to_ms(job.split_ns);
                stat.gpu = to_ms(job.gpu_wall_ns);
                stat.gpu_exec = job.gpu_exec_ns < 0 ? -1.0 : to_ms(job.gpu_exec_ns);
                stat.snapshot = to_ms(job.snapshot_ns);
                stat.snapshot_bytes = job.snapshot_bytes;
                stat.gpu_slot_hold = to_ms(job.gpu_slot_hold_ns);
                stat.entropy = to_ms(job.entropy_ns);
                stat.entropy_queue_wait = to_ms(job.entropy_queue_wait_ns);
                stat.hybrid_bytes = job.hybrid_bytes;
                stat.int8_tiles = job.int8_tiles;
                stat.int16_fallback_tiles = job.int16_fallback_tiles;
                stat.total_tiles = job.total_tiles;
                stat.writer = to_ms(job.writer_ns);
                stat.entropy_worker = job.entropy_worker;
                stat.payload = job.vc5_bytes;
                stat.packet = packet;
                stat.crc = job.crc;
                stat.crc_computed = job.crc_computed;
                stat.finish = job.finish;
                stats.push_back(stat);

                if (csv) {
                    (*csv) << job.frame << ',' << job.source_slot << ','
                           << job.entropy_worker << ',' << std::fixed << std::setprecision(3)
                           << stat.latency << ',' << stat.split << ',' << stat.gpu << ','
                           << stat.gpu_exec << ',' << stat.snapshot << ',' << stat.snapshot_bytes << ','
                           << stat.gpu_slot_hold << ',' << stat.entropy_queue_wait << ','
                           << 0.0 << ',' << 0.0 << ',' << stat.hybrid_bytes << ','
                           << 0 << ',' << 0 << ',' << stat.int8_tiles << ','
                           << stat.int16_fallback_tiles << ',' << stat.total_tiles << ','
                           << stat.entropy << ',' << stat.writer << ',' << stat.payload << ','
                           << stat.packet << ',' << stat.crc << ','
                           << (stat.crc_computed ? "YES" : "NO") << '\n';
                    csv->flush();
                    if (!*csv) throw std::runtime_error("per-frame CSV write failed");
                }
            }

            const size_t pending_now = nonnegative_frame_difference(
                capture_frames_enqueued.load(std::memory_order_acquire),
                completed_frames.load(std::memory_order_acquire));
            if (collect && capture_pending_at_stop.load(std::memory_order_relaxed) > 0 &&
                (pending_now == 0 || pending_now % 30 == 0)) {
                log_event(pending_now == 0 ? "INFO" : "WARN", "drain",
                          pending_now == 0
                              ? "BUFFER DRAIN COMPLETE: all captured frames encoded."
                              : "post-record encode drain: " + std::to_string(pending_now) +
                                " frames remaining");
            }

            if (final_gpr && collect && job.frame > retained_final_frame) {
                if (final_gpr->buffer) {
                    const int owner = retained_final_worker >= 0 ? retained_final_worker : 0;
                    direct_encoders[size_t(owner)]->free_buffer(*final_gpr);
                }
                *final_gpr = job.gpr;
                job.gpr = {nullptr, 0};
                retained_final_frame = job.frame;
                retained_final_worker = job.entropy_worker;
            } else {
                const int owner = job.entropy_worker >= 0 ? job.entropy_worker : 0;
                direct_encoders[size_t(owner)]->free_buffer(job.gpr);
            }
            job.entropy_worker = -1;
            if (!job_freeq.push(job_index))
                throw std::runtime_error("snapshot job queue closed during ordered commit");
        };

        for (;;) {
            int job_index = -1;
            if (!writeq.pop(job_index)) return;
            if (job_index == END) {
                if (++ended_workers != entropy_worker_count) continue;
                while (true) {
                    auto it = reorder.find(next_expected_frame);
                    if (it == reorder.end()) break;
                    const int ready = it->second;
                    reorder.erase(it);
                    commit_job(ready);
                }
                if (!reorder.empty())
                    throw std::runtime_error("snapshot entropy workers ended with reorder gap");
                break;
            }
            auto& job = jobs[size_t(job_index)];
            if (job.frame != next_expected_frame)
                out_of_order_entropy_finishes.fetch_add(1, std::memory_order_relaxed);
            if (!reorder.emplace(job.frame, job_index).second)
                throw std::runtime_error("duplicate snapshot frame in reorder buffer");
            const size_t held_out_of_order = reorder.size() -
                (reorder.count(next_expected_frame) ? 1u : 0u);
            size_t previous = reorder_high_water.load(std::memory_order_relaxed);
            while (held_out_of_order > previous && !reorder_high_water.compare_exchange_weak(
                       previous, held_out_of_order, std::memory_order_relaxed)) {}
            while (true) {
                auto it = reorder.find(next_expected_frame);
                if (it == reorder.end()) break;
                const int ready = it->second;
                reorder.erase(it);
                commit_job(ready);
            }
        }
    }); });

    capture_thread.join();
    producer.join();
    completion.join();
    for (auto& thread : entropy_workers) thread.join();
    writer.join();
    const auto end = Clock::now();
    cancel();
    for (auto& job : jobs) {
        const int owner = job.entropy_worker >= 0 ? job.entropy_worker : 0;
        direct_encoders[size_t(owner)]->free_buffer(job.gpr);
    }
    if (error) std::rethrow_exception(error);
    if (collect)
        std::sort(stats.begin(), stats.end(), [](const FrameStat& a, const FrameStat& b) {
            return a.frame < b.frame;
        });

    Result result;
    result.frames = produced.load(std::memory_order_relaxed);
    result.submission_seconds = std::chrono::duration<double>(submission_end - start).count();
    result.total_seconds = std::chrono::duration<double>(end - start).count();
    result.capture_queue_capacity = size_t(capture_total_capacity);
    result.capture_queue_high_water = capture_data_high_water;
    result.capture_protection_high_water = result.capture_queue_high_water > size_t(NORMAL_CAPTURE_DEPTH)
        ? result.capture_queue_high_water - size_t(NORMAL_CAPTURE_DEPTH) : 0u;
    result.capture_frames_enqueued = capture_frames_enqueued.load(std::memory_order_relaxed);
    result.capture_frames_dequeued = capture_frames_dequeued.load(std::memory_order_relaxed);
    result.capture_backpressure_events = capture_backpressure_events.load(std::memory_order_relaxed);
    result.capture_dynamic_allocated = capture_dynamic_allocated.load(std::memory_order_relaxed);
    result.capture_pending_at_stop = capture_pending_at_stop.load(std::memory_order_relaxed);
    result.capture_drain_seconds = result.capture_pending_at_stop > 0
        ? std::chrono::duration<double>(end - capture_stop_time).count() : 0.0;
    result.capture_longest_wait_ms = to_ms(capture_longest_wait_ns.load(std::memory_order_relaxed));
    result.capture_max_encoder_deficit = capture_max_encoder_deficit.load(std::memory_order_relaxed);
    const ProcessMemoryInfo process_mem = read_process_memory_info();
    result.process_rss_bytes = process_mem.rss_bytes;
    result.process_peak_rss_bytes = process_mem.peak_rss_bytes;
    result.reorder_high_water = reorder_high_water.load(std::memory_order_relaxed);
    result.out_of_order_entropy_finishes = out_of_order_entropy_finishes.load(std::memory_order_relaxed);
    result.cacheable_hybrid_handoff = true;
    result.snapshot_bytes = 0;
    result.worker_frames.resize(size_t(entropy_worker_count));
    result.worker_avg_entropy_ms.resize(size_t(entropy_worker_count));
    result.worker_avg_queue_wait_ms.resize(size_t(entropy_worker_count));
    result.worker_utilisation_pct.resize(size_t(entropy_worker_count));
    for (int worker = 0; worker < entropy_worker_count; ++worker) {
        const uint64_t frames = worker_frames[size_t(worker)].load(std::memory_order_relaxed);
        const int64_t busy = worker_entropy_ns[size_t(worker)].load(std::memory_order_relaxed);
        const int64_t wait = worker_queue_wait_ns[size_t(worker)].load(std::memory_order_relaxed);
        result.worker_frames[size_t(worker)] = frames;
        result.worker_avg_entropy_ms[size_t(worker)] = frames ? to_ms(busy) / double(frames) : 0.0;
        result.worker_avg_queue_wait_ms[size_t(worker)] = frames ? to_ms(wait) / double(frames) : 0.0;
        result.worker_utilisation_pct[size_t(worker)] = result.total_seconds > 0.0
            ? 100.0 * (double(busy) / 1.0e9) / result.total_seconds : 0.0;
    }
    if (!collect || stats.empty()) return result;
    if (stats.size() != size_t(result.frames))
        throw std::runtime_error("snapshot frame count mismatch: produced=" +
                                 std::to_string(result.frames) + " completed=" +
                                 std::to_string(stats.size()));

    result.first_frame_crc = stats.front().crc;
    result.first_frame_crc_computed = stats.front().crc_computed;
    const double frame_count = double(stats.size());
    std::vector<double> latencies;
    latencies.reserve(stats.size());
    const CompletionTiming completion_metrics = completion_timing(stats);
    double sum_split = 0.0, sum_gpu = 0.0, sum_gpu_exec = 0.0;
    double sum_snapshot = 0.0, sum_gpu_slot_hold = 0.0;
    double sum_entropy = 0.0, sum_writer = 0.0;
    uint64_t sum_snapshot_bytes = 0;
    uint64_t sum_hybrid_bytes = 0, sum_int8_tiles = 0;
    uint64_t sum_fallback_tiles = 0, sum_total_tiles = 0;
    size_t gpu_exec_frames = 0;
    uint64_t payload_bytes = 0, packet_bytes = 0;
    for (const auto& stat : stats) {
        latencies.push_back(stat.latency);
        sum_split += stat.split;
        sum_gpu += stat.gpu;
        sum_snapshot += stat.snapshot;
        sum_snapshot_bytes += stat.snapshot_bytes;
        sum_gpu_slot_hold += stat.gpu_slot_hold;
        sum_hybrid_bytes += stat.hybrid_bytes;
        sum_int8_tiles += stat.int8_tiles;
        sum_fallback_tiles += stat.int16_fallback_tiles;
        sum_total_tiles += stat.total_tiles;
        sum_entropy += stat.entropy;
        sum_writer += stat.writer;
        payload_bytes += stat.payload;
        packet_bytes += stat.packet;
        if (stat.gpu_exec >= 0.0) {
            sum_gpu_exec += stat.gpu_exec;
            ++gpu_exec_frames;
        }
    }
    auto percentile = [](std::vector<double> values, double p) {
        if (values.empty()) return 0.0;
        std::sort(values.begin(), values.end());
        const size_t index = std::min(values.size() - 1,
            size_t(std::ceil(p * double(values.size())) - 1.0));
        return values[index];
    };
    result.avg_latency = std::accumulate(latencies.begin(), latencies.end(), 0.0) /
                         frame_count;
    result.p99_latency = percentile(latencies, 0.99);
    result.p99_gap = percentile(completion_metrics.gaps_ms, 0.99);
    result.avg_split = sum_split / frame_count;
    result.avg_gpu = sum_gpu / frame_count;
    result.avg_gpu_exec = gpu_exec_frames ? sum_gpu_exec / double(gpu_exec_frames) : -1.0;
    result.avg_snapshot = sum_snapshot / frame_count;
    result.snapshot_bytes = sum_snapshot_bytes / uint64_t(result.frames);
    result.avg_gpu_slot_hold = sum_gpu_slot_hold / frame_count;
    result.avg_hybrid_bytes = uint64_t(sum_hybrid_bytes / uint64_t(result.frames));
    result.avg_int8_tiles = uint32_t(sum_int8_tiles / uint64_t(result.frames));
    result.avg_int16_fallback_tiles = uint32_t(sum_fallback_tiles / uint64_t(result.frames));
    result.avg_total_tiles = uint32_t(sum_total_tiles / uint64_t(result.frames));
    result.avg_entropy = sum_entropy / frame_count;
    result.avg_writer = sum_writer / frame_count;
    result.payload = size_t(payload_bytes / uint64_t(result.frames));
    result.packet = size_t(packet_bytes / uint64_t(result.frames));
    const size_t input = size_t(o.width) * size_t(o.height) * 2u;
    result.payload_ratio = double(input) / double(std::max<size_t>(1u, result.payload));
    result.packet_ratio = double(input) / double(std::max<size_t>(1u, result.packet));
    if (result.frames > 1) {
        result.fps = completion_metrics.span_seconds > 0.0
            ? double(result.frames - 1) / completion_metrics.span_seconds : 0.0;
    } else {
        result.fps = result.avg_latency > 0.0 ? 1000.0 / result.avg_latency : 0.0;
    }
    result.output_mibs = double(result.packet) * result.fps / 1048576.0;
    return result;
}


static Result run_pass(VulkanPipeline& vp,
                       GpuHybridPacker* hybrid_packer,
                       GpuOnePassHybridPacker* onepass_packer,
                       CpuBandHybridProvider* cpu_hybrid_provider,
                       CpuTileHybridProvider* cpu_tile_provider,
                       FusedMirrorHybridProvider* fused_provider,
                       TileDirectHybridProvider* tile_provider,
                       std::vector<std::unique_ptr<DirectGprEncoder>>& direct_encoders,
                       const ModeSpec* run_mode, const Options& o,
                       const std::vector<uint16_t>& src,
                       const std::vector<uint16_t>& lut,
                       int fixed_frames, double duration, bool collect,
                       std::ostream* csv, gpr_buffer* final_gpr) {
    if (use_cacheable_hybrid_snapshot(vp, tile_provider, o)) {
        return run_pass_cacheable_snapshot(vp, *tile_provider, direct_encoders,
                                           run_mode, o, src, lut, fixed_frames,
                                           duration, collect, csv, final_gpr);
    }
    constexpr int END = -1;
    const int entropy_worker_count = int(direct_encoders.size());
    if (entropy_worker_count < 1) throw std::runtime_error("no direct GPR entropy encoders");
    const int gpu_inflight_limit = effective_gpu_inflight_limit(o, entropy_worker_count);
    Queue<int> freeq(size_t(o.buffers));
    Queue<int> gpu_tokens{size_t(gpu_inflight_limit)};
    Queue<int> gpuq(size_t(o.buffers) + 1u);
    Queue<int> entq(size_t(o.buffers) + 1u);
    Queue<int> writeq(size_t(o.buffers) + 1u);
    for (int i = 0; i < o.buffers; ++i) {
        if (!freeq.push(i)) throw std::runtime_error("failed to initialise slot queue");
    }
    for (int i = 0; i < gpu_inflight_limit; ++i) {
        if (!gpu_tokens.push(i)) throw std::runtime_error("failed to initialise GPU in-flight limiter");
    }
    (void)run_mode;
    std::vector<FrameStat> stats;
    if (collect && duration <= 0.0) stats.reserve(size_t(fixed_frames));
    std::atomic<int> produced{0};
    std::array<std::atomic<uint64_t>,4> worker_frames{};
    std::array<std::atomic<int64_t>,4> worker_entropy_ns{};
    std::array<std::atomic<int64_t>,4> worker_queue_wait_ns{};
    for(auto&v:worker_frames)v.store(0);
    for(auto&v:worker_entropy_ns)v.store(0);
    for(auto&v:worker_queue_wait_ns)v.store(0);
    std::atomic<size_t> reorder_high_water{0};
    std::atomic<uint64_t> out_of_order_entropy_finishes{0};
    std::exception_ptr error;
    std::mutex error_mu;
    std::mutex gpu_submit_mu;
    std::mutex dump_mu;
    bool coefficient_dumped = false;
    auto cancel = [&] {
        freeq.close();
        gpu_tokens.close();
        gpuq.close();
        entq.close();
        writeq.close();
    };
    auto guard = [&](const char* stage, auto&& fn) {
        try {
            log_event("INFO", stage, "stage started");
            fn();
            log_event("INFO", stage, "stage finished");
        } catch (const std::exception& e) {
            log_event("ERROR", stage, e.what());
            {
                std::lock_guard<std::mutex> lock(error_mu);
                if (!error) error = std::current_exception();
            }
            cancel();
        } catch (...) {
            log_event("ERROR", stage, "unknown non-standard exception");
            {
                std::lock_guard<std::mutex> lock(error_mu);
                if (!error) error = std::current_exception();
            }
            cancel();
        }
    };

    const auto start = Clock::now();
    const auto deadline = start + std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(duration));
    Clock::time_point submission_end = start;

    // Two-tier IMX585 Clear HDR RAW16 capture model. Twelve sensor/DMA frames form
    // the normal low-latency processing pipeline. The Capture Protection Buffer
    // allocates additional untouched RAW16 frame slots only when capture is
    // outrunning encode. GP-LOG2 is applied only after a frame leaves this queue.
    constexpr int NORMAL_CAPTURE_DEPTH = 12;
    const int capture_total_capacity = std::max(NORMAL_CAPTURE_DEPTH, o.capture_queue);
    std::vector<std::unique_ptr<std::vector<uint16_t>>> capture_buffers{size_t(capture_total_capacity)};
    std::vector<Clock::time_point> capture_enqueue_time{size_t(capture_total_capacity)};
    auto capture_free = std::make_unique<Queue<int>>(size_t(capture_total_capacity));
    auto capture_ready = std::make_unique<Queue<int>>(size_t(capture_total_capacity) + 1u);
    for (int i = 0; i < NORMAL_CAPTURE_DEPTH; ++i) {
        capture_buffers[size_t(i)] = std::make_unique<std::vector<uint16_t>>(src.size());
        capture_free->push(i);
    }
    std::mutex capture_alloc_mu;
    int next_dynamic_slot = NORMAL_CAPTURE_DEPTH;
    std::thread capture_thread;
    std::atomic<uint64_t> capture_frames_enqueued{0};
    std::atomic<uint64_t> capture_frames_dequeued{0};
    std::atomic<uint64_t> capture_backpressure_events{0};
    std::atomic<uint64_t> completed_frames{0};
    std::atomic<size_t> capture_dynamic_allocated{0};
    std::atomic<size_t> capture_pending_at_stop{0};
    std::atomic<size_t> capture_max_encoder_deficit{0};
    std::atomic<int64_t> capture_longest_wait_ns{0};
    Clock::time_point capture_stop_time = start;
    size_t capture_data_high_water = 0;

    capture_thread = std::thread([&] { guard("capture-queue", [&] {
        int frame = 0;
        for (;;) {
            if (duration <= 0.0) {
                if (frame >= fixed_frames) break;
            } else if (frame > 0 && Clock::now() >= deadline) {
                break;
            }

            // In camera-simulation mode, release frames at the requested cadence.
            // Uncapped mode remains available for pure maximum-throughput tests.
            if (o.capture_fps > 0.0) {
                const auto release_time = start + std::chrono::duration_cast<Clock::duration>(
                    std::chrono::duration<double>(double(frame) / o.capture_fps));
                std::this_thread::sleep_until(release_time);
                if (duration > 0.0 && frame > 0 && Clock::now() >= deadline) break;
            }

            int ci = -1;
            if (!capture_free->try_pop(ci)) {
                std::lock_guard<std::mutex> lock(capture_alloc_mu);
                if (next_dynamic_slot < capture_total_capacity) {
                    ci = next_dynamic_slot++;
                    capture_buffers[size_t(ci)] =
                        std::make_unique<std::vector<uint16_t>>(src.size());
                    capture_dynamic_allocated.fetch_add(1, std::memory_order_relaxed);
                    log_event("WARN", "capture-protection",
                              "normal 12-frame processing pipeline exhausted; dynamically allocated Capture Protection Buffer slot " +
                              std::to_string(ci - NORMAL_CAPTURE_DEPTH + 1) + "/" +
                              std::to_string(capture_total_capacity - NORMAL_CAPTURE_DEPTH));
                }
            }
            if (ci < 0) {
                capture_backpressure_events.fetch_add(1, std::memory_order_relaxed);
                if (!capture_free->pop(ci)) return;
            }

            std::memcpy(capture_buffers[size_t(ci)]->data(), src.data(),
                        src.size() * sizeof(uint16_t));
            capture_enqueue_time[size_t(ci)] = Clock::now();
            // Count a captured frame before publishing it. A consumer may complete a
            // tiny frame immediately after Queue::push wakes it; publishing first used
            // to allow completed > enqueued briefly and underflow the unsigned deficit.
            const uint64_t enqueued_now =
                capture_frames_enqueued.fetch_add(1, std::memory_order_release) + 1u;
            if (!capture_ready->push(ci)) {
                capture_frames_enqueued.fetch_sub(1, std::memory_order_relaxed);
                return;
            }
            const uint64_t completed_now = completed_frames.load(std::memory_order_acquire);
            const size_t deficit_now = nonnegative_frame_difference(enqueued_now, completed_now);
            size_t prior = capture_max_encoder_deficit.load(std::memory_order_relaxed);
            while (deficit_now > prior && !capture_max_encoder_deficit.compare_exchange_weak(
                       prior, deficit_now, std::memory_order_relaxed)) {}
            ++frame;
        }

        capture_stop_time = Clock::now();
        const size_t pending = nonnegative_frame_difference(
            capture_frames_enqueued.load(std::memory_order_acquire),
            completed_frames.load(std::memory_order_acquire));
        capture_pending_at_stop.store(pending, std::memory_order_relaxed);
        if (collect) {
            if (pending > 0) {
                log_event("WARN", "record-stop",
                          "RECORDING STOPPED: " + std::to_string(pending) +
                          " captured frames remain in the RAW/GPU/entropy pipeline. "
                          "DO NOT REMOVE POWER until drain completes or buffered frames will be lost.");
            } else {
                log_event("INFO", "record-stop",
                          "RECORDING STOPPED: no buffered frames remain; safe to close immediately.");
            }
        }
        // Record the RAW-data high-water mark before adding the END sentinel.
        // Otherwise the sentinel can make utilisation exceed 100%.
        capture_data_high_water = capture_ready->high_water();
        (void)capture_ready->push(END);
    }); });

    std::thread producer([&] { guard("split", [&] {
        int frame = 0;
        for (;;) {
            int capture_index = -1;
            if (!capture_ready->pop(capture_index)) return;
            if (capture_index == END) break;
            capture_frames_dequeued.fetch_add(1, std::memory_order_relaxed);
            const auto queue_wait_ns = std::chrono::duration_cast<Ns>(
                Clock::now() - capture_enqueue_time[size_t(capture_index)]).count();
            int64_t longest = capture_longest_wait_ns.load(std::memory_order_relaxed);
            while (queue_wait_ns > longest && !capture_longest_wait_ns.compare_exchange_weak(
                       longest, queue_wait_ns, std::memory_order_relaxed)) {}
            const std::vector<uint16_t>* frame_src = capture_buffers[size_t(capture_index)].get();

            // Once recording stops, continue waiting for a processing slot until
            // every queued RAW frame has been encoded. The capture deadline must
            // never discard already captured frames.
            int gpu_token = -1;
            if (!gpu_tokens.pop(gpu_token)) return;
            int index = -1;
            if (!freeq.pop(index)) {
                (void)gpu_tokens.push(gpu_token);
                return;
            }

            auto& slot = vp.slots[size_t(index)];
            slot.frame = frame;
            slot.start = Clock::now();
            slot.vc5_bytes = 0;
            slot.crc = 0;
            slot.crc_computed = false;
            slot.entropy_worker = -1;
            slot.int8_range_ns = 0;
            slot.int8_pack_ns = 0;
            slot.cpu_tail_ns = 0;
            slot.snapshot_ns = 0;
            slot.hybrid_bytes = 0;
            slot.int8_bands = 0;
            slot.int16_fallback_bands = 0;
            slot.int8_tiles = 0;
            slot.int16_fallback_tiles = 0;
            slot.total_tiles = 0;
            const auto split_start = Clock::now();
            split_compand(o, frame_src->data(), lut, slot.mapped_ping, vp.plane_stride_elems);
            slot.split_ns = elapsed_ns(split_start);
            if (!capture_free->push(capture_index)) return;
            {
                std::lock_guard<std::mutex> submit_lock(gpu_submit_mu);
                if (onepass_packer) onepass_packer->submit_with_transform(size_t(index));
                else vp.submit(slot);
            }
            ++frame;
            produced.store(frame, std::memory_order_relaxed);
            if (!gpuq.push(index)) return;
        }
        submission_end = Clock::now();
        (void)gpuq.push(END);
    }); });

    std::thread completion([&] { guard("gpu", [&] {
        for (;;) {
            int index = -1;
            if (!gpuq.pop(index)) return;
            if (index == END) break;
            auto& completed_slot = vp.slots[size_t(index)];
            if (onepass_packer) {
                onepass_packer->wait_and_prepare(size_t(index));
                const auto& hm = onepass_packer->metrics(size_t(index));
                completed_slot.int8_range_ns = hm.range_ns;
                completed_slot.int8_pack_ns = hm.pack_ns;
                completed_slot.hybrid_bytes = hm.aligned_bytes;
                completed_slot.int8_bands = hm.int8_bands;
                completed_slot.int16_fallback_bands = hm.fallback_bands;
            } else {
                vp.wait(completed_slot);
                if (hybrid_packer) {
                    {
                        std::lock_guard<std::mutex> submit_lock(gpu_submit_mu);
                        hybrid_packer->submit_range(size_t(index));
                    }
                    hybrid_packer->wait_range_and_prepare_pack(size_t(index));
                    {
                        std::lock_guard<std::mutex> submit_lock(gpu_submit_mu);
                        hybrid_packer->submit_pack(size_t(index));
                    }
                    hybrid_packer->wait_pack(size_t(index));
                    const auto& hm = hybrid_packer->metrics(size_t(index));
                    completed_slot.int8_range_ns = hm.range_ns;
                    completed_slot.int8_pack_ns = hm.pack_ns;
                    completed_slot.hybrid_bytes = hm.aligned_bytes;
                    completed_slot.int8_bands = hm.int8_bands;
                    completed_slot.int16_fallback_bands = hm.fallback_bands;
                }
            }
            if (!gpu_tokens.push(0)) return;
            completed_slot.entropy_queue_enter=Clock::now();
            if (!entq.push(index)) return;
        }
        for (int worker = 0; worker < entropy_worker_count; ++worker)
            if (!entq.push(END)) return;
    }); });

    std::vector<std::thread> entropy_workers;
    entropy_workers.reserve(size_t(entropy_worker_count));
    for (int worker = 0; worker < entropy_worker_count; ++worker) {
        entropy_workers.emplace_back([&, worker] {
            const std::string stage_name = "entropy-" + std::to_string(worker);
            guard(stage_name.c_str(), [&] {
            DirectGprEncoder& direct = *direct_encoders[size_t(worker)];
            for (;;) {
                int index = -1;
                if (!entq.pop(index)) return;
                if (index == END) break;
                auto& slot = vp.slots[size_t(index)];
                const auto entropy_start = Clock::now();
                slot.entropy_worker = worker;
                slot.entropy_queue_wait_ns = std::chrono::duration_cast<Ns>(
                    entropy_start - slot.entropy_queue_enter).count();

                std::vector<int16_t> reconstructed;
                const int16_t* baseline_coefficients = nullptr;
                const vc5_pretransformed_hybrid_frame* hybrid_frame = nullptr;

                if (fused_provider) {
                    fused_provider->prepare(size_t(index));
                    const auto& metrics = fused_provider->metrics(size_t(index));
                    slot.hybrid_bytes = metrics.aligned_bytes;
                    slot.int8_bands = metrics.int8_bands;
                    slot.int16_fallback_bands = metrics.fallback_bands;
                    hybrid_frame = &fused_provider->frame(size_t(index));
                    baseline_coefficients = vp.coefficient_ptr(slot);
                } else if (tile_provider) {
                    tile_provider->prepare(size_t(index));
                    const auto& metrics = tile_provider->metrics(size_t(index));
                    slot.hybrid_bytes = metrics.physical_bytes;
                    slot.int8_tiles = metrics.int8_tiles;
                    slot.int16_fallback_tiles = metrics.fallback_tiles;
                    slot.total_tiles = metrics.total_tiles;
                    hybrid_frame = &tile_provider->frame(size_t(index));
                    if ((o.hybrid_verify_baseline && slot.frame == 0) ||
                        (collect && !o.dump_coeff.empty())) {
                        reconstructed = tile_provider->reconstruct_full_int16(size_t(index));
                        baseline_coefficients = reconstructed.data();
                    }
                } else if (vp.retained_int16_levels) {
                    if (vp.use_readback_copy || (collect && !o.dump_coeff.empty())) {
                        vp.gather_fused_coefficients(slot, slot.readback);
                        baseline_coefficients = slot.readback.data();
                    }
                } else {
                    const size_t total = vp.plane_stride_elems * 4u;
                    if (o.gpu_levels == 0) {
                        slot.readback.resize(total);
                        std::memcpy(slot.readback.data(), slot.mapped_ping,
                                    total * sizeof(int16_t));
                        baseline_coefficients = slot.readback.data();
                    } else {
                        baseline_coefficients = vp.coefficient_ptr(slot);
                        if (vp.use_readback_copy) {
                            const auto copy_start = Clock::now();
                            slot.readback.resize(total);
                            std::memcpy(slot.readback.data(), baseline_coefficients,
                                        total * sizeof(int16_t));
                            slot.snapshot_ns = elapsed_ns(copy_start);
                            baseline_coefficients = slot.readback.data();
                        }
                    }
                }

                if (o.gpu_levels < 3) {
                    if (!baseline_coefficients)
                        throw std::runtime_error("CPU wavelet tail has no coefficient source");
                    const auto tail_start = Clock::now();
                    cpu_transform_tail_inplace(o, run_mode,
                                               const_cast<int16_t*>(baseline_coefficients),
                                               vp.plane_stride_elems, vp.row_stride,
                                               o.gpu_levels == 0 ? 1 : 2);
                    slot.cpu_tail_ns = elapsed_ns(tail_start);
                }
                if (cpu_tile_provider) {
                    cpu_tile_provider->prepare(size_t(index), baseline_coefficients);
                    const auto& metrics = cpu_tile_provider->metrics(size_t(index));
                    slot.int8_range_ns = metrics.range_ns;
                    slot.int8_pack_ns = metrics.pack_ns;
                    slot.hybrid_bytes = metrics.physical_bytes;
                    slot.int8_tiles = metrics.int8_tiles;
                    slot.int16_fallback_tiles = metrics.fallback_tiles;
                    slot.total_tiles = metrics.total_tiles;
                    hybrid_frame = &cpu_tile_provider->frame(size_t(index));
                } else if (cpu_hybrid_provider) {
                    cpu_hybrid_provider->prepare(size_t(index), baseline_coefficients);
                    const auto& metrics = cpu_hybrid_provider->metrics(size_t(index));
                    slot.int8_range_ns = metrics.range_ns;
                    slot.int8_pack_ns = metrics.pack_ns;
                    slot.hybrid_bytes = metrics.aligned_bytes;
                    slot.int8_bands = metrics.int8_bands;
                    slot.int16_fallback_bands = metrics.fallback_bands;
                    hybrid_frame = &cpu_hybrid_provider->frame(size_t(index));
                } else if (hybrid_packer) {
                    hybrid_frame = &hybrid_packer->frame(size_t(index));
                } else if (onepass_packer) {
                    hybrid_frame = &onepass_packer->frame(size_t(index));
                }

                if (collect && !o.dump_coeff.empty()) {
                    std::lock_guard<std::mutex> lock(dump_mu);
                    if (!coefficient_dumped) {
                        if (!baseline_coefficients) {
                            if (tile_provider) {
                                reconstructed = tile_provider->reconstruct_full_int16(size_t(index));
                                baseline_coefficients = reconstructed.data();
                            } else if (vp.retained_int16_levels) {
                                vp.gather_fused_coefficients(slot, slot.readback);
                                baseline_coefficients = slot.readback.data();
                            }
                        }
                        if (!baseline_coefficients)
                            throw std::runtime_error("coefficient dump has no int16 reconstruction");
                        const std::filesystem::path dump_path(o.dump_coeff);
                        if (dump_path.has_parent_path())
                            std::filesystem::create_directories(dump_path.parent_path());
                        std::ofstream dump(dump_path, std::ios::binary | std::ios::trunc);
                        if (!dump)
                            throw std::runtime_error("cannot create coefficient dump: " + dump_path.string());
                        const size_t total = vp.plane_stride_elems * 4u;
                        dump.write(reinterpret_cast<const char*>(baseline_coefficients),
                                   std::streamsize(total * sizeof(int16_t)));
                        if (!dump)
                            throw std::runtime_error("coefficient dump write failed: " + dump_path.string());
                        coefficient_dumped = true;
                        log_event("INFO", "coeff-dump", "wrote " + dump_path.string() +
                                  " bytes=" + std::to_string(total * sizeof(int16_t)));
                    }
                }

                if (hybrid_frame) {
                    if (o.hybrid_verify_baseline && slot.frame == 0) {
                        if (!baseline_coefficients) {
                            reconstructed = tile_provider->reconstruct_full_int16(size_t(index));
                            baseline_coefficients = reconstructed.data();
                        }
                        gpr_buffer baseline{nullptr, 0};
                        size_t baseline_vc5_bytes = 0;
                        direct.encode(baseline_coefficients, baseline, baseline_vc5_bytes);
                        direct.encode_hybrid(*hybrid_frame, slot.gpr, slot.vc5_bytes);
                        const bool exact = baseline.size == slot.gpr.size &&
                            baseline_vc5_bytes == slot.vc5_bytes &&
                            std::memcmp(baseline.buffer, slot.gpr.buffer, baseline.size) == 0;
                        direct.free_buffer(baseline);
                        if (!exact)
                            throw std::runtime_error("hybrid GPR differs from int16 baseline");
                        std::lock_guard<std::mutex> output_lock(g_log_mu);
                        std::cout << "HYBRID_GPR_VERIFY exact_gpr=YES frame=0 mode="
                                  << (o.mode.empty() ? std::string("base-q") +
                                      std::to_string(o.quant) : o.mode)
                                  << " storage=" << coeff_storage_name(o) << "\n";
                    } else {
                        direct.encode_hybrid(*hybrid_frame, slot.gpr, slot.vc5_bytes);
                    }
                } else if (vp.retained_int16_levels && !vp.use_readback_copy) {
                    direct.encode_levels(vp.fused_level_planes(slot), slot.gpr, slot.vc5_bytes);
                } else {
                    if (!baseline_coefficients)
                        throw std::runtime_error("entropy stage has no coefficient source");
                    direct.encode(baseline_coefficients, slot.gpr, slot.vc5_bytes);
                }
                slot.entropy_ns = elapsed_ns(entropy_start);
                slot.entropy_finish = Clock::now();
                worker_frames[size_t(worker)].fetch_add(1, std::memory_order_relaxed);
                worker_entropy_ns[size_t(worker)].fetch_add(slot.entropy_ns, std::memory_order_relaxed);
                worker_queue_wait_ns[size_t(worker)].fetch_add(
                    slot.entropy_queue_wait_ns, std::memory_order_relaxed);
                if (!writeq.push(index)) return;
            }
            (void)writeq.push(END);
            });
        });
    }

    std::thread writer([&] { guard("writer", [&] {
        int completed = 0;
        int ended_workers = 0;
        int next_expected_frame = 0;
        int retained_final_frame = -1;
        int retained_final_worker = -1;
        std::map<int,int> reorder;
        auto commit_slot = [&](int index) {
            auto& slot = vp.slots[size_t(index)];
            if(slot.frame!=next_expected_frame) throw std::runtime_error("ordered writer internal sequence mismatch");
            const auto writer_start = Clock::now();
            slot.crc_computed = should_compute_frame_crc(o.no_crc, collect, slot.frame);
            slot.crc = slot.crc_computed ? crc32_bytes(slot.gpr.buffer, slot.gpr.size) : 0u;
            const size_t packet = slot.gpr.size;
            slot.writer_ns = elapsed_ns(writer_start);
            slot.finish = Clock::now();

            ++completed;
            ++next_expected_frame;
            completed_frames.store(uint64_t(completed), std::memory_order_release);
            if (collect) {
                FrameStat stat;
                stat.frame = slot.frame;
                stat.latency = to_ms(std::chrono::duration_cast<Ns>(slot.finish - slot.start).count());
                stat.split = to_ms(slot.split_ns);
                stat.gpu = to_ms(slot.gpu_wall_ns);
                stat.gpu_exec = slot.gpu_exec_ns < 0 ? -1.0 : to_ms(slot.gpu_exec_ns);
                stat.snapshot = to_ms(slot.snapshot_ns);
                stat.snapshot_bytes = (slot.snapshot_ns > 0 || vp.use_vulkan_readback_copy) ? coefficient_int16_bytes(o) : 0u;
                stat.cpu_tail = to_ms(slot.cpu_tail_ns);
                stat.entropy = to_ms(slot.entropy_ns);
                stat.entropy_queue_wait = to_ms(slot.entropy_queue_wait_ns);
                stat.int8_range = to_ms(slot.int8_range_ns);
                stat.int8_pack = to_ms(slot.int8_pack_ns);
                stat.hybrid_bytes = slot.hybrid_bytes;
                stat.int8_bands = slot.int8_bands;
                stat.int16_fallback_bands = slot.int16_fallback_bands;
                stat.int8_tiles = slot.int8_tiles;
                stat.int16_fallback_tiles = slot.int16_fallback_tiles;
                stat.total_tiles = slot.total_tiles;
                stat.writer = to_ms(slot.writer_ns);
                stat.entropy_worker = slot.entropy_worker;
                stat.payload = slot.vc5_bytes;
                stat.packet = packet;
                stat.crc = slot.crc;
                stat.crc_computed = slot.crc_computed;
                stat.finish = slot.finish;
                stats.push_back(stat);

                if (csv) {
                    (*csv) << slot.frame << ',' << index << ',' << slot.entropy_worker << ','
                           << std::fixed << std::setprecision(3)
                           << stat.latency << ',' << stat.split << ',' << stat.gpu << ','
                           << stat.gpu_exec << ',' << stat.snapshot << ',' << stat.snapshot_bytes << ',' << 0.0 << ','
                           << stat.cpu_tail << ',' << stat.entropy_queue_wait << ',' << stat.int8_range << ',' << stat.int8_pack << ','
                           << stat.hybrid_bytes << ',' << stat.int8_bands << ','
                           << stat.int16_fallback_bands << ',' << stat.int8_tiles << ','
                           << stat.int16_fallback_tiles << ',' << stat.total_tiles << ','
                           << stat.entropy << ',' << stat.writer << ','
                           << stat.payload << ',' << stat.packet << ',' << stat.crc << ','
                           << (stat.crc_computed ? "YES" : "NO") << '\n';
                    csv->flush();
                    if (!*csv) throw std::runtime_error("per-frame CSV write failed");
                }
                if (completed == 1 || completed % o.progress_every == 0 ||
                    (duration <= 0.0 && completed == fixed_frames)) {
                    log_event("INFO", "progress",
                              "completed=" + std::to_string(completed) +
                              " latency_ms=" + std::to_string(stat.latency) +
                              " vc5_bytes=" + std::to_string(stat.payload) +
                              " gpr_bytes=" + std::to_string(stat.packet));
                }
            }
            const size_t pending_now = nonnegative_frame_difference(
                capture_frames_enqueued.load(std::memory_order_acquire),
                completed_frames.load(std::memory_order_acquire));
            if (collect && capture_pending_at_stop.load(std::memory_order_relaxed) > 0 &&
                (pending_now == 0 || pending_now % 30 == 0)) {
                log_event(pending_now == 0 ? "INFO" : "WARN", "drain",
                          pending_now == 0
                              ? "BUFFER DRAIN COMPLETE: all captured frames encoded; power may now be removed safely."
                              : "post-record encode drain: " + std::to_string(pending_now) +
                                " frames remaining. DO NOT REMOVE POWER.");
            }
            if (final_gpr && collect && slot.frame > retained_final_frame) {
                if(final_gpr->buffer){
                    const int owner=retained_final_worker>=0?retained_final_worker:0;
                    direct_encoders[size_t(owner)]->free_buffer(*final_gpr);
                }
                *final_gpr = slot.gpr;
                slot.gpr = {nullptr,0};
                retained_final_frame = slot.frame;
                retained_final_worker = slot.entropy_worker;
            } else {
                const int owner=slot.entropy_worker>=0?slot.entropy_worker:0;
                direct_encoders[size_t(owner)]->free_buffer(slot.gpr);
            }
            slot.entropy_worker=-1;
            if (!freeq.push(index)) throw std::runtime_error("processing slot queue closed during ordered commit");
        };

        for (;;) {
            int index = -1;
            if (!writeq.pop(index)) return;
            if (index == END) {
                if (++ended_workers != entropy_worker_count) continue;
                while(true){auto it=reorder.find(next_expected_frame);if(it==reorder.end())break;const int ready=it->second;reorder.erase(it);commit_slot(ready);}
                if(!reorder.empty()) throw std::runtime_error("entropy workers ended with a gap in ordered output commit");
                break;
            }
            auto& slot=vp.slots[size_t(index)];
            if(slot.frame!=next_expected_frame) out_of_order_entropy_finishes.fetch_add(1,std::memory_order_relaxed);
            if(!reorder.emplace(slot.frame,index).second) throw std::runtime_error("duplicate frame in ordered output reorder buffer");
            const size_t held_out_of_order = reorder.size() - (reorder.count(next_expected_frame) ? 1u : 0u);
            size_t previous=reorder_high_water.load(std::memory_order_relaxed);
            while(held_out_of_order>previous && !reorder_high_water.compare_exchange_weak(previous,held_out_of_order,std::memory_order_relaxed)){}
            while(true){auto it=reorder.find(next_expected_frame);if(it==reorder.end())break;const int ready=it->second;reorder.erase(it);commit_slot(ready);}
        }
    }); });

    if (capture_thread.joinable()) capture_thread.join();
    producer.join();
    completion.join();
    for (auto& thread : entropy_workers) thread.join();
    writer.join();
    const auto end = Clock::now();
    cancel();
    for (auto& slot : vp.slots) { const int owner=slot.entropy_worker>=0?slot.entropy_worker:0; direct_encoders[size_t(owner)]->free_buffer(slot.gpr); }
    if (error) std::rethrow_exception(error);
    if (collect)
        std::sort(stats.begin(), stats.end(), [](const FrameStat& a, const FrameStat& b) {
            return a.frame < b.frame;
        });

    Result result;
    result.frames = produced.load(std::memory_order_relaxed);
    result.submission_seconds = std::chrono::duration<double>(submission_end - start).count();
    result.total_seconds = std::chrono::duration<double>(end - start).count();
    result.capture_queue_capacity = size_t(capture_total_capacity);
    result.capture_queue_high_water = capture_data_high_water;
    result.capture_protection_high_water = result.capture_queue_high_water > size_t(NORMAL_CAPTURE_DEPTH)
        ? result.capture_queue_high_water - size_t(NORMAL_CAPTURE_DEPTH) : 0u;
    result.capture_frames_enqueued = capture_frames_enqueued.load(std::memory_order_relaxed);
    result.capture_frames_dequeued = capture_frames_dequeued.load(std::memory_order_relaxed);
    result.capture_backpressure_events = capture_backpressure_events.load(std::memory_order_relaxed);
    result.capture_dynamic_allocated = capture_dynamic_allocated.load(std::memory_order_relaxed);
    result.capture_pending_at_stop = capture_pending_at_stop.load(std::memory_order_relaxed);
    result.capture_drain_seconds = result.capture_pending_at_stop > 0
        ? std::chrono::duration<double>(end - capture_stop_time).count() : 0.0;
    result.capture_longest_wait_ms = to_ms(capture_longest_wait_ns.load(std::memory_order_relaxed));
    result.capture_max_encoder_deficit = capture_max_encoder_deficit.load(std::memory_order_relaxed);
    const ProcessMemoryInfo process_mem = read_process_memory_info();
    result.process_rss_bytes = process_mem.rss_bytes;
    result.process_peak_rss_bytes = process_mem.peak_rss_bytes;
    result.reorder_high_water = reorder_high_water.load(std::memory_order_relaxed);
    result.out_of_order_entropy_finishes = out_of_order_entropy_finishes.load(std::memory_order_relaxed);
    result.worker_frames.resize(size_t(entropy_worker_count));
    result.worker_avg_entropy_ms.resize(size_t(entropy_worker_count));
    result.worker_avg_queue_wait_ms.resize(size_t(entropy_worker_count));
    result.worker_utilisation_pct.resize(size_t(entropy_worker_count));
    for(int worker=0;worker<entropy_worker_count;++worker){const uint64_t frames=worker_frames[size_t(worker)].load(std::memory_order_relaxed);const int64_t busy=worker_entropy_ns[size_t(worker)].load(std::memory_order_relaxed);const int64_t wait=worker_queue_wait_ns[size_t(worker)].load(std::memory_order_relaxed);result.worker_frames[size_t(worker)]=frames;result.worker_avg_entropy_ms[size_t(worker)]=frames?to_ms(busy)/double(frames):0.0;result.worker_avg_queue_wait_ms[size_t(worker)]=frames?to_ms(wait)/double(frames):0.0;result.worker_utilisation_pct[size_t(worker)]=result.total_seconds>0.0?100.0*(double(busy)/1.0e9)/result.total_seconds:0.0;}
    if (!collect || stats.empty()) return result;
    // First measured (non-warmup) frame; stats is sorted ascending by frame index.
    // This is a CRC of the complete GPR container, not merely the VC-5 payload.
    // The legacy first_frame_payload_crc32 token is retained for compatibility,
    // while first_frame_gpr_crc32 and the explicit computed marker are authoritative.
    result.first_frame_crc = stats.front().crc;
    result.first_frame_crc_computed = stats.front().crc_computed;
    if (stats.size() != size_t(result.frames)) {
        throw std::runtime_error("frame count mismatch: produced=" +
                                 std::to_string(result.frames) + " completed=" +
                                 std::to_string(stats.size()));
    }

    const double frame_count = static_cast<double>(stats.size());
    std::vector<double> latencies;
    latencies.reserve(stats.size());
    const CompletionTiming completion_metrics = completion_timing(stats);
    double sum_split = 0.0, sum_gpu = 0.0, sum_gpu_exec = 0.0;
    double sum_snapshot = 0.0, sum_cpu_tail = 0.0;
    uint64_t sum_snapshot_bytes = 0;
    double sum_int8_range = 0.0, sum_int8_pack = 0.0;
    double sum_entropy = 0.0, sum_writer = 0.0;
    uint64_t sum_hybrid_bytes = 0, sum_int8_bands = 0, sum_fallback_bands = 0;
    uint64_t sum_int8_tiles = 0, sum_fallback_tiles = 0, sum_total_tiles = 0;
    size_t gpu_exec_frames = 0;
    uint64_t payload_bytes = 0, packet_bytes = 0;
    for (size_t i = 0; i < stats.size(); ++i) {
        latencies.push_back(stats[i].latency);
        sum_split += stats[i].split;
        sum_gpu += stats[i].gpu;
        sum_snapshot += stats[i].snapshot;
        sum_snapshot_bytes += stats[i].snapshot_bytes;
        sum_cpu_tail += stats[i].cpu_tail;
        sum_int8_range += stats[i].int8_range;
        sum_int8_pack += stats[i].int8_pack;
        sum_hybrid_bytes += stats[i].hybrid_bytes;
        sum_int8_bands += stats[i].int8_bands;
        sum_fallback_bands += stats[i].int16_fallback_bands;
        sum_int8_tiles += stats[i].int8_tiles;
        sum_fallback_tiles += stats[i].int16_fallback_tiles;
        sum_total_tiles += stats[i].total_tiles;
        sum_entropy += stats[i].entropy;
        sum_writer += stats[i].writer;
        payload_bytes += stats[i].payload;
        packet_bytes += stats[i].packet;
        if (stats[i].gpu_exec >= 0.0) {
            sum_gpu_exec += stats[i].gpu_exec;
            ++gpu_exec_frames;
        }
    }
    auto percentile = [](std::vector<double> values, double p) {
        if (values.empty()) return 0.0;
        std::sort(values.begin(), values.end());
        const size_t index = std::min(
            values.size() - 1,
            size_t(std::ceil(p * static_cast<double>(values.size())) - 1.0));
        return values[index];
    };

    result.avg_latency = std::accumulate(latencies.begin(), latencies.end(), 0.0) /
                         frame_count;
    result.p99_latency = percentile(latencies, 0.99);
    result.p99_gap = percentile(completion_metrics.gaps_ms, 0.99);
    result.avg_split = sum_split / frame_count;
    result.avg_gpu = sum_gpu / frame_count;
    result.avg_gpu_exec = gpu_exec_frames
        ? sum_gpu_exec / static_cast<double>(gpu_exec_frames) : -1.0;
    result.avg_snapshot = sum_snapshot / frame_count;
    result.snapshot_bytes = uint64_t(sum_snapshot_bytes / uint64_t(result.frames));
    result.cacheable_hybrid_handoff = vp.use_readback_copy || vp.use_vulkan_readback_copy ||
        o.coeff_storage == Options::CoeffStorage::CpuHybridBand ||
        o.coeff_storage == Options::CoeffStorage::CpuHybridTile ||
        o.coeff_storage == Options::CoeffStorage::GpuHybridOnePass;
    result.avg_cpu_tail = sum_cpu_tail / frame_count;
    result.avg_int8_range = sum_int8_range / frame_count;
    result.avg_int8_pack = sum_int8_pack / frame_count;
    result.avg_hybrid_bytes = uint64_t(sum_hybrid_bytes / uint64_t(result.frames));
    result.avg_int8_bands = uint32_t(sum_int8_bands / uint64_t(result.frames));
    result.avg_int16_fallback_bands = uint32_t(sum_fallback_bands / uint64_t(result.frames));
    result.avg_int8_tiles = uint32_t(sum_int8_tiles / uint64_t(result.frames));
    result.avg_int16_fallback_tiles = uint32_t(sum_fallback_tiles / uint64_t(result.frames));
    result.avg_total_tiles = uint32_t(sum_total_tiles / uint64_t(result.frames));
    result.avg_entropy = sum_entropy / frame_count;
    result.avg_writer = sum_writer / frame_count;
    result.payload = size_t(payload_bytes / uint64_t(result.frames));
    result.packet = size_t(packet_bytes / uint64_t(result.frames));
    const size_t input = size_t(o.width) * size_t(o.height) * 2u;
    result.payload_ratio = double(input) / double(std::max<size_t>(1u, result.payload));
    result.packet_ratio = double(input) / double(std::max<size_t>(1u, result.packet));
    if (result.frames > 1) {
        result.fps = completion_metrics.span_seconds > 0.0
            ? double(result.frames - 1) / completion_metrics.span_seconds : 0.0;
    } else {
        result.fps = result.avg_latency > 0.0 ? 1000.0 / result.avg_latency : 0.0;
    }
    result.output_mibs = double(result.packet) * result.fps / 1048576.0;
    return result;
}


// Prove the NEON wavelet is bit-identical to the scalar reference. Runs on every
// invocation, before any timing, over geometries and value patterns that exercise
// both edge coefficient sets, both prescale values, the vector remainder, and the
// quantiser sign path. A mismatch is fatal: the scalar path is the oracle the
// 12,400-case gate validates against, so a divergent NEON kernel would silently
// corrupt both the reference and the output.
static void self_test_cpu_wavelet_neon() {
#if CINEPI_HAVE_NEON_WAVELET
    uint32_t rs = 20260804u;
    auto rng = [&]() { rs = rs * 1664525u + 1013904223u; return rs >> 8; };
    const int quant_sets[4][3] = {{1,1,1},{95,95,143},{24,24,12},{2,3,320}};
    // dir==0 writes count rows x 2*outs columns; dir==1 writes 2*outs rows x count
    // columns, and 2*outs == axis_len for even axis_len. One square buffer of
    // max(axis,count)+8 therefore covers both directions without aliasing --
    // getting this wrong makes the test itself overflow and report phantom
    // mismatches, which is how the first version of it failed.
    struct G { uint32_t axis, count; };
    static const G geoms[] = {
        {16,8},{16,16},{24,16},{32,32},{64,24},{80,64},{128,120},{240,120},
        {1920,1080},{1080,1920},{960,540},{540,960},{480,270},{270,480},
    };
    size_t compared = 0;
    for (const auto& g : geoms) {
        const size_t dim = size_t(std::max(g.axis, g.count)) + 8u;
        std::vector<int16_t> src(dim * dim, 0);
        const int patterns = (dim > 600) ? 2 : 3;
        for (int pattern = 0; pattern < patterns; ++pattern) {
            for (size_t k = 0; k < src.size(); ++k) {
                if (pattern == 0) src[k] = int16_t(int(rng() % 8191u) - 4095);
                else if (pattern == 1) src[k] = int16_t((k & 1) ? 4094 : -4094);
                else src[k] = int16_t(int(rng() % 32767u) - 16383);
            }
            for (uint32_t dir : {0u, 1u}) {
                for (int prescale : {0, 2}) {
                    for (const auto& q : quant_sets) {
                        CpuWaveletPush pc{};
                        pc.axis_len = g.axis; pc.count = g.count;
                        pc.stride = uint32_t(dim); pc.out_stride = uint32_t(dim);
                        pc.dir = dir; pc.prescale = prescale;
                        pc.q_lh = q[0]; pc.q_hl = q[1]; pc.q_hh = q[2];
                        // v1.7.53: the scalar reference, the v1.7.52 column-block
                        // sweep and the v1.7.53 row-major pass at both vector
                        // widths must all agree bit for bit. Anything that only
                        // checks the shipped default cannot catch a regression in
                        // the variant the next A/B run will select.
                        std::vector<int16_t> a(src.size(), 0), b(src.size(), 0);
                        cpu_wavelet_pass_scalar(src.data(), a.data(), pc);
                        cpu_wavelet_pass_neon(src.data(), b.data(), pc);
                        for (int blocks : {1, 2}) {
                            const int saved_blocks = g_cpu_wavelet_vec_blocks;
                            g_cpu_wavelet_vec_blocks = blocks;
                            std::vector<int16_t> c(src.size(), 0);
                            cpu_wavelet_pass_v53(src.data(), c.data(), pc);
                            g_cpu_wavelet_vec_blocks = saved_blocks;
                            for (size_t k = 0; k < a.size(); ++k)
                                if (a[k] != c[k])
                                    throw std::runtime_error(
                                        "v1.7.53 CPU wavelet diverged from the scalar reference: axis=" +
                                        std::to_string(g.axis) + " count=" + std::to_string(g.count) +
                                        " dir=" + std::to_string(dir) + " prescale=" + std::to_string(prescale) +
                                        " vec_blocks=" + std::to_string(blocks) +
                                        " index=" + std::to_string(k) +
                                        " scalar=" + std::to_string(a[k]) + " v53=" + std::to_string(c[k]));
                        }
                        for (size_t k = 0; k < a.size(); ++k)
                            if (a[k] != b[k])
                                throw std::runtime_error(
                                    "NEON CPU wavelet diverged from the scalar reference: axis=" +
                                    std::to_string(g.axis) + " count=" + std::to_string(g.count) +
                                    " dir=" + std::to_string(dir) + " prescale=" + std::to_string(prescale) +
                                    " index=" + std::to_string(k) +
                                    " scalar=" + std::to_string(a[k]) + " neon=" + std::to_string(b[k]));
                        ++compared;
                    }
                }
            }
        }
    }
    std::cout << "CPU_WAVELET_NEON_SELFTEST PASS cases=" << compared
              << " geometries=" << (sizeof(geoms) / sizeof(geoms[0]))
              << " variants=scalar+v52+v53x8+v53x16\n";
#endif
}

static void self_test() {
    self_test_cpu_wavelet_neon();
    Options base;
    base.width = 128;
    base.height = 96;
    base.threads = 2;
    base.bayer = "rggb";

    const auto log_lut = make_lut(base);
    if (log_lut.front() != 0 || log_lut[size_t(base.white)] != uint16_t(base.working_max)) {
        throw std::runtime_error("LUT endpoint test failed");
    }

    const int pw = base.width / 2;
    const int ph = base.height / 2;
    const size_t row = size_t(pw);
    const size_t plane_elems = row * size_t(ph);
    std::vector<uint16_t> identity(size_t(1u) << unsigned(base.effective_bits));
    std::iota(identity.begin(), identity.end(), uint16_t(0));
    std::vector<uint16_t> src(size_t(base.width) * size_t(base.height), uint16_t(0));

    auto cell_values = [&](int x, int y, int& r, int& g1, int& g2, int& b) {
        const int pattern = (y * pw + x) % 500;
        r = 100 + pattern;
        g1 = 500 + pattern;
        g2 = 1000 + pattern;
        b = 1500 + pattern;
    };
    for (int y = 0; y < ph; ++y) {
        for (int x = 0; x < pw; ++x) {
            int r, g1, g2, b;
            cell_values(x, y, r, g1, g2, b);
            const size_t row0 = size_t(2*y) * size_t(base.width);
            const size_t row1 = row0 + size_t(base.width);
            src[row0 + size_t(2*x)] = uint16_t(r);
            src[row0 + size_t(2*x+1)] = uint16_t(g1);
            src[row1 + size_t(2*x)] = uint16_t(g2);
            src[row1 + size_t(2*x+1)] = uint16_t(b);
        }
    }

    uint32_t transform_crc = 0;
    for (const char* codec_name : {"vc5-444"}) {
        const std::string codec(codec_name);
        Options o = base;
        o.codec = codec;
        std::vector<int16_t> planes(plane_elems * 4u, int16_t(0));
        split_compand(o, src.data(), identity, planes.data(), plane_elems);
        const int16_t* gs_plane = planes.data();
        const int16_t* rg_plane = planes.data() + plane_elems;
        const int16_t* bg_plane = planes.data() + 2u * plane_elems;
        const int16_t* gd_plane = planes.data() + 3u * plane_elems;

        for (int y = 0; y < ph; ++y) {
            for (int x = 0; x < pw; ++x) {
                int r, g1, g2, b;
                cell_values(x, y, r, g1, g2, b);
                const int gs = (g1 + g2) >> 1;
                const int rg = (r - gs + 4096) >> 1;
                const int bg = (b - gs + 4096) >> 1;
                const int gd = (g1 - g2 + 4096) >> 1;
                const size_t i = size_t(y) * row + size_t(x);
                if (gs_plane[i] != gs || rg_plane[i] != rg ||
                    bg_plane[i] != bg || gd_plane[i] != gd) {
                    throw std::runtime_error(codec + " GPR component split test failed");
                }
            }
        }

        const auto specs = plane_specs(o);
        const uint32_t expected_id = 444u;
        if (codec_profile_id(o) != expected_id) {
            throw std::runtime_error(codec + " profile-ID test failed");
        }
        if (dispatches_per_frame(o) != 24) {
            throw std::runtime_error(codec + " dispatch-count test failed");
        }
        const int expected_chroma_width = pw;
        const int expected_chroma_height = ph;
        const int expected_chroma_levels = 3;
        if (specs[0].width != expected_chroma_width ||
            specs[0].height != expected_chroma_height ||
            specs[0].levels != expected_chroma_levels ||
            specs[3].width != expected_chroma_width ||
            specs[3].height != expected_chroma_height ||
            specs[3].levels != expected_chroma_levels ||
            specs[1].width != pw || specs[1].height != ph || specs[1].levels != 3 ||
            specs[2].width != pw || specs[2].height != ph || specs[2].levels != 3) {
            throw std::runtime_error(codec + " plane-spec test failed");
        }
        cpu_transform_schedule(o, nullptr, planes.data(), plane_elems, row);
        int max_abs = 0;
        for (int p = 0; p < 4; ++p) {
            for (int y = 0; y < specs[size_t(p)].height; ++y) {
                for (int x = 0; x < specs[size_t(p)].width; ++x) {
                    const size_t i = size_t(p) * plane_elems + size_t(y) * row + size_t(x);
                    max_abs = std::max(max_abs, std::abs(int(planes[i])));
                }
            }
        }
        if (max_abs >= 32767) throw std::runtime_error(codec + " CPU transform saturated int16");

        transform_crc ^= crc32_bytes(planes.data(), planes.size() * sizeof(int16_t));
    }

    // GPR mode table checks. The active encoder passes these exact quantisers
    // to the SDK Table-17 entropy writer; there is no secondary entropy path.
    {
        if (mode_index_by_name("m3") != 3u || mode_index_by_name("zz") != 0u)
            throw std::runtime_error("mode lookup failed");

        // The production ladder is exact and singular. Guard every mode so a
        // future edit cannot silently change one divisor or reintroduce a
        // historical table.
        const std::array<std::array<int,10>,10> expected_quant = {{
            {{1,4,4,2,8,8,6,26,26,39}},
            {{1,4,4,2,8,8,6,33,33,50}},
            {{1,4,4,2,10,10,6,43,43,66}},
            {{1,6,6,4,14,14,10,55,55,87}},
            {{1,8,8,4,16,16,11,73,73,111}},
            {{1,10,10,6,20,20,14,93,93,143}},
            {{1,14,14,8,26,26,18,126,126,175}},
            {{1,18,18,9,34,34,22,155,155,235}},
            {{1,22,22,13,44,44,30,201,201,301}},
            {{1,29,29,13,45,45,35,272,272,521}},
        }};
        for (int k = 1; k <= 10; ++k) {
            const auto got = resolve_mode("m" + std::to_string(k), 599.0, 4094).quant_table;
            if (got != expected_quant[size_t(k - 1)])
                throw std::runtime_error("production quant table mismatch at m" + std::to_string(k));
        }
        // Quality may not improve as the mode number rises. Divisor validity is
        // enforced by pack_gpu_quantizer() below, which is the actual codec/GPU
        // representation guard; the obsolete policy ceiling of 400 is removed
        // because Universal v3 intentionally uses 521 at M10 L1-HH.
        for (int k = 1; k < 10; ++k) {
            const ModeSpec lo = resolve_mode("m" + std::to_string(k), 599.0, 4094);
            const ModeSpec hi = resolve_mode("m" + std::to_string(k + 1), 599.0, 4094);
            for (size_t i = 1; i < 10; ++i) {
                if (lo.quant_table[i] > hi.quant_table[i])
                    throw std::runtime_error("ladder non-monotonic: m" + std::to_string(k)
                        + " is coarser than m" + std::to_string(k + 1)
                        + " at band " + std::to_string(i));
            }
        }

        int previous_strength = 0;
        for (const auto& profile : kModeProfiles) {
            const ModeSpec resolved = resolve_mode(profile.name, 599.0, 4094);
            const int strength = std::accumulate(resolved.quant_table.begin() + 1,
                                                 resolved.quant_table.end(), 0);
            if (strength < previous_strength)
                throw std::runtime_error(std::string("mode quantisation ladder regressed at ") + profile.name);
            previous_strength = strength;

            // Prove that the packed GPU constant is bit-identical to the
            // original codec quantiser for every active divisor and the full
            // signed int16 coefficient domain.
            for (const int divisor : resolved.quant_table) {
                const int32_t packed = pack_gpu_quantizer(divisor);
                for (int value = -32768; value <= 32767; ++value) {
                    if (cpu_quantize_packed(value, packed) != cpu_quantize_exact(value, divisor))
                        throw std::runtime_error(std::string("packed GPU quantizer mismatch in ") +
                                                 profile.name + " divisor=" + std::to_string(divisor) +
                                                 " value=" + std::to_string(value));
                }
            }
        }
    }

    if (transform_crc == 0) throw std::runtime_error("transform CRC self-test produced an unexpected zero");

    // Cacheable handoff compact-copy test. Only int16 fallback tiles may be
    // copied from the uncached Vulkan mapping. All int8 tiles must leave the
    // cacheable fallback pool untouched, while edge tiles copy only their
    // active rows and columns. Exercise both bitmap and one-u32-per-tile flags.
    {
        auto test_compact_fallback_copy = [](bool u32_flags) {
            VulkanPipeline vp;
            vp.row_stride = 64u;
            vp.plane_elems = 64u * 64u;
            vp.tile_linear_layout = true;
            vp.tile_flags_u32 = u32_flags;

            uint32_t next_tile = 0u;
            uint32_t next_coefficient = 0u;
            for (uint32_t plane = 0; plane < 4u; ++plane) {
                uint32_t cw = uint32_t(vp.row_stride);
                uint32_t ch = uint32_t(vp.plane_elems / vp.row_stride);
                for (uint32_t level = 0; level < 3u; ++level) {
                    const uint32_t bw = cw >> 1u;
                    const uint32_t bh = ch >> 1u;
                    const uint32_t tiles_x = (bw + 7u) / 8u;
                    const uint32_t tiles_y = (bh + 7u) / 8u;
                    const uint32_t tile_count = tiles_x * tiles_y;
                    for (uint32_t band = 0; band < 3u; ++band) {
                        vp.tile_base[plane][level][band] = next_tile;
                        vp.tile_coeff_base[plane][level][band] = next_coefficient;
                        next_tile += tile_count;
                        next_coefficient += bw * bh;
                    }
                    cw = bw;
                    ch = bh;
                }
            }
            vp.tile_total_count = next_tile;
            vp.tile_linear_coefficient_count = next_coefficient;
            vp.tile_i16_buffer_bytes = size_t(next_coefficient) * sizeof(int16_t);
            vp.tile_flags_buffer_bytes = u32_flags
                ? size_t(next_tile) * sizeof(uint32_t)
                : size_t((next_tile + 31u) / 32u) * sizeof(uint32_t);

            std::vector<int16_t> source(next_coefficient);
            std::vector<int16_t> destination(next_coefficient, int16_t(-23117));
            for (uint32_t i = 0; i < next_coefficient; ++i)
                source[i] = int16_t((i * 37u + 11u) & 0x7fffu);
            std::vector<uint32_t> flags(vp.tile_flags_buffer_bytes / sizeof(uint32_t), 0u);

            uint64_t expected_bytes = 0u;
            std::vector<uint8_t> expected(next_coefficient, uint8_t(0));
            for (uint32_t plane = 0; plane < 4u; ++plane) {
                uint32_t cw = uint32_t(vp.row_stride);
                uint32_t ch = uint32_t(vp.plane_elems / vp.row_stride);
                for (uint32_t level = 0; level < 3u; ++level) {
                    const uint32_t bw = cw >> 1u;
                    const uint32_t bh = ch >> 1u;
                    const uint32_t tiles_x = (bw + 7u) / 8u;
                    const uint32_t tiles_y = (bh + 7u) / 8u;
                    const uint32_t tile_count = tiles_x * tiles_y;
                    for (uint32_t band = 0; band < 3u; ++band) {
                        const uint32_t tile_base = vp.tile_base[plane][level][band];
                        const uint32_t coeff_base = vp.tile_coeff_base[plane][level][band];
                        // Select a different tile per band, including the final
                        // tile so partial edge geometry is covered if dimensions
                        // change in a future self-test.
                        const uint32_t selected = (plane + level + band) % tile_count;
                        const uint32_t tile = tile_base + selected;
                        if (u32_flags) flags[tile] = 1u;
                        else flags[tile >> 5u] |= 1u << (tile & 31u);
                        const uint32_t tx = selected % tiles_x;
                        const uint32_t ty = selected / tiles_x;
                        const uint32_t x0 = tx * 8u;
                        const uint32_t y0 = ty * 8u;
                        const uint32_t columns = std::min(8u, bw - x0);
                        const uint32_t rows = std::min(8u, bh - y0);
                        expected_bytes += uint64_t(columns) * rows * sizeof(int16_t);
                        for (uint32_t y = 0; y < rows; ++y)
                            for (uint32_t x = 0; x < columns; ++x)
                                expected[size_t(coeff_base) + size_t(y0 + y) * bw + x0 + x] = 1u;
                    }
                    cw = bw;
                    ch = bh;
                }
            }

            const uint64_t copied = copy_tile_direct_fallback_int16(
                vp, flags.data(), source.data(), destination.data());
            if (copied != expected_bytes)
                throw std::runtime_error("compact fallback snapshot byte count mismatch");
            for (size_t i = 0; i < destination.size(); ++i) {
                const int16_t expected_value = expected[i] ? source[i] : int16_t(-23117);
                if (destination[i] != expected_value)
                    throw std::runtime_error("compact fallback snapshot copied the wrong coefficient set");
            }
        };
        test_compact_fallback_copy(false);
        test_compact_fallback_copy(true);
    }

    // Capture budget policy. The class table cannot be exercised on this host's
    // real RAM, so drive the pure policy function directly with the MemTotal a
    // board of each class actually reports (firmware reserves a few per cent).
    {
        const uint64_t mib = 1024ull * 1024ull;
        struct Case { uint64_t total_mib, avail_mib, expect_mib; const char* cls; const char* why; };
        const Case cases[] = {
            // Idle board: the class budget must be reachable, not clipped by a
            // second OS reserve. 16 GB is the one class where MemAvailable can
            // still bind slightly, since 14.5 GiB plus the working margin is
            // close to everything the board has.
            {  7800,  7400,  6656,  "8GB", "class"     },
            {  3800,  3600,  2560,  "4GB", "class"     },
            {  1900,  1800,   512,  "2GB", "class"     },
            { 15700, 15600, 14848, "16GB", "class"     },
            { 15700, 15000, 14488, "16GB", "available" },
            // Below the 2 GB class there is no overflow budget at all.
            {   900,   850,     0, "minimal", "class"  },
            {   450,   400,     0, "minimal", "class"  },
            // Busy system: MemAvailable less the 1.5 GiB OS reserve binds first.
            { 15700,  4000,  3488, "16GB", "available" },
            {  7800,  3000,  2488,  "8GB", "available" },
            {  3800,  2000,  1488,  "4GB", "available" },
            // Available at or below the reserve yields nothing rather than
            // pushing the OS into reclaim.
            {  7800,   512,     0,  "8GB", "available" },
            {  7800,   400,     0,  "8GB", "available" },
            {  1900,   600,    88,  "2GB", "available" },
            // Exact class boundaries.
            { 14336, 99999, 14848, "16GB", "class"     },
            { 14335, 99999,  6656,  "8GB", "class"     },
            {  6144, 99999,  6656,  "8GB", "class"     },
            {  6143, 99999,  2560,  "4GB", "class"     },
            {  3072, 99999,  2560,  "4GB", "class"     },
            {  3071, 99999,   512,  "2GB", "class"     },
            {  1536, 99999,   512,  "2GB", "class"     },
            {  1535, 99999,     0, "minimal", "class"  },
        };
        for (const auto& c : cases) {
            const CaptureBudget b = capture_budget(c.total_mib * mib, c.avail_mib * mib);
            const uint64_t got_mib = b.bytes / mib;
            if (got_mib != c.expect_mib || std::string(b.class_name) != c.cls)
                throw std::runtime_error("capture budget self-test failed at total_mib=" +
                    std::to_string(c.total_mib) + " avail_mib=" + std::to_string(c.avail_mib) +
                    ": got " + std::to_string(got_mib) + " MiB class " + b.class_name +
                    ", expected " + std::to_string(c.expect_mib) + " MiB class " + c.cls);
            const char* why = b.bytes == 0 ? "class" : (b.available_limited ? "available" : "class");
            if (b.bytes != 0 && std::string(why) != c.why)
                throw std::runtime_error("capture budget limiter mismatch at total_mib=" +
                    std::to_string(c.total_mib));
        }
        // MemAvailable unknown (zero) must fall back to the class budget.
        if (capture_budget(7800ull * mib, 0ull).bytes != 6656ull * mib)
            throw std::runtime_error("capture budget must use the class budget when MemAvailable is unknown");
    }

    {
        const uint32_t cw=64u,ch=48u;
        const size_t stride=cw,plane=size_t(cw)*ch;
        std::vector<int16_t> coeff(plane*4u);
        for(size_t i=0;i<coeff.size();++i) coeff[i]=int16_t(int(i%181u)-90);
        // Force a small number of fallback tiles/bands without saturating int16.
        coeff[17]=500; coeff[plane+size_t(20)*stride+20]=-700;
        CpuBandHybridProvider band_provider(1,stride,plane);
        CpuTileHybridProvider tile_provider(1,cw,ch,stride,plane);
        band_provider.prepare(0,coeff.data());
        tile_provider.prepare(0,coeff.data());
        const auto bands=make_hybrid_bands(cw,ch);
        for(const auto& band:bands){
            const uint32_t li=band.band_id==0?0u:band.level-1u;
            const auto& bv=band_provider.frame(0).band[band.plane][li][band.band_id];
            const auto& tv=tile_provider.frame(0).band[band.plane][li][band.band_id];
            for(uint32_t y=0;y<band.height;++y)for(uint32_t x=0;x<band.width;++x){
                const int16_t expected=coeff[size_t(band.plane)*plane+size_t(band.y+y)*stride+band.x+x];
                if(hybrid_value(bv,x,y)!=expected)throw std::runtime_error("CPU band hybrid self-test mismatch");
                if(hybrid_value(tv,x,y)!=expected)throw std::runtime_error("CPU tile hybrid self-test mismatch");
            }
        }
        const auto& tm=tile_provider.metrics(0);
        if(tm.int8_tiles==0||tm.fallback_tiles==0||tm.total_tiles!=tm.int8_tiles+tm.fallback_tiles)
            throw std::runtime_error("CPU tile hybrid self-test did not exercise both formats");
    }

    std::cout << "Self-test PASS: capture budget classes, log LUT, Bayer split, VC5-444 (GPRAW) plane layout, "
                 "M1-M10 SDK quant tables, compact cacheable fallback snapshot, CPU band/tile hybrid, CPU transform schedule and CRC\n";
}

/* The library build (cinepi_qraw_encoder.cpp) includes this file to reuse the
   exact pipeline the correctness gates prove, and drops the benchmark's
   entry point. A copied-out second implementation would drift from the one
   that was measured. */
#ifndef CINEPI_NO_MAIN
int main(int argc,char**argv){
    std::ofstream logf;
    std::unique_ptr<TeeBuf> err_tee;
    std::unique_ptr<TeeBuf> out_tee;
    std::streambuf* old_err = nullptr;
    std::streambuf* old_out = nullptr;
    try{
        if(argc==2 && std::string(argv[1])=="--version"){std::cout<<"CINEPI_VC5_BENCH_VERSION "<<CINEPI_VERSION<<'\n';return 0;}
        Options o=parse_args(argc,argv);
        std::filesystem::path lp(o.log);if(lp.has_parent_path())std::filesystem::create_directories(lp.parent_path());std::filesystem::path cp(o.frame_log);if(cp.has_parent_path())std::filesystem::create_directories(cp.parent_path());
        logf.open(o.log,std::ios::out|std::ios::trunc);if(!logf)throw std::runtime_error("Cannot create log: "+o.log);
        old_err=std::cerr.rdbuf(); old_out=std::cout.rdbuf();
        err_tee=std::make_unique<TeeBuf>(old_err,logf.rdbuf()); out_tee=std::make_unique<TeeBuf>(old_out,logf.rdbuf());
        std::cerr.rdbuf(err_tee.get()); std::cout.rdbuf(out_tee.get());
        log_event("INFO","startup",std::string("CinePi VC-5 direct GPR benchmark v")+CINEPI_VERSION+"  / V51 handoff, split-level and GPU diagnostics");
        if (o.true_12bit) self_test_true_12bit_roundtrip(o.compand_bits);
        if(o.self_test){self_test();std::cerr.rdbuf(old_err);std::cout.rdbuf(old_out);return 0;}
        { std::ostringstream ex; ex<<"wide_separable="<<(o.wide_separable?1:0)<<" wide_rows_per_march="<<o.wide_rows_per_march<<" wide_hybrid_pack="<<(o.wide_hybrid_pack?1:0)<<" workgroup="<<o.workgroup<<" barrier_scope="<<o.barrier_scope<<" command_usage="<<o.command_usage<<" dispatch_order="<<o.dispatch_order<<" tiled_dispatch="<<(o.tiled_dispatch?1:0)<<" fused2d_dispatch="<<(o.fused2d_dispatch?1:0)<<" require_v3d="<<(o.require_v3d?1:0)<<" readback_mode="<<(o.readback_mode==Options::ReadbackMode::Auto?"auto":(o.readback_mode==Options::ReadbackMode::Copy?"cpu-copy":(o.readback_mode==Options::ReadbackMode::VulkanCopy?"vulkan-copy":"direct")))<<" hybrid_handoff="<<(o.hybrid_handoff==Options::HybridHandoff::Auto?"auto":(o.hybrid_handoff==Options::HybridHandoff::CacheableSnapshot?"snapshot":"direct"))<<" snapshot_jobs="<<o.snapshot_jobs<<" buffers="<<o.buffers<<" threads="<<o.threads<<" gpu_inflight="<<effective_gpu_inflight_limit(o,o.threads)<<" execution="<<o.execution<<" planes_in_z="<<(o.planes_in_z?1:0)<<" device_local_input="<<(o.device_local_input?1:0)<<" device_local_coeff="<<(o.device_local_coeff?1:0); log_event("INFO","experiment",ex.str()); std::cout<<"EXPERIMENT "<<ex.str()<<"\n"; }
        // Select the CPU wavelet kernel before any worker thread exists.
        g_cpu_wavelet_use_neon = (o.cpu_wavelet != "scalar") && (CINEPI_HAVE_NEON_WAVELET != 0);
        if (o.cpu_wavelet == "neon" && !CINEPI_HAVE_NEON_WAVELET)
            throw std::runtime_error("--cpu-wavelet neon requested but this build has no NEON");
        g_cpu_wavelet_kernel_v53 = (o.cpu_wavelet_kernel == "v53");
        g_cpu_wavelet_vec_blocks = o.cpu_wavelet_vec_blocks;
        g_cpu_nontemporal_bands = o.cpu_nontemporal && (CINEPI_HAVE_STNP != 0);
        g_cpu_wavelet_reuse_context = o.cpu_wavelet_reuse_context;
        g_cpu_fused_prefetch = o.cpu_gpr_prefetch;
        g_cpu_split_neon = o.cpu_split_neon && (CINEPI_HAVE_NEON_WAVELET != 0);
        cinepi_vle_prequant_skip = o.vle_prequant_skip ? 1 : 0;
        /* v0.27: the winner stack has EIGHT members. The v2 kernel carries
           stride_split, split16 and stnp_reg by construction, and the option
           parser sets vle_sidecar + sidecar_zskip -- but lowpass_bulk,
           vle_signlut and handoff_pool were declared extern here and never
           assigned, so the production binary had been running 5 of 8 while
           the m5 suite ran all 8. That gap is why suite and benchmark fps
           disagreed. Set them together, from one place. */
        {
            const auto pick = [](int override_, bool dflt) {
                return override_ < 0 ? dflt : (override_ != 0);
            };
            const bool w = o.cpu_winner;
            /* v1.0: lowpass_bulk left the winner -- the E-mode search
               tried it at every level and it never won. Still available
               explicitly via --win-lowpass-bulk on. */
            cinepi_lowpass_bulk = pick(o.win_lowpass_bulk, false) ? 1 : 0;
            cinepi_vle_signlut  = pick(o.win_vle_signlut,  w) ? 1 : 0;
/* v1.1.2: acc64 and scan8 are PREREQUISITES of the winner's
               entropy members, not independent options -- exactly as the
               m5 suite encodes them (its vle_sidecar spec sets acc64=1,
               and vle_signlut "rides on scan8"). Both were declared extern
               here and never assigned, so the production binary ran the
               entropy stage with the 64-bit accumulator and the mask
               scanner OFF while the suite ran both ON. That is the whole
               33.5 vs 39.7 fps gap: entropy 58.2 ms/frame against 34.9.
               acc64 is what actually gates the sidecar (encoder.c:2333),
               so wiring scan8 alone changed nothing -- measured 58.19 and
               58.40 ms on the CM5, unchanged. */
            /* v1.1.2: the sidecar nonzero-mask is written by the NEON
               register-direct emit. Without NEON it is never written, the
               mask reads all-zero, and the entropy coder drops every
               coefficient -- a 3,052,650 byte frame becomes 308,546 and
               still "succeeds". The m5 suite refuses vle_sidecar on
               non-NEON hosts for this reason; production had no guard and
               silently produced corrupt frames. Fail closed. */
            if (o.cpu_sidecar && !CINEPI_HAVE_NEON_WAVELET)
                throw std::runtime_error(
                    "the winner stack's sidecar needs the NEON "
                    "register-direct emit to build the nonzero mask; on "
                    "this host the mask would be empty and every "
                    "coefficient dropped. Use --cpu-winner off, or run "
                    "on aarch64.");
            cinepi_vle_acc64 = pick(o.win_vle_acc64, w) ? 1 : 0;
            cinepi_vle_scan8 = pick(o.win_vle_scan8, w) ? 1 : 0;
            cinepi_set_handoff_pool(pick(o.win_handoff_pool, w));
            g_cpu_splice_inplace = o.cpu_gpr_shared_inplace;
            cinepi_vle_prefetch_distance = o.cpu_vle_prefetch_distance;
            cinepi_vle_prefetch_locality = std::max(0, std::min(3, o.cpu_vle_prefetch_locality));
        }
        if (o.cpu_nontemporal && !CINEPI_HAVE_STNP)
            std::cout << "CPU_NONTEMPORAL status=unavailable reason=not-aarch64\n";
        if (o.cpu_wavelet_fused && !cpu_fused_geometry_ok(o))
            std::cout << "CPU_WAVELET_FUSED status=declined reason=plane-dims-not-divisible-by-8 "
                         "note=six-pass-schedule-over-reads-on-odd-level-heights\n";
        else
            std::cout << "CPU_WAVELET_FUSED status=" << (o.cpu_wavelet_fused ? "on" : "off")
                      << " split_fused=" << (o.cpu_split_fused ? "on" : "off") << "\n";
        std::cout << "CPU_WAVELET kernel=" << (g_cpu_wavelet_use_neon ? "neon" : "scalar")
                  << " variant=" << (g_cpu_wavelet_use_neon ? o.cpu_wavelet_kernel : std::string("n/a"))
                  << " vec_blocks=" << g_cpu_wavelet_vec_blocks
                  << " neon_available=" << (CINEPI_HAVE_NEON_WAVELET ? "YES" : "NO") << "\n";
        const bool host_only = (o.execution == "cpu-gpr" || o.execution == "capture");
        if (!host_only) o.shader=resolve_file(argv[0],o.shader);
        if (o.wide_separable && !host_only) {
            o.wide_h_shader=resolve_file(argv[0],o.wide_h_shader);
            o.wide_v_shader=resolve_file(argv[0],o.wide_v_shader);
            if (o.wide_hybrid_pack) o.wide_p_shader=resolve_file(argv[0],o.wide_p_shader);
        }
        o.gpr_params=resolve_file(argv[0],o.gpr_params);
        if (o.coeff_storage == Options::CoeffStorage::GpuHybridV15) {
            o.int8_range_shader=resolve_file(argv[0],o.int8_range_shader);
            o.int8_pack_shader=resolve_file(argv[0],o.int8_pack_shader);
        } else if (o.coeff_storage == Options::CoeffStorage::GpuHybridOnePass) {
            o.int8_onepass_shader=resolve_file(argv[0],o.int8_onepass_shader);
        } else if (o.coeff_storage == Options::CoeffStorage::GpuHybridTileDirect) {
            // Tile-direct always runs --tile-direct-shader. Silently discarding an
            // explicit --shader here previously let a caller pair the scan-linear
            // CPU reader with the fixed-slot shader: the GPU writes 8x8 tile slots
            // while the entropy scanner reads band row-scan addresses, which
            // produces a well-formed but wrong bitstream. --verify-gpu catches it,
            // but plain timing runs do not, so refuse the ambiguous combination.
            o.tile_direct_shader=resolve_file(argv[0],o.tile_direct_shader);
            if (o.shader_explicit && !o.tile_direct_shader_explicit)
                throw std::runtime_error(
                    "gpu-hybrid-tile-direct selects the shader with --tile-direct-shader; "
                    "--shader was given instead. Pass --tile-direct-shader " + o.shader +
                    " (a scan-linear shader is required by --tile-layout linear).");
            if (o.shader_explicit && o.tile_direct_shader_explicit &&
                resolve_file(argv[0], o.shader) != o.tile_direct_shader)
                throw std::runtime_error(
                    "--shader and --tile-direct-shader disagree for gpu-hybrid-tile-direct");
            o.shader=o.tile_direct_shader;
        }
        log_event("INFO","startup","reading input");const auto src=read_raw16(o);
        const int requested_capture_queue = o.capture_queue;
        if (o.execution == "strict-workers" || o.execution == "cpu-gpr" ||
            o.execution == "capture") {
            o.capture_queue = 0;
            std::ostringstream q;
            q << "execution=strict-workers queue=DISABLED local_raw_buffers=" << o.threads
              << " capture_fps=" << o.capture_fps;
            std::cout << "STRICT_WORKER_CAPTURE " << q.str() << "\n";
            log_event("INFO", "capture-buffer", q.str());
        } else {
            o.capture_queue = resolve_capture_queue_frames(o, src.size() * sizeof(uint16_t));
            const MemoryInfo mem = read_memory_info();
            std::ostringstream q;
            const CaptureBudget budget = capture_budget(mem.total_bytes, mem.available_bytes);
            q << "adaptive=" << (requested_capture_queue < 0 ? "YES" : "NO")
              << " memory_class=" << budget.class_name
              << " class_budget_mib=" << budget.class_budget_mib
              << " budget_mib=" << (double(budget.bytes) / 1048576.0)
              << " budget_limited_by=" << (budget.bytes == 0 ? "class"
                                          : (budget.available_limited ? "available" : "class"))
              << " os_allowance_mib=" << kCaptureOsAllowanceMib
              << " working_margin_mib=" << kCaptureWorkingMarginMib
              << " total_mib=" << (double(mem.total_bytes) / 1048576.0)
              << " available_mib=" << (double(mem.available_bytes) / 1048576.0)
              << " queue_frames=" << std::max(12, o.capture_queue)
              << " queue_mib=" << (double(std::max(12, o.capture_queue)) * double(src.size()) * 2.0 / 1048576.0)
              << " capture_fps=" << o.capture_fps;
            std::cout << "RAW_EMERGENCY_BUFFER " << q.str() << "\n";
            log_event("INFO", "capture-buffer", q.str());
        }
        log_event("INFO","startup","building LUT");const auto lut=make_lut(o);
        configure_cpu_hot_lut(o, lut);
        ModeSpec run_mode_storage; const ModeSpec* run_mode_ptr=nullptr;
        if(!o.mode.empty()){
            run_mode_storage=resolve_mode(o.mode,o.log_strength,o.working_max);
            if (o.compand_bits < 12 && o.compand_quant_scale) {
                const int sh = 12 - o.compand_bits;
                for (auto& q : run_mode_storage.quant_table)
                    q = std::max(1, (q + (1 << sh >> 1)) >> sh);
            }
            if (o.compand_inframe_bits < 12) {
                // In-frame precision: the container stays a full 12-bit GPR;
                // the curve is stepped (in make_lut) and the quant ladder
                // absorbs the same step. Identical to the m5 suite path.
                const int S = 1 << (12 - o.compand_inframe_bits);
                for (auto& q : run_mode_storage.quant_table)
                    q = std::min(32767, q * S);
            }
            /* EXPERIMENTAL per-subband quantiser override (wavelet band
               pruning test plan E1/E2/E3/E6). Same knob, same indexing as the
               live library face, so a candidate measured on the camera can be
               reproduced HERE on the fixed reference frame -- which is the
               only way to get a true per-pixel PSNR for it, because the static
               source does not drift between legs.

                 0     LL3 lowpass
                 1,2,3 level 3 LH/HL/HH    4,5,6 level 2 LH/HL/HH
                 7,8,9 level 1 LH/HL/HH    -- 9 is HH1

               Applied at the CALL SITE, not inside resolve_mode(), so the
               ladder self-test still validates the shipped tables.

               Unset = the shipped ladder, bit-identical output. That is what
               keeps the canonical re-pin honest: with no CINEPI_BAND_Q in the
               environment this block does nothing at all. */
            if (const char *bq = std::getenv("CINEPI_BAND_Q")) {
                std::string spec(bq);
                size_t pos = 0;
                while (pos < spec.size()) {
                    const size_t comma = spec.find(',', pos);
                    const std::string item = spec.substr(pos, comma - pos);
                    const size_t eq = item.find('=');
                    if (eq != std::string::npos) {
                        const int band = std::atoi(item.substr(0, eq).c_str());
                        const int qv   = std::atoi(item.substr(eq + 1).c_str());
                        if (band >= 0 && band < 10 && qv >= 1 && qv <= 32767)
                            run_mode_storage.quant_table[size_t(band)] = qv;
                    }
                    if (comma == std::string::npos) break;
                    pos = comma + 1;
                }
            }
            run_mode_ptr=&run_mode_storage;
            std::ostringstream mode_log;
            mode_log << o.mode;
            if (o.compand_bits < 12)
                mode_log << (o.compand_quant_scale ? " quant-scaled-to-" : " UNSCALED-at-")
                         << o.compand_bits << "-bit";
            mode_log << " SDK quant table=";
            for (size_t i = 0; i < run_mode_storage.quant_table.size(); ++i) {
                if (i) mode_log << '/';
                mode_log << run_mode_storage.quant_table[i];
            }
            log_event("INFO", "mode", mode_log.str());
        }
        // v1.7.55: the all-CPU GPR pipeline returns before any Vulkan object
        // exists. No instance, no device, no queue -- and on a shared-power SoC
        // that also returns the GPU's thermal budget to the A76 cluster.
        if (o.execution == "capture" || o.execution == "cpu-gpr") {
            const int rc = (o.execution == "capture")
                ? run_capture_simulation(o, run_mode_ptr, src, lut)
                : (o.core0_stage_pipeline != "off"
                    ? run_cpu_gpr_stage_pipeline(o, run_mode_ptr, src, lut)
                    : run_cpu_gpr_pipeline(o, run_mode_ptr, src, lut));
            // EVERY other exit path from main restores the stream buffers before
            // returning, because std::cout is pointing at a TeeBuf that owns a
            // reference to `logf`, and both are main's locals. Returning without
            // restoring leaves std::cout with a dangling rdbuf: main's locals are
            // destroyed, then static destruction flushes cout and segfaults, and
            // every buffered line written since the last flush is lost with it.
            // That is exactly what "stops after building LUT, no error" was.
            std::cout.flush();
            std::cerr.flush();
            if (old_err) std::cerr.rdbuf(old_err);
            if (old_out) std::cout.rdbuf(old_out);
            return rc;
        }

        log_event("INFO","startup","initialising Vulkan");VulkanPipeline vp;vp.init(o,run_mode_ptr,o.shader);
        if (!vp.host_cpu_transform) std::cout << "VULKAN_TRANSFORM backend=vulkan-compute-shader\n";
        { const bool resolved_snapshot = o.execution=="pipeline" && o.coeff_storage==Options::CoeffStorage::GpuHybridTileDirect && (o.hybrid_handoff==Options::HybridHandoff::CacheableSnapshot || (o.hybrid_handoff==Options::HybridHandoff::Auto && !vp.mapped_host_cached)); std::ostringstream cfg; cfg<<"shader="<<o.shader<<" wide_separable="<<(vp.wide_separable?"YES":"NO")<<" wide_rows_per_march="<<vp.wide_rows_per_march<<" wide_hybrid_pack="<<(vp.wide_hybrid_pack?"YES":"NO")<<" wide_wg="<<vp.wide_h_wg<<"/"<<vp.wide_v_wg<<"/"<<vp.wide_p_wg<<" tiled_dispatch="<<(vp.tiled_dispatch?"YES":"NO")<<" fused2d_dispatch="<<(vp.fused2d_dispatch?"YES":"NO")<<" dispatch_order="<<vp.dispatch_order<<" workgroup="<<vp.workgroup_size<<" buffers="<<o.buffers<<" gpu_inflight="<<effective_gpu_inflight_limit(o,o.threads)<<" planes_in_z="<<(vp.planes_in_z?"YES":"NO")<<" flags="<<(vp.tile_flags_u32?"u32":"bitmap")<<" readback="<<(vp.use_vulkan_readback_copy?"vulkan-cached-copy":(vp.use_readback_copy?"cpu-cached-copy":"direct-mapped"))<<" hybrid_handoff="<<(resolved_snapshot?"cacheable-snapshot":"direct-mapped")<<" device_local_input="<<(o.device_local_input?"YES":"NO")<<" device_local_coeff="<<(o.device_local_coeff?"YES":"NO")<<" input_mem_flags="<<memory_flags_name(vp.input_memory_flags)<<" coeff_mem_flags="<<memory_flags_name(vp.coeff_memory_flags); std::cout<<"VULKAN_CONFIG "<<cfg.str()<<"\n"; log_event("INFO","vulkan",cfg.str()); }
        if(o.verify_gpu && o.coeff_storage != Options::CoeffStorage::GpuHybridTileDirect)
            verify_gpu_transform(vp,o,src,lut);
        else if(o.verify_gpu && o.coeff_storage == Options::CoeffStorage::GpuHybridTileDirect)
            std::cout << "GPU_VERIFY deferred=tile-direct-GPR-and-reconstruction-gate\n";
        std::unique_ptr<GpuHybridPacker> hybrid_packer;
        std::unique_ptr<GpuOnePassHybridPacker> onepass_packer;
        std::unique_ptr<CpuBandHybridProvider> cpu_hybrid_provider;
        std::unique_ptr<CpuTileHybridProvider> cpu_tile_provider;
        std::unique_ptr<FusedMirrorHybridProvider> fused_provider;
        std::unique_ptr<TileDirectHybridProvider> tile_provider;
        if (o.coeff_storage == Options::CoeffStorage::CpuHybridBand) {
            cpu_hybrid_provider = std::make_unique<CpuBandHybridProvider>(
                vp.slots.size(), vp.row_stride, vp.plane_elems);
        } else if (o.coeff_storage == Options::CoeffStorage::CpuHybridTile) {
            cpu_tile_provider = std::make_unique<CpuTileHybridProvider>(
                vp.slots.size(), uint32_t(vp.row_stride),
                uint32_t(vp.plane_elems / vp.row_stride),
                vp.row_stride, vp.plane_stride_elems);
        } else if (o.coeff_storage == Options::CoeffStorage::GpuHybridV15) {
            hybrid_packer = std::make_unique<GpuHybridPacker>(vp, o);
            if (o.verify_gpu) verify_hybrid_gpu_packer(*hybrid_packer, vp);
        } else if (o.coeff_storage == Options::CoeffStorage::GpuHybridOnePass) {
            onepass_packer = std::make_unique<GpuOnePassHybridPacker>(vp, o);
        } else if (o.coeff_storage == Options::CoeffStorage::GpuHybridFusedMirror) {
            fused_provider = std::make_unique<FusedMirrorHybridProvider>(vp);
        } else if (o.coeff_storage == Options::CoeffStorage::GpuHybridTileDirect) {
            tile_provider = std::make_unique<TileDirectHybridProvider>(vp);
            if (o.verify_gpu)
                verify_tile_direct_gpu_transform(vp, *tile_provider, o, src, lut);
        }
        const int processing_worker_count = o.execution == "strict-workers"
            ? o.threads : std::max(1, std::min(o.threads, o.buffers));
        std::vector<std::unique_ptr<DirectGprEncoder>> direct_encoders;
        direct_encoders.reserve(size_t(processing_worker_count));
        for (int worker = 0; worker < processing_worker_count; ++worker)
            direct_encoders.emplace_back(std::make_unique<DirectGprEncoder>(
                o, run_mode_ptr, vp.row_stride, vp.plane_stride_elems));
        if (o.execution == "strict-workers") {
            log_event("INFO", "workers", "strict end-to-end processing workers=" +
                      std::to_string(processing_worker_count) + "; application service threads=0");
            const int effective_gpu_inflight = effective_gpu_inflight_limit(o, processing_worker_count);
            std::cout << "CINEPI_THREAD_MODEL execution=strict-workers processing_workers="
                      << processing_worker_count
                      << " application_service_threads=0 main_thread_role=worker-0"
                      << " gpu_inflight_limit=" << effective_gpu_inflight
                      << " driver_threads=external-uncontrolled\n";
        } else {
            log_event("INFO", "entropy", "pipeline whole-frame entropy workers=" +
                      std::to_string(processing_worker_count));
            std::cout << "CINEPI_THREAD_MODEL execution=pipeline entropy_workers="
                      << processing_worker_count
                      << " application_service_threads=4 gpu_inflight_limit="
                      << effective_gpu_inflight_limit(o, processing_worker_count)
                      << " driver_threads=external-uncontrolled\n";
        }
        auto selected_pass = [&](int frames, double seconds, bool collect_pass,
                                 std::ostream* frame_csv, gpr_buffer* retained) {
            if (o.execution == "strict-workers")
                return run_strict_workers(vp, hybrid_packer.get(), onepass_packer.get(), cpu_hybrid_provider.get(), cpu_tile_provider.get(), fused_provider.get(), tile_provider.get(), direct_encoders, run_mode_ptr, o, src, lut,
                                          frames, seconds, collect_pass, frame_csv, retained);
            return run_pass(vp, hybrid_packer.get(), onepass_packer.get(), cpu_hybrid_provider.get(), cpu_tile_provider.get(), fused_provider.get(), tile_provider.get(), direct_encoders,
                            run_mode_ptr, o, src, lut,
                            frames, seconds, collect_pass, frame_csv, retained);
        };
        if(o.warmup>0){log_event("INFO","warmup","frames="+std::to_string(o.warmup));(void)selected_pass(o.warmup,0,false,nullptr,nullptr);}
        std::ofstream csv(o.frame_log,std::ios::out|std::ios::trunc);
        if(!csv) throw std::runtime_error("Cannot create frame log");
        if (o.execution == "pipeline") {
            csv << "frame,slot,entropy_worker,latency_ms,split_ms,gpu_wall_ms,gpu_exec_ms,snapshot_ms,snapshot_bytes,gpu_slot_hold_ms,cpu_tail_ms,entropy_queue_wait_ms,int8_range_ms,int8_pack_ms,hybrid_coeff_bytes,int8_bands,int16_fallback_bands,int8_tiles,int16_fallback_tiles,total_tiles,vc5_entropy_container_ms,gpr_accounting_ms,vc5_bytes,gpr_bytes,crc32,crc_computed\n";
        } else {
            csv << "frame,slot,latency_ms,split_ms,gpu_wall_ms,gpu_exec_ms,snapshot_ms,snapshot_bytes,cpu_tail_ms,int8_range_ms,int8_pack_ms,hybrid_coeff_bytes,int8_bands,int16_fallback_bands,int8_tiles,int16_fallback_tiles,total_tiles,vc5_entropy_container_ms,gpr_accounting_ms,vc5_bytes,gpr_bytes,crc32,crc_computed,engine\n";
        }
        gpr_buffer final_gpr{nullptr,0};
        if(o.output!="none"&&!o.output.empty()){
            std::filesystem::path outp(o.output);
            if(lower(outp.extension().string())!=".gpr") throw std::runtime_error("--output must use the .GPR extension");
            if(outp.has_parent_path()) std::filesystem::create_directories(outp.parent_path());
        }
        log_event("INFO","run",o.codec+" duration="+std::to_string(o.duration)+" frames_fallback="+std::to_string(o.frames));Result r=selected_pass(o.frames,o.duration,true,&csv,(o.output!="none"&&!o.output.empty())?&final_gpr:nullptr);
        const LogicalTrafficModel traffic = logical_traffic_model(
            o, (vp.use_readback_copy || vp.use_vulkan_readback_copy), r.packet, r.avg_hybrid_bytes);
        if(o.output!="none"&&!o.output.empty()){
            if(!final_gpr.buffer||final_gpr.size==0) throw std::runtime_error("no final GPR frame was retained");
            if(write_to_file(&final_gpr,o.output.c_str())!=0) throw std::runtime_error("failed writing GPR output: "+o.output);
            log_event("INFO","gpr","wrote "+o.output+" bytes="+std::to_string(final_gpr.size));
        }
        std::cout<<std::fixed<<std::setprecision(3)
                 <<"Codec                   : "<<o.codec<<'\n'
                 <<"Quality mode              : "<<(o.mode.empty()?std::string("base-q")+std::to_string(o.quant):o.mode)<<'\n'
                 <<"Frames                  : "<<r.frames<<'\n'
                 <<"Submission window       : "<<r.submission_seconds<<" s\n"
                 <<"Total incl. drain       : "<<r.total_seconds<<" s\n"
                 <<"Pipeline throughput     : "<<r.fps<<" fps\n"
                 <<"Latency avg / p99        : "<<r.avg_latency<<" / "<<r.p99_latency<<" ms\n"
                 <<"Completion gap p99       : "<<r.p99_gap<<" ms\n"
                 <<((o.execution=="strict-workers")?"CPU RAW copy+compand/split: ":"CPU compand+split avg    : ")<<r.avg_split<<" ms\n"
                 <<"GPU queue wall avg       : "<<r.avg_gpu<<" ms\n"
                 <<"GPU execution avg        : "<<r.avg_gpu_exec<<" ms\n"
                 <<"Cacheable snapshot avg    : "<<r.avg_snapshot<<" ms\n"
                 <<"GPU slot hold avg          : "<<r.avg_gpu_slot_hold<<" ms\n"
                 <<"Snapshot bytes/frame       : "<<r.snapshot_bytes<<" bytes\n"
                 <<"Readback / handoff          : "<<readback_mode_name(vp)<<"\n"
                 <<"GPU int8 range avg        : "<<r.avg_int8_range<<" ms\n"
                 <<"GPU int8 pack avg         : "<<r.avg_int8_pack<<" ms\n"
                 <<"VC-5 entropy + GPR wrap   : "<<r.avg_entropy<<" ms\n"
                 <<"GPR CRC/accounting avg   : "<<r.avg_writer<<" ms\n"
                 <<"VC-5 bytes/frame         : "<<r.payload<<'\n'
                 <<"GPR bytes/frame          : "<<r.packet<<'\n'
                 <<"VC-5 compression ratio   : "<<r.payload_ratio<<":1\n"
                 <<"GPR compression ratio    : "<<r.packet_ratio<<":1\n"
                 <<"GPR output rate          : "<<r.output_mibs<<" MiB/s\n"
                 <<"Transform dispatches/frame: "<<dispatches_per_frame(o)<<'\n'
                 <<"Hybrid dispatches/frame   : "<<hybrid_extra_dispatches(o)<<'\n'
                 <<"Total GPU dispatches/frame: "<<(dispatches_per_frame(o)+hybrid_extra_dispatches(o))<<'\n'
                 <<"Modelled transform DRAM   : "<<modelled_dram_mb_per_frame(o)<<" MiB/frame\n"
                 <<"Logical traffic warning    : model only; not a physical LPDDR counter\n"
                 <<"CSI RAW16 write            : "<<traffic.csi_raw_write_mib<<" MiB/frame\n"
                 <<"CPU RAW16 read             : "<<traffic.cpu_raw_read_mib<<" MiB/frame\n"
                 <<"CPU component-plane write : "<<traffic.cpu_component_write_mib<<" MiB/frame\n"
                 <<"GPU input upload traffic    : "<<traffic.gpu_input_upload_mib<<" MiB/frame\n"
                 <<"GPU transform base         : "<<traffic.gpu_transform_base_mib<<" MiB/frame\n"
                 <<"Hybrid GPU-write adjustment: "<<traffic.hybrid_write_adjustment_mib<<" MiB/frame\n"
                 <<"GPU transform adjusted     : "<<traffic.gpu_transform_mib<<" MiB/frame\n"
                 <<"Readback-copy overhead     : "<<traffic.readback_copy_mib<<" MiB/frame\n"
                 <<"Entropy coefficient read   : "<<traffic.entropy_coefficient_read_mib<<" MiB/frame\n"
                 <<"GPR output write           : "<<traffic.gpr_output_mib<<" MiB/frame\n"
                 <<"Logical encoder excl. CSI  : "<<traffic.encoder_excluding_csi_mib<<" MiB/frame; "
                    <<traffic_gib_per_second(traffic.encoder_excluding_csi_mib,30.0)<<" GiB/s at 30 fps\n"
                 <<"Logical complete incl. CSI : "<<traffic.complete_including_csi_mib<<" MiB/frame; "
                    <<traffic_gib_per_second(traffic.complete_including_csi_mib,24.0)<<" GiB/s at 24 fps; "
                    <<traffic_gib_per_second(traffic.complete_including_csi_mib,30.0)<<" GiB/s at 30 fps\n"
                 <<"Coefficient storage       : "<<coeff_storage_name(o)<<"\n"
                 <<"Int16 coefficient bytes   : "<<coefficient_int16_bytes(o)<<" bytes/frame\n"
                 <<"Hybrid coefficient bytes  : "<<r.avg_hybrid_bytes<<" bytes/frame\n"
                 <<"Entropy-read reduction    : "<<(r.avg_hybrid_bytes?100.0*(1.0-double(r.avg_hybrid_bytes)/double(coefficient_int16_bytes(o))):0.0)<<" %\n"
                 <<"Int8 / fallback bands     : "<<r.avg_int8_bands<<" / "<<r.avg_int16_fallback_bands<<"\n"
                 <<"Int8 / fallback tiles     : "<<r.avg_int8_tiles<<" / "<<r.avg_int16_fallback_tiles<<" of "<<r.avg_total_tiles<<"\n"
                 <<"Entropy-read saving @ fps : "<<(r.avg_hybrid_bytes?(double(coefficient_int16_bytes(o)-r.avg_hybrid_bytes)*r.fps/1048576.0):0.0)<<" MiB/s\n"
                 <<"GPU-write saving @ fps    : "<<(o.coeff_storage==Options::CoeffStorage::GpuHybridTileDirect&&r.avg_hybrid_bytes?(double(coefficient_int16_bytes(o)-r.avg_hybrid_bytes)*r.fps/1048576.0):0.0)<<" MiB/s\n"
                 <<"Write+read saving @ fps   : "<<(o.coeff_storage==Options::CoeffStorage::GpuHybridTileDirect&&r.avg_hybrid_bytes?(2.0*double(coefficient_int16_bytes(o)-r.avg_hybrid_bytes)*r.fps/1048576.0):0.0)<<" MiB/s\n"
                 <<"Total RAW capture capacity   : "<<r.capture_queue_capacity<<" frames ("<<(double(r.capture_queue_capacity)*double(o.width)*double(o.height)*2.0/1048576.0)<<" MiB RAW16)\n"
                 <<"Normal capture pipeline       : "<<(o.execution=="strict-workers"?0:12)<<" frames\n"
                 <<"Capture Protection ceiling    : "<<(r.capture_queue_capacity > 12u ? r.capture_queue_capacity-12u : 0u)<<" overflow frames\n"
                 <<"Capture pacing                : "<<(o.capture_fps > 0.0 ? std::to_string(o.capture_fps)+" fps" : (o.execution=="strict-workers"?std::string("uncapped strict-worker mode"):std::string("uncapped throughput mode")))<<"\n"
                 <<"RAW waiting-queue high-water  : "<<r.capture_queue_high_water<<" frames ("<<(r.capture_queue_capacity?100.0*double(r.capture_queue_high_water)/double(r.capture_queue_capacity):0.0)<<"%)\n"
                 <<"Protection usage high-water   : "<<r.capture_protection_high_water<<" overflow frames\n"
                 <<"RAW queued / dequeued        : "<<r.capture_frames_enqueued<<" / "<<r.capture_frames_dequeued<<" frames\n"
                 <<"Dynamic protection slots used: "<<r.capture_dynamic_allocated<<" frames\n"
                 <<"Longest RAW queue wait       : "<<r.capture_longest_wait_ms<<" ms\n"
                 <<"Maximum encoder deficit      : "<<r.capture_max_encoder_deficit<<" frames\n"
                 <<"Frames pending at record stop: "<<r.capture_pending_at_stop<<" frames\n"
                 <<"Post-record encode drain     : "<<r.capture_drain_seconds<<" s\n"
                 <<"Capture hard-limit waits     : "<<r.capture_backpressure_events<<'\n'
                 <<"Process RSS / peak RSS       : "<<(double(r.process_rss_bytes)/1048576.0)<<" / "<<(double(r.process_peak_rss_bytes)/1048576.0)<<" MiB\n"
                 <<"Protection if encoder stops  : "<<(double(r.capture_queue_capacity)/24.0)<<" s at 24 fps; "<<(double(r.capture_queue_capacity)/30.0)<<" s at 30 fps\n"
                 <<"Protection at 1/2/4 fps lag  : "<<double(r.capture_queue_capacity)<<" / "<<(double(r.capture_queue_capacity)/2.0)<<" / "<<(double(r.capture_queue_capacity)/4.0)<<" s\n"
                 <<"Pipeline scope            : GPR component split + "<<(o.fused2d_dispatch?"Vulkan fused 2-D 2/6":"Vulkan two-pass 2/6")<<(o.coeff_storage==Options::CoeffStorage::GpuHybridTileDirect?" + direct tile int8/int16 output + tiled entropy":(o.coeff_storage!=Options::CoeffStorage::Int16?" + GPU hybrid int8/int16 entropy":" + int16 entropy"))<<" + GPR DNG container\n"
                 <<"Execution model            : "<<o.execution<<"\n"
                 <<(o.execution=="strict-workers"?"CinePi processing workers  : ":"Entropy frame workers      : ")<<processing_worker_count<<"\n"
                 <<"Application service threads: "<<(o.execution=="strict-workers"?0:4)<<"\n"
                 <<"Driver/runtime threads     : external to benchmark worker count\n"
                 <<"Ordered output commit       : YES\n"
                 <<"Ordered reorder high-water : "<<r.reorder_high_water<<" completed frames\n"
                 <<"Out-of-order entropy finish: "<<r.out_of_order_entropy_finishes<<" frames\n"
                 <<"Duplicate SDK encode      : NO\n"
                 <<"File output               : "<<(o.output=="none"?std::string("none"):o.output)<<"\n";
        if (o.execution == "pipeline") {
            for (size_t worker = 0; worker < r.worker_frames.size(); ++worker) {
                std::cout << "Entropy worker " << worker
                          << " frames/avg_ms/queue_ms/util: "
                          << r.worker_frames[worker] << " / "
                          << r.worker_avg_entropy_ms[worker] << " / "
                          << r.worker_avg_queue_wait_ms[worker] << " / "
                          << r.worker_utilisation_pct[worker] << "%\n";
                std::cout << "ENTROPY_WORKER id=" << worker
                          << " frames=" << r.worker_frames[worker]
                          << " avg_entropy_ms=" << r.worker_avg_entropy_ms[worker]
                          << " avg_queue_wait_ms=" << r.worker_avg_queue_wait_ms[worker]
                          << " utilisation_pct=" << r.worker_utilisation_pct[worker] << '\n';
            }
        }
        vp.report_trace();
        if (o.cpu_wavelet_workers > 0) {
            std::cout << "DUAL_ENGINE cpu_wavelet_workers=" << r.dual_cpu_workers
                      << " gpu_workers=" << (o.threads - r.dual_cpu_workers)
                      << " gpu_frames=" << r.gpu_frames
                      << " cpu_frames=" << r.cpu_frames
                      << " gpu_fps=" << r.gpu_fps
                      << " cpu_fps=" << r.cpu_fps
                      << " aggregate_fps=" << r.fps
                      << " gpu_latency_ms=" << r.gpu_avg_latency
                      << " cpu_latency_ms=" << r.cpu_avg_latency
                      << " cpu_wavelet_kernel=" << o.cpu_wavelet_kernel
                      << " note=shares-one-frame-counter-and-one-memory-controller\n";
        }
        std::cout<<"RESULT family=vc5 codec="<<o.codec<<" mode="<<(o.mode.empty()?std::string("base-q")+std::to_string(o.quant):o.mode)<<" scope=benchmark-pipeline frames="<<r.frames<<" measured_s="<<r.submission_seconds<<" fps="<<r.fps<<" latency_ms="<<r.avg_latency<<" p99_gap_ms="<<r.p99_gap<<" split_ms="<<r.avg_split<<" gpu_ms="<<r.avg_gpu<<" gpu_exec_ms="<<r.avg_gpu_exec<<" snapshot_ms="<<r.avg_snapshot<<" gpu_slot_hold_ms="<<r.avg_gpu_slot_hold<<" snapshot_bytes="<<r.snapshot_bytes<<" cpu_tail_ms="<<r.avg_cpu_tail<<" gpu_levels="<<o.gpu_levels<<" hybrid_handoff="<<(r.cacheable_hybrid_handoff?"cacheable-snapshot":readback_mode_name(vp))<<" readback_mode="<<readback_mode_name(vp)<<" device_local_input="<<(o.device_local_input?"YES":"NO")<<" device_local_coeff="<<(o.device_local_coeff?"YES":"NO")<<" input_mem_flags="<<memory_flags_name(vp.input_memory_flags)<<" coeff_mem_flags="<<memory_flags_name(vp.coeff_memory_flags)<<" int8_range_ms="<<r.avg_int8_range<<" int8_pack_ms="<<r.avg_int8_pack<<" coeff_storage="<<coeff_storage_name(o)<<" int16_coeff_bytes="<<coefficient_int16_bytes(o)<<" hybrid_coeff_bytes="<<r.avg_hybrid_bytes<<" int8_bands="<<r.avg_int8_bands<<" int16_fallback_bands="<<r.avg_int16_fallback_bands<<" int8_tiles="<<r.avg_int8_tiles<<" int16_fallback_tiles="<<r.avg_int16_fallback_tiles<<" total_tiles="<<r.avg_total_tiles<<" entropy_read_reduction_pct="<<(r.avg_hybrid_bytes?100.0*(1.0-double(r.avg_hybrid_bytes)/double(coefficient_int16_bytes(o))):0.0)<<" entropy_read_saving_mibs="<<(r.avg_hybrid_bytes?(double(coefficient_int16_bytes(o)-r.avg_hybrid_bytes)*r.fps/1048576.0):0.0)<<" coeff_write_saving_mibs="<<(o.coeff_storage==Options::CoeffStorage::GpuHybridTileDirect&&r.avg_hybrid_bytes?(double(coefficient_int16_bytes(o)-r.avg_hybrid_bytes)*r.fps/1048576.0):0.0)<<" coeff_write_read_saving_mibs="<<(o.coeff_storage==Options::CoeffStorage::GpuHybridTileDirect&&r.avg_hybrid_bytes?(2.0*double(coefficient_int16_bytes(o)-r.avg_hybrid_bytes)*r.fps/1048576.0):0.0)<<" entropy_ms="<<r.avg_entropy<<" writer_ms="<<r.avg_writer<<" payload_bytes="<<r.payload<<" packet_bytes="<<r.packet<<" payload_ratio="<<r.payload_ratio<<" packet_ratio="<<r.packet_ratio<<" ratio="<<r.packet_ratio<<" output_mibs="<<r.output_mibs<<" submission_s="<<r.submission_seconds<<" total_s="<<r.total_seconds<<" transform_dispatches="<<dispatches_per_frame(o)<<" hybrid_dispatches="<<hybrid_extra_dispatches(o)<<" dispatches="<<(dispatches_per_frame(o)+hybrid_extra_dispatches(o))<<" planes_in_z="<<(o.planes_in_z?"YES":"NO")<<" wide_separable="<<(o.wide_separable?"YES":"NO")<<" wide_rows_per_march="<<o.wide_rows_per_march<<" wide_hybrid_pack="<<(o.wide_hybrid_pack?"YES":"NO")<<" tile_flag_layout="<<(o.tile_flag_layout==Options::TileFlagLayout::U32PerTile?"u32":"bitmap")<<" dram_mib_frame="<<modelled_dram_mb_per_frame(o)
                 <<" logical_csi_write_mib_frame="<<traffic.csi_raw_write_mib
                 <<" logical_cpu_raw_read_mib_frame="<<traffic.cpu_raw_read_mib
                 <<" logical_component_write_mib_frame="<<traffic.cpu_component_write_mib
                 <<" logical_gpu_transform_base_mib_frame="<<traffic.gpu_transform_base_mib
                 <<" logical_hybrid_write_adjustment_mib_frame="<<traffic.hybrid_write_adjustment_mib
                 <<" logical_gpu_transform_mib_frame="<<traffic.gpu_transform_mib
                 <<" logical_readback_copy_mib_frame="<<traffic.readback_copy_mib
                 <<" logical_entropy_read_mib_frame="<<traffic.entropy_coefficient_read_mib
                 <<" logical_gpr_output_mib_frame="<<traffic.gpr_output_mib
                 <<" logical_encoder_excl_csi_mib_frame="<<traffic.encoder_excluding_csi_mib
                 <<" logical_complete_incl_csi_mib_frame="<<traffic.complete_including_csi_mib
                 <<" logical_complete_gib_s_24="<<traffic_gib_per_second(traffic.complete_including_csi_mib,24.0)
                 <<" logical_complete_gib_s_30="<<traffic_gib_per_second(traffic.complete_including_csi_mib,30.0)
                 <<" logical_traffic_is_model=YES readback_policy="<<(vp.use_readback_copy?"cached-copy":"direct-mapped")
                 <<" capture_queue_frames="<<r.capture_queue_capacity<<" capture_queue_mib="<<(double(r.capture_queue_capacity)*double(o.width)*double(o.height)*2.0/1048576.0)<<" capture_fps="<<o.capture_fps<<" capture_queue_high_water_frames="<<r.capture_queue_high_water<<" capture_queue_high_water_pct="<<(r.capture_queue_capacity?100.0*double(r.capture_queue_high_water)/double(r.capture_queue_capacity):0.0)<<" capture_protection_high_water_frames="<<r.capture_protection_high_water<<" capture_enqueued="<<r.capture_frames_enqueued<<" capture_dequeued="<<r.capture_frames_dequeued<<" capture_backpressure_events="<<r.capture_backpressure_events<<" capture_dynamic_allocated_frames="<<r.capture_dynamic_allocated<<" capture_pending_at_stop="<<r.capture_pending_at_stop<<" capture_drain_s="<<r.capture_drain_seconds
                 <<" capture_longest_wait_ms="<<r.capture_longest_wait_ms
                 <<" capture_max_encoder_deficit_frames="<<r.capture_max_encoder_deficit
                 <<" process_rss_mib="<<(double(r.process_rss_bytes)/1048576.0)<<" process_peak_rss_mib="<<(double(r.process_peak_rss_bytes)/1048576.0)
                 <<" first_frame_payload_crc32="<<r.first_frame_crc
                 <<" first_frame_gpr_crc32="<<r.first_frame_crc
                 <<" first_frame_integrity_crc_computed="<<(r.first_frame_crc_computed?"YES":"NO")
                 <<" reorder_high_water_frames="<<r.reorder_high_water
                 <<" out_of_order_entropy_finishes="<<r.out_of_order_entropy_finishes
                 <<" execution="<<o.execution
                 <<" processing_workers="<<processing_worker_count
                 <<" app_service_threads="<<(o.execution=="strict-workers"?0:4)
                 <<" gpu_inflight="<<effective_gpu_inflight_limit(o, processing_worker_count)<<'\n';
        direct_encoders[0]->free_buffer(final_gpr);
        log_event("INFO","complete","frame_log="+o.frame_log);
        std::cerr.rdbuf(old_err);std::cout.rdbuf(old_out);return 0;
    }catch(const std::exception&e){
        if (old_err && old_out) {
            log_event("FATAL", "main", e.what());
        } else if (logf) {
            logf << now_local() << " [FATAL] [main] " << e.what() << '\n';
            logf.flush();
        }
        if(old_err) std::cerr.rdbuf(old_err);
        if(old_out) std::cout.rdbuf(old_out);
        std::cerr<<"FATAL: "<<e.what()<<'\n';return 1;
    }
}
#endif /* CINEPI_NO_MAIN */
