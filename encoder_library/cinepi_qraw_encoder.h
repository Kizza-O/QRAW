/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2026 Kieran Olsson
 */
/* ===========================================================================
 * cinepi_qraw_encoder.h -- the CinePi GPR encoder as a library.
 *
 * Everything the benchmark measures, behind an interface a capture pipeline
 * can call: hand it a frame of 16-bit Bayer, get back a finished GPR/DNG
 * container. No files, no argv, no benchmark harness.
 *
 * THREADING. One encoder object is single-threaded and owns roughly 20 MB of
 * scratch. Run one per worker thread; they share nothing except a
 * process-wide mutex inside the Adobe DNG container writer, which the splice
 * avoids after its first few frames.
 *
 * HOW MANY. At RAW12 the measured operating point is FOUR encoders, one per
 * core: 45.5 fps against 40.5 for three, at 3840x2160 with 60 requested, and
 * fewer drops with it. The camera thread costs 6.8% of a core at RAW12 because
 * there is no byte swap to do, so Core 0 has the headroom. Pair it with a
 * zero-copy inflight limit of 8 -- with four owners each holding a camera
 * buffer for a whole encode, the default 6 leaves only two frames able to queue
 * and the gate becomes the thing being measured.
 *
 * At RAW16 (ClearHDR) it is THREE. libcamera byte-swaps 16.8 MB per frame on
 * its camera thread there, roughly 10 ms of a 33 ms budget, and a fourth encode
 * worker on Core 0 starves capture.
 *
 * TYPICAL USE, one encoder per worker thread:
 *
 *     CinepiQrawConfig cfg;
 *     cinepi_qraw_config_defaults(&cfg);
 *     cfg.width = 3840; cfg.height = 2160;
 *     cfg.bayer = CINEPI_BAYER_GBRG;
 *     cfg.mode  = "m5";
 *     CinepiQrawEncoder *enc = cinepi_qraw_create(&cfg, NULL);
 *
 *     // per frame, from the camera callback or a queue behind it
 *     void  *gpr = NULL; size_t qraw_size = 0;
 *     if (cinepi_qraw_encode(enc, raw16, &gpr, &qraw_size) == 0) {
 *         write(fd, gpr, qraw_size);
 *         cinepi_qraw_release(enc, gpr);
 *     }
 *
 *     cinepi_qraw_destroy(enc);
 *
 * THE INPUT. cfg.width x cfg.height 16-bit Bayer, tightly packed, one sample
 * per uint16_t, MSB-justified -- which is exactly what a Pi 5 hands you when
 * you ask for an unpacked format. Do NOT pass PISP_COMP1 buffers: they are
 * block-delta coded, not sensor values. Both dimensions must be even, and for
 * the fused path both halves must be divisible by 8; the encoder falls back to
 * a slower schedule otherwise and says so through the log callback.
 *
 * THE OUTPUT. A complete GPR container. It stays valid until you call
 * cinepi_qraw_release() with it, which you must do or you will leak several MB
 * per frame.
 * ========================================================================= */
#ifndef CINEPI_QRAW_ENCODER_H
#define CINEPI_QRAW_ENCODER_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CINEPI_BAYER_GBRG = 0,
    CINEPI_BAYER_RGGB = 1
} CinepiBayerOrder;

typedef enum {
    CINEPI_LOG_INFO = 0,
    CINEPI_LOG_WARN = 1,
    CINEPI_LOG_ERROR = 2
} CinepiLogLevel;

/* Optional. Called from whichever thread hit the condition, so make it
 * thread-safe or make it cheap. */
typedef void (*CinepiLogFn)(CinepiLogLevel level, const char *message,
                            void *user);

typedef struct {
    int   width;                /* frame width in sensor samples            */
    int   height;               /* frame height                             */
    CinepiBayerOrder bayer;
    const char *mode;           /* "m1".."m10"; NULL means base quant       */
    int   quant;                /* used only when mode is NULL              */

    /* Component-Aware Quantisation profile: 0 = off, 1 = soft, 2 = medium,
     * 3 = strong. A RELATIVE modifier on whichever quant table `mode` resolves
     * to -- it spends fewer bits on the colour-residual components (RG/BG) and,
     * more gently, on GD, while leaving GS untouched. The output remains ordinary
     * GPR: the four standard components keep their meanings and dimensions, and
     * each channel advertises in the VC-5 stream exactly the divisor it was
     * quantised by, so a stock decoder needs no special knowledge.
     *
     * 0 is bit-exact current behaviour. Also settable with CINEPI_CAQ. */
    int   caq;

    /* --- Pixel Clean -------------------------------------------------------
     * A dead-zone at 125% of the normal half-step, plus a post-quant rule that
     * forces quantised RG/BG LEVEL-1 coefficients of +-1 to zero. An
     * independent lever: combinable with CAQ and with band pruning (HH1
     * zeroing), and orthogonal to both.
     *
     * Implemented as ONE exact integer threshold per band, resolved at encoder
     * setup -- the two rules collapse into `|coeff| <= T` because the exact
     * quantiser is monotone in |coeff|. The widened threshold also feeds the
     * existing provable-zero compute skips, so the lever can make the encoder
     * cheaper as well as the file smaller.
     *
     * Output stays ordinary GPR: no component, dimension, wavelet-topology or
     * entropy-syntax change, and every band still advertises the divisor it was
     * quantised by, so a stock VC-5 decoder needs no special knowledge.
     *
     * 0 is bit-exact current behaviour. Also settable with CINEPI_PIXEL_CLEAN. */
    int   pixel_clean;

    /* --- Noise Clean (revised, production-shaped) --------------------------
     * 0 off, 1 soft, 2 medium, 3 strong, plus a strength scalar (0.50..1.50) for
     * refining a profile without redefining it.
     *
     * NOT a denoise pass. It widens the encoder's own "this coefficient becomes
     * zero" threshold on selected level-1 bands, in units of quantised levels,
     * and changes nothing else: no second pass, no per-frame statistic, no state.
     * The zero happens inside the quantiser, so every non-zero mask and sidecar
     * is built from the widened result by construction, and the wider threshold
     * feeds the provable-zero compute skips -- which is why it is smaller AND
     * slightly faster rather than a trade. Output is ordinary GPR, with no
     * decoder change of any kind.
     *
     * Medium is the shipped default for QRAW and QRAW STABLE: measured -3.5 to
     * -4.4% container size with a small fps gain, at 50.9 dB against the OFF
     * decode. Strong also prunes GS, the plane the picture's sharpness lives in,
     * so it is offered rather than defaulted.
     *
     * Also settable with CINEPI_NOISE_CLEAN=off|soft|medium|strong and
     * CINEPI_NOISE_CLEAN_STRENGTH. */
    int    noise_clean_mode;
    double noise_clean_strength;

    /* --- Noise Clean, the ISO-calibrated prototype (LEGACY) ----------------
     * Superseded by noise_clean_mode above and reachable only when a run sets
     * CINEPI_NOISE_CLEAN_ISO=1, so the two cannot both threshold one frame.
     * Kept because the per-gain measurement behind it, and its self-test, are
     * real work: a calibrated floor beats an estimated one WHERE it exists.
     *
     * An ISO-CALIBRATED dead-zone in pre-quant wavelet coefficient units: a
     * coefficient is zeroed when its magnitude is statistically inside the noise
     * this sensor produces at the active gain. Independent of Pixel Clean and of
     * band pruning.
     *
     * `iso` is REQUIRED and must be an ISO the profile was measured at (100,
     * 200, 400, 800, 1600, 3200 for the IMX585 RAW12 stacks). There is no
     * interpolation on purpose: a noise threshold guessed between measured gains
     * is a threshold nobody validated. An unknown ISO makes the lever a no-op
     * rather than an approximation.
     *
     * `noise_clean_k` scales the threshold against the measured sigma. Small.
     * The log curve amplifies shadow SIGNAL by the same 34-54x it amplifies
     * shadow noise, so noise and real detail are comparable there -- which is
     * why they are hard to separate, and why this removes shadow content as well
     * as noise. Default 0.5, and the result must be looked at, not assumed.
     *
     * 0 is bit-exact current behaviour. Also settable with CINEPI_NOISE_CLEAN
     * and CINEPI_NOISE_CLEAN_K. */
    int    noise_clean;
    double noise_clean_k;
    int    iso;

    /* --- v1.16.6 WINNING STACK (IMMUTABLE IN r22). -------------------
     * These fields remain in the ABI for source compatibility, but the
     * production library ignores attempts to disable them. There is one
     * encoder implementation only. -------------------------------------- */
    int   fused_wavelet;        /* 1: fused 3-level cascade      (1.64x)    */
    int   fused_split;          /* 1: split folded in            (+7.2%)    */
    int   neon_split;           /* 1: NEON split arithmetic      (+3.8%)    */
    int   nontemporal;          /* 1: STNP band writes           (+4.3%)    */
    int   dng_splice;           /* 1: retained container template (+12.9%)  */
    int   splice_shared;        /* 1: one template for the process          */

    /* --- image pipeline ------------------------------------------------- */
    int   effective_bits;       /* 16 for a Pi 5 unpacked stream            */
    int   white;                /* sensor ceiling; 0 = (1<<effective_bits)-1.
                                 * NOTE: before the v0.1 integration fix the
                                 * implementation ignored the 0 contract and
                                 * left the internal default of 4095, which is
                                 * the settled cause of the +278,080 byte
                                 * library/benchmark discrepancy.             */
    int   black;                /* sensor black level in source code values
                                 * (16-bit scale on a Pi 5 unpacked stream,
                                 * e.g. 4096 for a typical IMX585 pedestal).
                                 * Subtracted inside the GP-Log2 LUT. 0 = no
                                 * pedestal, which is correct for the bundled
                                 * sample but NOT for real sensor frames.     */
    int   compand_bits;         /* 12, 11 or 10 working precision           */
    int   true_12bit;           /* 1: unhalved chroma, needs a matching
                                 *    inverse on the decode side            */
    double log_strength;        /* GP-Log2 curve strength, 0 = default      */
    int   gradation_compand;    /* 1: input is IMX585 ClearHDR gradation-
                                 * companded 12-bit. Decode that curve and
                                 * re-encode as GP-Log2, so the container holds
                                 * the transfer function the GPR format expects.
                                 * Storing gradation codes raw makes any reader
                                 * treat them as linear -- highlights collapse
                                 * and colour skews. Mutually exclusive with
                                 * applying GP-Log2 to the codes directly.  */
    const char *gpr_params_json;/* container metadata; NULL = built in      */

    /* --- v3.10: strided input ------------------------------------------- *
     * By default the encoder wants width*height samples tightly packed, so
     * a camera whose DMA rows are padded has to memcpy every frame first.
     * At 3856x2180 that is 16.8 MB per frame -- roughly 14% of the per-frame
     * budget -- purely to remove padding the cascade can skip over.
     *
     * Set src_stride_elems to the SAMPLES per row of the buffer you own and
     * the encoder reads it where it lies. src_crop_x/y select the active
     * window inside it. 0 keeps the old tightly-packed contract, so existing
     * callers are unaffected.
     *
     *     cfg.width = 3840; cfg.height = 2160;    // active window
     *     cfg.src_stride_elems = 7744 / 2;        // DMA row pitch, samples
     *     cfg.src_crop_x = 8; cfg.src_crop_y = 10;
     *
     * NOTE the encoder then reads the caller's buffer directly during
     * cinepi_qraw_encode(), so it must stay valid and unmodified for the
     * duration of that call. If the buffer is camera DMA that will be
     * recycled asynchronously, keep the copy.                              */
    size_t src_stride_elems;    /* samples per row; 0 = tightly packed      */
    int    src_crop_x;          /* active window origin inside that buffer  */
    int    src_crop_y;

    /* --- v3.13: source sample shift -------------------------------------- *
     * Right-shift applied IN-REGISTER to every source sample as it is
     * loaded, before the companding LUT is indexed. An IMX585 RAW12 stream
     * on a Pi 5 arrives MSB-justified in 16-bit words (the low 4 bits are
     * exactly zero), so until now RAW12 needed a staging pass that shifted
     * every sample >>4 into a tight buffer -- 16.8 MB read + 16.6 MB write
     * per frame -- before the encoder could use a 12-bit LUT. With
     *
     *     cfg.effective_bits   = 12;
     *     cfg.src_shift        = 4;
     *     cfg.src_stride_elems = 7744 / 2;      // and the crop, as above
     *
     * the encoder reads the DMA buffer where it lies and indexes the same
     * 4096-entry LUT; the shift is one register op folded into a gather
     * chain that is already load-latency bound, so it is effectively free.
     *
     * SCALES. cfg.black and cfg.white are on the POST-shift scale -- the
     * value that actually indexes the LUT, i.e. the sensor's native code
     * scale. For a RAW12 IMX585 with a 256-code pedestal that is
     * black = 256, white = 4095 (or 0 for the effective_bits default) --
     * NOT the 4096/65535 you would pass for the same sensor delivered as an
     * unpacked 16-bit stream with src_shift = 0. The compand curve is then
     * bit-identical between the two deliveries.
     *
     * The caller guarantees (sample >> src_shift) < 2^effective_bits, which
     * for an MSB-justified stream holds by construction whenever
     * src_shift >= 16 - effective_bits. 0 keeps today's behaviour. Like the
     * stride, the shift is honoured by the v2 load path, which is the one
     * this library always runs.                                            */
    int    src_shift;

    /* --- v3.14: source sample byte-swap ----------------------------------- *
     * Byte-swap applied IN-REGISTER to every source sample as it is loaded,
     * BEFORE src_shift and before the companding LUT is indexed (order:
     * load -> byteswap -> shift -> LUT index).
     *
     * The IMX585 emits big-endian RAW16 and the Pi 5's RP1 has no hardware
     * 16-bit swab, so libcamera byte-swaps the whole 16.81 MB frame on the
     * camera thread every frame. With that software swap skipped
     * (LIBCAMERA_RPI_SKIP_16BIT_ENDIAN_SWAP=1) the DMA buffer holds
     * big-endian samples; set
     *
     *     cfg.src_byteswap = 1;
     *
     * and the encoder swaps each sample in a register inside the LUT gather
     * chain, which is already load-latency bound, so the swap is effectively
     * free -- and the frame-wide staging swap (2 x 16.81 MB of DRAM traffic
     * per frame plus ~6.4 ms/frame of camera-thread CPU) disappears.
     *
     * cfg.black and cfg.white stay on the POST-swap (true sample) scale.
     * DO NOT set this while libcamera still performs its own swap: the
     * samples would be swapped twice. Like the stride and the shift it is
     * honoured by the v2 load path, which this library always runs.
     * 0 keeps today's behaviour.                                           */
    int    src_byteswap;
} CinepiQrawConfig;

/* Fills cfg with the measured operating point. Always call this first: it is
 * how new fields acquire sane values when you rebuild against a later header
 * without editing your call site. */
void cinepi_qraw_config_defaults(CinepiQrawConfig *cfg);

typedef struct CinepiQrawEncoder CinepiQrawEncoder;

/* Returns NULL on failure. log may be NULL. */
CinepiQrawEncoder *cinepi_qraw_create(const CinepiQrawConfig *cfg,
                                    CinepiLogFn log, void *log_user);

/* Encodes one frame. Returns 0 on success, negative on failure.
 * On success *out_buffer / *out_size describe a complete GPR container owned
 * by the encoder until released. Not reentrant: one call at a time per
 * encoder. */
int cinepi_qraw_encode(CinepiQrawEncoder *enc, const uint16_t *raw16,
                      void **out_buffer, size_t *out_size);

/* Hands a buffer from cinepi_qraw_encode back. */
void cinepi_qraw_release(CinepiQrawEncoder *enc, void *buffer);

/* Per-frame timings from the most recent encode, in milliseconds. Any pointer
 * may be NULL. dng_wrap is included in entropy and is the part that is
 * serialised across the whole process. */
void cinepi_qraw_last_timings(const CinepiQrawEncoder *enc,
                             double *wavelet_ms, double *entropy_ms,
                             double *dng_wrap_ms);

void cinepi_qraw_destroy(CinepiQrawEncoder *enc);

/* "1.11.2" etc. Matches the benchmark that measured this configuration. */
const char *cinepi_qraw_version(void);

#ifdef __cplusplus
}
#endif
#endif /* CINEPI_QRAW_ENCODER_H */
