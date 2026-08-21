/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2026 Kieran Olsson
 */
// verify_crc.cpp -- the regression gate, standalone.
//
// The bit-exactness gate used to live inside the candidate-search suite. That
// suite is scaffolding and does not belong in a release, but the gate does:
// it is the one thing that says the optimised encoder still produces byte-
// identical output. So it moves here, depending on nothing but the encoder.
//
// It encodes the bundled sample several times through the real pipeline and
// CRC32s the LAST container. Several, not one, because the shared splice
// template needs a few frames to learn and verify before the in-place path
// activates -- a single-frame gate would only ever exercise the learn path.
// Identical input gives identical output every frame, so the last-frame CRC
// equals the first-frame CRC when everything is correct, and differs
// precisely when a steady-state path is broken.
//
//   bash tools/verify.sh                 # build and check
//   ./verify_crc --input F --expect 455e725e
//
// Exit 0 on match, 1 on mismatch, 2 on a usage or I/O problem.
#define CINEPI_NO_MAIN 1
// cinepi_qraw_encoder.cpp includes vc5_bench.cpp itself, so including both
// defines every static in that file twice. Pull in the library only; it
// brings the encoder with it.
#include "cinepi_qraw_encoder.h"
#include "cinepi_qraw_encoder.cpp"

#include <cstdio>
#include <cstring>
#include <string>

int main(int argc, char** argv) {
    std::string input, mode_name = "m5", expect, o_params;
    bool use_v2 = false;   // --v2 exercises the emit the camera now uses
    bool use_lib = false;  // --library drives cinepi_qraw_encode itself
    bool use_stride = false; // --strided feeds a PADDED buffer via src_stride
    /* v3.13 src_shift gate pair. Both derive the SAME 12-bit samples from the
       bundled 16-bit frame (v >> 4), so the two runs must produce identical
       containers -- a different gpr_bytes means they encoded different
       pixels.
         --tight12   : tight pre-shifted buffer, effective_bits=12, src_shift=0
                       (today's staging-pass behaviour)
         --shifted12 : padded 3872-pitch buffer holding the MSB-justified
                       samples (v12 << 4), crop 8,8, effective_bits=12,
                       src_shift=4 -- read where it lies, no staging pass */
    bool tight12 = false, shifted12 = false;
    /* v3.14 src_byteswap gate. The SAME 16-bit samples as the plain
       --library run, but stored BIG-ENDIAN in a poisoned padded buffer and
       read with cfg.src_byteswap = 1 (plus the stride/crop, so the gate
       covers the exact production combination: a big-endian RAW16 DMA
       buffer read where it lies). Must produce the byte-identical container
       -- the same expected CRC as every other path. */
    bool swapped16 = false;
    int black = 0;   /* library mode only; POST-shift scale, like cfg.black */
    int bits = 12;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--input" && i + 1 < argc) input = argv[++i];
        else if (a == "--mode" && i + 1 < argc) mode_name = argv[++i];
        else if (a == "--compand-bits" && i + 1 < argc) bits = std::stoi(argv[++i]);
        else if (a == "--expect" && i + 1 < argc) expect = argv[++i];
        else if (a == "--gpr-params" && i + 1 < argc) o_params = argv[++i];
        else if (a == "--v2") use_v2 = true;
        else if (a == "--library") use_lib = true;
        else if (a == "--strided") use_stride = true;
        else if (a == "--tight12")   { use_lib = true; tight12 = true; }
        else if (a == "--shifted12") { use_lib = true; shifted12 = true; }
        else if (a == "--swapped16") { use_lib = true; swapped16 = true; }
        else if (a == "--black" && i + 1 < argc) black = std::stoi(argv[++i]);
        else {
            std::fprintf(stderr,
                "usage: verify_crc --input RAW16 [--mode m5] "
                "[--compand-bits 12] [--expect HEX] [--gpr-params JSON]\n");
            return 2;
        }
    }
    if (input.empty()) {
        std::fprintf(stderr, "verify_crc: --input is required\n");
        return 2;
    }

    // These must match the pipeline the reference CRC was recorded against,
    // which is the live one: a 16-bit linear sample, GP-Log2 companded into a
    // 12-bit container. Getting any of it wrong produces a plausible-looking
    // container with a different size, which is how the first attempt at this
    // gate quietly disagreed with the reference.
    if (use_lib) {
        // Drive the PRODUCTION library, not a hand-built pipeline. This is
        // the configuration cinepi-raw actually runs, so it is the one worth
        // gating: everything above only proves the benchmark's path.
        CinepiQrawConfig cfg{};
        cinepi_qraw_config_defaults(&cfg);
        cfg.width = 3840; cfg.height = 2160;
        cfg.bayer = CINEPI_BAYER_GBRG;
        cfg.mode = mode_name.c_str();
        cfg.effective_bits = 16;
        cfg.white = 0;              /* 0 => (1<<effective_bits)-1 */
        cfg.black = black;          /* POST-shift scale (cfg contract) */
        if (!o_params.empty()) cfg.gpr_params_json = o_params.c_str();
        /* --strided proves the whole point of the stride path: a padded
           buffer read WHERE IT LIES must produce the identical container to
           the tight one. The padding is deliberately non-zero so that reading
           it by mistake cannot go unnoticed. */
        const bool pad = use_stride || shifted12 || swapped16;
        const size_t CROP_X = pad ? 8 : 0;
        const size_t CROP_Y = use_stride ? 10 : ((shifted12 || swapped16) ? 8 : 0);
        const size_t PITCH  = pad ? 3872 : 3840;   /* samples per row */
        const size_t PADH   = use_stride ? 2180 : ((shifted12 || swapped16) ? 2176 : 2160);
        std::vector<uint16_t> lraw(size_t(3840) * 2160);
        {
            FILE* f = std::fopen(input.c_str(), "rb");
            if (!f) { std::fprintf(stderr, "cannot open %s\n", input.c_str()); return 2; }
            if (std::fread(lraw.data(), 2, lraw.size(), f) != lraw.size()) {
                std::fclose(f); std::fprintf(stderr, "short read\n"); return 2; }
            std::fclose(f);
        }
        /* The v3.13 gate pair encodes the same 12-bit samples two ways.
           tight12 stages them (v >> 4, tight, the historical copy pass);
           shifted12 presents them MSB-justified ((v >> 4) << 4 -- low nibble
           exactly zero, as the IMX585 delivers RAW12) and lets the encoder
           shift at load. The bundled sample is genuinely 16-bit, so both
           sides quantise it to the same 12 bits first. */
        if (tight12) {
            for (auto& v : lraw) v = uint16_t(v >> 4);
            cfg.effective_bits = 12;   /* white 0 => 4095 */
        }
        if (shifted12) {
            for (auto& v : lraw) v = uint16_t((v >> 4) << 4);
            cfg.effective_bits = 12;
            cfg.src_shift = 4;
        }
        /* swapped16: byte-swap every stored word. cfg.src_byteswap makes the
           encoder undo it at load, so the encoded pixels -- and therefore
           the container -- must be identical to the plain --library run. */
        if (swapped16) {
            for (auto& v : lraw) v = uint16_t((v >> 8) | (v << 8));
            cfg.src_byteswap = 1;
        }
        /* Build the padded buffer BEFORE creating the encoder, because the
           stride fields are read at create time. The padding is poisoned so
           that reading it by mistake could not go unnoticed. */
        std::vector<uint16_t> padded;
        const uint16_t *feed = lraw.data();
        if (pad) {
            padded.assign(PITCH * PADH, uint16_t(0xDEAD));
            for (size_t y = 0; y < 2160; ++y)
                std::memcpy(padded.data() + (y + CROP_Y) * PITCH + CROP_X,
                            lraw.data() + y * 3840, 3840 * sizeof(uint16_t));
            feed = padded.data();
            cfg.src_stride_elems = PITCH;
            cfg.src_crop_x = int(CROP_X);
            cfg.src_crop_y = int(CROP_Y);
        }
        CinepiQrawEncoder* h = cinepi_qraw_create(&cfg, nullptr, nullptr);
        if (!h) { std::fprintf(stderr, "library create failed\n"); return 2; }

        void* buf = nullptr; size_t sz = 0;
        const int lframes = 2 + DirectGprEncoder::SPLICE_VERIFY_FRAMES;
        for (int f = 0; f < lframes; ++f)
            if (cinepi_qraw_encode(h, feed, &buf, &sz) != 0) {
                std::fprintf(stderr, "library encode failed\n"); return 2; }
        uint32_t lc = 0xFFFFFFFFu;
        const auto* lp = static_cast<const unsigned char*>(buf);
        for (size_t i = 0; i < sz; ++i) {
            lc ^= lp[i];
            for (int b = 0; b < 8; ++b)
                lc = (lc >> 1) ^ (0xEDB88320u & (0u - (lc & 1u)));
        }
        lc = ~lc;
        char lhex[16]; std::snprintf(lhex, sizeof(lhex), "%08x", lc);
        std::printf("CINEPI_GATE emit=%s mode=%s frames=%d gpr_bytes=%zu "
                    "crc32=%s\n",
                    tight12 ? "library-tight12" :
                    shifted12 ? "library-shifted12" :
                    swapped16 ? "library-swapped16" :
                    use_stride ? "library-strided" : "library",
                    mode_name.c_str(), lframes, sz, lhex);
        cinepi_qraw_release(h, buf);
        cinepi_qraw_destroy(h);
        if (!expect.empty()) {
            if (expect != lhex) {
                std::printf("CINEPI_GATE FAIL expected=%s got=%s\n",
                            expect.c_str(), lhex);
                return 1;
            }
            std::printf("CINEPI_GATE PASS\n");
        }
        return 0;
    }

    Options o;
    o.width = 3840; o.height = 2160; o.bayer = "gbrg";
    o.mode = mode_name;
    o.execution = "cpu-gpr";
    o.effective_bits = 16;                 // the sample is 16-bit linear
    o.white = (1 << o.effective_bits) - 1; // normalised here, as main() does
    o.black = 0;                           // the bundled sample has no pedestal
    o.compand_bits = 12;                   // container stays 12-bit
    o.compand_inframe_bits = bits;         // effective precision
    if (!o_params.empty()) o.gpr_params = o_params;
    o.cpu_wavelet_fused = true;
    o.cpu_split_fused = true;
    o.cpu_split_neon = true;
    o.cpu_nontemporal = true;
    o.cpu_gpr_dng_splice = true;
    o.cpu_gpr_splice_shared = true;
    o.cpu_gpr_raw_copy = false;
    o.cpu_gpr_helper = false;
    o.cpu_v2_kernel = use_v2;
    o.cpu_sidecar = use_v2;
    o.cpu_sidecar_zskip = use_v2;

    ModeSpec ms = resolve_mode(o.mode, o.log_strength, o.working_max);

    const size_t npix = size_t(o.width) * size_t(o.height);
    std::vector<uint16_t> raw(npix);
    {
        FILE* f = std::fopen(input.c_str(), "rb");
        if (!f) { std::fprintf(stderr, "cannot open %s\n", input.c_str()); return 2; }
        const size_t got = std::fread(raw.data(), 2, npix, f);
        std::fclose(f);
        if (got != npix) {
            std::fprintf(stderr, "short read: %zu of %zu samples\n", got, npix);
            return 2;
        }
    }

    const auto lut = make_lut(o);
    const size_t row_stride = size_t(o.width) / 2;
    const size_t plane_stride = row_stride * size_t(o.height / 2);
    std::vector<int16_t> coeff(plane_stride * 4, 0);

    CpuFusedContext ctx;
    V2Frame v2ctx;
    std::vector<uint8_t> mask;
    V2Sidecar sc{};
    const V2Sidecar* scp = nullptr;
    if (use_v2 && o.cpu_sidecar) {
        mask.assign(coeff.size() / 8u + 64u, 0u);
        sc.mask_base = mask.data();
        sc.coeff_base = coeff.data();
        scp = &sc;
    }
    cpu_fused_frame_from_raw(o, &ms, raw.data(), lut, coeff.data(),
                             plane_stride, row_stride, ctx, scp, &v2ctx);

    DirectGprEncoder enc(o, &ms, row_stride, plane_stride);
    // The shipped encoder carries a nonzero-mask sidecar; the gate has to
    // exercise the same path or it is checking a pipeline nobody runs.
    std::vector<uint8_t> sidecar(coeff.size() / 8 + 64, 0);
    enc.set_sidecar(sidecar.data(), coeff.data());
    gpr_buffer out{nullptr, 0};
    size_t vc5_bytes = 0;
    const int frames = 2 + DirectGprEncoder::SPLICE_VERIFY_FRAMES;
    for (int f = 0; f < frames; ++f) enc.encode(coeff.data(), out, vc5_bytes);
    if (!out.buffer || out.size < 8) {
        std::fprintf(stderr, "no container produced\n");
        return 2;
    }

    uint32_t crc = 0xFFFFFFFFu;
    const auto* p = static_cast<const unsigned char*>(out.buffer);
    for (size_t i = 0; i < out.size; ++i) {
        crc ^= p[i];
        for (int b = 0; b < 8; ++b)
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    crc = ~crc;
    const size_t gpr_bytes = out.size;   // free_buffer zeroes it
    enc.free_buffer(out);

    char hex[16];
    std::snprintf(hex, sizeof(hex), "%08x", crc);
    std::printf("CINEPI_GATE emit=%s mode=%s bits=%d frames=%d gpr_bytes=%zu "
                "crc32=%s\n", use_v2 ? "v2" : "shipped",
                mode_name.c_str(), bits, frames, gpr_bytes, hex);

    if (!expect.empty()) {
        if (expect != hex) {
            std::printf("CINEPI_GATE FAIL expected=%s got=%s\n",
                        expect.c_str(), hex);
            return 1;
        }
        std::printf("CINEPI_GATE PASS\n");
    }
    return 0;
}
