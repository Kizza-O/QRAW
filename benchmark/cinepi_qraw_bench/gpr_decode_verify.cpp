/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2026 Kieran Olsson
 */
// ===========================================================================
// gpr_decode_verify -- decode every GPR on the command line through the GPR
// SDK's reader (VC-5 decoder + Adobe DNG SDK, the same stack Adobe tooling
// uses) and report the result. Exit nonzero if any file fails to decode.
//
// This is the fail-closed gate for the benchmark's output files: a GPR that
// encodes fast but does not decode is not a result, it is a bug.
// ===========================================================================
#include "gpr.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <cstdint>
#include <cmath>

static void* verify_alloc(size_t n) { return malloc(n); }
static void  verify_free(void* p)   { free(p); }

// Optional: --reference <raw16> makes the tool compare the DECODED image
// against the original sensor frame and report PSNR. Decoding without error
// is a weak gate -- it passed happily while 11- and 10-bit files were being
// reconstructed with a large negative offset and clamped to black. Comparing
// against the source is the gate that actually catches that.
int main(int argc, char** argv) {
    if (argc < 2) { std::printf("usage: gpr_decode_verify [--reference raw.raw16] [--dump-raw PREFIX] file.gpr [...]\n"); return 2; }
    const char *dump_prefix = nullptr;
    std::vector<uint16_t> ref;
    int first = 1;
    int dump_count = 0;
    for (int i = 1; i + 1 < argc; ++i)
        if (std::strcmp(argv[i], "--dump-raw") == 0) {
            dump_prefix = argv[i + 1];
            for (int j = i; j + 2 < argc; ++j) argv[j] = argv[j + 2];
            argc -= 2;
            break;
        }
    if (argc > 2 && std::strcmp(argv[1], "--reference") == 0) {
        FILE* rf = std::fopen(argv[2], "rb");
        if (!rf) { std::printf("cannot open reference %s\n", argv[2]); return 2; }
        std::fseek(rf, 0, SEEK_END); const long rn = std::ftell(rf); std::fseek(rf, 0, SEEK_SET);
        ref.resize(static_cast<size_t>(rn) / 2);
        if (std::fread(ref.data(), 1, static_cast<size_t>(rn), rf) != static_cast<size_t>(rn)) {
            std::printf("short read on reference\n"); return 2; }
        std::fclose(rf);
        first = 3;
    }
    int fails = 0;
    for (int i = first; i < argc; ++i) {
        FILE* f = std::fopen(argv[i], "rb");
        if (!f) { std::printf("DECODE %s FAIL open\n", argv[i]); ++fails; continue; }
        std::fseek(f, 0, SEEK_END);
        const long n = std::ftell(f);
        std::fseek(f, 0, SEEK_SET);
        std::vector<uint8_t> buf(static_cast<size_t>(n));
        const size_t got = std::fread(buf.data(), 1, size_t(n), f);
        std::fclose(f);
        if (got != size_t(n)) { std::printf("DECODE %s FAIL read\n", argv[i]); ++fails; continue; }

        gpr_allocator alloc{verify_alloc, verify_free};
        gpr_buffer in{buf.data(), size_t(n)};
        gpr_buffer raw{nullptr, 0};
        const bool ok = gpr_convert_gpr_to_raw(&alloc, &in, &raw);
        if (!ok || !raw.buffer || raw.size == 0) {
            std::printf("DECODE %s FAIL gpr_convert_gpr_to_raw\n", argv[i]);
            ++fails;
            continue;
        }
        // Sampled stats so a decode that "succeeds" into garbage is visible.
        const uint16_t* px = static_cast<const uint16_t*>(raw.buffer);
        const size_t count = raw.size / 2;
        uint16_t mn = 0xffff, mx = 0;
        unsigned long long sum = 0, samples = 0;
        for (size_t k = 0; k < count; k += 97) {
            mn = px[k] < mn ? px[k] : mn;
            mx = px[k] > mx ? px[k] : mx;
            sum += px[k]; ++samples;
        }
        if (!ref.empty() && ref.size() == count) {
            // The companding is monotonic, so a correct decode tracks the
            // source closely; a broken inverse shows up immediately here.
            double se = 0.0; size_t n = 0;
            for (size_t k = 0; k < count; k += 7) {
                const double d = double(px[k]) - double(ref[k]); se += d * d; ++n;
            }
            const double rmse = std::sqrt(se / double(n ? n : 1));
            const double psnr = rmse > 0.0 ? 20.0 * std::log10(65535.0 / rmse) : 999.0;
            const bool ok = psnr >= 30.0;
            std::printf("DECODE %s %s raw_bytes=%zu min=%u max=%u mean=%llu rmse=%.1f psnr=%.1f dB\n",
                        argv[i], ok ? "PASS" : "FAIL-PSNR", raw.size, mn, mx,
                        samples ? sum / samples : 0ull, rmse, psnr);
            if (!ok) ++fails;
        } else {
            if (dump_prefix) {
                char dn[512];
                std::snprintf(dn, sizeof dn, "%s_%d.raw16", dump_prefix, dump_count++);
                FILE *df = std::fopen(dn, "wb");
                if (df) {
                    std::fwrite(raw.buffer, 1, raw.size, df);
                    std::fclose(df);
                    std::printf("DUMPED %s bytes=%zu\n", dn, raw.size);
                }
            }
            std::printf("DECODE %s PASS raw_bytes=%zu sample_min=%u sample_max=%u sample_mean=%llu\n",
                        argv[i], raw.size, mn, mx, samples ? sum / samples : 0ull);
        }
        verify_free(raw.buffer);
    }
    std::printf(fails ? "DECODE_VERIFY FAILURES=%d\n" : "DECODE_VERIFY ALL PASS\n", fails);
    return fails ? 1 : 0;
}
