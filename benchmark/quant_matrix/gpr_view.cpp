/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2026 Kieran Olsson
 */
// ===========================================================================
// gpr_view -- decode a .gpr through the GPR SDK and write a viewable image.
//
// The v4.0 package ships gpr_decode_verify (decode + PSNR) but no viewer:
// gpr_dng_export.py looks for a `gpr_decode_validator` binary that this
// package never builds. This fills that gap for the quant matrix, where the
// whole point is to LOOK at what a quant table did to the picture.
//
//   gpr_view out_prefix file.gpr [...]
//     -> out_prefix.ppm   full-resolution demosaiced RGB, 8-bit
//
// PPM because it needs no image library and every viewer and ffmpeg reads it.
// Full resolution, not the SDK's quarter-res default: the bands this matrix
// prunes are the FINEST detail bands, so a downscaled preview would hide
// precisely the damage being measured.
// ===========================================================================
#include "gpr.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

static void* v_alloc(size_t n) { return malloc(n); }
static void  v_free(void* p)   { free(p); }

int main(int argc, char** argv) {
    if (argc < 3) {
        std::printf("usage: gpr_view OUT_PREFIX file.gpr [file.gpr ...]\n");
        return 2;
    }
    const std::string prefix = argv[1];
    int fails = 0;

    for (int i = 2; i < argc; ++i) {
        FILE* f = std::fopen(argv[i], "rb");
        if (!f) { std::printf("VIEW %s FAIL open\n", argv[i]); ++fails; continue; }
        std::fseek(f, 0, SEEK_END);
        const long n = std::ftell(f);
        std::fseek(f, 0, SEEK_SET);
        std::vector<uint8_t> buf(size_t(n < 0 ? 0 : n));
        const size_t got = std::fread(buf.data(), 1, buf.size(), f);
        std::fclose(f);
        if (got != buf.size()) { std::printf("VIEW %s FAIL read\n", argv[i]); ++fails; continue; }

        gpr_allocator alloc{v_alloc, v_free};
        gpr_buffer in{buf.data(), buf.size()};
        gpr_rgb_buffer rgb{nullptr, 0, 0, 0};

        if (!gpr_convert_gpr_to_rgb(&alloc, GPR_RGB_RESOLUTION_FULL, 8, &in, &rgb)
            || !rgb.buffer || !rgb.width || !rgb.height) {
            std::printf("VIEW %s FAIL gpr_convert_gpr_to_rgb\n", argv[i]);
            ++fails;
            continue;
        }

        const std::string out = (argc == 3) ? prefix + ".ppm"
                                            : prefix + "_" + std::to_string(i - 2) + ".ppm";
        FILE* o = std::fopen(out.c_str(), "wb");
        if (!o) {
            std::printf("VIEW %s FAIL open out %s\n", argv[i], out.c_str());
            ++fails;
            v_free(rgb.buffer);
            continue;
        }
        std::fprintf(o, "P6\n%zu %zu\n255\n", rgb.width, rgb.height);
        std::fwrite(rgb.buffer, 1, rgb.width * rgb.height * 3, o);
        std::fclose(o);
        std::printf("VIEW %s -> %s (%zux%zu)\n", argv[i], out.c_str(),
                    rgb.width, rgb.height);
        v_free(rgb.buffer);
    }
    std::printf(fails ? "GPR_VIEW FAILURES=%d\n" : "GPR_VIEW ALL OK\n", fails);
    return fails ? 1 : 0;
}
