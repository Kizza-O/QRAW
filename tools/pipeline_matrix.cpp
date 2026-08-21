/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2026 Kieran Olsson
 */
/* pipeline_matrix.cpp -- isolate each way the live camera path differs from
 * the static benchmark, driving the REAL encoder library with no camera.
 *
 *   g++ -O3 -mcpu=cortex-a76 -std=c++17 -o pipeline_matrix pipeline_matrix.cpp \
 *       -I/opt/cinepi-qraw/include \
 *       -Wl,--start-group /opt/cinepi-qraw/lib/*.a -Wl,--end-group -lpthread -lm
 *
 *   ./pipeline_matrix --buffers 8 --copy on --capture 60
 *
 * See docs/FINDING_CLEARHDR_BLACK_FRAME.md section 4 for the measured table.
 */
/* Controlled matrix over the REAL v1.16.6 encoder library.
 *
 * Isolates each way the live camera path differs from the static benchmark,
 * with no camera involved, so every variable moves one at a time:
 *
 *   buffers   1 = every worker encodes the SAME source buffer (static bench)
 *             N = each frame comes from a rotating pool (camera staging pool)
 *   copy      per-frame 16.6 MB staging copy on a producer thread (core 0)
 *   capture   background thread emulating CFE write + ISP read of a 16.81 MB
 *             frame at the capture rate (the traffic a live sensor adds)
 */
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <atomic>
#include <thread>
#include <vector>
#include <chrono>
#include <string>
#include <sched.h>
#include <unistd.h>
#include <pthread.h>
#include <arm_neon.h>
extern "C" {
#include "cinepi_qraw_encoder.h"
}

static const int EW = 3840, EH = 2160;
static const int CW = 3856, CH = 2180, CSTRIDE = 7744;   // camera geometry
static const size_t FRAME_ELEMS = size_t(EW) * EH;
static const size_t FRAME_BYTES = FRAME_ELEMS * 2;
static const size_t CAM_BYTES   = size_t(CSTRIDE) * CH;

static double now() {
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}
static void pin(int c) {
    cpu_set_t s; CPU_ZERO(&s); CPU_SET(c, &s);
    pthread_setaffinity_np(pthread_self(), sizeof(s), &s);
}

struct Cfg {
    bool bits16 = false;   // MSB-justified source, effective_bits=16
    int nbuf = 1;     // source pool size (1 = shared, like the static bench)
    bool copy = false;
    int capture_fps = 0;  // background CFE+ISP traffic, 0 = none
    int workers = 3;
    double seconds = 6.0;
    const char *mode = "m7";
};

static std::atomic<bool> stop_bg{false};

/* Emulates the sensor path: a full-frame write (CFE) plus a full-frame read
 * (ISP re-reading the raw to build the preview), at the capture rate. */
static void bg_capture(int fps) {
    pin(0);
    std::vector<uint8_t> w(CAM_BYTES), r(CAM_BYTES);
    memset(r.data(), 0x3c, CAM_BYTES);
    volatile uint64_t sink = 0;
    const double period = 1.0 / fps;
    double next = now();
    while (!stop_bg.load(std::memory_order_relaxed)) {
        memset(w.data(), 0x5a, CAM_BYTES);              // CFE write
        uint64_t acc = 0;
        const uint64_t *p = (const uint64_t *)r.data();
        for (size_t i = 0; i < CAM_BYTES / 8; i += 8) acc += p[i];   // ISP read
        sink += acc;
        next += period;
        double d = next - now();
        if (d > 0) std::this_thread::sleep_for(std::chrono::duration<double>(d));
        else next = now();
    }
    (void)sink;
}

int main(int argc, char **argv) {
    Cfg c; c.bits16=false; c.nbuf=1; c.copy=false; c.capture_fps=0; c.workers=3; c.seconds=6.0; c.mode="m7";
    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        auto val = [&]{ return std::string(argv[++i]); };
        if (a == "--buffers") c.nbuf = atoi(val().c_str());
        else if (a == "--copy") c.copy = val() == "on";
        else if (a == "--capture") c.capture_fps = atoi(val().c_str());
        else if (a == "--workers") c.workers = atoi(val().c_str());
        else if (a == "--seconds") c.seconds = atof(val().c_str());
        else if (a == "--mode") c.mode = strdup(val().c_str());
        else if (a == "--bits16") c.bits16 = val() == "on";
    }

    /* Source pool. Same synthetic content in every buffer so the entropy stage
     * has identical work to do regardless of how many buffers there are --
     * only the memory footprint changes. */
    std::vector<uint16_t *> pool(c.nbuf);
    for (int i = 0; i < c.nbuf; i++) {
        void *p = nullptr;
        if (posix_memalign(&p, 64, FRAME_BYTES)) { fprintf(stderr, "oom\n"); return 1; }
        uint16_t *b = (uint16_t *)p;
        for (size_t k = 0; k < FRAME_ELEMS; k++)
            b[k] = uint16_t(((k * 2654435761u >> 13) & 0x0fff) << (c.bits16 ? 4 : 0));
        pool[i] = b;
    }
    /* A camera-like padded source for the staging-copy leg. */
    uint8_t *cam = nullptr;
    if (c.copy) {
        void *p = nullptr;
        if (posix_memalign(&p, 64, CAM_BYTES)) return 1;
        cam = (uint8_t *)p;
        /* Same deterministic noise as the pool, MSB-justified so the >>4 in the
         * staging copy reproduces the identical 12-bit content. Filling it with
         * a constant would make every copied frame uniform and collapse the
         * entropy stage to nothing, which is not the workload under test. */
        for (int y = 0; y < CH; y++) {
            uint16_t *row = (uint16_t *)(cam + size_t(y) * CSTRIDE);
            for (int x = 0; x < CW; x++) {
                size_t k = (y >= 8 && x >= 8 && y - 8 < EH && x - 8 < EW)
                         ? size_t(y - 8) * EW + (x - 8) : 0;
                row[x] = uint16_t((((k * 2654435761u) >> 13) & 0x0fff) << 4);
            }
        }
    }

    std::vector<CinepiQrawEncoder *> enc(c.workers);
    for (int i = 0; i < c.workers; i++) {
        CinepiQrawConfig cfg;
        cinepi_qraw_config_defaults(&cfg);
        cfg.width = EW; cfg.height = EH; cfg.mode = c.mode;
        if (c.bits16) { cfg.effective_bits = 16; cfg.white = 4095 << 4; cfg.black = 200 << 4; }
        else           { cfg.effective_bits = 12; cfg.white = 4095;      cfg.black = 200; }
        cfg.compand_bits = 12;
        cfg.gpr_params_json = "/opt/cinepi-qraw/share/cinepi-qraw/gpr_params.json";
        enc[i] = cinepi_qraw_create(&cfg, nullptr, nullptr);
        if (!enc[i]) { fprintf(stderr, "cinepi_qraw_create failed\n"); return 1; }
    }

    std::thread bg;
    if (c.capture_fps > 0) bg = std::thread(bg_capture, c.capture_fps);

    std::atomic<uint64_t> frames{0};
    std::atomic<int> next_buf{0};
    std::atomic<bool> run{true};
    std::atomic<double> wav_sum{0}, ent_sum{0};
    std::atomic<size_t> last_size{0};

    std::vector<std::thread> ws;
    const long ncpu = sysconf(_SC_NPROCESSORS_ONLN);
    for (int w = 0; w < c.workers; w++) {
        ws.emplace_back([&, w] {
            pin(int(ncpu - c.workers + w));       // the winner's high-core policy
            while (run.load(std::memory_order_relaxed)) {
                int idx = c.nbuf == 1 ? 0
                        : (next_buf.fetch_add(1, std::memory_order_relaxed) % c.nbuf);
                uint16_t *src = pool[idx];
                if (c.copy) {   // producer-side staging copy, charged to this frame
                    for (int y = 0; y < EH; y++) {
                        const uint16_t *s = (const uint16_t *)(cam + size_t(y + 8) * CSTRIDE + 16);
                        uint16_t *o = src + size_t(y) * EW;
                        for (int x = 0; x + 8 <= EW; x += 8)
                            vst1q_u16(o + x, vshrq_n_u16(vld1q_u16(s + x), 4));
                    }
                }
                void *out = nullptr; size_t osz = 0;
                if (cinepi_qraw_encode(enc[w], src, &out, &osz) == 0) {
                    last_size.store(osz, std::memory_order_relaxed);
                    double wv = 0, en = 0, dn = 0;
                    cinepi_qraw_last_timings(enc[w], &wv, &en, &dn);
                    double e;
                    e = wav_sum.load(); while (!wav_sum.compare_exchange_weak(e, e + wv)) {}
                    e = ent_sum.load(); while (!ent_sum.compare_exchange_weak(e, e + en)) {}
                    cinepi_qraw_release(enc[w], out);
                    frames.fetch_add(1, std::memory_order_relaxed);
                }
            }
        });
    }

    double t0 = now();
    std::this_thread::sleep_for(std::chrono::duration<double>(c.seconds));
    run = false;
    for (auto &t : ws) t.join();
    double el = now() - t0;
    stop_bg = true;
    if (bg.joinable()) bg.join();

    uint64_t f = frames.load();
    printf("buffers=%-3d copy=%-3s capture=%-3d workers=%d  fps=%6.2f  wavelet=%5.1f entropy=%5.1f ms\n",
           c.nbuf, c.copy ? "on" : "off", c.capture_fps, c.workers,
           f / el, f ? wav_sum.load() / f : 0.0, f ? ent_sum.load() / f : 0.0);
    printf("    encoded_bytes=%zu  effective_bits=%d\n", last_size.load(), c.bits16 ? 16 : 12);

    for (int i = 0; i < c.workers; i++) cinepi_qraw_destroy(enc[i]);
    return 0;
}
