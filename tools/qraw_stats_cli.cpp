/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2026 Kieran Olsson
 */
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

#include "../integration/cinepi-raw/qraw_live_stats.hpp"

static void print_snapshot(const QrawLiveStats &s)
{
    double repack_sum = 0.0, wave_sum = 0.0, ent_sum = 0.0, copy_sum = 0.0;
    unsigned active = 0;
    for (unsigned i = 0; i < std::min<unsigned>(s.workers, QRAW_LIVE_MAX_WORKERS); ++i) {
        if (s.worker[i].frames) {
            ++active;
            repack_sum += s.worker[i].repack_ms;
            wave_sum += s.worker[i].avg_wavelet_ms;
            ent_sum += s.worker[i].avg_entropy_ms;
            copy_sum += s.worker[i].copy_out_ms;
        }
    }
    const double div = active ? double(active) : 1.0;
    std::printf(
        "QRAW_LIVE_SNAPSHOT mode=%s encoder=%s width=%u height=%u bits=%u "
        "workers=%u frames=%llu submitted=%llu dropped=%llu bytes_total=%llu "
        "last_bytes=%llu ratio=%.6f fps_now=%.6f fps_avg=%.6f camera_fps=%.6f "
        "output_mibs=%.6f encode_queue=%u encode_limit=%u disk_queue=%u "
        "recording=%u buffer_full=%u stage_ms=%.6f wavelet_ms=%.6f "
        "entropy_ms=%.6f copy_out_ms=%.6f "
        /* layout 3: why frames were lost, and the ones no drop counter sees */
        "drops_queue_full=%llu drops_inflight=%llu drops_gate=%llu "
        "drops_other=%llu frames_missed=%llu "
        /* layout 7/8: the grade actually in force. mode= above is the bare ladder
         * position, which is all it can be -- readers string-compare it. This one
         * is the rung name in QRAW STABLE and the spelled-out fixed grade
         * ("M5E1+X") otherwise, i.e. what the camera UI shows. */
        /* layout 9: Pixel Clean. Not part of grade= -- that label has no room for
         * a suffix and is empty in QRAW STABLE, where Pixel Clean is in force on
         * every rung. Baseline on, so this normally reads 1; it reads 0 for an
         * A/B, which is a bit-exact different picture and the clip's filenames
         * lose their -pc tag to match. */
        /* layout 10: Noise Clean profile, 0..3. Normally 0 -- see the field
         * comment; a non-zero value means this clip's fine level-1 detail and
         * its throughput are both off the documented numbers. */
        "pixel_clean=%u noise_clean=%u grade=%s clip=%s\n",
        s.mode, s.encoder_version, s.width, s.height, s.effective_bits,
        s.workers,
        (unsigned long long)s.frames_encoded,
        (unsigned long long)s.frames_submitted,
        (unsigned long long)s.frames_dropped,
        (unsigned long long)s.bytes_total,
        (unsigned long long)s.last_bytes,
        s.last_ratio, s.fps_now, s.fps_avg, s.camera_fps, s.output_mibs,
        s.encode_queue_depth, s.encode_queue_limit, s.disk_queue_depth,
        s.recording, s.buffer_full,
        repack_sum / div, wave_sum / div, ent_sum / div, copy_sum / div,
        (unsigned long long)s.drops_queue_full,
        (unsigned long long)s.drops_inflight,
        (unsigned long long)s.drops_gate,
        (unsigned long long)s.drops_other,
        (unsigned long long)s.frames_missed,
        s.pixel_clean,
        s.noise_clean,
        s.stable_label[0] ? s.stable_label
                          : (s.grade_label[0] ? s.grade_label : s.mode),
        s.clip_name);
    std::fflush(stdout);
}

int main(int argc, char **argv)
{
    int wait_ms = 0;
    bool watch = false;
    int watch_ms = 1000;
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--wait-ms") && i + 1 < argc)
            wait_ms = std::max(0, std::atoi(argv[++i]));
        else if (!std::strcmp(argv[i], "--watch-ms") && i + 1 < argc) {
            watch = true;
            watch_ms = std::max(50, std::atoi(argv[++i]));
        } else if (!std::strcmp(argv[i], "--help")) {
            std::puts("usage: qraw_stats_cli [--wait-ms N] [--watch-ms N]");
            return 0;
        }
    }

    QrawLiveStatsReader r;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(wait_ms);
    do {
        if (r.attach()) break;
        if (wait_ms <= 0) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    } while (std::chrono::steady_clock::now() < deadline);

    if (!r.attached()) {
        std::fprintf(stderr, "QRAW_LIVE_STATS_UNAVAILABLE shm=%s\n", QRAW_LIVE_STATS_SHM_NAME);
        return 2;
    }

    do {
        QrawLiveStats s{};
        if (r.snapshot(s)) print_snapshot(s);
        else std::fprintf(stderr, "QRAW_LIVE_STATS_RACE\n");
        if (!watch) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(watch_ms));
    } while (true);
    return 0;
}
