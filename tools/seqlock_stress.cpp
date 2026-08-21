/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2026 Kieran Olsson
 */
/* ===========================================================================
 * seqlock_stress.cpp -- tear detector for the QrawLiveStats seqlock.
 *
 *   g++ -O2 -std=c++17 seqlock_stress.cpp -lpthread -o seqlock_stress
 *   ./seqlock_stress          # fixed writer: expect 0 torn snapshots
 *   ./seqlock_stress --buggy  # pre-fix writer: expect torn snapshots
 *
 * The writer publishes frames where every payload word carries the same
 * counter value; a reader snapshot whose words disagree is a torn read the
 * seqlock failed to reject. --buggy reproduces the original publish(): the
 * whole struct memcpy'd over the mapping, momentarily replacing the live
 * odd sequence with the source struct's stale even value, so a reader whose
 * two sequence reads both land in clobber windows of consecutive publishes
 * accepts torn data. The fixed writer keeps the in-shm sequence odd for the
 * entire copy.
 * ========================================================================= */
#include "../cinepi-raw/qraw_live_stats.hpp"

#include <atomic>
#include <cstdio>
#include <cstring>
#include <thread>

static std::atomic<bool> stop{false};
static std::atomic<uint64_t> snapshots{0}, torn{0};

/* the original, pre-fix publish() */
static void buggy_publish(QrawLiveStats *map, const QrawLiveStats &s)
{
    const uint32_t seq = map->sequence;
    map->sequence = seq + 1;
    std::atomic_thread_fence(std::memory_order_release);
    const uint32_t keep = map->sequence;
    std::memcpy(map, &s, sizeof(QrawLiveStats));   /* clobbers sequence! */
    map->magic = QRAW_LIVE_STATS_MAGIC;
    map->layout = QRAW_LIVE_STATS_LAYOUT;
    map->sequence = keep;
    std::atomic_thread_fence(std::memory_order_release);
    map->sequence = keep + 1;
}

int main(int argc, char **argv)
{
    const bool buggy = (argc > 1 && std::strcmp(argv[1], "--buggy") == 0);

    shm_unlink(QRAW_LIVE_STATS_SHM_NAME);
    QrawLiveStatsWriter writer;
    if (!writer.open()) { std::fprintf(stderr, "shm open failed\n"); return 1; }

    /* raw mapping for the buggy variant */
    int fd = shm_open(QRAW_LIVE_STATS_SHM_NAME, O_RDWR, 0);
    auto *map = static_cast<QrawLiveStats *>(
        mmap(nullptr, sizeof(QrawLiveStats), PROT_READ | PROT_WRITE,
             MAP_SHARED, fd, 0));
    close(fd);

    std::thread w([&] {
        QrawLiveStats s{};
        uint64_t n = 0;
        while (!stop) {
            ++n;
            /* every payload word carries n */
            s.frames_encoded = s.frames_submitted = s.bytes_total =
                s.last_bytes = n;
            for (auto &wk : s.worker) wk.frames = n;
            if (buggy) buggy_publish(map, s);
            else       writer.publish(s);
        }
    });

    std::thread r([&] {
        QrawLiveStatsReader reader;
        while (!reader.attach()) {}
        QrawLiveStats s;
        while (!stop) {
            if (!reader.snapshot(s)) continue;
            ++snapshots;
            const uint64_t n = s.frames_encoded;
            bool ok = s.frames_submitted == n && s.bytes_total == n &&
                      s.last_bytes == n;
            for (auto &wk : s.worker) ok = ok && wk.frames == n;
            if (!ok) ++torn;
        }
    });

    std::this_thread::sleep_for(std::chrono::seconds(10));
    stop = true;
    w.join(); r.join();
    shm_unlink(QRAW_LIVE_STATS_SHM_NAME);

    std::printf("%s writer: %llu snapshots, %llu torn\n",
                buggy ? "BUGGY" : "FIXED",
                (unsigned long long)snapshots.load(),
                (unsigned long long)torn.load());
    std::printf("verdict: %s\n",
                torn.load() == 0 ? "clean" : "TORN READS DETECTED");
    return torn.load() == 0 ? 0 : 1;
}
