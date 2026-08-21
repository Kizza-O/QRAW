/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2026 Kieran Olsson
 */
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

using Clock = std::chrono::steady_clock;
static volatile uint64_t sink_u64 = 0;
static volatile double sink_f64 = 0.0;

static size_t bytes_for(const std::string& mode) {
    if (mode == "l2") return 256u * 1024u;          // comfortably inside 512 KiB private L2
    if (mode == "l3") return 1536u * 1024u;         // > L2, < 2 MiB shared L3
    if (mode == "dram_read" || mode == "dram_rw") return 64u * 1024u * 1024u;
    return 0;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "usage: core0_mem_stress MODE SECONDS [DUTY_PERCENT]\n"
                     "modes: alu l2 l3 dram_read dram_rw\n";
        return 2;
    }
    const std::string mode = argv[1];
    const double seconds = std::max(0.1, std::atof(argv[2]));
    const double duty = argc >= 4 ? std::clamp(std::atof(argv[3]), 1.0, 100.0) : 100.0;
    const size_t bytes = bytes_for(mode);
    std::vector<uint64_t> buf;
    if (bytes) {
        buf.resize(bytes / sizeof(uint64_t));
        for (size_t i = 0; i < buf.size(); ++i) buf[i] = (i * 0x9e3779b97f4a7c15ULL) ^ (i >> 3);
    }

    const auto start = Clock::now();
    auto period_start = start;
    constexpr auto period = std::chrono::microseconds(10000);
    uint64_t logical_bytes = 0;
    uint64_t iters = 0;
    uint64_t x = 0x123456789abcdef0ULL;
    double d = 1.000000119;

    while (std::chrono::duration<double>(Clock::now() - start).count() < seconds) {
        const auto busy_until = period_start + std::chrono::microseconds((long long)(100.0 * duty)); // 10ms * duty/100
        do {
            if (mode == "alu") {
                for (int i = 0; i < 4096; ++i) {
                    x ^= x << 13; x ^= x >> 7; x ^= x << 17;
                    d = d * 1.0000000001 + double(x & 1023u) * 1e-15;
                }
                ++iters;
            } else if (mode == "l2" || mode == "l3" || mode == "dram_read") {
                uint64_t acc = x;
                // one load per cache line. For l2/l3 repeated passes become cache-resident;
                // the 64 MiB case remains a streaming DRAM workload on Pi 5.
                for (size_t i = 0; i < buf.size(); i += 8) acc += buf[i];
                x ^= acc;
                logical_bytes += (buf.size() / 8) * 64;
                ++iters;
            } else if (mode == "dram_rw") {
                uint64_t acc = x;
                for (size_t i = 0; i < buf.size(); i += 8) {
                    uint64_t v = buf[i];
                    acc += v;
                    buf[i] = v + 0x9e3779b97f4a7c15ULL;
                }
                x ^= acc;
                logical_bytes += (buf.size() / 8) * 128; // approximate read + write traffic
                ++iters;
            } else {
                std::cerr << "unknown mode: " << mode << "\n";
                return 2;
            }
        } while (Clock::now() < busy_until);

        const auto next_period = period_start + period;
        if (duty < 100.0 && Clock::now() < next_period) std::this_thread::sleep_until(next_period);
        period_start = next_period;
        if (Clock::now() > period_start + period) period_start = Clock::now();
    }
    const double elapsed = std::chrono::duration<double>(Clock::now() - start).count();
    sink_u64 = x; sink_f64 = d;
    const double gib_s = elapsed > 0.0 ? double(logical_bytes) / elapsed / (1024.0*1024.0*1024.0) : 0.0;
    std::cout << "CORE0_STRESS mode=" << mode << " duty=" << duty
              << " seconds=" << elapsed << " working_set_bytes=" << bytes
              << " logical_gib_s=" << gib_s << " iterations=" << iters << "\n";
    return 0;
}
