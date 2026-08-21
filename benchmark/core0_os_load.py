#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Kieran Olsson
"""Small CPU-duty generator intended to be pinned to Pi core 0 with taskset."""
import argparse, time, signal, sys

ap = argparse.ArgumentParser()
ap.add_argument('--percent', type=float, required=True)
ap.add_argument('--period-ms', type=float, default=10.0)
a = ap.parse_args()
if not 0.0 <= a.percent <= 100.0:
    raise SystemExit('--percent must be 0..100')
period = max(0.001, a.period_ms / 1000.0)
busy = period * a.percent / 100.0
running = True

def stop(*_):
    global running
    running = False
signal.signal(signal.SIGTERM, stop)
signal.signal(signal.SIGINT, stop)
# Keep a small integer dependency so the interpreter cannot optimise away work.
x = 0x12345678
while running:
    start = time.perf_counter()
    end_busy = start + busy
    while running and time.perf_counter() < end_busy:
        x = ((x * 1664525) + 1013904223) & 0xffffffff
    remain = period - (time.perf_counter() - start)
    if remain > 0:
        time.sleep(remain)
# Make x observable in the unlikely event an alternative interpreter gets clever.
if x == -1:
    print(x, file=sys.stderr)
