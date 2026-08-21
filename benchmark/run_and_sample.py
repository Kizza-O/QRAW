#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Kieran Olsson
"""Run a benchmark command while recording Pi clocks, thermals, memory and process RSS."""
from __future__ import annotations
import argparse, csv, os, pathlib, selectors, subprocess, sys, threading, time


def read_text(path: str) -> str | None:
    try:
        return pathlib.Path(path).read_text().strip()
    except Exception:
        return None


def read_int(path: str) -> int | None:
    t = read_text(path)
    try:
        return int(t) if t is not None else None
    except ValueError:
        return None


def mem_available_kib() -> int | None:
    try:
        for line in pathlib.Path('/proc/meminfo').read_text().splitlines():
            if line.startswith('MemAvailable:'):
                return int(line.split()[1])
    except Exception:
        pass
    return None


def process_rss_kib(pid: int) -> int | None:
    try:
        for line in pathlib.Path(f'/proc/{pid}/status').read_text().splitlines():
            if line.startswith('VmRSS:'):
                return int(line.split()[1])
    except Exception:
        pass
    return None


def cpu_clock_khz() -> int | None:
    vals=[]
    for p in pathlib.Path('/sys/devices/system/cpu').glob('cpu[0-9]*/cpufreq/scaling_cur_freq'):
        v=read_int(str(p))
        if v is not None: vals.append(v)
    return max(vals) if vals else None


def temperature_millic() -> int | None:
    vals=[]
    for p in pathlib.Path('/sys/class/thermal').glob('thermal_zone*/temp'):
        v=read_int(str(p))
        if v is not None and 0 < v < 200000: vals.append(v)
    return max(vals) if vals else None


def gpu_busy() -> float | None:
    candidates=list(pathlib.Path('/sys/class/drm').glob('card*/device/gpu_busy_percent'))
    for p in candidates:
        t=read_text(str(p))
        try:
            return float(t) if t is not None else None
        except ValueError:
            continue
    return None


def vcgencmd_clock_hz(domain: str) -> int | None:
    try:
        p=subprocess.run(['vcgencmd','measure_clock',domain],text=True,capture_output=True,timeout=1)
        if p.returncode==0 and '=' in p.stdout:
            return int(p.stdout.strip().split('=',1)[1])
    except Exception:
        pass
    return None


def throttled() -> str | None:
    try:
        p=subprocess.run(['vcgencmd','get_throttled'],text=True,capture_output=True,timeout=1)
        if p.returncode==0: return p.stdout.strip().split('=',1)[-1]
    except Exception:
        pass
    return None


def main() -> int:
    ap=argparse.ArgumentParser()
    ap.add_argument('--output',required=True)
    ap.add_argument('--interval',type=float,default=0.25)
    ap.add_argument('command',nargs=argparse.REMAINDER)
    ns=ap.parse_args()
    cmd=ns.command
    if cmd and cmd[0]=='--': cmd=cmd[1:]
    if not cmd:
        ap.error('missing command after --')
    out=pathlib.Path(ns.output)
    out.parent.mkdir(parents=True,exist_ok=True)
    console=out.with_suffix('.console.txt')
    samples=out.with_suffix('.samples.csv')
    before=out.with_suffix('.environment_before.txt')
    after=out.with_suffix('.environment_after.txt')
    before.write_text(f'throttled={throttled()}\ncpu_clock_khz={cpu_clock_khz()}\nv3d_clock_hz={vcgencmd_clock_hz("v3d")}\ntemperature_millic={temperature_millic()}\nmem_available_kib={mem_available_kib()}\n')

    proc=subprocess.Popen(cmd,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1)
    stop=threading.Event()

    def sampler() -> None:
        start=time.monotonic()
        with samples.open('w',newline='') as f:
            w=csv.writer(f)
            w.writerow(['elapsed_s','pid','rss_kib','mem_available_kib','cpu_clock_khz','v3d_clock_hz','temperature_millic','gpu_busy_pct','throttled'])
            last_throttle = throttled()
            next_throttle = start + 2.0
            while not stop.is_set():
                now = time.monotonic()
                if now >= next_throttle:
                    last_throttle = throttled()
                    next_throttle = now + 2.0
                w.writerow([f'{now-start:.3f}',proc.pid,process_rss_kib(proc.pid),mem_available_kib(),cpu_clock_khz(),vcgencmd_clock_hz('v3d'),temperature_millic(),gpu_busy(),last_throttle])
                f.flush()
                stop.wait(max(0.05,ns.interval))
    thread=threading.Thread(target=sampler,daemon=True)
    thread.start()
    with console.open('w') as f:
        assert proc.stdout is not None
        for line in proc.stdout:
            sys.stdout.write(line); sys.stdout.flush()
            f.write(line); f.flush()
    rc=proc.wait()
    stop.set(); thread.join(timeout=2)
    after.write_text(f'exit_code={rc}\nthrottled={throttled()}\ncpu_clock_khz={cpu_clock_khz()}\nv3d_clock_hz={vcgencmd_clock_hz("v3d")}\ntemperature_millic={temperature_millic()}\nmem_available_kib={mem_available_kib()}\n')
    return rc

if __name__=='__main__':
    raise SystemExit(main())
