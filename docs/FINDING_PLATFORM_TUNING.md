# Settled: what is left in the Pi 5 platform, and what is already done

> **Dated record, not a specification.** Measured 2026-08-17 on this CM5 (8 GB,
> kernel 6.12.96+rpt-rpi-2712, 16 KB pages), one scene, one software state.
> Throughput is a property of a scene and a machine. The figures are kept so a
> change can be compared against the same conditions — do not read them as what
> another rig will do.

The encoder campaign closed with the pipeline **DRAM-bandwidth bound, with Core 0
as the busiest core**. This is the platform-side follow-up: kernel, firmware,
drivers and configuration, asked to give back bandwidth or stop wasting it.

One thing was found, and it is worth more than everything else here combined.

---

## FOUND: an attached display costs ~7 % of encode throughput, continuously

A connected HDMI output scans its framebuffer out of DRAM every frame, forever,
whether anything is drawn or not. On this box that is a 3440x1440 desktop at
60 Hz:

    3440 x 1440 x 4 bytes x 60 Hz = 1.19 GB/s of DRAM read, permanently

Measured on the static path — fixed input, so this is the trustworthy
instrument — m5, 4 workers, 6 s per pass, 3 passes:

| display | fps | pass spread | p50 ms | p99 ms |
|---|---|---|---|---|
| on, 3440x1440@60 | 53.83 | 1.50 | 54.0 | 67.8 |
| **off (CRTC disabled)** | **57.83** | **0.50** | 51.5 | **56.8** |

**+4.00 fps, +7.4 %**, against a noise floor of 0.50–1.50 fps. Two other things
improved with it:

- **p99 frame latency fell 16 %** (67.8 → 56.8 ms). Scanout is periodic
  contention, so it shows up in the tail, not just the mean.
- **Run-to-run spread collapsed 3x** (1.50 → 0.50 fps). The display was also the
  largest single source of measurement variance on this machine, which matters
  for every A/B anyone runs here in future.

Expect it to matter at least as much on live capture, which sits closer to the
bandwidth ceiling than the static path does.

### It is the scanout, not the compositor

The obvious explanation — labwc, Xwayland and the panel stealing CPU from the
encoder cores — was tested and **refuted**. Pinning every desktop process to
Core 0 with the display still on:

| configuration | fps | spread |
|---|---|---|
| baseline, desktop free on all cores | 53.83 | 1.50 |
| desktop pinned to Core 0 | 52.94 | 2.67 |

No gain, and slightly worse. It is *expected* to be worse at 4 workers: Core 0 is
already running the SB8x4 entropy assist, so moving the compositor there puts it
in the way of the encoder rather than out of it. **Do not pin the desktop to
Core 0.** The cost is DMA bandwidth, and no amount of scheduling fixes DMA.

### What to do about it

In order of how much they give back:

1. **Record with no display attached**, or with the output asleep. This recovers
   the whole 7.4 %.
2. **Let it sleep by itself.** `swayidle` and `wlopm` are both installed; a short
   idle timeout DPMS-offs the output during an unattended run and it comes back
   on a keypress. This is the cheapest way to get the gain without changing how
   the box is used.
3. **Shrink the mode if a screen must stay awake.** Scanout scales with
   pixels x refresh, so 1920x1080@60 is 0.50 GB/s instead of 1.19 — about 60 % of
   the tax gone for a smaller desktop. `video=HDMI-A-1:1920x1080@60` on the
   kernel command line pins it. (Not applied here: it changes the desktop you
   look at, so it is your call.)

Turning the display off with `wlopm --off HDMI-A-1` is not suitable *during* a
run you are watching in the terminal UI — the UI is on that screen.

---

## Applied: two kernel command-line tokens

`/boot/firmware/cmdline.txt`, backed up first as `cmdline.txt.bak-<stamp>`:

    irqaffinity=0 mitigations=off

- **`irqaffinity=0`** makes the package's IRQ isolation policy the boot default
  instead of something re-applied per run. The runners already move every movable
  interrupt to Core 0 and restore it afterwards; this covers interrupts set up
  before the first run and any that appear later.
- **`mitigations=off`** drops the Cortex-A76 branch-history clearing on kernel
  entry (`spectre_v2: Mitigation: CSV2, BHB` today). The camera path crosses into
  the kernel constantly — ioctls, interrupts, dma-buf — so this is where such a
  cost would land.

### Measured after the reboot (2026-08-17, display ON, static m5 3-pass)

Both tokens confirmed live in `/proc/cmdline`, `default_smp_affinity` = `1`, and
`spectre_v2` moved from `Mitigation: CSV2, BHB` to `Mitigation: CSV2, but not
BHB`. Three independent legs of the exact baseline command:

| leg | fps | pass spread | p50 ms | p99 ms |
|---|---|---|---|---|
| 1 | 56.00 | 1.00 | 52.9 | 59.8 |
| 2 | 56.00 | 1.50 | 53.0 | 58.5 |
| 3 | 56.33 | 0.50 | 52.8 | 57.6 |
| **mean** | **56.11** | — | 52.9 | 58.6 |

Against the 53.83 fps display-ON baseline: **+2.28 fps, +4.2 %**. Leg-to-leg
variation was 0.33 fps — tighter than the within-run spread — so the gain clears
the noise floor by a wide margin and survived the repeat that killed the earlier
writeback "win". The tail improved more than the mean: p99 67.8 → 58.6 ms
(−13.6 %), consistent with removing a per-kernel-entry cost that shows up as
jitter rather than as steady throughput.

### The gain is entirely on the display-ON path

Same command, display OFF via `wlopm`, three legs: 57.50 / 58.00 / 57.67, mean
**57.72** against the 57.83 display-OFF baseline — a 0.11 fps difference, inside
the noise floor. So:

| configuration | pre-reboot | post-reboot | delta |
|---|---|---|---|
| display ON | 53.83 | **56.11** | +2.28 (+4.2 %) |
| display OFF | 57.83 | **57.72** | −0.11 (noise) |
| **display cost** | **7.4 %** | **2.9 %** | penalty cut by ~60 % |

The tokens bought nothing when there was no display, and +4.2 % when there was.
Two readings fit this data and it cannot separate them:

1. Part of the display cost is *not* raw scanout bandwidth but the kernel-entry
   and interrupt work that comes with a compositor waking at 60 Hz — which is
   exactly what these two tokens reduce. This would revise the earlier conclusion
   that "the display cost is DMA bandwidth; scheduling cannot fix it".
2. ~57.8 fps is a hard ceiling on the static m5 path, so the tokens' headroom has
   nowhere to show once the display is not stealing. The display-ON figure would
   then be the tokens recovering ground toward a ceiling they cannot exceed.

Distinguishing them needs a configuration that is below the ceiling for a reason
other than the display — the live path is the obvious candidate.

**Methodology correction:** §2 of the handoff recommends display-OFF as the clean
instrument for "anything subtle" because of its 0.50 fps floor. For *these tokens*
that is exactly wrong — display-OFF is quiet but blind to them. Attribution and
any further cmdline work must be measured display-ON.

**Caveats, both real:**

- ~~**The two tokens are not separated.**~~ **Now separated** — see the
  attribution section immediately below. This block's numbers remain the
  both-tokens reference.
- **Machine state differed.** The post-reboot legs ran on a freshly booted box
  (3 min uptime); the baseline did not. Thermals were comparable (53 °C vs ~54 °C,
  `throttled=0x0` both) and the desktop was idle in both, so this is unlikely to
  account for 2.28 fps — but it is not controlled for.

The standing rule still applies to `mitigations=off` specifically: it is a genuine
reduction in speculative-execution hardening, and the +4.2 % measured here belongs
to the *pair*. Until it is attributed, the token is carried on a shared result, not
its own.

Revert either by removing it from `cmdline.txt`, or restore the backup wholesale.

### ATTRIBUTED: `irqaffinity=0` carries about 60 % of the gain, `mitigations=off` the rest

2026-08-17 evening, second reboot. `mitigations=off` removed from `cmdline.txt`,
`irqaffinity=0` kept. Verified live: no `mitigations` token in `/proc/cmdline`,
`spectre_v2` back to `Mitigation: CSV2, BHB`, full hardening restored. Same exact
command, display ON, static m5 3-pass, three legs, `pkill swayidle` first, 2 min
uptime, 48.8 °C at start and 58.7 °C at the end, `throttled=0x0` throughout.

| leg | fps | pass spread |
|---|---|---|
| 1 | 55.50 | 0.00 |
| 2 | 55.00 | 2.00 |
| 3 | 55.11 | 1.17 |
| **mean** | **55.20** | — |

| configuration | display ON | delta vs no tokens | share of the +4.2 % |
|---|---|---|---|
| neither token | 53.83 | — | — |
| **`irqaffinity=0` only** | **55.20** | **+1.37 (+2.5 %)** | **~60 %** |
| both tokens | 56.11 | +2.28 (+4.2 %) | 100 % |

So this is the middle outcome the handoff's decision rule allowed for: **both
tokens contribute**. `irqaffinity=0` is the larger half and is free — it stays.
`mitigations=off` is worth a further **+0.91 fps (+1.7 %)** on top, and that is
the number to weigh against the hardening it gives up.

**How solid is the 0.91 fps?** Leg-to-leg spread here was 0.50 fps (55.00–55.50),
against 0.33 fps on the both-tokens legs, so 0.91 is roughly twice the leg-to-leg
noise — probably real, but it is a small effect measured close to the instrument's
floor, and much weaker evidence than the +2.28 fps of the pair. Within-leg pass
spread was also worse today (up to 2.00 fps vs 1.50), and the p99 tail was noisier
(66.2 / 64.1 ms on legs 2–3). Treat +1.7 % as an upper-ish estimate.

**Not measured:** `mitigations=off` alone. Isolating whether the two interact
(rather than simply adding) needs a third reboot. Given that the residual is
1.7 % and the noise is 0.5–1 fps, the answer would be at the edge of what this
instrument can resolve — not obviously worth a reboot.

**Current state of the box: `irqaffinity=0` only, full speculative-execution
hardening.**

**DECIDED by the user, 2026-08-17 evening: leave `mitigations=off` out
permanently.** 1.7 % is not worth giving up the branch-history clearing on this
box. `irqaffinity=0` is kept. **The kernel-token question is closed — do not
re-open it without a new reason.** If it ever is revisited, restore
`cmdline.txt.bak-20260817-151501` and reboot.

---

## Confirmed on the LIVE path: the display cost is real, and it is headroom, not frames

2026-08-17, both tokens active. `SOURCE=camera INPUT_BITS=16
CINEPI_CLEARHDR_BITS=12 MODES=m5 DURATION=20 PASSES=1 THREADS=3`, six legs
alternating display ON/OFF so thermal drift could not bias one configuration.
Exposure was pinned automatically — that configuration defaults to
`CINEPI_AE=fixed`, `CINEPI_GAIN=1` (ISO 100), shutter 16666 us — and the scene
was held static.

**Every leg captured at 29.999 fps with `encoder_drops=0`.** The live path is
**sensor-capped at 30 fps** (the IMX585 3856x2180 mode's maximum), so encoder fps
cannot express the display cost: 30.21 ON vs 30.11 OFF is the cap, not a result.
The per-worker stage timings express it instead, and cleanly:

| metric | display ON | display OFF | cost of the display |
|---|---|---|---|
| `fps` | 30.21 | 30.11 | none — both at the sensor cap |
| `ms_wavelet` | 40.69 | 31.94 | **+27.4 %** |
| `ms_entropy` | 30.56 | 28.59 | +6.9 % |
| **`ms_frame_per_worker`** | **71.25** | **60.53** | **+17.7 %** |
| `sequence_gaps` | 48 / 48 / 45 | 41 / 41 / 41 | +15 % |

The groups do not overlap: ON spans 70.92–71.50 ms, OFF spans 60.42–60.65 ms, a
10.3 ms gap against a within-group range of 0.58 ms. Pinned exposure plus a static
scene removed the variance that made the earlier live attempt useless.

**What this means:** the display costs about a sixth of the encoder's per-frame
work budget on the live path — larger in work terms than the 7.4 % it cost static
throughput. But at 30 fps that budget is not exhausted, so the cost is paid out of
headroom and thermal margin, not out of frames. It becomes real if the capture
rate or resolution rises, or if a future mode pushes past the current cap.

The wavelet stage carrying most of the cost (+27.4 %, against +6.9 % for entropy)
is consistent with scanout DMA contending for memory bandwidth with the stage that
touches the most memory — i.e. it supports the original "it is the scanout"
conclusion rather than overturning it.

---

## Applied: swayidle blanks the output after 10 minutes idle

2026-08-17. `~/.config/labwc/autostart`, 600 s timeout, `wlopm --off HDMI-A-1` on
timeout and `--on` on resume. Verified working by running the same command with an
8 s timeout: the output blanked and restored. Running now and on every future
login.

### CORRECTION 2026-08-17 evening: labwc **does** run both autostart files

The claim originally recorded here — that labwc uses only the first autostart file
found across the XDG config dirs, so `~/.config/labwc/autostart` *shadows*
`/etc/xdg/labwc/autostart` — is **wrong on this box** (labwc 0.8.4-1+rpt1). It runs
**both**. That claim was never actually tested: the user file was written as a
verbatim copy of the four system lines plus the swayidle block, and the desktop was
not restarted afterwards, so nothing duplicated until the next boot.

The first reboot after it exposed the error. Every shared line started twice —
`pcmanfm --desktop` ×2, `wf-panel-pi` ×2, `kanshi` ×2 — while `swayidle`, which
appears only in the user file, started once. Two panels stacked on the same output
is what the user saw and reported as "the top bar showing twice".

`lwrespawn` does not protect against this, though it looks like it should: its
guard is `pgrep $1`, i.e. `pgrep /usr/bin/wf-panel-pi`, and pgrep matches against
the process *name* `wf-panel-pi`, so the pattern never matches an existing
instance. Both copies also launch in the same second, so the check would lose the
race regardless.

**Fixed** by cutting the four copied lines back out of `~/.config/labwc/autostart`,
leaving only the swayidle block (previous version kept as
`autostart.bak-dupe-panel-20260817`), and killing the live duplicates. Verified:
one `pcmanfm`, one `wf-panel-pi`, one `kanshi`, one `swayidle`.

**So the rule is the opposite of what was written:** put *only* user additions in
`~/.config/labwc/autostart`, never a copy of the system file — and there is nothing
to re-sync when an OS update changes `/etc/xdg/labwc/autostart`.

**Also a measurement note:** a duplicate `wf-panel-pi` is a second 60 Hz-waking
compositor client and therefore a throughput confound. It was killed *before* the
attribution legs above, so those numbers are unaffected — but any measurement taken
between the first reboot and that fix would have been.

**MEASUREMENT WARNING — new confound as of today.** If swayidle fires part-way
through a benchmark, throughput changes mid-run and the result is silently wrong.
The 600 s timeout was chosen so it cannot fire while an operator reads terminal
output or watches a short run, but any long or unattended measurement must either
account for it or stop it first:

```bash
pkill swayidle      # restore by re-logging in, or re-running the autostart line
```

---

## Already optimal — do not re-open these

Most of the platform was already right, and some of it was never ours to claim:

**Set in `config.txt` on this box:** `arm_freq=2800` with
`over_voltage_delta=50000` and `arm_boost=1`; `dtoverlay=rp1-400mhz` (the I/O
bridge the CSI-2 capture crosses); `dtparam=pciex1_gen=3`; a fan curve that keeps
the SoC at ~54 °C with `throttled=0x0` under load; `kernel=kernel_2712.img`
(16 KB pages, A/B'd to neutral).

**Injected by the firmware itself, not configured here** — worth knowing, because
it looks like local tuning and is not: `numa=fake=8`, `numa_policy=interleave`,
`iommu_dma_numa_policy=interleave`, `system_heap.max_order=0`,
`cgroup_disable=memory`, `nvme.max_host_mem_size_mb=0`, `coherent_pool=1M`,
`pci=pcie_bus_safe`. The bootloader sets these on BCM2712; the fake-NUMA
interleave is the platform's own DRAM-bandwidth measure. `cmdline.txt` itself is
still the file as imaged.

**Ruled out, with the reason:**

| candidate | why not |
|---|---|
| `nohz_full=1-3`, `rcu_nocbs=1-3` | `# CONFIG_NO_HZ_FULL is not set` in this kernel — the tokens would be silently ignored. Needs a custom kernel; not worth it given CPU contention was refuted above. |
| `isolcpus=1-3` | Would work, but the thing it fixes is not the bottleneck, and it would confine parallel builds and the static benchmark to one core. |
| Disabling deeper CPU idle states | There are none. BCM2712 exposes no `cpuidle` states here — WFI only, nothing to disable. |
| `pci=pcie_bus_perf` | The NVMe link already negotiates its maximum: `MaxPayload 512 bytes` (device capability), Gen 3 8 GT/s x1. Nothing left. |
| Transparent huge pages | Not available on this kernel, and 16 KB base pages already cut TLB pressure 4x against 4 KB. |
| Reclaim tuning (`min_free_kbytes`, `watermark_scale_factor`) | The counters say it is not needed: `allocstall_movable 3` and `pgscan_direct 1070` since boot, against `pgscan_kswapd 1698521`. kswapd is doing essentially all of it; direct reclaim is not on the capture path. |
| Larger CMA | `CmaFree` runs low but nothing has failed: no CMA or dma-buf allocation failures in the log at all. The camera path uses the dma-buf system heap, not CMA. |
| More overclock | The pipeline is DRAM-bound, so ARM clock is not the limit — and this is a recorder. A marginal clock that corrupts one frame in a take is worth less than 2 % of anything. |
| Writeback tuning | Tested and **not established** — see below. |

---

## Tested and NOT established: dirty-writeback tuning

The default `vm.dirty_background_ratio=10` / `vm.dirty_ratio=20` on 8 GB means
writeback starts at ~0.8 GB dirty and writers block at ~1.6 GB, so a sustained
capture accumulates and then flushes in bursts. Replacing the cliff with
continuous writeback (`dirty_background_bytes=64M`, `dirty_bytes=256M`) looked
like a clear win on the first pair, and did not survive a repeat.

Live RAW12, m1, 20 s, 3 workers, ~3.7 GB written per leg — well past the
threshold:

| config | fps per leg | mean | range |
|---|---|---|---|
| default | 26.67, 28.07, 27.43 | 27.39 | 1.40 |
| 64 MB / 256 MB | 28.46, 27.97 | 28.22 | 0.49 |

The tuned mean is higher, but the *default* configuration's own spread covers the
difference. The confound is visible in the same runs: `capture_fps_actual`
wandered from 27.5 to 36.9 fps across identical configurations, because auto 3A
re-exposes, the scene noise changes, and the data rate changes with it. **Left at
the defaults.**

If it is worth resolving, do it on the static path with `SAVE_QRAW=on` and the
display off — deterministic input and a 0.50 fps noise floor — not on live
camera, where the scene moves more than the effect does.

---

## Noted, not acted on

- **NVMe interrupts land on the encoder cores and cannot be moved.** `nvme0q2`,
  `nvme0q3` and `nvme0q4` are bound to CPU 1, 2 and 3 — blk-mq *managed*
  interrupts, which reject affinity changes from userspace, so the runners' IRQ
  isolation cannot include them and `irqaffinity=0` will not either. The measured
  load is small (`BLOCK` softirqs in the dozens; queue interrupts far below the
  timer), so this is recorded as a known limit rather than a problem. If it ever
  matters, the lever is the number of I/O queues the driver creates, not affinity.
  **Checked 2026-08-17: that lever does not exist here.** `nvme` is builtin and
  its parameters are `use_threaded_interrupts`, `use_cmb_sqes`,
  `max_host_mem_size_mb`, `sgl_threshold`, `io_queue_depth`, `write_queues`,
  `poll_queues`, `noacpi` — `write_queues`/`poll_queues` only *add* queue sets,
  and there is no `nr_io_queues`. The device runs 4 hardware queues (`mq/0..3`,
  5 `nvme0q*` interrupt lines including admin). Closed: no lever, negligible load.
- **Swap is enabled** (`/var/swap`, 200 MB, `vm.swappiness=60`) and has been used
  since boot. Irrelevant at 6 GB available, but the frame-target test deliberately
  allocates an overflow buffer sized to installed RAM less 1 GB — which is exactly
  the condition that would page something out mid-recording. Consider `swapoff -a`
  before long record runs.
- **`/media/RAW` is the root filesystem** (`/dev/nvme0n1p2` mounted twice), so
  saved frames and the OS share one ext4. Already `noatime`.
