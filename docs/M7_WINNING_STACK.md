# Canonical CinePi M7 Winning Stack

> **Dated record, not a specification.** The figures below were measured on one
> CM5, one scene and one software state, on the date given. Throughput, file size
> and compression ratio are properties of a scene and a machine: a noisy frame
> compresses less at every quality mode, and a different Pi, kernel, thermal
> state or desktop session moves the numbers again. They are kept so a change can
> be compared against the same conditions — do not read them as what your rig
> will do, and do not quote them as limits.


The production encoder is **one immutable implementation**: the SHA-pinned v1.16.6 `vc5_bench.cpp` source, reused by the live encoder library.

Standard runtime stack:

- original v1.16.6 winning encoder settings;
- VLE prefetch distance **128**, locality **0**;
- movable IRQ affinities isolated to **Core0** while encoding;
- fused NEON v53 + fused split;
- non-temporal coefficient writes;
- v2 sidecar, sidecar zero-skip, handoff pool, sign LUT, scan8, 64-bit accumulator, input prefetch;
- shared in-place QRAW output;
- 20 ms cyclic wavelet rendezvous with 40 us release lead for 2+ owners;
- selector 4 = three owners on cores 1/2/3 plus Core0 SB8x4 entropy assist.

There are no A/B encoders, Vulkan encoder variants, historical CPU-wavelet labs, fallback encoder implementations, or optimisation sweep runners in this package. Unsupported fused geometry fails closed instead of selecting another encoder path.
