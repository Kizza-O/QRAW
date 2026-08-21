# CinePi Production Winning Stack v1.16.6

> **Dated record, not a specification.** The figures below were measured on one
> CM5, one scene and one software state, on the date given. Throughput, file size
> and compression ratio are properties of a scene and a machine: a noisy frame
> compresses less at every quality mode, and a different Pi, kernel, thermal
> state or desktop session moves the numbers again. They are kept so a change can
> be compared against the same conditions — do not read them as what your rig
> will do, and do not quote them as limits.


This package has one production encoder standard only:

**v1.16.6 winning stack + IRQ isolation to Core0 + VLE128/locality0**.

All original winning-stack settings are retained:

- fused NEON v53 + fused split;
- non-temporal coefficient writes;
- v2 sidecar + sidecar zero-skip;
- stride split and handoff pool;
- VLE sign LUT, scan8 and 64-bit accumulator;
- input prefetch;
- shared verified in-place QRAW output;
- 20 ms cyclic wavelet rendezvous and 40 us release lead for 2+ owners;
- selector 4 = three full frame owners on cores 1/2/3 plus Core0 SB8x4 entropy assist.

The runtime also moves movable IRQ affinities to Core0 for encode work and restores their previous affinities on exit.

There are no production A/B encoder variants, historical shader encoders, CPU-wavelet labs, VLE192/VLE208 stacks, low-pass/LUT experiment stacks, or fallback encoder implementations in this package. The live wrapper fails closed if the canonical fused-split/NEON requirements cannot be met.

## Production Quant Ladder

The sole production quant ladder is **CinePi Universal Standard Quant v3**. Band order is `LL / L3-LH / L3-HL / L3-HH / L2-LH / L2-HL / L2-HH / L1-LH / L1-HL / L1-HH`.

- M1: `1/4/4/2/8/8/6/26/26/39`
- M2: `1/4/4/2/8/8/6/33/33/50`
- M3: `1/4/4/2/10/10/6/43/43/66`
- M4: `1/6/6/4/14/14/10/55/55/87`
- M5: `1/8/8/4/16/16/11/73/73/111`
- M6: `1/10/10/6/20/20/14/93/93/143`
- M7: `1/14/14/8/26/26/18/126/126/175`
- M8: `1/18/18/9/34/34/22/155/155/235`
- M9: `1/22/22/13/44/44/30/201/201/301`
- M10: `1/29/29/13/45/45/35/272/272/521`

No Previous/v19 ladder or runtime ladder selector is retained.
