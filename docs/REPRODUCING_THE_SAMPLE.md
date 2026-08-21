# The sample input in this package is the real data, not a placeholder

> **Dated record, not a specification.** The figures below were measured on one
> CM5, one scene and one software state, on the date given. Throughput, file size
> and compression ratio are properties of a scene and a machine: a noisy frame
> compresses less at every quality mode, and a different Pi, kernel, thermal
> state or desktop session moves the numbers again. They are kept so a change can
> be compared against the same conditions — do not read them as what your rig
> will do, and do not quote them as limits.


`cinepi_qraw_bench/input/sample_imx585_3840x2160_gbrg_16bit.raw16` in this
zip is byte-for-byte identical to the sample this whole project has used
throughout — verified via SHA256, not assumed.

## How it was reconstructed

`cinepi_qraw_bench/input/SAMPLE_INPUT.json` (already part of the package)
documents the sample's exact provenance:

- Source: a public 12-bit GBRG DNG at
  `https://github.com/yl-data/yl-data.github.io/blob/master/2201.process_raw/raw-12bit-GBRG.dng`
  (4024x3036, 12-bit, uncompressed TIFF/DNG)
- Crop: `[x=92, y=438, width=3840, height=2160]`
- Storage: little-endian uint16, values stored directly (0-4095), not
  bit-shifted to fill the 16-bit container
- Documented SHA256 of the resulting file: `ac57b81d...0ffb6d6`

Reconstruction steps:
1. Fetched the source DNG from the documented URL.
2. Confirmed it independently via `file`: TIFF, little-endian, 4024x3036,
   12 bits/sample, uncompressed — matching `SAMPLE_INPUT.json` exactly
   before any pixel data was even touched.
3. Decoded the 12-bit packed sensor data with `tifffile` (+ `imagecodecs`
   for the 12-bit unpacking).
4. Applied the documented crop `[92, 438, 3840, 2160]`.
5. Checked the cropped region's min/max against the documented values
   (`min=221, max=4095`) — exact match, confirming the crop coordinates
   and source file were both correct before writing anything to disk.
6. Wrote the cropped data as little-endian uint16, no scaling.
7. Computed the SHA256 of the result: **exact match** to the value
   documented in `SAMPLE_INPUT.json`.

This is the same sample data this project's benchmark numbers (v21 through
v90z) were measured against — not a synthetic substitute standing in for
it.

## Reproducing it yourself

If you ever need to regenerate it (e.g. the file is somehow lost or
corrupted):

```python
import tifffile, numpy as np
# pip install tifffile imagecodecs --break-system-packages

arr = tifffile.TiffFile("raw-12bit-GBRG.dng").pages[0].asarray()
x0, y0, w, h = 92, 438, 3840, 2160
cropped = arr[y0:y0+h, x0:x0+w].astype('<u2')
cropped.tofile("sample_imx585_3840x2160_gbrg_16bit.raw16")
```

Then verify: `sha256sum sample_imx585_3840x2160_gbrg_16bit.raw16` should
print `ac57b81dd7e860f4df896eecdaa2737f40ef4f7908d7ec91a2defff6b0ffb6d6`.
