# Reference input frame

`sample_imx585_3840x2160_gbrg_16bit.raw16` is **not in this repository** — at
16 MB it does not belong in git history. It is attached to the GitHub Release
for this version.

    cd benchmark/cinepi_qraw_bench/input
    curl -LO https://github.com/<you>/qraw-encoder/releases/download/v4.0/sample_imx585_3840x2160_gbrg_16bit.raw16
    sha256sum -c SHA256SUM.txt

Format: 3840x2160, 16-bit Bayer, GBRG, tightly packed, one sample per uint16,
MSB-justified. 12-bit GP-Log2 companded values. IMX585.

Run parameters are in `SAMPLE_INPUT.json`.
