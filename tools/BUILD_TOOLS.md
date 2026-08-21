# Building the helper binaries

`gpr_tools`, `dng_validate`, `qraw_stats_cli` and `gpr_view` are **not committed**.
They compile Adobe DNG SDK and GoPro GPR code; shipping the binaries would pull
in the binary-form redistribution clauses of both BSD licences for no benefit,
since anyone can build them in a minute.

    # gpr_tools and dng_validate come from the GPR submodule
    cd benchmark/third_party/gpr
    mkdir -p build && cd build
    cmake .. && make -j4
    cp source/app/gpr_tools/gpr_tools ../../../../tools/

    # qraw_stats_cli
    ./tools/build_qraw_stats_cli.sh

    # gpr_view
    c++ -O2 -o benchmark/quant_matrix/gpr_view benchmark/quant_matrix/gpr_view.cpp

If you would rather ship them, that is permitted — you just have to include the
Adobe and GoPro licence texts alongside the binaries, which a GitHub Release
body can do.
