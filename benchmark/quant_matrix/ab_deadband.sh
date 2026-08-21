#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Kieran Olsson
# A/B: control vs static dead-band elision.
#
# dead_hh fires when a level's HH quantiser sends every int16 to zero. Band
# indices per level are base = 10 - 3*level, so the HH entries are 3 (L3),
# 6 (L2) and 9 (L1). M_0766 prunes only entry 9, which is level 1 -- the
# largest level, and therefore the best case for this optimisation.
#
# Three tables:
#   m7        nothing dead  -> REGRESSION CHECK, must not get slower
#   M_0766    entry 9 dead  -> the shipping candidate
#   allHH     3,6,9 dead    -> the ceiling of this optimisation
set -uo pipefail
cd "$(dirname "$0")/.."
Q=quant_matrix
IN=cinepi_qraw_bench/input/sample_imx585_3840x2160_gbrg_12bit_gplog2.raw16
DUR=${DUR:-4}; REPS=${REPS:-3}; TH=${TH:-3}

declare -A T=(
 [m7]="1=14,2=14,3=8,4=26,5=26,6=18,7=126,8=126,9=175"
 [M_0766]="1=14,2=14,3=16,4=26,5=26,6=36,7=126,8=126,9=32767"
 [allHH]="1=14,2=14,3=32767,4=26,5=26,6=32767,7=126,8=126,9=32767"
)

echo "=== bit-exactness gate ==="
for k in m7 M_0766 allHH; do
  a=$(mktemp -d -p /dev/shm); b=$(mktemp -d -p /dev/shm)
  for pair in "control:$a" "deadband:$b"; do
    bin=${pair%%:*}; dir=${pair##*:}
    CINEPI_BAND_Q="${T[$k]}" "$Q/vc5_bench.$bin" --input "$IN" --input-companded on \
      --width 3840 --height 2160 --duration 0 --frames 1 --warmup 1 --mode m7 \
      --execution cpu-gpr --cpu-gpr-threads 1 --save-gpr "$dir" >/dev/null 2>&1
  done
  ha=$(md5sum "$a"/*.gpr 2>/dev/null | awk '{print $1}')
  hb=$(md5sum "$b"/*.gpr 2>/dev/null | awk '{print $1}')
  sz=$(stat -c%s "$a"/*.gpr 2>/dev/null)
  [[ -n "$ha" && "$ha" == "$hb" ]] \
     && echo "  $k  IDENTICAL ($sz bytes)" \
     || echo "  $k  *** DIFFERS *** $ha vs $hb"
  rm -rf "$a" "$b"
done

echo
echo "=== ms_wavelet and fps, interleaved, $REPS reps, $TH threads ==="
printf "%-8s %-24s %-24s\n" table "wavelet ctl->dead" "fps ctl->dead"
for k in m7 M_0766 allHH; do
  cw=""; dw=""; cf=""; df=""
  for _ in $(seq "$REPS"); do
    for bin in control deadband; do
      r=$(CINEPI_BAND_Q="${T[$k]}" "$Q/vc5_bench.$bin" --input "$IN" --input-companded on \
          --width 3840 --height 2160 --duration "$DUR" --mode m7 --execution cpu-gpr \
          --cpu-gpr-threads "$TH" 2>/dev/null | grep "^CPU_QRAW_RESULT")
      w=$(echo "$r" | grep -oE 'ms_wavelet=[0-9.]+' | cut -d= -f2)
      f=$(echo "$r" | grep -oE ' fps=[0-9.]+' | cut -d= -f2)
      if [[ $bin == control ]]; then cw="$cw $w"; cf="$cf $f"; else dw="$dw $w"; df="$df $f"; fi
    done
  done
  python3 -c "
def m(s):
    v=[float(x) for x in s.split()]
    return sum(v)/len(v) if v else 0.0
cw,dw,cf,df = m('$cw'),m('$dw'),m('$cf'),m('$df')
dwp = (dw/cw-1)*100 if cw else 0
dfp = (df/cf-1)*100 if cf else 0
print(f'{\"$k\":<8} {cw:7.2f} -> {dw:7.2f} ({dwp:+6.2f}%)  {cf:6.2f} -> {df:6.2f} ({dfp:+6.2f}%)')"
done
