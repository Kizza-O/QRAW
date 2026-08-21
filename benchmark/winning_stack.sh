#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Kieran Olsson
# CinePi production winner policy, v1.16.6 + IRQ isolation + VLE128.
# This is the ONLY supported production encoder stack in this package.
# Source this file, call cinepi_winning_stack_env "$THREADS", then
# cinepi_irq_isolation_apply before timed/live encode work.

cinepi_winning_stack_env() {
  local selected="${1:-3}"
  # 1..3 = that many full-frame owners; 4 = three owners + the Core0 SB8x4
  # entropy assistant; 5 = four full-frame owners, one per core (RAW12/60
  # experiment -- see the selector note in qraw_encoder.cpp).
  [[ "$selected" =~ ^[1-5]$ ]] || { echo "FATAL: winner worker selector must be 1..5" >&2; return 2; }
  export WINNER=on
  # These two are exported unconditionally, which silently overwrote anything
  # the caller had already set -- so CINEPI_WAVELET_RENDEZVOUS_US=50000 on the
  # command line was accepted, ignored, and still reported as 20000 in
  # CPU_WAVELET_CONCURRENCY. The canonical winner values are unchanged; the
  # CINEPI_TUNE_* variables are the opt-in way to sweep them without editing
  # this file, and the run banner reports whatever actually took effect.
  export CINEPI_WAVELET_RELEASE_LEAD_US="${CINEPI_TUNE_RELEASE_LEAD_US:-40}"
  if (( selected >= 2 )); then
    export CINEPI_WAVELET_RENDEZVOUS_US="${CINEPI_TUNE_RENDEZVOUS_US:-20000}"
  else
    export CINEPI_WAVELET_RENDEZVOUS_US=0
  fi

  # Selector 4 is three frame owners plus the measured Core0 SB8x4 helper.
  if (( selected == 4 )); then
    export CINEPI_ENTROPY_ASSIST_SUBBAND=8
    export CINEPI_ENTROPY_ASSIST_CHANNELS=4
    export CINEPI_ENTROPY_ASSIST_SCHED=normal
    export CINEPI_ENTROPY_ASSIST_NICE=0
    export CINEPI_ENTROPY_ASSIST_VERIFY=8
  else
    unset CINEPI_ENTROPY_ASSIST_SUBBAND CINEPI_ENTROPY_ASSIST_CHANNELS \
          CINEPI_ENTROPY_ASSIST_SCHED CINEPI_ENTROPY_ASSIST_NICE \
          CINEPI_ENTROPY_ASSIST_VERIFY
  fi

  # Encoder-side VLE is hard locked to 128/locality0 in the canonical wrapper
  # and standalone runner. Export an identity token for logs/integration gates.
  export CINEPI_WINNING_STACK_ID='v1.16.6+irq-core0+vle128'
}

cinepi_winning_stack_desc() {
  local selected="${1:-3}"
  case "$selected" in
    1) echo "1 full owner; Core0 IRQ/control isolation; VLE128" ;;
    2) echo "2 full owners; synchronized wavelet burst; Core0 IRQ/control isolation; VLE128" ;;
    3) echo "3 full owners; synchronized wavelet burst; Core0 IRQ/control isolation; VLE128" ;;
    4) echo "3 full owners + Core0 SB8x4 entropy assist; IRQ/control isolation; VLE128" ;;
    *) echo "invalid worker selector" ;;
  esac
}

# Move every writable IRQ affinity to Core0. This is an external scheduling
# condition of the measured winner, not another encoder implementation.
# The original affinity list is restored by cinepi_irq_isolation_restore.
cinepi_irq_isolation_apply() {
  if [[ "${CINEPI_IRQ_ISOLATION_ACTIVE:-0}" == 1 ]]; then
    echo "IRQ_ISOLATION baseline=ON target_core=0 status=already-active"
    return 0
  fi
  [[ "$(uname -s)" == Linux ]] || { echo "FATAL: IRQ isolation requires Linux" >&2; return 2; }
  local snap="${1:-${TMPDIR:-/tmp}/cinepi_irq_affinity_$$.tsv}"
  local changed=0 pinned=0 seen=0 auth_tried=0 f cur after
  : > "$snap"

  (( EUID == 0 )) || command -v sudo >/dev/null 2>&1 || {
    echo "FATAL: sudo is required for the winning IRQ isolation" >&2; rm -f "$snap"; return 2; }

  for f in /proc/irq/[0-9]*/smp_affinity_list; do
    [[ -r "$f" ]] || continue
    cur=$(cat "$f" 2>/dev/null || true); [[ -n "$cur" ]] || continue
    printf '%s\t%s\n' "$f" "$cur" >> "$snap"
    seen=$((seen+1))
    if [[ "$cur" == 0 ]]; then pinned=$((pinned+1)); continue; fi
    if (( EUID == 0 )); then
      printf '0\n' > "$f" 2>/dev/null || true
    else
      if ! printf '0\n' | sudo -n tee "$f" >/dev/null 2>&1; then
        if (( auth_tried == 0 )) && [[ -t 0 || -t 1 || -t 2 ]]; then
          auth_tried=1
          sudo -v >/dev/null 2>&1 || true
          printf '0\n' | sudo -n tee "$f" >/dev/null 2>&1 || true
        fi
      fi
    fi
    after=$(cat "$f" 2>/dev/null || true)
    if [[ "$after" == 0 ]]; then changed=$((changed+1)); pinned=$((pinned+1)); fi
  done
  (( seen > 0 && pinned > 0 )) || {
    echo "FATAL: no IRQ affinity could be confirmed on Core0; refusing non-winning runtime" >&2
    rm -f "$snap"; return 2
  }
  export CINEPI_IRQ_ISOLATION_ACTIVE=1
  export CINEPI_IRQ_SNAPSHOT="$snap"
  echo "IRQ_ISOLATION baseline=ON target_core=0 changed_affinities=$changed pinned_affinities=$pinned restore_on_exit=YES"
}

cinepi_irq_isolation_restore() {
  [[ "${CINEPI_IRQ_ISOLATION_ACTIVE:-0}" == 1 ]] || return 0
  local snap="${CINEPI_IRQ_SNAPSHOT:-}" f old
  if [[ -n "$snap" && -f "$snap" ]]; then
    while IFS=$'\t' read -r f old; do
      [[ -n "$f" && -n "$old" && -e "$f" ]] || continue
      if (( EUID == 0 )); then printf '%s\n' "$old" > "$f" 2>/dev/null || true
      else printf '%s\n' "$old" | sudo -n tee "$f" >/dev/null 2>&1 || true
      fi
    done < "$snap"
    rm -f "$snap"
  fi
  unset CINEPI_IRQ_ISOLATION_ACTIVE CINEPI_IRQ_SNAPSHOT
  echo "IRQ_ISOLATION restored=YES"
}
