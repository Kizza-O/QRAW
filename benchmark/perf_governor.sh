#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Kieran Olsson
# ===========================================================================
# perf_governor.sh -- sourced, not executed.
#
# A benchmark that measures the encoder while the CPU is ramping its own clock
# is measuring the governor. Every run therefore pins `performance` for its
# duration and puts the original back on the way out, including on Ctrl-C or a
# crash, because leaving a machine pinned at maximum clock after a benchmark is
# rude.
#
# Needs root to write scaling_governor. It tries `sudo -n` and never prompts:
# a password prompt would hang a run started from the UI, where the curses
# display owns the screen and nobody would see the question. If it cannot get
# permission it says so, prints the one command that fixes it, and carries on
# rather than refusing to run.
#
#   CINEPI_NO_GOVERNOR=1   leave the governor alone entirely
# ===========================================================================

CINEPI_GOV_SAVED=""
CINEPI_GOV_PATHS=(/sys/devices/system/cpu/cpu*/cpufreq/scaling_governor)

_cinepi_gov_write() {
    local value="$1"
    if [[ $EUID -eq 0 ]]; then
        printf '%s\n' "$value" | tee "${CINEPI_GOV_PATHS[@]}" >/dev/null 2>&1
    elif command -v sudo >/dev/null 2>&1; then
        printf '%s\n' "$value" | sudo -n tee "${CINEPI_GOV_PATHS[@]}" >/dev/null 2>&1
    else
        return 1
    fi
}

cinepi_governor_boost() {
    local first="${CINEPI_GOV_PATHS[0]}"
    [[ -r "$first" ]] || return 0          # no cpufreq on this machine
    local cur; cur=$(cat "$first" 2>/dev/null || echo "")
    [[ -n "$cur" ]] || return 0

    if [[ "${CINEPI_NO_GOVERNOR:-0}" == "1" ]]; then
        echo "governor:  $cur (left alone, CINEPI_NO_GOVERNOR=1)"
        [[ "$cur" == "performance" ]] || \
            echo "           WARNING: timings will be low and noisy"
        return 0
    fi
    if [[ "$cur" == "performance" ]]; then
        echo "governor:  performance (already set)"
        return 0
    fi
    if _cinepi_gov_write performance; then
        CINEPI_GOV_SAVED="$cur"
        echo "governor:  $cur -> performance for this run, restored on exit"
    else
        echo "governor:  $cur -- COULD NOT CHANGE IT, and timings will be low and noisy."
        echo "           Run this once, then start again:"
        echo "             echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor"
    fi
}

cinepi_governor_restore() {
    [[ -n "$CINEPI_GOV_SAVED" ]] || return 0
    local saved="$CINEPI_GOV_SAVED"
    CINEPI_GOV_SAVED=""                     # so a second call is a no-op
    if _cinepi_gov_write "$saved"; then
        echo "governor:  restored to $saved"
    else
        echo "governor:  WARNING -- could not restore $saved; still on performance"
    fi
}
