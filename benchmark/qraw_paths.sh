#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Kieran Olsson
# Sourced, or run directly to list candidate output locations.
#
# Writable places a GPR set could go, each with free space, because the choice
# is usually "which card has room". How much room a set needs depends on the
# mode and the scene; the runner prints its own estimate before it starts.
cinepi_qraw_paths() {
    local here="${1:-$PWD}" seen="" d
    local -a cand=("$here")
    [[ -n "${HOME:-}" && -d "$HOME" ]] && cand+=("$HOME")
    for d in /media/*/* /media/* /mnt/* /run/media/*/*; do
        [[ -d "$d" ]] && cand+=("$d")
    done
    for d in "${cand[@]}"; do
        d=$(readlink -f -- "$d" 2>/dev/null || echo "$d")
        [[ -d "$d" && -w "$d" ]] || continue
        case ":$seen:" in *":$d:"*) continue ;; esac
        seen="$seen:$d"
        local free
        free=$(df -PB1 -- "$d" 2>/dev/null | awk 'NR==2{print $4}')
        [[ -n "$free" ]] || free=0
        printf '%s\t%s\n' "$d" "$free"
    done
}
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    cinepi_qraw_paths "$(cd "$(dirname "$0")" && pwd)" | while IFS=$'\t' read -r p f; do
        printf '%-52s %8.1f GiB free\n' "$p" "$(awk -v b="$f" 'BEGIN{print b/1073741824}')"
    done
fi
