#!/usr/bin/env bash
# linewatch.sh — line-count tracker for surgical edits.
#
# Why this exists: LLM-driven editing has a real risk of silently
# truncating files or growing them way beyond the announced "plan".
# The Edit tool reports successful replacements but doesn't surface
# "you removed 200 lines you didn't intend to". This script catches
# unintended mass deletions / insertions BEFORE commit.
#
# Usage (run from the repo root):
#
#   tools/linewatch.sh begin
#     Snapshot current line counts for every tracked file.
#     Wipe any previous plans. Run this at the start of an editing
#     session.
#
#   tools/linewatch.sh plan +N -M [-f FILE] [-r REASON]
#     Declare a planned edit: adding N lines, removing M lines on FILE
#     (default: MemForge2.src.c). REASON is a free-form string for
#     the log. Multiple plan calls accumulate.
#     Example: tools/linewatch.sh plan +50 -10 -r "add helper function"
#
#   tools/linewatch.sh check [--tolerance N]
#     Compare actual line counts to (baseline + sum of plans). If the
#     deviation exceeds tolerance (default: 0 lines), print MISMATCH
#     and exit non-zero. Otherwise OK and exit 0.
#
#   tools/linewatch.sh status
#     Show baseline, plans so far, current line count, and the running
#     delta for every tracked file. Always exits 0.
#
#   tools/linewatch.sh reset
#     Wipe state. Equivalent to begin without snapshotting.
#
# State file: .linewatch-state (gitignored — must not be committed).

set -e

# Files we track. Add new ones here as needed.
TRACK_FILES=(
    "MemForge2.src.c"
    "README.md"
    "quantai.ini"
)

STATE=".linewatch-state"
TOLERANCE=0

# --- helpers ---------------------------------------------------------

# Count lines in a file. Empty if file missing.
count_lines() {
    if [ -f "$1" ]; then
        wc -l < "$1" | tr -d ' '
    else
        echo "0"
    fi
}

# Look up baseline for FILE. Empty if missing.
get_baseline() {
    grep "^baseline:$1:" "$STATE" 2>/dev/null | tail -1 | cut -d: -f3
}

# Sum of all plan deltas for FILE. Echoes "added removed".
sum_plans_for() {
    local f="$1"
    local added=0 removed=0
    if [ ! -f "$STATE" ]; then
        echo "0 0"; return
    fi
    while IFS=: read -r kind file a r rest; do
        [ "$kind" = "plan" ] || continue
        [ "$file" = "$f" ] || continue
        added=$((added + a))
        removed=$((removed + r))
    done < "$STATE"
    echo "$added $removed"
}

# --- subcommands ----------------------------------------------------

cmd_begin() {
    > "$STATE"
    echo "# linewatch state — created $(date -u +%Y-%m-%dT%H:%M:%SZ)" >> "$STATE"
    for f in "${TRACK_FILES[@]}"; do
        if [ ! -f "$f" ]; then
            echo "WARN: tracked file '$f' does not exist — skipping baseline" >&2
            continue
        fi
        local n
        n=$(count_lines "$f")
        echo "baseline:$f:$n" >> "$STATE"
        echo "  baseline: $f = $n lines"
    done
    echo "ok — baselines recorded"
}

cmd_plan() {
    local added="" removed="" file="MemForge2.src.c" reason=""
    while [ $# -gt 0 ]; do
        case "$1" in
            +*)  added="${1#+}"; shift ;;
            -[0-9]*) removed="${1#-}"; shift ;;
            -f)  file="$2"; shift 2 ;;
            -r)  reason="$2"; shift 2 ;;
            *)   echo "linewatch plan: unknown arg '$1'" >&2; exit 2 ;;
        esac
    done
    if [ -z "$added" ] || [ -z "$removed" ]; then
        echo "linewatch plan: need +ADDED and -REMOVED" >&2
        echo "Example: linewatch.sh plan +50 -10 -r 'add helper'" >&2
        exit 2
    fi
    if [ ! -f "$STATE" ]; then
        echo "linewatch plan: no state — run 'begin' first" >&2
        exit 2
    fi
    echo "plan:$file:$added:$removed:$reason" >> "$STATE"
    echo "ok — planned +$added / -$removed for $file${reason:+ ($reason)}"
}

cmd_check() {
    while [ $# -gt 0 ]; do
        case "$1" in
            --tolerance) TOLERANCE="$2"; shift 2 ;;
            *) echo "linewatch check: unknown arg '$1'" >&2; exit 2 ;;
        esac
    done
    if [ ! -f "$STATE" ]; then
        echo "linewatch check: no state — run 'begin' first" >&2
        exit 2
    fi
    local rc=0
    for f in "${TRACK_FILES[@]}"; do
        [ -f "$f" ] || continue
        local baseline
        baseline=$(get_baseline "$f")
        if [ -z "$baseline" ]; then
            echo "skip: $f has no baseline (added since begin?)"
            continue
        fi
        local actual
        actual=$(count_lines "$f")
        local sums added removed expected_delta actual_delta abs_diff
        sums=$(sum_plans_for "$f")
        added=$(echo "$sums" | awk '{print $1}')
        removed=$(echo "$sums" | awk '{print $2}')
        expected_delta=$((added - removed))
        actual_delta=$((actual - baseline))
        abs_diff=$((actual_delta - expected_delta))
        if [ "$abs_diff" -lt 0 ]; then abs_diff=$((-abs_diff)); fi

        if [ "$abs_diff" -gt "$TOLERANCE" ]; then
            printf "MISMATCH: %-22s baseline=%5d actual=%5d  delta=%+d  expected=%+d (+%d -%d)  drift=%+d\n" \
                "$f" "$baseline" "$actual" "$actual_delta" "$expected_delta" "$added" "$removed" $((actual_delta - expected_delta))
            rc=1
        else
            printf "ok:       %-22s baseline=%5d actual=%5d  delta=%+d  expected=%+d (+%d -%d)\n" \
                "$f" "$baseline" "$actual" "$actual_delta" "$expected_delta" "$added" "$removed"
        fi
    done
    if [ "$rc" -ne 0 ]; then
        echo ""
        echo "ABORT: actual line delta does NOT match planned. Possible causes:"
        echo "  - An Edit removed more than intended (check git diff)"
        echo "  - An Edit added more than intended"
        echo "  - A 'plan' call was forgotten before editing"
        echo ""
        echo "Investigate with: git diff --stat   and   git diff <file>"
        echo "Recover with: tools/linewatch.sh reset && tools/linewatch.sh begin"
    fi
    return $rc
}

cmd_status() {
    if [ ! -f "$STATE" ]; then
        echo "(no state — run 'begin' first)"
        return 0
    fi
    echo "=== linewatch status ==="
    for f in "${TRACK_FILES[@]}"; do
        [ -f "$f" ] || continue
        local baseline actual sums added removed expected_delta actual_delta
        baseline=$(get_baseline "$f")
        actual=$(count_lines "$f")
        sums=$(sum_plans_for "$f")
        added=$(echo "$sums" | awk '{print $1}')
        removed=$(echo "$sums" | awk '{print $2}')
        expected_delta=$((added - removed))
        actual_delta=$((actual - ${baseline:-$actual}))
        printf "  %-22s baseline=%5s  actual=%5d  planned=+%d/-%d (net %+d)  actual delta=%+d\n" \
            "$f" "${baseline:-N/A}" "$actual" "$added" "$removed" "$expected_delta" "$actual_delta"
    done
    echo ""
    echo "Plans logged this session:"
    grep "^plan:" "$STATE" 2>/dev/null | sed 's/^plan:/  /' || echo "  (none yet)"
}

cmd_reset() {
    rm -f "$STATE"
    echo "ok — state wiped"
}

# --- dispatch -------------------------------------------------------

case "${1:-}" in
    begin)  shift; cmd_begin  "$@" ;;
    plan)   shift; cmd_plan   "$@" ;;
    check)  shift; cmd_check  "$@" ;;
    status) shift; cmd_status "$@" ;;
    reset)  shift; cmd_reset  "$@" ;;
    "")
        echo "linewatch.sh — line-count tracker for surgical edits"
        echo ""
        echo "Usage: $0 {begin|plan|check|status|reset}"
        echo ""
        echo "Run $0 (no args) or read the script header for details."
        exit 2
        ;;
    *)
        echo "linewatch: unknown command '$1'" >&2
        echo "Valid: begin, plan, check, status, reset" >&2
        exit 2
        ;;
esac
