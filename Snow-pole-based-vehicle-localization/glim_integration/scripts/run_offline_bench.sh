#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# Throughput benchmark: run GLIM over the whole converted bag and report the
# real-time factor (bag_seconds / wall_seconds) plus GLIM's own per-frame
# processing times. Directly comparable to Faster-LIO's ~1.0x in
# fasterlio_integration/SPEED_AND_RELIABILITY_COMPARISON.md.  See PLAN.md §5.
#
#   scripts/run_offline_bench.sh [label]     # e.g. gpu, cpu
#
# This is just 02_run_glim.sh with timing surfaced + a parse of the log for the
# mean per-scan processing time (GLIM prints odometry timing lines).
# ---------------------------------------------------------------------------
set -euo pipefail

SCRIPTS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INTEG="$(dirname "$SCRIPTS")"
OUT="$INTEG/output"
LABEL="${1:-gpu}"

"$SCRIPTS/02_run_glim.sh" "$LABEL"

LOG="$OUT/glim_run_${LABEL}.log"
echo "[bench] realtime factor:"; grep -E 'REALTIME_FACTOR|wall-clock' "$LOG" || true
echo "[bench] GLIM timing lines (per-frame ms, tail):"
grep -Ei 'total|odom|processing|throughput|fps|\[ms\]' "$LOG" | tail -20 || \
  echo "  (no timing lines matched; inspect $LOG — GLIM's timing format may differ)"
echo "[bench] DONE label=$LABEL  (full log: $LOG)"
