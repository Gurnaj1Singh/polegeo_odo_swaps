#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# GNSS-% sweep for a given odometry CSV, seeded (GNSS_SEED=0) so the same frames
# get GNSS at every level — mirrors the Faster-LIO head-to-head sweep. Reuses the
# existing odometry CSV (no GLIM re-run). Prints the pole-corrected median at
# each % and a final table parsed from the timing JSONs.
#
#   scripts/40_gnss_sweep.sh [csv] [label]
#     csv   default incremental_navigation_results_glim.csv
#     label default glim
#   env GNSS_PCTS="0 10 25 50" to change the levels
# ---------------------------------------------------------------------------
set -euo pipefail

SCRIPTS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INTEG="$(dirname "$SCRIPTS")"
REPO="$(dirname "$INTEG")"
OUT="$INTEG/output"
PY="$HOME/miniconda3/envs/polegeo/bin/python"

CSV="${1:-incremental_navigation_results_glim.csv}"
LABEL="${2:-glim}"
PCTS="${GNSS_PCTS:-0 10 25 50}"

[ -x "$PY" ] || { echo "polegeo python missing: $PY" >&2; exit 1; }
[ -f "$REPO/$CSV" ] || { echo "odometry CSV missing: $REPO/$CSV" >&2; exit 1; }

cd "$REPO"   # pipeline uses relative ./model and default relative bag path
for pct in $PCTS; do
  echo "==================== GNSS ${pct}% (seed 0) ===================="
  log="$OUT/sweep_${LABEL}_gnss${pct}.log"
  env -u PYTHONPATH MPL_BACKEND=Agg \
    INCREMENTAL_NAV_CSV="$CSV" \
    GNSS_PERCENTAGE="$pct" GNSS_SEED=0 \
    RESULTS_CSV="snowpole_results_${LABEL}_gnss${pct}.csv" \
    MAP_FIG="glim_integration/output/live_map_${LABEL}_gnss${pct}.png" \
    "$PY" snowpole_based_vehicle_localization_GNSS_percentage.py > "$log" 2>&1
  grep -E 'GNSS Percentage:|pole-corrected median|odometry-only median' "$log" | tail -3
done

echo ""
echo "==================== SWEEP SUMMARY (${LABEL}) ===================="
"$PY" - "$LABEL" "$REPO/fasterlio_integration/output" $PCTS <<'PY'
import json, sys, os
label, tdir = sys.argv[1], sys.argv[2]
pcts = sys.argv[3:]
print(f"{'GNSS %':>7} | {'pole-corr median':>16} | {'mean':>7} | {'max':>7} | {'gnss_used':>9}")
print("-"*60)
for p in pcts:
    f = os.path.join(tdir, f"timing_{label.upper() if label.lower()=='glim' else label}_gnss_percentage_gnss{p}.json")
    # try a couple of label casings
    cands = [f, os.path.join(tdir, f"timing_GLIM_gnss_percentage_gnss{p}.json")]
    j = next((c for c in cands if os.path.exists(c)), None)
    if not j:
        print(f"{p:>7} | (no timing json found)"); continue
    d = json.load(open(j))
    print(f"{p:>7} | {d.get('err_pred_median',float('nan')):>16.2f} | "
          f"{d.get('err_pred_mean',float('nan')):>7.2f} | {d.get('err_pred_max',float('nan')):>7.1f} | "
          f"{d.get('gnss_used_count','?'):>9}")
PY
echo "== sweep done =="
