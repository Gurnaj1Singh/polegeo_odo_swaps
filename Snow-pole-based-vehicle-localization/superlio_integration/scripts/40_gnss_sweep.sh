#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# GNSS-% sweep for the Super-LIO odometry CSV, seeded (GNSS_SEED=0) so the same
# frames get GNSS at every level — mirrors the Faster-LIO / GLIM head-to-head
# sweep. Reuses the existing CSV (no Super-LIO re-run). Prints the pole-corrected
# median at each % and a final table parsed from the timing JSONs.
#
#   scripts/40_gnss_sweep.sh [csv] [label]
#     csv   default incremental_navigation_results_superlio.csv
#     label default superlio
#   env GNSS_PCTS="0 10 25 50" to change the levels
# ---------------------------------------------------------------------------
set -euo pipefail

SCRIPTS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INTEG="$(dirname "$SCRIPTS")"
REPO="$(dirname "$INTEG")"
OUT="$INTEG/output"
PY="$HOME/miniconda3/envs/polegeo/bin/python"

CSV="${1:-incremental_navigation_results_superlio.csv}"
LABEL="${2:-superlio}"
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
    MAP_FIG="superlio_integration/output/live_map_${LABEL}_gnss${pct}.png" \
    "$PY" snowpole_based_vehicle_localization_GNSS_percentage.py > "$log" 2>&1
  grep -E 'GNSS Percentage:|pole-corrected median|odometry-only median' "$log" | tail -3
done

echo ""
echo "==================== SWEEP SUMMARY (${LABEL}) ===================="
# RunTimer names files timing_<odom-no-spaces>_<label>_gnss<pct>.json with
# odom='Super-LIO' (hyphen kept, spaces stripped) and label='gnss_percentage'.
"$PY" - "$REPO/fasterlio_integration/output" $PCTS <<'PY'
import json, sys, os
tdir = sys.argv[1]
pcts = sys.argv[2:]
def num(d, k):
    # key may be absent OR present-but-null (e.g. no predictions) -> coerce to NaN
    v = d.get(k)
    return float('nan') if v is None else float(v)
print(f"{'GNSS %':>7} | {'pole-corr median':>16} | {'mean':>7} | {'max':>7} | {'gnss_used':>9}")
print("-"*60)
for p in pcts:
    f = os.path.join(tdir, f"timing_Super-LIO_gnss_percentage_gnss{p}.json")
    if not os.path.exists(f):
        print(f"{p:>7} | (no timing json: {os.path.basename(f)})"); continue
    d = json.load(open(f))
    print(f"{p:>7} | {num(d,'err_pred_median'):>16.2f} | "
          f"{num(d,'err_pred_mean'):>7.2f} | {num(d,'err_pred_max'):>7.1f} | "
          f"{d.get('gnss_used_count','?'):>9}")
PY
echo "== sweep done =="
