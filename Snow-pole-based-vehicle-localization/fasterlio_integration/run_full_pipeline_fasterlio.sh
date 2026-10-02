#!/usr/bin/env bash
# =============================================================================
# ONE COMMAND: run the COMPLETE snow-pole localization pipeline with FASTER-LIO
# odometry and render the final temporal-evolution visualization.
#
#   Stage A  odometry  -> incremental_navigation_results_fasterlio.csv
#   Stage B  pipeline  -> snowpole_results_fasterlio.csv  (+ live map PNG)
#   Stage C  final viz -> output/temporal_evolution_fasterlio.mp4 (+ summary_*.png)
#
# Stage A is SKIPPED automatically if the odometry CSV already exists (fast path:
# you go straight to the pipeline + visualization). Pass --from-bag to force a
# fresh Faster-LIO run from the raw rosbag (needs Docker + the FULL bag).
#
# Usage:
#   fasterlio_integration/run_full_pipeline_fasterlio.sh            # reuse CSV, then B + C
#   fasterlio_integration/run_full_pipeline_fasterlio.sh --from-bag # regenerate odometry first
# =============================================================================
set -euo pipefail

INTEG="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"          # fasterlio_integration
REPO="$(dirname "$INTEG")"                                     # Snow-pole project dir
FLIO_ROOT="$(dirname "$REPO")"                                 # Fasterlio (repo root)
PY="$HOME/miniconda3/envs/polegeo/bin/python"                  # pipeline + viz interpreter
VENV="$FLIO_ROOT/.baginspect_venv"                             # bridge interpreter (rosbags)
OUT="$INTEG/output"; mkdir -p "$OUT"

LABEL="fasterlio"
ODOM_NAME="Faster-LIO"
CSV="incremental_navigation_results_${LABEL}.csv"
RESULTS="snowpole_results_${LABEL}.csv"
FULL_BAG="$REPO/snow_pole_geo_localization_data/2024-02-28-12-59-51.bag"
POLES="$REPO/Groundtruth_pole_location_at_test_site_E39_Hemnekjølen.csv"

FROM_BAG=0
[ "${1:-}" = "--from-bag" ] && FROM_BAG=1

[ -x "$PY" ] || { echo "polegeo python missing: $PY  (see fasterlio_integration/scripts/setup_polegeo_env.sh)" >&2; exit 1; }

# ---- Stage A: Faster-LIO odometry -> CSV ------------------------------------
if [ "$FROM_BAG" -eq 1 ] || [ ! -f "$REPO/$CSV" ]; then
  echo "########## [A] Faster-LIO odometry -> $CSV ##########"
  [ -f "$FULL_BAG" ] || { echo "full bag (with /ouster/points) needed for --from-bag: $FULL_BAG" >&2; exit 1; }
  bash "$INTEG/run_all.sh" "$FULL_BAG"                         # Docker stage-0 -> output/fasterlio_odometry.bag
  set +u; source "$VENV/bin/activate"; set -u
  python "$INTEG/scripts/10_fasterlio_traj_to_csv.py" \
    --dataset-bag "$FULL_BAG" \
    --odom-bag "$OUT/fasterlio_odometry.bag" \
    --tum-out "$OUT/fasterlio_traj_tum.txt" \
    --align start --out "$REPO/$CSV"
  deactivate || true
else
  echo "########## [A] reusing existing $REPO/$CSV  (pass --from-bag to regenerate) ##########"
fi

# ---- Stage B: snow-pole localization pipeline (0% GNSS = proposed method) ----
echo "########## [B] snow-pole pipeline -> $RESULTS ##########"
( cd "$REPO"
  env -u PYTHONPATH MPL_BACKEND=Agg \
    INCREMENTAL_NAV_CSV="$CSV" \
    RESULTS_CSV="$RESULTS" \
    MAP_FIG="$OUT/live_map_${LABEL}.png" \
    "$PY" snowpole_based_vehicle_localization.py )

# ---- Stage C: final temporal-evolution visualization -------------------------
echo "########## [C] temporal-evolution visualization ##########"
"$PY" "$REPO/fasterlio_integration/scripts/20_temporal_evolution_visualization.py" \
  --fasterlio-csv "$REPO/$CSV" \
  --poles "$POLES" \
  --results-csv "$REPO/$RESULTS" \
  --odom-label "$ODOM_NAME" \
  --outdir "$OUT"
[ -f "$OUT/temporal_evolution.mp4" ] && mv -f "$OUT/temporal_evolution.mp4" "$OUT/temporal_evolution_${LABEL}.mp4"
[ -f "$OUT/temporal_evolution.gif" ] && mv -f "$OUT/temporal_evolution.gif" "$OUT/temporal_evolution_${LABEL}.gif"

echo ""
echo "########## DONE (Faster-LIO) ##########"
echo "  odometry CSV : $REPO/$CSV"
echo "  results CSV  : $REPO/$RESULTS"
echo "  live map     : $OUT/live_map_${LABEL}.png"
echo "  ANIMATION    : $OUT/temporal_evolution_${LABEL}.mp4   <-- open this (the fastreg-style dynamic view)"
echo "  summaries    : $OUT/summary_trajectories.png, summary_error_{hist,cdf}.png, summary_error_vs_distance.png"
