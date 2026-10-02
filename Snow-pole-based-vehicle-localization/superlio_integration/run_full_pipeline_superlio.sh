#!/usr/bin/env bash
# =============================================================================
# ONE COMMAND: run the COMPLETE snow-pole localization pipeline with SUPER-LIO
# odometry and render the final temporal-evolution visualization.
#
#   Stage A  odometry  -> incremental_navigation_results_superlio.csv
#   Stage B  pipeline  -> snowpole_results_superlio.csv  (+ live map PNG)
#   Stage C  final viz -> output/temporal_evolution_superlio.mp4 (+ summary_*.png)
#
# Stage A is SKIPPED automatically if the odometry CSV already exists. Pass
# --from-bag to force a fresh Super-LIO run from the ROS2 bag (needs the native
# `colcon build` of ../../Super-LIO and the converted ROS2 bag, which is reused
# from glim_integration/output/ros2_bag).
#
# Usage:
#   superlio_integration/run_full_pipeline_superlio.sh             # reuse CSV, then B + C
#   superlio_integration/run_full_pipeline_superlio.sh --from-bag  # regenerate odometry first
# =============================================================================
set -euo pipefail

INTEG="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"          # superlio_integration
REPO="$(dirname "$INTEG")"                                     # Snow-pole project dir
FLIO_ROOT="$(dirname "$REPO")"                                 # Fasterlio (holds Super-LIO/)
PY="$HOME/miniconda3/envs/polegeo/bin/python"
VENV="$FLIO_ROOT/.baginspect_venv"                             # bridge interpreter (rosbags)
OUT="$INTEG/output"; mkdir -p "$OUT"

LABEL="superlio"
ODOM_NAME="Super-LIO"
CSV="incremental_navigation_results_${LABEL}.csv"
RESULTS="snowpole_results_${LABEL}.csv"
FULL_BAG="$REPO/snow_pole_geo_localization_data/2024-02-28-12-59-51.bag"
ROS2_BAG="$REPO/glim_integration/output/ros2_bag"
POLES="$REPO/Groundtruth_pole_location_at_test_site_E39_Hemnekjølen.csv"

FROM_BAG=0
[ "${1:-}" = "--from-bag" ] && FROM_BAG=1

[ -x "$PY" ] || { echo "polegeo python missing: $PY  (see fasterlio_integration/scripts/setup_polegeo_env.sh)" >&2; exit 1; }

# ---- Stage A: Super-LIO odometry -> CSV -------------------------------------
if [ "$FROM_BAG" -eq 1 ] || [ ! -f "$REPO/$CSV" ]; then
  echo "########## [A] Super-LIO odometry -> $CSV ##########"
  [ -d "$ROS2_BAG" ] || { echo "ROS2 bag missing: $ROS2_BAG  (run glim_integration/scripts/00_convert_bag.sh first)" >&2; exit 1; }
  "$INTEG/scripts/00_run_superlio.sh" "$ROS2_BAG"             # native Super-LIO -> output/superlio_odom
  set +u; source "$VENV/bin/activate"; set -u
  python "$INTEG/scripts/10_superlio_traj_to_csv.py" \
    --dataset-bag "$FULL_BAG" \
    --odom-bag "$OUT/superlio_odom" \
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
echo "########## DONE (Super-LIO) ##########"
echo "  odometry CSV : $REPO/$CSV"
echo "  results CSV  : $REPO/$RESULTS"
echo "  live map     : $OUT/live_map_${LABEL}.png"
echo "  ANIMATION    : $OUT/temporal_evolution_${LABEL}.mp4   <-- open this (the fastreg-style dynamic view)"
echo "  summaries    : $OUT/summary_trajectories.png, summary_error_{hist,cdf}.png, summary_error_vs_distance.png"
