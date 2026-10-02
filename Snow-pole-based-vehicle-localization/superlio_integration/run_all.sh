#!/usr/bin/env bash
# One-shot: run the full Super-LIO pipeline hands-off:
#   0) Super-LIO odometry on the ROS2 bag        -> output/superlio_odom + FPS
#   1) trajectory -> incremental_navigation_results_superlio.csv
#   2) snow-pole pipeline @ 0% GNSS              -> snowpole_results_superlio.csv
#   3) GNSS-% sweep (0/10/25/50, seed 0)
#
# Prereqs (one-time): apt deps + `cd ../../Super-LIO && colcon build`
#   sudo apt install -y python3-colcon-common-extensions ros-jazzy-pcl-ros libgflags-dev
# See superlio_integration/README.md.
#
# Usage: run_all.sh [ros2_bag_dir] [play_rate] [config_yaml]
set -euo pipefail

INTEG="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"      # superlio_integration
REPO="$(dirname "$INTEG")"                                  # Snow-pole project dir
FLIO_ROOT="$(dirname "$REPO")"
WS="$FLIO_ROOT/Super-LIO"
VENV="$FLIO_ROOT/.baginspect_venv"
PY="$HOME/miniconda3/envs/polegeo/bin/python"

BAG="${1:-$REPO/glim_integration/output/ros2_bag}"
RATE="${2:-1.0}"
CFG="${3:-$INTEG/config/ouster_os2_128.yaml}"
FULL_BAG="$REPO/snow_pole_geo_localization_data/2024-02-28-12-59-51.bag"
CSV="incremental_navigation_results_superlio.csv"

[ -f "$WS/install/setup.bash" ] || { echo "Super-LIO not built. Run: cd $WS && colcon build" >&2; exit 1; }

echo "########## [1/4] Super-LIO odometry ##########"
"$INTEG/scripts/00_run_superlio.sh" "$BAG" "$RATE" "$CFG"

echo "########## [2/4] trajectory -> CSV ##########"
set +u; source "$VENV/bin/activate"; set -u
python "$INTEG/scripts/10_superlio_traj_to_csv.py" \
    --dataset-bag "$FULL_BAG" \
    --odom-bag "$INTEG/output/superlio_odom" \
    --align start \
    --out "$REPO/$CSV"
deactivate || true

echo "########## [3/4] snow-pole pipeline @ 0% GNSS ##########"
( cd "$REPO"
  env -u PYTHONPATH MPL_BACKEND=Agg \
    INCREMENTAL_NAV_CSV="$CSV" \
    RESULTS_CSV="snowpole_results_superlio.csv" \
    MAP_FIG="superlio_integration/output/live_map_superlio.png" \
    "$PY" snowpole_based_vehicle_localization.py )

echo "########## [4/4] GNSS-% sweep ##########"
"$INTEG/scripts/40_gnss_sweep.sh" "$CSV" superlio

echo "########## DONE ##########"
echo "CSV:     $REPO/$CSV"
echo "results: $REPO/snowpole_results_superlio.csv"
echo "outputs: $INTEG/output/"
