#!/usr/bin/env bash
# Score one method's SnowPole trajectory exactly like the others:
#   bridge (start-anchored, + gravity levelling for LiDAR-only methods) -> GNSS sweep
#   (seed 0 at 0/10/25/50 %, seeds 1-2 at 10/25/50 %) -> refresh results/snowpole_all_methods.*
#
#   bash scripts/13_score_method.sh <Name> <tum file> <csv key> <level: 0|1>
#   bash scripts/13_score_method.sh PIN-SLAM outputs/pin_slam/pin_slam_tum.txt pin_slam 1
set -euo pipefail
cd "$(dirname "$0")/.."
NAME=$1; TUM=$2; KEY=$3; LEVEL=$4
PY=${PY:-../venv/Scripts/python}
CSV=outputs/csv/incremental_navigation_results_${KEY}.csv
LV=""; [ "$LEVEL" = "1" ] && LV="--level-gravity"
"$PY" scripts/02_bridge_to_csv.py $LV --tum "$TUM" --out "$CSV" 2>&1 | grep -E "level|align"
"$PY" scripts/04_gnss_sweep.py --csv "$CSV" --name "$NAME" --seed 0 --pcts 0 10 25 50 2>/dev/null \
  | grep -o '"gnss_pct": [0-9.]*, "err_pred_median": [0-9.]*'
for s in 1 2; do
  "$PY" scripts/04_gnss_sweep.py --csv "$CSV" --name "$NAME" --seed $s --pcts 10 25 50 >/dev/null 2>&1
done
"$PY" scripts/12_collect_all_methods.py > /dev/null && echo "[$NAME] scored -> results/snowpole_all_methods.md"
