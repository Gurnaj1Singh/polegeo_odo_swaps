#!/usr/bin/env bash
# FAST-LIO2 (hku-mars/FAST_LIO @ 7cc4175, docker/Dockerfile.fastlio2) on SnowPole, Faster-LIO's Ouster
# parameters (configs/fastlio2/). Scored as "FAST-LIO2" (LiDAR + IMU -> no gravity levelling) with both
# pole detectors: signal YOLOv5 -> outputs/scores, Combination-4 YOLO11n -> outputs/scores_comb4.
cd "$(dirname "$0")"
export MSYS_NO_PATHCONV=1 PYTHONIOENCODING=utf-8
PY=../venv/Scripts/python
W="D:/Odometry/odometry_benchmark"
log() { echo "$(date +%T) $*"; }
until ! docker ps --format '{{.Image}}' | grep -q .; do sleep 60; done
mkdir -p outputs/fastlio2
if [ ! -f outputs/fastlio2/timing.json ]; then
  log "[FAST-LIO2] start"
  docker run --rm -v "$W:/work" -v "$W/bag_full:/bag:ro" -v "$W/configs/fastlio2:/cfg:ro" fastlio2:noetic \
    bash /work/scripts/21_fastlio2_in_container.sh > outputs/fastlio2/container.log 2>&1
fi
[ -f outputs/fastlio2/timing.json ] || { log "[FAST-LIO2] FAILED - see outputs/fastlio2/"; exit 1; }
log "[FAST-LIO2] trajectory done: $(tr -d '\r\n' < outputs/fastlio2/timing.json | cut -c1-200)"
bash scripts/13_score_method.sh FAST-LIO2 outputs/fastlio2/fastlio2_tum.txt fastlio2 0 \
  && log "[FAST-LIO2] signal scored" || { log "[FAST-LIO2] FAILED signal scoring"; exit 1; }
CSV=outputs/csv/incremental_navigation_results_fastlio2.csv
R=$($PY scripts/04_gnss_sweep.py --csv $CSV --name FAST-LIO2 --seed 0 --pcts 0 10 25 50 \
      --cache-dir cache_comb4 --out-dir outputs/scores_comb4 2>/dev/null \
    | grep -o '"err_pred_median": [0-9.]*' | grep -o '[0-9.]*$' | tr '\n' ' ')
for s in 1 2; do
  $PY scripts/04_gnss_sweep.py --csv $CSV --name FAST-LIO2 --seed $s --pcts 10 25 50 \
      --cache-dir cache_comb4 --out-dir outputs/scores_comb4 >/dev/null 2>&1
done
log "[FAST-LIO2] comb4 median m at 0/10/25/50 %: $R"
$PY scripts/12_collect_all_methods.py --detector comb4 > /dev/null && log "[FAST-LIO2] done" || log "[FAST-LIO2] FAILED collecting"
