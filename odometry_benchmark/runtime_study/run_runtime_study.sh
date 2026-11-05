#!/usr/bin/env bash
# Per-stage runtime of the snow-pole pipeline (README.md). One job at a time on an idle machine:
#   1. KISS-ICP odometry alone, default multithreading -> out/kiss_icp_mt/timing.json
#   2. stage-timed original localization script (YOLOv5 signal model, GNSS 0 %) with KISS-ICP odometry
#   3. the same with FastReg odometry (FastReg-run trajectory from this laptop)
cd "$(dirname "$0")"
export PYTHONIOENCODING=utf-8 MPL_BACKEND=Agg POLE_MODEL=./model/pole_best_signal.pt GNSS_PERCENTAGE=0 GNSS_SEED=0
export BAG_PATH=D:/Odometry/odometry_benchmark/bag_full/2024-02-28-12-59-51.bag
PY=D:/Odometry/venv/Scripts/python
log() { echo "$(date +%T) $*"; }
log "[kiss-icp odometry] start"
(cd .. && $PY -u scripts/01_run_lidar_odometry.py --method kiss_icp --cache cache --out runtime_study/out/kiss_icp_mt) \
  > out/kiss_icp_mt.log 2>&1 && log "[kiss-icp odometry] $(tr -d '\r\n' < out/kiss_icp_mt/timing.json)" \
  || log "[kiss-icp odometry] FAILED - see out/kiss_icp_mt.log"
for M in kiss_icp fastreg_run; do
  log "[localization $M] start"
  INCREMENTAL_NAV_CSV=D:/Odometry/odometry_benchmark/outputs/csv/incremental_navigation_results_$M.csv \
  STAGE_TIMING_OUT=out/stage_$M.json $PY -u timed_GNSS_percentage.py > out/localization_$M.log 2>&1 \
    && log "[localization $M] $(tr -d '\r\n ' < out/stage_$M.json | cut -c1-400)" \
    || log "[localization $M] FAILED - see out/localization_$M.log"
done
log "[runtime-study] done"
