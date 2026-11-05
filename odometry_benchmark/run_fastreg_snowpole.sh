#!/usr/bin/env bash
# FastReg run on SnowPole on this machine (the "FastReg" row is the reference repo's trajectory, which has
# no runtime here). scripts/07_run_fastreg.py: shipped KITTI checkpoint, consecutive pairs, 8-bit reflectivity
# / 255 as intensity. Scored as "FastReg-run" (LiDAR-only -> gravity levelling) with both pole detectors:
# signal YOLOv5 -> outputs/scores, Combination-4 YOLO11n -> outputs/scores_comb4.
cd "$(dirname "$0")"
export MSYS_NO_PATHCONV=1 PYTHONIOENCODING=utf-8
PY=../venv/Scripts/python
W="D:/Odometry/odometry_benchmark"
log() { echo "$(date +%T) $*"; }
until ! docker ps --format '{{.Image}}' | grep -q .; do sleep 60; done
mkdir -p outputs/fastreg
if [ ! -f outputs/fastreg/timing.json ]; then
  log "[FastReg-run] start"
  docker run --rm --gpus all -v "$W:/work" -v "$W/cache:/data:ro" fastreg:cu128 \
    python /work/scripts/07_run_fastreg.py --cache /data --out /work/outputs/fastreg > outputs/fastreg/run.log 2>&1
fi
[ -f outputs/fastreg/timing.json ] || { log "[FastReg-run] FAILED - see outputs/fastreg/run.log"; exit 1; }
log "[FastReg-run] trajectory done: $(tr -d '\r\n' < outputs/fastreg/timing.json)"
bash scripts/13_score_method.sh FastReg-run outputs/fastreg/fastreg_tum.txt fastreg_run 1 \
  && log "[FastReg-run] signal scored" || { log "[FastReg-run] FAILED signal scoring"; exit 1; }
CSV=outputs/csv/incremental_navigation_results_fastreg_run.csv
R=$($PY scripts/04_gnss_sweep.py --csv $CSV --name FastReg-run --seed 0 --pcts 0 10 25 50 \
      --cache-dir cache_comb4 --out-dir outputs/scores_comb4 2>/dev/null \
    | grep -o '"err_pred_median": [0-9.]*' | grep -o '[0-9.]*$' | tr '\n' ' ')
for s in 1 2; do
  $PY scripts/04_gnss_sweep.py --csv $CSV --name FastReg-run --seed $s --pcts 10 25 50 \
      --cache-dir cache_comb4 --out-dir outputs/scores_comb4 >/dev/null 2>&1
done
log "[FastReg-run] comb4 median m at 0/10/25/50 %: $R"
$PY scripts/12_collect_all_methods.py --detector comb4 > /dev/null && log "[FastReg-run] done" || log "[FastReg-run] FAILED collecting"
