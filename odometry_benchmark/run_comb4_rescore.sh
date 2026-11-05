#!/usr/bin/env bash
# Re-score every executed odometry method with the YOLO11n Combination-4 pole detector.
#   1. wait for run_comb4_train.sh (6 trainings + COCO-vs-scratch comparison -> models/yolo11n_comb4_final.pt)
#   2. cache its detections over the whole bag -> cache_comb4/detections.pkl (03_cache_detections.py --input comb4)
#   3. GNSS sweep for each method's existing trajectory CSV (seed 0 at 0/10/25/50 %, seeds 1-2 at 10/25/50 %)
#      -> outputs/scores_comb4/ ; the signal-detector scores in outputs/scores/ are not touched
#   4. results/snowpole_all_methods_comb4.* and results/snowpole_signal_vs_comb4.md
# Everything is appended to run_comb4_rescore.log.
cd "$(dirname "$0")"
export PYTHONIOENCODING=utf-8
PY=../venv/Scripts/python
log() { echo "$(date +%T) $*"; }
until [ -f models/yolo11n_comb4_final.pt ] && [ -f results/detector_yolo11n_comb4.json ] \
      && ! ps -ef | grep -v grep | grep -qE "run_comb4_train.sh|17_train_yolo11n_comb4|20_compare_yolo_init"; do sleep 60; done
log "[comb4] detector chosen: $(tr -d '\n ' < results/detector_yolo11n_comb4.json | grep -o '"final":{"run":"[^"]*"')"

mkdir -p cache_comb4 outputs/scores_comb4
cp -n cache/kriged_gnss.npz cache_comb4/
if [ ! -f cache_comb4/detections.pkl ]; then
  log "[comb4] caching detections"
  $PY scripts/03_cache_detections.py --input comb4 --weights models/yolo11n_comb4_final.pt \
      --out cache_comb4/detections.pkl > cache_comb4/detections.log 2>&1 || { log "[comb4] FAILED caching"; exit 1; }
fi
log "[comb4] $(tail -1 cache_comb4/detections.log)"
$PY scripts/19_compare_detections.py cache/detections.pkl cache_comb4/detections.pkl > results/detections_signal_vs_comb4.md
log "[comb4] detection comparison -> results/detections_signal_vs_comb4.md"

# method name : trajectory CSV key (the same CSVs that produced outputs/scores/)
for m in KISS-ICP:kiss_icp MAD-ICP:mad_icp Faster-LIO:fasterlio GLIM:glim FastReg:fastreg \
         DeepPointMap:deeppointmap PIN-SLAM:pin_slam RKO-LIO:rko_lio GenZ-ICP:genz_icp Super-LIO:superlio \
         BUFFER-X:bufferx UGP:ugp; do
  NAME=${m%%:*}; CSV=outputs/csv/incremental_navigation_results_${m##*:}.csv
  [ -f "$CSV" ] || { log "[$NAME] no trajectory CSV - skipped"; continue; }
  R=$($PY scripts/04_gnss_sweep.py --csv "$CSV" --name "$NAME" --seed 0 --pcts 0 10 25 50 \
        --cache-dir cache_comb4 --out-dir outputs/scores_comb4 2>/dev/null \
      | grep -o '"err_pred_median": [0-9.]*' | grep -o '[0-9.]*$' | tr '\n' ' ')
  for s in 1 2; do
    $PY scripts/04_gnss_sweep.py --csv "$CSV" --name "$NAME" --seed $s --pcts 10 25 50 \
        --cache-dir cache_comb4 --out-dir outputs/scores_comb4 >/dev/null 2>&1
  done
  log "[$NAME] comb4 median m at 0/10/25/50 %: $R"
done
$PY scripts/12_collect_all_methods.py --detector comb4 > /dev/null && log "[comb4] done -> results/snowpole_all_methods_comb4.md, results/snowpole_signal_vs_comb4.md" \
  || log "[comb4] FAILED collecting"
