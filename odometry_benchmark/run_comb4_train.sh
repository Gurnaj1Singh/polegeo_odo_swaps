#!/usr/bin/env bash
# YOLO11n on Combination 4: COCO-pretrained vs from scratch, seeds 0-2 each (up to 500 epochs, early stop
# patience 50), one run at a time on the GPU, then the statistical comparison that picks the final detector
# (scripts/20_compare_yolo_init.py -> results/detector_yolo11n_comb4.*, models/yolo11n_comb4_final.pt).
# Interleaved order so a first COCO-vs-scratch pair exists early. Re-running skips finished runs and
# resumes an interrupted one from its last.pt. run_comb4_rescore.sh waits for this script.
cd "$(dirname "$0")"
export PYTHONIOENCODING=utf-8
PY=../venv/Scripts/python
log() { echo "$(date +%T) $*"; }
for s in 0 1 2; do
  for i in coco scratch; do
    R=comb4_${i}_s${s}
    [ -f models/$R/eval.json ] && { log "[$R] already done"; continue; }
    for attempt in 1 2 3; do
      log "[$R] start (attempt $attempt)"
      $PY scripts/17_train_yolo11n_comb4.py --init $i --seed $s >> models/$R.log 2>&1
      [ -f models/$R/eval.json ] && break
      log "[$R] crashed - see models/$R.log"; sleep 30
    done
    [ -f models/$R/eval.json ] || { log "[$R] FAILED"; exit 1; }
    log "[$R] done: $(tr -d '\n ' < models/$R/eval.json | grep -o '"epochs_run":[0-9]*'), test $(tr -d '\n ' < models/$R/eval.json | grep -o '"test":{[^}]*}')"
  done
done
log "[compare] start"
$PY scripts/20_compare_yolo_init.py > results/detector_yolo11n_comb4_compare.log 2>&1 \
  && log "[compare] done: $(grep -o 'Decision: .*' results/detector_yolo11n_comb4.md)" || { log "[compare] FAILED"; exit 1; }
