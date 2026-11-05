#!/usr/bin/env bash
# UGP run. At the shared 150 m range the earlier attempts ran out of GPU memory / hit "CUDA driver
# error: device not ready" on the open stretch around scans 3700-4500, so UGP's input is capped at 80 m
# (the HDL-64 KITTI reach its checkpoint was trained on; scripts/11_run_ugp.py --max-range).
# expandable_segments; checkpoint every 250 pairs; a pair that still breaks the CUDA context is saved as
# "bad", the container exits and the next one (fresh context) registers it at a fallback level
# (up to 8 container attempts).
# One Docker workload at a time: waits for CLID-SLAM; Splat-LOAM (extra) waits for it.
cd "$(dirname "$0")"
export MSYS_NO_PATHCONV=1 PYTHONIOENCODING=utf-8
PY=../venv/Scripts/python
W="D:/Odometry/odometry_benchmark"
log() { echo "$(date +%T) $*"; }
[ -f outputs/scores/UGP_seed0.csv ] && { log "already scored"; exit 0; }
until ! ps -ef | grep -v grep | grep -q "run_clidslam.sh" && ! docker ps --format '{{.Image}}' | grep -q .; do sleep 60; done
mkdir -p outputs/ugp; rm -f outputs/ugp/ugp_poses_kitti.txt
# "bash run_ugp_rerun.sh resume" continues from outputs/ugp/ugp_partial.npz instead of starting over
if [ "$1" = "resume" ] && [ -f outputs/ugp/ugp_partial.npz ]; then RES="--resume"; log "[UGP] start (resume)"
else RES=""; rm -f outputs/ugp/ugp_partial.npz; log "[UGP] start"; fi
for attempt in 1 2 3 4 5 6 7 8; do
  docker run --rm --gpus all --shm-size=4g -e PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True -v "$W:/work" ugp:cu128 \
    python /work/scripts/11_run_ugp.py --bins /work/cache/kitti_bins --out /work/outputs/ugp $RES >> outputs/ugp/run.log 2>&1
  [ -f outputs/ugp/ugp_poses_kitti.txt ] && break
  log "[UGP] attempt $attempt crashed - resuming from checkpoint"; RES="--resume"; sleep 30
done
[ -f outputs/ugp/ugp_poses_kitti.txt ] || { log "[UGP] FAILED - see outputs/ugp/run.log"; exit 1; }
$PY scripts/08_kitti_poses_to_tum.py --poses outputs/ugp/ugp_poses_kitti.txt --out outputs/ugp/ugp_tum.txt \
  && bash scripts/13_score_method.sh UGP outputs/ugp/ugp_tum.txt ugp 1 && log "[UGP] done" || log "[UGP] FAILED scoring"
