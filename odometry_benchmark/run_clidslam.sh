#!/usr/bin/env bash
# CLID-SLAM (RA-L 2025) on SnowPole - category 4 (DL/ML, 2025-26): LiDAR-inertial odometry against a
# neural implicit SDF map trained online. Replaces Hi-LOAM (no model/ package in its repo); Splat-LOAM,
# the earlier stand-in, has no learned component and is reported as an extra.
# One Docker workload at a time: runs after the UGP rerun; Splat-LOAM (extra) waits for it.
#   bash run_clidslam.sh > run_clidslam.log 2>&1
cd "$(dirname "$0")"
export MSYS_NO_PATHCONV=1 PYTHONIOENCODING=utf-8
W="D:/Odometry/odometry_benchmark"
log() { echo "$(date +%T) $*"; }
[ -f outputs/scores/CLID-SLAM_seed0.csv ] && { log "already scored"; exit 0; }
busy() { ps -ef | grep -v grep | grep -qE "run_new_methods.sh|run_ugp_rerun.sh" \
         || docker ps --format '{{.Image}}' | grep -q .; }

# the image build is CPU-only and small (no CUDA compile), but still waits its turn
while busy; do sleep 60; done
docker image inspect clidslam:cu128 >/dev/null 2>&1 || {
  log "[build] clidslam:cu128"
  docker build -t clidslam:cu128 -f docker/Dockerfile.clidslam . > docker/build_clidslam.log 2>&1 \
    && log "[ok] clidslam:cu128" || { log "[FAILED] build clidslam:cu128"; exit 1; }
}
[ -f cache/clid_seq/pose_ts.txt ] || {
  log "[CLID-SLAM] exporting sequence"
  docker run --rm -v "$W:/work" clidslam:cu128 python /work/scripts/16_export_clid_sequence.py \
    --cache /work/cache --out /work/cache/clid_seq > outputs/clid_export.log 2>&1 \
    || { log "[CLID-SLAM] FAILED export - see outputs/clid_export.log"; exit 1; }
}
log "[CLID-SLAM] start"; mkdir -p outputs/clidslam; s=$(date +%s)
docker run --rm --gpus all --shm-size=4g -v "$W:/work" -v "$W/cache/clid_seq:/seq:ro" clidslam:cu128 \
  python slam.py /work/configs/clid_slam_snowpole.yaml > outputs/clidslam/run.log 2>&1
echo "{\"wall_s\": $(( $(date +%s) - s )), \"n_scans\": 5419}" > outputs/clidslam/timing_wall.json
EST=$(find outputs/clidslam -name "odom_poses_tum.txt" -printf "%T@ %p\n" | sort -nr | head -1 | cut -d" " -f2-)
[ -n "$EST" ] || { log "[CLID-SLAM] FAILED - no trajectory, see outputs/clidslam/run.log"; exit 1; }
cp "$EST" outputs/clidslam/clidslam_tum.txt
# world frame = IMU frame at the first scan (gravity is estimated, the frame is not re-levelled) -> level 1
bash scripts/13_score_method.sh CLID-SLAM outputs/clidslam/clidslam_tum.txt clidslam 1
[ -f outputs/scores/CLID-SLAM_seed0.csv ] || { log "[CLID-SLAM] FAILED scoring"; exit 1; }
log "[CLID-SLAM] done"
