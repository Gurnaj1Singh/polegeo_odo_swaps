#!/usr/bin/env bash
# Run and score the methods added for the 12-method comparison (+ re-runs of the three methods
# previously copied from the reference repo). Idempotent: a method whose scores already exist
# (outputs/scores/<Name>_seed0.csv) is skipped, so the script can be re-launched after a failure.
# Two lanes:
#   (run strictly one after another - see the note at the bottom)
#   GPU methods: PIN-SLAM, DeepPointMap, BUFFER-X, UGP, Hi-LOAM, GLIM
#   CPU methods: Faster-LIO, Super-LIO
# Each job waits for its Docker image (and for the GPU to be free), runs, converts its trajectory to
# TUM (sensor clock) and is scored by scripts/13_score_method.sh (same bridge / detector / correction
# as every method). GPU containers get --shm-size=4g (PyTorch DataLoader workers need > 64 MB /dev/shm).
#
#   bash run_new_methods.sh > run_new_methods.log 2>&1
cd "$(dirname "$0")"
export MSYS_NO_PATHCONV=1 PYTHONIOENCODING=utf-8
PY=../venv/Scripts/python
W="D:/Odometry/odometry_benchmark"
GPU="docker run --rm --gpus all --shm-size=4g"
log() { echo "$(date +%T) $*"; }
scored() { [ -f "outputs/scores/$1_seed0.csv" ]; }
wait_image() { until docker image inspect "$1" >/dev/null 2>&1; do sleep 30; done; }
wait_gpu_free() {  # no other GPU method container running
  while docker ps --format '{{.Image}}' | grep -qE "pin_slam|deeppointmap|bufferx|ugp|hiloam|fastreg|glim_ros2"; do sleep 30; done
}
score() { bash scripts/13_score_method.sh "$@" || log "[$1] SCORING FAILED"; }

gpu_lane() {
  if ! scored PIN-SLAM; then
    wait_gpu_free
    P=$(ls -t outputs/pin_slam/pin_slam_snowpole_*/odom_poses_kitti.txt 2>/dev/null | head -1)
    if [ -n "$P" ]; then
      $PY scripts/08_kitti_poses_to_tum.py --poses "$P" --out outputs/pin_slam/pin_slam_tum.txt \
        && score PIN-SLAM outputs/pin_slam/pin_slam_tum.txt pin_slam 1
    else
      log "[PIN-SLAM] no poses - run it first (see README)"
    fi
  fi

  if ! scored DeepPointMap; then
    wait_image deeppointmap:cu128; wait_gpu_free; log "[DeepPointMap] start"; mkdir -p outputs/deeppointmap; s=$(date +%s)
    $GPU -v "$W:/work" -v "$W/cache:/data:ro" deeppointmap:cu128 \
      python pipeline/infer.py --yaml_file /work/configs/deeppointmap_snowpole.yaml --weight ./DeepPointMapAAAI.pth \
      > outputs/deeppointmap_run.log 2>&1
    echo "{\"wall_s\": $(( $(date +%s) - s )), \"n_scans\": 5419}" > outputs/deeppointmap/timing_wall.json
    D=outputs/deeppointmap/Seq00
    { [ -f $D/trajectory.allframes.txt ] && $PY scripts/08_kitti_poses_to_tum.py --poses $D/trajectory.allframes.txt \
        --frame-ids $D/trajectory.allsteps.txt --out outputs/deeppointmap/deeppointmap_tum.txt --project-rotations \
      && score DeepPointMap outputs/deeppointmap/deeppointmap_tum.txt deeppointmap 1; } || log "[DeepPointMap] FAILED"
  fi

  if ! scored BUFFER-X; then
    wait_image bufferx:cu128; wait_gpu_free; log "[BUFFER-X] start"; mkdir -p outputs/bufferx
    $GPU -v "$W:/work" bufferx:cu128 \
      python /work/scripts/10_run_bufferx.py --bins /work/cache/kitti_bins --out /work/outputs/bufferx > outputs/bufferx/run.log 2>&1
    { [ -f outputs/bufferx/bufferx_poses_kitti.txt ] && $PY scripts/08_kitti_poses_to_tum.py \
        --poses outputs/bufferx/bufferx_poses_kitti.txt --out outputs/bufferx/bufferx_tum.txt \
      && score BUFFER-X outputs/bufferx/bufferx_tum.txt bufferx 1; } || log "[BUFFER-X] FAILED"
  fi

  if ! scored UGP; then
    wait_image ugp:cu128; wait_gpu_free; log "[UGP] start"; mkdir -p outputs/ugp
    $GPU -v "$W:/work" ugp:cu128 \
      python /work/scripts/11_run_ugp.py --bins /work/cache/kitti_bins --out /work/outputs/ugp > outputs/ugp/run.log 2>&1
    { [ -f outputs/ugp/ugp_poses_kitti.txt ] && $PY scripts/08_kitti_poses_to_tum.py \
        --poses outputs/ugp/ugp_poses_kitti.txt --out outputs/ugp/ugp_tum.txt \
      && score UGP outputs/ugp/ugp_tum.txt ugp 1; } || log "[UGP] FAILED"
  fi

  if ! scored Hi-LOAM; then
    wait_image hiloam:cu128; wait_gpu_free; log "[Hi-LOAM] start"; mkdir -p outputs/hiloam; s=$(date +%s)
    $GPU -v "$W:/work" -v "$W/cache:/data:ro" hiloam:cu128 \
      python Hi_LOAM.py /work/configs/hiloam_snowpole.yaml > outputs/hiloam/run.log 2>&1
    echo "{\"wall_s\": $(( $(date +%s) - s )), \"n_scans\": 5419}" > outputs/hiloam/timing_wall.json
    H=$(ls -t outputs/hiloam/*/pose.txt outputs/hiloam/*pose.txt 2>/dev/null | head -1)
    { [ -n "$H" ] && $PY scripts/08_kitti_poses_to_tum.py --poses "$H" --out outputs/hiloam/hiloam_tum.txt \
      && score Hi-LOAM outputs/hiloam/hiloam_tum.txt hiloam 1; } || log "[Hi-LOAM] FAILED"
  fi

  if ! scored GLIM; then
    wait_gpu_free; log "[GLIM] start"; s=$(date +%s)
    bash scripts/14_run_ros_methods.sh glim
    echo "{\"wall_s\": $(( $(date +%s) - s )), \"n_scans\": 5419}" > outputs/glim/timing_wall.json
    { [ -f outputs/glim/glim_odom_lidar_tum.txt ] && score GLIM outputs/glim/glim_odom_lidar_tum.txt glim 0; } \
      || log "[GLIM] FAILED"
    [ -f outputs/glim/glim_traj_lidar_tum.txt ] && score "GLIM-globalopt" outputs/glim/glim_traj_lidar_tum.txt glim_globalopt 0
  fi
  log "GPU lane done"
}

cpu_lane() {
  if ! scored Faster-LIO; then
    wait_image fasterlio:noetic; log "[Faster-LIO] start"
    bash scripts/14_run_ros_methods.sh fasterlio
    { [ -s outputs/fasterlio/fasterlio_tum.txt ] && score Faster-LIO outputs/fasterlio/fasterlio_tum.txt fasterlio 0; } \
      || log "[Faster-LIO] FAILED"
  fi
  if ! scored Super-LIO; then
    wait_image superlio:jazzy; log "[Super-LIO] start"
    bash scripts/14_run_ros_methods.sh superlio
    { [ -s outputs/superlio/superlio_tum.txt ] && score Super-LIO outputs/superlio/superlio_tum.txt superlio 0; } \
      || log "[Super-LIO] FAILED"
  fi
  log "CPU lane done"
}

build() {  # tag dockerfile context - one build at a time, skipped if the image exists
  docker image inspect "$1" >/dev/null 2>&1 && return
  log "[build] $1"
  docker build -t "$1" -f "$2" "$3" > "docker/build_${1%%:*}.log" 2>&1 && log "[ok] $1" || log "[FAILED] build $1"
}

# Strictly ONE Docker workload at a time: running a method next to a C++/CUDA build (or two
# methods) exhausted the Docker VM's ~7.6 GB and crashed the engine twice on 2026-10-30/31.
build superlio:jazzy docker/Dockerfile.superlio third_party/polegeo_odo_swaps/Super-LIO
build bufferx:cu128  docker/Dockerfile.bufferx  .
cpu_lane
gpu_lane
$PY scripts/12_collect_all_methods.py > /dev/null
log "ALL DONE"
