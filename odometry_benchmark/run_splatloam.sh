#!/usr/bin/env bash
# Splat-LOAM (ICCV 2025) on SnowPole - category 4 replacement for Hi-LOAM, whose public repository
# never contained its model/ package. One Docker workload at a time: waits for run_new_methods.sh.
#   bash run_splatloam.sh > run_splatloam.log 2>&1
cd "$(dirname "$0")"
export MSYS_NO_PATHCONV=1 PYTHONIOENCODING=utf-8
PY=../venv/Scripts/python
W="D:/Odometry/odometry_benchmark"
log() { echo "$(date +%T) $*"; }
[ -f outputs/scores/Splat-LOAM_seed0.csv ] && { log "already scored"; exit 0; }

# wait until the orchestrator has finished and no method container is running
until ! ps -ef | grep -v grep | grep -q "run_new_methods.sh" \
      && ! ps -ef | grep -v grep | grep -qE "run_ugp_rerun.sh|run_clidslam.sh" && ! docker ps --format "{{.Image}}" | grep -q .; do sleep 60; done

docker image inspect splatloam:cu128 >/dev/null 2>&1 || {
  log "[build] splatloam:cu128"
  docker build -t splatloam:cu128 -f docker/Dockerfile.splatloam . > docker/build_splatloam.log 2>&1 \
    && log "[ok] splatloam:cu128" || { log "[FAILED] build splatloam:cu128"; exit 1; }
}
RUN="docker run --rm --gpus all --shm-size=4g -v $W:/work -v $W/cache/kitti_bins:/seq/velodyne:ro
     -v $W/cache/splatloam_times.txt:/seq/times.txt:ro -v $W/cache/splatloam_calib.txt:/seq/calib.txt:ro splatloam:cu128"

log "[Splat-LOAM] smoke test (30 frames)"
mkdir -p outputs/_splatloam_test/velodyne
for i in $(seq -f "%06g" 0 29); do cp cache/kitti_bins/$i.bin outputs/_splatloam_test/velodyne/; done
head -30 cache/splatloam_times.txt > outputs/_splatloam_test/times.txt; cp cache/splatloam_calib.txt outputs/_splatloam_test/
docker run --rm --gpus all --shm-size=4g -v "$W:/work" -v "$W/outputs/_splatloam_test:/seq:ro" splatloam:cu128 \
  python run.py slam /work/configs/splatloam_snowpole.yaml output.folder=/work/outputs/_splatloam_test/out \
  > outputs/_splatloam_test.log 2>&1
if ! find outputs/_splatloam_test/out -name "*.txt" | grep -q .; then
  log "[Splat-LOAM] smoke test FAILED - see outputs/_splatloam_test.log"; tail -15 outputs/_splatloam_test.log; exit 1
fi
log "[Splat-LOAM] smoke test ok"

log "[Splat-LOAM] start"; mkdir -p outputs/splatloam; s=$(date +%s)
$RUN python run.py slam /work/configs/splatloam_snowpole.yaml > outputs/splatloam/run.log 2>&1
echo "{\"wall_s\": $(( $(date +%s) - s )), \"n_scans\": 5419}" > outputs/splatloam/timing_wall.json
EST=$(find outputs/splatloam -name "*.txt" ! -name "*_tum.txt" -printf "%T@ %p\n" | sort -nr | head -1 | cut -d" " -f2-)
[ -n "$EST" ] || { log "[Splat-LOAM] FAILED - no trajectory, see outputs/splatloam/run.log"; exit 1; }
# TUM written with times.txt (seconds since the first scan) -> back to the Ouster sensor clock
T0=$($PY -c "import pandas as pd; print(repr(float(pd.read_csv('cache/scan_stamps.csv').sensor_stamp[0])))")
$PY -c "
import numpy as np; d=np.loadtxt('$EST'); d[:,0]+=$T0; np.savetxt('outputs/splatloam/splatloam_tum.txt', d, fmt='%.9f')
print(len(d), 'poses ->', 'outputs/splatloam/splatloam_tum.txt')"
bash scripts/13_score_method.sh Splat-LOAM outputs/splatloam/splatloam_tum.txt splatloam 1
[ -f outputs/scores/Splat-LOAM_seed0.csv ] || { log "[Splat-LOAM] FAILED scoring"; exit 1; }
log "[Splat-LOAM] done"
