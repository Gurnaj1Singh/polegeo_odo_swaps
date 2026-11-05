#!/usr/bin/env bash
# IPAS Phase 3 follow-up (biasfix): gyro bias held at its standstill estimate (configs/imu_tuning/biasfix/), LIO initialised (gravity + gyro bias) over 600 IMU samples = 6 s of the
# standstill (docker/Dockerfile.lio_init6s; upstream averages 10 / 20 samples = 0.1-0.2 s). Default parameters
# (configs/imu_degradation/), unchanged IMU data (outputs/imu_variants/rate100.bag). Scored -> outputs/scores_imu/.
cd "$(dirname "$0")"
export MSYS_NO_PATHCONV=1 PYTHONIOENCODING=utf-8
PY=../venv/Scripts/python
W="D:/Odometry/odometry_benchmark"
log() { echo "$(date +%T) $*"; }
until ! docker ps --format '{{.Image}}' | grep -q .; do sleep 60; done
T=biasfix
for M in fasterlio fastlio2; do
  NAME=$([ $M = fastlio2 ] && echo FAST-LIO2 || echo Faster-LIO)_$T
  O=outputs/imu_degradation/${M}_$T
  if [ ! -s $O/tum.txt ]; then
    log "[$NAME] start"
    docker run --rm -v "$W:/work" -v "$W/bag_full:/bag:ro" -v "$W/configs/imu_tuning/biasfix:/cfg:ro" fastlio2:noetic-init6s \
      bash /work/scripts/24_lio_imu_config.sh $M rate100 $T > outputs/imu_degradation_${M}_$T.log 2>&1
  fi
  [ -s $O/tum.txt ] || { log "[$NAME] FAILED - see outputs/imu_degradation_${M}_$T.log"; continue; }
  CSV=outputs/csv/incremental_navigation_results_imudeg_${M}_$T.csv
  A=$($PY scripts/02_bridge_to_csv.py --tum $O/tum.txt --out $CSV 2>&1 | grep -o "median [0-9.]* m")
  R=$($PY scripts/04_gnss_sweep.py --csv $CSV --name $NAME --seed 0 --pcts 0 2 5 10 25 50 \
        --cache-dir cache_comb4 --out-dir outputs/scores_imu 2>/dev/null \
      | grep -o '"err_pred_median": [0-9.]*' | grep -o '[0-9.]*$' | tr '\n' ' ')
  for s in 1 2; do
    $PY scripts/04_gnss_sweep.py --csv $CSV --name $NAME --seed $s --pcts 10 25 50 \
        --cache-dir cache_comb4 --out-dir outputs/scores_imu >/dev/null 2>&1
  done
  log "[$NAME] $(grep -c . $O/tum.txt) poses, raw drift $A; median m at 0/2/5/10/25/50 %: $R"
done
log "[imu-biasfix] done"
