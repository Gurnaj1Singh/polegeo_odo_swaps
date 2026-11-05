#!/usr/bin/env bash
# IMU-degradation study for the IPAS paper (paper_ipas/PHASES.md, Phase 3): FAST-LIO2 and Faster-LIO re-run with the
# IMU stream replaced by outputs/imu_variants/<variant>.bag (scripts/22_make_imu_variants.py), all other parameters
# unchanged. rate100 = control (unchanged data through the same replay path). Each trajectory is bridged like the
# paper's (no gravity levelling: LIO) and scored with the Combination-4 detector -> outputs/scores_imu/.
cd "$(dirname "$0")"
export MSYS_NO_PATHCONV=1 PYTHONIOENCODING=utf-8
PY=../venv/Scripts/python
W="D:/Odometry/odometry_benchmark"
log() { echo "$(date +%T) $*"; }
until ! docker ps --format '{{.Image}}' | grep -q .; do sleep 60; done
mkdir -p outputs/scores_imu
for V in rate100 rate50 rate25 rate10 noise3 noise10; do
  for M in fasterlio fastlio2; do
    NAME=$([ $M = fastlio2 ] && echo FAST-LIO2 || echo Faster-LIO)_$V
    O=outputs/imu_degradation/${M}_$V
    if [ ! -s $O/tum.txt ]; then
      log "[$NAME] start"
      docker run --rm -v "$W:/work" -v "$W/bag_full:/bag:ro" -v "$W/configs/imu_degradation:/cfg:ro" fastlio2:noetic \
        bash /work/scripts/23_lio_imu_variant.sh $M $V > outputs/imu_degradation_${M}_$V.log 2>&1
    fi
    [ -s $O/tum.txt ] || { log "[$NAME] FAILED - see outputs/imu_degradation_${M}_$V.log"; continue; }
    CSV=outputs/csv/incremental_navigation_results_imudeg_${M}_$V.csv
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
done
log "[imu-degradation] done"
