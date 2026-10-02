#!/usr/bin/env bash
# Run faster-lio in OFFLINE mode (reads the bag directly at max CPU speed) as a
# clean, rate-independent throughput benchmark. Prints the per-scan mean compute
# time and "Faster LIO average FPS" (= realtime factor at 10 Hz), and writes a
# TUM trajectory the stage-1 bridge can consume.
#
# Usage: run_offline_bench.sh <config_basename> <label>
#   e.g. run_offline_bench.sh ouster_os2_128_offline_base.yaml base
set -euo pipefail

CFG_NAME="${1:?config basename under fasterlio_integration/config/}"
LABEL="${2:?short label, e.g. base or ds}"

INTEG="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"     # fasterlio_integration
REPO="$(dirname "$INTEG")"                                    # Snow-pole project dir
IMAGE="fasterlio:noetic"
BAG="/work/snow_pole_geo_localization_data/2024-02-28-12-59-51.bag"
CFG="/work/fasterlio_integration/config/${CFG_NAME}"
TRAJ="/work/fasterlio_integration/output/fl_traj_offline_${LABEL}.txt"
TLOG="/work/fasterlio_integration/output/fl_time_offline_${LABEL}.log"
HOSTLOG="$INTEG/output/offline_bench_${LABEL}.log"

echo "[bench] label=$LABEL config=$CFG_NAME" | tee "$HOSTLOG"
t0=$(date +%s)
docker run --rm -v "$REPO:/work:rw" -w /work "$IMAGE" bash -lc "
  source /opt/ros/noetic/setup.bash; source /root/ws/devel/setup.bash
  rosrun faster_lio run_mapping_offline \
    --config_file=$CFG --bag_file=$BAG \
    --traj_log_file=$TRAJ --time_log_file=$TLOG
" 2>&1 | tee -a "$HOSTLOG"
t1=$(date +%s)
echo "[bench] WALLCLOCK_SECONDS=$((t1 - t0))" | tee -a "$HOSTLOG"
echo "[bench] DONE label=$LABEL" | tee -a "$HOSTLOG"
