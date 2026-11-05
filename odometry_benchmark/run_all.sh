#!/usr/bin/env bash
# End-to-end reproduction of the RKO-LIO / GenZ-ICP / MAD-ICP snow-pole results.
#
#   bash run_all.sh                     # writes everything to ./outputs_rerun and checks it
#   OUT=outputs CACHE=cache bash run_all.sh
#
# Needs: the Windows venv (env/windows_venv_requirements.lock.txt), Docker Desktop,
# the two SnowPole bags in ../snow_pole_geo_localization_data (checksums in env/data_sha256.txt).
set -euo pipefail
cd "$(dirname "$0")"
export MSYS_NO_PATHCONV=1 PYTHONIOENCODING=utf-8

PY=${PY:-../venv/Scripts/python}
OUT=${OUT:-outputs_rerun}
CACHE=${CACHE:-cache_rerun}
IMAGE=odom-bench:1.0
FULL_BAG=../snow_pole_geo_localization_data/2024-02-28-12-59-51.bag
mkdir -p "$OUT"/{rko_lio,genz_icp,mad_icp,kiss_icp,csv,scores} "$CACHE" bag_full
log() { printf '\n=== %s ===\n' "$*"; }

log "0. environment checks"
"$PY" -c "import rko_lio, importlib.metadata as m; assert m.version('rko_lio')=='0.4.0', m.version('rko_lio')"
docker image inspect "$IMAGE" >/dev/null 2>&1 || docker build -t "$IMAGE" -f docker/Dockerfile .
# rko_lio reads every *.bag in a folder, so it gets a folder holding only the full bag
[ -e bag_full/2024-02-28-12-59-51.bag ] || powershell -NoProfile -Command \
  "New-Item -ItemType HardLink -Path bag_full/2024-02-28-12-59-51.bag -Target $FULL_BAG" >/dev/null

log "1. RKO-LIO (native)"
"${PY%python}rko_lio" -d rosbag -c configs/rko_lio_os2_128.yaml --imu /ouster/imu --lidar /ouster/points \
  -o "$OUT/rko_lio" -n rko_lio bag_full > "$OUT/rko_lio/run.log" 2>&1

log "2. scan extraction for the Docker methods"
[ -f "$CACHE/scan_stamps.csv" ] || "$PY" scripts/00_extract_scans.py --bag bag_full/2024-02-28-12-59-51.bag --out "$CACHE"

log "3. GenZ-ICP, MAD-ICP (Docker) and KISS-ICP (native), in parallel"
# GenZ-ICP and KISS-ICP sum with tbb::parallel_reduce, whose floating-point order
# depends on the thread split; pinning each to ONE CPU makes them bit-reproducible
# (oneTBB sizes its pool from the affinity mask). MAD-ICP and RKO-LIO are
# bit-reproducible multithreaded.
docker run --rm --cpuset-cpus=2 -v "$PWD:/work" "$IMAGE" python -u scripts/01_run_lidar_odometry.py \
  --method genz_icp --cache "$CACHE" --out "$OUT/genz_icp" --threads 1 > "$OUT/genz_icp/run.log" 2>&1 &
docker run --rm -v "$PWD:/work" "$IMAGE" python -u scripts/01_run_lidar_odometry.py \
  --method mad_icp --cache "$CACHE" --out "$OUT/mad_icp" --threads 4 > "$OUT/mad_icp/run.log" 2>&1 &
"$PY" -u scripts/01_run_lidar_odometry.py --method kiss_icp --cache "$CACHE" --out "$OUT/kiss_icp" \
  --threads 1 --cpu 6 > "$OUT/kiss_icp/run.log" 2>&1 &
wait

log "4. bridge trajectories to per-GNSS-frame UTM CSVs"
cp fixed_inputs/incremental_navigation_results_fastreg.csv "$OUT/csv/"
B="--ref-cache $CACHE/gnss_clock.npz"
"$PY" scripts/02_bridge_to_csv.py $B --tum "$OUT/rko_lio/rko_lio_0/rko_lio_0_tum.txt" --out "$OUT/csv/incremental_navigation_results_rko_lio.csv"
"$PY" scripts/02_bridge_to_csv.py $B --level-gravity --tum "$OUT/genz_icp/genz_icp_tum.txt" --out "$OUT/csv/incremental_navigation_results_genz_icp.csv"
"$PY" scripts/02_bridge_to_csv.py $B --level-gravity --tum "$OUT/mad_icp/mad_icp_tum.txt" --out "$OUT/csv/incremental_navigation_results_mad_icp.csv"
"$PY" scripts/02_bridge_to_csv.py $B --level-gravity --tum "$OUT/kiss_icp/kiss_icp_tum.txt" --out "$OUT/csv/incremental_navigation_results_kiss_icp.csv"

log "5. YOLO pole detections (backend independent)"
[ -f "$CACHE/detections.pkl" ] || "$PY" scripts/03_cache_detections.py --out "$CACHE/detections.pkl" > "$CACHE/detections.log" 2>&1

log "6. GNSS sweep (seed 0 at 0/10/25/50 %, seeds 1-2 at 10/25/50 %)"
for m in fastreg:FastReg kiss_icp:KISS-ICP rko_lio:RKO-LIO genz_icp:GenZ-ICP mad_icp:MAD-ICP; do
  csv="$OUT/csv/incremental_navigation_results_${m%%:*}.csv"
  "$PY" scripts/04_gnss_sweep.py --csv "$csv" --name "${m##*:}" --seed 0 --pcts 0 10 25 50 \
    --cache-dir "$CACHE" --out-dir "$OUT/scores" 2>/dev/null | tail -1
  for s in 1 2; do
    "$PY" scripts/04_gnss_sweep.py --csv "$csv" --name "${m##*:}" --seed $s --pcts 10 25 50 \
      --cache-dir "$CACHE" --out-dir "$OUT/scores" 2>/dev/null | tail -1
  done
done

log "7. compare with the published table"
"$PY" scripts/05_check_results.py --run "$OUT"
