#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# Stage 02: run GLIM on the converted ROS2 bag, GPU-accelerated + headless, and
# capture the estimated trajectory (TUM) for the bridge. See PLAN.md §2.
#
# glim_rosbag reads the bag directly and auto-throttles to avoid dropping scans.
# GLIM dumps trajectories to /tmp/dump on close; we bind-mount that to the host
# so traj_lidar.txt survives the --rm container.
#
#   scripts/02_run_glim.sh [label]        # label defaults to "gpu"
#
# Outputs (output/):
#   glim_dump/traj_lidar.txt   authoritative TUM trajectory (sensor-clock stamps)
#   glim_traj_<label>.txt      copy the bridge consumes
#   glim_run_<label>.log       full GLIM log (per-frame timing)
# ---------------------------------------------------------------------------
set -euo pipefail

SCRIPTS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INTEG="$(dirname "$SCRIPTS")"
REPO="$(dirname "$INTEG")"
OUT="$INTEG/output"
LABEL="${1:-gpu}"
BAG_SECONDS="${BAG_SECONDS:-541.8}"          # dataset bag duration (for realtime factor)

BAG_HOST="$OUT/ros2_bag"
DUMP_HOST="$OUT/glim_dump"
DUMP_NAME="$(basename "$DUMP_HOST")"
LOG="$OUT/glim_run_${LABEL}.log"
BAG_IN="/work/glim_integration/output/ros2_bag"
IMAGE="${GLIM_IMAGE:-koide3/glim_ros2:jazzy_cuda13.1}"   # for root-owned dump cleanup/chown

[ -d "$BAG_HOST" ] || { echo "converted ROS2 bag missing: run 00_convert_bag.sh" >&2; exit 1; }
[ -f "$INTEG/config/config.json" ] || { echo "configs missing: run 01_extract_and_patch_configs.sh" >&2; exit 1; }

# The GLIM container runs as root, so a previous run leaves root-owned files in the
# dump that the host user cannot delete (plain `rm -rf` -> "Permission denied" ->
# set -e aborts the whole pipeline on any RE-run). Clean as root via the image (no
# sudo needed) when the host-side rm can't.
if [ -d "$DUMP_HOST" ]; then
  rm -rf "$DUMP_HOST" 2>/dev/null || \
    docker run --rm -v "$OUT:/out" "$IMAGE" bash -lc "rm -rf /out/$DUMP_NAME"
fi
mkdir -p "$DUMP_HOST"

echo "[02] running GLIM (label=$LABEL) on $BAG_IN — headless, --gpus all" | tee "$LOG"
# auto_quit:=true is ESSENTIAL: glim_rosbag otherwise blocks on a keyboard press
# after playback (headless => hangs forever). With auto_quit it processes to the
# end, saves to /tmp/dump, and exits. (glim_rosbag.cpp: wait(auto_quit); save().)
t0=$(date +%s)
GLIM_MOUNTS="-v $DUMP_HOST:/tmp/dump:rw" \
  "$INTEG/docker/run_glim_docker.sh" \
    ros2 run glim_ros glim_rosbag "$BAG_IN" \
      --ros-args -p config_path:=/glim/config -p auto_quit:=true 2>&1 | tee -a "$LOG"
t1=$(date +%s); WALL=$((t1 - t0))

# Report the TRUE throughput from GLIM's own processing timestamps (span from the
# first to the last processing log line, ignoring any >60s idle gap), which is
# robust even if the container lingers. Wall-clock is kept as a secondary figure.
echo "[02] wall-clock ${WALL}s for a ${BAG_SECONDS}s bag" | tee -a "$LOG"
python3 - "$LOG" "$BAG_SECONDS" <<'PY' | tee -a "$LOG"
import re, sys, datetime
log, bag = sys.argv[1], float(sys.argv[2])
ts = []
for line in open(log):
    m = re.match(r'\[(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d\.\d+)\] \[(odom|glim|local|global)\]', line)
    if m: ts.append(datetime.datetime.fromisoformat(m.group(1)))
if len(ts) < 2:
    print("[02] REALTIME_FACTOR=NA (no processing timestamps in log)"); sys.exit()
proc = [ts[0]]
for a, b in zip(ts, ts[1:]):
    if (b - a).total_seconds() > 60: break
    proc.append(b)
span = (proc[-1] - proc[0]).total_seconds()
if span > 0:
    print(f"[02] processing span {span:.1f}s -> REALTIME_FACTOR={bag/span:.2f}x (bag/processing)")
PY

# The dump was written by the container as root; hand it back to the host user so
# the harvest below — and the NEXT run's cleanup — don't need root.
docker run --rm -v "$OUT:/out" "$IMAGE" \
  bash -lc "chown -R $(id -u):$(id -g) /out/$DUMP_NAME" 2>/dev/null || true

# Harvest the trajectory dump (GLIM writes TUM: timestamp x y z qx qy qz qw).
if [ -f "$DUMP_HOST/traj_lidar.txt" ]; then
  cp -v "$DUMP_HOST/traj_lidar.txt" "$OUT/glim_traj_${LABEL}.txt"
  [ -f "$DUMP_HOST/traj_imu.txt" ] && cp -v "$DUMP_HOST/traj_imu.txt" "$OUT/glim_traj_imu_${LABEL}.txt"
  echo "[02] DONE -> $OUT/glim_traj_${LABEL}.txt ($(wc -l < "$OUT/glim_traj_${LABEL}.txt") poses)"
else
  echo "[02] WARNING: no traj_lidar.txt in $DUMP_HOST." >&2
  echo "     GLIM may not have auto-dumped. Contents:" >&2; ls -la "$DUMP_HOST" >&2
  echo "     If empty, GLIM needs an explicit save — see README 'Trajectory dump'." >&2
  exit 2
fi
