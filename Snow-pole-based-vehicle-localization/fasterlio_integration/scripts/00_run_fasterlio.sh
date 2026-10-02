#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# Stage 0: run Faster-LIO on the dataset bag and capture its odometry.
#
# Run this INSIDE the Noetic container (docker/run_docker.sh), which has
# faster_lio built and sourced. It:
#   1. starts roscore
#   2. launches Faster-LIO with our OS-2-128 config
#   3. records /Odometry (+ /path) to a new bag
#   4. plays the dataset bag on the sensor clock
#   5. also copies the TUM trajectory Faster-LIO writes to disk
#
# Output (under fasterlio_integration/output/):
#   fasterlio_odometry.bag     nav_msgs/Odometry stream (authoritative)
#   fasterlio_traj_tum.txt     TUM trajectory (fallback)
#
# Usage (inside container):
#   /work/Snow-pole-based-vehicle-localization/fasterlio_integration/scripts/00_run_fasterlio.sh \
#       /work/Snow-pole-based-vehicle-localization/snow_pole_geo_localization_data/2024-02-28-12-59-51.bag
# ---------------------------------------------------------------------------
set -euo pipefail

BAG="${1:?path to dataset .bag (full bag with /ouster/points recommended)}"
RATE="${2:-1.0}"          # rosbag play rate; use 0.5 if CPU-bound to avoid drops

SCRIPTS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="$(dirname "$SCRIPTS")/output"
mkdir -p "$OUT"

source /opt/ros/noetic/setup.bash
source /root/ws/devel/setup.bash

echo "[00] starting roscore"
roscore & ROSCORE_PID=$!

# Wait until the master actually ANSWERS before launching anything. A fixed
# sleep is not enough: roscore's "checking log disk usage" step can exceed it,
# and then roslaunch starts its OWN master on the shared (--net=host) :11311,
# the two collide, and the mapping node is shut down cleanly at startup --
# before it even loads params -> empty odometry bag -> downstream fails with
# "no /Odometry messages". Polling the master removes that race.
echo "[00] waiting for ROS master..."
for i in $(seq 1 60); do
  rosnode list >/dev/null 2>&1 && break
  if [ "$i" -eq 60 ]; then echo "[00] ERROR: ROS master never came up" >&2; exit 1; fi
  sleep 0.5
done
# use the LiDAR/IMU sensor clock coming from the bag
rosparam set use_sim_time true

echo "[00] launching Faster-LIO (OS-2-128 config)"
# Use the config mounted from the host (/work) so edits take effect WITHOUT
# rebuilding the image. Falls back to the baked-in copy if not mounted.
CFG=/work/fasterlio_integration/config/ouster_os2_128.yaml
[ -f "$CFG" ] || CFG="$(rospack find faster_lio)/config/ouster_os2_128.yaml"
echo "[00] config: $CFG"
roslaunch faster_lio mapping_ouster_os2_128.launch rviz:=false config_file:="$CFG" & LAUNCH_PID=$!

# Verify the mapping node actually came up and cleared InitROS before we commit
# to recording + the full (~18 min) play. The node advertises /Odometry at init
# (before any data), so a publisher on /Odometry is proof it is alive; if it
# never appears the node died at startup and we must NOT record an empty bag.
echo "[00] waiting for /laserMapping to advertise /Odometry..."
NODE_OK=0
for i in $(seq 1 60); do
  if rostopic info /Odometry 2>/dev/null | grep -q "/laserMapping"; then NODE_OK=1; break; fi
  sleep 0.5
done
if [ "$NODE_OK" -ne 1 ]; then
  echo "[00] ERROR: Faster-LIO mapping node did not come up (no /Odometry publisher)." >&2
  echo "[00]        See the node output above for the cause; aborting before the play." >&2
  kill -INT "$LAUNCH_PID" "$ROSCORE_PID" 2>/dev/null || true
  exit 1
fi

echo "[00] recording /Odometry -> $OUT/fasterlio_odometry.bag"
rosbag record -O "$OUT/fasterlio_odometry.bag" /Odometry /path & REC_PID=$!
sleep 2

echo "[00] playing dataset bag: $BAG (rate=$RATE)"
rosbag play --clock -r "$RATE" "$BAG" \
    --topics /ouster/points /ouster/imu

echo "[00] play finished; draining callbacks"
sleep 5
kill -INT "$REC_PID"  2>/dev/null || true
sleep 2
kill -INT "$LAUNCH_PID" 2>/dev/null || true
kill -INT "$ROSCORE_PID" 2>/dev/null || true

# TUM-trajectory fallback. The authoritative output is the /Odometry bag above;
# this text file is only a convenience copy of what the node's Savetrajectory()
# dumps to ./Log/traj.txt at shutdown. Match traj.txt SPECIFICALLY and require it
# to be non-empty (-s): the old glob grabbed the first *.txt it found, which was
# the empty imu_.txt debug stub, leaving a misleading 0-byte fallback. The online
# node often does not reach Savetrajectory() under roslaunch's shutdown, so no
# traj.txt is the normal case -> skip cleanly rather than fabricate a stale file.
TRAJ=""
for f in ./Log/traj.txt "$HOME/.ros/Log/traj.txt" \
         /root/ws/src/faster-lio/Log/traj.txt /root/ws/Log/traj.txt; do
  [ -s "$f" ] && { TRAJ="$f"; break; }
done
if [ -n "$TRAJ" ]; then
  cp -v "$TRAJ" "$OUT/fasterlio_traj_tum.txt"
else
  echo "[00] note: node produced no non-empty traj.txt; the /Odometry bag is authoritative." >&2
  rm -f "$OUT/fasterlio_traj_tum.txt"   # don't leave a stale/empty fallback behind
fi

echo "[00] DONE. Odometry bag: $OUT/fasterlio_odometry.bag"
