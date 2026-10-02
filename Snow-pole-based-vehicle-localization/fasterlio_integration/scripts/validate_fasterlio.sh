#!/usr/bin/env bash
# Quick validation: run Faster-LIO on the FIRST N seconds of the bag and report
# whether /Odometry is produced (and how many poses). Runs INSIDE the container.
#   args: <bag_in_container> [seconds=40] [rate=1.0]
set -uo pipefail
BAG="${1:?bag path (in container)}"
SECS="${2:-40}"
RATE="${3:-1.0}"

OUT=/work/fasterlio_integration/output
mkdir -p "$OUT"
NODELOG="$OUT/validate_node.log"
ODOM="$OUT/validate_odom.bag"
rm -f "$ODOM"

source /opt/ros/noetic/setup.bash
source /root/ws/devel/setup.bash

roscore >/dev/null 2>&1 & RC=$!; sleep 3
rosparam set use_sim_time true
roslaunch faster_lio mapping_ouster_os2_128.launch rviz:=false > "$NODELOG" 2>&1 & LP=$!; sleep 5
rosbag record -O "$ODOM" /Odometry >/dev/null 2>&1 & RECP=$!; sleep 2

echo "[validate] playing first ${SECS}s of $BAG"
rosbag play --clock -r "$RATE" -u "$SECS" "$BAG" --topics /ouster/points /ouster/imu
sleep 5
kill -INT $RECP 2>/dev/null; sleep 2
kill -INT $LP $RC 2>/dev/null; sleep 1

echo "===== /Odometry message count ====="
rosbag info "$ODOM" 2>/dev/null | grep -E "messages|/Odometry|duration" || echo "no odom bag"
echo "===== Faster-LIO node log (tail) ====="
tail -n 25 "$NODELOG"
