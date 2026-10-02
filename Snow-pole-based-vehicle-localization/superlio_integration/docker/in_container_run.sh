#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# Runs INSIDE the superlio:jazzy container (mounted in at runtime, not baked into
# the image, so edits don't need a rebuild). Mirrors scripts/00_run_superlio.sh:
# record /lio/odom, launch the node, play the bag, then SIGINT the node binary to
# flush printTimeRecord() (the rate-independent FPS) and TERM the recorder.
#
# Args:  <ros2_bag_dir> <config_yaml> <rate> <out_dir>
# ---------------------------------------------------------------------------
set -uo pipefail
BAG="${1:?ros2_bag dir}"; CFG="${2:?config yaml}"; RATE="${3:-3.0}"; OUT="${4:-/out}"

set +u   # ROS 2 setup scripts reference unset vars — don't let -u abort the source
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
set -u
mkdir -p "$OUT"
LOG="$OUT/superlio_run.log"

rm -rf "$OUT/superlio_odom"
echo "[docker] recording /lio/odom -> $OUT/superlio_odom"
ros2 bag record -o "$OUT/superlio_odom" /lio/odom >"$OUT/_rec.log" 2>&1 &
sleep 2

echo "[docker] launching super_lio_node (config: $CFG)  log: $LOG"
ros2 run super_lio super_lio_node --ros-args --params-file "$CFG" >"$LOG" 2>&1 &
sleep 3

echo "[docker] playing ROS 2 bag: $BAG (rate=$RATE)"
ros2 bag play "$BAG" --rate "$RATE"

echo "[docker] draining + flushing timing"
sleep 3
pkill -INT  -f 'super_lio/lib/super_lio/super_lio_node' 2>/dev/null || true
sleep 3
pkill -INT  -f 'bag record .*superlio_odom' 2>/dev/null || true
sleep 2
pkill -TERM -f 'bag record .*superlio_odom' 2>/dev/null || true
sleep 2

echo ""
echo "[docker] ===== Super-LIO per-stage compute timing ====="
grep -E "Using Lidar type|average time usage" "$LOG" || echo "  (no timer output — inspect superlio_run.log)"
awk '/average time usage/ {
        for (i = 1; i <= NF; i++) if ($i == "usage:") { s += $(i+1); n++ }
     }
     END {
        if (n > 0)
          printf("[docker] per-scan compute = %.2f ms  (%d stages)  ->  %.1f FPS  =  %.2fx realtime @10Hz\n",
                 s, n, 1000.0/s, (1000.0/s)/10.0)
        else
          print "[docker] (could not parse stage timings — check superlio_run.log)"
     }' "$LOG"

echo "[docker] pose count in odom bag:"
ros2 bag info "$OUT/superlio_odom" 2>/dev/null | grep -E "Count|/lio/odom" || true
echo "[docker] DONE. odom bag: $OUT/superlio_odom"
