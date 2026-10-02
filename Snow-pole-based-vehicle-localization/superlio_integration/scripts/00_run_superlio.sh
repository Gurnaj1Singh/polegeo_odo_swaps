#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# Stage 0: run Super-LIO (native ROS 2 Jazzy) on the ROS 2 dataset bag and
# capture its odometry (/lio/odom) + the per-stage compute timing (FPS).
#
# Unlike Faster-LIO (ROS1, Docker) this runs natively: Super-LIO's active branch
# is ROS 2 Jazzy, matching this host. We reuse the ROS 2 bag produced for GLIM
# (/ouster/points + /ouster/imu) — no new conversion needed.
#
# Super-LIO has no offline bag app, so we play the bag; the FPS is taken from the
# node's own per-stage timer (lio.eva.timer: true), which is rate-independent and
# is flushed by printTimeRecord() when the node gets SIGINT.
#
# Usage: 00_run_superlio.sh [ros2_bag_dir] [play_rate] [config_yaml]
#   ros2_bag_dir  default: ../glim_integration/output/ros2_bag
#   play_rate     default: 1.0   (drop to 0.5 if scans are dropped)
#   config_yaml   default: config/ouster_os2_128_base.yaml
# ---------------------------------------------------------------------------
set -euo pipefail

SCRIPTS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INTEG="$(dirname "$SCRIPTS")"                       # superlio_integration
REPO="$(dirname "$INTEG")"                          # Snow-pole project dir
FLIO_ROOT="$(dirname "$REPO")"                      # Fasterlio (holds Super-LIO/)
WS="$FLIO_ROOT/Super-LIO"
OUT="$INTEG/output"
mkdir -p "$OUT"

BAG="${1:-$REPO/glim_integration/output/ros2_bag}"
RATE="${2:-1.0}"
CFG="${3:-$INTEG/config/ouster_os2_128_base.yaml}"
LABEL="$(basename "$CFG" .yaml)"
ODOM_BAG="$OUT/superlio_odom"
LOG="$OUT/superlio_run_${LABEL}.log"

# Portable path: run Super-LIO inside Docker (no host ROS 2 Jazzy / colcon build).
# Opt in with SUPERLIO_DOCKER=1; everything downstream (the recorded odom bag, the
# bridge, the pipeline) is identical to the native path.
if [ "${SUPERLIO_DOCKER:-0}" = "1" ]; then
  exec "$INTEG/docker/run_superlio_docker.sh" "$BAG" "$RATE" "$CFG"
fi

[ -d "$BAG" ] || { echo "ROS2 bag dir not found: $BAG" >&2; exit 1; }
[ -f "$WS/install/setup.bash" ] || { echo "Super-LIO not built: $WS/install (run 'cd $WS && colcon build')" >&2; exit 1; }
[ -f "$CFG" ] || { echo "config not found: $CFG" >&2; exit 1; }

echo "[00] sourcing ROS 2 Jazzy + Super-LIO workspace"
set +u; source /opt/ros/jazzy/setup.bash; source "$WS/install/setup.bash"; set -u

rm -rf "$ODOM_BAG"
echo "[00] recording /lio/odom -> $ODOM_BAG"
ros2 bag record -o "$ODOM_BAG" /lio/odom >"$OUT/_rec_${LABEL}.log" 2>&1 &
REC_PID=$!
sleep 2

echo "[00] launching super_lio_node (config: $CFG)  log: $LOG"
ros2 run super_lio super_lio_node --ros-args --params-file "$CFG" >"$LOG" 2>&1 &
NODE_PID=$!
sleep 3

echo "[00] playing ROS 2 bag: $BAG (rate=$RATE)"
ros2 bag play "$BAG" --rate "$RATE"

echo "[00] play finished; draining callbacks + flushing timing"
sleep 3
# SIGINT the ACTUAL node binary — the `ros2 run` wrapper (PID=$NODE_PID) does NOT
# forward signals to its child, so we target the binary directly. This makes
# spin() return -> printTimeRecord() flushes the per-stage FPS.
pkill -INT -f 'super_lio/lib/super_lio/super_lio_node' 2>/dev/null || true
sleep 3
# ros2 bag record ignores SIGINT in this environment; send INT then TERM
# (TERM finalizes the mcap + writes metadata.yaml cleanly).
pkill -INT  -f "bag record .*superlio_odom" 2>/dev/null || true
sleep 2
pkill -TERM -f "bag record .*superlio_odom" 2>/dev/null || true
sleep 2
wait "$NODE_PID" 2>/dev/null || true
wait "$REC_PID"  2>/dev/null || true

echo ""
echo "[00] ===== Super-LIO per-stage compute timing ($LABEL) ====="
grep -E "Using Lidar type|average time usage" "$LOG" || echo "  (no timer output — inspect $LOG)"

# rate-independent throughput: sum the per-stage mean ms -> ms/scan -> FPS @10 Hz
awk '/average time usage/ {
        for (i = 1; i <= NF; i++) if ($i == "usage:") { s += $(i+1); n++ }
     }
     END {
        if (n > 0)
          printf("[00] per-scan compute = %.2f ms  (%d stages)  ->  %.1f FPS  =  %.2fx realtime @10Hz\n",
                 s, n, 1000.0/s, (1000.0/s)/10.0)
        else
          print "[00] (could not parse stage timings — check the log / eva.timer)"
     }' "$LOG"

echo "[00] pose count in odom bag:"
( set +u; source /opt/ros/jazzy/setup.bash
  ros2 bag info "$ODOM_BAG" 2>/dev/null | grep -E "Count|/lio/odom" || true )

echo "[00] DONE. odom bag: $ODOM_BAG   log: $LOG"
