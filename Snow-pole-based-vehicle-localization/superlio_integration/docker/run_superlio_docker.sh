#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# Build (if needed) and run Super-LIO odometry INSIDE Docker — no host ROS 2
# Jazzy / colcon build required (the image carries the built workspace). Produces
# the same output/superlio_odom bag + per-stage FPS as the native
# scripts/00_run_superlio.sh, so the bridge (10_) and the rest of the pipeline are
# unchanged. All ROS 2 runs inside the container (node + bag play + record), so the
# host needs only Docker.
#
# Usage:  run_superlio_docker.sh [ros2_bag_dir] [rate] [config_yaml]
#   ros2_bag_dir  default: ../glim_integration/output/ros2_bag
#   rate          default: 3.0   (reliable-QoS source mod keeps all scans)
#   config_yaml   default: config/ouster_os2_128_base.yaml
# ---------------------------------------------------------------------------
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # .../superlio_integration/docker
INTEG="$(dirname "$HERE")"                              # superlio_integration
REPO="$(dirname "$INTEG")"                              # Snow-pole project dir
FLIO_ROOT="$(dirname "$REPO")"                          # repo root (holds Super-LIO/)
SUPERLIO="$FLIO_ROOT/Super-LIO"
OUT="$INTEG/output"; mkdir -p "$OUT"

BAG="${1:-$REPO/glim_integration/output/ros2_bag}"
RATE="${2:-3.0}"
CFG="${3:-$INTEG/config/ouster_os2_128_base.yaml}"
IMAGE="${SUPERLIO_IMAGE:-superlio:jazzy}"

# pick docker (fall back to sudo if the daemon socket isn't reachable as this user)
DOCKER_BIN="${DOCKER:-docker}"
if ! $DOCKER_BIN info >/dev/null 2>&1; then
  echo "[docker] daemon not reachable as $(id -un); using 'sudo docker'"
  DOCKER_BIN="sudo docker"
fi

[ -d "$SUPERLIO/src" ] || { echo "vendored Super-LIO source missing: $SUPERLIO/src" >&2; exit 1; }
[ -d "$BAG" ] || { echo "ROS 2 bag dir not found: $BAG" >&2; exit 1; }
[ -f "$CFG" ] || { echo "config not found: $CFG" >&2; exit 1; }

if ! $DOCKER_BIN image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "[docker] building $IMAGE from vendored source (first time: apt + colcon, a few minutes)…"
  $DOCKER_BIN build -t "$IMAGE" -f "$INTEG/docker/Dockerfile" "$SUPERLIO"
fi

BAG_ABS="$(cd "$BAG" && pwd)"
CFG_ABS="$(cd "$(dirname "$CFG")" && pwd)/$(basename "$CFG")"
echo "[docker] running Super-LIO on $BAG_ABS (rate=$RATE, config=$(basename "$CFG"))"
$DOCKER_BIN run --rm \
  -v "$BAG_ABS":/data/ros2_bag:ro \
  -v "$OUT":/out \
  -v "$CFG_ABS":/cfg/config.yaml:ro \
  -v "$INTEG/docker/in_container_run.sh":/opt/in_container_run.sh:ro \
  "$IMAGE" bash /opt/in_container_run.sh /data/ros2_bag /cfg/config.yaml "$RATE" /out

echo "[docker] DONE. odom bag: $OUT/superlio_odom   log: $OUT/superlio_run.log"
