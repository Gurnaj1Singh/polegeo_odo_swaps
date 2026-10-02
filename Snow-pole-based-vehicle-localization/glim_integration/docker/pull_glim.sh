#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# Pull the prebuilt GLIM image. Default = CUDA build matching this host
# (ROS 2 Jazzy + driver 595 → CUDA 13.1). No sudo needed (docker works as user).
#
#   docker/pull_glim.sh            # GPU image (default)
#   docker/pull_glim.sh cpu        # CPU-only fallback (validation, NOT the speed test)
#
# Sizes are large (~10-15 GB). Ensure disk headroom first (see PLAN.md §7).
# ---------------------------------------------------------------------------
set -euo pipefail

VARIANT="${1:-gpu}"
case "$VARIANT" in
  gpu) IMAGE="koide3/glim_ros2:jazzy_cuda13.1" ;;
  cpu) IMAGE="koide3/glim_ros2:jazzy" ;;
  *)   echo "usage: $0 [gpu|cpu]" >&2; exit 2 ;;
esac

echo "[pull] $IMAGE"
docker pull "$IMAGE"
echo "[pull] done. Local GLIM images:"
docker images | grep -E 'koide3/glim' || true
