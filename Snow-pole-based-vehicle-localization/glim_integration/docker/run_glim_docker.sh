#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# Run a command inside the GLIM container with the GPU, our config, and the
# whole repo mounted at /work. Mirrors fasterlio_integration/docker/run_docker.sh.
#
#   docker/run_glim_docker.sh <cmd...>     # run one command
#   docker/run_glim_docker.sh              # interactive shell
#
# Env overrides:
#   GLIM_IMAGE   (default koide3/glim_ros2:jazzy_cuda13.1)
#   GLIM_GPUS    (default "--gpus all"; set "" to force CPU image)
#   GLIM_CONFIG  (default <integration>/config ; bind-mounted at /glim/config)
# ---------------------------------------------------------------------------
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"        # .../docker
INTEG="$(dirname "$HERE")"                                   # .../glim_integration
REPO="$(dirname "$INTEG")"                                   # Snow-pole project dir

IMAGE="${GLIM_IMAGE:-koide3/glim_ros2:jazzy_cuda13.1}"
GPUS="${GLIM_GPUS:---gpus all}"
CONFIG="${GLIM_CONFIG:-$INTEG/config}"

# docker without sudo works on this host; keep the auto-sudo fallback anyway.
DOCKER_BIN="${DOCKER:-docker}"
if ! $DOCKER_BIN info >/dev/null 2>&1; then
  echo "[run_glim] docker not reachable as $(id -un); using sudo docker"
  DOCKER_BIN="sudo docker"
fi

if ! $DOCKER_BIN image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "[run_glim] image $IMAGE not found locally — run docker/pull_glim.sh first" >&2
  exit 1
fi

TTYFLAGS="-i"; [ -t 0 ] && [ -t 1 ] && TTYFLAGS="-it"

# Optional extra docker args (e.g. extra -v mounts). Word-split into an array.
EXTRA=()
if [ -n "${GLIM_MOUNTS:-}" ]; then
  # shellcheck disable=SC2206
  EXTRA=($GLIM_MOUNTS)
fi

# Mount X only if a display is actually available (so headless/background works).
XARGS=()
if [ -n "${DISPLAY:-}" ] && [ -S /tmp/.X11-unix/X"${DISPLAY##*:}" ] 2>/dev/null; then
  xhost +local:root >/dev/null 2>&1 || true
  XARGS=(-e DISPLAY="$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix:rw)
fi

exec $DOCKER_BIN run --rm $TTYFLAGS $GPUS \
  --net=host --ipc=host --pid=host \
  "${XARGS[@]}" "${EXTRA[@]}" \
  -v "$CONFIG:/glim/config:rw" \
  -v "$REPO:/work:rw" \
  -w /work \
  "$IMAGE" "$@"
