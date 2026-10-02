#!/usr/bin/env bash
# Build (if needed) and enter the Faster-LIO Noetic container with the project
# mounted. The bags and the whole repo are bind-mounted read-write at /work.
#
# Usage:
#   fasterlio_integration/docker/run_docker.sh            # interactive shell
#   fasterlio_integration/docker/run_docker.sh <cmd...>   # run one command
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"        # .../docker
INTEG="$(dirname "$HERE")"                                   # .../fasterlio_integration
REPO="$(dirname "$INTEG")"                                   # project root (repo)

IMAGE="fasterlio:noetic"

# --- pick how to invoke docker (this user may not be in the 'docker' group) ---
# Override with DOCKER="sudo docker" if needed. Auto-fallback to sudo when the
# daemon socket is not reachable as the current user.
DOCKER_BIN="${DOCKER:-docker}"
if ! $DOCKER_BIN info >/dev/null 2>&1; then
  echo "[run_docker] docker daemon not reachable as $(id -un); using 'sudo docker'"
  echo "             (tip: 'sudo usermod -aG docker $(id -un)' + re-login removes the need for sudo)"
  DOCKER_BIN="sudo docker"
fi

if ! $DOCKER_BIN image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "[run_docker] building $IMAGE ..."
  $DOCKER_BIN build -t "$IMAGE" -f "$INTEG/docker/Dockerfile" "$INTEG"
fi

# Use an interactive TTY only when we actually have one (so this also works when
# backgrounded / piped to a log file).
TTYFLAGS="-i"; [ -t 0 ] && [ -t 1 ] && TTYFLAGS="-it"

# Allow GUI (RViz) if an X server is available; harmless if not.
XSOCK=/tmp/.X11-unix
xhost +local:root >/dev/null 2>&1 || true

exec $DOCKER_BIN run --rm $TTYFLAGS \
  --net=host \
  -e DISPLAY="${DISPLAY:-}" \
  -v "$XSOCK:$XSOCK:rw" \
  -v "$REPO:/work:rw" \
  -w /work \
  "$IMAGE" "$@"
