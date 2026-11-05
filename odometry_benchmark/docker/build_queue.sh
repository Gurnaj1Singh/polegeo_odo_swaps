#!/usr/bin/env bash
# Build the method images one at a time (parallel CUDA/C++ builds exhausted the Docker VM's
# 8 GB on 2026-10-17). Skips images that already exist.
cd "$(dirname "$0")/.."
FL=third_party/polegeo_odo_swaps/Snow-pole-based-vehicle-localization/fasterlio_integration
build() {  # tag dockerfile context
  docker image inspect "$1" >/dev/null 2>&1 && { echo "[have] $1"; return; }
  echo "[build] $1 ($(date +%T))"
  docker build -t "$1" -f "$2" "$3" > "docker/build_${1%%:*}.log" 2>&1 \
    && echo "[ok] $1 ($(date +%T))" || { echo "[FAILED] $1 - see docker/build_${1%%:*}.log"; grep -E " error|ERROR" "docker/build_${1%%:*}.log" | tail -3; }
}
until docker image inspect deeppointmap:cu128 >/dev/null 2>&1 || ! pgrep -f "Dockerfile.deeppointmap" >/dev/null 2>&1; do sleep 20; done
build bufferx:cu128    docker/Dockerfile.bufferx    .
build ugp:cu128        docker/Dockerfile.ugp        .
build hiloam:cu128     docker/Dockerfile.hiloam     .
build fasterlio:noetic docker/Dockerfile.fasterlio  "$FL"
build superlio:jazzy   docker/Dockerfile.superlio  third_party/polegeo_odo_swaps/Super-LIO
echo "[queue done] $(date +%T)"
