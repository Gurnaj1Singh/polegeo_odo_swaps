#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# One-time HOST setup: install + wire NVIDIA Container Toolkit so Docker can
# use the GPU (`docker run --gpus all`). Needed for the GLIM CUDA image.
#
# Needs root. Run as:
#   ! sudo bash Snow-pole-based-vehicle-localization/glim_integration/docker/setup_nvidia_container_toolkit.sh
#
# Idempotent: safe to re-run. Verified target: Ubuntu/Mint, NVIDIA driver 595,
# RTX 3050 Ti. Does NOT touch the GPU driver (already present per nvidia-smi).
# ---------------------------------------------------------------------------
set -euo pipefail

SUDO=""; [ "$(id -u)" -ne 0 ] && SUDO="sudo"

echo "[toolkit] adding NVIDIA container-toolkit apt repo"
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
  | $SUDO gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
  | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
  | $SUDO tee /etc/apt/sources.list.d/nvidia-container-toolkit.list >/dev/null

echo "[toolkit] installing nvidia-container-toolkit"
$SUDO apt-get update
$SUDO apt-get install -y nvidia-container-toolkit

echo "[toolkit] configuring the docker runtime + restarting docker"
$SUDO nvidia-ctk runtime configure --runtime=docker
$SUDO systemctl restart docker

echo "[toolkit] smoke test: nvidia-smi inside a container"
# Uses an image already present locally (see 'docker images') to avoid a download.
TESTIMG="nvidia/cuda:11.0.3-base-ubuntu20.04"
if docker run --rm --gpus all "$TESTIMG" nvidia-smi -L; then
  echo "[toolkit] OK — GPU is visible inside Docker."
else
  echo "[toolkit] FAILED — GPU not visible in container. Check 'nvidia-ctk' + docker restart." >&2
  exit 1
fi
