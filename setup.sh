#!/usr/bin/env bash
# =============================================================================
# Top-level bootstrap for the combined Fasterlio/ repo.
#
# Run this ONCE after cloning, from the repo root:   ./setup.sh
#
# It performs the pieces that can be automated portably, and for the pieces that
# genuinely can't (installing ROS 2 Jazzy, the NVIDIA container toolkit, and the
# 47 GB Kaggle dataset) it DETECTS what's present and prints the exact next step.
# It is idempotent: already-done steps are skipped, so re-running is safe.
#
# By default it sets up the lightweight, always-safe things:
#   * .baginspect_venv          (pure-python `rosbags` — the odometry→CSV bridges)
#   * the `polegeo` conda env    (pipeline + visualization; if miniconda is present)
#   * builds Super-LIO           (if ROS 2 Jazzy is installed on the host)
# The heavy Docker images are OPT-IN (they are 1–15 GB) — see flags below.
#
# Usage:
#   ./setup.sh                      # venv + conda env + Super-LIO build + verify/guide
#   ./setup.sh --apt                # also `sudo apt` the Super-LIO build deps
#   ./setup.sh --fasterlio-image    # also build the Faster-LIO Docker image (ROS 1)
#   ./setup.sh --glim-image[=gpu|cpu]  # also pull the GLIM Docker image (~10–15 GB)
#   ./setup.sh --all-images         # both Docker images
#   ./setup.sh --skip-conda --skip-venv --skip-superlio   # opt out of pieces
#   ./setup.sh -h | --help
# =============================================================================
set -uo pipefail   # NOT -e: we want to continue past a missing optional dep and report it

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SNOW="$ROOT/Snow-pole-based-vehicle-localization"
SUPERLIO="$ROOT/Super-LIO"
VENV="$ROOT/.baginspect_venv"
DATA_DIR="$SNOW/snow_pole_geo_localization_data"
POLEGEO_PY="$HOME/miniconda3/envs/polegeo/bin/python"

# ---- flags ------------------------------------------------------------------
DO_APT=0 DO_FL_IMAGE=0 DO_GLIM_IMAGE="" SKIP_CONDA=0 SKIP_VENV=0 SKIP_SUPERLIO=0
for a in "$@"; do
  case "$a" in
    --apt)              DO_APT=1 ;;
    --fasterlio-image)  DO_FL_IMAGE=1 ;;
    --glim-image)       DO_GLIM_IMAGE="gpu" ;;
    --glim-image=gpu)   DO_GLIM_IMAGE="gpu" ;;
    --glim-image=cpu)   DO_GLIM_IMAGE="cpu" ;;
    --all-images)       DO_FL_IMAGE=1; DO_GLIM_IMAGE="gpu" ;;
    --skip-conda)       SKIP_CONDA=1 ;;
    --skip-venv)        SKIP_VENV=1 ;;
    --skip-superlio)    SKIP_SUPERLIO=1 ;;
    -h|--help)          sed -n '2,33p' "$0"; exit 0 ;;
    *) echo "unknown flag: $a  (use -h for help)" >&2; exit 2 ;;
  esac
done

# ---- pretty logging + status accumulators -----------------------------------
if [ -t 1 ]; then B=$'\e[1m'; G=$'\e[32m'; Y=$'\e[33m'; R=$'\e[31m'; C=$'\e[36m'; Z=$'\e[0m'
else B= G= Y= R= C= Z=; fi
declare -a READY TODO
hdr()  { printf '\n%s========== %s ==========%s\n' "$B" "$1" "$Z"; }
ok()   { printf '  %s✓%s %s\n' "$G" "$Z" "$1"; READY+=("$1"); }
info() { printf '  %s•%s %s\n' "$C" "$Z" "$1"; }
warn() { printf '  %s!%s %s\n' "$Y" "$Z" "$1"; }
todo() { printf '  %s→ TODO%s %s\n' "$Y" "$Z" "$1"; TODO+=("$1"); }
have() { command -v "$1" >/dev/null 2>&1; }

[ -d "$SNOW" ] || { echo "${R}Run this from the repo root (Fasterlio/); $SNOW not found.${Z}" >&2; exit 1; }

# ---- 0. detect the environment ----------------------------------------------
hdr "Environment detection"
have conda && info "conda:        $(command -v conda)" || { [ -x "$HOME/miniconda3/bin/conda" ] && info "conda:        $HOME/miniconda3/bin/conda" || warn "conda:        not found"; }
[ -f /opt/ros/jazzy/setup.bash ] && info "ROS 2 Jazzy:  /opt/ros/jazzy" || warn "ROS 2 Jazzy:  not found (needed by Super-LIO)"
have colcon && info "colcon:       yes" || warn "colcon:       not found (ros-jazzy / python3-colcon-common-extensions)"
have docker && info "docker:       $(command -v docker)" || warn "docker:       not found (needed by Faster-LIO & GLIM)"
have nvidia-smi && info "NVIDIA GPU:   $(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1)" || warn "NVIDIA GPU:   not detected (GLIM GPU path needs one)"
have python3 && info "python3:      $(python3 --version 2>&1)" || warn "python3:      not found"

# ---- 1. .baginspect_venv (odometry→CSV bridge interpreter) ------------------
if [ "$SKIP_VENV" -eq 0 ]; then
  hdr "Bridge venv (.baginspect_venv)"
  if [ -x "$VENV/bin/rosbags-convert" ]; then
    ok ".baginspect_venv already present"
  elif have python3; then
    if python3 -m venv "$VENV" \
       && "$VENV/bin/pip" install -q --upgrade pip \
       && "$VENV/bin/pip" install -q numpy pandas scipy pyproj rosbags; then
      ok "created .baginspect_venv (rosbags + bridge deps)"
    else
      todo "venv creation failed — ensure python3-venv + network, then re-run"
    fi
  else
    todo "install python3 (and python3-venv), then re-run for .baginspect_venv"
  fi
fi

# ---- 2. polegeo conda env (pipeline + visualization) ------------------------
if [ "$SKIP_CONDA" -eq 0 ]; then
  hdr "Pipeline conda env (polegeo)"
  if [ -x "$POLEGEO_PY" ]; then
    ok "conda env 'polegeo' already present"
  elif [ -x "$HOME/miniconda3/bin/conda" ]; then
    info "running setup_polegeo_env.sh (pip-downloads torch/opencv/… ~2 GB)…"
    if bash "$SNOW/fasterlio_integration/scripts/setup_polegeo_env.sh"; then
      ok "created conda env 'polegeo'"
    else
      todo "polegeo env setup failed — re-run: bash $SNOW/fasterlio_integration/scripts/setup_polegeo_env.sh"
    fi
  elif have conda; then
    todo "conda is not at \$HOME/miniconda3 (setup_polegeo_env.sh assumes that path). Install Miniconda there, or adapt that script, then re-run."
  else
    todo "install Miniconda (https://docs.conda.io/en/latest/miniconda.html), then re-run — needed for the pipeline/viz env."
  fi
fi

# ---- 3. Super-LIO (ROS 2 Jazzy, built from vendored source) -----------------
if [ "$SKIP_SUPERLIO" -eq 0 ]; then
  hdr "Super-LIO build (ROS 2 Jazzy)"
  if [ -f "$SUPERLIO/install/setup.bash" ]; then
    ok "Super-LIO already built ($SUPERLIO/install)"
  elif [ -f /opt/ros/jazzy/setup.bash ]; then
    if [ "$DO_APT" -eq 1 ]; then
      info "installing build deps via apt (sudo)…"
      sudo apt-get update -qq && sudo apt-get install -y python3-colcon-common-extensions ros-jazzy-pcl-ros libgflags-dev \
        || warn "apt install failed — install those three packages manually"
    fi
    if have colcon; then
      info "building Super-LIO with colcon (first build takes a few minutes)…"
      if ( set +u; source /opt/ros/jazzy/setup.bash; set -u; cd "$SUPERLIO" && colcon build ); then
        ok "built Super-LIO"
      else
        todo "colcon build failed — check the output above (missing dep? see README §3.2)"
      fi
    else
      todo "colcon missing — run: sudo apt install python3-colcon-common-extensions ros-jazzy-pcl-ros libgflags-dev   (or re-run: ./setup.sh --apt)"
    fi
  else
    todo "ROS 2 Jazzy not installed (/opt/ros/jazzy). Install it (https://docs.ros.org/en/jazzy), then re-run to build Super-LIO."
  fi
fi

# ---- 4. Faster-LIO Docker image (opt-in) ------------------------------------
if [ "$DO_FL_IMAGE" -eq 1 ]; then
  hdr "Faster-LIO Docker image (opt-in)"
  if have docker; then
    info "building image 'fasterlio:noetic' (pulls ROS Noetic base)…"
    if bash "$SNOW/fasterlio_integration/docker/run_docker.sh" true; then
      ok "Faster-LIO image ready (fasterlio:noetic)"
    else
      todo "Faster-LIO image build failed — see fasterlio_integration/README.md"
    fi
  else
    todo "install Docker, then: ./setup.sh --fasterlio-image"
  fi
fi

# ---- 5. GLIM Docker image (opt-in) ------------------------------------------
if [ -n "$DO_GLIM_IMAGE" ]; then
  hdr "GLIM Docker image (opt-in, ~10–15 GB)"
  if have docker; then
    info "pulling GLIM $DO_GLIM_IMAGE image…"
    if bash "$SNOW/glim_integration/docker/pull_glim.sh" "$DO_GLIM_IMAGE"; then
      ok "GLIM image pulled ($DO_GLIM_IMAGE)"
      have nvidia-smi || warn "no GPU detected — GLIM GPU path needs one (CPU fallback only validates)."
      todo "one-time (sudo) GPU-in-Docker: sudo bash $SNOW/glim_integration/docker/setup_nvidia_container_toolkit.sh"
    else
      todo "GLIM image pull failed — see glim_integration/README.md"
    fi
  else
    todo "install Docker, then: ./setup.sh --glim-image"
  fi
fi

# ---- 6. dataset presence (never auto-downloaded — manual Kaggle) ------------
hdr "Dataset (download manually — ~47 GB)"
mkdir -p "$DATA_DIR"
FULL="$DATA_DIR/2024-02-28-12-59-51.bag"
CAM="$DATA_DIR/2024-02-28-12-59-51_no_unwanted_topics.bag"
[ -f "$CAM" ]  && ok  "camera bag present ($(du -h "$CAM"  | cut -f1))"  || todo "download the 5.71 GB '…_no_unwanted_topics.bag' (minimum to run the pipeline) → $DATA_DIR/"
[ -f "$FULL" ] && ok  "full bag present ($(du -h "$FULL" | cut -f1))"    || todo "download the 41.24 GB '2024-02-28-12-59-51.bag' (needed to regenerate odometry) → $DATA_DIR/"
info "Kaggle: https://doi.org/10.34740/KAGGLE/DSV/14311103  (folder snow_pole_geo_localization_data)"

# ---- 7. report --------------------------------------------------------------
hdr "Summary"
printf '%sReady:%s\n' "$G" "$Z"
if [ -n "${READY+x}" ]; then for x in "${READY[@]}"; do echo "  ✓ $x"; done; else echo "  (nothing yet)"; fi
if [ -n "${TODO+x}" ]; then
  printf '\n%sStill needed (do these, then re-run ./setup.sh to re-verify):%s\n' "$Y" "$Z"
  for x in "${TODO[@]}"; do echo "  → $x"; done
else
  printf '\n%sAll set.%s\n' "$G" "$Z"
fi
cat <<EOF

Next: run a pipeline from ${SNOW#$ROOT/}/ — e.g.
  (cd "$SNOW" && superlio_integration/run_full_pipeline_superlio.sh)
See README.md §4 for Faster-LIO / GLIM and the per-backend docs for detail.
EOF
