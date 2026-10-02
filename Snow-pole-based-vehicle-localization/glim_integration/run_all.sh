#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# One-shot GLIM pipeline: convert bag -> configs -> run GLIM -> bridge to CSV.
# Assumes provisioning is done (docker/setup_nvidia_container_toolkit.sh once,
# docker/pull_glim.sh). See PLAN.md §8.
#
#   glim_integration/run_all.sh [full_bag] [label]
# ---------------------------------------------------------------------------
set -euo pipefail

INTEG="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(dirname "$INTEG")"
VENV="$(dirname "$REPO")/.baginspect_venv"
S="$INTEG/scripts"

BAG="${1:-$REPO/snow_pole_geo_localization_data/2024-02-28-12-59-51.bag}"
LABEL="${2:-gpu}"
IMAGE="${GLIM_IMAGE:-koide3/glim_ros2:jazzy_cuda13.1}"

docker image inspect "$IMAGE" >/dev/null 2>&1 || {
  echo "GLIM image $IMAGE not found. Run docker/pull_glim.sh first." >&2; exit 1; }

echo "== [1/4] convert bag =="
[ -d "$INTEG/output/ros2_bag" ] || bash "$S/00_convert_bag.sh" "$BAG"

echo "== [2/4] configs =="
bash "$S/01_extract_and_patch_configs.sh"

echo "== [3/4] run GLIM =="
bash "$S/02_run_glim.sh" "$LABEL"

echo "== [4/4] bridge -> CSV =="
"$VENV/bin/python" "$S/10_glim_traj_to_csv.py" \
  --dataset-bag "$BAG" \
  --tum "$INTEG/output/glim_traj_${LABEL}.txt" \
  --out "$REPO/incremental_navigation_results_glim.csv" \
  --align start

echo "== DONE =="
echo "CSV: $REPO/incremental_navigation_results_glim.csv"
echo "Next: run the pipeline with INCREMENTAL_NAV_CSV pointing at it (see README)."
