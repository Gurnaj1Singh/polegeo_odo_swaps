#!/usr/bin/env bash
# One-shot: build the Faster-LIO Noetic image (if needed) and run stage 0
# (Faster-LIO odometry on the full bag) hands-off, logging to output/.
#
# Usage:
#   fasterlio_integration/run_all.sh [full_bag_path] [play_rate]
#
# If this user is not in the 'docker' group, run_docker.sh auto-uses sudo, so
# invoke this through the session so the password prompt is visible, e.g.:
#   ! bash Snow-pole-based-vehicle-localization/fasterlio_integration/run_all.sh
set -euo pipefail

INTEG="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"        # .../fasterlio_integration
PROJ="$(dirname "$INTEG")"                                    # Snow-pole-based-...

BAG="${1:-$PROJ/snow_pole_geo_localization_data/2024-02-28-12-59-51.bag}"
RATE="${2:-0.5}"                                              # 0.5x: safer on 8 vCPU
LOG="$INTEG/output/fasterlio_run.log"
mkdir -p "$INTEG/output"

if [ ! -f "$BAG" ]; then echo "bag not found: $BAG" >&2; exit 1; fi

# run_docker.sh bind-mounts the Snow-pole repo dir ($PROJ) at /work, so translate
# host paths under $PROJ to their in-container /work equivalents.
BAG_IN="/work${BAG#$PROJ}"
STAGE0_IN="/work${INTEG#$PROJ}/scripts/00_run_fasterlio.sh"

echo "[run_all] bag=$BAG rate=$RATE" | tee "$LOG"
echo "[run_all] logging to $LOG"

# build (if needed) + run stage 0 inside the container; tee everything to LOG
"$INTEG/docker/run_docker.sh" bash -lc "chmod +x '$STAGE0_IN'; '$STAGE0_IN' '$BAG_IN' '$RATE'" 2>&1 | tee -a "$LOG"

echo "[run_all] DONE -> $INTEG/output/fasterlio_odometry.bag" | tee -a "$LOG"
