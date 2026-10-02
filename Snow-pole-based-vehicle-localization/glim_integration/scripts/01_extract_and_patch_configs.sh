#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# Stage 01: materialise GLIM's default configs from the image, then patch our
# OS-2-128 overrides (topics, extrinsic, GPU odometry, headless). See PLAN.md §4.
#
# Extraction runs the container WITHOUT mounting over /glim/config (otherwise we
# would hide the very defaults we want to copy), copies them to our config dir,
# and chowns them back to the host user. Patching runs on the host with plain
# python3 (patch_configs.py is stdlib-only).
#
#   scripts/01_extract_and_patch_configs.sh            # extract if missing, then patch
#   scripts/01_extract_and_patch_configs.sh --force    # re-extract fresh defaults first
# ---------------------------------------------------------------------------
set -euo pipefail

SCRIPTS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INTEG="$(dirname "$SCRIPTS")"                 # glim_integration
REPO="$(dirname "$INTEG")"                    # Snow-pole project dir
CFG="$INTEG/config"
IMAGE="${GLIM_IMAGE:-koide3/glim_ros2:jazzy_cuda13.1}"

FORCE=0; [ "${1:-}" = "--force" ] && FORCE=1

if [ "$FORCE" -eq 1 ] || [ ! -f "$CFG/config.json" ]; then
  echo "[01] extracting default configs from $IMAGE -> $CFG"
  docker run --rm \
    -e HOST_UID="$(id -u)" -e HOST_GID="$(id -g)" \
    -v "$REPO:/work:rw" \
    "$IMAGE" bash -lc '
      set -e
      SRC=""
      for d in /root/ros2_ws/src/glim/config \
               /root/ros2_ws/install/glim/share/glim/config \
               /glim/config; do
        [ -d "$d" ] && SRC="$d" && break
      done
      [ -n "$SRC" ] || { echo "no GLIM config dir found in image" >&2; exit 1; }
      echo "[01] source config dir in image: $SRC"
      cp -a "$SRC"/. /work/glim_integration/config/
      chown -R "$HOST_UID:$HOST_GID" /work/glim_integration/config
    '
else
  echo "[01] configs already present in $CFG (use --force to re-extract)"
fi

echo "[01] patching overrides"
python3 "$SCRIPTS/patch_configs.py" --config-dir "$CFG" --overrides "$CFG/overrides.json"

echo "[01] DONE. Key files:"
ls -1 "$CFG"/config*.json 2>/dev/null || true
