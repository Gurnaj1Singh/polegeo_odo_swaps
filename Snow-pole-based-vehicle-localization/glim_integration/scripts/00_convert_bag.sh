#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# Stage 00: convert the ROS1 dataset .bag -> a ROS2 bag GLIM can read, keeping
# ONLY /ouster/points + /ouster/imu (PointCloud2/Imu are standard messages, so
# no custom .msg definitions are needed). Uses the repo-root .baginspect_venv
# (already has `rosbags`). See PLAN.md §2, §7.
#
#   scripts/00_convert_bag.sh [full_bag] [--force]
#
# Env:
#   GLIM_BAG_STORAGE   sqlite3 (default) | mcap
#   GLIM_BAG_COMPRESS  none (default) | lz4 | zstd
#       NOTE: compression saves disk but makes GLIM decompress on read, which
#       muddies the throughput benchmark. Keep 'none' for the speed test; use
#       'lz4' (cheap decode) only if disk-constrained. ~33 GB uncompressed.
# ---------------------------------------------------------------------------
set -euo pipefail

SCRIPTS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INTEG="$(dirname "$SCRIPTS")"                 # glim_integration
REPO="$(dirname "$INTEG")"                    # Snow-pole project dir
VENV="$(dirname "$REPO")/.baginspect_venv"    # repo-root venv (has rosbags)

# args: an optional bag path plus an optional --force, in any order.
FORCE=0
BAG="$REPO/snow_pole_geo_localization_data/2024-02-28-12-59-51.bag"
for a in "$@"; do
  if [ "$a" = "--force" ]; then FORCE=1; else BAG="$a"; fi
done
DST="$INTEG/output/ros2_bag"
STORAGE="${GLIM_BAG_STORAGE:-sqlite3}"
COMPRESS="${GLIM_BAG_COMPRESS:-none}"

[ -f "$BAG" ] || { echo "bag not found: $BAG" >&2; exit 1; }
[ -x "$VENV/bin/rosbags-convert" ] || { echo "rosbags-convert not in $VENV" >&2; exit 1; }

if [ -e "$DST" ]; then
  if [ "$FORCE" -eq 1 ]; then echo "[00] removing existing $DST"; rm -rf "$DST";
  else echo "[00] $DST exists — pass --force to overwrite" >&2; exit 1; fi
fi

echo "[00] disk before:"; df -h "$INTEG/output" | awk 'NR==1||NR==2'
echo "[00] converting $BAG"
echo "     -> $DST  (storage=$STORAGE compress=$COMPRESS, topics: /ouster/points /ouster/imu)"

CARGS=(--src "$BAG" --dst "$DST" --dst-storage "$STORAGE"
       --include-topic /ouster/points /ouster/imu)
[ "$COMPRESS" != "none" ] && CARGS+=(--compress "$COMPRESS" --compress-mode message)

t0=$(date +%s)
"$VENV/bin/rosbags-convert" "${CARGS[@]}"
t1=$(date +%s)

echo "[00] converted in $((t1-t0))s. Output:"
du -sh "$DST"
echo "[00] disk after:"; df -h "$INTEG/output" | awk 'NR==2'
echo "[00] DONE -> $DST   (delete after the GLIM run to reclaim ~33 GB)"
