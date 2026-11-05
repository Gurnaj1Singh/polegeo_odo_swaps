#!/usr/bin/env bash
# Restore the upstream method code used by the benchmark into third_party/, at the exact commits recorded in
# third_party/<name>/VENDORED_FROM.txt (the copies the published results were produced with).
#
#   bash third_party/fetch_third_party.sh          # from odometry_benchmark/
#
# - Method repositories are fetched from their upstream at the pinned commit; git submodules are fetched
#   recursively at the commits the parent repository pins (for Splat-LOAM these equal the submodule commits
#   recorded in Splat-LOAM/VENDORED_FROM.txt; its submodules carry their own glm / eigen submodules).
# - fastreg/ and polegeo_odo_swaps/ (original snow-pole pipeline + Super-LIO) come from THIS repository at commit
#   a8510ad (`git archive`), or from a clone of it when the script runs outside the repository.
# - UGP's checkpoints are not on GitHub (Google Drive, see UGP/VENDORED_FROM.txt): put kitti.pth.tar and
#   nuscenes.pth.tar into third_party/UGP/ckpts/; this script then verifies them against UGP/ckpts/SHA256SUMS.
# Checked before publishing: the copies used for the results are identical to these commits apart from line
# endings (they were checked out on Windows with CRLF).
set -euo pipefail
cd "$(dirname "$0")"

present() {  # a directory that holds more than the provenance files counts as already restored
  [ -d "$1" ] && [ -n "$(ls -A "$1" | grep -v -x -e VENDORED_FROM.txt -e ckpts)" ]
}

fetch() {  # dir url sha
  local dir=$1 url=$2 sha=$3 tmp p u c
  if present "$dir"; then echo "[$dir] already present - skipped"; return; fi
  tmp=$(mktemp -d)
  git init -q "$tmp"
  git -C "$tmp" fetch -q --depth 1 "$url" "$sha"
  git -C "$tmp" checkout -q FETCH_HEAD
  [ "$(git -C "$tmp" rev-parse HEAD)" = "$sha" ] || { echo "[$dir] commit mismatch"; exit 1; }
  # submodules: path, url and pinned commit, read before .git is removed
  local subs=()
  if [ -f "$tmp/.gitmodules" ]; then
    while read -r key p; do
      u=$(git -C "$tmp" config -f .gitmodules --get "${key%.path}.url")
      c=$(git -C "$tmp" ls-tree HEAD "$p" | awk '{print $3}')
      subs+=("$p|$u|$c")
    done < <(git -C "$tmp" config -f .gitmodules --get-regexp '\.path$')
  fi
  mkdir -p "$dir"
  cp -a "$tmp"/. "$dir"/
  rm -rf "$dir/.git" "$tmp"
  echo "[$dir] $url @ ${sha:0:7}"
  for s in ${subs[@]+"${subs[@]}"}; do
    IFS='|' read -r p u c <<< "$s"
    rmdir "$dir/$p" 2>/dev/null || true
    fetch "$dir/$p" "$u" "$c"
  done
}

sha_of() { grep -oE '[0-9a-f]{40}' "$1/VENDORED_FROM.txt" | head -1; }

fetch BUFFER-X      https://github.com/MIT-SPARK/BUFFER-X        "$(sha_of BUFFER-X)"
fetch CLID-SLAM     https://github.com/DUTRobot/CLID-SLAM        "$(sha_of CLID-SLAM)"
fetch DeepPointMap  https://github.com/ZhangXiaze/DeepPointMap   "$(sha_of DeepPointMap)"
fetch Hi-LOAM       https://github.com/Zhangjyhhh/Hi-LOAM        "$(sha_of Hi-LOAM)"
fetch PIN_SLAM      https://github.com/PRBonn/PIN_SLAM           "$(sha_of PIN_SLAM)"
fetch UGP           https://github.com/peakpang/UGP              "$(sha_of UGP)"
fetch Splat-LOAM    https://github.com/rvp-group/Splat-LOAM      "$(sha_of Splat-LOAM)"

# fastreg/ and polegeo_odo_swaps/ from this repository at a8510ad
REPO_SHA=a8510ad29e3015b687cba4a012358cff34e0c89c
TOP=$(git -C .. rev-parse --show-toplevel 2>/dev/null || true)
if [ -n "$TOP" ] && git -C "$TOP" rev-parse -q --verify "$REPO_SHA^{commit}" >/dev/null; then
  SRC=$(mktemp -d)
  git -C "$TOP" archive "$REPO_SHA" fastreg Snow-pole-based-vehicle-localization Super-LIO | tar -x -C "$SRC"
else
  SRC=$(mktemp -d)/repo
  fetch "$SRC" https://github.com/Gurnaj1Singh/polegeo_odo_swaps "$REPO_SHA"
fi
if present fastreg; then echo "[fastreg] already present - skipped"
else cp -a "$SRC/fastreg/." fastreg/ && echo "[fastreg] polegeo_odo_swaps @ ${REPO_SHA:0:7}"; fi
if present polegeo_odo_swaps; then echo "[polegeo_odo_swaps] already present - skipped"
else cp -a "$SRC/Snow-pole-based-vehicle-localization" "$SRC/Super-LIO" polegeo_odo_swaps/ \
       && echo "[polegeo_odo_swaps] polegeo_odo_swaps @ ${REPO_SHA:0:7}"; fi
rm -rf "$SRC"

# UGP checkpoints (manual download, then verified)
if ls UGP/ckpts/*.pth.tar >/dev/null 2>&1; then
  (cd UGP && sha256sum -c ckpts/SHA256SUMS)
else
  echo "[UGP] put kitti.pth.tar and nuscenes.pth.tar from the Google Drive folder in UGP/VENDORED_FROM.txt"
  echo "      into third_party/UGP/ckpts/ and re-run to verify them."
fi
