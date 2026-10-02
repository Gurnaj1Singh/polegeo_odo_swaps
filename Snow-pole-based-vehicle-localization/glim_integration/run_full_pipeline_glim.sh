#!/usr/bin/env bash
# =============================================================================
# ONE COMMAND: run the COMPLETE snow-pole localization pipeline with GLIM
# odometry and render the final temporal-evolution visualization.
#
#   Stage A  odometry  -> incremental_navigation_results_glim.csv
#   Stage B  pipeline  -> snowpole_results_glim.csv  (+ live map PNG)
#   Stage C  final viz -> output/temporal_evolution_glim.mp4 (+ summary_*.png)
#
# Stage A is SKIPPED automatically if the odometry CSV already exists. Pass
# --from-bag to force a fresh GLIM run from the raw rosbag (needs Docker +
# NVIDIA container toolkit + GPU; run_all.sh converts the bag, runs GLIM, and
# writes the CSV).
#
# Usage:
#   glim_integration/run_full_pipeline_glim.sh             # reuse CSV, then B + C
#   glim_integration/run_full_pipeline_glim.sh --from-bag  # regenerate odometry first
# =============================================================================
set -euo pipefail

INTEG="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"          # glim_integration
REPO="$(dirname "$INTEG")"                                     # Snow-pole project dir
PY="$HOME/miniconda3/envs/polegeo/bin/python"
OUT="$INTEG/output"; mkdir -p "$OUT"

LABEL="glim"
ODOM_NAME="GLIM"
CSV="incremental_navigation_results_${LABEL}.csv"
RESULTS="snowpole_results_${LABEL}.csv"
FULL_BAG="$REPO/snow_pole_geo_localization_data/2024-02-28-12-59-51.bag"
POLES="$REPO/Groundtruth_pole_location_at_test_site_E39_Hemnekjølen.csv"

FROM_BAG=0
[ "${1:-}" = "--from-bag" ] && FROM_BAG=1

[ -x "$PY" ] || { echo "polegeo python missing: $PY  (see fasterlio_integration/scripts/setup_polegeo_env.sh)" >&2; exit 1; }

# ---- Stage A: GLIM odometry -> CSV ------------------------------------------
CLOCK_RESID_MS=""        # captured below for the metrics row (no persistent log kept)
if [ "$FROM_BAG" -eq 1 ] || [ ! -f "$REPO/$CSV" ]; then
  echo "########## [A] GLIM odometry -> $CSV ##########"
  [ -f "$FULL_BAG" ] || { echo "full bag needed for --from-bag: $FULL_BAG" >&2; exit 1; }
  # Capture Stage A to a TEMP file only to scrape the bridge's clock-fit residual,
  # then delete it — nothing extra is left on disk (only run_metrics.* are kept).
  SA_TMP="$(mktemp)"
  bash "$INTEG/run_all.sh" "$FULL_BAG" 2>&1 | tee "$SA_TMP"   # convert->configs->GLIM->bridge->CSV
  CLOCK_RESID_MS="$(grep -oE 'fit residual [0-9.]+ ms' "$SA_TMP" | grep -oE '[0-9.]+' | head -1 || true)"
  rm -f "$SA_TMP"
else
  echo "########## [A] reusing existing $REPO/$CSV  (pass --from-bag to regenerate) ##########"
fi

# ---- Stage B: snow-pole localization pipeline (0% GNSS = proposed method) ----
echo "########## [B] snow-pole pipeline -> $RESULTS ##########"
( cd "$REPO"
  env -u PYTHONPATH MPL_BACKEND=Agg \
    INCREMENTAL_NAV_CSV="$CSV" \
    RESULTS_CSV="$RESULTS" \
    MAP_FIG="$OUT/live_map_${LABEL}.png" \
    "$PY" snowpole_based_vehicle_localization.py )

# ---- Stage C: final temporal-evolution visualization -------------------------
echo "########## [C] temporal-evolution visualization ##########"
"$PY" "$REPO/fasterlio_integration/scripts/20_temporal_evolution_visualization.py" \
  --fasterlio-csv "$REPO/$CSV" \
  --poles "$POLES" \
  --results-csv "$REPO/$RESULTS" \
  --odom-label "$ODOM_NAME" \
  --outdir "$OUT"
[ -f "$OUT/temporal_evolution.mp4" ] && mv -f "$OUT/temporal_evolution.mp4" "$OUT/temporal_evolution_${LABEL}.mp4"
[ -f "$OUT/temporal_evolution.gif" ] && mv -f "$OUT/temporal_evolution.gif" "$OUT/temporal_evolution_${LABEL}.gif"

# ---- Stage D: append this run's METRICS (headline table) one row per run --------
# All fixed-name artifacts above are overwritten each run; this preserves the
# metrics table so every run's numbers survive. THE ONLY FILES THIS KEEPS are
# run_metrics.csv/.md. Sources: persistent GLIM log (realtime factor), the clock
# residual scraped in Stage A (passed in, no log kept), the CSV/traj (rows + poses
# + odom-vs-GNSS drift, re-derived), and the pipeline timing JSON (pole/odom err).
echo "########## [D] saving per-run metrics ##########"
TIMING_JSON="$REPO/fasterlio_integration/output/timing_${ODOM_NAME}_pipeline_gnss0.json"
"$PY" - "$ODOM_NAME" "$REPO/$CSV" "$OUT/glim_traj_gpu.txt" "$TIMING_JSON" \
        "$OUT/glim_run_gpu.log" "${CLOCK_RESID_MS:-}" \
        "$OUT/run_metrics.csv" "$OUT/run_metrics.md" <<'PY'
import csv, json, os, re, statistics, sys, datetime
backend, csv_path, traj_path, timing_json, glim_log, clock_resid_arg, mcsv, mmd = sys.argv[1:9]

def read(p):
    try: return open(p, errors='replace').read()
    except OSError: return ''

def last(pat, text):
    m = re.findall(pat, text)
    return float(m[-1]) if m else None

def count_lines(p):
    try:
        with open(p) as f: return sum(1 for _ in f)
    except OSError: return None

glog = read(glim_log)
rt_factor      = last(r'REALTIME_FACTOR=([\d.]+)x', glog)        # odometry throughput
proc_s         = last(r'processing span ([\d.]+)s', glog)
clock_resid_ms = float(clock_resid_arg) if clock_resid_arg else None   # from Stage A (no log kept)
poses          = count_lines(traj_path)
csv_lines      = count_lines(csv_path)
csv_rows       = (csv_lines - 1) if csv_lines else None          # minus header
drift_med      = None
if os.path.exists(csv_path):                                     # odom-vs-GNSS drift = median CSV gnss_error
    with open(csv_path) as f:
        vals = [float(r['gnss_error']) for r in csv.DictReader(f)
                if r.get('gnss_error') not in (None, '', 'nan')]
    drift_med = round(statistics.median(vals), 2) if vals else None
pole_med = odom_med = None
if os.path.exists(timing_json):
    d = json.load(open(timing_json))
    pole_med, odom_med = d.get('err_pred_median'), d.get('err_odom_median')

row = {'run': datetime.datetime.now().isoformat(timespec='seconds'), 'backend': backend,
       'throughput_rt_factor': rt_factor, 'odom_processing_s': proc_s,
       'trajectory_poses': poses, 'clock_resid_ms': clock_resid_ms,
       'odom_drift_median_m': drift_med, 'csv_rows': csv_rows,
       'pole_corr_median_m': pole_med, 'odom_only_median_m': odom_med}
cols = list(row)
new = not os.path.exists(mcsv)
with open(mcsv, 'a', newline='') as f:
    w = csv.DictWriter(f, fieldnames=cols)
    if new: w.writeheader()
    w.writerow(row)

with open(mcsv) as f: rows = list(csv.DictReader(f))
nice = ('run','backend','RT factor','odom proc (s)','poses','clock resid (ms)',
        'drift med (m)','CSV rows','pole-corr med (m)','odom-only med (m)')
with open(mmd, 'w') as f:
    f.write('# GLIM pipeline — per-run metrics (one row per run)\n\n')
    f.write('| ' + ' | '.join(nice) + ' |\n')
    f.write('|' + '|'.join('---' for _ in nice) + '|\n')
    for r in rows:
        f.write('| ' + ' | '.join(str(r.get(k) or '') for k in cols) + ' |\n')

print('[D] metrics row appended:')
for k in cols: print(f'      {k:22s}: {row[k]}')
print(f'[D] history -> {mcsv}  (+ {mmd})')
PY

echo ""
echo "########## DONE (GLIM) ##########"
echo "  odometry CSV : $REPO/$CSV"
echo "  results CSV  : $REPO/$RESULTS"
echo "  live map     : $OUT/live_map_${LABEL}.png"
echo "  ANIMATION    : $OUT/temporal_evolution_${LABEL}.mp4   <-- open this (the fastreg-style dynamic view)"
echo "  summaries    : $OUT/summary_trajectories.png, summary_error_{hist,cdf}.png, summary_error_vs_distance.png"
echo "  METRICS      : $OUT/run_metrics.md  (+ run_metrics.csv)  <-- one row saved PER RUN (not overwritten)"
