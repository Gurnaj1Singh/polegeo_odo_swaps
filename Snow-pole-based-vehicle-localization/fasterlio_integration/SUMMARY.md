# Faster-LIO ↔ Snow-Pole Localization — Full Experiment Summary

> One-page, self-contained overview of swapping the pipeline's odometry from FastReg to
> Faster-LIO and running the complete snow-pole localization pipeline on it. For full
> tables see `FASTREG_VS_FASTERLIO_COMPARISON.md`, `EXPERIMENT_REPORT.md`,
> `SPEED_AND_RELIABILITY_COMPARISON.md`, `DOWNSAMPLING_RESULTS.md`, `README.md`.

## Goal
Swap the pipeline's LiDAR odometry source from **FastReg** (GPU, learning-based pairwise
registration) to **Faster-LIO** (CPU, LiDAR-**inertial** iESKF + incremental iVox map),
run the *complete* snow-pole localization pipeline on it, and compare speed/accuracy.
Faster-LIO is a **drop-in replacement**: the pipeline only ever consumes a precomputed
`easting,northing` CSV, so swapping the odometry means swapping the CSV producer.

## Dataset
E39 Hemnekjølen snow-pole drive. One ROS1 bag, **541.75 s, ~5,414 scans @ 10 Hz**,
Ouster **OS-2-128** (`/ouster/points` 128×1024, `/ouster/imu` 100 Hz — both on one
sensor clock), plus GNSS (`/gps_left_position`, `/gps_right_position`) and 290
ground-truth pole locations. Host: ROS2 Jazzy, no GPU use.

## Architecture / data flow
```
raw .bag ──[Stage A: Docker Noetic]──> Faster-LIO ──> /Odometry ──> fasterlio_odometry.bag
                                                             │
             10_fasterlio_traj_to_csv.py  ◄─────────────────┘ (+ GNSS & IMU clock from raw bag)
                                                             │
          incremental_navigation_results_fasterlio.csv  (easting/northing)  + fasterlio_traj_tum.txt
                                                             │
   [Stage B] snowpole_based_vehicle_localization.py ──> snowpole_results_fasterlio.csv + live map
                                                             │
   [Stage C] 20_temporal_evolution_visualization.py ──> temporal_evolution_fasterlio.mp4 + summary PNGs
```
One command runs all three: `run_full_pipeline_fasterlio.sh [--from-bag]`.

## Key technical decisions (the non-obvious glue)
- **Docker Noetic for Stage A only.** Faster-LIO is ROS1/catkin; the bags are ROS1. A
  `fasterlio:noetic` image builds upstream `gaoxiang12/faster-lio` and `rosbag play`s the
  dataset natively; the host repo is bind-mounted at `/work`, so config edits apply
  **without rebuilding**. Stages B/C run on the host in the `polegeo` conda env.
- **Config (`config/ouster_os2_128.yaml`):** `lidar_type:3` (Ouster), `scan_line:128`,
  `time_sync_en:false` (LiDAR+IMU share the clock), extrinsic `R=I, t≈(-6,12,-8) mm`
  from `Trip068.json`. **Downsampled for speed** (`point_filter_num:4`,
  `filter_size_surf:1.0`) — halves/8×-reduces points into the IEKF; global drift barely
  changes (poles correct it anyway), local increments stay excellent.
- **Clock bridging:** Faster-LIO stamps are Ouster sensor-clock seconds.
  `10_...py` fits `epoch = a·sensor + b` from the IMU (shares that clock, ~ms residual)
  to map onto the GNSS/Unix timeline.
- **UTM anchoring/alignment:** rigid **2-D Kabsch (rotation+translation, NO scale)**.
  Default `--align start`: anchor pose+heading using only a moving band of the first
  ~400 m of GNSS travel (skipping idle), so **drift is preserved** — the fair,
  odometry-only comparison. `--align full` = best-fit overlay for viz only. Result is
  resampled onto GNSS frame timestamps → same CSV layout the pipeline already reads.
- **"0% GNSS" = the proposed method:** vehicle dead-reckons on odometry and is corrected
  *only* by detected snow poles (no GNSS fixes injected). A GNSS sweep (10/25/50%) tests
  degraded-GNSS robustness.

## How to run
```bash
# full, regenerate odometry from the raw bag (needs Docker + full bag), ~25–30 min
bash fasterlio_integration/run_full_pipeline_fasterlio.sh --from-bag
# fast path: reuse existing CSV, just Stages B+C, seconds
bash fasterlio_integration/run_full_pipeline_fasterlio.sh
```
Inspect ROS1 bags on this ROS2 host with the repo-root `.baginspect_venv` (pure-python
`rosbags`).

## Results (measured on this bag/host)
- **Odometry speed:** Faster-LIO **12.65 ms/scan on CPU (~7.9× realtime)** vs FastReg
  ~320–410 ms/scan on GPU (~0.25–1× realtime) → **~25–32× faster, GPU-free**. *(Stage A
  wall-clock is ~18 min only because we deliberately `rosbag play -r 0.5` for safe
  capture, not an odometry limit.)*
- **Full-pipeline speed:** identical (~2 min, ~66 fps) — the YOLO/pipeline stage
  dominates and is odometry-agnostic.
- **Accuracy (pole-corrected, 0% GNSS):** median **~8.7 m** (comparable to FastReg's
  8.4 m) but **tighter mean (~6.9 m) and worst case (~20 m)**, and it's **GNSS-free**
  (FastReg's shipped trajectory is GNSS-anchored to ~36–50 m, so raw odometry-only error
  — Faster-LIO ~113–118 m — is *not* apples-to-apples; use the pole-corrected metric).
- **With any GNSS, Faster-LIO wins at every level** (10/25/50% → 0.80/0.46/0.19 m vs
  FastReg 2.13/1.29/0.53 m): locally more accurate ⇒ better dead-reckoning between fixes.
  FastReg leads *only* at exactly 0%.
- **Poles:** localizes nearest GT pole to **~2.15 m median** (best of all configs).
- **Bottom line:** Faster-LIO needs LiDAR+IMU but **no GPU**, is far faster per scan,
  GNSS-free, and matches/beats FastReg's deployed accuracy.

## Pole detection accounting (0% GNSS)
*"How many poles were detected, and how many were actually used?"* YOLO fires a
**bounding box on every pole candidate in every processed frame**; each box is then
dropped if **(a)** it has no valid 3-D return in the range image, or **(b)** its
nearest 3-D point is beyond the `distance_threshold = 5 m` false-positive gate
(`snowpole_based_vehicle_localization.py`). Survivors become **used** geo-localization
events — one row in the results CSV. The same physical pole is seen across many
consecutive frames (plus false positives), so raw boxes ≈ 6–7× the used count.

Faster-LIO (0% GNSS): **2325 raw boxes → −5 (no 3-D point) −1965 (>5 m gate) = 355
used** (`= rows of snowpole_results_fasterlio.csv`), re-sighting **135 distinct**
ground-truth poles of **290** at the site; YOLO ran on **2141** in-bounds frames.

**Why one pole becomes several events (and why that is intentional).** The vehicle
drives *past* each pole, so the detector re-acquires the **same physical pole on every
frame it stays in view and within the 5 m range gate** — typically ~2–3 consecutive
frames (mean **2.6 events/pole**, median 3, up to 5; 25 poles seen only once). Each
re-sighting is an *independent* range+bearing fix that re-anchors the drifting
dead-reckoned track, so the pipeline keeps all of them and reports error over
**events**, not unique poles. The 355 events therefore map onto **135 distinct
ground-truth poles** (47 % of the 290 at the site); the rest of the poles were off the
traversed one-way section.

Detection is **almost backend-independent** (same camera/LiDAR images); only the
in-bounds test uses the odometry-predicted position, so a few boundary frames differ.
*(Super-LIO row is the default `filter_rate 2` config.)*

| Backend | YOLO frames | raw boxes | drop (no-pt) | drop (>5 m) | **used events** | distinct poles | events/pole |
|---|---:|---:|---:|---:|---:|---:|---:|
| **Faster-LIO** | 2141 | 2325 | 5 | 1965 | **355** | 135 | 2.6 |
| GLIM | 2147 | 2328 | 5 | 1967 | 356 | 134 | 2.7 |
| Super-LIO | 2145 | 2326 | 5 | 1966 | 355 | 131 | 2.7 |

**Check it yourself:**
```bash
# USED count — persistent (results CSV rows, or the metrics JSON):
echo $(( $(wc -l < snowpole_results_fasterlio.csv) - 1 ))                   # -> 355
grep -E 'pole_detection_events|detection_frames' \
     fasterlio_integration/output/timing_Faster-LIO_pipeline_gnss0.json     # 355 ; 2141
# RAW + drop reasons — from the pipeline STDOUT log:
LOG=fasterlio_integration/output/pipeline_run_gnss0.log
grep -c "sequence number used for geo localization" "$LOG"   # raw boxes  -> 2325 (misnamed: counts ALL boxes)
grep -c "no valid nearest point found"              "$LOG"   # no 3-D pt  -> 5
grep -c "skipping this bounding box"                "$LOG"   # > 5 m gate -> 1965
# DISTINCT physical poles mapped:
~/miniconda3/envs/polegeo/bin/python -c "import pandas as pd; r=pd.read_csv('snowpole_results_fasterlio.csv'); print(r[['Ground Truth Easting','Ground Truth Northing']].round(2).drop_duplicates().shape[0])"
```
(If `pipeline_run_gnss0.log` is absent, regenerate it by redirecting a Stage-B run's
stdout to a file and grep that — the raw/drop breakdown lives only in the stdout.)

## Reliability fixes (the gotchas)
1. **Startup race (was producing an empty bag + silent failure).**
   `scripts/00_run_fasterlio.sh` used fixed `sleep`s; on shared `--net=host :11311`, if
   roscore wasn't up in time, `roslaunch` started a *second* master, the two collided,
   and the mapping node died cleanly at startup → empty odometry → `no /Odometry
   messages`. **Fix:** poll the master before launching, then verify `/laserMapping`
   advertises `/Odometry` before the long play — else abort fast with a clear error.
2. **Guaranteed TUM trajectory.** The node's `Savetrajectory()` writes `./Log/traj.txt`
   relative to cwd and only handles SIGINT, so under roslaunch shutdown it never reliably
   produced a file (left an empty `imu_.txt` stub). **Fix:** `10_...py --tum-out` derives
   `fasterlio_traj_tum.txt` (full 6-DoF, sensor-clock, local frame) straight from the
   authoritative `/Odometry` bag; the copy glob now matches only a real non-empty
   `traj.txt`.

## Outputs & disk behavior
All outputs use **fixed filenames and overwrite in place** — resting footprint stays
~1.2 GB (dominated by `fasterlio_odometry.bag`); no runaway growth, nothing to clean
between runs. Two exceptions: `timing_comparison.csv` **appends one tiny row per run**
(a deliberate FastReg-vs-Faster-LIO ledger), and during recording the old bag coexists
with the new `.active` → **~2.4 GB transient peak** (keep ~2.5 GB free). The
authoritative odometry output is the bag; the TUM/CSV are derived from it.
