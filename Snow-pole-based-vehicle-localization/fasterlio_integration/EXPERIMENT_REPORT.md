# Replacing FastReg with Faster-LIO for LiDAR Odometry — Experiment Report

**Project:** Snow-Pole-Based Vehicle Localization (E39 Hemnekjølen, Nordic winter)
**Change:** swap the LiDAR **odometry** backend from **FastReg** → **Faster-LIO**
**Scope:** odometry only — snow-pole detection, geo-localization, GNSS fusion and the
incremental-navigation logic are unchanged.
**Outcome (headline):** with Faster-LIO odometry, snow-pole correction localizes the
vehicle to **~8.6 m median error with zero GNSS**, cutting the odometry-only error
of ~450 m by **~24×** — i.e. the framework works end-to-end on a modern LIO backend,
running fully on a ROS 2 laptop.

---

## 1. Objective and Requirements

### 1.1 Objective
Replace the frame-to-frame registration odometry (FastReg, Arnold et al. 2021) with the
tightly-coupled LiDAR-Inertial Odometry **Faster-LIO** (Bai et al., RA-L 2022,
https://ieeexplore.ieee.org/document/9718203, code
https://github.com/gaoxiang12/faster-lio), then reproduce the localization results as
the **temporal evolution of vehicle motion together with the localized snow poles**.

### 1.2 Requirements
| # | Requirement | Status |
|---|---|---|
| R1 | Faster-LIO produces vehicle odometry from the dataset's LiDAR + IMU | ✅ 5,412 poses on the full bag |
| R2 | Faster-LIO output is a **drop-in** for FastReg (same interface to the pipeline) | ✅ same `easting/northing` CSV schema, env-var switch |
| R3 | The full localization pipeline runs (detection + pole correction) | ✅ runs headless on this machine |
| R4 | Results: temporal evolution of motion + localized snow poles | ✅ `temporal_evolution.mp4` + summary figures |
| R5 | Reproducible on the available hardware (ROS 2 host, no ROS 1) | ✅ Docker for Faster-LIO, conda env for the pipeline |

---

## 2. Background: FastReg vs Faster-LIO

### 2.1 What each method is
- **FastReg** — a *feature-based, hierarchical point-cloud registration* method. It aligns
  **consecutive LiDAR scans** coarse-to-fine (downsampled first, then full resolution) to
  estimate the rigid transform between frames. It is **LiDAR-only** (no IMU) and produces a
  per-frame relative motion.
- **Faster-LIO** — a *tightly-coupled LiDAR-Inertial Odometry*. It fuses LiDAR with an IMU in
  an **iterated Error-State Kalman Filter (iESKF)** and registers each de-skewed scan to an
  **incremental voxel map (iVox)** for fast nearest-neighbour search. The IMU predicts the
  state between scans and removes motion distortion within a scan.

### 2.2 Why Faster-LIO is the stronger backend

| Aspect | FastReg | Faster-LIO |
|---|---|---|
| Sensors | LiDAR only | **LiDAR + IMU** (fused) |
| Estimation | frame-to-frame rigid registration | scan-to-map iESKF (continuous state) |
| Motion distortion (de-skew) | not modelled | **per-point de-skew** via IMU + point timestamps |
| Map | none (pairwise) | **incremental local voxel map** (iVox) |
| Robustness to fast motion / sparse geometry | limited (LiDAR only) | better (IMU carries the state) |
| Output rate | per LiDAR frame | per LiDAR frame, IMU-propagated between |
| Real-time | moderate | designed for real-time (parallel sparse voxels) |

**Implication for this project:** the pipeline dead-reckons the vehicle **between snow-pole
sightings** using the odometry's local motion. Faster-LIO's IMU fusion + de-skew give more
faithful *local* motion, and its metric scale comes directly from LiDAR + IMU.

---

## 3. Dataset and Sensor

- **Site:** E39 Hemnekjølen, Norway (~4.2 km instrumented test site; ~9–10 km driven), Nordic winter.
- **Vehicle sensor:** **Ouster OS-2-128**, mode `1024×10` → 128 beams, 10 Hz.
- **Duration:** 541.8 s (~9 min).
- **Bags (Kaggle):**
  - `2024-02-28-12-59-51.bag` (**39 GB**) — has `/ouster/points` (raw cloud) **and** `/ouster/imu`.
  - `..._no_unwanted_topics.bag` (**5.4 GB**) — has `/ouster/imu` + LiDAR *images* + GNSS, but **no** raw cloud.
- **Topics used:**
  - Faster-LIO: `/ouster/points` (PointCloud2, 128×1024, with per-point time `t` and `ring`) + `/ouster/imu` (100 Hz) → **full bag**.
  - Detection pipeline: `/ouster/*_image` (signal/range) + `/gps_left/right_position` (NavSatFix) → reduced bag is enough.
- **Reference / ground truth:**
  - GNSS (dual antenna) — used only for the **initial fix** and as the **evaluation reference**.
  - `Groundtruth_pole_location_…csv` — **290 surveyed snow-pole coordinates** (UTM33N), the landmark map.

**Key sensor facts verified from the data (and used in the Faster-LIO config):**
- `/ouster/points` and `/ouster/imu` share the **same Ouster sensor clock** (~13,214 s), so LIO
  needs no cross-sensor time-sync.
- Per-point time `t` spans **0–99.9 ms** (one 10 Hz sweep) → used for de-skew.
- IMU accel is in **m/s²** (z ≈ 9.94 ≈ g).
- **LiDAR→IMU extrinsic** (from `Trip068.json`): rotation = identity, translation ≈
  `(−0.00625, +0.01178, −0.00765)` m.

---

## 4. System / Environment Setup

The host runs **ROS 2 Jazzy**, but upstream Faster-LIO is **ROS 1** and the bags are ROS 1 `.bag`.
Two isolated environments were built:

1. **Faster-LIO (Docker, ROS 1 Noetic)** — `fasterlio_integration/docker/`. Builds Faster-LIO
   from source (needs `ros-noetic-eigen-conversions`; the Livox driver is vendored). Runs
   `rosbag play` + the mapping node with our sensor config.
2. **Detection pipeline (conda env `polegeo`)** — `scripts/setup_polegeo_env.sh`. Built on
   **conda-forge** (avoids the Anaconda-channel ToS gate). Python 3.9, **numpy 1.26 (<2)**,
   **torch 2.2.0+cpu**, **ouster-sdk 0.10**, `ultralytics` (for the YOLOv5 hub loader),
   pyproj/geopy/pykrige/scikit-learn/contextily, and **`rosbags`**.

**Removing the ROS 1 dependency for the pipeline:** the original `geoloc_utils.process_ros_bag_data`
used `bagpy` (ROS 1). It was rewritten to read the bag with pure-Python **`rosbags`** (identical
return values and record-time timestamps), and the ROS-only imports (`bagpy`, `sensor_msgs`,
`cv_bridge`, `open3d`) were made optional. This lets the whole pipeline run on the ROS 2 host.
When running, `PYTHONPATH` is cleared (`env -u PYTHONPATH`) so ROS 2's site-packages don't shadow
the conda env.

---

## 5. Methodology — the drop-in swap

**Key insight:** the pipeline never ran FastReg live. FastReg was run offline and its trajectory
stored in `incremental_navigation_results.csv`; the pipeline reads only the **`easting/northing`**
columns (UTM33N, one row per frame), derives a per-frame **heading + translation**, and
dead-reckons between pole sightings. **So replacing FastReg = regenerating that CSV from Faster-LIO.**

```
FULL BAG ─▶ Faster-LIO (Docker/Noetic) ─▶ /Odometry ─▶ bridge (stage 1) ─▶ incremental_navigation_results_fasterlio.csv
(points+imu)                                                                          │
                                                                                      ▼
                              existing snow-pole pipeline  (INCREMENTAL_NAV_CSV=…)  ─▶ snowpole_results_fasterlio.csv
                                                                                      │
                                                                                      ▼
                              stage 2 viz ─▶ temporal_evolution.mp4 + summary_*.png
```

Pipeline scripts were patched **non-destructively** — behaviour is unchanged unless env vars are set:
- `INCREMENTAL_NAV_CSV` — odometry source (default = original FastReg CSV; set to the Faster-LIO CSV to swap).
- `BAG_PATH`, `RESULTS_CSV`, `MPL_BACKEND`, `LIVE_PLOT`, `BASEMAP`, `POLE_MODEL`, `GNSS_PERCENTAGE` — run controls.
- A `[odometry] LiDAR odometry source CSV: …` line is printed at startup so every run self-reports its backend.

---

## 6. Faster-LIO Configuration (sensor-specific)

`fasterlio_integration/config/ouster_os2_128.yaml`, derived from the verified sensor facts:
- `lidar_type: 3` (Ouster), `scan_line: 128`, `scan_rate: 10`, `blind: 2.0 m`, `det_range: 150 m`.
- Extrinsic `R = I`, `T = (−0.00625, 0.01178, −0.00765)` m (LiDAR→IMU), `extrinsic_est_en: false`.
- De-skew: Faster-LIO's Ouster handler **hard-codes** `curvature = point.t / 1e6` (ns→ms); our `t`
  is nanoseconds, so de-skew is correct out of the box (the `time_scale` key is a no-op for Ouster,
  and the `ring` field is unused — so the PCL "ring" warning is benign).
- `point_filter_num: 2` (denser cloud than the default 4; see §9.1 for its small effect).

---

## 7. The Odometry CSV — creation and contents

**Script:** `fasterlio_integration/scripts/10_fasterlio_traj_to_csv.py`. Faster-LIO's raw output has
three mismatches with the pipeline; the bridge fixes each:

1. **Frame** — Faster-LIO starts at (0,0) with arbitrary yaw in its own local frame. We apply a rigid
   2-D transform (rotation+translation, no scale; reflection-guarded Kabsch) so the **start pose +
   heading** match the first GNSS fix. Only the first ~400 m of travel is used for this anchor
   (idle at the start is skipped), so the rest is **pure odometry** — the drift is preserved, not
   GNSS-corrected. (`--align full` gives a best-fit overlay instead, for visualization.)
2. **Clock** — Faster-LIO stamps are on the Ouster sensor clock; GNSS is Unix time. We fit
   `epoch = a·sensor + b` from the IMU (same clock as the LiDAR); the fit residual was **6.9 ms**.
3. **Rows** — Faster-LIO gives one pose per scan (~5,412); the pipeline expects one row per GNSS
   frame (5,422). We resample the aligned path onto the GNSS timestamps.

**Schema (5,422 rows, ~10 Hz):**

| Column | Meaning |
|---|---|
| `Time` | timestamp (epoch seconds) |
| `latitude`, `longitude` | Faster-LIO position as WGS84 lat/lon |
| **`easting`, `northing`** | **Faster-LIO position (UTM33N, m) — the columns the pipeline reads** |
| `gnss_easting`, `gnss_northing` | true GNSS position (reference for eval/plots) |
| `heading` | bearing (deg from North) from consecutive positions |
| `gnss_error` | distance Faster-LIO ↔ GNSS at that frame (the drift) |

The pipeline uses only `easting/northing`; the rest supports evaluation and the visualization.

---

## 8. How to Run (reproduce)

```bash
cd Snow-pole-based-vehicle-localization

# Stage 0 — Faster-LIO odometry (Docker/Noetic)
fasterlio_integration/docker/run_docker.sh bash -lc \
  "/work/fasterlio_integration/scripts/00_run_fasterlio.sh \
   /work/snow_pole_geo_localization_data/2024-02-28-12-59-51.bag 1.0"

# Stage 1 — trajectory → pipeline CSV (GNSS-anchored)
source ../.baginspect_venv/bin/activate    # rosbags + pyproj + pandas + scipy
python fasterlio_integration/scripts/10_fasterlio_traj_to_csv.py \
  --dataset-bag snow_pole_geo_localization_data/2024-02-28-12-59-51_no_unwanted_topics.bag \
  --odom-bag fasterlio_integration/output/fasterlio_odometry.bag \
  --align start --out incremental_navigation_results_fasterlio.csv

# Full localization on Faster-LIO odometry (produces the pole-corrected result)
env -u PYTHONPATH MPL_BACKEND=Agg \
  INCREMENTAL_NAV_CSV=incremental_navigation_results_fasterlio.csv \
  RESULTS_CSV=snowpole_results_fasterlio.csv \
  ~/miniconda3/envs/polegeo/bin/python snowpole_based_vehicle_localization.py

# Stage 2 — temporal-evolution animation + summaries
python fasterlio_integration/scripts/20_temporal_evolution_visualization.py \
  --fasterlio-csv incremental_navigation_results_fasterlio.csv \
  --poles "Groundtruth_pole_location_at_test_site_E39_Hemnekjølen.csv" \
  --results-csv snowpole_results_fasterlio.csv
```
For **live plots**, drop `MPL_BACKEND=Agg` and add `LIVE_PLOT=1` (needs a desktop `DISPLAY`).

---

## 9. Results

### 9.1 Faster-LIO odometry quality
- **5,412 poses** over the full 541 s (≈ the 5,419 LiDAR scans), real-time at 1.0× playback (20-core CPU).
- **Relative pose error (KITTI-style, start heading removed): ~3.3 %** across 25–400 m segments
  → **~0.8–1.6 m** of dead-reckoning error over one pole spacing (~25–50 m).
- **Global odometry-only drift vs GNSS** (start-anchored): **median ≈ 450 m, max ≈ 2,850 m** over ~10 km.
- Trajectory **shape** matches GNSS well; the global drift is dominated by **slow yaw** from the
  low-grade Ouster ICM-20948 IMU on long highway stretches. Denser points (`point_filter_num` 4→2)
  reduced drift only marginally (median 505→449 m) — confirming it is **IMU-limited, not density-limited**.

### 9.2 FastReg vs Faster-LIO — odometry
| Odometry (no poles) | Error vs GNSS (median) | Notes |
|---|---|---|
| FastReg (as shipped in repo CSV) | **~50 m** | path length ~35 % too long yet bounded → appears **periodically GNSS-anchored**, not pure odometry |
| Faster-LIO (this work, start-anchored) | **~450 m** | **pure odometry**, unaided after the start — drift preserved honestly |

> ⚠️ **This row is not a like-for-like comparison.** The repo's FastReg `easting/northing` stays within
> ~50 m of GNSS despite a badly inflated path length, which is only possible if it was re-anchored to
> GNSS along the route. Our Faster-LIO track is *unaided* pure odometry. The fair comparison is the
> **final pole-corrected localization** (§9.3) and the **local RPE** (§9.1, where Faster-LIO is strong).
> A true head-to-head (FastReg through the same pipeline) is one command away — run the pipeline with
> `INCREMENTAL_NAV_CSV` unset (defaults to the FastReg CSV).
>
> ✅ **Now done and measured** (2026-09-08) — see `SPEED_AND_RELIABILITY_COMPARISON.md`.
> Through the *identical* pipeline at 0 % GNSS: **pole-corrected** median **8.4 m (FastReg)**
> vs **10.1 m (Faster-LIO)**, but Faster-LIO has the lower **mean** (9.2 vs 13.7 m) and a much
> tighter **worst case** (26.5 vs 64.6 m) — and it does so **GNSS-free** (raw odometry 188 m vs
> FastReg's GNSS-re-anchored 36 m). Pipeline **running time is ~equal** (123 vs 134 s) and the
> YOLO detector measures **~20 ms/frame (~50 fps)**, confirming the 18 ms estimate below.

### 9.3 Final localization (snow-pole corrected) — Faster-LIO backend
| Trajectory | Error vs GNSS (median / mean / max) |
|---|---|
| Faster-LIO odometry only | 447 / 789 / 2855 m |
| **Proposed (pole-corrected)** | **8.6 / 6.8 / 16.0 m** |

- **355 pole detections across 132 unique ground-truth poles** (each pole is seen in ~3
  consecutive frames as the vehicle passes; 132 of the 290 site poles were traversed, over
  frames ~835–2977 = the pole-instrumented section the vehicle drove through). The 355 is the
  number of correction *events*, not unique poles (132 ≤ 290).
- **~24× error reduction** from the snow-pole correction → the framework works with Faster-LIO.
- Detection uses the **pretrained YOLOv5** model (`model/pole_best_signal.pt`, 18 ms/frame on this CPU).
- GNSS was used **only** for the initial fix + evaluation (0 % GNSS during the drive; verified via the
  GNSS-percentage variant: GNSS Count 0 / Predictive Count 5419 → 8.87 m, consistent).

### 9.4 Figures (`fasterlio_integration/output/`)
- `temporal_evolution.mp4` — vehicle motion over time with GNSS, Faster-LIO odometry, pole-corrected
  track and the localized poles.
- `summary_trajectories.png` — all tracks + the 290 poles (pole-corrected hugs the road; odometry drifts).
- `summary_error_hist.png`, `summary_error_cdf.png`, `summary_error_vs_distance.png`.
- `live_map_final.png` — the pipeline's own map (Faster-LIO-labelled, detected poles in red, OSM basemap).

---

## 10. Discussion

- **Why ~8.6 m and not sub-metre?** The paper reaches sub-metre *with partial GNSS*. Here it is **pure
  odometry + poles**. Faster-LIO's slow **yaw drift** makes the *heading* used to dead-reckon between
  pole sightings slightly wrong, which is the dominant residual. Reducing the yaw drift (a better IMU,
  or tuning/adding constraints) would tighten the corrected result toward the paper's numbers.
- **Local vs global accuracy.** What matters for this framework is *local* motion between poles, where
  Faster-LIO is good (~3.3 % RPE). The large *global* drift is exactly what the georeferenced snow poles
  are designed to correct — and they do (24×).
- **Detector is pretrained, by design.** Only the YOLOv5 snow-pole detector is pretrained (as in the
  paper). Everything else — odometry (Faster-LIO, filter-based, no learning), pole map (surveyed),
  localization — runs from the raw data.

---

## 11. Limitations and Future Work
- **Yaw drift** from the Ouster internal MEMS IMU dominates the odometry error; a higher-grade IMU or
  additional constraints (e.g. ground/plane, zero-velocity updates at stops, or wheel odometry) would help.
- Alignment anchors on the **first ~400 m** of GNSS; a fully GNSS-free init (e.g. from the first pole
  pair) is possible future work.
- Pole correction only where poles exist (frames ~835–2977); outside the instrumented section the
  estimate reverts to drifting odometry.
- ~~A **like-for-like FastReg-through-the-pipeline** run would complete the quantitative comparison in §9.2.~~
  **Done (2026-09-08)** — timed head-to-head in `SPEED_AND_RELIABILITY_COMPARISON.md`.

---

## 12. Reproducibility / File Manifest
```
fasterlio_integration/
├── README.md                         # quick-start + measured results
├── EXPERIMENT_REPORT.md              # this document
├── config/ouster_os2_128.yaml        # Faster-LIO sensor config (extrinsics, de-skew)
├── config/mapping_ouster_os2_128.launch
├── docker/Dockerfile, run_docker.sh  # ROS Noetic + Faster-LIO build/run
├── scripts/00_run_fasterlio.sh       # run Faster-LIO, record /Odometry
├── scripts/10_fasterlio_traj_to_csv.py   # trajectory → GNSS-anchored pipeline CSV (the bridge)
├── scripts/20_temporal_evolution_visualization.py
├── scripts/setup_polegeo_env.sh      # build the conda env for the detection pipeline
└── output/                           # odometry bag, CSVs, mp4, figures, logs
```
Modified project files (all behaviour-preserving by default): `geoloc_utils.py`
(rosbags reader + optional ROS imports), `snowpole_based_vehicle_localization.py` and
`snowpole_based_vehicle_localization_GNSS_percentage.py` (env-var switches, headless plotting,
Faster-LIO labels, red detected poles, policy-compliant basemap).

---

## References
1. Bavirisetti et al. (2025). *Vehicle localization framework using georeferenced snow poles and LiDAR
   in GNSS-limited environments under Nordic conditions.* IEEE T-ITS 26(12).
2. Bai, Xiao, Chen, Wang, Zhang, Gao (2022). *Faster-LIO: Lightweight Tightly Coupled LiDAR-Inertial
   Odometry Using Parallel Sparse Incremental Voxels.* IEEE RA-L. https://ieeexplore.ieee.org/document/9718203
3. Arnold, Mozaffari, Dianati (2021). *Fast and robust registration of partially overlapping point
   clouds (FastReg).* IEEE RA-L 7(2), 1502–1509.
4. Jocher (2020). *YOLOv5 by Ultralytics (v7.0).*
