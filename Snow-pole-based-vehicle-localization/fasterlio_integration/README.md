# Replacing FastReg with Faster-LIO for LiDAR Odometry

This folder swaps the project's LiDAR **odometry** backend from **FastReg**
(frame-to-frame point-cloud registration) to **Faster-LIO**
(tightly-coupled LiDAR-Inertial Odometry using an iterated ESKF + incremental
voxels), as proposed in:

> C. Bai, T. Xiao, Y. Chen, H. Wang, F. Zhang, X. Gao,
> *"Faster-LIO: Lightweight Tightly Coupled LiDAR-Inertial Odometry Using
> Parallel Sparse Incremental Voxels,"* IEEE RA-L 2022.
> Code: https://github.com/gaoxiang12/faster-lio · Paper: https://ieeexplore.ieee.org/document/9718203

**Only the odometry source changes.** Snow-pole detection, geo-localization,
GNSS fusion, and the incremental-navigation logic are untouched.

---

## 1. Why this is a clean swap (how FastReg is actually used)

The localization pipeline does **not** run FastReg live. FastReg was run offline
and its trajectory was stored in `incremental_navigation_results.csv`. The
pipeline reads only the **`easting` / `northing`** columns (one row per frame,
UTM33N / EPSG:32633), derives a per-frame **heading + translation** from
consecutive rows, and dead-reckons the vehicle between snow-pole sightings; the
same track is the odometry-only error baseline.

So *"replace FastReg with Faster-LIO"* reduces to: **regenerate that CSV from a
Faster-LIO run.** That is exactly what this folder does.

```
FULL BAG ──▶ Faster-LIO (Docker/Noetic) ──▶ /Odometry ──▶ traj→CSV bridge ──▶ incremental_navigation_results_fasterlio.csv
(points+imu)   stage 0                                      stage 1 (GNSS-anchored)          │
                                                                                              ▼
                                                        existing snow-pole pipeline (INCREMENTAL_NAV_CSV=…)
                                                                                              │
                                                                                              ▼
                                                        stage 2: temporal-evolution viz + summary figures
```

## 2. Dataset facts (verified from the bags + `Trip068.json`)

| Property | Value |
|---|---|
| Sensor | Ouster **OS-2-128**, `1024x10` → 128 beams, 10 Hz |
| `/ouster/points` | `PointCloud2` 128×1024, official fields `x,y,z,intensity,t,reflectivity,ring,ambient,range`; per-point `t` = 0–99.9 ms/scan. **Full bag only** |
| `/ouster/imu` | `Imu` @100 Hz, accel in m/s² (z≈9.94). **In both bags** |
| Clock | `/ouster/points` and `/ouster/imu` share the **Ouster sensor clock** (≈13214 s). GNSS is Unix epoch → handled by stage 1 |
| Extrinsic LiDAR(`os_sensor`)→IMU(`os_imu`) | R = I, t = (−0.00625, +0.01178, −0.00765) m (from `Trip068.json`) |

➡️ Faster-LIO needs `/ouster/points` **and** `/ouster/imu` → **use the full bag**
`2024-02-28-12-59-51.bag`. The reduced bag lacks `/ouster/points`
(a range-image→PointCloud2 rebuild is possible but unnecessary since you have
the full bag — see §7).

## 3. Prerequisites

- **Docker** (upstream Faster-LIO is ROS 1; this machine runs ROS 2 Jazzy, and
  the bags are ROS 1 `.bag`). The image handles everything.
- A Python env with `numpy pandas scipy pyproj rosbags` for stage 1, and
  `numpy pandas matplotlib` for stage 2. A ready venv already exists at repo
  root: `.baginspect_venv` (has `rosbags`); add the rest:
  ```bash
  source ../../.baginspect_venv/bin/activate      # from this folder
  pip install pyproj pandas scipy matplotlib
  ```

## 4. Run it

Paths below assume you are in `Snow-pole-based-vehicle-localization/`.

**Stage 0 — Faster-LIO odometry (in Docker).** The Snow-pole repo dir is mounted
at `/work`. Easiest is the one-shot orchestrator (build image + run, log to
`output/fasterlio_run.log`):
```bash
fasterlio_integration/run_all.sh          # optional args: <bag> <play_rate>
```
Or drive it manually:
```bash
fasterlio_integration/docker/run_docker.sh            # shell in the container
# inside the container (/work == Snow-pole repo dir):
/work/fasterlio_integration/scripts/00_run_fasterlio.sh \
    /work/snow_pole_geo_localization_data/2024-02-28-12-59-51.bag
# → fasterlio_integration/output/fasterlio_odometry.bag
```
**Docker permissions:** if `docker` needs sudo (user not in the `docker` group),
the scripts auto-fall back to `sudo docker`; run via the session so the password
prompt shows, e.g. `! bash Snow-pole-based-vehicle-localization/fasterlio_integration/run_all.sh`.
One-time fix to avoid sudo: `sudo usermod -aG docker $USER` then re-login.
*Sanity check the de-skew scaling:* the launch prints per-scan info; the scan
time span must be ≈0–100 ms. If it is ~0–1e5, flip `time_scale` in
`config/ouster_os2_128.yaml` (ns↔µs). See the comment there.

**Stage 1 — Faster-LIO trajectory → pipeline CSV (GNSS-anchored):**
```bash
python fasterlio_integration/scripts/10_fasterlio_traj_to_csv.py \
    --dataset-bag snow_pole_geo_localization_data/2024-02-28-12-59-51.bag \
    --odom-bag    fasterlio_integration/output/fasterlio_odometry.bag \
    --align start \
    --out incremental_navigation_results_fasterlio.csv
```
`--align start` (default) anchors only the **initial pose + heading** to GNSS
(first 30 m) and preserves drift → the fair, odometry-only comparison. Use
`--align full` for a best-fit overlay (visualization / upper bound), not for the
drift metric.

**Run the existing pipeline on Faster-LIO odometry** (in the `polegeo` env, which
has YOLO/torch/ouster-sdk/bagpy). No code edits needed — three env-var switches
are wired in, all with sensible defaults:
```bash
conda activate polegeo
INCREMENTAL_NAV_CSV=incremental_navigation_results_fasterlio.csv \
    python snowpole_based_vehicle_localization.py
#   INCREMENTAL_NAV_CSV  odometry source (default = original FastReg CSV)
#   BAG_PATH             reduced bag (default = snow_pole_geo_localization_data/...no_unwanted_topics.bag)
#   RESULTS_CSV          output (default = snowpole_results_fasterlio.csv)
```
The results CSV is now saved automatically right after the processing loop (before
the blocking plot windows). It contains the proposed pole-corrected track
(`Predicted Vehicle Easting/Northing`), the localized poles (`Target Easting/
Northing`), and the reference (`vehicle_easting_original/northing_original`).

**Stage 2 — temporal evolution of vehicle motion + localized snow poles:**
```bash
python fasterlio_integration/scripts/20_temporal_evolution_visualization.py \
    --fasterlio-csv incremental_navigation_results_fasterlio.csv \
    --poles "Groundtruth_pole_location_at_test_site_E39_Hemnekjølen.csv" \
    --results-csv snowpole_results_fasterlio.csv     # adds pole-corrected track + localized poles
# → output/temporal_evolution.mp4 + summary_{trajectories,error_hist,error_cdf,error_vs_distance}.png
```
Without `--results-csv` you get the odometry-level animation (GNSS + Faster-LIO +
georeferenced poles), which is already generated in `output/`.

## 4b. Measured results (this dataset, full bag)

Faster-LIO produced **5412 poses** (≈ the 5419 LiDAR scans), full 541 s span,
running at real time (1.0× playback, 20 vCPU) with `point_filter_num: 2`.

| Metric | Value | Meaning |
|---|---|---|
| Relative pose error (KITTI-style, start-heading removed) | **~3.3 %** across 25–400 m segments | local frame-to-frame accuracy |
| Dead-reckoning error over one pole spacing (~25–50 m) | **~0.8–1.6 m** | error the poles must snap back |
| Global odometry-only drift vs GNSS (start-anchored) | median **~450 m**, max ~2850 m over 10 km | the drift the snow poles exist to correct |
| Trajectory shape vs GNSS | excellent (see `output/summary_trajectories.png`) | |

Interpretation: the **local** odometry is good (what the pipeline dead-reckons
with between pole sightings); the **global** drift is dominated by slow yaw from
the low-grade Ouster ICM-20948 IMU on long highway stretches, and is exactly what
the georeferenced snow poles correct downstream. Denser points (`point_filter_num`
4→2) only marginally reduced it (505→449 m median), confirming it is IMU- not
density-limited. Note the original FastReg CSV tracks GNSS to ~50 m — consistent
with it being periodically GNSS-anchored rather than a pure-odometry baseline — so
compare against the **pole-corrected** output, not raw odometry, for a fair result.

**End-to-end result (pole-corrected, run on this machine).** The full detection
pipeline was executed on Faster-LIO odometry (YOLOv5 pole detection + geo-loc +
pole-corrected dead-reckoning), **355 pole detections across 132 unique poles** (each pole
seen in ~3 consecutive frames; 132 of 290 site poles were on the traversed section):

| Trajectory | Error vs GNSS (median / mean / max) |
|---|---|
| Faster-LIO odometry only | 447 / 789 / 2855 m |
| **Proposed (pole-corrected)** | **8.6 / 6.8 / 16.0 m** |

→ a **~24× reduction** in localization error from the snow-pole correction, i.e.
the framework works with Faster-LIO as the odometry backend. The ~9 m residual
(vs the paper's sub-metre with partial GNSS) is because the pure-odometry yaw
drift skews the *heading* used to dead-reckon between pole sightings; reducing the
Faster-LIO yaw drift (better IMU / tuning) would tighten it further.

To run the pipeline on this ROS 2 host (no `polegeo`/ROS 1 needed): a conda env is
built by `scripts/setup_polegeo_env.sh`, and the ROS 1 `bagpy` reader in
`geoloc_utils.process_ros_bag_data` was replaced with pure-python `rosbags`. Run:
```bash
env -u PYTHONPATH MPL_BACKEND=Agg \
    INCREMENTAL_NAV_CSV=incremental_navigation_results_fasterlio.csv \
    RESULTS_CSV=snowpole_results_fasterlio.csv \
    ~/miniconda3/envs/polegeo/bin/python snowpole_based_vehicle_localization.py
```

## 5. Files

| File | Purpose |
|---|---|
| `config/ouster_os2_128.yaml` | Faster-LIO config for this exact sensor (extrinsics, 128-line, 10 Hz, de-skew) |
| `config/mapping_ouster_os2_128.launch` | roslaunch for the mapping node |
| `docker/Dockerfile`, `docker/run_docker.sh` | reproducible ROS Noetic + Faster-LIO build/run |
| `scripts/00_run_fasterlio.sh` | run Faster-LIO on the bag, record `/Odometry` |
| `scripts/10_fasterlio_traj_to_csv.py` | **the bridge**: trajectory → GNSS-anchored `easting/northing` CSV |
| `scripts/20_temporal_evolution_visualization.py` | final animated + static results |
| `output/` | generated odometry bag, CSV, figures |

## 6. Design notes / correctness

- **Clock:** stage 1 fits `epoch = a·sensor + b` from the IMU (shares the LiDAR
  clock; cheap) to place Faster-LIO poses on the GNSS timeline, then resamples
  onto the GNSS frames → the output CSV has the **same row layout** as the
  original (drop-in).
- **Frame alignment (SE2, no scale)** is a reflection-guarded Kabsch solve
  (unit-tested: recovers a known rotation/translation to ~1 cm). Heading uses the
  pipeline's convention (0°=N, 90°=E, clockwise), also verified.
- **UTM33N (EPSG:32633)** throughout, matching the pipeline and the poles file.
  (The bag's `/utm_position` topic is zone 32 — deliberately not used.)

## 7. Fallback: reduced bag (no `/ouster/points`)

If only the reduced bag is available, rebuild organized clouds from
`/ouster/range_image` with the Ouster `XYZLut` + `Trip068.json`
(as in `rosbag_utils/range_image_to_pointcloud_visualization.py`), attach
per-point `t`/`ring`, and republish as `PointCloud2` for Faster-LIO. IMU is
already present. This is only needed if the 39 GB bag is unavailable.
