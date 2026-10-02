# GLIM integration — complete guide & map

*The one document that explains the whole `glim_integration/` folder: what GLIM is,
why it's here, how the pipeline works end-to-end, how to run it from bash, and what
results it produces. Written so that a reader with **no prior knowledge** of SLAM,
LiDAR, or this codebase can follow it and leave informed comments.*

> **TL;DR** — We swapped the project's LiDAR odometry front-end (originally *FastReg*,
> later *Faster-LIO*) for **GLIM**, a GPU-accelerated LiDAR-inertial SLAM system, to
> see whether a GPU could speed up and/or improve the vehicle-motion estimate that the
> snow-pole localization pipeline depends on. GLIM runs in a Docker container, consumes
> the LiDAR + IMU data, and produces a vehicle **trajectory**. A small "bridge" script
> turns that trajectory into the exact CSV the existing pipeline already reads, so
> **nothing downstream changes**. Result on this laptop: GLIM **matches** Faster-LIO's
> localization accuracy but does **not** beat it on speed (the 4 GB laptop GPU can't
> out-run the 20-thread CPU for this workload). Full numbers in §7.

---

## Table of contents

1. [Background for a newcomer](#1-background-for-a-newcomer)
2. [What GLIM is (the odometry method)](#2-what-glim-is-the-odometry-method)
3. [Advantages & disadvantages](#3-advantages--disadvantages)
4. [Where GLIM fits in this project](#4-where-glim-fits-in-this-project)
5. [Hardware & environment](#5-hardware--environment)
6. [The complete pipeline, stage by stage](#6-the-complete-pipeline-stage-by-stage)
7. [Results (measured)](#7-results-measured)
8. [How to run it from bash](#8-how-to-run-it-from-bash)
9. [Configuration explained](#9-configuration-explained)
10. [Folder map — every file](#10-folder-map--every-file)
11. [Gotchas, bugs found & fixed](#11-gotchas-bugs-found--fixed)
12. [Verdict — when to use GLIM](#12-verdict--when-to-use-glim)
13. [Glossary & references](#13-glossary--references)

---

## 1. Background for a newcomer

**The problem this project solves.** A vehicle drives along a snowy Nordic road
(test site E39 Hemnekjølen, Norway). Along the road there are **snow poles** —
roadside marker poles. The goal is to figure out, at every moment, *where the vehicle
is* and *where each snow pole is*, accurately, even when satellite positioning (GNSS /
GPS) is weak or switched off. A camera detects the poles; but to place a detected pole
on a map you must know where the *vehicle* was when it saw the pole. That vehicle
position-over-time is called **odometry**.

**What is odometry?** "Odometry" = estimating how far and in what direction something
has moved, step by step, from its own sensors — *without* relying on an external map or
GPS. Think of it as dead-reckoning: "I moved 1 m forward and turned 2° left, then 1 m
more…". Add all the little steps up and you get a **trajectory** (a path through space).

**LiDAR odometry.** A **LiDAR** is a laser scanner that spins and measures the distance
to everything around it ~10 times per second, producing a dense 3-D **point cloud**
(here: an Ouster OS-2-128, ~131,000 points per scan). If you can figure out how the
cloud at time *t* has rotated/translated relative to the cloud at time *t−1* (a process
called **registration** / **scan matching**), you know how the sensor moved in that
0.1 s. Chain those together → a trajectory. LiDAR odometry is accurate locally but
slowly **drifts** over long distances (small per-scan errors accumulate).

**LiDAR-*inertial* odometry (LIO).** Add an **IMU** (Inertial Measurement Unit — an
accelerometer + gyroscope, ~100 Hz) and you can predict the motion *between* LiDAR
scans and correct the LiDAR match. Fusing LiDAR + IMU = **LIO**. It's more robust (the
IMU handles fast motion and fills gaps) and gives orientation (especially roll/pitch,
via gravity). GLIM, Faster-LIO, Super-LIO, Point-LIO are all LIO systems.

**Why the vehicle still drifts here.** The Ouster's *built-in* IMU is a low-grade MEMS
unit. It helps rotation but barely helps translation. Over this ~10 km one-way drive
the heading (yaw) slowly drifts, so the *raw* odometry can be hundreds of metres off by
the end — **for every LIO backend tested**, GLIM included. That is expected and is
*fixed downstream* by the snow-pole + GNSS correction (see §4). So the number that
actually matters is the **pole-corrected** error, not the raw-odometry error.

---

## 2. What GLIM is (the odometry method)

**GLIM** (by Kenji Koide, a.k.a. *koide3*) is an open-source, **GPU-accelerated**,
tightly-coupled **range-inertial** SLAM framework. "SLAM" = Simultaneous Localization
And Mapping: it builds a map *and* localizes within it at the same time. The three
ideas that define GLIM:

1. **GPU VGICP registration.** To match two point clouds, GLIM uses **VGICP** (Voxelized
   Generalized Iterative Closest Point). Instead of matching point-to-point, it models
   each small voxel (3-D cube) of space as a little Gaussian distribution and aligns the
   *distributions*. This is robust and, crucially, **parallelizes well on a GPU (CUDA)**
   — thousands of voxels matched at once. This is GLIM's headline feature: the heavy
   per-scan matching runs on the graphics card instead of the CPU.

2. **Factor-graph optimization (tight fusion).** GLIM does not just chain frame-to-frame
   guesses. It builds a **factor graph** — a mathematical structure where "nodes" are
   poses/velocities/IMU-biases and "factors" are constraints (a LiDAR match says *these
   two poses are related like this*; an IMU reading says *the motion between them was
   like that*). An optimizer (GTSAM / iSAM2) finds the set of poses that best satisfies
   **all** constraints jointly. This "tight coupling" of LiDAR + IMU in one optimization
   is more accurate than loosely averaging two separate estimates.

3. **Local + global mapping (drift fighting).** GLIM has three layers:
   - **Odometry** — fast, per-scan pose estimation (the real-time part). *[GPU here]*
   - **Local/sub-mapping** — groups scans into "submaps" and refines them.
   - **Global mapping** — a pose graph over all submaps that can, in principle, apply
     **loop closures** (recognizing "I've been here before") to erase accumulated drift.
     *(On a one-way drive there are no loops to close, so this helps little here — see
     §3 disadvantages.)*

In short: **GLIM = GPU VGICP scan matching + IMU, fused in a factor graph, with a global
pose-graph back-end.** It is a more "heavyweight, map-building" approach than a pure
odometry filter like Faster-LIO's iVox+ESIKF.

### How GLIM compares to the other two backends in this repo

| | **FastReg** (original) | **Faster-LIO** | **GLIM** (this folder) |
|---|---|---|---|
| Type | learned pairwise registration (deep net) | LiDAR-inertial odometry filter | LiDAR-inertial SLAM (factor graph) |
| Engine | PointNet++ GNN + RANSAC, **GPU only** | CPU iVox + ESIKF | **GPU VGICP** + GTSAM factor graph |
| Uses IMU? | no | yes | yes |
| Global opt. / loop closure | no | no | **yes** (pose graph) |
| Runtime | Python/torch | Docker `ros:noetic` (ROS 1) | Docker `koide3/glim_ros2` (ROS 2) |
| Role here | provided a precomputed CSV | regenerates the CSV | regenerates the CSV |

---

## 3. Advantages & disadvantages

**Advantages of GLIM (in general, and here)**
- **GPU offload** — the dense per-scan matching moves off the CPU onto the GPU. On a
  strong desktop GPU this is a big win on dense clouds.
- **Tight LiDAR-IMU fusion in a factor graph** — generally more accurate and more robust
  to aggressive motion than frame-to-frame filters.
- **Global optimization / loop closure** — can remove accumulated drift when the path
  revisits places; produces a globally consistent map.
- **Modular & configurable** — GPU/CPU variants per stage (odometry / sub-mapping /
  global mapping) are swappable via config, which let us fit a 4 GB GPU (§9).
- **Native ROS 2** — matches this host (ROS 2 Jazzy); the prebuilt CUDA image runs with
  no source build.
- **Matched the accuracy** of the tuned Faster-LIO baseline here (§7) — the GPU route
  cost no accuracy.

**Disadvantages (general, and specifically on this hardware)**
- **GPU + CUDA dependency.** Needs an NVIDIA GPU, the right driver, and the
  `nvidia-container-toolkit` for Docker. More moving parts than a pure-CPU backend.
- **VRAM-bound.** This laptop GPU has only **4 GB**, forcing GPU odometry + **CPU**
  global mapping (full-GPU would risk out-of-memory over 10 km). That caps the speedup.
- **Did NOT beat Faster-LIO on speed here.** GLIM ran at ~2.5× real-time; Faster-LIO
  *offline* runs at 4.7–7.9× on the 20-thread CPU. GLIM also does more work
  (factor graph + sub/global mapping). The entry-level GPU can't out-run the strong CPU
  for this workload. **(This is the single most important, non-obvious result.)**
- **No loop closures on a one-way drive** → GLIM's global back-end can't fix the global
  yaw drift here; raw-odometry drift is ~460 m median, similar to Faster-LIO.
- **Extra one-time cost:** the ROS 1 bag must be converted to a ROS 2 bag (~33 GB,
  ~160 s) before GLIM can read it (Faster-LIO played the ROS 1 bag natively).
- **Low-grade IMU limits the tight-fusion edge** — GLIM itself logs *"IMU prediction is
  not good… rot=0.93, trans=0.08, vel=0.14"*: the Ouster built-in IMU helps rotation but
  not translation, so GLIM's tight-fusion advantage is blunted on this dataset.

---

## 4. Where GLIM fits in this project

The crucial insight that makes this a *drop-in* swap:

> **The snow-pole pipeline never runs the odometry live.** It reads a CSV file,
> `incremental_navigation_results*.csv`, and uses only its **`easting` / `northing`**
> columns (UTM33N map coordinates, one row per GNSS frame = 5422 rows). From those it
> derives the per-frame heading + translation to dead-reckon the vehicle between pole
> sightings.

So "replace the odometry backend" = **regenerate that CSV** from a new source. Every
backend (FastReg, Faster-LIO, GLIM, Super-LIO) ends by writing its own
`incremental_navigation_results_<backend>.csv`; the pipeline is pointed at it with the
`INCREMENTAL_NAV_CSV` environment variable. **Nothing downstream of the CSV changes.**

```
  GLIM trajectory ──► [bridge] ──► incremental_navigation_results_glim.csv ──► snow-pole pipeline
                                   (easting, northing per GNSS frame)            (unchanged)
```

---

## 5. Hardware & environment

| | |
|---|---|
| GPU | **NVIDIA RTX 3050 Ti Laptop, 4 GB**, driver 595.84 (supports CUDA 13.1 container) |
| CPU | Intel i7-12700H, 20 threads |
| Host OS / ROS | Ubuntu (Linux Mint), **ROS 2 Jazzy** |
| Docker | works **without sudo**; `nvidia-container-toolkit` installed (`nvidia` runtime present) |
| GLIM image | `koide3/glim_ros2:jazzy_cuda13.1` (~14.4 GB; matches host natively) |
| Dataset bag | `snow_pole_geo_localization_data/2024-02-28-12-59-51.bag` (ROS 1, ~41 GB, 541.8 s) |
| Sensor | Ouster **OS-2-128**, 1024×128 @ 10 Hz; `/ouster/points` (PointCloud2) + `/ouster/imu` (100 Hz) |
| LiDAR↔IMU extrinsic | from `Trip068.json`: R = I, t = (−0.006253, +0.011775, −0.007645) m |
| Python envs | `polegeo` conda env (pipeline + viz); repo-root `.baginspect_venv` (bag reading + bridge) |

**Why Docker + ROS 2?** GLIM ships only ROS 2 images. The host is ROS 2 Jazzy, so the
`jazzy_cuda13.1` image matches with no source build. But the dataset bags are **ROS 1**,
and GLIM's `glim_rosbag` reads **ROS 2** bags — hence the conversion step (Stage 00).

---

## 6. The complete pipeline, stage by stage

```
                      ROS 1 .bag (41 GB, /ouster/points + /ouster/imu + GPS)
                                        │
   [00] 00_convert_bag.sh    ───────────▼───────────   rosbags-convert (ROS1→ROS2, points+imu only)
                                        │               → output/ros2_bag/   (~32 GB, scratch)
                                        │
   [01] 01_extract_and_patch_configs.sh ▼               pull GLIM defaults from the image, then
                                        │               patch our OS-2-128 overrides → config/*.json
                                        │
   [02] 02_run_glim.sh       ───────────▼───────────   glim_rosbag in Docker (--gpus all, headless)
                                        │               → output/glim_traj_gpu.txt   (TUM trajectory)
                                        │               + realtime factor from GLIM's own log
                                        │
   [10] 10_glim_traj_to_csv.py ─────────▼───────────   bridge: TUM + GNSS/IMU-clock from the ORIGINAL bag
                                        │               Kabsch start-anchor → UTM33N (EPSG:32633)
                                        │               → incremental_navigation_results_glim.csv  (5422 rows)
                                        │
   [B]  snowpole_based_vehicle_localization.py ▼        YOLO pole detection + geo-loc + GNSS fusion
                                        │               → snowpole_results_glim.csv + live_map_glim.png
                                        │
   [C]  20_temporal_evolution_visualization.py ▼        animation + static summary figures
                                        │               → temporal_evolution_glim.mp4 + summary_*.png
                                        │
   [D]  (Stage D of run_full_pipeline_glim.sh) ▼        append ONE metrics row per run
                                                        → output/run_metrics.csv + run_metrics.md
```

### Stage 00 — convert the bag (`scripts/00_convert_bag.sh`)
- **What:** `rosbags-convert` rewrites the ROS 1 bag as a ROS 2 bag, keeping **only**
  `/ouster/points` + `/ouster/imu` (both standard message types, so no custom message
  definitions are needed).
- **Why:** GLIM reads ROS 2 bags; the dataset is ROS 1.
- **Output:** `output/ros2_bag/` — sqlite3, ~32 GB, exact counts /ouster/points = 5419,
  /ouster/imu = 54202, ~160 s to convert. **Delete it after the run** to reclaim ~32 GB
  (it's only needed to re-run GLIM itself).
- Uses the repo-root `.baginspect_venv` (already has the `rosbags` library).

### Stage 01 — configs (`scripts/01_extract_and_patch_configs.sh` + `patch_configs.py`)
- **What:** Runs the GLIM container once to copy its *default* config files out of the
  image (`/root/ros2_ws/src/glim/config`), then `patch_configs.py` overlays only our
  keys (topics, extrinsic, GPU odometry, headless). See §9.
- **Why this way:** We never hand-write full GLIM configs — we patch the image's real
  defaults, so we can't drift from GLIM's schema. The patcher *warns* (doesn't silently
  append) if a key is missing, so schema changes are visible.

### Stage 02 — run GLIM (`scripts/02_run_glim.sh`)
- **What:** Launches `glim_rosbag` inside the container with `--gpus all`, headless,
  reading `output/ros2_bag`. GLIM processes every scan and, on exit, dumps the estimated
  trajectory as a **TUM** file (`timestamp x y z qx qy qz qw`) to `/tmp/dump`, which is
  bind-mounted to `output/glim_dump/`. We copy `traj_lidar.txt` → `output/glim_traj_gpu.txt`.
- **Essential flag:** `-p auto_quit:=true` — without it `glim_rosbag` blocks on a
  keypress after playback and hangs forever in headless mode.
- **Throughput measurement:** wall-clock is corrupted by any idle, so the script parses
  GLIM's own per-scan log timestamps (first→last, ignoring >60 s gaps) to report a clean
  `REALTIME_FACTOR = bag_seconds / processing_span`.
- **Output:** `output/glim_traj_gpu.txt` (5407 poses), `output/glim_run_gpu.log`.

### Stage 10 — the bridge (`scripts/10_glim_traj_to_csv.py`)
This is the glue that makes GLIM a drop-in replacement. It is a 1:1 copy of the
Faster-LIO bridge maths. Steps:
1. Read GLIM's TUM trajectory (x, y, timestamps on the Ouster **sensor clock**, ~13215 s).
2. Read the **original** ROS 1 bag for GNSS (both GPS antennas, averaged) and the IMU,
   which shares the sensor clock. Fit an affine map **epoch = a·sensor + b** from the IMU
   so GLIM's sensor-clock timestamps line up with GNSS Unix-epoch time (fit residual ~6.9 ms).
   *(A guard re-bases the trajectory if GLIM ever emits timestamps relative to 0.)*
3. **Rigidly align** the GLIM local frame into the UTM33N map (EPSG:32633) with a 2-D
   **Kabsch** (rotation + translation, **no scale**):
   - `--align start` *(default, the fair metric)* — fit using only a moving band of the
     **initial** GNSS travel (skip the first ~20 m idle, use the next ~400 m). This pins
     the start pose + heading and **preserves the drift** → a faithful odometry-only track.
   - `--align full` — best-fit over the whole track (visualization only; hides drift).
4. Resample onto the GNSS frame timestamps (reproduces the original CSV's 5422-row layout)
   and write `easting`, `northing` (+ `latitude`, `longitude`, `gnss_easting`,
   `gnss_northing`, `heading`, `gnss_error`).
- **Output:** `incremental_navigation_results_glim.csv` (in the repo root; 5422 rows).

### Stage B — snow-pole localization (`snowpole_based_vehicle_localization.py`)
- The **unchanged** main pipeline, pointed at the GLIM CSV via `INCREMENTAL_NAV_CSV`.
- Loads a YOLOv5 pole detector, reads camera frames from the (reduced) bag, detects
  poles, geo-locates each by combining the detection with the vehicle position
  (dead-reckoned from the CSV's easting/northing), and optionally fuses a fraction of
  GNSS fixes. "0 % GNSS" = the proposed GNSS-free method.
- Self-reports the backend from the CSV name (`glim` → **"GLIM"**) and writes timing
  metrics to `fasterlio_integration/output/timing_GLIM_pipeline_gnss0.json`.
- **Output:** `snowpole_results_glim.csv` (per-detection results) + `output/live_map_glim.png`.

### Stage C — visualization (`fasterlio_integration/scripts/20_temporal_evolution_visualization.py`)
- Renders the FastReg-style **temporal-evolution animation** (vehicle + poles over time)
  and four static summary figures. `--odom-label GLIM` labels the plots correctly.
- **Output:** `output/temporal_evolution_glim.mp4`, `output/summary_{trajectories,error_hist,error_cdf,error_vs_distance}.png`.

### Stage D — per-run metrics (inside `run_full_pipeline_glim.sh`)
- Appends **one row per run** (never overwritten) to `output/run_metrics.csv` and
  regenerates a readable `output/run_metrics.md` table, capturing the headline numbers
  (throughput, poses, clock residual, drift, CSV rows, pole-corrected & odometry-only
  median error). It keeps **only** those two small files — no bulky copies. Needed
  because every other artifact uses a fixed filename and is overwritten each run.

---

## 7. Results (measured)

*Measured on this laptop (i7-12700H + RTX 3050 Ti 4 GB). Re-verified end-to-end on
2026-10-02 across two full `--from-bag` runs + one GNSS sweep; all exit 0.*

### 7.1 Speed (odometry throughput)

| Backend | Engine | Wall / processing | Throughput |
|---|---|---|---|
| **GLIM** (GPU odom + CPU global) | GPU VGICP + factor graph | ~220 s processing | **~2.5× real-time** (26 FPS) |
| Faster-LIO (offline, downsampled) | CPU iVox + ESIKF | 111 s | 7.9× (79 FPS) |
| Faster-LIO (offline, baseline) | CPU iVox + ESIKF | 159 s | 4.7× (47 FPS) |

**GLIM does not beat Faster-LIO on this hardware.** GLIM's "2.5× real-time" only looked
like a win when mistakenly compared against Faster-LIO's *rosbag-play* run (rate-capped
at ~1×). Run like-for-like (both offline / max speed), the CPU wins here.

### 7.2 Accuracy — pole-corrected median error vs GNSS availability (`GNSS_SEED=0`)

| GNSS used | FastReg | Faster-LIO | **GLIM** (this run) | mean / max | poses w/ GNSS |
|---|---|---|---|---|---|
| 0 %  | 8.41 m | 10.11 m | **10.20 m** | 9.66 / 27.6 | 0 |
| 10 % | 2.13 m | 1.05 m | **1.17 m** | 2.34 / 16.8 | 561 |
| 25 % | 1.29 m | 0.61 m | **0.64 m** | 1.02 / 9.9 | 1312 |
| 50 % | 0.53 m | 0.25 m | **0.26 m** | 0.48 / 9.2 | 2669 |

- **GLIM tracks Faster-LIO 1:1** across the whole sweep (within ~0.1 m at every level).
  The GPU route costs no accuracy.
- **0 % GNSS is the hardest case** for any LIO backend. Once ≥10 % GNSS is available the
  pole-correction collapses the error ~5–10× and both LIO backends beat FastReg.
- The honest metric is **pole-corrected** error. GLIM's *raw* odometry-only median at
  0 % is ~178 m (drift), corrected down to ~10 m by poles — a ~17× improvement.

### 7.3 Raw odometry drift (odometry-vs-GNSS, start-anchored)

~460 m median / ~760 m mean / ~2580 m max over the 10 km drive — comparable to
Faster-LIO's median (IMU-limited yaw drift), with a slightly heavier tail. There are no
loop closures on a one-way drive, so GLIM's global back-end can't remove it.

### 7.4 Pipeline cost (odometry-agnostic)

Wall-clock ~130 s, ~63 frames/s over 5420 frames; YOLO detection ~23 ms/frame (~43 fps).
Essentially identical for every backend — the localization stage doesn't care which
odometry produced the CSV.

### 7.5 Reproducibility — the per-run metrics log (`output/run_metrics.md`)

| run | backend | RT factor | odom proc (s) | poses | clock resid (ms) | drift med (m) | CSV rows | pole-corr med (m) | odom-only med (m) |
|---|---|---|---|---|---|---|---|---|---|
| 2026-10-02T00:37 | GLIM | 2.49 | 217.8 | 5407 | 6.9 | 462.19 | 5422 | 10.22 | 173.89 |
| 2026-10-02T00:52 | GLIM | 2.46 | 220.1 | 5407 | 6.9 | 460.07 | 5422 | 10.20 | 177.98 |

Numbers reproduce deterministically run-to-run (small GPU-timing + YOLO variance only).

### 7.6 Pole detection accounting (0 % GNSS run)

*"How many poles were detected, and how many were actually used?"* The detector
(YOLO) fires a **bounding box on every pole candidate in every processed frame**;
each box is then filtered out if **(a)** it has no valid 3-D return in the range
image, or **(b)** its nearest 3-D point is beyond the `distance_threshold = 5 m`
false-positive gate (`snowpole_based_vehicle_localization.py`). Survivors become
**used** geo-localization events — one row in the results CSV. Because the *same
physical pole is seen across many consecutive frames* (plus some false positives),
the raw box count is ~6–7× the number actually used.

For **GLIM** (0 % GNSS):

| Step | Count | Meaning |
|---|---:|---|
| Frames YOLO ran on | 2146 | in-bounds frames the detector saw |
| **Raw detections (boxes)** | **2326** | every candidate box |
| — dropped: no 3-D point in box | 5 | no usable range return there |
| — dropped: nearest point > 5 m | 1966 | false-positive gate |
| **Used (geo-localized → CSV rows)** | **355** | = rows of `snowpole_results_glim.csv` |
| Distinct ground-truth poles hit | 133 | of **290** poles at the site |

Balance: `2326 − 5 − 1966 = 355`. ✓ (The 355 events re-sight 133 distinct physical
poles; the rest are repeat views of the same poles.)

Detection is **almost backend-independent** — it runs on the same camera/LiDAR
images regardless of odometry. The only coupling is the *in-bounds* test, which uses
the odometry-**predicted** vehicle position, so a few boundary frames differ per
backend (hence the small spread below):

| Backend | YOLO frames | raw boxes | drop (no-pt) | drop (>5 m) | **used** | distinct poles |
|---|---:|---:|---:|---:|---:|---:|
| Faster-LIO | 2141 | 2325 | 5 | 1965 | 355 | 135 |
| **GLIM** | 2146 | 2326 | 5 | 1966 | **355** | 133 |
| Super-LIO | 2139 | 2317 | 5 | 1956 | 356 | 133 |

**Check it yourself:**

```bash
# USED count — always available (persistent artifacts):
echo $(( $(wc -l < snowpole_results_glim.csv) - 1 ))                        # -> 355
grep -E 'pole_detection_events|detection_frames' \
     fasterlio_integration/output/timing_GLIM_pipeline_gnss0.json           # 355 ; 2146

# RAW + drop reasons — from the pipeline STDOUT (captured in the 0 % sweep log):
LOG=glim_integration/output/sweep_glim_gnss0.log
grep -c "sequence number used for geo localization" "$LOG"   # raw boxes  -> 2326
grep -c "no valid nearest point found"              "$LOG"   # no 3-D pt  -> 5
grep -c "skipping this bounding box"                "$LOG"   # > 5 m gate -> 1966

# DISTINCT physical poles mapped:
~/miniconda3/envs/polegeo/bin/python -c "import pandas as pd; r=pd.read_csv('snowpole_results_glim.csv'); print(r[['Ground Truth Easting','Ground Truth Northing']].round(2).drop_duplicates().shape[0])"
```

> ⚠️ The log line `sequence number used for geo localization` is **misnamed** — it
> prints for *every* box *before* filtering, so it counts **raw detections**, not
> used ones. If the sweep log is gone, regenerate any pipeline run with stdout
> redirected to a file and grep that.

---

## 8. How to run it from bash

> All commands below are run from the project directory:
> `Snow-pole-based-vehicle-localization/`

### 8.1 One-time provisioning (only once per machine)

```bash
# 1) Install the NVIDIA container toolkit so Docker can use the GPU (needs sudo).
#    Run with a leading "!" in the Claude session so the sudo password prompt is visible:
!  sudo bash glim_integration/docker/setup_nvidia_container_toolkit.sh

# 2) Pull the prebuilt GLIM CUDA image (~14 GB, no sudo):
bash glim_integration/docker/pull_glim.sh
#    CPU-only fallback (validation only, NOT the speed test):  pull_glim.sh cpu
```

### 8.2 The whole experiment in one command (recommended)

```bash
# Regenerate everything from the raw bag: convert → configs → GLIM → bridge → CSV →
# snow-pole pipeline (0 % GNSS) → animation → per-run metrics.
glim_integration/run_full_pipeline_glim.sh --from-bag

# Reuse an existing incremental_navigation_results_glim.csv (skip odometry, just
# re-run the pipeline + viz + metrics):
glim_integration/run_full_pipeline_glim.sh
```
Outputs are listed at the end of the run (odometry CSV, results CSV, live map, the
`temporal_evolution_glim.mp4` animation, the summary PNGs, and `run_metrics.md`).

### 8.3 Running the stages individually

```bash
# 00  ROS1 .bag → ROS2 bag (points + imu).  --force to overwrite an existing one.
glim_integration/scripts/00_convert_bag.sh

# 01  materialise + patch GLIM configs (idempotent).  --force to re-extract defaults.
glim_integration/scripts/01_extract_and_patch_configs.sh

# 02  run GLIM headless on the GPU; prints REALTIME_FACTOR; dumps the TUM trajectory.
glim_integration/scripts/02_run_glim.sh            # label defaults to "gpu"

# 10  bridge the trajectory into the pipeline CSV (GNSS start-anchor, UTM33N).
.baginspect_venv/bin/python glim_integration/scripts/10_glim_traj_to_csv.py \
    --dataset-bag snow_pole_geo_localization_data/2024-02-28-12-59-51.bag \
    --tum        glim_integration/output/glim_traj_gpu.txt \
    --out        incremental_navigation_results_glim.csv \
    --align start

# run_all.sh chains 00 → 01 → 02 → 10 (skips convert if the ROS2 bag already exists):
glim_integration/run_all.sh
```

### 8.4 Run just the snow-pole pipeline on the GLIM CSV

```bash
env -u PYTHONPATH MPL_BACKEND=Agg \
    INCREMENTAL_NAV_CSV=incremental_navigation_results_glim.csv \
    ~/miniconda3/envs/polegeo/bin/python snowpole_based_vehicle_localization.py
```
*(`env -u PYTHONPATH` is essential — it stops ROS 2 Jazzy's site-packages leaking into
the conda env. `MPL_BACKEND=Agg` runs matplotlib headless.)*

### 8.5 The accuracy sweep & the speed benchmark

```bash
# GNSS-% sweep (0/10/25/50 %, seeded) → prints the §7.2 table:
glim_integration/scripts/40_gnss_sweep.sh incremental_navigation_results_glim.csv glim

# Throughput-only benchmark (realtime factor vs Faster-LIO):
glim_integration/scripts/run_offline_bench.sh gpu
```

### 8.6 Reclaim disk after a run

```bash
rm -rf glim_integration/output/ros2_bag     # ~32 GB, only needed to re-run GLIM
rm -rf glim_integration/output/glim_dump    # ~1.1 GB submaps, not used downstream
```

---

## 9. Configuration explained

Configs live in `config/`. They are GLIM's own defaults (extracted from the image by
Stage 01) **plus** our overrides from `config/overrides.json`, applied by
`patch_configs.py` (which sets each key wherever it appears in the JSON tree, so it
doesn't depend on GLIM's exact wrapper nesting). Our overrides:

| File | Key | Value | Why |
|---|---|---|---|
| `config_ros.json` | `points_topic` / `imu_topic` | `/ouster/points` / `/ouster/imu` | our sensor topics |
| | `acc_scale` | `1.0` | Ouster accel is already m/s² (z ≈ 9.94); GLIM's default 0.0 is wrong for us |
| | `extension_modules` | `[]` | drop the viewer libs → **headless** |
| | `enable_local_mapping` / `enable_global_mapping` | `true` / `true` | run the full stack |
| `config_sensors.json` | `T_lidar_imu` | `[-0.006253, 0.011775, -0.007645, 0,0,0,1]` | LiDAR↔IMU extrinsic from `Trip068.json` (R=I → identity quaternion) |
| `config.json` | `config_odometry` | `config_odometry_gpu.json` | **GPU** VGICP — the speed-critical part |
| | `config_sub_mapping` | `config_sub_mapping_cpu.json` | **CPU** (async, doesn't gate real-time) |
| | `config_global_mapping` | `config_global_mapping_pose_graph.json` | **CPU** pose graph — **VRAM-safe** on 4 GB over 10 km |

**The 4 GB VRAM decision (important):** GPU odometry (bounded memory, speed-critical) but
CPU sub/global mapping (they run asynchronously and don't gate the odometry real-time
factor). Full-GPU global mapping would risk out-of-memory on a 10 km drive. To go
full-GPU on a bigger card later: point `config_global_mapping` at the GPU variant, or set
`enable_global_mapping=false` for a pure odometry-vs-Faster-LIO comparison.

`config_preprocess.json` downsamples each scan to ~10,000 points
(`random_downsample_target: 10000`, `downsample_resolution: 1.0 m`) — GLIM's analogue of
Faster-LIO's voxel filter. Benign startup warnings like *"param … not found"* are GLIM
noting optional defaults we don't override.

---

## 10. Folder map — every file

```
glim_integration/
├── summary.md                      ← THIS FILE (the whole map)
├── PLAN.md                         original design document (deep rationale, read for history)
├── README.md                       quick-start / layout
├── GLIM_VS_FASTERLIO_COMPARISON.md head-to-head write-up (speed + accuracy, the §7 source)
│
├── run_full_pipeline_glim.sh       ★ ONE COMMAND: odometry → pipeline → viz → metrics
├── run_all.sh                        chains the odometry stages 00→01→02→10
│
├── docker/
│   ├── setup_nvidia_container_toolkit.sh   one-time host prep (sudo): GPU-in-Docker
│   ├── pull_glim.sh                        pull the CUDA (or cpu) image
│   └── run_glim_docker.sh                  run a command in the container (--gpus all, mounts)
│
├── scripts/
│   ├── 00_convert_bag.sh           ROS1 .bag → ROS2 bag (points+imu) via .baginspect_venv
│   ├── 01_extract_and_patch_configs.sh   materialise + patch GLIM configs
│   ├── patch_configs.py            stdlib JSON patcher used by 01 (recursive key-set, warns on miss)
│   ├── 02_run_glim.sh              run GLIM headless on GPU; dump TUM traj; print realtime factor
│   ├── 10_glim_traj_to_csv.py      ★ bridge: GLIM TUM + GNSS → pipeline CSV (Kabsch start-anchor)
│   ├── 40_gnss_sweep.sh            GNSS-% accuracy sweep (0/10/25/50 %, seeded)
│   └── run_offline_bench.sh        throughput-only benchmark wrapper around 02
│
├── config/                         GLIM defaults (from the image) + our overrides
│   ├── overrides.json              ← the authoritative record of what we change
│   ├── config.json                 module selection (GPU odom + CPU sub/global mapping)
│   ├── config_ros.json             topics, acc_scale, headless
│   ├── config_sensors.json         T_lidar_imu extrinsic, per-point time handling
│   ├── config_odometry_gpu.json    GPU VGICP odometry params
│   ├── config_preprocess.json      downsampling (~10k pts/scan)
│   ├── config_global_mapping_pose_graph.json   CPU pose-graph back-end
│   └── … (other GLIM default module configs: cpu/gpu/ct variants, viewer, logging)
│
└── output/                         artifacts (gitignored)
    ├── run_metrics.csv / .md       ★ ONE ROW PER RUN — the only per-run history kept
    ├── glim_traj_gpu.txt           GLIM trajectory (TUM) — the bridge input
    ├── glim_traj_imu_gpu.txt       IMU-rate trajectory (diagnostic)
    ├── glim_run_gpu.log            full GLIM log (per-frame timing, realtime factor)
    ├── live_map_glim.png           final localization map (Stage B)
    ├── temporal_evolution_glim.mp4 the animation (Stage C) ← open this
    ├── summary_*.png               4 static summary figures (Stage C)
    ├── sweep_glim_gnss{0,10,25,50}.log   per-% sweep logs (Stage 40)
    ├── ros2_bag/                   ~32 GB converted bag  (scratch — delete after run)
    └── glim_dump/                  ~1.1 GB GLIM submaps  (scratch — not used downstream)

# Produced in the repo root / shared output (not inside glim_integration/):
incremental_navigation_results_glim.csv           the pipeline input CSV (Stage 10)
snowpole_results_glim.csv                          per-detection results (Stage B)
fasterlio_integration/output/timing_GLIM_*.json    timing metrics (shared across backends)
```

**What persists vs. what's overwritten:** every artifact except `run_metrics.*` uses a
**fixed filename and is overwritten each run** (a single "latest" copy — nothing is
accumulated). `run_metrics.csv`/`.md` **append** one row per run to preserve history.

---

## 11. Gotchas, bugs found & fixed

1. **`glim_rosbag` hangs headless.** After playback it waits for a keypress, then saves.
   Fixed by `-p auto_quit:=true` (baked into `02_run_glim.sh`).
2. **Real-time factor must be log-based.** Outer wall-clock is corrupted by any idle;
   `02_run_glim.sh` parses GLIM's own per-scan log timestamps instead.
3. **Re-run permission abort (found & fixed on final proofread).** GLIM's container runs
   as **root**, so its dump (`glim_dump/…`) is written root-owned on the host. On a
   *repeat* run, the host-user `rm -rf` of the dump hit *"Permission denied"*, and under
   `set -euo pipefail` that aborted the whole pipeline at the GLIM stage. **Fix:** the
   pre-run cleanup falls back to an in-container `rm` (root, no sudo), and a post-run
   `chown` hands the dump back to the host user so it never recurs. (The first-ever run
   didn't hit this because the dump didn't exist yet — a classic latent re-run bug.)
4. **Backend mislabel.** The pipeline used to tag metrics as "FastReg" regardless of the
   CSV. Fixed: it now detects `glim` in the CSV name → labels **"GLIM"** and writes
   `timing_GLIM_*.json` (no longer clobbers FastReg's metrics).
5. **PYTHONPATH leak.** The pipeline must run with `env -u PYTHONPATH` or ROS 2 Jazzy's
   site-packages shadow the conda env's numpy/torch.

---

## 12. Verdict — when to use GLIM

**On this laptop:** GLIM **matches** Faster-LIO's pole-corrected accuracy at every GNSS
level but is **slower** (≈2.5× real-time vs Faster-LIO offline's 4.7–7.9×), and adds a
one-time 33 GB/160 s bag conversion. For this project the **highest-value quick win is
running Faster-LIO in *offline* mode** (no new dependency).

**GLIM earns its place when:**
- you have a **stronger (desktop) GPU** where GPU VGICP pulls ahead of the CPU;
- the route has **loop closures** (revisited places) so GLIM's global pose-graph can
  erase drift — not the case on this one-way drive;
- you want a **globally consistent map**, not just a trajectory.

**It would not help here to:** chase speed on the 4 GB laptop GPU. The accuracy is
already at parity with Faster-LIO, and the drift is **IMU-limited**, not
registration-limited — a better IMU would move the needle more than a better matcher.

---

## 13. Glossary & references

- **Odometry** — estimating motion step-by-step from onboard sensors (dead reckoning).
- **Point cloud** — the set of 3-D points a LiDAR returns per scan.
- **Registration / scan matching** — finding the rigid transform aligning two clouds.
- **IMU** — accelerometer + gyroscope; measures acceleration & rotation rate.
- **LIO** — LiDAR-Inertial Odometry (fuses LiDAR + IMU).
- **VGICP** — Voxelized Generalized ICP; GPU-friendly distribution-to-distribution matching.
- **Factor graph** — graph of variables (poses…) + constraints (factors), solved jointly.
- **Loop closure** — recognizing a revisited place to correct accumulated drift.
- **Drift** — slow accumulation of small per-scan errors over distance.
- **Kabsch** — algorithm for the optimal rigid rotation+translation aligning two point sets.
- **TUM format** — trajectory file: `timestamp tx ty tz qx qy qz qw` per line.
- **UTM33N / EPSG:32633** — the projected map coordinate system for this site (metres).
- **Pole-corrected error** — vehicle-position error *after* the snow-pole + GNSS
  correction; the fair accuracy metric here.

**References:**
- GLIM — Koide et al., *GLIM: 3D range-inertial SLAM with GPU acceleration* (koide3/glim).
- Faster-LIO — Bai et al., RA-L 2022 (gaoxiang12/faster-lio).
- FastReg — Arnold et al., RA-L 2022 (eduardohenriquearnold/fastreg).
- Project baselines: `../fasterlio_integration/` (and its
  `SPEED_AND_RELIABILITY_COMPARISON.md`), `../superlio_integration/`.
- Deep rationale for this folder: `PLAN.md`; head-to-head data: `GLIM_VS_FASTERLIO_COMPARISON.md`.

---

*Last updated 2026-10-02 (final GLIM session). Pipeline re-validated end-to-end — two
full `--from-bag` runs + the GNSS sweep, all exit 0, results reproduced.*
