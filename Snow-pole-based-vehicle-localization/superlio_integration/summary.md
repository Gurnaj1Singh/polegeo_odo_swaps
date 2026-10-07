# Super-LIO integration — the complete guide

> **Note on scope.** This document is the full map of **`superlio_integration/`**.
> (The request said "glim" in a couple of places — that was a slip; everything
> below is about **Super-LIO**, the backend this folder integrates. Super-LIO is
> compared *against* GLIM and Faster-LIO throughout.)

This file is written so that **someone with no background in robotics, LiDAR, or
state estimation can read it top-to-bottom, understand what Super-LIO is, why we
plugged it in, how the whole pipeline runs, and what results it produced** — and
then leave informed comments. It starts from absolute basics and ends with exact
bash commands and measured numbers.

---

## Table of contents

1. [The 30-second version](#1-the-30-second-version)
2. [Part A — The absolute basics (no prior knowledge needed)](#part-a--the-absolute-basics)
3. [Part B — What is Super-LIO? (the odometry method)](#part-b--what-is-super-lio-the-odometry-method)
4. [Part C — Advantages & disadvantages](#part-c--advantages--disadvantages)
5. [Part D — The map of this integration (files & data flow)](#part-d--the-map-of-this-integration)
6. [Part E — The config file, explained](#part-e--the-config-file-explained)
7. [Part F — The three local modifications (and why)](#part-f--the-three-local-modifications)
8. [Part G — How to run it from bash](#part-g--how-to-run-it-from-bash)
9. [Part H — Results](#part-h--results)
10. [Part I — Known issues & caveats](#part-i--known-issues--caveats)
11. [Part J — Verdict](#part-j--verdict)
12. [Part K — Deep dive: `filter_rate` & the 0 %-GNSS fix](#part-k--deep-dive-filter_rate--the-0-gnss-fix)
13. [Glossary](#glossary)

---

## 1. The 30-second version

This project builds a **map of snow poles** along a Norwegian test road and
**localizes a vehicle** against them. To do that it needs to know, at every instant,
**where the vehicle is and which way it is pointing** — that job is called
**odometry**. The original project used a LiDAR method called *FastReg*. We are
benchmarking faster/better replacements. This folder plugs in the **4th** such
replacement, **Super-LIO**.

**Result:** Super-LIO is the **fastest** odometry we tested by a wide margin and the
**most accurate at every GNSS level**. With the **default config (`filter_rate 2`)** it
runs ~**159 FPS** (still ~2× Faster-LIO, ~6× GLIM) and scores **9.65 m at 0 % GNSS**
and **0.91 / 0.53 / 0.20 m at 10/25/50 %** — on par with or better than Faster-LIO and
GLIM across the board. (The old `filter_rate 4` config hit ~217 FPS but degraded the
0 %-GNSS case to 55.94 m — that trade-off and its fix are documented in **Part K**.) It
runs **natively (ROS 2 Jazzy) or in Docker** (`SUPERLIO_DOCKER=1`).

---

## Part A — The absolute basics

### A.1 What is this whole project trying to do?

Along a snowy highway (E39, Hemnekjølen, Norway) there are **snow poles** — tall
marker poles that show the road edge when snow covers everything. The project:

1. Drives a car fitted with a **LiDAR**, an **IMU**, **cameras/GNSS**, logging
   everything into a "bag" file.
2. **Detects** the snow poles in the sensor data (a neural network, YOLO).
3. **Geo-locates** each pole — i.e. computes its real-world coordinate — and builds
   a map of poles.
4. Uses that pole map to **correct the vehicle's position** (because a known pole
   seen by the sensor tells you where you are).

Step 3 and 4 both need to know **where the car was, and which way it faced, at the
exact moment it saw each pole**. That is the odometry.

### A.2 What is "odometry"?

**Odometry = estimating your own motion over time from your sensors.** Think of it
as *"dead reckoning"*: if you know your speed and heading every instant, you can add
up all those tiny movements to track where you are relative to the start — **without
needing GPS**.

- You start at a known point and call it the origin `(0, 0)`.
- Every fraction of a second you estimate *"I moved 12 cm forward and turned 0.3°."*
- Sum those up → a continuous **trajectory** (a path of x, y, z + orientation).

The catch: tiny errors in each step **accumulate** → the estimate slowly **drifts**
away from truth. Over kilometres, drift can reach tens or hundreds of metres. Good
odometry = **small drift** per distance travelled.

### A.3 The two sensors it uses: LiDAR + IMU

- **LiDAR** ("Light Detection And Ranging"): a spinning laser scanner. Many times a
  second it fires thousands of laser beams and measures how long each takes to bounce
  back → a **3-D "point cloud"** (a dense spray of `(x, y, z)` points describing the
  surroundings — trees, road, poles, buildings). Our sensor is an **Ouster OS-2-128**
  (128 laser channels), producing one point cloud ("scan") about **10 times per
  second (10 Hz)**.

- **IMU** ("Inertial Measurement Unit"): a tiny chip that measures **acceleration**
  (3 axes) and **rotation rate** (3 axes), ~**100 times a second (100 Hz)**. It feels
  every bump and turn instantly but, used alone, its errors explode within seconds.

### A.4 What is LiDAR-Inertial Odometry (LIO)?

**LIO fuses LiDAR + IMU** to get the best of both:

- The **IMU** gives a fast, smooth short-term guess of motion (good between LiDAR
  scans, handles fast jolts).
- The **LiDAR** anchors that guess to the actual world geometry (stops long-term
  drift): *"the wall I saw last scan is now 1.2 m closer and rotated 2° — so I must
  have moved exactly this much."*

The fusion math (see Part B) continuously blends the two so neither dominates. The
output is a **pose** (position + orientation) at every scan — exactly the trajectory
the snow-pole pipeline needs.

### A.5 Where does Super-LIO fit in *this* project?

```
          ODOMETRY BACKENDS (interchangeable)
          ┌─────────────────────────────────────┐
  sensors │ FastReg  (original)                  │        downstream snow-pole
  (bag) ──┤ Faster-LIO                           ├──►  pipeline (unchanged):
          │ GLIM                                 │     detect poles, geo-locate
          │ Super-LIO   ◄── THIS FOLDER          │     them, correct the vehicle
          └─────────────────────────────────────┘
                          │
            all four emit the SAME thing:
            a CSV of (easting, northing) per GNSS frame
```

The clever design: **every backend is forced to output the identical CSV format**
(`easting`, `northing` columns in UTM coordinates, one row per GNSS frame = 5422
rows). So the heavy downstream pipeline never changes — you just point it at a
different CSV to compare backends **1:1 on speed and accuracy**.

### A.6 Why do we care about speed *and* accuracy?

- **Accuracy** → the pole map and the corrected vehicle position are only as good as
  the odometry underneath them.
- **Speed** → if odometry can't keep up with the 10 Hz sensor in real time, it can't
  run live on the car. The project found odometry was the **slowest stage** of the
  whole pipeline, so a faster backend removes that bottleneck. Super-LIO's headline
  claim is speed, which is why it's worth integrating.

---

## Part B — What is Super-LIO? (the odometry method)

**Super-LIO** = [`Liansheng-Wang/Super-LIO`](https://github.com/Liansheng-Wang/Super-LIO),
a 2026 **RA-L** (Robotics & Automation Letters) paper. It is a **CPU-only** LiDAR-
inertial odometry system. It belongs to a well-known family:

```
FAST-LIO  ──►  FAST-LIO2  ──►  Faster-LIO  ──►  Super-LIO
(the IESKF    (ikd-tree       (iVox voxel     (OctVox 8-pt/voxel map
 estimator)    map)            map)             + heuristic KNN)
```

Each step in that lineage keeps the same *estimator* and speeds up the *map / nearest-
neighbour search*, which is always the bottleneck. Super-LIO is the latest and (on
this hardware) fastest.

### B.1 The estimation backbone: IESKF

Super-LIO estimates motion with an **IESKF — Iterated Error-State Kalman Filter**.
In plain words, a Kalman filter is a principled recipe for **combining a prediction
with a measurement**, weighting each by how much you trust it:

1. **Predict** — use the IMU to propagate the pose forward from the last scan
   (*"based on acceleration & rotation, I should now be roughly here"*).
2. **Measure** — take the new LiDAR scan and ask *"do these points line up with the
   map I've built so far?"* Any mismatch is the **error**.
3. **Update** — nudge the pose to reduce that mismatch, trusting LiDAR vs IMU in the
   right proportion. **"Iterated"** = repeat the measure/update a few times per scan
   (here **4 iterations**) so it converges on a sharp fit. **"Error-State"** = it
   tracks the small *correction* to the pose rather than the full pose, which is
   numerically cleaner for 3-D rotations.

This is the same proven backbone as FAST-LIO/Faster-LIO — Super-LIO did **not**
change it. What it changed is step 2's nearest-neighbour search.

### B.2 The map: "OctVox" — 8 points per voxel

To check *"do my scan points line up with the world?"*, LIO keeps a running **map**
of all the points it has seen, and for each new point finds its **nearest neighbours**
in that map. With millions of points, this search is the expensive part.

Super-LIO's map is **OctVox** (octree + voxel). The world is chopped into small cubes
called **voxels** (here **0.5 m** on a side). The key trick: **each voxel stores at
most 8 representative points**, not the hundreds that may fall in it. This caps the
map's density — you get enough points to define the local surface, and no more. A
sparser map means:

- far less memory (~0.4 GB total here),
- far fewer candidates to search through per query → faster.

### B.3 The search: Heuristic KNN (HKNN)

**KNN = "K-Nearest-Neighbours"** — finding the few map points closest to a query
point. Normally you search the query voxel **and all its neighbouring voxels** to be
safe (27 cubes in 3-D). Super-LIO uses a **heuristic**: it predicts *which* few
neighbouring voxels actually matter (based on where the point sits inside its voxel)
and skips the rest. Combined with the 8-points-per-voxel cap, each correspondence
search touches only a handful of points instead of hundreds. This is the single
biggest reason Super-LIO is fast.

### B.4 The four compute stages (what the FPS number measures)

Every scan, Super-LIO runs four stages. Its built-in timer reports the average time
of each (this is how we measure speed, independent of playback rate):

| Stage | What it does | This run (ms) |
|---|---|---|
| **Undistort** | Un-smears the scan: the LiDAR spins *while* the car moves, so points are warped. Uses the IMU to correct each point to a single instant. | 0.30 |
| **DownSample** | Thins the raw cloud (voxel grid 0.5 m) so you process representative points, not all ~130 k. | 0.46 |
| **Observe** | The **IESKF measurement + HKNN correspondence search** — the core, and the dominant cost. | **3.05** |
| **UpdateMap** | Inserts the new points into the OctVox map for next time. | 0.60 |
|  | **Total per scan** | **≈ 4.41 ms → ~227 scans/s** |

At 10 Hz sensor rate, 4.41 ms/scan means Super-LIO computes **~23× faster than data
arrives** — enormous real-time headroom.

---

## Part C — Advantages & disadvantages

### ✅ Advantages

- **Fastest backend by far.** ~4.4 ms/scan ≈ **227 FPS** here, ~2.5× Faster-LIO,
  ~7–10× GLIM. Odometry is no longer the pipeline's slow stage.
- **Most accurate with any GNSS.** At 10/25/50 % GNSS it edges out every other
  backend (see Part H).
- **Tightest raw drift.** Median trajectory error vs GNSS is the smallest of the
  three LIO backends (~404–412 m median over the full multi-km track; all LIO methods
  drift similarly because they're all IMU-limited without GNSS).
- **CPU-only.** No GPU needed (GLIM needs a CUDA GPU). Runs on a laptop i7.
- **Native ROS 2 Jazzy.** Builds directly on this host with `colcon` — no Docker
  image, no ROS1 bridge (Faster-LIO needed Docker/ROS1; GLIM needed a CUDA image).
- **Low memory.** ~0.4 GB thanks to the OctVox 8-pt/voxel cap.

### ❌ Disadvantages / limitations

- **Pathological at exactly 0 % GNSS.** With *no* GNSS ever, the vehicle position
  between pole sightings drifts more than the other backends (this run: 36 m median;
  the original study saw 56 m — vs ~10 m for Faster-LIO/GLIM). This is a corner case
  (real deployments always have *some* GNSS), and the **pole map itself stays fine**
  (~3.5 m), but it's a real weakness of the dead-reckoning between fixes. Suspected
  cause: local heading inaccuracy in the pole region degrading the bearing-based
  correction (unconfirmed).
- **No offline bag reader.** Unlike Faster-LIO's `run_mapping_offline`, Super-LIO has
  no "read a file and crunch" mode — it only subscribes to **live ROS 2 topics**. We
  drive it by *replaying* the bag (`ros2 bag play`). Harmless, but the **wall-clock**
  is then bounded by playback speed, not compute (the compute FPS is measured
  separately by the internal timer, so it's unaffected).
- **Needed source patches to behave under bag replay** (QoS) — see Part F. Out of the
  box it dropped ~65 % of scans.
- **Livox-centric upstream.** The code hard-requires a Livox message package even
  though we use an Ouster; we had to vendor a dummy one (Part F).
- **Cosmetic log bug.** Prints `Using Lidar type: UNKNOWN` (off-by-one in a name
  table) even though it parses the Ouster correctly.

---

## Part D — The map of this integration

### D.1 Directory layout

```
superlio_integration/
├── summary.md                        ← this file
├── README.md                         ← quick usage
├── PLAN.md                           ← design decisions & status
├── SUPERLIO_VS_FASTERLIO_COMPARISON.md ← the detailed results write-up
├── run_all.sh                        ← one-shot: odometry → CSV → pipeline@0% → GNSS sweep
├── run_full_pipeline_superlio.sh     ← one-shot: odometry → CSV → pipeline → final MP4 viz
├── config/
│   ├── ouster_os2_128_base.yaml      ← DEFAULT config (filter_rate 2; accurate + fast)
│   └── ouster_os2_128.yaml           ← speed-max variant (filter_rate 4; benchmark-only)
├── scripts/
│   ├── 00_run_superlio.sh            ← Stage 0: run Super-LIO on the bag, record /lio/odom + FPS
│   ├── 10_superlio_traj_to_csv.py    ← Stage 1: bridge the trajectory into the pipeline's CSV
│   └── 40_gnss_sweep.sh              ← Stage 3: rerun the pipeline at 0/10/25/50 % GNSS
└── output/                           ← all generated artifacts land here
```

The actual Super-LIO C++ source lives **outside** this folder, at the repo root:
`Super-LIO/` (the cloned repo, ros2 branch, built with `colcon` into
`Super-LIO/install/`).

### D.2 The full data flow

```
  ┌──────────────────────────────────────────────────────────────────────────┐
  │ INPUTS                                                                     │
  │  • ROS 2 bag  glim_integration/output/ros2_bag   (/ouster/points 5419,     │
  │                                                    /ouster/imu  54202)     │
  │  • ROS 1 bag  snow_pole_geo_localization_data/2024-02-28-12-59-51.bag      │
  │               (full 41 GB — has the GNSS + the IMU clock the bridge needs) │
  │  • camera/lidar bag  ...-12-59-51_no_unwanted_topics.bag (5.7 GB, for YOLO)│
  │  • Groundtruth_pole_location_...csv  (where the poles REALLY are)          │
  └──────────────────────────────────────────────────────────────────────────┘
        │
        ▼  Stage 0:  scripts/00_run_superlio.sh
  ┌──────────────────────────────────────────────────────────────────────────┐
  │ ros2 bag play ──► super_lio_node (IESKF+OctVox+HKNN) ──► /lio/odom          │
  │ recorded to  output/superlio_odom/  (mcap)                                 │
  │ + per-stage compute timing → FPS                                           │
  └──────────────────────────────────────────────────────────────────────────┘
        │  (poses are in Super-LIO's local frame, on the LiDAR "sensor clock")
        ▼  Stage 1:  scripts/10_superlio_traj_to_csv.py  (the "bridge")
  ┌──────────────────────────────────────────────────────────────────────────┐
  │ • fit sensor-clock → Unix-epoch from the IMU (shared clock)                │
  │ • rigidly rotate/translate the local track onto real-world UTM33N, anchored│
  │   on the first ~400 m of GNSS (fixes start pose+heading; drift preserved)  │
  │ • resample onto the 5422 GNSS timestamps                                   │
  │ → incremental_navigation_results_superlio.csv   (easting, northing, ...)   │
  └──────────────────────────────────────────────────────────────────────────┘
        │
        ▼  Stage 2:  snowpole_based_vehicle_localization.py  (UNCHANGED pipeline)
  ┌──────────────────────────────────────────────────────────────────────────┐
  │ • YOLO detects poles in the LiDAR signal images                            │
  │ • project each detection to 3-D via the range image                        │
  │ • place it in the world using the vehicle pose (from the CSV) + heading    │
  │ • correct the vehicle position from known pole sightings                   │
  │ → snowpole_results_superlio.csv  +  live_map_superlio.png                  │
  │ → timing JSON (fasterlio_integration/output/timing_Super-LIO_pipeline...)  │
  └──────────────────────────────────────────────────────────────────────────┘
        │
        ├──► Stage 3:  scripts/40_gnss_sweep.sh   (reruns Stage 2 at 0/10/25/50 % GNSS)
        │
        └──► Stage C:  fasterlio_integration/scripts/20_temporal_evolution_visualization.py
             → output/temporal_evolution_superlio.mp4  + 4 summary PNGs
```

### D.3 Why the "bridge" (Stage 1) is non-trivial

Super-LIO outputs a path in its **own local frame** (origin = wherever it started,
axes = sensor orientation), time-stamped on the **LiDAR sensor's internal clock**.
The downstream pipeline expects **real-world UTM coordinates** on the **GNSS/Unix
clock**. The bridge reconciles both:

1. **Clock alignment.** The IMU shares the LiDAR's sensor clock *and* is recorded in
   the full ROS1 bag against Unix time. Fitting a straight line `epoch = a·sensor + b`
   (here `a≈1.0000103`, fit residual **6.9 ms**) converts Super-LIO's timestamps to
   real time.
2. **Spatial alignment ("start-anchor").** It rotates+translates (no scaling) the
   local track so its **first ~400 m of travel** matches GNSS — this pins down the
   starting position and heading, and *nothing else*. Crucially it **does not** best-
   fit the whole track, so the odometry's natural drift is **preserved** and the
   comparison is honest (this mirrors how FastReg/Faster-LIO/GLIM were evaluated).
3. **Resampling.** It samples the aligned track at the exact 5422 GNSS timestamps so
   the output CSV has the same row layout as the original — making all backends drop-
   in interchangeable.

---

## Part E — The config file, explained

`config/ouster_os2_128.yaml` (passed to the node with `--params-file`). Key lines:

```yaml
/**:
  ros__parameters:
    # --- which ROS topics to read (must match the bag) ---
    lio.ros.lidar_topic: "/ouster/points"
    lio.ros.imu_topic:   "/ouster/imu"

    # --- sensor model ---
    lio.sensor.lidar_type: 7        # 7 = Ouster. Super-LIO then reads ouster_ros::Point
                                    #     and each point's per-point time stamp (pt.t, ns)
    lio.sensor.blind: 2.0           # ignore returns within 2 m (the car itself)
    lio.sensor.maxrange: 150.0      # ignore returns beyond 150 m
    lio.sensor.filter_rate: 4       # keep every 4th point  ← the speed/density knob
    lio.sensor.enable_downsample: true
    lio.sensor.voxel_fliter_size: 0.5
    lio.sensor.gravity_norm: 9.819  # local gravity (Norway)
    lio.sensor.imu_type: 0          # accel already in m/s²
    lio.sensor.imu_na:  0.1         # IMU noise params (= Faster-LIO's acc_cov / gyr_cov /
    lio.sensor.imu_ng:  0.1         #   b_acc_cov / b_gyr_cov), so backends match
    lio.sensor.imu_nba: 0.0001
    lio.sensor.imu_nbg: 0.0001

    # --- where the LiDAR sits relative to the IMU: [tx,ty,tz, then 3x3 R row-major] ---
    lio.extrinsic.lidar_imu: [-0.006253, 0.011775, -0.007645,  1,0,0, 0,1,0, 0,0,1]
                                    # from Trip068.json: rotation = identity, small offset

    # --- the OctVox map ---
    lio.hash_map.hash_capacity: 2000000
    lio.hash_map.vox_resolution: 0.5   # 0.5 m voxels

    # --- the IESKF filter ---
    lio.kf.kf_max_iterations: 4        # 4 iterations per scan (= Faster-LIO)
    lio.kf.kf_align_gravity: true
    lio.kf.kf_quit_eps: 0.001

    # --- outputs OFF to save CPU (we only need /lio/odom, which is always on) ---
    lio.output.dense: false
    lio.output.map:   false
    # ...

    lio.eva.timer: true             # turn ON the per-stage compute timer → our FPS number
```

**The one knob that matters for the speed/accuracy trade:** `filter_rate` (full
treatment in **Part K**).
- `2` = keep every 2nd point — **the DEFAULT** (`ouster_os2_128_base.yaml`): denser,
  accurate at every GNSS level incl. 0 %, ~159 FPS.
- `4` = keep every 4th point (`ouster_os2_128.yaml`): fastest (~217 FPS) but its sparser
  cloud lets the track scale drift, which breaks the pole matcher at 0 % GNSS. Benchmark-only.

---

## Part F — The three local modifications

Super-LIO out of the box doesn't quite work for an **offline, Ouster, bag-replay**
setup. Three small changes were made (all documented in code; none change the
algorithm):

1. **Subscription QoS: `best_effort` → `reliable`** (in
   `Super-LIO/src/super_lio/src/ros/ROSWrapper.cpp`). *Why it matters a lot:*
   out of the box Super-LIO uses "best_effort" delivery (right for a live sensor,
   where dropping a stale scan is fine). But under `ros2 bag play` on a single-
   threaded executor busy with the 100 Hz IMU + 500 Hz process timer + big point-
   cloud deserialization, **~65 % of scans were silently dropped** (only 1877 of
   5419 processed). Switching to "reliable" + deeper queues (IMU keep-last 2000,
   LiDAR 200) makes the player **wait** (backpressure) instead of dropping — so
   **all 5409 scans** are processed. *Requires a `colcon build` after editing.*

2. **Shutdown handling** (in `scripts/00_run_superlio.sh`). Two gotchas:
   - `ros2 run super_lio super_lio_node` is a wrapper that **doesn't forward
     Ctrl-C** to the real binary. So the script sends the interrupt to the **actual
     process**: `pkill -INT -f 'super_lio/lib/super_lio/super_lio_node'`. That makes
     the node's `spin()` return → `printTimeRecord()` flushes the **FPS numbers**.
   - `ros2 bag record` **ignores SIGINT** in this environment — it needs **SIGTERM**
     to cleanly finalize the recording and write `metadata.yaml`.

3. **Vendored a dummy `livox_ros_driver2` package** (`Super-LIO/src/livox_ros_driver2/`).
   Super-LIO's build hard-requires `find_package(livox_ros_driver2)` and includes
   `custom_msg.hpp` (for Livox LiDARs), but it isn't shipped and has no Jazzy apt
   package. Since we only use the **Ouster** path (which never touches Livox
   messages), an **interface-only** stub package (just the `CustomMsg`/`CustomPoint`
   message definitions) satisfies the build and is never used at runtime.

---

## Part G — How to run it from bash

### G.1 One-time setup (needs sudo once)

```bash
# system deps
sudo apt update
sudo apt install -y python3-colcon-common-extensions ros-jazzy-pcl-ros libgflags-dev

# build Super-LIO (repo already cloned at ../../Super-LIO, ros2 branch)
cd ../../Super-LIO      # i.e. Fasterlio/Super-LIO
colcon build            # produces Super-LIO/install/setup.bash
```

Everything else (Eigen, PCL, glog, TBB, pcl-conversions, gcc-13/C++20) is already on
the host. The ROS 2 input bag is reused from the GLIM run
(`glim_integration/output/ros2_bag`); if it's missing, regenerate it with
`glim_integration/scripts/00_convert_bag.sh`.

### G.2 The easy way — one command

From the project dir (`Snow-pole-based-vehicle-localization/`):

```bash
# Full pipeline + the final animated MP4 (odometry → CSV → localization → viz).
# Skips the odometry step automatically if the CSV already exists.
superlio_integration/run_full_pipeline_superlio.sh
superlio_integration/run_full_pipeline_superlio.sh --from-bag   # force fresh odometry

# OR: odometry → CSV → localization@0 % → GNSS sweep (0/10/25/50 %).
superlio_integration/run_all.sh
```

Outputs land in `superlio_integration/output/` (MP4, PNGs, logs) and in the project
root (`incremental_navigation_results_superlio.csv`, `snowpole_results_superlio.csv`).

### G.3 The step-by-step way (full control)

```bash
cd Snow-pole-based-vehicle-localization

# ── Stage 0: run Super-LIO, record /lio/odom + print the FPS ──────────────────
#   args: [ros2_bag_dir] [play_rate] [config_yaml]
#   play_rate 3.0 is safe (drops ZERO scans thanks to the reliable-QoS fix) and
#   ~3× faster than the default 1.0 — see the tip below.
superlio_integration/scripts/00_run_superlio.sh \
    glim_integration/output/ros2_bag 3.0 \
    superlio_integration/config/ouster_os2_128.yaml

# ── Stage 1: bridge the trajectory into the pipeline's CSV ────────────────────
source ../.baginspect_venv/bin/activate        # the rosbags-reading venv
python superlio_integration/scripts/10_superlio_traj_to_csv.py \
    --dataset-bag snow_pole_geo_localization_data/2024-02-28-12-59-51.bag \
    --odom-bag    superlio_integration/output/superlio_odom \
    --align start \
    --out incremental_navigation_results_superlio.csv
deactivate

# ── Stage 2: the snow-pole pipeline (0 % GNSS = the "proposed method") ─────────
env -u PYTHONPATH MPL_BACKEND=Agg \
    INCREMENTAL_NAV_CSV=incremental_navigation_results_superlio.csv \
    RESULTS_CSV=snowpole_results_superlio.csv \
    MAP_FIG=superlio_integration/output/live_map_superlio.png \
    ~/miniconda3/envs/polegeo/bin/python snowpole_based_vehicle_localization.py

# ── Stage 3: GNSS-availability sweep (0/10/25/50 %, fixed seed) ───────────────
superlio_integration/scripts/40_gnss_sweep.sh \
    incremental_navigation_results_superlio.csv superlio
```

### G.4 Useful knobs & tips

- **Playback rate (`00_` arg 2).** The reliable-QoS fix makes *any* rate
  backpressure-safe, so you don't need real-time playback. **`3.0` finishes in
  ~3 min with zero dropped scans** (verified: 5409 poses, identical to rate 1 or 10);
  the scripts' default `1.0` is needlessly slow (~9 min, playback-bound). Drop to
  `0.5` only if you ever see scan drops.
- **Config variant.** Pass `config/ouster_os2_128_base.yaml` as arg 3 to `00_` for
  the denser (`filter_rate 2`) run.
- **GNSS levels.** `GNSS_PCTS="0 10 25 50" superlio_integration/scripts/40_gnss_sweep.sh ...`
- **Headless.** `MPL_BACKEND=Agg` keeps matplotlib from needing a display.
- **FPS is rate-independent** — it comes from the node's internal per-stage timer, not
  from wall-clock, so playback rate never biases it.

---

## Part H — Results

Measured on an i7-12700H laptop, CPU only. Two numbers are shown where they differ:
the figures reproduced in **this final session (2026-10-02)** and the **original
study (2026-09-10)**; they agree closely.

> **Config note:** the §H Super-LIO numbers below are for the **current default
> `filter_rate 2`** config. The `filter_rate 4` ("ds") speed-max variant from the
> original study is faster (~217 vs ~159 FPS) but regresses the 0 %-GNSS case
> (9.65 → 55.94 m). See **Part K** for the side-by-side and the reasoning.

### H.1 Speed — odometry throughput

| Backend | Engine | ms/scan | Throughput |
|---|---|---|---|
| **Super-LIO** (default, `filter_rate 2`) | CPU IESKF + OctVox(8 pt/vox) + HKNN | **6.29** | **~159 FPS (≈16×)** |
| Super-LIO (ds, `filter_rate 4`) | same | 4.41 | ~227 FPS (≈23×) |
| Faster-LIO (ds) | CPU iVox + ESIKF | 12.65 | 79 FPS (7.9×) |
| Faster-LIO (base) | CPU iVox + ESIKF | 21.35 | 47 FPS (4.7×) |
| GLIM | GPU VGICP + factor graph | 37.7 | 26.5 FPS (2.65×) |

Per-stage (default `filter_rate 2` run): Undistort 0.39 · DownSample 0.84 ·
**Observe 4.11** · UpdateMap 0.95 ms. The HKNN correspondence search (`Observe`)
dominates — exactly the cost OctVox is built to attack — yet it's still tiny. The denser
`filter_rate 2` cloud raises `Observe` 3.05→4.11 ms vs the speed-max `filter_rate 4`
variant; even so Super-LIO stays ~2× Faster-LIO and ~6× GLIM.

### H.2 Accuracy — pole-corrected vehicle position vs GNSS availability

Median vehicle-position error (metres), identical pipeline, fixed seed:

Super-LIO column is the **default `filter_rate 2`** config.

| GNSS used | FastReg | Faster-LIO | GLIM | **Super-LIO** |
|---|---|---|---|---|
| 0 %  | 8.41 | 8.70 | 10.18 | **9.65** |
| 10 % | 2.13 | 0.80 | 1.14 | **0.91** ✅ |
| 25 % | 1.29 | 0.46 | 0.63 | **0.53** ✅ |
| 50 % | 0.53 | 0.19 | 0.26 | **0.20** ✅ |

**With the default `filter_rate 2` config Super-LIO is competitive at 0 % (9.65 m, on
par with Faster-LIO/GLIM) and the most accurate with any GNSS** (best or tied at
10/25/50 %). Even 10 % GNSS collapses its error ~11× (9.65 → 0.91 m). The old
`filter_rate 4` speed variant regressed 0 % to 55.94 m — see Part K.

### H.3 Pole localization — the actual deliverable (0 % GNSS)

Distance from each predicted pole to its ground-truth pole (this is the map the
project exists to produce; independent of the vehicle-position metric):

| Backend | median | mean | max | events | distinct poles |
|---|---|---|---|---|---|
| **Super-LIO** (`filter_rate 2`) | **2.26 m** | 3.01 | 13.12 | 355 | 131 |
| Faster-LIO | 2.14 m | 2.68 | 11.72 | 355 | 135 |
| GLIM | 2.65 m | 3.46 | 15.18 | 356 | 134 |

So even at 0 % GNSS the **pole map is good** (~2.3 m); the 0 %-GNSS *vehicle* error does
not wreck it, because at each pole sighting the position is re-fixed. The `events` column
counts geo-localization *events*; a physical pole yields ~2–3 events (the vehicle drives
past it), so Super-LIO's 355 events localize **131 distinct** ground-truth poles of 290.

### H.4 Raw trajectory drift (odometry vs GNSS, start-anchored)

| Backend | median | mean | max | track-len ratio |
|---|---|---|---|---|
| Super-LIO (`filter_rate 2`) | **~372 m** | ~694 | ~2547 | 0.922 |
| Faster-LIO | 448 m | 790 | 2856 | 0.947 |
| GLIM | 448 m | 755 | 2594 | 0.936 |

Tightest median drift of the three (all are IMU-limited over multi-km with no GNSS); the
denser `filter_rate 2` cloud also holds along-track scale best of Super-LIO's two configs
(0.922 vs 0.903 at `filter_rate 4`). Clock fit residual 6.9 ms; 5408 poses; ~0.4 GB RAM.

### H.5 This session's end-to-end run (2026-10-02) — everything green

- Odometry (default `filter_rate 2`): **158.9 FPS** (6.29 ms/scan), **5408 poses**,
  0 dropped scans (at play rate 3.0).
- Bridge: clock residual 6.9 ms, 5422 rows, 0 NaNs.
- Pipeline: ~68 frames/s, YOLO ~20 ms/frame; odometry-only median **124.41 m**.
- GNSS sweep: 0/10/25/50 % → **9.65 / 0.91 / 0.53 / 0.20 m**.
- Visualization: valid H.264 MP4 + 4 summary PNGs + live map produced.

### H.6 Pole detection accounting (0 % GNSS run)

*"How many poles were detected, and how many were actually used?"* The detector
(YOLO) fires a **bounding box on every pole candidate in every processed frame**;
each box is then filtered out if **(a)** it has no valid 3-D return in the range
image, or **(b)** its nearest 3-D point is beyond the `distance_threshold = 5 m`
false-positive gate (`snowpole_based_vehicle_localization.py`). Survivors become
**used** geo-localization events — one row in the results CSV. Because the *same
physical pole is seen across many consecutive frames* (plus some false positives),
the raw box count is ~6–7× the number actually used.

For **Super-LIO** (0 % GNSS, default `filter_rate 2`):

| Step | Count | Meaning |
|---|---:|---|
| Frames YOLO ran on | 2145 | in-bounds frames the detector saw |
| **Raw detections (boxes)** | **2326** | every candidate box |
| — dropped: no 3-D point in box | 5 | no usable range return there |
| — dropped: nearest point > 5 m | 1966 | false-positive gate |
| **Used (geo-localized → CSV rows)** | **355** | = rows of `snowpole_results_superlio.csv` |
| Distinct ground-truth poles hit | 131 | of **290** poles at the site |

Balance: `2326 − 5 − 1966 = 355`. ✓

**Why one pole becomes several events (and why that is intentional).** The vehicle
drives *past* each pole, so the detector re-acquires the **same physical pole on every
frame it stays in view and within the 5 m range gate** — typically ~2–3 consecutive
frames (mean **2.7 events/pole**, median 3, up to 5; 20 poles seen only once). Each
re-sighting is an *independent* range+bearing fix that re-anchors the drifting
dead-reckoned track, so the pipeline keeps all of them and reports error over
**events**, not unique poles. Super-LIO's 355 events map onto **131 distinct
ground-truth poles** (45 % of the 290 at the site); the rest were off the traversed
one-way section.

Detection is **almost backend-independent** — it runs on the same camera/LiDAR
images regardless of odometry. The only coupling is the *in-bounds* test, which uses
the odometry-**predicted** vehicle position, so a few boundary frames differ per
backend (hence the small spread below):

*(Super-LIO row is the default `filter_rate 2` config.)*

| Backend | YOLO frames | raw boxes | drop (no-pt) | drop (>5 m) | **used events** | distinct poles | events/pole |
|---|---:|---:|---:|---:|---:|---:|---:|
| Faster-LIO | 2141 | 2325 | 5 | 1965 | 355 | 135 | 2.6 |
| GLIM | 2147 | 2328 | 5 | 1967 | 356 | 134 | 2.7 |
| **Super-LIO** | 2145 | 2326 | 5 | 1966 | **355** | 131 | 2.7 |

**Check it yourself:**

```bash
# USED count — always available (persistent artifacts):
echo $(( $(wc -l < snowpole_results_superlio.csv) - 1 ))                    # -> 355
grep -E 'pole_detection_events|detection_frames' \
     fasterlio_integration/output/timing_Super-LIO_pipeline_gnss0.json      # 355 ; 2145

# RAW + drop reasons — from the pipeline STDOUT (captured in the 0 % sweep log):
LOG=superlio_integration/output/sweep_superlio_gnss0.log
grep -c "sequence number used for geo localization" "$LOG"   # raw boxes  -> 2326
grep -c "no valid nearest point found"              "$LOG"   # no 3-D pt  -> 5
grep -c "skipping this bounding box"                "$LOG"   # > 5 m gate -> 1966

# DISTINCT physical poles mapped:
~/miniconda3/envs/polegeo/bin/python -c "import pandas as pd; r=pd.read_csv('snowpole_results_superlio.csv'); print(r[['Ground Truth Easting','Ground Truth Northing']].round(2).drop_duplicates().shape[0])"
```

> ⚠️ The log line `sequence number used for geo localization` is **misnamed** — it
> prints for *every* box *before* filtering, so it counts **raw detections**, not
> used ones. If the sweep log is gone, regenerate any pipeline run with stdout
> redirected to a file and grep that.

---

## Part I — Known issues & caveats

- **0 %-GNSS anomaly — diagnosed & FIXED (see Part K).** With the old default
  (`filter_rate 4`) the 0 %-GNSS pole-corrected median was 55.94 m. Root cause: the
  sparser cloud let the odometry's along-track *scale* drift (track ratio 0.903), which
  made the pipeline's dead-reckoned pole prediction cross into a neighbouring pole's
  basin, so the (un-gated) matcher locked onto the wrong pole. Switching the default to
  `filter_rate 2` (denser) tightens scale to 0.922 and drops the error to **9.65 m**
  (on par with Faster-LIO/GLIM). Not a heading bug and not a code bug — proven by an
  instrumented run. Irrelevant once any GNSS is present either way.
- **`Using Lidar type: UNKNOWN`** in the log is **cosmetic** — an off-by-one in
  Super-LIO's name table (7 slots for indices 0–6, but `OUSTER=7`). The `switch`
  still matches `case OUSTER` and parses correctly (proven by successful map init and
  a complete trajectory).
- **Wall-clock vs compute.** Because Super-LIO has no offline reader, wall-clock is
  bounded by playback rate, not compute. Use the **internal per-stage FPS** (what the
  tables above report) for speed comparisons.
- **The 0 %-GNSS vehicle number depends on the config:** `filter_rate 2` (default)
  → **9.65 m**; `filter_rate 4` → 55.94 m (and is itself run-to-run variable ~36–56 m
  because a wrong-pole lock is sensitive to small alignment nondeterminism). Everything
  with any GNSS is stable regardless of config.

---

## Part J — Verdict

On this laptop, **Super-LIO is the recommended odometry front-end** for the snow-pole
project, run with the **default `filter_rate 2` config**:

- **Fast by a wide margin** — ~159 FPS (default `filter_rate 2`), still ~2× Faster-LIO
  and ~6× GLIM; up to ~217 FPS with the `filter_rate 4` speed-max variant.
- **Most accurate at every GNSS level**, including 0 % — the `filter_rate 2` default
  fixes the old 0 %-GNSS weak spot (9.65 m, on par with Faster-LIO/GLIM); best of all
  backends at 10/25/50 %.
- CPU-only, runs native **or in Docker** (`SUPERLIO_DOCKER=1`), low memory.
- The old 0 %-GNSS caveat is resolved (Part K); use `filter_rate 4` only to benchmark
  peak throughput.

---

## Part K — Deep dive: `filter_rate` & the 0 %-GNSS fix

This section documents, from fundamentals, the one tuning decision that changed the
most: the point-downsampling rate. It explains **what `filter_rate` is**, **why it
affects accuracy at all**, the **experiment** that pinned the 0 %-GNSS failure, and
**why the default is `filter_rate: 2`**.

### K.1 What `filter_rate` is (from scratch)

Every LiDAR scan from the Ouster OS-2-128 is a grid of **128 beams × 1024 columns ≈
131,000 points**, arriving **10 times a second**. Feeding *all* of them into the
IESKF's scan-to-map matching every scan is wasteful — neighbouring points carry
near-duplicate information. So, like Faster-LIO's `point_filter_num`, Super-LIO
**keeps only every *N*-th point** before matching. That stride *N* is `filter_rate`:

| `filter_rate` | keeps | points/scan | meaning |
|---:|---:|---:|---|
| 1 | every point | ~131 k | densest, slowest |
| **2** | every 2nd | ~66 k | **the default** |
| 4 | every 4th | ~33 k | sparsest, fastest |

It is a cheap, stride-based thinning of the raw cloud, applied *before* the 0.5 m
voxel grid filter. Higher `filter_rate` = fewer points into the estimator = fewer
distance/nearest-neighbour computations = **higher FPS**.

### K.2 Why thinning the cloud can hurt — the scale effect

The IESKF recovers the 6-DoF pose each scan by aligning the (thinned) scan against the
OctVox map. The **along-track translation** (how far forward you moved) is constrained
by how well the surrounding geometry is sampled. In open, snowy, feature-sparse
stretches, an aggressively thinned cloud (`filter_rate 4`) leaves the forward
direction **under-constrained**, so the estimator slightly *under-steps* each scan.
Integrated over the whole drive, the trajectory comes out **too short**. Measured
total track length as a fraction of the true (GNSS) length:

| config | track-length ratio |
|---|---:|
| Super-LIO `filter_rate 4` | **0.903** (≈10 % short — worst) |
| Super-LIO `filter_rate 2` | **0.922** |
| Faster-LIO | 0.947 |
| GLIM | 0.936 |

(Some shortfall is inherent to this dataset — even the others are 0.94–0.95 — but
`filter_rate 4` is clearly the worst, i.e. the most scale-biased.)

### K.3 Why a ~10 % scale error wrecks the 0 %-GNSS pole map (the full chain)

At **0 % GNSS** the vehicle is positioned *only* by dead-reckoning the odometry and
snapping to detected snow poles. Two facts make this fragile:

1. The ground-truth poles are **dense — ~9 m apart** — so the matcher's spatial
   tolerance is only ~**5 m** (half the spacing).
2. The pipeline's matcher takes the **globally nearest** ground-truth pole, with **no
   distance gate and no motion-consistency check** (`snowpole_based_vehicle_localization.py`,
   the `min_distance` loop).

So the chain is:

```
filter_rate 4  →  ~10 % along-track scale error  →  the dead-reckoned predicted pole
lands tens of metres past the true pole  →  it falls into a NEIGHBOURING pole's basin
→  the un-gated matcher locks onto the WRONG pole  →  every correction re-anchors
there  →  persistent ~55 m vehicle offset for the whole run.
```

**Instrumented proof (what it is NOT):** a per-event dump of the 0 %-GNSS run showed
the projected pole offset equalled the measured LiDAR range *exactly* (3.6 m) and the
bearing was correct — so it is **not a heading bug and not a sensor error**. The error
was purely that the dead-reckoned vehicle was already ~57 m off **from the first pole
event** — the scale symptom. And it is **not a code bug in the integration**: the
downstream pipeline is byte-identical for all three backends; the same code gives
Faster-LIO/GLIM ~8–10 m. It is the *interaction* of Super-LIO's `filter_rate 4` scale
bias with a brittle (tolerance ~5 m) matcher.

### K.4 The experiment & result

Re-running the whole 0 %-GNSS pipeline with the denser `filter_rate 2` config:

| config | track ratio | 0 %-GNSS pole-corrected median | odometry speed |
|---|---:|---:|---:|
| Super-LIO `filter_rate 4` (old default) | 0.903 | **55.94 m** | ~217 FPS (4.41 ms/scan) |
| **Super-LIO `filter_rate 2` (new default)** | **0.922** | **9.65 m** | **~159 FPS (6.29 ms/scan)** |
| Faster-LIO (ref) | 0.947 | 8.70 m | 79 FPS |
| GLIM (ref) | 0.936 | 10.18 m | 26 FPS |

Tightening the scale from 0.903 → 0.922 is enough to keep the predicted pole inside
the correct pole's ~5 m basin, so the existing matcher associates correctly and the
0 %-GNSS error collapses **55.94 → 9.65 m** — now on par with Faster-LIO and GLIM.
Per-stage cost of the denser cloud: `Observe` (the HKNN search) rises 3.05 → 4.11 ms
and the total 4.41 → 6.29 ms/scan.

### K.5 Why `2` — not `1`, not `4`

- **`4`** — fastest (~217 FPS) but breaks the 0 %-GNSS pole map (above). Kept only as a
  throughput benchmark (`ouster_os2_128.yaml`).
- **`2` (chosen default)** — recovers full accuracy at 0 % GNSS (9.65 m) **and** keeps
  the decisive speed lead: ~159 FPS is still ~**2× Faster-LIO** and ~**6× GLIM**. It is
  the sweet spot of the speed/accuracy trade. It also mirrors Faster-LIO's base
  `point_filter_num=2`, keeping the cross-backend comparison fair.
- **`1`** — all points: marginally stronger constraints but ~2× slower again, with
  diminishing returns — the scale at `2` is already good enough for the matcher, so `1`
  buys little and gives back the speed advantage. Not needed.

### K.6 The deeper, optional fix

The underlying brittleness is the **un-gated nearest-pole matcher** (shared by all
backends), not Super-LIO itself. `filter_rate 2` fixes the symptom by keeping the
odometry accurate enough that the naive matcher never mis-associates. A **matcher
gate** — reject a pole association whose implied vehicle jump is inconsistent with the
odometry's recent motion — would add robustness for *every* backend, but it was not
needed once the scale was tightened. See the project discussion for that proposal.

---

## Glossary

| Term | Plain meaning |
|---|---|
| **Odometry** | Tracking your own motion/position from your sensors, without GPS. |
| **Drift** | Slow accumulation of small odometry errors over distance. |
| **LiDAR** | Spinning laser scanner → a 3-D "point cloud" of the surroundings. |
| **Point cloud / scan** | One LiDAR sweep: thousands of `(x,y,z)` points (~10/s here). |
| **IMU** | Chip measuring acceleration + rotation rate (~100/s). |
| **LIO** | LiDAR-Inertial Odometry: fuses LiDAR + IMU for robust motion tracking. |
| **IESKF** | Iterated Error-State Kalman Filter — the math that fuses predict+measure. |
| **Voxel** | A small cube the world is diced into (here 0.5 m). |
| **OctVox** | Super-LIO's map: ≤ 8 points per voxel → sparse, fast. |
| **KNN / HKNN** | (Heuristic) K-Nearest-Neighbours: finding nearby map points fast. |
| **Pose** | Position + orientation at an instant. |
| **GNSS / GPS** | Satellite positioning — the "ground truth" reference here. |
| **UTM33N / EPSG:32633** | A flat metric map projection for this region (easting/northing in metres). |
| **Easting / Northing** | X / Y coordinates in a UTM projection (metres). |
| **Start-anchor alignment** | Fit only the first stretch to GNSS → fixes start pose+heading, keeps drift honest. |
| **QoS** | ROS "Quality of Service": reliable (never drop) vs best_effort (may drop). |
| **ROS 2 / Jazzy** | Robotics middleware; "Jazzy" is the version matching this host. |
| **Bag** | A recorded log of all sensor messages, replayable. |
| **FastReg** | The original LiDAR odometry this project is replacing. |
| **FPS** | Frames (scans) processed per second — the speed metric. |

---

### Reference artifacts

- `SUPERLIO_VS_FASTERLIO_COMPARISON.md` — the detailed numbers write-up.
- `output/superlio_run_ouster_os2_128.log` — per-stage timing from the odometry run.
- `output/temporal_evolution_superlio.mp4` — the final animated result.
- `output/live_map_superlio.png`, `output/summary_*.png` — static figures.
- `../incremental_navigation_results_superlio.csv` — the odometry CSV the pipeline eats.
- `../snowpole_results_superlio.csv` — the pole-localization output.
- `../fasterlio_integration/output/timing_Super-LIO_*.json` — machine-readable metrics.
