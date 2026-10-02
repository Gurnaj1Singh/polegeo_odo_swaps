# GLIM integration plan (GPU LiDAR-inertial odometry)

Replace the Faster-LIO odometry front-end with **GLIM** (koide3), a GPU-accelerated
tightly-coupled range-**inertial** SLAM framework, to break the ~1.0× real-time
ceiling Faster-LIO hits on the dense Ouster OS-2-128 cloud — and, as a bonus, to
get GLIM's built-in global optimisation (which can attack the ~450 m yaw drift).

This mirrors `fasterlio_integration/` exactly: run the LIO, dump a trajectory,
bridge it into `incremental_navigation_results_glim.csv` (the file the snow-pole
pipeline already reads via `INCREMENTAL_NAV_CSV`), then reuse the same viz +
head-to-head comparison. **Nothing downstream of the CSV changes.**

---

## 0. Why GLIM, and what actually changes vs Faster-LIO

| | Faster-LIO (current) | GLIM (this plan) |
|---|---|---|
| Registration | CPU iVox + ESIKF | **GPU VGICP** (CUDA) + factor graph |
| Bottleneck on OS-2-128 | NN search over ~131k pts/scan on CPU | offloaded to the RTX 3050 Ti |
| Drift handling | none (pure odometry) | local + **global** optimisation (pose graph) |
| Runtime env | Docker `ros:noetic` (ROS1) | Docker `koide3/glim_ros2:jazzy_cuda13.1` (ROS2) |
| Bag input | plays ROS1 `.bag` natively | needs ROS1→ROS2 bag conversion first |

The speed win is expected to come from moving the dense per-scan matching to the
GPU. **Caveat set by the hardware:** the laptop GPU is an **RTX 3050 Ti with only
4 GB VRAM**, so we run **GPU odometry** (the speed-critical, bounded-memory part)
but keep **global mapping on the CPU pose-graph** backend (bounded VRAM over a
10 km drive). This is a deliberate default, flippable in config (see §4).

---

## 1. Host facts (verified 2026-09-09)

- GPU: **NVIDIA RTX 3050 Ti Laptop, 4 GB**, driver **595.84** → supports the CUDA
  **13.1** container (`jazzy_cuda13.1`). CPU: i7-12700H, 20 threads.
- Host ROS: **ROS 2 Jazzy** → the `jazzy` GLIM image matches natively.
- Docker works **without sudo**. **`nvidia-container-toolkit` is NOT installed** →
  must be added before `--gpus all` works (one-time, sudo — see §3.1).
- Disk: **~61 GB free (/ at 87 %)**. The ROS2 bag conversion needs **~33 GB** and the
  image is **~10–15 GB** → after provisioning ≈ 15 GB free. **Tight.** Reclaim first
  (see §7) and delete the converted bag after the run.
- Dataset bag `2024-02-28-12-59-51.bag` (ROS1, 39 GB, 541.8 s) contains:
  `/ouster/points` (5419 × PointCloud2 128×1024), `/ouster/imu` (54202 × Imu, 100 Hz),
  `/gps_left_position` + `/gps_right_position` (NavSatFix, 5422 each) — all present,
  so the GNSS bridge reads GPS from this same bag exactly like the Faster-LIO one.

---

## 2. Pipeline stages (mirrors fasterlio_integration)

```
                         ROS1 .bag (39 GB)
                                │
   [00] rosbags-convert  ──────▼──────  points+imu only  →  output/ros2_bag/  (~33 GB, scratch)
                                │
   [02] glim_rosbag  ──────────▼──────  koide3/glim_ros2:jazzy_cuda13.1  (--gpus all, headless)
                                │                     dumps /tmp/dump/traj_lidar.txt (TUM)
                                ▼
   [10] glim_traj_to_csv.py  ──────────  reads GLIM TUM + GNSS/IMU-clock from the ORIGINAL bag
                                │          Kabsch start-anchor → UTM33N (EPSG:32633)
                                ▼
        incremental_navigation_results_glim.csv   (easting/northing — pipeline input)
                                │
   [20] snow-pole pipeline / temporal-evolution viz   (unchanged, reused from fasterlio_integration)
```

- **Stage 00 — convert** (`scripts/00_convert_bag.sh`): `rosbags-convert` the full bag
  to a ROS2 bag keeping only `/ouster/points` + `/ouster/imu`. PointCloud2/Imu are
  standard messages, so no custom `.msg` defs are needed. Uses the existing
  repo-root `.baginspect_venv` (already has `rosbags`).
- **Stage 01 — configs** (`scripts/01_extract_and_patch_configs.sh`): copy GLIM's
  default `/glim/config/*` out of the image, then patch only our fields (§4). We do
  **not** hand-write full GLIM configs — we override the real defaults so we can't
  drift from the image's schema.
- **Stage 02 — run** (`scripts/02_run_glim.sh`): `glim_rosbag` in the container with
  `--gpus all`, headless (no viewer). `glim_rosbag` auto-throttles playback to avoid
  dropping scans, and logs per-frame processing time. On close it writes TUM
  trajectories to `/tmp/dump/` → we copy `traj_lidar.txt` to `output/`.
- **Stage 10 — bridge** (`scripts/10_glim_traj_to_csv.py`): adapted 1:1 from the
  Faster-LIO bridge. Same Kabsch **start-anchor** (skip idle, fix pose+heading only,
  preserve drift) → same CSV columns. GLIM's TUM stamps are the Ouster sensor clock,
  so the existing IMU-clock affine (`epoch = a·sensor + b`) maps them to GNSS epoch
  unchanged. A guard re-bases the stamps if GLIM ever emits them relative to 0.
- **Stage 20 — viz/compare**: reuse `fasterlio_integration/scripts/20_temporal_evolution_visualization.py`
  and the head-to-head harness, pointing them at the GLIM CSV.

---

## 3. Environment setup

### 3.1 One-time host prep (sudo) — `docker/setup_nvidia_container_toolkit.sh`
Installs `nvidia-container-toolkit`, wires it as the Docker runtime, restarts Docker,
and smoke-tests `docker run --rm --gpus all <cuda> nvidia-smi`. **Run this yourself**
(needs sudo): `! sudo bash Snow-pole-based-vehicle-localization/glim_integration/docker/setup_nvidia_container_toolkit.sh`

### 3.2 Pull the image — `docker/pull_glim.sh`
`docker pull koide3/glim_ros2:jazzy_cuda13.1` (~10–15 GB, no sudo). CPU fallback:
`koide3/glim_ros2:jazzy` if the toolkit can't be installed (much slower — for
validating the pipeline only, not for the speed benchmark).

### 3.3 Container invocation — `docker/run_glim_docker.sh`
```
docker run --rm --gpus all --net=host --ipc=host --pid=host \
  -e DISPLAY -v <config>:/glim/config -v <repo>:/work -w /work \
  koide3/glim_ros2:jazzy_cuda13.1 <cmd…>
```
Headless by design (viewer modules stripped in config); `-e DISPLAY` + X socket only
mounted if a display exists, so it also runs backgrounded/piped to a log.

---

## 4. GLIM config overrides (`config/overrides.json`, applied by `patch_configs.py`)

Applied on top of the image's real defaults:

- **`config_ros.json`**
  - `points_topic` → `/ouster/points`, `imu_topic` → `/ouster/imu`
  - `acc_scale` → `1.0` (Ouster IMU accel already in m/s², z≈9.94; default 0.0 is wrong for us)
  - `extension_modules` → `[]` (drop `libstandard_viewer.so`/`librviz_viewer.so` → headless)
  - `enable_local_mapping` true, `enable_global_mapping` true
- **`config_sensors.json`**
  - `T_lidar_imu` → `[-0.006253, 0.011775, -0.007645, 0, 0, 0, 1]` (t in m from `Trip068.json`,
    R=I ⇒ quaternion identity). Sub-cm, so sign is negligible; **verify against Trip068.json**
    during bring-up. Confirm GLIM's per-point time handling picks up the Ouster `t` field
    (GLIM auto-detects Ouster; verify the deskew looks right).
- **`config.json`** (module selection)
  - odometry → `config_odometry_gpu.json` (**CUDA VGICP** — the speed source)
  - global mapping → `config_global_mapping_pose_graph.json` (**CPU**, VRAM-safe on 4 GB)
  - To go all-GPU later (if VRAM allows): point global mapping back at the GPU variant
    and/or set `enable_global_mapping=false` for a pure odometry-vs-Faster-LIO comparison.

---

## 5. Benchmark methodology (apples-to-apples with the existing report)

- **Throughput** (`scripts/run_offline_bench.sh`): wall-clock of `glim_rosbag` over the
  541.8 s bag → **real-time factor** = 541.8 / wall. Also parse GLIM's per-frame
  processing-time log. Compare directly against the Faster-LIO numbers in
  `fasterlio_integration/SPEED_AND_RELIABILITY_COMPARISON.md` (Faster-LIO ≈ 1.0× RT).
- **Accuracy**: run the bridge in `--align start` (drift-preserving) and feed the CSV
  through the identical snow-pole pipeline at the same GNSS-% sweep (0/10/25/50 %,
  `GNSS_SEED=0`) used for the Faster-LIO head-to-head. Report pole-corrected
  median/mean/max, exactly as `SPEED_AND_RELIABILITY_COMPARISON.md` does.
- Deliverable: `glim_integration/GLIM_VS_FASTERLIO_COMPARISON.md`.

---

## 6. What to expect (and what won't change)

- **Speed**: GPU VGICP should push well past 1.0× RT on the dense cloud; the exact
  factor depends on how the 4 GB VRAM handles 131k-pt scans (may need the preprocess
  voxel resolution bumped — the GLIM analogue of Faster-LIO's `filter_size_surf`).
- **Drift**: GLIM's global pose-graph can reduce the ~450 m global yaw drift that pure
  Faster-LIO odometry showed — but there are **no loop closures** on a one-way drive,
  so don't expect miracles globally. The pipeline's pole-correction + GNSS anchoring
  still does the heavy lifting; the fair metric remains **pole-corrected** error, not
  raw odometry (same lesson as the Faster-LIO write-up).
- **Local accuracy** (what the pipeline dead-reckons between pole sightings) should be
  at least as good as Faster-LIO; that's what actually matters here.

---

## 7. Risks / mitigations

- **Disk (highest)**: 61 GB free vs ~45 GB of new artifacts. Before converting, reclaim:
  `fasterlio_integration/output/fasterlio_odometry.bag` (1.2 GB) and the multi-GB `*.log`.
  Convert **points+imu only**; **delete `output/ros2_bag/` after** stage 02.
- **VRAM 4 GB**: mitigated by CPU global mapping; if odometry still OOMs, raise the GLIM
  preprocess downsample resolution and/or `voxelmap` resolution.
- **Bag conversion drops/timestamps**: verify converted message count == 5419 points /
  54202 imu and that header stamps are preserved (bridge affine depends on it).
- **CUDA/driver**: driver 595 ≥ CUDA 13.1 min; if the container can't see the GPU,
  it's the toolkit step (§3.1), not the driver.
- **Alternative if disk/toolkit blocks us**: a ROS1-bag→ROS2-topic republisher feeding
  live `glim_rosnode` (no 33 GB bag) — documented in README as a fallback, not the default
  (live topics can drop scans; `glim_rosbag` is the no-drop path).

---

## 8. Provisioning order (checklist)

1. `[sudo]` `docker/setup_nvidia_container_toolkit.sh`  → GPU-in-Docker works
2. `docker/pull_glim.sh`                                → image local
3. reclaim disk (§7), then `scripts/00_convert_bag.sh`  → `output/ros2_bag/`
4. `scripts/01_extract_and_patch_configs.sh`            → `config/*.json` ready
5. `scripts/02_run_glim.sh`                             → `output/glim_traj_lidar.txt`
6. `scripts/10_glim_traj_to_csv.py`                     → `incremental_navigation_results_glim.csv`
7. run the pipeline with `INCREMENTAL_NAV_CSV=…_glim.csv` + the GNSS sweep → comparison doc

`run_all.sh` chains 3→6 once 1–2 are done.
