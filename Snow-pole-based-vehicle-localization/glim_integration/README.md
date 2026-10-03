# glim_integration

GPU LiDAR-inertial odometry (GLIM) as a drop-in replacement for the Faster-LIO /
FastReg front-end of the snow-pole localization pipeline. Produces
`incremental_navigation_results_glim.csv` (same `easting`/`northing` schema the
pipeline reads via `INCREMENTAL_NAV_CSV`). Full design + rationale in **PLAN.md**.

Why: Faster-LIO is CPU-bound at ~1.0× real-time on the dense OS-2-128 cloud. GLIM
moves the per-scan matching to the **RTX 3050 Ti (CUDA)** and adds global
optimisation. Runs in the prebuilt `koide3/glim_ros2:jazzy_cuda13.1` image
(matches this host: ROS 2 Jazzy + driver 595).

## Layout
```
glim_integration/
  PLAN.md                         complete plan (read this first)
  config/
    overrides.json                our OS-2-128 overrides (topics, extrinsic, GPU odom, headless)
    config*.json                  GENERATED: GLIM defaults extracted from the image + patched
  docker/
    setup_nvidia_container_toolkit.sh   one-time host prep (sudo) — GPU in Docker
    pull_glim.sh                        pull the CUDA image (gpu|cpu)
    run_glim_docker.sh                  run a cmd in the container (--gpus all, mounts)
  scripts/
    00_convert_bag.sh             ROS1 .bag -> ROS2 bag (points+imu) via .baginspect_venv
    01_extract_and_patch_configs.sh  materialise + patch GLIM configs
    patch_configs.py                 stdlib config patcher (used by 01)
    02_run_glim.sh                run GLIM headless, dump TUM trajectory, print realtime factor
    run_offline_bench.sh          throughput benchmark (realtime factor vs Faster-LIO)
    10_glim_traj_to_csv.py        bridge: GLIM TUM + GNSS -> pipeline CSV
  run_all.sh                      chain 00->01->02->10
  output/                         artifacts (gitignored)
```

## One-time provisioning
```bash
# 1) GPU-in-Docker (sudo, once). Run through the session so the prompt is visible:
!  sudo bash Snow-pole-based-vehicle-localization/glim_integration/docker/setup_nvidia_container_toolkit.sh

# 2) Pull the image (~10-15 GB, no sudo):
bash Snow-pole-based-vehicle-localization/glim_integration/docker/pull_glim.sh

# 3) Reclaim disk first (see PLAN.md §7), then it's ready.
```

## Run
```bash
cd Snow-pole-based-vehicle-localization
glim_integration/run_all.sh            # convert -> configs -> GLIM -> CSV
```
Then feed the pipeline exactly like the Faster-LIO run:
```bash
env -u PYTHONPATH MPL_BACKEND=Agg \
  INCREMENTAL_NAV_CSV=incremental_navigation_results_glim.csv \
  ~/miniconda3/envs/polegeo/bin/python snowpole_based_vehicle_localization.py
```

## Throughput benchmark (the headline number)
```bash
glim_integration/scripts/run_offline_bench.sh gpu   # prints REALTIME_FACTOR
```
Compare to Faster-LIO ≈ 1.0× in `fasterlio_integration/SPEED_AND_RELIABILITY_COMPARISON.md`.

## Results (0 % GNSS, measured on this bag/host)

| Metric | GLIM | Faster-LIO (ref) |
|---|---|---|
| Pole-corrected vehicle error (median / mean / max) | **10.18 / 9.58 / 27.14 m** | 8.70 / 6.95 / 20.82 m |
| Pole-localization error to nearest GT pole (median) | 2.65 m | 2.14 m |
| Odometry-only error (median) | 170.15 m | 118.03 m |
| GNSS sweep, pole-corrected median (10 / 25 / 50 %) | 1.14 / 0.63 / 0.26 m | 0.80 / 0.46 / 0.19 m |
| Pole **detection events** / **distinct poles** (of 290) | **356 / 134** | 355 / 135 |

**Distinct poles vs. detection events.** The pipeline logs one row per *detection
event*, not per physical pole. A single pole produces several events because the vehicle
drives *past* it: the YOLO detector re-acquires the **same physical pole on every frame
it stays in view and within the 5 m range gate** — typically ~2–3 consecutive frames
(mean **2.7 events/pole**, up to 5). Each re-sighting is an *independent* range+bearing
fix that re-anchors the drifting dead-reckoned track, so all events are kept and the
error is reported per **event**. GLIM's 356 events map onto **134 distinct ground-truth
poles** (46 % of the 290 at the site); the rest were off the one-way traversed section.
Full accounting + balance in **summary.md** §7.6.

## Notes / gotchas
- **Disk**: the converted ROS2 bag is ~33 GB (`output/ros2_bag/`). Delete it after the
  run. `GLIM_BAG_COMPRESS=lz4 scripts/00_convert_bag.sh` trades disk for a slightly
  slower (decode) read — don't use it for the speed benchmark.
- **VRAM (4 GB)**: default config runs GPU odometry + CPU pose-graph global mapping.
  If odometry OOMs, raise the GLIM preprocess/voxel resolution in `config/`.
- **Trajectory dump**: GLIM writes TUM files to `/tmp/dump` on close; `02_run_glim.sh`
  bind-mounts that to `output/glim_dump/`. If it comes back empty, GLIM didn't
  auto-save — check the log and enable the save (see PLAN.md §7 / GLIM docs).
- **Clock**: GLIM stamps poses on the Ouster sensor clock; the bridge maps them to
  GNSS epoch with the same IMU affine as Faster-LIO, and re-bases automatically if
  GLIM ever emits relative stamps.
- **CPU fallback** (no GPU toolkit): `pull_glim.sh cpu` +
  `GLIM_IMAGE=koide3/glim_ros2:jazzy GLIM_GPUS='' run_all.sh` — validates the
  pipeline but is NOT the speed comparison.
