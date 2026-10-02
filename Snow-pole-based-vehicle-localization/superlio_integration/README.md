# Super-LIO integration

Swap the snow-pole pipeline's LiDAR odometry for **Super-LIO**
([Liansheng-Wang/Super-LIO](https://github.com/Liansheng-Wang/Super-LIO), RA-L 2026)
— a CPU IESKF LIO with a compact "OctVox" 8-pt/voxel map + heuristic-guided KNN,
claiming 1.2–4× higher realtime speed than peers. Mirrors `fasterlio_integration/`
and `glim_integration/`: produces `incremental_navigation_results_superlio.csv`
(same `easting`/`northing` schema) that the pipeline consumes unchanged.

Unlike the others this runs **natively** — Super-LIO's active branch is ROS 2 Jazzy,
matching this host (no Docker). It reuses the ROS 2 bag produced for GLIM.

## One-time setup

```bash
# 1) system deps (needs sudo; the rest is unprivileged)
sudo apt update
sudo apt install -y python3-colcon-common-extensions ros-jazzy-pcl-ros libgflags-dev

# 2) build Super-LIO (repo already cloned at ../../Super-LIO, ros2 branch)
#    a minimal interface-only livox_ros_driver2 msg pkg is vendored under src/
#    to satisfy find_package(livox_ros_driver2) — the Ouster path never uses it.
cd ../../Super-LIO
colcon build
```

Everything else (Eigen, PCL, glog, TBB, pcl-conversions, gcc-13/C++20) is already
present on the host.

## Run

```bash
# full pipeline (odometry -> CSV -> localization -> GNSS sweep)
superlio_integration/run_all.sh

# or step by step:
superlio_integration/scripts/00_run_superlio.sh          # -> output/superlio_odom + FPS
#   (reuses ../glim_integration/output/ros2_bag by default; drop to rate 0.5 if scans drop)
source .baginspect_venv/bin/activate
python superlio_integration/scripts/10_superlio_traj_to_csv.py \
    --dataset-bag snow_pole_geo_localization_data/2024-02-28-12-59-51.bag \
    --odom-bag superlio_integration/output/superlio_odom --align start \
    --out incremental_navigation_results_superlio.csv
env -u PYTHONPATH MPL_BACKEND=Agg \
    INCREMENTAL_NAV_CSV=incremental_navigation_results_superlio.csv \
    RESULTS_CSV=snowpole_results_superlio.csv \
    ~/miniconda3/envs/polegeo/bin/python snowpole_based_vehicle_localization.py
superlio_integration/scripts/40_gnss_sweep.sh            # 0/10/25/50% GNSS, seed 0
```

## Key facts (why the config looks the way it does)

- **Ouster = `lio.sensor.lidar_type: 7`** — Super-LIO's `case OUSTER` reads
  `ouster_ros::Point` and per-point `pt.t` (ns), i.e. our `/ouster/points` exactly.
- Topics `lio.ros.{lidar,imu}_topic` = `/ouster/points`, `/ouster/imu`.
- Extrinsic `lio.extrinsic.lidar_imu = [tx,ty,tz, R row-major(9)]`; from `Trip068.json`
  R=I, t=(-0.006253, 0.011775, -0.007645) m.
- IMU noise `imu_na/ng/nba/nbg` = Faster-LIO's `acc_cov/gyr_cov/b_*`; accel is raw m/s².
- `filter_rate: 4` (config) mirrors Faster-LIO's downsampled `point_filter_num=4` for a
  fair speed A/B; `ouster_os2_128_base.yaml` uses `filter_rate: 2` (denser base).
- Output is `nav_msgs/Odometry` on **`/lio/odom`** (no TUM file) → `00_` records it.
- `lio.eva.timer: true` → per-stage compute times; flushed on SIGINT → FPS benchmark
  (rate-independent). Target to beat: Faster-LIO ds 12.65 ms/79 FPS, GLIM 26.5 FPS.

## Config variants

- `config/ouster_os2_128.yaml` — primary, `filter_rate: 4` (downsampled/fast).
- `config/ouster_os2_128_base.yaml` — `filter_rate: 2` (denser base for the A/B).

Run a variant: `scripts/00_run_superlio.sh <ros2_bag> 1.0 config/ouster_os2_128_base.yaml`.
