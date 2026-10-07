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
- **`filter_rate: 2` is the DEFAULT** (`ouster_os2_128_base.yaml`): keep every 2nd point.
  The denser cloud holds scale (track ratio ~0.922) so the pole matcher stays correct at
  0 % GNSS (pole-corrected 9.65 m vs 55.94 m at `filter_rate: 4`). `ouster_os2_128.yaml`
  (`filter_rate: 4`) is the speed-max variant (~217 FPS) but degrades 0 % GNSS — see
  `summary.md` Part K for the experiment.
- Output is `nav_msgs/Odometry` on **`/lio/odom`** (no TUM file) → `00_` records it.
- `lio.eva.timer: true` → per-stage compute times; flushed on SIGINT → FPS benchmark
  (rate-independent). Target to beat: Faster-LIO ds 12.65 ms/79 FPS, GLIM 26.5 FPS.

## Config variants

- `config/ouster_os2_128_base.yaml` — **DEFAULT**, `filter_rate: 2` (denser; accurate at
  all GNSS levels incl. 0 %; ~159 FPS). The runners use this with no `config` arg.
- `config/ouster_os2_128.yaml` — speed-max, `filter_rate: 4` (~217 FPS) but degrades
  0 % GNSS (pole-corrected 55.94 m). Benchmark-only.

Run the speed-max variant explicitly:
`scripts/00_run_superlio.sh <ros2_bag> 1.0 config/ouster_os2_128.yaml`.
Full reasoning + the filter_rate experiment: `summary.md` Part K.

## Results (default `filter_rate 2`, measured on this bag/host)

| Metric | Super-LIO | Faster-LIO (ref) | GLIM (ref) |
|---|---|---|---|
| Odometry speed | **6.29 ms/scan, ~159 FPS** | 12.65 ms, 79 FPS (ds) | 37.7 ms, 26.5 FPS |
| Pole-corrected error @0 % (median / mean / max) | 9.65 / 7.94 / 24.85 m | 8.70 / 6.95 / 20.82 m | 10.18 / 9.58 / 27.14 m |
| GNSS sweep, pole-corrected median (10 / 25 / 50 %) | **0.91 / 0.53 / 0.20 m** ✅ | 0.80 / 0.46 / 0.19 m | 1.14 / 0.63 / 0.26 m |
| Pole-localization error @0 % (median) | 2.26 m | 2.14 m | 2.65 m |
| Pole **detection events** / **distinct poles** (of 290) | **355 / 131** | 355 / 135 | 356 / 134 |

With any GNSS, Super-LIO is the most accurate backend; at 0 % the default `filter_rate 2`
config gives 9.65 m (the old `filter_rate 4` speed variant regressed to 55.94 m — see
`summary.md` Part K).

**Distinct poles vs. detection events.** The pipeline writes one row per *detection
event*, not per physical pole. One pole yields several events because the vehicle drives
*past* it: the detector re-acquires the **same physical pole on every frame it stays in
view and within the 5 m range gate** — ~2–3 consecutive frames (mean **2.7 events/pole**,
median 3, up to 5; 20 poles seen only once). Each re-sighting is an *independent*
range+bearing fix that re-anchors the drifting dead-reckoned track, so all events are
kept and error is reported per **event**. Super-LIO's 355 events map onto **131 distinct
ground-truth poles** (45 % of the 290 at the site); the rest were off the traversed
one-way section. Full ledger + balance in `summary.md` §H.6.
