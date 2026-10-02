# Super-LIO integration — design & status

## Why
Third odometry backend for the FastReg-replacement benchmark, after Faster-LIO and
GLIM. Super-LIO (RA-L 2026) is a **CPU** IESKF LIO whose "OctVox" 8-pt/voxel map +
heuristic KNN target exactly the dense-cloud NN-search cost that caps Faster-LIO —
the strongest pure-CPU speed candidate. Goal: beat Faster-LIO's odometry throughput
(ds 12.65 ms/scan = 79 FPS) at comparable pole-corrected accuracy.

## Design decisions (non-obvious)
- **Native ROS 2 Jazzy, not Docker.** Super-LIO's active branch is ROS 2 Jazzy =
  this host. Faster-LIO needed Docker (ROS1) and GLIM needed a CUDA image; neither
  applies here. All C++ deps (Eigen/PCL/glog/TBB/pcl-conversions, gcc-13/C++20) are
  already on the host — only `colcon`, `ros-jazzy-pcl-ros`, `libgflags-dev` are added.
- **Reuse GLIM's ROS 2 bag** (`../glim_integration/output/ros2_bag`, /ouster/points
  5419 + /ouster/imu 54202). Super-LIO subscribes to ROS 2 topics → no new conversion.
- **Vendored `livox_ros_driver2` msg shim** (`Super-LIO/src/livox_ros_driver2/`).
  Super-LIO hard-requires `find_package(livox_ros_driver2)` and includes
  `custom_msg.hpp`, but it's not vendored and has no Jazzy apt pkg. We only use the
  Ouster PointCloud2 path, so an interface-only CustomMsg/CustomPoint package suffices.
- **Ouster = lidar_type 7**; point structs (`ouster_ros::Point`, etc.) are
  self-contained in `basic/include/basic/alias.h` — no external sensor pkgs.
- **No TUM file / no offline app.** Super-LIO publishes `nav_msgs/Odometry` on
  `/lio/odom`; `00_` records that topic, and the bridge reads it (rosbags AnyReader
  reads ROS 2 bags). FPS comes from the node's own per-stage timer
  (`lio.eva.timer: true`), flushed by `printTimeRecord()` on SIGINT (spin() returns) —
  rate-independent, so `ros2 bag play` rate doesn't bias it.
- **Bridge maths identical** to the Faster-LIO/GLIM bridge (IMU sensor→epoch affine,
  Kabsch start-anchor to UTM33N, resample to 5422 GNSS frames) so backends compare
  1:1. GNSS + IMU clock read from the full ROS 1 bag (the ROS 2 bag has no GPS).

## Pipeline
`00_run_superlio.sh` (odometry + FPS) → `10_superlio_traj_to_csv.py`
(`incremental_navigation_results_superlio.csv`) → snow-pole pipeline @ 0% GNSS →
`40_gnss_sweep.sh` (0/10/25/50%, seed 0) → `SUPERLIO_VS_FASTERLIO_COMPARISON.md`.
`run_all.sh` chains all four. Pipeline scripts patched to tag the backend `Super-LIO`.

## Status — COMPLETE (2026-09-10)
- [x] Repo cloned (`../../Super-LIO`, ros2 branch); scaffold + config + scripts; label patched.
- [x] apt deps installed + `colcon build` (native ROS 2 Jazzy).
- [x] odometry / bridge / pipeline / sweep run + `SUPERLIO_VS_FASTERLIO_COMPARISON.md`.

**Result:** fastest backend (~194–271 FPS, ~2.5× Faster-LIO, ~7× GLIM; ~74 s wall
vs 111–205 s), and most accurate with any GNSS (pole-corrected median 10 %/25 %/50 %
= 0.97/0.57/0.21 m — best of all four). 0 % GNSS is the weak spot (vehicle 55.9 m;
pole-loc still 3.5 m). Two local source mods were needed — see the comparison doc §5:
subscription QoS `best_effort→reliable` (else 35 % of scans drop under bag-play), and
SIGINT the node binary / TERM the recorder in `00_run_superlio.sh`.
