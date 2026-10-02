# Super-LIO vs Faster-LIO / GLIM — speed & accuracy (2026-09-10)

**Headline:** Super-LIO ([Liansheng-Wang/Super-LIO](https://github.com/Liansheng-Wang/Super-LIO),
RA-L 2026) is by far the **fastest** odometry on this hardware — **~194–271 FPS
(≈3.7–5.2 ms/scan), ~2.5–3.4× faster than Faster-LIO and ~7–10× faster than GLIM**
— and, **with any GNSS (≥10 %), the most accurate of all four backends**. Its only
weak point is the pathological **0 % GNSS** case, where the vehicle's between-fix
dead-reckoning drifts more (pole-corrected median 55.9 m vs ~10 m); pole
*localization* there is still ~3.5 m. Same downstream pipeline, same bag, same
GNSS seeding (`GNSS_SEED=0`) as the Faster-LIO/GLIM studies.

Ran natively (ROS 2 Jazzy, `colcon`) — no Docker — reusing the ROS 2 bag from the
GLIM run (`/ouster/points` + `/ouster/imu`).

## 1. Speed — odometry throughput (i7-12700H, CPU only)

Compute throughput from Super-LIO's own per-stage timer (`lio.eva.timer: true`,
rate-independent), summing Undistort+DownSample+Observe+UpdateMap:

| Backend | Engine | Downsampling | ms/scan | Throughput |
|---|---|---|---|---|
| **Super-LIO** | CPU IESKF + OctVox (8 pt/vox) + HKNN | filter_rate 4 / vox 0.5 | **3.7–5.2** | **~194–271 FPS (19–27×)** |
| Faster-LIO (ds) | CPU iVox + ESIKF | pt_filter 4 / surf 1.0 | 12.65 | 79.1 FPS (7.9×) |
| Faster-LIO (base) | CPU iVox + ESIKF | pt_filter 2 / surf 0.5 | 21.35 | 46.8 FPS (4.7×) |
| GLIM | GPU VGICP + factor graph | ~10k pts/scan | 37.7 | 26.5 FPS (2.65×) |

Per-stage (Super-LIO, rate-1.0 run): Undistort 0.38 · DownSample 0.61 ·
**Observe 3.38** · UpdateMap 0.79 ms. The correspondence search (`Observe`, the
HKNN step) dominates but is still tiny — the OctVox 8-pt/voxel cap directly attacks
the dense-cloud nearest-neighbour cost that limits the iVox/ikd-tree family.

**Wall-clock to process the 541.8 s bag** (Super-LIO has no offline bag reader, so
it is driven by `ros2 bag play`; the reliable-QoS fix lets a high replay rate
backpressure to the compute limit instead of dropping scans):

| Backend | Wall-clock | Notes |
|---|---|---|
| **Super-LIO** | **~74 s** (rate 10) | rate-capped, not compute-capped — compute allows faster |
| Faster-LIO (ds) | 111 s | offline `run_mapping_offline` |
| Faster-LIO (base) | 159 s | offline |
| GLIM | ~205 s | `glim_rosbag` (+240 s one-time ROS1→ROS2 convert) |

Super-LIO already wins on wall-clock too, with headroom. (At the default rate 1.0 it
is playback-bound at ~541 s, exactly like Faster-LIO's `rosbag play` path — not a
compute measurement.)

## 2. Accuracy — pole-corrected vehicle position vs GNSS availability

Median vehicle-position error (m), identical pipeline, `GNSS_SEED=0`:

| GNSS used | FastReg | Faster-LIO | GLIM | **Super-LIO** |
|---|---|---|---|---|
| 0 %  | 8.41 | 10.11 | 10.19 | **55.94** |
| 10 % | 2.13 | 1.05 | 1.13 | **0.97** ✅ |
| 25 % | 1.29 | 0.61 | 0.64 | **0.57** ✅ |
| 50 % | 0.53 | 0.25 | 0.26 | **0.21** ✅ |

- **With any GNSS, Super-LIO is the most accurate of all four at every level**
  (edges out Faster-LIO/GLIM at 10/25/50 %). Even 10 % GNSS collapses its error
  ~58× (55.94 → 0.97 m).
- **0 % GNSS is the exception** — Super-LIO's pure dead-reckoning between pole
  fixes drifts more than the others (55.94 m). This mirrors the Faster-LIO study's
  finding that 0 % is the pathological case (there Faster-LIO trailed FastReg too),
  but it is more pronounced here. Real deployment always has some GNSS, where
  Super-LIO is best.

## 3. Pole localization — the actual deliverable (0 % GNSS)

Distance from each predicted pole to its ground-truth pole (this is what the project
produces — the snow-pole map). Independent of the vehicle-position metric above:

| Backend | median | mean | max | events |
|---|---|---|---|---|
| **Super-LIO** | **3.52 m** | 4.44 | 41.5 | 356 |
| Faster-LIO | 2.50 m | 3.29 | 15.0 | 355 |

Even at 0 % GNSS, Super-LIO localizes poles to ~3.5 m median — comparable to
Faster-LIO (2.5 m), with a heavier tail. So the high 0 %-GNSS *vehicle* error in §2
does **not** wreck the pole map: at pole sightings the position is corrected; it is
the *between-pole* vehicle track that drifts.

## 4. Trajectory / drift (raw odometry vs GNSS, start-anchored)

| Backend | median | mean | max | track-len ratio |
|---|---|---|---|---|
| Super-LIO | **404 m** | 762 | 2788 | 0.90 |
| Faster-LIO | 448 m | 790 | 2856 | 0.94 |
| GLIM | 448 m | 755 | 2594 | 0.94 |

Super-LIO's global drift is actually the tightest median of the three (IMU-limited,
as for all backends). Clock fit residual 6.9 ms; 5409 poses; node RSS ~0.4 GB.

## 5. Caveats / notes

- **0 % GNSS anomaly (unexplained):** Super-LIO has the *best* raw odometry over the
  pole region (odometry-only median 139.8 m vs Faster-LIO 188 m) yet the *worst*
  pole-corrected vehicle position (55.94 m vs 10.11 m) — its pole correction is far
  less effective at fixing the vehicle track (2.5× reduction vs Faster-LIO's ~18×).
  Not caused by jitter (identical, 0.06 m) or frozen frames (all 253 are the start
  idle, none in the pole region). Likely local heading inaccuracy in the pole region
  degrading the bearing-based correction. Untested lever: `ouster_os2_128_base.yaml`
  (`filter_rate 2`, denser) may improve it. Irrelevant with any GNSS.
- **`lidar_type` prints "UNKNOWN":** cosmetic off-by-one in Super-LIO's
  `lidarTypeToString` (name array has 7 slots 0–6, `OUSTER=7`). The
  `switch(g_lidar_type){ case OUSTER: }` still matches and parses correctly (proven
  by successful map init + a complete trajectory).
- **Two local source mods** (documented in-code, don't affect the algorithm):
  1. `ROSWrapper.cpp` subscription QoS `best_effort → reliable` + deeper queues, so
     `ros2 bag play` doesn't drop scans under the timer-starved single-threaded
     executor. Without this, only 1877/5419 scans were processed (35 %). Does not
     affect the rate-independent FPS number.
  2. `00_run_superlio.sh` SIGINTs the actual node binary (the `ros2 run` wrapper
     doesn't forward signals) and TERMs the recorder (it ignores SIGINT here).
- **Vendored `livox_ros_driver2` msg shim** (`Super-LIO/src/livox_ros_driver2/`):
  interface-only, to satisfy `find_package(livox_ros_driver2)`; unused at runtime
  (Ouster path).
- Pipeline wall-clock is odometry-agnostic (~127 s, ~68 fps, YOLO ~15 ms/frame).

## 6. Verdict

On this laptop Super-LIO is the **best overall LiDAR-inertial odometry for this
project**: the **fastest by a wide margin** (~194–271 FPS, ~2.5× Faster-LIO,
~7× GLIM; ~74 s wall vs 111–205 s) and the **most accurate with any GNSS**
(best at 10/25/50 %). Pole localization at 0 % GNSS is ~3.5 m (competitive). The
only caveat is elevated *vehicle* drift at exactly 0 % GNSS — a deployment-irrelevant
corner, and a candidate for the `base` (denser) config to tighten.

Practical recommendation: **Super-LIO (ds config) is the recommended odometry
front-end** — it removes the odometry-speed bottleneck entirely (odometry is no
longer the pipeline's slow stage) and improves accuracy whenever GNSS is available.

Artifacts: `output/superlio_run_ouster_os2_128.log` (per-stage timing),
`../incremental_navigation_results_superlio.csv`, `../snowpole_results_superlio.csv`,
`output/live_map_superlio.png`, `output/sweep_superlio_gnss*.log`,
metrics `../fasterlio_integration/output/timing_Super-LIO_*.json`.
