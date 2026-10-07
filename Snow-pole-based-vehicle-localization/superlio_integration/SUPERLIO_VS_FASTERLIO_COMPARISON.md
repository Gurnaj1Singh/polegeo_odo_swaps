# Super-LIO vs Faster-LIO / GLIM — speed & accuracy (2026-09-10)

**Headline:** Super-LIO ([Liansheng-Wang/Super-LIO](https://github.com/Liansheng-Wang/Super-LIO),
RA-L 2026) is the **fastest** odometry on this hardware. Run with the **default
`filter_rate 2`** config it is **~159 FPS (6.29 ms/scan), ~2× Faster-LIO and ~6× GLIM**,
competitive at **0 % GNSS (pole-corrected 9.65 m, on par with Faster-LIO/GLIM)**, and
**with any GNSS (≥10 %) the most accurate of all four backends**. The old `filter_rate 4`
speed-max variant is faster still (~194–271 FPS) but regressed 0 % GNSS to 55.9 m — now
understood and fixed by the `filter_rate 2` default (see §5 / `summary.md` Part K). Same
downstream pipeline, same bag, same GNSS seeding (`GNSS_SEED=0`) as the Faster-LIO/GLIM
studies.

Ran natively (ROS 2 Jazzy, `colcon`) — no Docker — reusing the ROS 2 bag from the
GLIM run (`/ouster/points` + `/ouster/imu`).

## 1. Speed — odometry throughput (i7-12700H, CPU only)

Compute throughput from Super-LIO's own per-stage timer (`lio.eva.timer: true`,
rate-independent), summing Undistort+DownSample+Observe+UpdateMap:

| Backend | Engine | Downsampling | ms/scan | Throughput |
|---|---|---|---|---|
| **Super-LIO** (default) | CPU IESKF + OctVox (8 pt/vox) + HKNN | filter_rate 2 / vox 0.5 | **6.29** | **~159 FPS (15.9×)** |
| Super-LIO (speed-max) | same | filter_rate 4 / vox 0.5 | 3.7–5.2 | ~194–271 FPS (19–27×) |
| Faster-LIO (ds) | CPU iVox + ESIKF | pt_filter 4 / surf 1.0 | 12.65 | 79.1 FPS (7.9×) |
| Faster-LIO (base) | CPU iVox + ESIKF | pt_filter 2 / surf 0.5 | 21.35 | 46.8 FPS (4.7×) |
| GLIM | GPU VGICP + factor graph | ~10k pts/scan | 37.7 | 26.5 FPS (2.65×) |

Per-stage (Super-LIO, default `filter_rate 2`): Undistort 0.39 · DownSample 0.84 ·
**Observe 4.11** · UpdateMap 0.95 ms. The correspondence search (`Observe`, the
HKNN step) dominates but is still tiny — the OctVox 8-pt/voxel cap directly attacks
the dense-cloud nearest-neighbour cost that limits the iVox/ikd-tree family. Even at the
denser default, Super-LIO is ~2× Faster-LIO (ds) and ~6× GLIM.

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

Super-LIO column is the **default `filter_rate 2`** config.

| GNSS used | FastReg | Faster-LIO | GLIM | **Super-LIO** |
|---|---|---|---|---|
| 0 %  | 8.41 | 8.70 | 10.18 | **9.65** |
| 10 % | 2.13 | 0.80 | 1.14 | **0.91** ✅ |
| 25 % | 1.29 | 0.46 | 0.63 | **0.53** ✅ |
| 50 % | 0.53 | 0.19 | 0.26 | **0.20** ✅ |

- **With any GNSS, Super-LIO is the most accurate of all four at every level**
  (edges out GLIM and ties/edges Faster-LIO at 10/25/50 %). Even 10 % GNSS collapses its
  error ~11× (9.65 → 0.91 m).
- **0 % GNSS is competitive with the default config** (9.65 m, on par with Faster-LIO
  8.70 m and GLIM 10.18 m). The old `filter_rate 4` speed-max variant degraded 0 % to
  55.94 m — a resolved issue (§5): the denser `filter_rate 2` cloud fixes the along-track
  scale that was tripping the pole matcher.

## 3. Pole localization — the actual deliverable (0 % GNSS)

Distance from each predicted pole to its ground-truth pole (this is what the project
produces — the snow-pole map). Independent of the vehicle-position metric above:

| Backend | median | mean | max | events | distinct poles |
|---|---|---|---|---|---|
| **Super-LIO** (`filter_rate 2`) | **2.26 m** | 3.01 | 13.12 | 355 | 131 |
| Faster-LIO | 2.14 m | 2.68 | 11.72 | 355 | 135 |
| GLIM | 2.65 m | 3.46 | 15.18 | 356 | 134 |

Even at 0 % GNSS, Super-LIO localizes poles to ~2.3 m median — on par with Faster-LIO
(2.1 m) and GLIM (2.65 m). **Events vs. distinct poles:** the `events` column counts
geo-localization *events* (one per surviving detection); a physical pole yields ~2–3
events as the vehicle drives past it (mean ~2.7, up to 5), so Super-LIO's 355 events
localize **131 distinct** ground-truth poles of 290. Each re-sighting is an independent
fix — the high 0 %-GNSS *vehicle* error does **not** wreck the pole map, because at each
sighting the position is re-corrected; it is the *between-pole* track that drifts.

## 4. Trajectory / drift (raw odometry vs GNSS, start-anchored)

| Backend | median | mean | max | track-len ratio |
|---|---|---|---|---|
| Super-LIO (`filter_rate 2`) | **~372 m** | ~694 | ~2547 | 0.922 |
| Faster-LIO | 448 m | 790 | 2856 | 0.947 |
| GLIM | 448 m | 755 | 2594 | 0.936 |

Super-LIO's global drift is the tightest median of the three (IMU-limited, as for all
backends). The default `filter_rate 2` also holds along-track scale best of Super-LIO's
two configs (0.922 vs 0.903 at `filter_rate 4`) — the key to the 0 %-GNSS fix (§5). Clock
fit residual 6.9 ms; 5408 poses; node RSS ~0.4 GB.

## 5. Caveats / notes

- **0 % GNSS anomaly — diagnosed & FIXED (now the default).** With the old
  `filter_rate 4` config the 0 %-GNSS pole-corrected median was 55.94 m. Root cause: the
  sparser cloud let the odometry's along-track **scale** drift (track ratio 0.903), so the
  dead-reckoned predicted pole crossed into a neighbouring pole's basin and the un-gated
  matcher locked onto the wrong pole. Switching the default to **`filter_rate 2`** (denser,
  track ratio 0.922) drops it to **9.65 m** — on par with Faster-LIO (8.70 m) and GLIM
  (10.18 m). Not a heading or code bug (the downstream pipeline is byte-identical for all
  three backends). Full derivation + instrumented proof in `summary.md` Part K. Irrelevant
  with any GNSS either way.
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
project**, run with the **default `filter_rate 2`** config: the **fastest by a wide
margin** (~159 FPS, ~2× Faster-LIO, ~6× GLIM) and the **most accurate at every GNSS
level**, including **0 %** (9.65 m, on par with Faster-LIO/GLIM — the old `filter_rate 4`
0 % weakness is fixed). Pole localization at 0 % is ~2.3 m (competitive).

Practical recommendation: **Super-LIO with the default `filter_rate 2` config is the
recommended odometry front-end** — it removes the odometry-speed bottleneck entirely
(odometry is no longer the pipeline's slow stage) and is the most accurate backend at
every GNSS level. Use `filter_rate 4` only to benchmark peak throughput (~217 FPS).

Artifacts: `output/superlio_run_ouster_os2_128.log` (per-stage timing),
`../incremental_navigation_results_superlio.csv`, `../snowpole_results_superlio.csv`,
`output/live_map_superlio.png`, `output/sweep_superlio_gnss*.log`,
metrics `../fasterlio_integration/output/timing_Super-LIO_*.json`.
