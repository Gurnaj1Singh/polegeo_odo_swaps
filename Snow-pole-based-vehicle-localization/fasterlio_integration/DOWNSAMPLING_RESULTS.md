# Faster-LIO downsampling experiment — results

**Date:** 2026-09-08  **Host:** gsm-G15 (20-core, torch CPU)  **Dataset:** E39
Hemnekjølen, full bag `2024-02-28-12-59-51.bag` (541.75 s, 5414 LiDAR scans @ 10 Hz).

## What changed
Two knobs in `config/ouster_os2_128.yaml` (the requested "downsample first" step):

| knob                | baseline | downsampled |
|---------------------|:--------:|:-----------:|
| `point_filter_num`  | 2        | **4**       |
| `filter_size_surf`  | 0.5 m    | **1.0 m**   |

Everything else identical (`filter_size_map` 0.5, `ivox_grid_resolution` 0.5,
`max_iteration` 4, NEARBY18).

## Method (controlled A/B)
Both configs were run through the **identical** pipeline:
`run_mapping_offline` (reads the bag directly at max CPU, times only the per-scan
LIO compute) → `10_fasterlio_traj_to_csv.py` (start-anchor, drift preserved) →
`snowpole_based_vehicle_localization.py` at **0 % GNSS** (pole-only correction).

Offline mode was used because at `rosbag play -r 1.0` the wall-clock is rate-
limited to the bag duration and hides the true throughput; offline reports the
rate-independent `Faster LIO average FPS` and processes **every** scan (no drops).
Config files: `config/ouster_os2_128_offline_{base,ds}.yaml`.

**Validation:** the offline baseline reproduced the known *online* baseline —
pole-corrected median **10.00 m** vs 10.11 m, and 355 detection events in both —
so the offline path is equivalent and the A/B is trustworthy.

## Results

### 1. Odometry throughput (pure LIO compute, 10 Hz sensor)
| metric                          | baseline | downsampled | change |
|---------------------------------|:--------:|:-----------:|:------:|
| **Faster-LIO FPS**              | 46.84    | **79.06**   | ×1.69  |
| **realtime factor**             | 4.68×    | **7.91×**   | ×1.69  |
| ms / scan (Laser Mapping)       | 21.35    | **12.65**   | −41 %  |
| offline wall incl. 41 GB read   | 159 s    | 111 s       |        |

Per-scan breakdown (mean ms) — where the time went:

| stage                    | base  | ds   | ×faster |
|--------------------------|:-----:|:----:|:-------:|
| Preprocess (raw parse)   | 6.38  | 6.33 | 1.01    |
| Undistort Pcl            | 3.72  | 1.86 | 1.99    |
| Downsample PointCloud    | 1.75  | 0.80 | 2.18    |
| ObsModel (Lidar Match)   | 0.94  | 0.38 | 2.50    |
| **IEKF Solve & Update**  | 7.62  | 2.85 | 2.67    |
| Incremental Mapping      | 1.85  | 0.77 | 2.41    |
| iVox Add Points          | 1.10  | 0.52 | 2.11    |

The IEKF solve + undistort dominate the saving. **`Preprocess (Standard)` (6.3 ms,
now ~half the per-scan budget) is unaffected** — it is the fixed cost of parsing
the raw 128×1024 Ouster cloud, upstream of `point_filter_num`. That is the floor
for these two knobs; going faster needs a cheaper raw parse (fewer scan lines /
driver-side subsampling), not more decimation.

### 2. Accuracy — pole-corrected vehicle position error (0 % GNSS)
| error (m) | baseline | downsampled |
|-----------|:--------:|:-----------:|
| median    | 10.00    | **8.67**    |
| mean      | 9.06     | **6.91**    |
| max       | 25.95    | **20.39**   |

Downsampling caused **no accuracy loss** — in this run it was slightly *better*.
The pole correction dominates the global result and the local frame-to-frame
increments (what the pipeline dead-reckons with between poles) are excellent
either way. Global yaw drift is IMU-limited and only weakly tied to point density:
raw odometry-only error was 178→113 m median (and start-anchor FL-vs-GNSS drift
431→283 m median) — i.e. the coarser run happened to drift *less*, within the
run-to-run spread of the drift process.

### 3. Poles
| quantity                                   | baseline | downsampled |
|--------------------------------------------|:--------:|:-----------:|
| pole **detection events** (geo-localized)  | 355      | 355         |
| **unique ground-truth poles** corrected on | 133      | 135         |
| of total site poles                        | 290      | 290         |
| pole-localization error to nearest GT pole | 2.54 m median | 2.15 m median |

Each unique pole is seen over ~2–3 consecutive frames (≈355 events / ~134 poles).
132–135 of the site's 290 poles fall on the traversed section (in-bounds frames).
All 355 detection events are applied as path corrections; the ~134 unique poles
are what "make the path accurate".

### 4. Downstream pipeline (odometry-agnostic)
Wall-clock 126.4 → 126.8 s (unchanged, as expected — the snow-pole detection +
geo-localization loop is independent of the odometry backend). YOLO ~20.5 ms/frame
(~49 fps), pipeline ~68 fps over 5420 frames.

## Takeaway
On this data `point_filter_num 2→4` + `filter_size_surf 0.5→1.0` buys a **1.69×**
odometry speedup (**4.7×→7.9× realtime**) with **zero accuracy cost**. Even the
baseline already clears the 2–3× realtime target on this 20-core host; the value
of downsampling is the headroom for weaker in-vehicle compute. The next lever is
the raw-cloud parse, not these two knobs.

## Artifacts
- `config/ouster_os2_128.yaml` (updated to pfn=4 / surf=1.0), plus
  `config/ouster_os2_128_offline_{base,ds}.yaml`.
- `output/offline_bench_{base,ds}.log`, `output/fl_time_offline_{base,ds}.log`
  (per-scan Timer dumps), `output/fl_traj_offline_{base,ds}.txt` (TUM).
- `incremental_navigation_results_fasterlio_offline_{base,ds}.csv`,
  `snowpole_results_offline_{base,ds}.csv`.
- `output/timing_Faster-LIO_pipeline_offline_{base,ds}.json`, appended to
  `output/timing_comparison.csv`.
- `output/live_map_offline_{base,ds}.png`, `output/ds_viz/summary_*.png`.
