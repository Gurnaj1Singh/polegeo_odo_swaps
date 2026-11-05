# Per-stage runtime study

Purpose: a per-stage runtime table (LiDAR odometry / pole geolocation / vehicle geolocation) for the KISS-ICP pipeline
that is comparable with the published FastReg table (210.0 / 25.3 / 10.2 ms per frame, total 245.5 ms).

- `timed_GNSS_percentage.py` is `original_GNSS_percentage.py` (a copy of
  `third_party/polegeo_odo_swaps_git/.../snowpole_based_vehicle_localization_GNSS_percentage.py`) with stage timers
  added and nothing else changed (check with `diff`). Two compatibility lines in its header: `ouster.client` aliased to
  `ouster.sdk.core` (ouster-sdk 1.0), and `contextily.py` here is a stub because the package is not installed; the
  original's try/except then skips the OpenStreetMap basemap, which is drawn once before the timed loop.
- Stage split inside the original frame loop:
  - pole geolocation: range image -> XYZ, image normalisation, YOLOv5 inference and render, and per box the nearest
    point, XYZ, distance gate, heading, azimuth and the pole's UTM projection;
  - vehicle geolocation: everything else in the loop - GNSS / dead-reckoning update, prints, the per-frame matplotlib
    scatter calls, nearest-surveyed-pole search, back-projection of the vehicle position, error bookkeeping.
- Bag loading, model loading and the final plots are outside the stages, as in the published table.
- LiDAR odometry is timed by each method's own timer (`out/kiss_icp_mt/timing.json`; FastReg:
  `outputs/fastreg/timing.json`).
- Per-frame times are given both over all 5,420 frames and over the frames inside the surveyed pole section, where
  the detector runs.

Run: `bash runtime_study/run_runtime_study.sh` (log: `runtime_study/run_runtime_study.log`).
