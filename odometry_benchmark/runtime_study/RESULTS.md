# Per-stage runtime, YOLOv5 signal detector (run 2026-11-05, idle laptop)

Intel Core i7-13700HX, 16 GB RAM, NVIDIA RTX 5050 Laptop GPU. ms per frame inside the surveyed pole section
(2,151-2,173 frames), where every stage runs; GNSS 0 %. See README.md for the stage split.

| Stage (ms/frame) | Published FastReg | FastReg, this laptop | KISS-ICP pipeline |
|---|---|---|---|
| LiDAR odometry | 210.0 | 254.0 | 10.9 |
| Pole geolocation | 25.3 | 19.7 | 19.4 |
| Vehicle geolocation | 10.2 | 16.3 | 13.3 |
| Total | 245.5 | 290.0 | 43.6 |
| Odometry share | 85.5 % | 87.6 % | 25 % |

Sources
- LiDAR odometry: FastReg `outputs/fastreg/timing.json` (network on the GPU + Open3D RANSAC on the CPU per scan
  pair, `torch.cuda.synchronize()` before the time is read; scan loading and FastReg's 0.3 m voxel sampling run BEFORE
  the timer and are not included, so 254.0 ms is a lower bound; mean over all 5,418 pairs, median 253.1 ms);
  KISS-ICP `out/kiss_icp_mt/timing.json` (CPU, TBB default threading, `register_frame`, which includes KISS-ICP's own
  preprocessing: range crop, de-skewing, voxel downsampling; mean 10.88 ms, median 10.88 ms over all 5,419 scans;
  reading scans from the bag into .npy is done once beforehand and not included).
- ROS bag preprocessing is NOT in the table (neither pipeline):
  - images + GNSS + IMU for the geolocation stages: the original script's `bag_load` stage, 37.41 s (KISS-ICP run) /
    36.76 s (FastReg run) for the drive = 6.9 / 6.8 ms per frame over 5,420 frames. It reads the full 41 GB bag
    (incl. the camera topics the reduced bag of the published setup did not contain) and all four LiDAR image
    streams, of which the signal model uses two (signal, range).
  - point clouds for the odometry (`scripts/00_extract_scans.py`: /ouster/points -> xyz + per-point time, .npy):
    ~135 s for 5,419 scans = ~25 ms per scan; FastReg also needs reflectivity, extracted separately: ~98 s = ~18 ms
    per scan. Not timed by a timer: first-to-last output file write time, so it includes writing .npy to disk.
- No warm-up frames are excluded in any stage; model loading is excluded. YOLOv5 has no explicit CUDA synchronize,
  but its NMS (boolean indexing of GPU tensors) and box drawing force the host to wait for the GPU inside the timed
  pole-geolocation interval. YOLOv5 inference alone (original script's own timer): mean 9.36 ms, median 8.31 ms over
  2,173 frames (`fasterlio_integration/output/timing_FastReg_gnss_percentage_gnss0.json`).
- Pole / vehicle geolocation: `out/stage_kiss_icp.json`, `out/stage_fastreg_run.json` from `timed_GNSS_percentage.py`
  (the original script with stage timers). Same code for both columns; the vehicle-geolocation difference
  (13.3 vs 16.3 ms) is run-to-run variation of identical code (prints, per-frame matplotlib scatter calls).
- Over all 5,420 frames instead of the pole section: pole 7.7 / 7.9 ms, vehicle 5.3 / 6.5 ms (KISS-ICP / FastReg).
- Validity check: the runs reproduce the localization errors of `results/snowpole_all_methods.csv` at 0 % GNSS
  (KISS-ICP 7.70 m, FastReg 11.01 m; `fasterlio_integration/output/timing_comparison.csv`, where the original's
  timer tags both rows "FastReg").
- Earlier KISS-ICP figure (54.0 ms, `outputs/kiss_icp/timing.json`) was single-threaded, pinned to one core, with
  GenZ-ICP and MAD-ICP running at the same time; superseded by this measurement.
