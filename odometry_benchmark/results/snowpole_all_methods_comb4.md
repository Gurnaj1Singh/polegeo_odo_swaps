# SnowPole: all odometry methods

Median vehicle-position error after snow-pole correction (m, lower is better), random seed 0, YOLO11n `models/yolo11n_comb4_final.pt` trained on Combination 4 (near-IR, signal, reflectivity stacked; results/detector_yolo11n_comb4.md) for every method. Same bridge (start-anchored, GNSS travel 20-420 m), same correction and scoring (`scripts/04_gnss_sweep.py`).

| Category | Method | Venue | 0 % | 10 % | 25 % | 50 % | Raw drift median | ms/scan | Source |
|---|---|---|---|---|---|---|---|---|---|
| 1. Non-DL, up to 2024 | KISS-ICP | RA-L 2023 | 2.29 | 0.80 | 0.47 | 0.18 | 188.02 | 54.00 | run in this repo |
| 1. Non-DL, up to 2024 | MAD-ICP | RA-L 2024 | 9.75 | 1.09 | 0.62 | 0.29 | 430.78 | 92.70 | run in this repo |
| 1. Non-DL, up to 2024 | FAST-LIO2 | T-RO 2022 | 10.03 | 1.24 | 0.68 | 0.33 | 514.36 | 10.70 | run in this repo |
| 1. Non-DL, up to 2024 | Faster-LIO | RA-L 2022 | 8.63 | 0.86 | 0.50 | 0.22 | 282.73 | 13.10 | run in this repo |
| 1. Non-DL, up to 2024 | GLIM | RAS 2024 | 10.03 | 1.23 | 0.69 | 0.32 | 464.61 | 317.80 | run in this repo |
| 2. DL/ML, up to 2024 | FastReg | RA-L 2021 | 8.91 | 2.26 | 1.33 | 0.60 |  |  | author trajectory, re-scored here |
| 2. DL/ML, up to 2024 | FastReg-run | RA-L 2021 | 11.26 | 2.11 | 0.95 | 0.46 | 774.67 | 254.00 | run in this repo |
| 2. DL/ML, up to 2024 | DeepPointMap | AAAI 2024 | 2136.35 | 3.59 | 1.39 | 0.57 | 631.95 | 116.80 | run in this repo |
| 2. DL/ML, up to 2024 | PIN-SLAM | T-RO 2024 | 18.16 | 1.66 | 0.85 | 0.38 | 580.90 | 552.30 | run in this repo |
| 3. Non-DL, 2025-2026 | RKO-LIO | arXiv/RA-L 2025 | 1.14 | 0.66 | 0.38 | 0.14 | 99.07 | 20.60 | run in this repo |
| 3. Non-DL, 2025-2026 | GenZ-ICP | RA-L 2025 | 9.56 | 1.05 | 0.62 | 0.27 | 355.21 | 259.00 | run in this repo |
| 3. Non-DL, 2025-2026 | Super-LIO | RA-L 2026 | 9.38 | 0.99 | 0.58 | 0.25 | 370.64 | 5.70 | run in this repo |
| 4. DL/ML, 2025-2026 | BUFFER-X | ICCV 2025 | 61.68 | 5.20 | 1.86 | 0.74 | 1372.72 | 2149.30 | run in this repo |
| 4. DL/ML, 2025-2026 | UGP | CVPR 2025 | 1080.43 | 3.63 | 1.48 | 0.65 | 560.64 | 427.10 | run in this repo |
| 4. DL/ML, 2025-2026 | CLID-SLAM | RA-L 2025 |  |  |  |  |  |  | FAILED - lost track on the highway: stopped itself at scan 911/5419 with the repository's 1-60 m Ouster-128 config and at scan 831/5419 with the shared 2-150 m range (it tracks RKO-LIO to within 2 m for the first 500 scans, then under-estimates speed above ~15 m/s); outputs/clidslam_attempts/ |
| 4. DL/ML, 2025-2026 | Hi-LOAM | IEEE TMM 2026 |  |  |  |  |  |  | not runnable: public repo (Zhangjyhhh/Hi-LOAM @ a8add39) never contained its model/ package (FeatureOctree, Decoder); replaced by CLID-SLAM |
| Extra: non-DL, 2025 (outside the 12) | Splat-LOAM | ICCV 2025 |  |  |  |  |  |  | FAILED - lost track on the highway: from sensor t~150 s its estimate moved ~0.6 m/s (true 15-20 m/s) and climbed to +34 m, inserting a keyframe every scan (~3 min/scan, GPU memory full); stopped manually at t~278 s of ~542 s after 6 h; outputs/splatloam_attempts/full_run_lost_track/ |

Detector training images are frames 844-2999 of this drive (5420 frames; results/detector_dataset_frame_map.csv), so its detections there are optimistic.
