# SnowPole: all odometry methods

Median vehicle-position error after snow-pole correction (m, lower is better), random seed 0, YOLOv5 `pole_best_signal.pt` (signal modality) for every method. Same bridge (start-anchored, GNSS travel 20-420 m), same correction and scoring (`scripts/04_gnss_sweep.py`).

| Category | Method | Venue | 0 % | 10 % | 25 % | 50 % | Raw drift median | ms/scan | Source |
|---|---|---|---|---|---|---|---|---|---|
| 1. Non-DL, up to 2024 | KISS-ICP | RA-L 2023 | 7.70 | 0.75 | 0.42 | 0.16 | 188.02 | 54.00 | run in this repo |
| 1. Non-DL, up to 2024 | MAD-ICP | RA-L 2024 | 9.78 | 1.02 | 0.59 | 0.24 | 430.78 | 92.70 | run in this repo |
| 1. Non-DL, up to 2024 | FAST-LIO2 | T-RO 2022 | 10.25 | 1.17 | 0.65 | 0.26 | 514.36 | 10.70 | run in this repo |
| 1. Non-DL, up to 2024 | Faster-LIO | RA-L 2022 | 8.67 | 0.79 | 0.46 | 0.19 | 282.73 | 13.10 | run in this repo |
| 1. Non-DL, up to 2024 | GLIM | RAS 2024 | 10.22 | 1.20 | 0.65 | 0.27 | 464.61 | 317.80 | run in this repo |
| 2. DL/ML, up to 2024 | FastReg | RA-L 2021 | 8.41 | 2.13 | 1.29 | 0.53 |  |  | author trajectory, re-scored here |
| 2. DL/ML, up to 2024 | FastReg-run | RA-L 2021 | 11.01 | 2.19 | 0.91 | 0.38 | 774.67 | 254.00 | run in this repo |
| 2. DL/ML, up to 2024 | DeepPointMap | AAAI 2024 | 2144.89 | 3.67 | 1.41 | 0.52 | 631.95 | 116.80 | run in this repo |
| 2. DL/ML, up to 2024 | PIN-SLAM | T-RO 2024 | 34.00 | 1.74 | 0.82 | 0.34 | 580.90 | 552.30 | run in this repo |
| 3. Non-DL, 2025-2026 | RKO-LIO | arXiv/RA-L 2025 | 1.20 | 0.60 | 0.33 | 0.12 | 99.07 | 20.60 | run in this repo |
| 3. Non-DL, 2025-2026 | GenZ-ICP | RA-L 2025 | 9.84 | 0.97 | 0.57 | 0.23 | 355.21 | 259.00 | run in this repo |
| 3. Non-DL, 2025-2026 | Super-LIO | RA-L 2026 | 9.64 | 0.90 | 0.53 | 0.20 | 370.64 | 5.70 | run in this repo |
| 4. DL/ML, 2025-2026 | BUFFER-X | ICCV 2025 | 15.98 | 4.85 | 1.83 | 0.70 | 1372.72 | 2149.30 | run in this repo |
| 4. DL/ML, 2025-2026 | UGP | CVPR 2025 | 922.17 | 3.66 | 1.50 | 0.59 | 560.64 | 427.10 | run in this repo |
| 4. DL/ML, 2025-2026 | CLID-SLAM | RA-L 2025 |  |  |  |  |  |  | FAILED - lost track on the highway: stopped itself at scan 911/5419 with the repository's 1-60 m Ouster-128 config and at scan 831/5419 with the shared 2-150 m range (it tracks RKO-LIO to within 2 m for the first 500 scans, then under-estimates speed above ~15 m/s); outputs/clidslam_attempts/ |
| 4. DL/ML, 2025-2026 | Hi-LOAM | IEEE TMM 2026 |  |  |  |  |  |  | not runnable: public repo (Zhangjyhhh/Hi-LOAM @ a8add39) never contained its model/ package (FeatureOctree, Decoder); replaced by CLID-SLAM |
| Extra: non-DL, 2025 (outside the 12) | Splat-LOAM | ICCV 2025 |  |  |  |  |  |  | FAILED - lost track on the highway: from sensor t~150 s its estimate moved ~0.6 m/s (true 15-20 m/s) and climbed to +34 m, inserting a keyframe every scan (~3 min/scan, GPU memory full); stopped manually at t~278 s of ~542 s after 6 h; outputs/splatloam_attempts/full_run_lost_track/ |
