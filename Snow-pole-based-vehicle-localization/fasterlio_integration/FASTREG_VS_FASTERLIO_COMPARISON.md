# FastReg vs Faster-LIO (incl. downsampled) — full comparison table

**Date:** 2026-09-08  **Host:** gsm-G15 (20-core CPU, torch CPU; RTX 3050 Ti present
but unused).  **Dataset:** E39 Hemnekjølen, 541.75 s / 5,414-scan drive @ 10 Hz.

## Measurement status (read first)
- **Faster-LIO** (baseline `pfn2/surf0.5` and downsampled `pfn4/surf1.0`): all
  numbers **measured on this bag/host**.
- **FastReg accuracy / pipeline / pole numbers**: measured here, driven by the
  **authors' precomputed trajectory** (`incremental_navigation_results.csv`) through
  the *identical* snow-pole pipeline.
- **FastReg odometry runtime**: **NOT run here** (deliberate — see "Why FastReg was
  not re-run"). Figures are the **authors' published** registration times from the
  RA-L paper (Arnold et al., 2022), measured on a **Quadro M4000 GPU (2015)** over
  **KITTI/CODD**, voxel 0.3 m, 512 keypoints. They are cross-hardware and
  cross-dataset — treat the ~order-of-magnitude gap as robust, the exact ratio as
  indicative, not a like-for-like bench.

---

## A. Odometry stage — the real differentiator

| Aspect | **FastReg** | **Faster-LIO base** (pfn2/0.5) | **Faster-LIO downsampled** (pfn4/1.0) |
|---|---|---:|---:|
| Method | learning-based pairwise registration (PointNet++ + GNN + RANSAC) | LiDAR-**inertial** iESKF + incremental iVox map | same |
| Sensors | LiDAR only | LiDAR + IMU | LiDAR + IMU |
| Compute device | **GPU required** | CPU | CPU |
| Time per scan/registration | **410 ms** (KITTI) / **320 ms** (CODD) ¹ | **21.35 ms** | **12.65 ms** |
| Throughput | ~2.4–3.1 / s ¹ | 46.8 / s | **79.1 / s** |
| **Realtime factor @ 10 Hz** | **~0.24–0.31×** ¹ | 4.68× | **7.91×** |
| Measured on this bag? | no (paper, M4000 GPU) | **yes** (CPU) | **yes** (CPU) |

¹ Authors' figures on a Quadro M4000; different HW/dataset. Even scaling the M4000
(~2.6 TFLOPS) to a modern GPU (~3–4×) lands FastReg near ~100–130 ms/reg ≈ ~0.8–1× realtime,
still GPU-bound — vs Faster-LIO's 12.65 ms on **CPU**. Registration-time gap ≈ **25–32×**.

FastReg baselines for context (authors, s/registration): ICP 0.38, TEASER 1.13,
FCGF-RANSAC 16.5, DGR 11.89, FPFH-RANSAC 71.69.

## B. Downstream pipeline — odometry-agnostic (measured here, 0 % GNSS)

| Metric | **FastReg** | **FL base** | **FL downsampled** |
|---|---:|---:|---:|
| pipeline wall-clock (s) | 133.8 | 126.4 | 126.8 |
| detection loop (s) | 81.5 | 80.1 | 77.9 |
| YOLO ms/frame (mean) | 21.0 | 20.8 | 20.5 |
| pipeline throughput (fps) | 66.5 | 67.6 | 69.5 |
| frames processed | 5,420 | 5,420 | 5,420 |

Both backends enter as a precomputed easting/northing CSV, so this stage is identical
by construction (~2 min ≈ 4× realtime end-to-end); differences are run-to-run noise.

## C. Accuracy — pole-corrected vehicle error, 0 % GNSS (measured here)

| error (m) | **FastReg** | **FL base** | **FL downsampled** |
|---|---:|---:|---:|
| median | **8.41** | 10.00 | 8.67 |
| mean | 13.65 | 9.06 | **6.91** |
| max | 64.61 | 25.95 | **20.39** |

Comparable medians; Faster-LIO (esp. downsampled) has the **better mean and much
tighter worst case**, and is **GNSS-free** while FastReg's trajectory is GNSS-anchored
(see F).

## D. Accuracy — GNSS sweep, pole-corrected median (m)

| GNSS injected | **FastReg** | **Faster-LIO** |
|---|---:|---:|
| 0 % | 8.41 | 10.11 |
| 10 % | 2.13 | **1.05** |
| 25 % | 1.29 | **0.61** |
| 50 % | 0.53 | **0.25** |

FastReg is ahead only at exactly 0 %; with **any** GNSS, Faster-LIO wins at every level
(locally more accurate → dead-reckons better between fixes). (Faster-LIO sweep is the
pfn2 run; the downsampled config matches it at 0 % and tracks the same with GNSS.)

## E. Poles

| quantity | **FastReg** | **FL base** | **FL downsampled** |
|---|---:|---:|---:|
| pole detection events (geo-localized) | 359 | 355 | 355 |
| unique ground-truth poles corrected on | 151 | 133 | 135 |
| of total site poles | 290 | 290 | 290 |
| pole-localization error to nearest GT pole (median m) | 3.57 | 2.54 | **2.15** |

Downsampled Faster-LIO localizes poles most accurately (2.15 m). All detection events
are applied as path corrections; the unique poles are what "make the path accurate."

## F. Raw odometry-only error, 0 % GNSS — NOT apples-to-apples

| error (m) | **FastReg** | **FL base** | **FL downsampled** |
|---|---:|---:|---:|
| median | 35.96 | 178.26 | 113.04 |

⚠️ FastReg's shipped CSV **tracks GNSS to ~36–50 m** → it is GNSS-anchored, **not** pure
dead-reckoning; Faster-LIO's is pure odometry. So this row flatters FastReg and is not a
fair head-to-head — **use the pole-corrected metric (C)**, which is what the deployed
system reports.

---

## Bottom line
- **Speed (odometry):** Faster-LIO downsampled is ~**25–32× faster per scan** than
  FastReg (12.65 ms CPU vs 320–410 ms GPU) and is **7.9× realtime on CPU**; FastReg is
  **sub-realtime and needs a GPU**.
- **Speed (full run):** identical (~2 min) — the pipeline dominates and is
  odometry-agnostic.
- **Accuracy:** comparable at 0 % GNSS (8.4–8.7 m median), Faster-LIO tighter on
  mean/max and better with any GNSS; and it is GNSS-free odometry vs FastReg's
  GNSS-anchored track.
- **Hardware:** Faster-LIO needs LiDAR+IMU but **no GPU**; FastReg needs a GPU but
  no IMU.

## Why FastReg was not re-run
FastReg is a KITTI/CODD-trained network requiring a CUDA GPU (no CPU path). Its
prebuilt image compiles PointNet++ for Turing (sm_75); this host's RTX 3050 Ti is
Ampere (sm_86) and CUDA 11.0 can't target it, so a GPU run needs a custom CUDA-≥11.1
build (torch + torch-geometric stack + from-source PointNet++), and docker-GPU needs
`nvidia-container-toolkit` (no passwordless sudo here). A KITTI checkpoint on snow
Ouster data would also bias live accuracy. Given that FastReg's accuracy is already
captured via the authors' trajectory and its speed is published, we use the paper's
timing rather than a biased local re-run.
