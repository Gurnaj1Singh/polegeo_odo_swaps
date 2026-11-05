# SnowPole odometry benchmark

Sixteen LiDAR and LiDAR-inertial odometry methods (thirteen of which ran end to end) compared inside the snow-pole vehicle-localization pipeline of this
repository (`../Snow-pole-based-vehicle-localization/`), on the SnowPole drive (E39 Hemnekjølen, Norway, 28 Feb 2024;
Ouster OS2-128 with built-in IMU, dual GNSS; 10.3 km, 5,420 frames). Every method's trajectory goes through the
same bridge, the same snow-pole detector, the same correction and the same scoring, with two pole detectors:

- **YOLOv5 signal** (`polegeo/model/pole_best_signal.pt`), the detector of the original pipeline;
- **YOLO11n Combination 4** (`models/yolo11n_comb4_final.pt`), trained here on the SnowPole Detection dataset's
  near-IR + signal + reflectivity input.

It also contains the IMU studies, the analysis and visualization code, and a per-stage runtime study.

Everything in this folder is a **byte-identical copy** of the files that produced the results (SHA-256 of each in
[`MANIFEST.sha256`](MANIFEST.sha256)). The only files written afterwards are this README, `.gitignore` and
`third_party/fetch_third_party.sh`. File paths inside the scripts are relative, which is why the folder keeps the
layout it was run in.

---

## Contents

1. [Results](#results)
2. [Methods](#methods)
3. [Folder layout](#folder-layout)
4. [Reproducing the results](#reproducing-the-results)
5. [Analyses and figures](#analyses-and-figures)
6. [Runtime study](#runtime-study)
7. [Reproducibility notes](#reproducibility-notes)
8. [What is not included](#what-is-not-included)

---

## Results

Median position error after snow-pole correction (m, lower is better), GNSS seed 0, at 0/10/25/50 % GNSS availability.
Scored only on the 4.5 km section with surveyed poles (km 0.73-5.19). Full tables with venues, sources and failure
notes: [`results/snowpole_all_methods.md`](results/snowpole_all_methods.md) (YOLOv5 signal) and
[`results/snowpole_all_methods_comb4.md`](results/snowpole_all_methods_comb4.md) (YOLO11n Combination 4); seeds 1-2 in
`outputs/scores*/`.

| Category | Method | YOLOv5 signal 0 / 10 / 25 / 50 % | YOLO11n Comb. 4 0 / 10 / 25 / 50 % | Raw drift median (m) |
|---|---|---|---|---|
| Non-DL, ≤2024 | KISS-ICP | 7.70 / 0.75 / 0.42 / 0.16 | 2.29 / 0.80 / 0.47 / 0.18 | 188 |
| | MAD-ICP | 9.78 / 1.02 / 0.59 / 0.24 | 9.75 / 1.09 / 0.62 / 0.29 | 431 |
| | FAST-LIO2 | 10.25 / 1.17 / 0.65 / 0.26 | 10.03 / 1.24 / 0.68 / 0.33 | 514 |
| | Faster-LIO | 8.67 / 0.79 / 0.46 / 0.19 | 8.63 / 0.86 / 0.50 / 0.22 | 283 |
| | GLIM | 10.22 / 1.20 / 0.65 / 0.27 | 10.03 / 1.23 / 0.69 / 0.32 | 465 |
| DL/ML, ≤2024 | FastReg (original trajectory) | 8.41 / 2.13 / 1.29 / 0.53 | 8.91 / 2.26 / 1.33 / 0.60 | – |
| | FastReg (run here) | 11.01 / 2.19 / 0.91 / 0.38 | 11.26 / 2.11 / 0.95 / 0.46 | 775 |
| | DeepPointMap | 2144.89 / 3.67 / 1.41 / 0.52 | 2136.35 / 3.59 / 1.39 / 0.57 | 632 |
| | PIN-SLAM | 34.00 / 1.74 / 0.82 / 0.34 | 18.16 / 1.66 / 0.85 / 0.38 | 581 |
| Non-DL, 2025-26 | **RKO-LIO** | **1.20 / 0.60 / 0.33 / 0.12** | **1.14 / 0.66 / 0.38 / 0.14** | **99** |
| | GenZ-ICP | 9.84 / 0.97 / 0.57 / 0.23 | 9.56 / 1.05 / 0.62 / 0.27 | 355 |
| | Super-LIO | 9.64 / 0.90 / 0.53 / 0.20 | 9.38 / 0.99 / 0.58 / 0.25 | 371 |
| DL/ML, 2025-26 | BUFFER-X | 15.98 / 4.85 / 1.83 / 0.70 | 61.68 / 5.20 / 1.86 / 0.74 | 1373 |
| | UGP | 922.17 / 3.66 / 1.50 / 0.59 | 1080.43 / 3.63 / 1.48 / 0.65 | 561 |
| | CLID-SLAM | failed: lost track at scan 911 / 831 (two range settings) | | |
| | Hi-LOAM | not runnable: model code missing from the public repository | | |
| Extra | Splat-LOAM | failed: lost track, stopped at t≈278 s of 542 s | | |

Detector (YOLO11n, Combination 4, test split): P 0.910, R 0.871, mAP50 0.917, mAP50-95 0.439
([`results/detector_yolo11n_comb4.md`](results/detector_yolo11n_comb4.md)). Its training images are frames 844-2999 of
this drive ([`results/detector_dataset_frame_map.csv`](results/detector_dataset_frame_map.csv)), so its detections in
that stretch are optimistic; the same detections are used for every odometry method.

---

## Methods

Every method reads the same drive and uses the same 2 m / 150 m range limits where it has them. LiDAR-only trajectories
are levelled once with gravity from the accelerometer at standstill before scoring (`02_bridge_to_csv.py
--level-gravity`). Third-party code is pinned to the commit in `third_party/<name>/VENDORED_FROM.txt`.

| Method (venue) | Runs in | Built from | Runner | Method-specific scripts / config | Trajectory |
|---|---|---|---|---|---|
| KISS-ICP (RA-L 2023) | Windows venv | `env/windows_venv_requirements.lock.txt` | `run_all.sh` | `scripts/01_run_lidar_odometry.py` | `outputs/kiss_icp/` |
| MAD-ICP (RA-L 2024) | Docker `odom-bench:1.0` | `docker/Dockerfile` + `third_party/sdist/` | `run_all.sh` | `scripts/01_run_lidar_odometry.py` | `outputs/mad_icp/` |
| GenZ-ICP (RA-L 2025) | Docker `odom-bench:1.0` | `docker/Dockerfile` + `third_party/sdist/` | `run_all.sh` | `scripts/01_run_lidar_odometry.py` | `outputs/genz_icp/` |
| RKO-LIO (2025) | Windows venv | `third_party/wheels/` | `run_all.sh` | `configs/rko_lio_os2_128.yaml` | `outputs/rko_lio/` |
| Faster-LIO (RA-L 2022) | Docker `fasterlio:noetic` | `docker/Dockerfile.fasterlio` | `scripts/14_run_ros_methods.sh fasterlio` | config of `../Snow-pole-based-vehicle-localization/fasterlio_integration` | `outputs/fasterlio/` |
| FAST-LIO2 (T-RO 2022) | Docker `fastlio2:noetic` | `docker/Dockerfile.fastlio2` | `run_fastlio2_snowpole.sh` | `scripts/21_fastlio2_in_container.sh`, `configs/fastlio2/` | `outputs/fastlio2/` |
| Super-LIO (RA-L 2026) | Docker `superlio:jazzy` | `docker/Dockerfile.superlio` | `scripts/14_run_ros_methods.sh superlio` | `scripts/15_ros2_odom_to_tum.py`, `../Super-LIO` | `outputs/superlio/` |
| GLIM (RAS 2024) | Docker `koide3/glim_ros2` | digest in `env/glim_image_digest.txt` | `scripts/14_run_ros_methods.sh glim` | config of `../Snow-pole-based-vehicle-localization/glim_integration` | `outputs/glim/` |
| FastReg (RA-L 2021) | Docker `fastreg:cu128` | `docker/Dockerfile.fastreg` | `run_fastreg_snowpole.sh` | `scripts/07_run_fastreg.py` | `outputs/fastreg/` |
| FastReg, original trajectory | – | – | `run_all.sh` (scoring only) | `fixed_inputs/incremental_navigation_results_fastreg.csv` | – |
| PIN-SLAM (T-RO 2024) | Docker `pin_slam:cu128` | `docker/Dockerfile.pin_slam` | `run_new_methods.sh` | `configs/pin_slam_snowpole.yaml`, `scripts/08_kitti_poses_to_tum.py` | `outputs/pin_slam/` |
| DeepPointMap (AAAI 2024) | Docker `deeppointmap:cu128` | `docker/Dockerfile.deeppointmap` | `run_new_methods.sh` | `configs/deeppointmap_snowpole.yaml`, `scripts/08_kitti_poses_to_tum.py` | `outputs/deeppointmap/` |
| BUFFER-X (ICCV 2025) | Docker `bufferx:cu128` | `docker/Dockerfile.bufferx` | `run_new_methods.sh` | `scripts/09_export_kitti_bins.py`, `scripts/10_run_bufferx.py` | `outputs/bufferx/` |
| UGP (CVPR 2025) | Docker `ugp:cu128` | `docker/Dockerfile.ugp` | `run_ugp_rerun.sh` | `scripts/09_export_kitti_bins.py`, `scripts/11_run_ugp.py` | `outputs/ugp/` |
| CLID-SLAM (RA-L 2025) | Docker `clidslam:cu128` | `docker/Dockerfile.clidslam` | `run_clidslam.sh` | `scripts/06_export_snowpole_imu.py`, `scripts/16_export_clid_sequence.py`, `configs/clid_slam_snowpole.yaml` | failed, `outputs/clidslam_attempts/` |
| Hi-LOAM (TMM 2026) | Docker `hiloam:cu128` | `docker/Dockerfile.hiloam` | `run_new_methods.sh` | `configs/hiloam_snowpole.yaml` | not runnable, `outputs/hiloam/run.log` |
| Splat-LOAM (ICCV 2025) | Docker `splatloam:cu128` | `docker/Dockerfile.splatloam` | `run_splatloam.sh` | `configs/splatloam_snowpole.yaml` | failed, `outputs/splatloam_attempts/` |

`docker/build_queue.sh` builds the method images one at a time.

### Shared pipeline

| Step | Script | Output |
|---|---|---|
| Point clouds from the bag (xyz + per-point time; reflectivity for FastReg) | `scripts/00_extract_scans.py` | `cache/scans*/` (not included, regenerated) |
| IMU table + extrinsics for the scan cache | `scripts/06_export_snowpole_imu.py` | `cache/imu.csv`, `cache/extrinsics.yaml` |
| Trajectory → per-GNSS-frame UTM CSV (2-D rigid fit on the first 20-420 m; gravity levelling for LiDAR-only methods) | `scripts/02_bridge_to_csv.py` | `outputs/csv/incremental_navigation_results_<key>.csv` + `.align.json` |
| Pole detections, once per detector (YOLOv5 signal, or YOLO11n Combination 4 with `--input comb4`) | `scripts/03_cache_detections.py` | `cache/detections.pkl`, `cache_comb4/detections.pkl` |
| Snow-pole correction at a GNSS percentage and seed | `scripts/04_gnss_sweep.py` | `outputs/scores*/<Name>_seed<s>.csv` |
| Bridge + sweep for one method (seed 0 at 0/10/25/50 %, seeds 1-2 at 10/25/50 %) | `scripts/13_score_method.sh` | |
| All methods into one table | `scripts/12_collect_all_methods.py [--detector comb4]` | `results/snowpole_all_methods*.{csv,md}` |
| Check the original five methods against frozen scores | `scripts/05_check_results.py` | `results/expected_scores.csv` |

### Pole detector (YOLO11n, Combination 4)

| Step | Script / runner | Output |
|---|---|---|
| Train COCO-pretrained vs from-scratch YOLO11n (≤500 epochs, early stop 50) | `scripts/17_train_yolo11n_comb4.py`, `run_comb4_train.sh` | `models/comb4_*_s0/` (eval, curves, test predictions) |
| Statistical comparison and choice of the detector | `scripts/20_compare_yolo_init.py` | `models/yolo11n_comb4_final.pt`, `results/detector_yolo11n_comb4.*` |
| Dataset image → bag frame map | `scripts/18_dataset_frame_map.py` | `results/detector_dataset_frame_map.csv` |
| Detections over the drive, signal vs Combination 4 | `scripts/03_cache_detections.py --input comb4`, `scripts/19_compare_detections.py`, `run_comb4_rescore.sh` | `results/detections_signal_vs_comb4.md`, `results/snowpole_signal_vs_comb4.md` |

The training run used seed 0 per init (seeds 1-2 were stopped; see `results/detector_yolo11n_comb4.md`).

### IMU studies (FAST-LIO2 / Faster-LIO)

| Study | Script / runner | Output |
|---|---|---|
| IMU stream and its native noise at standstill | `paper_ipas/analysis/imu_extract.py` | `paper_ipas/data/imu.npz`, `imu_native_noise.json` |
| IMU at 50 / 25 / 10 Hz and 3× / 10× noise (100 Hz = control) | `scripts/22_make_imu_variants.py`, `scripts/23_lio_imu_variant.sh`, `run_imu_degradation.sh` | `outputs/imu_degradation/*`, `outputs/scores_imu/` |
| IMU noise parameters from the measured noise | `configs/imu_tuning/{gyrcov,imucov}/`, `scripts/24_lio_imu_config.sh`, `run_imu_tuning.sh` | same |
| 6 s IMU initialisation; + gyro bias held fixed | `docker/Dockerfile.lio_init6s`, `run_imu_init.sh`, `run_imu_biasfix.sh` | same |
| IMU-only dead reckoning, gyro heading + LiDAR distance, FastReg without levelling | `paper_ipas/analysis/phase3_imu_only.py` | `outputs/imu_only/`, `outputs/scores_imu/` |

---

## Folder layout

```
odometry_benchmark/
├── README.md, MANIFEST.sha256, .gitignore, .dockerignore
├── run_*.sh                  drivers (one per method group / study; see the tables above)
├── scripts/                  00-24: pipeline steps and method runners
├── docker/                   one Dockerfile per method image
├── configs/                  method configs (incl. imu_degradation/, imu_tuning/)
├── env/                      lock files: Windows venv, Docker image, data checksums, GLIM image digest
├── polegeo/                  original pipeline helpers + YOLOv5 signal model, Trip068.json, surveyed poles
├── fixed_inputs/             original FastReg trajectory (as shipped with the original pipeline)
├── third_party/              VENDORED_FROM.txt per method, fetch_third_party.sh, PyPI sdists, RKO-LIO wheel
├── models/                   YOLO11n Combination 4 detector + training/evaluation records
├── cache/, cache_comb4/      small derived inputs: detections, kriged GNSS, scan stamps, IMU table
├── outputs/                  per-method trajectories (TUM), timings, bridged CSVs, scores
├── results/                  final tables
├── paper_ipas/               analysis/ (code), data/ (computed tables), figures/
└── runtime_study/            stage-timed copy of the original localization script + results
```

---

## Reproducing the results

### Requirements

- Windows 11 with Git Bash, Python 3.11 venv at `../venv` (repository root) from `env/windows_venv_requirements.lock.txt`.
  The drivers call `../venv/Scripts/python`.
- Docker Desktop (WSL 2) with an NVIDIA GPU for the learned methods and the detectors (run here on an RTX 5050 Laptop
  GPU, CUDA 12.8). One Docker / GPU job at a time.
- The SnowPole data, placed at the repository root (checksums in `env/data_sha256.txt`; see `../README.md` for the
  download):
  ```
  ../snow_pole_geo_localization_data/2024-02-28-12-59-51.bag                       (full bag, 41 GB)
  ../snow_pole_geo_localization_data/2024-02-28-12-59-51_no_unwanted_topics.bag
  ../Trip068.json
  ../Groundtruth_pole_location_at_test_site_E39_Hemnekjlen.csv
  ```
  `run_all.sh` hard-links the full bag into `bag_full/`. Super-LIO and GLIM read a ROS 2 conversion of the full bag at
  `C:/datasets/snowpole/ros2_bag` (path set in `scripts/14_run_ros_methods.sh`; conversion as in
  `../Snow-pole-based-vehicle-localization/glim_integration/scripts/00_convert_bag.sh`).
- For the YOLO11n training only: the SnowPole Detection dataset (`combined_color` images and labels).

### Steps

```bash
cd odometry_benchmark
bash third_party/fetch_third_party.sh   # method code at the pinned commits (+ UGP checkpoints, see below)
bash docker/build_queue.sh              # build the method images

bash run_all.sh                         # KISS-ICP, MAD-ICP, GenZ-ICP, RKO-LIO, scan extraction, YOLOv5 detections,
                                        # scoring; exits 0 only if all scores match results/expected_scores.csv
bash scripts/14_run_ros_methods.sh fasterlio    # likewise superlio, glim; then scripts/13_score_method.sh
bash run_new_methods.sh                 # PIN-SLAM, DeepPointMap, BUFFER-X, Hi-LOAM (+ scoring)
bash run_ugp_rerun.sh
bash run_fastreg_snowpole.sh
bash run_fastlio2_snowpole.sh
bash run_clidslam.sh; bash run_splatloam.sh     # both fail on this drive (see Results)

bash run_comb4_train.sh                 # YOLO11n Combination 4 (needs the detection dataset)
bash run_comb4_rescore.sh               # all methods with the YOLO11n detector
../venv/Scripts/python scripts/12_collect_all_methods.py
../venv/Scripts/python scripts/12_collect_all_methods.py --detector comb4
```

**Re-scoring without the data.** The trajectories, bridged CSVs, detections and GNSS reference are included, so the
correction and every score can be recomputed without the bag, e.g.

```bash
../venv/Scripts/python scripts/04_gnss_sweep.py --csv outputs/csv/incremental_navigation_results_kiss_icp.csv \
    --name KISS-ICP --seed 0 --pcts 0 10 25 50 --cache-dir cache_comb4 --out-dir /tmp/scores
```

**UGP checkpoints** are distributed via Google Drive (folder in `third_party/UGP/VENDORED_FROM.txt`); put
`kitti.pth.tar` and `nuscenes.pth.tar` into `third_party/UGP/ckpts/` and re-run `fetch_third_party.sh`, which checks
them against `third_party/UGP/ckpts/SHA256SUMS`.

---

## Analyses and figures

Code in `paper_ipas/analysis/`, computed tables in `paper_ipas/data/`, figures in `paper_ipas/figures/`. The scripts
re-use `scripts/04_gnss_sweep.py`; `phase2.py`, `phase3.py` and `fig_traj.py` assert that they reproduce the scores in
`outputs/scores*/` and `results/` before writing anything.

| Script | Produces |
|---|---|
| `phase2.py` | drift / heading / turn-rate / distance errors, GNSS sweep at 0-50 % (10 levels, 3 seeds), modality ablation (with and without poles), pole-association diagnosis; `fig_drift`, `fig_gnss` |
| `phase3.py` | ways of using the IMU and IMU degradation tables; `fig_imu` |
| `phase3_imu_only.py` | IMU-only, gyro heading + LiDAR distance, FastReg without levelling (trajectories + scores) |
| `imu_extract.py` | IMU stream and native noise |
| `fig_traj.py` (+ `osm_basemap.py`) | ground truth vs FastReg vs KISS-ICP trajectories, whole drive and a fixed 400 m stretch in the middle of the pole section: `fig_traj` (10 % GNSS), `fig_traj_gnss0` (`--pct 0`); `--sat` draws Esri World Imagery under it (tiles downloaded and cached on first use, not included) |

```bash
../venv/Scripts/python paper_ipas/analysis/phase2.py
../venv/Scripts/python paper_ipas/analysis/fig_traj.py --pct 10 --sat
```

---

## Runtime study

`runtime_study/` times the stages of the original localization pipeline per frame (LiDAR odometry / pole geolocation /
vehicle geolocation) for FastReg and KISS-ICP with the YOLOv5 signal model, using the original script itself with stage
timers added (`timed_GNSS_percentage.py` vs `original_GNSS_percentage.py`). Results, stage definitions and what is and
is not included (ROS bag decoding, voxel sampling, CUDA synchronisation, warm-up):
[`runtime_study/RESULTS.md`](runtime_study/RESULTS.md).

| ms per frame (pole section) | FastReg | KISS-ICP |
|---|---|---|
| LiDAR odometry | 254.0 | 10.9 |
| Pole geolocation | 19.7 | 19.4 |
| Vehicle geolocation | 16.3 | 13.3 |
| Total | 290.0 | 43.6 |

The KISS-ICP `ms/scan` in `results/` (54.0) is the single-threaded, core-pinned run of `run_all.sh`, measured while
GenZ-ICP and MAD-ICP ran alongside; 10.9 ms is the default multithreaded run on an idle machine.

---

## Reproducibility notes

- **Byte-identical code.** `MANIFEST.sha256` lists every copied file; `sha256sum -c MANIFEST.sha256` checks them.
- **Third-party code.** `fetch_third_party.sh` restores each method at its pinned commit (and git submodules at the
  commits the parent pins). Before publishing, the restored code was compared file by file with the copies that
  produced the results: identical, apart from line endings (the originals were checked out on Windows with CRLF).
  FastReg and the original pipeline / Super-LIO come from this repository at commit `a8510ad`.
- **Pinned environments.** `docker/Dockerfile` pins its base image by digest and every package by
  `env/docker_requirements.lock.txt`; GenZ-ICP and MAD-ICP are built from the SHA-256-checked sdists in
  `third_party/sdist/`. Method Dockerfiles pin upstream repositories by commit. GLIM by image digest.
- **Determinism.** RKO-LIO and MAD-ICP are bit-identical run to run. KISS-ICP and GenZ-ICP sum with
  `tbb::parallel_reduce`, so they are run pinned to one CPU in `run_all.sh` to be bit-identical. FAST-LIO2 and the IMU
  variants are fed by `rosbag play` at 1×; a control run reproduced the offline Faster-LIO scores to within 0.03 m.
- **Network.** `03_cache_detections.py` loads YOLOv5 through `torch.hub` at the pinned commit `ultralytics/yolov5@0675a31`.
- **Scoring frames.** Errors are scored only where the estimated position is inside the surveyed pole area, and
  against the same GNSS track that supplies the fixes.

## What is not included

- The SnowPole bags, extracted scans, the detection dataset and the training-image copy (`data/`).
- The prebuilt Docker image tarball (725 MB) - rebuild with `docker/Dockerfile`.
- Third-party method code (restored by `third_party/fetch_third_party.sh`) and UGP checkpoints (Google Drive).
- Intermediate run artefacts (GLIM maps, rosbag recordings, IMU variant bags, detector training batches, logs other
  than those cited as evidence), YOLO11n COCO starting weights (`yolo11n.pt`, downloaded by Ultralytics), aborted
  training runs, public-dataset experiments, satellite tiles, and manuscript files.
