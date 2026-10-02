# Snow-Pole Vehicle Localization — LiDAR Odometry Benchmark (combined repo)

A single, self-contained repository for the snow-pole-based vehicle localization
pipeline and the **three interchangeable LiDAR-odometry backends** benchmarked
against the original **FastReg** baseline:

| Backend | Engine | Needs |
|---|---|---|
| **Faster-LIO** | CPU iVox + iESKF | Docker (ROS 1 Noetic image) |
| **GLIM** | GPU VGICP + factor graph | Docker + NVIDIA GPU + CUDA image (~14 GB) |
| **Super-LIO** | CPU IESKF + OctVox + HKNN | ROS 2 Jazzy + `colcon` (built from the vendored `Super-LIO/`) |

Each backend produces the **same** `easting,northing` CSV that the snow-pole
pipeline consumes unchanged, so they can be compared 1:1 on speed and accuracy.

> **The dataset (~47 GB) is NOT in this repo — download it first (see below).**
> The source tree is ~29 MB; the sensor bags, build artifacts, virtualenvs and all
> regenerable outputs are git-ignored.

---

## 1. Repository layout

```
Fasterlio/                               ← repo root (clone this)
├── README.md                            ← this file
├── .gitignore
├── Snow-pole-based-vehicle-localization/   ← the pipeline + the 3 integrations
│   ├── snowpole_based_vehicle_localization.py            (main pipeline)
│   ├── snowpole_based_vehicle_localization_GNSS_percentage.py
│   ├── model/pole_best_signal.pt         (YOLO snow-pole detector, shipped)
│   ├── incremental_navigation_results.csv (FastReg baseline odometry, shipped)
│   ├── Groundtruth_pole_location_*.csv   (290 ground-truth poles, shipped)
│   ├── Trip068.json  environment.yml  README.md
│   ├── fasterlio_integration/   (scripts, configs, docker/, SUMMARY.md)
│   ├── glim_integration/        (scripts, configs, docker/, summary.md)
│   ├── superlio_integration/    (scripts, config, summary.md)
│   └── snow_pole_geo_localization_data/  ← YOU download the bags here (git-ignored)
├── Super-LIO/                           ← vendored Super-LIO source (ROS 2 Jazzy)
│   └── src/ …                             incl. local mods (QoS fix + livox shim)
└── fastreg/                             ← vendored FastReg source (baseline, optional)
```

The per-backend folders each have their own `README.md`, `PLAN.md`,
`summary.md`/`SUMMARY.md` and a comparison doc — read those for deep detail.

---

## 2. Download the dataset (do this first)

The sensor bags are published on **Kaggle** (folder `snow_pole_geo_localization_data`,
DOI **https://doi.org/10.34740/KAGGLE/DSV/14311103**). Two files:

| File | Size | Needed for |
|---|---|---|
| `2024-02-28-12-59-51_no_unwanted_topics.bag` | **5.71 GB** | the snow-pole pipeline (camera / range images for YOLO) — **minimum to run** |
| `2024-02-28-12-59-51.bag` | **41.24 GB** | regenerating a backend's odometry (GNSS + IMU clock for the bridge) |

Place whichever you download into:

```
Snow-pole-based-vehicle-localization/snow_pole_geo_localization_data/
```

**Manual download (recommended):** open the Kaggle dataset page (DOI link above),
download the bag file(s), and move them into the folder above.

**Or via the Kaggle CLI** (needs a Kaggle account + API token at
`~/.kaggle/kaggle.json`, and the dataset *slug* from its URL `kaggle.com/datasets/<slug>`):

```bash
pip install kaggle
mkdir -p Snow-pole-based-vehicle-localization/snow_pole_geo_localization_data
kaggle datasets download -d <slug> \
  -p Snow-pole-based-vehicle-localization/snow_pole_geo_localization_data --unzip
```

**What you need depending on your goal:**
- *Just see the pipeline work:* download only the **5.71 GB** bag — the repo already
  ships the FastReg odometry CSV (`incremental_navigation_results.csv`), so the
  snow-pole pipeline runs with no odometry step.
- *Reproduce a backend end-to-end (regenerate its CSV):* also download the **41.24 GB**
  bag and set up that backend's stack (§3).

---

## 3. One-time setup

**Quick path — run the bootstrap from the repo root:**

```bash
./setup.sh                   # venv + polegeo conda env + build Super-LIO + verify/guide
./setup.sh --apt             # also sudo-apt the Super-LIO (ROS 2 Jazzy) build deps
./setup.sh --fasterlio-image # also build the Faster-LIO Docker image
./setup.sh --glim-image      # also pull the GLIM CUDA image (~10–15 GB)
./setup.sh --all-images      # both Docker images
```

`setup.sh` is idempotent: it does the automatable pieces, skips anything already
present, and ends with a report of what's **Ready** vs **Still needed** (with the
exact command for each remaining step). It never auto-installs ROS 2 Jazzy, the
NVIDIA toolkit, or the dataset — those it detects and guides. The manual equivalents
are documented below.

### 3.1 Python environments (needed by every backend)

```bash
# (a) the pipeline + visualization env (conda).  Creates env `polegeo`.
bash Snow-pole-based-vehicle-localization/fasterlio_integration/scripts/setup_polegeo_env.sh
#     → $HOME/miniconda3/envs/polegeo  (needs miniconda installed)

# (b) the bag-reading venv used by the odometry→CSV bridges (pure-python rosbags).
python3 -m venv .baginspect_venv
. .baginspect_venv/bin/activate
pip install numpy pandas scipy pyproj rosbags
deactivate
```

### 3.2 Per-backend stacks (only for the backend(s) you want to run)

- **Super-LIO** — ROS 2 Jazzy host, then build the vendored source:
  ```bash
  sudo apt install -y python3-colcon-common-extensions ros-jazzy-pcl-ros libgflags-dev
  cd Super-LIO && colcon build && cd ..
  ```
  Super-LIO consumes a **ROS 2 bag**, which is produced from the raw ROS 1 bag by a
  lightweight converter (uses only `.baginspect_venv`, **no Docker/GPU**). Run it once
  (needs the 41.24 GB bag downloaded); Super-LIO then reuses the result:
  ```bash
  cd Snow-pole-based-vehicle-localization
  glim_integration/scripts/00_convert_bag.sh      # -> glim_integration/output/ros2_bag
  ```
  (The `run_full_pipeline_superlio.sh --from-bag` run will otherwise stop with a
  message telling you to do exactly this.)
- **Faster-LIO** — Docker (the integration builds a ROS 1 Noetic image on first run):
  see `Snow-pole-based-vehicle-localization/fasterlio_integration/README.md`.
- **GLIM** — Docker + NVIDIA Container Toolkit + GPU; pull the CUDA image:
  see `Snow-pole-based-vehicle-localization/glim_integration/README.md`.

> `setup_polegeo_env.sh` and the venv are cross-platform; the ROS 2 / Docker / CUDA
> stacks are OS-specific (Ubuntu 24.04 + ROS 2 Jazzy for Super-LIO). If a prerequisite
> is missing, the run scripts fail fast with a clear message telling you what to install.

---

## 4. Run a pipeline

All commands are run **from `Fasterlio/`**. Each backend has a one-command runner
that does odometry → CSV bridge → snow-pole localization → final animation
(odometry stage is skipped automatically if its CSV already exists).

```bash
cd Snow-pole-based-vehicle-localization

# Super-LIO  (fastest; CPU, ROS 2 Jazzy)
superlio_integration/run_full_pipeline_superlio.sh            # reuse CSV if present
superlio_integration/run_full_pipeline_superlio.sh --from-bag # regenerate odometry

# Faster-LIO (CPU, Docker ROS 1)
fasterlio_integration/run_full_pipeline_fasterlio.sh [--from-bag]

# GLIM       (GPU, Docker CUDA)
glim_integration/run_full_pipeline_glim.sh [--from-bag]
```

Just the snow-pole pipeline on a given odometry CSV (no odometry step):

```bash
cd Snow-pole-based-vehicle-localization
env -u PYTHONPATH MPL_BACKEND=Agg \
    INCREMENTAL_NAV_CSV=incremental_navigation_results.csv \
    RESULTS_CSV=snowpole_results.csv \
    ~/miniconda3/envs/polegeo/bin/python snowpole_based_vehicle_localization.py
```

Outputs (CSVs, figures, the `temporal_evolution_*.mp4` animation, timing JSONs) land
in each backend's `output/` folder — all git-ignored and fully regenerable.

---

## 5. Results & documentation

- Head-to-head speed/accuracy and the full "map" of each integration:
  - `superlio_integration/summary.md`  (+ `SUPERLIO_VS_FASTERLIO_COMPARISON.md`)
  - `glim_integration/summary.md`      (+ `GLIM_VS_FASTERLIO_COMPARISON.md`)
  - `fasterlio_integration/SUMMARY.md`  (+ `FASTREG_VS_FASTERLIO_COMPARISON.md`)
- Headline: **Super-LIO** is the fastest (~227 FPS, CPU) and the most accurate with
  any GNSS; **Faster-LIO** is a strong CPU option; **GLIM** is the GPU option.

## 6. Credits & dataset citation

Pipeline, dataset and snow-pole detector by **Bavirisetti et al.** — original project:
https://github.com/bdps1989/snowpole_geolocalization
Dataset (Kaggle, DOI 10.34740/KAGGLE/DSV/14311103) and papers are cited in
`Snow-pole-based-vehicle-localization/README.md`. Super-LIO
(`Liansheng-Wang/Super-LIO`), Faster-LIO (`gaoxiang12/faster-lio`) and GLIM
(`koide3/glim`) are the respective upstream odometry projects.
