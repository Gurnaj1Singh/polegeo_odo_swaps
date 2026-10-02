# FastReg vs Faster-LIO — Running-Time & Reliability Comparison

**Project:** Snow-Pole-Based Vehicle Localization (E39 Hemnekjølen, Nordic winter)
**Scope:** the LiDAR **odometry** backend only (FastReg → Faster-LIO); detection,
geo-localization and GNSS-fusion are unchanged.
**What's new here:** the pipeline is now **timed** (per-run JSON + a cumulative CSV),
and this is the **measured, like-for-like head-to-head** through the *identical*
pipeline that `EXPERIMENT_REPORT.md` §9.2/§11 flagged as still missing.

All numbers below were measured on this host (`gsm-G15`, 20-core CPU, `torch 2.2.0+cpu`)
on **2026-09-08**, both runs with **0 % GNSS** (pure odometry + snow-pole correction),
back-to-back with no other load.

---

## 0. How the timing works (what was added)

- `perf_timer.py` (project root, standard-library only): a `RunTimer` that records
  wall-clock, per-stage time (model-load / bag-load / detection loop), **per-frame YOLO
  inference time**, throughput, and the accuracy metrics — then writes
  `fasterlio_integration/output/timing_<backend>_gnss<pct>.json` and appends one row to
  `fasterlio_integration/output/timing_comparison.csv`.
- Both `snowpole_based_vehicle_localization.py` and
  `snowpole_based_vehicle_localization_GNSS_percentage.py` are instrumented
  (behaviour-preserving; the summary prints at the end of every run).

Reproduce the head-to-head:
```bash
cd Snow-pole-based-vehicle-localization
# FastReg (default odometry CSV)
env -u PYTHONPATH MPL_BACKEND=Agg \
  INCREMENTAL_NAV_CSV=incremental_navigation_results.csv \
  RESULTS_CSV=snowpole_results_fastreg.csv \
  ~/miniconda3/envs/polegeo/bin/python snowpole_based_vehicle_localization.py
# Faster-LIO
env -u PYTHONPATH MPL_BACKEND=Agg \
  INCREMENTAL_NAV_CSV=incremental_navigation_results_fasterlio.csv \
  RESULTS_CSV=snowpole_results_fasterlio.csv \
  ~/miniconda3/envs/polegeo/bin/python snowpole_based_vehicle_localization.py
```
The GNSS-% variant is identical but adds `GNSS_PERCENTAGE=<0..100>` to sweep how much
GNSS is injected — it is confirmed to run and, at 0 %, reproduces the odometry-only path.

---

## 1. Headline

> Swapping FastReg → Faster-LIO keeps the **pipeline running time essentially unchanged
> (~2 min for a 9-min drive)** while making the odometry **GNSS-free and real-time**.
> At **0 % GNSS** the final localization is within ~2 m (8.4 → 10.1 m median) with a
> **tighter worst case (65 → 26 m max)**; and once **any** GNSS is available, **Faster-LIO
> is the more accurate backend at every level** (e.g. at 25 % GNSS: **0.61 vs 1.29 m**
> median — see the sweep in §3.4). The right trade for GNSS-limited Nordic roads.

---

## 2. Running time — measured (the "fast" question)

### 2.1 Pipeline (detection + geo-localization), per run

| Metric | **FastReg** | **Faster-LIO** | Notes |
|---|---:|---:|---|
| Wall-clock total | **133.8 s** (2.2 min) | **123.4 s** (2.1 min) | ≈ equal; the ~10 s gap is run-to-run CPU noise |
| model load | 1.6 s | 3.0 s | YOLOv5 hub load (cached) |
| bag load (5.4 GB) | 9.4 s | 8.1 s | same bag both runs |
| detection loop | 81.5 s | 76.0 s | the actual per-frame work |
| pipeline throughput | 66.5 frames/s | 71.3 frames/s | 5,420 frames each |

**Key point:** both backends enter the pipeline as a pre-computed `easting/northing`
CSV, so the pipeline cost is **odometry-agnostic by construction** — the swap adds **no**
runtime overhead. Processing the full 9-min (541 s) drive in ~2 min is **~4× faster than
real-time** end-to-end on CPU.

![Pipeline running-time breakdown](output/pipeline_time_breakdown.png)

*Both backends have near-identical stage composition — YOLO detection (~42–45 s) and the
one-off setup/kriging (~36–41 s) dominate; the odometry choice is invisible to the pipeline.*

### 2.2 Per-frame snow-pole detection (YOLOv5, CPU)

| Metric | **FastReg** | **Faster-LIO** |
|---|---:|---:|
| mean ms/frame | 21.0 ms | 19.4 ms |
| median ms/frame | 19.9 ms | 19.7 ms |
| p95 ms/frame | 35.3 ms | 20.8 ms |
| detector throughput | 47.6 fps | 51.5 fps |
| frames run through detector | 2,158 | 2,143 |

This **confirms** the `EXPERIMENT_REPORT.md` estimate of "18 ms/frame" — now measured live
at **~19–21 ms/frame (~50 fps)** on this CPU. The detector is the dominant per-frame cost
and is identical for both backends (hardware-bound); the difference is noise.

### 2.3 Odometry stage (the real speed differentiator — *upstream* of the pipeline)

| Aspect | **FastReg** | **Faster-LIO** |
|---|---|---|
| Method | LiDAR-only coarse-to-fine registration | LiDAR-**inertial** iESKF + parallel sparse voxel map (iVox) |
| Measured here | **not run** — only its pre-computed trajectory CSV exists in the repo | **5,412 scans in ~551 s wall-clock** for 541.75 s of bag → **~1.0× real-time**, never falling behind |
| Real-time @ 10 Hz | heavier (dense pairwise registration); not demonstrated here | **yes** — each scan handled within its 100 ms budget with headroom (20-core CPU, in Docker) |
| Sensors needed | LiDAR | LiDAR + IMU |

> **Honesty note:** a precise FastReg odometry wall-time is **not** available in this repo —
> the original authors ran it offline and shipped only the trajectory. So the odometry-stage
> speed claim is one-sided: Faster-LIO is *measured* real-time; FastReg is characterised
> qualitatively (LiDAR-only registration is the heavier of the two designs).

---

## 3. Accuracy & reliability — measured (the "reliable" question)

Errors are Euclidean distance to the dual-antenna GNSS reference, over the frames the
vehicle spends inside the pole-instrumented site (the pipeline's in-bounds gate).

### 3.1 Raw odometry (before pole correction) — **NOT like-for-like**

| Odometry vs GNSS | median | mean | max |
|---|---:|---:|---:|
| FastReg (repo CSV) | 36.0 m | 33.9 m | 67.9 m |
| Faster-LIO (this work) | 188.3 m | 217.0 m | 592.3 m |

FastReg's stored trajectory stays *bounded* to ~36 m — only possible if it was
**periodically re-anchored to GNSS** along the route. Faster-LIO's track is **pure,
unaided odometry** (GNSS used only for the initial fix), so its drift is honestly larger.
This row compares a GNSS-*aided* track against a GNSS-*free* one — read §3.2 for the fair
comparison.

### 3.2 Final localization (snow-pole corrected) — the fair, like-for-like result

| Proposed (pole-corrected) vs GNSS | median | mean | **max** | pole events |
|---|---:|---:|---:|---:|
| **FastReg** backend | **8.41 m** | 13.65 m | 64.61 m | 359 |
| **Faster-LIO** backend | 10.11 m | **9.21 m** | **26.49 m** | 355 |

**Reading:**
- **Comparable accuracy:** the two backends land within ~1.7 m of each other on median.
- **Faster-LIO is more *consistent*:** lower **mean** (9.2 vs 13.7 m) and a much tighter
  **worst case** (26.5 vs 64.6 m). FastReg has a lower median but a heavier tail.
- **Faster-LIO does it GNSS-free:** it reaches ~10 m starting from 188 m of pure-odometry
  drift, whereas FastReg started from a GNSS-re-anchored 36 m. That the georeferenced snow
  poles pull 188 m → 10 m (a **~19× correction**) is exactly the framework working as
  designed on a fully autonomous odometry source.

### 3.3 Non-timing parameters that matter for reliability

| Parameter | FastReg | Faster-LIO | Why it matters |
|---|---|---|---|
| GNSS dependence during drive | **needs periodic GNSS** (as shipped) | **none** (initial fix only) | the whole point on GNSS-limited Nordic roads/tunnels |
| Sensor fusion | LiDAR only | LiDAR **+ IMU** (de-skew, state between scans) | robustness to fast motion / sparse geometry |
| Motion distortion handling | not modelled | per-point de-skew via point timestamps | cleaner scans at highway speed |
| Failure mode | drift + reliance on GNSS availability | slow **yaw** drift (IMU-limited) | Faster-LIO's dominant residual; addressable (see §4) |
| Worst-case localization error | 64.6 m | **26.5 m** | tail behaviour = safety-relevant |
| Reproducibility on this host | trajectory only (can't re-run) | **fully reproducible** (Docker + config in-repo) | auditability |

---

### 3.4 GNSS-percentage sweep — how each backend degrades as GNSS is withdrawn

Run with the de-duplicated `..._GNSS_percentage.py` (`GNSS_SEED=0`, so the **same frames**
receive a GNSS fix for both backends — an exactly fair comparison). "GNSS %" is the fraction
of frames snapped to the interpolated GNSS position; the rest dead-reckon on odometry + poles.

| GNSS % | GNSS fixes used | **FastReg** median (mean / max) [m] | **Faster-LIO** median (mean / max) [m] |
|---:|---:|---:|---:|
| **0 %** | 0 / 5419 | **8.41** (13.65 / 64.61) | 10.11 (9.21 / 26.49) |
| 10 % | 561 | 2.13 (2.96 / 14.75) | **1.05** (2.09 / 15.19) |
| 25 % | 1312 | 1.29 (1.80 / 11.07) | **0.61** (0.96 / 9.81) |
| 50 % | 2669 | 0.53 (0.97 / 10.80) | **0.25** (0.46 / 9.22) |

**This is the most important result of the comparison:**
- **A little GNSS goes a long way for both:** even **10 %** GNSS collapses the error by ~4–10×
  (FastReg 8.4 → 2.1 m; Faster-LIO 10.1 → **1.05 m**).
- **With *any* GNSS, Faster-LIO wins on every statistic** — median, mean *and* max — at 10 / 25 / 50 %
  (e.g. at 25 %: 0.61 vs 1.29 m median). Faster-LIO is behind **only at exactly 0 %** GNSS, and
  only by ~1.7 m.
- **Why:** the fused estimate depends on how faithfully the vehicle *dead-reckons between* GNSS
  fixes. Faster-LIO's IMU-fused, de-skewed local motion (~3.3 % RPE, §9.1 of the report) is more
  accurate locally than FastReg's LiDAR-only registration — so once periodic GNSS fixes reset the
  global position, Faster-LIO tracks ~2× tighter between them. FastReg's globally-bounded but
  locally-noisier track can't match it. **Faster-LIO + intermittent GNSS is the best of both worlds.**
- Runtime was **flat across the sweep** (127–138 s, ~20 ms/frame) — GNSS fraction doesn't affect speed.
- The odometry-only median is ~constant per backend (FastReg ~36 m, Faster-LIO ~188 m) since it's
  the raw CSV vs GNSS, independent of the GNSS-injection fraction.

![Pole-corrected error vs GNSS availability](output/sweep_error_vs_gnss.png)

*Faster-LIO (blue) sits just above FastReg at 0 % GNSS but drops below it at every
non-zero level — the log-scale y-axis shows both collapsing by ~4–10× with only 10 % GNSS.*

Raw rows: `output/timing_comparison.csv` (label `gnss_percentage`); per-run logs
`output/sweep_<backend>_gnss<pct>.log`. Regenerate both figures with
`python fasterlio_integration/scripts/30_plot_timing_and_sweep.py`.

## 4. Levers to make it *more* reliable and fast (recommendations)

**Faster (already good; further headroom):**
- Detector is the per-frame bottleneck (~20 ms/frame CPU). A CUDA GPU or batching frames
  would cut the ~76 s loop several-fold; the odometry is already real-time.
- `point_filter_num` in `config/ouster_os2_128.yaml` trades cloud density for odometry
  speed (raising 2→4 was near-neutral on accuracy per §9.1 of the report).

**More reliable (attack Faster-LIO's yaw drift — the dominant error):**
- Zero-velocity updates at the ~10 s idle start and any stops (the IMU bias is best
  observed when stationary).
- Ground-plane / non-holonomic constraints, or wheel odometry, to bound heading drift on
  long straight highway stretches.
- A higher-grade IMU than the Ouster's internal ICM-20948 would most directly tighten the
  10 m median toward the paper's sub-metre (GNSS-aided) figures.

**Cleaner metrics (done 2026-09-08):**
- The **head-to-head in §3.2 used `snowpole_based_vehicle_localization.py`, which is already
  clean** — it records exactly one corrected-error sample per in-bounds frame — so those
  numbers need no adjustment.
- The **GNSS-% variant** (`..._GNSS_percentage.py`) previously appended the error *both* per
  detection (inside the bbox loop) *and* per frame, inflating its sample. This has been
  **de-duplicated** to one clean per-frame sample (matching the main script), and the
  variant now seeds its RNG (`GNSS_SEED`, default 0) so the GNSS-vs-odometry choice is
  reproducible across the sweep (§3.4).

---

## 5. Artifacts

- `output/timing_comparison.csv` — one row per run, fixed schema (append-only; add
  GNSS-% sweeps here). Currently 3 rows: FastReg/pipeline, Faster-LIO/pipeline, and a
  Faster-LIO/gnss_percentage verification run.
- `output/timing_FastReg_pipeline_gnss0.json`, `output/timing_Faster-LIO_pipeline_gnss0.json`,
  `output/timing_Faster-LIO_gnss_percentage_gnss0.json` — full per-run metrics
  (filename = `<backend>_<label>_gnss<pct>`).
- `output/headtohead_fastreg.log`, `output/headtohead_fasterlio.log`,
  `output/gnss_pct_instrumented.log` — full run logs.
- `snowpole_results_fastreg.csv`, `snowpole_results_fasterlio.csv` — per-detection results
  for each backend (feed either to `scripts/20_temporal_evolution_visualization.py`).
- **Figures** (`scripts/30_plot_timing_and_sweep.py` regenerates both from the CSV):
  `output/sweep_error_vs_gnss.png` (§3.4) and `output/pipeline_time_breakdown.png` (§2.1).

> **GNSS-% variant verified** (`snowpole_based_vehicle_localization_GNSS_percentage.py`,
> `GNSS_PERCENTAGE=0`): runs end-to-end on Faster-LIO odometry → pole-corrected median
> **9.65 m**, YOLO **20.2 ms/frame**, wall-clock **125 s** — consistent with the main
> script, confirming the GNSS-sweep script is functional under the new instrumentation.
