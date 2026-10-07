# GLIM vs Faster-LIO — speed & accuracy (2026-09-09)

**Headline:** GLIM (GPU LiDAR-inertial, RTX 3050 Ti / 4 GB) **matches Faster-LIO's
pole-corrected accuracy** (§2) but **does NOT beat it on speed on this hardware**
(§1): run like-for-like (both offline / max-speed), Faster-LIO processes the bag in
111–159 s vs GLIM's ~205 s. GLIM's "2.65× real-time" only looked like a win against
Faster-LIO's *playback-capped* run (~551 s), which is an artifact of `rosbag play`
rate, not a compute measurement. Same downstream pipeline, same bag.

## 1. Speed — odometry throughput (offline / max-speed, 541.8 s bag, i7-12700H + RTX 3050 Ti)

Like-for-like: all read the bag at max speed (Faster-LIO via `run_mapping_offline`,
GLIM via `glim_rosbag`). GLIM = GPU odometry + CPU sub/global mapping (4 GB-safe),
downsampling ~10k pts/scan; measured from GLIM's own per-scan processing timestamps.

| Backend | Engine | Downsampling | Wall-clock | Throughput |
|---|---|---|---|---|
| Faster-LIO (ds) | CPU iVox+ESIKF | pt_filter 4 / surf 1.0 | **111 s** | **79.1 FPS (7.9×)** |
| Faster-LIO (base) | CPU iVox+ESIKF | pt_filter 2 / surf 0.5 | **159 s** | **46.8 FPS (4.7×)** |
| **GLIM** | GPU VGICP + factor graph | ~10k pts/scan | **~205 s** | **26.5 FPS (2.65×)** |

- **On this hardware Faster-LIO offline is faster than GLIM** (111–159 s vs ~205 s),
  despite GLIM downsampling harder and using the GPU. No 4 GB VRAM OOM.
- "2.65×" is 2.65× **real-time**, not vs Faster-LIO — Faster-LIO offline is 4.7–7.9×.
- ⚠️ The earlier ~551 s figure for Faster-LIO was its `rosbag play` run, capped at
  ~1.0× by the playback rate — an artifact of how it's driven, not its compute limit.

### Total time, raw bag → localization result (repeat run; traj→CSV bridge ≈ equal, omitted)

| Stage | Faster-LIO (ds) | Faster-LIO (base) | GLIM |
|---|---|---|---|
| one-time bag convert (ROS1→ROS2) | — | — | 240 s |
| odometry | 111 s | 159 s | ~205 s |
| pipeline / localization (odometry-agnostic) | ~127 s | ~127 s | ~118 s |
| **repeat-run total** | **~238 s** | **~286 s** | **~323 s** (+240 s first run) |

## 2. Accuracy — pole-corrected median error vs GNSS availability

Identical pipeline for all three backends; GNSS frames seeded (`GNSS_SEED=0`) so
the same frames get GNSS at each level.

| GNSS used | FastReg | Faster-LIO | **GLIM** |
|---|---|---|---|
| 0 %  | 8.41 m | 8.70 m | **10.18 m** |
| 10 % | 2.13 m | 0.80 m | **1.14 m** |
| 25 % | 1.29 m | 0.46 m | **0.63 m** |
| 50 % | 0.53 m | 0.19 m | **0.26 m** |

(Pole-corrected median, metres. GLIM odometry-only median at 0 % = 170 m vs
Faster-LIO 118 m — both are IMU-limited drift, corrected downstream by the poles.)

- **GLIM closely tracks Faster-LIO across the whole sweep** (within ~0.1–0.3 m at each
  GNSS level; ~1.5 m at the pathological 0 %) — the speedup costs essentially no accuracy.
- 0 % GNSS is the hardest case for any LIO backend; once ≥10 % GNSS is available
  the pole-correction collapses error ~5–10× and both GLIM and Faster-LIO beat
  FastReg. This reproduces the Faster-LIO study's finding, now for GLIM.
- Pipeline wall-clock is odometry-agnostic (~130 s, 67 fps, YOLO ~20 ms/frame).

**Distinct poles vs. detection events.** GLIM's run produces **356 geo-localization
events across 134 distinct ground-truth poles** (of 290 at the site; Faster-LIO: 355
events / 135 poles). One physical pole yields several events because the vehicle drives
*past* it — the detector re-acquires the same pole on every frame it stays in view and
within the 5 m range gate (~2–3 consecutive frames, mean **2.7 events/pole**, up to 5).
Each re-sighting is an independent fix that re-anchors the drifting track, so all events
are kept and the error is reported per event, not per unique pole.

## 3. Caveats / notes

- **Global drift**: GLIM raw odometry-vs-GNSS start-anchored drift is median
  ~450 m (≈ Faster-LIO), with a heavier tail (mean 756 / max 2588 m). Corrected
  downstream by poles + GNSS, so it doesn't move the pole-corrected number.
- **IMU**: GLIM logs "IMU prediction is not good … IMU better ratios rot=0.93,
  trans=0.08, vel=0.14" — the low-grade Ouster built-in IMU helps rotation but
  not translation, blunting GLIM's tight-fusion edge (same root cause as the
  known IMU-limited yaw drift). Not a config bug.
- **Label**: the pipeline's run-time summary prints "FastReg" as the backend
  name — a hardcoded default in `perf_timer`; the run used the GLIM CSV
  (`INCREMENTAL_NAV_CSV=incremental_navigation_results_glim.csv`). Metrics JSON
  was written as `timing_FastReg_pipeline_gnss0.json` (mislabeled, same reason).
- **glim_rosbag headless**: must pass `-p auto_quit:=true` (baked into
  `02_run_glim.sh`) or it blocks on a keypress after playback.

## 4. Verdict (corrected)

GLIM **matches Faster-LIO's pole-corrected accuracy** (§2) but, **on this laptop,
does NOT beat it on speed**: Faster-LIO run offline is 111–159 s vs GLIM's ~205 s
(+240 s one-time conversion). The 4 GB laptop GPU can't out-run the 20-thread CPU
for this workload, and GLIM does more work (factor graph + sub/global mapping).

Practical takeaways:
- **Fastest option on this hardware = Faster-LIO in OFFLINE mode**
  (`fasterlio_integration/scripts/run_offline_bench.sh`), 111–159 s. If the current
  pain is Faster-LIO taking ~551 s, that's the real-time *playback* path
  (`00_run_fasterlio.sh` / `rosbag play`) — switching to offline largely removes the
  speed problem with **no new dependency**. This is the highest-value quick win.
- **GLIM earns its place** only for its global optimization / loop-closure and
  factor-graph consistency (not exercised on this one-way drive), or on a stronger
  desktop GPU where GPU VGICP would pull ahead of the CPU.
- Gap-closing GLIM tuning to try: disable sub/global mapping (pure odometry),
  coarser preprocess downsampling — but unlikely to beat Faster-LIO ds's 79 FPS here.

Artifacts: `output/glim_traj_gpu.txt`, `../incremental_navigation_results_glim.csv`,
`../snowpole_results_glim.csv`, `output/live_map_glim.png`, `output/pipeline_glim.log`.

## 5. Follow-ups

- [x] GLIM GNSS-% sweep (0/10/25/50 %, `GNSS_SEED=0`) — done, table above
      (`scripts/40_gnss_sweep.sh`; per-level logs `output/sweep_glim_gnss*.log`,
      metrics `fasterlio_integration/output/timing_GLIM_gnss_percentage_gnss*.json`).
- [ ] Reclaim disk: delete `output/ros2_bag/` (~32 GB) — no longer needed.
- [ ] Optional: tune GLIM voxel/preprocess resolution for even higher FPS.
