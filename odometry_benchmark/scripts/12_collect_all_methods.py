"""Collect every SnowPole odometry method into one results file, grouped by the professor's four
categories (non-DL <=2024, DL <=2024, non-DL 2025-26, DL 2025-26).

  python scripts/12_collect_all_methods.py      # -> results/snowpole_all_methods.csv and .md
  python scripts/12_collect_all_methods.py --detector comb4
      # YOLO11n on Combination 4 (outputs/scores_comb4) -> results/snowpole_all_methods_comb4.* and
      # results/snowpole_signal_vs_comb4.md

Scores come from outputs/scores/<Name>_seed<s>.csv (04_gnss_sweep.py, same detector, bridge and
correction for every method). Methods without a run in this repository yet fall back to the
numbers published in the reference repo and are marked "reference repo (not re-run)".
"""
import argparse
import json
from pathlib import Path

import pandas as pd

ROOT = Path(__file__).resolve().parent.parent
CATS = {1: "1. Non-DL, up to 2024", 2: "2. DL/ML, up to 2024", 3: "3. Non-DL, 2025-2026", 4: "4. DL/ML, 2025-2026",
        5: "Extra: non-DL, 2025 (outside the 12)"}

# name, category, year, venue, sensors, output dir (for raw drift / timing), reference fallback
METHODS = [
    ("KISS-ICP",   1, 2023, "RA-L 2023",            "LiDAR (+IMU levelling)", "kiss_icp",     None),
    ("MAD-ICP",    1, 2024, "RA-L 2024",            "LiDAR (+IMU levelling)", "mad_icp",      None),
    ("FAST-LIO2",  1, 2022, "T-RO 2022",            "LiDAR + IMU",            "fastlio2",     None),
    ("Faster-LIO", 1, 2022, "RA-L 2022",            "LiDAR + IMU",            "fasterlio",    [8.70, 0.80, 0.46, 0.19]),
    ("GLIM",       1, 2024, "RAS 2024",             "LiDAR + IMU (GPU)",      "glim",         [10.18, 1.14, 0.63, 0.26]),
    ("FastReg",    2, 2021, "RA-L 2021",            "LiDAR",                  None,           None),
    # same method run on this machine (scripts/07_run_fastreg.py, KITTI checkpoint): own trajectory + runtime
    ("FastReg-run", 2, 2021, "RA-L 2021",           "LiDAR (+IMU levelling)", "fastreg",      None),
    ("DeepPointMap", 2, 2024, "AAAI 2024",          "LiDAR (+IMU levelling)", "deeppointmap", None),
    ("PIN-SLAM",   2, 2024, "T-RO 2024",            "LiDAR (+IMU levelling)", "pin_slam",     None),
    ("RKO-LIO",    3, 2025, "arXiv/RA-L 2025",      "LiDAR + IMU",            "rko_lio",      None),
    ("GenZ-ICP",   3, 2025, "RA-L 2025",            "LiDAR (+IMU levelling)", "genz_icp",     None),
    ("Super-LIO",  3, 2026, "RA-L 2026",            "LiDAR + IMU",            "superlio",     [9.65, 0.91, 0.53, 0.20]),
    ("BUFFER-X",   4, 2025, "ICCV 2025",            "LiDAR (+IMU levelling)", "bufferx",      None),
    ("UGP",        4, 2025, "CVPR 2025",            "LiDAR (+IMU levelling)", "ugp",          None),
    ("CLID-SLAM",  4, 2025, "RA-L 2025",            "LiDAR + IMU (+IMU levelling)", None,       None),
    ("Hi-LOAM",    4, 2026, "IEEE TMM 2026",        "LiDAR",                  None,           None),
    # Gaussian-splatting odometry: differentiable rendering optimised per run, no network or learned weights
    ("Splat-LOAM", 5, 2025, "ICCV 2025",            "LiDAR (+IMU levelling)", "splatloam",    None),
]
PCTS = [0, 10, 25, 50]
DETECTORS = {"signal": ("scores", "", "YOLOv5 `pole_best_signal.pt` (signal modality)"),
             "comb4": ("scores_comb4", "_comb4", "YOLO11n `models/yolo11n_comb4_final.pt` trained on "
                       "Combination 4 (near-IR, signal, reflectivity stacked; results/detector_yolo11n_comb4.md)")}
NOT_RUNNABLE = {"Hi-LOAM": "not runnable: public repo (Zhangjyhhh/Hi-LOAM @ a8add39) never contained its model/ "
                            "package (FeatureOctree, Decoder); replaced by CLID-SLAM",
                "CLID-SLAM": "FAILED - lost track on the highway: stopped itself at scan 911/5419 with the "
                             "repository's 1-60 m Ouster-128 config and at scan 831/5419 with the shared 2-150 m "
                             "range (it tracks RKO-LIO to within 2 m for the first 500 scans, then under-estimates "
                             "speed above ~15 m/s); outputs/clidslam_attempts/",
                "Splat-LOAM": "FAILED - lost track on the highway: from sensor t~150 s its estimate moved ~0.6 m/s "
                              "(true 15-20 m/s) and climbed to +34 m, inserting a keyframe every scan (~3 min/scan, "
                              "GPU memory full); stopped manually at t~278 s of ~542 s after 6 h; "
                              "outputs/splatloam_attempts/full_run_lost_track/"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--detector", choices=list(DETECTORS), default="signal")
    args = ap.parse_args()
    sdir, suffix, det_label = DETECTORS[args.detector]
    scores = ROOT / "outputs" / sdir
    rows = []
    for name, cat, year, venue, sensors, odir, ref in METHODS:
        r = {"category": CATS[cat], "method": name, "year": year, "venue": venue, "sensors": sensors}
        s0 = scores / f"{name}_seed0.csv"
        if s0.exists():
            d0 = pd.read_csv(s0).set_index("gnss_pct")
            r["source"] = ("author trajectory, re-scored here" if name == "FastReg" else "run in this repo")
            for p in PCTS:
                r[f"gnss{p}_median_m"] = round(float(d0.err_pred_median.get(p, float("nan"))), 2)
            rng = []
            for p in PCTS[1:]:
                vals = [pd.read_csv(f).set_index("gnss_pct").err_pred_median.get(p)
                        for f in sorted(scores.glob(f"{name}_seed*.csv"))]
                vals = [v for v in vals if v is not None]
                rng.append(f"{min(vals):.2f}-{max(vals):.2f}" if len(vals) > 1 else "")
            r["seed0-2_range_10/25/50"] = " / ".join(rng)
        elif ref is not None and args.detector == "signal":
            r["source"] = "reference repo (not re-run)"
            for p, v in zip(PCTS, ref):
                r[f"gnss{p}_median_m"] = v
        elif name in NOT_RUNNABLE:
            r["source"] = NOT_RUNNABLE[name]
        else:
            r["source"] = "pending"
        if odir:
            key = {"FastReg-run": "fastreg_run"}.get(name, odir)  # CSV key differs from the output dir
            a = ROOT / "outputs" / "csv" / f"incremental_navigation_results_{key}.align.json"
            if a.exists():
                aj = json.loads(a.read_text())
                r["raw_drift_median_m"], r["raw_drift_max_m"] = aj["raw_vs_gnss_median_m"], aj["raw_vs_gnss_max_m"]
            for t in [ROOT / "outputs" / odir / "timing.json", ROOT / "outputs" / odir / "timing_wall.json"]:
                if t.exists():
                    tj = json.loads(t.read_text())
                    ms = tj.get("ms_per_scan_mean") or (1000 * tj["wall_s"] / tj["n_scans"] if "wall_s" in tj else None)
                    r["ms_per_scan"] = round(float(ms), 1) if ms else None
                    break
        if name == "RKO-LIO" and "ms_per_scan" not in r:
            r["ms_per_scan"] = 20.6  # rko_lio CLI profiler, "Pipeline - Register Scan" (outputs/rko_lio/run.log)
        if name == "Faster-LIO" and "ms_per_scan" not in r:
            r["ms_per_scan"] = 13.1  # "Laser Mapping Single Run" timer, excl. 6.3 ms preprocessing (outputs/fasterlio/run.log)
        if name == "Super-LIO" and "ms_per_scan" not in r:
            # sum of its stateProcess() timers Undistort 0.35 + DownSample 0.51 + Observe 3.84 + UpdateMap 0.98
            # (super_lio.cpp:194-206; excl. ROS message conversion and publishing; outputs/superlio/superlio_run.log)
            r["ms_per_scan"] = 5.7
        rows.append(r)
    df = pd.DataFrame(rows)
    out = ROOT / "results" / f"snowpole_all_methods{suffix}.csv"
    df.to_csv(out, index=False)

    md = ["# SnowPole: all odometry methods",
          "",
          "Median vehicle-position error after snow-pole correction (m, lower is better), random seed 0, "
          f"{det_label} for every method. Same bridge (start-anchored, "
          "GNSS travel 20-420 m), same correction and scoring (`scripts/04_gnss_sweep.py`).",
          "",
          "| Category | Method | Venue | 0 % | 10 % | 25 % | 50 % | Raw drift median | ms/scan | Source |",
          "|---|---|---|---|---|---|---|---|---|---|"]
    fmt = lambda v: "" if pd.isna(v) else (f"{v:.2f}" if isinstance(v, float) else str(v))
    for _, r in df.iterrows():
        md.append(f"| {r.category} | {r.method} | {r.venue} | " + " | ".join(fmt(r.get(f'gnss{p}_median_m')) for p in PCTS)
                  + f" | {fmt(r.get('raw_drift_median_m'))} | {fmt(r.get('ms_per_scan'))} | {r.source} |")
    if args.detector != "signal":
        md += ["", "Detector training images are frames 844-2999 of this drive (5420 frames; "
               "results/detector_dataset_frame_map.csv), so its detections there are optimistic."]
    (ROOT / "results" / f"snowpole_all_methods{suffix}.md").write_text("\n".join(md) + "\n")
    print("\n".join(md))
    if args.detector != "signal" and (ROOT / "results" / "snowpole_all_methods.csv").exists():
        compare(df, pd.read_csv(ROOT / "results" / "snowpole_all_methods.csv"), args.detector, suffix)


def compare(new, old, det, suffix):
    """Side-by-side median error, signal YOLOv5 vs the new detector, for every method scored with both."""
    md = [f"# SnowPole: signal YOLOv5 vs {det} detector", "",
          "Median position error (m), seed 0, same odometry trajectories, bridge and scoring; only the pole "
          "detector differs. Delta = new - signal (negative = new detector better).", "",
          "| Method | " + " | ".join(f"{p} % signal | {p} % {det} | delta" for p in PCTS) + " |",
          "|---|" + "---|---|---|" * len(PCTS)]
    o = old.set_index("method")
    for _, r in new.iterrows():
        if pd.isna(r.get("gnss10_median_m")) or r.method not in o.index or pd.isna(o.loc[r.method].get("gnss10_median_m")):
            continue
        cells = []
        for p in PCTS:
            a, b = o.loc[r.method][f"gnss{p}_median_m"], r[f"gnss{p}_median_m"]
            cells.append(f"{a:.2f} | {b:.2f} | {b - a:+.2f}")
        md.append(f"| {r.method} | " + " | ".join(cells) + " |")
    (ROOT / "results" / f"snowpole_signal_vs{suffix}.md").write_text("\n".join(md) + "\n")
    print("\n" + "\n".join(md))


if __name__ == "__main__":
    main()
