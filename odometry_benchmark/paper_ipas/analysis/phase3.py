"""Phase 3 tables and figure for the IPAS paper: how the built-in IMU should be used.

  ../venv/Scripts/python paper_ipas/analysis/phase3.py            (from odometry_benchmark/)

Inputs: trajectories bridged by run_imu_degradation.sh, run_imu_tuning.sh and phase3_imu_only.py
(outputs/csv/incremental_navigation_results_<key>.csv). Every variant is re-scored here with the paper's localization
(phase2 / scripts/04_gnss_sweep.py, YOLO11n Combination 4), and checked against the score files the runs wrote.
Writes paper_ipas/data/imu_use.csv, imu_degradation.csv and figures/fig_imu.pdf|png.
"""
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
import phase2 as P  # noqa: E402

PCTS = [0, 2, 5, 10, 25, 50]
# label, csv key, how the IMU is used, name in outputs/scores_imu (None = paper baseline in outputs/scores_comb4)
USE = [("IMU only", "imu_only", "strapdown + vehicle constraint", "IMU-only"),
       ("FastReg, no levelling", "fastreg_nolevel", "none", "FastReg-nolevel"),
       ("FastReg", "fastreg_run", "gravity levelling once", None),
       ("FAST-LIO2", "fastlio2", "tightly coupled, default noise", None),
       ("Faster-LIO", "fasterlio", "tightly coupled, default noise", None),
       ("FAST-LIO2, measured gyro noise", "imudeg_fastlio2_gyrcov", "tightly coupled, measured gyro noise", "FAST-LIO2_gyrcov"),
       ("Faster-LIO, measured gyro noise", "imudeg_fasterlio_gyrcov", "tightly coupled, measured gyro noise", "Faster-LIO_gyrcov"),
       ("FAST-LIO2, measured IMU noise", "imudeg_fastlio2_imucov", "tightly coupled, measured IMU noise", "FAST-LIO2_imucov"),
       ("Faster-LIO, measured IMU noise", "imudeg_fasterlio_imucov", "tightly coupled, measured IMU noise", "Faster-LIO_imucov"),
       ("FAST-LIO2, 6 s IMU init", "imudeg_fastlio2_init6s", "tightly coupled, 6 s initialisation", "FAST-LIO2_init6s"),
       ("Faster-LIO, 6 s IMU init", "imudeg_fasterlio_init6s", "tightly coupled, 6 s initialisation", "Faster-LIO_init6s"),
       ("FAST-LIO2, 6 s init + fixed gyro bias", "imudeg_fastlio2_biasfix", "tightly coupled, gyro bias held",
        "FAST-LIO2_biasfix"),
       ("Faster-LIO, 6 s init + fixed gyro bias", "imudeg_fasterlio_biasfix", "tightly coupled, gyro bias held",
        "Faster-LIO_biasfix"),
       ("Gyro + FastReg distance", "gyro_fastreg", "gyro heading, LiDAR distance", "Gyro+FastReg"),
       ("Gyro + FAST-LIO2 distance", "gyro_fastlio2", "gyro heading, LIO distance", "Gyro+FAST-LIO2"),
       ("Gyro + Faster-LIO distance", "gyro_fasterlio", "gyro heading, LIO distance", "Gyro+Faster-LIO")]
VARIANTS = [("rate100", "100 Hz (control)"), ("rate50", "50 Hz"), ("rate25", "25 Hz"), ("rate10", "10 Hz"),
            ("noise3", "3x noise"), ("noise10", "10x noise")]


def score_rows(rows, det, ref_e, ref_n, poles, bounds):
    out = []
    for label, key, use, sname in rows:
        csv = str(P.csv_of(key))
        r = {"label": label, "key": key, "imu_use": use}
        for p in PCTS:
            r[f"gnss{p}"] = P.sweep.run(csv, p, det, ref_e, ref_n, poles, bounds, 0)["err_pred_median"]
        r10 = [r["gnss10"]] + [P.sweep.run(csv, 10, det, ref_e, ref_n, poles, bounds, s)["err_pred_median"] for s in (1, 2)]
        r["gnss10_seed_range"] = f"{min(r10):.2f}-{max(r10):.2f}"
        if sname:  # agreement with the score file written by the run
            f = P.ROOT / "outputs" / "scores_imu" / f"{sname}_seed0.csv"
            pub = pd.read_csv(f).set_index("gnss_pct").err_pred_median
            assert all(abs(pub[p] - r[f"gnss{p}"]) < 1e-9 for p in PCTS), (label, pub.to_dict())
        out.append(r)
        print(label, [r[f"gnss{p}"] for p in PCTS], flush=True)
    return pd.DataFrame(out)


def drift_rows(rows):
    P.METHODS = [(label, key, "", use) for label, key, use, _ in rows]
    df, _ = P.drift_analysis()
    return df.rename(columns={"method": "label"}).drop(columns=["modality"])


def assoc_rows(rows, det, ref_e, ref_n, poles, bounds):
    out = []
    for label, key, *_ in rows:
        r = {"label": label}
        for p in (0, 10):
            _, _, sn = P.run_diag(str(P.csv_of(key)), p, det, ref_e, ref_n, poles, bounds, 0)
            r[f"wrong_assoc_pct_gnss{p}"] = round(100 * float((~sn.correct).mean()), 1)
        out.append(r)
    return pd.DataFrame(out)


def figure(df):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.family": "serif", "font.size": 8, "axes.linewidth": 0.6, "axes.edgecolor": "#52514e",
                         "xtick.color": "#52514e", "ytick.color": "#52514e", "legend.frameon": False,
                         "axes.spines.top": False, "axes.spines.right": False, "pdf.fonttype": 42})
    # reference palette slots 1-5, fixed order; marker + dash as secondary encoding
    show = [("FastReg", "LiDAR only (FastReg)", "#2a78d6", "o", "-"),
            ("FAST-LIO2", "LiDAR+IMU, tight (FAST-LIO2)", "#eb6834", "s", "--"),
            ("Faster-LIO", "LiDAR+IMU, tight (Faster-LIO)", "#1baf7a", "D", ":"),
            ("Gyro + FastReg distance", "gyro heading + FastReg distance", "#eda100", "^", "-."),
            ("Gyro + Faster-LIO distance", "gyro heading + Faster-LIO distance", "#e87ba4", "v", (0, (5, 1)))]
    fig, ax = plt.subplots(figsize=(3.45, 2.5))
    d = df.set_index("label")
    for label, name, c, mk, ls in show:
        if label not in d.index:
            continue
        ax.plot(PCTS, [d.loc[label, f"gnss{p}"] for p in PCTS], color=c, marker=mk, ls=ls, lw=1.2, ms=4,
                mec="white", mew=0.6, label=name)
    ax.set_yscale("log")
    ax.set_xlabel("GNSS availability (% of frames)")
    ax.set_ylabel("Median position error (m)")
    ax.grid(color="#e4e3df", lw=0.5, which="both"); ax.set_axisbelow(True)
    ax.legend(loc="upper right", fontsize=6.5)
    fig.tight_layout(pad=0.3)
    fig.savefig(P.FIGS / "fig_imu.pdf"); fig.savefig(P.FIGS / "fig_imu.png", dpi=200)


def main():
    det, ref_e, ref_n, poles, bounds = P.setup()
    use = score_rows(USE, det, ref_e, ref_n, poles, bounds)
    use = use.merge(drift_rows(USE), on="label").merge(
        assoc_rows([u for u in USE if u[0] not in ("IMU only", "FastReg, no levelling")], det, ref_e, ref_n, poles,
                   bounds), on="label", how="left")
    use.to_csv(P.DATA / "imu_use.csv", index=False)
    deg = []
    for m, mlabel in (("fastlio2", "FAST-LIO2"), ("fasterlio", "Faster-LIO")):
        rows = [(f"{mlabel}, {vl}", f"imudeg_{m}_{v}", vl, f"{mlabel}_{v}") for v, vl in VARIANTS]
        d = score_rows(rows, det, ref_e, ref_n, poles, bounds).merge(drift_rows(rows), on="label")
        d.insert(0, "method", mlabel)
        deg.append(d)
    pd.concat(deg).to_csv(P.DATA / "imu_degradation.csv", index=False)
    figure(use)
    cols = ["label", "drift_at_2km_m", "sec_heading_err_median_deg", "sec_step_len_bias_pct"] + [f"gnss{p}" for p in PCTS] \
        + ["wrong_assoc_pct_gnss0", "wrong_assoc_pct_gnss10"]
    print(use[cols].round(2).to_string())
    print(pd.concat(deg)[["label", "drift_median_m", "sec_heading_err_median_deg", "gnss0", "gnss10", "gnss25"]].round(2).to_string())


if __name__ == "__main__":
    main()
