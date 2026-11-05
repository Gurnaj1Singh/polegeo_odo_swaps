"""Phase 2 analyses and figures for the IPAS paper (LiDAR-only vs LiDAR+IMU odometry, snow-pole correction).

  ../venv/Scripts/python paper_ipas/analysis/phase2.py            (from odometry_benchmark/)

Uses the same trajectories, detections (YOLO11n Combination 4, cache_comb4/) and localization loop as
results/snowpole_all_methods_comb4.* - scripts/04_gnss_sweep.py is imported, not re-implemented, except for
run_diag(), a copy of its run() with bookkeeping that is checked against run() before use.
Writes paper_ipas/data/*.csv|json and paper_ipas/figures/fig_{drift,gnss}.pdf|png.
"""
import importlib.util
import json
import pickle
import random
import sys
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[2]
PAPER = ROOT / "paper_ipas"
DATA, FIGS = PAPER / "data", PAPER / "figures"
spec = importlib.util.spec_from_file_location("sweep", ROOT / "scripts" / "04_gnss_sweep.py")
sweep = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sweep)
sweep.CACHE_DIR = ROOT / "cache_comb4"
gu = sweep.gu

# label, trajectory CSV key, name in outputs/scores_comb4, modality
METHODS = [("FastReg", "fastreg_run", "FastReg-run", "LiDAR only"),
           ("FAST-LIO2", "fastlio2", "FAST-LIO2", "LiDAR + IMU"),
           ("Faster-LIO", "fasterlio", "Faster-LIO", "LiDAR + IMU")]
STYLE = {"FastReg": dict(color="#2a78d6", marker="o", ls="-"),     # reference palette slots 1-3
         "FAST-LIO2": dict(color="#eb6834", marker="s", ls="--"),
         "Faster-LIO": dict(color="#1baf7a", marker="^", ls="-.")}
PCTS = [0, 2, 5, 10, 15, 20, 25, 30, 40, 50]
SEEDS = [0, 1, 2]
SECTION_KM = (0.727, 5.187)  # surveyed snow-pole section (frames 831-2983), km from the start
csv_of = lambda key: ROOT / "outputs" / "csv" / f"incremental_navigation_results_{key}.csv"


def setup():
    det = pickle.load(open(sweep.CACHE_DIR / "detections.pkl", "rb"))
    with sweep.quiet():
        ref_e, ref_n = sweep.reference_track(det)
        gt_n, gt_e, mn_n, mx_n, mn_e, mx_e, *_ = gu.load_gnss_data_and_calculate_utm_range(
            str(ROOT / "polegeo" / "groundtruth_poles.csv"))
    return det, ref_e, ref_n, np.c_[gt_e, gt_n], (mn_n, mx_n, mn_e, mx_e)


def run_diag(csv, pct, det, ref_e, ref_n, poles, bounds, seed=0):
    """sweep.run() plus: per-frame error, and for every pole snap whether the surveyed pole it chose is the one
    the same detection selects when projected from the GNSS reference pose (= correct association)."""
    fe, fn, headings, translations = sweep.odometry_increments(csv)
    mn_n, mx_n, mn_e, mx_e = bounds
    random.seed(seed)
    pe = pn = None
    pred_e, pred_n, err, frame = [], [], [], []
    snaps = []
    for i, dets in enumerate(det["frames"]):
        if i == 0:
            pe, pn = ref_e[0], ref_n[0]
        elif random.uniform(0, 100) < pct:
            pe, pn = ref_e[i], ref_n[i]
        else:
            pe, pn = sweep.new_position(pe, pn, headings[i], translations[i])
        pred_e.append(pe); pred_n.append(pn)
        if not (mn_n <= pn <= mx_n and mn_e <= pe <= mx_e):
            continue
        if i > 0:
            for _, dist, az in dets:
                if dist > 5:
                    continue
                with sweep.quiet():
                    hd = gu.calculate_vehicle_heading_from_two_utm(pred_e[i - 1], pred_n[i - 1], pe, pn)
                    la, lo = sweep.to_wgs84.transform(pe, pn)
                    te, tn, adj = gu.local_to_utm33_with_offset_proj(la, lo, hd, az, dist)
                    hr = gu.calculate_vehicle_heading_from_two_utm(ref_e[i - 1], ref_n[i - 1], ref_e[i], ref_n[i])
                    lar, lor = sweep.to_wgs84.transform(ref_e[i], ref_n[i])
                    tre, trn, _ = gu.local_to_utm33_with_offset_proj(lar, lor, hr, az, dist)
                k = int(np.argmin(np.hypot(poles[:, 0] - te, poles[:, 1] - tn)))
                k_true = int(np.argmin(np.hypot(poles[:, 0] - tre, poles[:, 1] - trn)))
                before = float(np.hypot(ref_e[i] - pe, ref_n[i] - pn))
                ge, gn = poles[k]
                glat, glon = sweep.to_wgs84.transform(ge, gn)
                vlon, vlat, _ = sweep.geod.fwd(glon, glat, (180 + adj) % 360, dist)
                pe, pn = sweep.to_utm33n.transform(vlat, vlon)
                snaps.append({"frame": i, "correct": k == k_true, "err_before": before,
                              "err_after": float(np.hypot(ref_e[i] - pe, ref_n[i] - pn))})
        err.append(float(np.hypot(ref_e[i] - pe, ref_n[i] - pn))); frame.append(i)
    return np.array(frame), np.array(err), pd.DataFrame(snaps)


def drift_analysis():
    """Uncorrected drift vs distance, plus the two quantities dead reckoning consumes: heading and step length."""
    rows, curves = [], {}
    for label, key, _, mod in METHODS:
        d = pd.read_csv(csv_of(key))
        ge, gn, oe, on = (d[c].to_numpy() for c in ("gnss_easting", "gnss_northing", "easting", "northing"))
        ok = ~np.isnan(oe)
        ge, gn, oe, on, e = ge[ok], gn[ok], oe[ok], on[ok], d.gnss_error.to_numpy()[ok]
        trav = np.r_[0, np.cumsum(np.hypot(np.diff(ge), np.diff(gn)))] / 1000
        k = 10  # 1 s windows (10 Hz)
        dge, dgn, doe, don = (x[k:] - x[:-k] for x in (ge, gn, oe, on))
        dg, do = np.hypot(dge, dgn), np.hypot(doe, don)
        mv = dg > 5  # moving (> 5 m/s)
        dh = np.degrees(np.arctan2(doe, don) - np.arctan2(dge, dgn))
        dh = (dh + 180) % 360 - 180
        r = {"method": label, "modality": mod, "drift_median_m": np.median(e), "drift_max_m": e.max(),
             "heading_err_median_deg": np.median(np.abs(dh[mv])), "heading_err_p95_deg": np.percentile(np.abs(dh[mv]), 95),
             "step_len_err_median_pct": 100 * np.median(np.abs(do[mv] / dg[mv] - 1)),
             "step_len_bias_pct": 100 * (do[mv].sum() / dg[mv].sum() - 1)}
        for km in (1, 2, 5):
            r[f"drift_at_{km}km_m"] = float(np.interp(km, trav, e))
        # inside the surveyed snow-pole section (where localization is scored): accumulated heading error,
        # local turn-rate error (heading change over 1 s, odometry vs GNSS) and travelled-distance bias
        sec = mv & (trav[k:] >= SECTION_KM[0]) & (trav[k:] <= SECTION_KM[1])
        hg, ho = np.unwrap(np.arctan2(dge, dgn)), np.unwrap(np.arctan2(doe, don))
        rate = np.degrees(np.abs((ho[k:] - ho[:-k]) - (hg[k:] - hg[:-k])))
        r.update({"sec_heading_err_median_deg": np.median(np.abs(dh[sec])),
                  "sec_turn_rate_err_median_deg_s": np.median(rate[sec[k:]]),
                  "sec_step_len_bias_pct": 100 * (do[sec].sum() / dg[sec].sum() - 1)})
        rows.append(r)
        curves[label] = (trav, e, trav[k:][mv], np.abs(dh[mv]))
    df = pd.DataFrame(rows).round(3)
    df.to_csv(DATA / "drift.csv", index=False)
    return df, curves


def dense_sweep(det, ref_e, ref_n, poles, bounds):
    out = DATA / "gnss_dense.csv"
    if out.exists():
        return pd.read_csv(out)
    rows = []
    for label, key, _, _ in METHODS:
        for s in SEEDS:
            for p in PCTS:
                if p == 0 and s > 0:
                    continue  # no random draw takes effect at 0 %
                r = sweep.run(str(csv_of(key)), p, det, ref_e, ref_n, poles, bounds, s)
                rows.append({"method": label, "seed": s, **r})
                print(label, s, p, r["err_pred_median"], flush=True)
    df = pd.DataFrame(rows)
    df.to_csv(out, index=False)
    return df


def modality_ablation(det, ref_e, ref_n, poles, bounds):
    """Each source of position information switched on in turn, per odometry (seed 0):
    odometry alone / + sparse GNSS / + snow poles / + both."""
    no_poles = dict(det, frames=[[] for _ in det["frames"]])
    rows = []
    for label, key, _, mod in METHODS:
        for p in (0, 10, 25, 50):
            a = sweep.run(str(csv_of(key)), p, no_poles, ref_e, ref_n, poles, bounds, 0)
            b = sweep.run(str(csv_of(key)), p, det, ref_e, ref_n, poles, bounds, 0)
            rows.append({"method": label, "modality": mod, "gnss_pct": p, "no_poles_median_m": a["err_pred_median"],
                         "with_poles_median_m": b["err_pred_median"], "n_scored_no_poles": a["n_scored_frames"],
                         "n_scored_with_poles": b["n_scored_frames"]})
    df = pd.DataFrame(rows)
    df.to_csv(DATA / "modality_ablation.csv", index=False)
    return df


def association(det, ref_e, ref_n, poles, bounds):
    """Pole-association correctness at 0 / 10 % GNSS; first checks run_diag() reproduces sweep.run()."""
    rows, traces = [], {}
    for label, key, _, _ in METHODS:
        for p in (0, 10):
            f, e, sn = run_diag(str(csv_of(key)), p, det, ref_e, ref_n, poles, bounds, 0)
            ref = sweep.run(str(csv_of(key)), p, det, ref_e, ref_n, poles, bounds, 0)["err_pred_median"]
            assert round(float(np.median(e)), 2) == ref, (label, p, np.median(e), ref)
            wrong = ~sn.correct
            rows.append({"method": label, "gnss_pct": p, "median_err_m": ref, "n_snaps": len(sn),
                         "wrong_assoc_pct": round(100 * wrong.mean(), 1),
                         "first_wrong_frame": int(sn.frame[wrong].iloc[0]) if wrong.any() else None,
                         "err_before_snap_median_correct_m": round(float(sn.err_before[~wrong].median()), 2),
                         "err_before_snap_median_wrong_m": round(float(sn.err_before[wrong].median()), 2) if wrong.any() else None,
                         "err_after_snap_median_correct_m": round(float(sn.err_after[~wrong].median()), 2)})
            if p == 0:
                traces[label] = (f, e)
    nn = np.sort(np.hypot(*(poles[:, None, :] - poles[None, :, :]).transpose(2, 0, 1)), axis=1)[:, 1]
    df = pd.DataFrame(rows)
    df.to_csv(DATA / "association.csv", index=False)
    return df, traces, {"pole_spacing_median_m": round(float(np.median(nn)), 1),
                        "pole_spacing_p10_m": round(float(np.percentile(nn, 10)), 1)}


def figures(curves, dense):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.family": "serif", "font.size": 8, "axes.linewidth": 0.6, "axes.edgecolor": "#52514e",
                         "xtick.color": "#52514e", "ytick.color": "#52514e", "axes.labelcolor": "#0b0b0b",
                         "legend.frameon": False, "axes.spines.top": False, "axes.spines.right": False,
                         "pdf.fonttype": 42})
    grid = dict(color="#e4e3df", lw=0.5)

    # Fig. 2: (a) position drift vs distance, (b) heading error vs distance (100 m median)
    fig, ax = plt.subplots(2, 1, figsize=(3.45, 3.3), sharex=True)
    for label, (trav, e, tv, dh) in curves.items():
        st = STYLE[label]
        ax[0].plot(trav, e, color=st["color"], ls=st["ls"], lw=1.4, label=f"{label}")
        bins = np.arange(0, trav.max() + 0.1, 0.1)
        idx = np.digitize(tv, bins)
        med = [np.median(dh[idx == b]) if (idx == b).sum() > 5 else np.nan for b in range(1, len(bins))]
        ax[1].plot(bins[:-1] + 0.05, med, color=st["color"], ls=st["ls"], lw=1.0)
    ax[0].set_ylabel("Position drift (m)")
    ax[1].set_ylabel("|Heading error| (deg)")
    ax[1].set_xlabel("Distance travelled (km)")
    ax[1].set_yscale("log")
    for a in ax:
        a.grid(**grid); a.set_axisbelow(True)
    ax[0].legend(loc="upper left", fontsize=7, bbox_to_anchor=(0, 0.92))
    ax[0].set_title("(a) Position drift, no corrections", loc="left", fontsize=8)
    ax[1].set_title("(b) Heading error (median per 100 m)", loc="left", fontsize=8)
    for a in ax:
        a.axvspan(*SECTION_KM, color="#e4e3df", alpha=0.5, lw=0)  # surveyed snow-pole section
    fig.tight_layout(pad=0.3)
    fig.savefig(FIGS / "fig_drift.pdf"); fig.savefig(FIGS / "fig_drift.png", dpi=200)

    # Fig. 3: median localization error vs GNSS availability, band = min-max over seeds
    fig, ax = plt.subplots(figsize=(3.45, 2.3))
    for label, *_ in METHODS:
        st = STYLE[label]
        g = dense[dense.method == label].groupby("gnss_pct").err_pred_median
        s0 = dense[(dense.method == label) & (dense.seed == 0)].set_index("gnss_pct").err_pred_median
        x = s0.index.to_numpy()
        ax.fill_between(x, g.min().reindex(x), g.max().reindex(x), color=st["color"], alpha=0.18, lw=0)
        ax.plot(x, s0.to_numpy(), color=st["color"], ls=st["ls"], marker=st["marker"], ms=4, lw=1.2,
                mec="white", mew=0.6, label=label)
    ax.set_yscale("log")
    ax.set_xlabel("GNSS availability (% of frames)")
    ax.set_ylabel("Median position error (m)")
    ax.grid(**grid, which="both"); ax.set_axisbelow(True)
    ax.legend(loc="upper right", fontsize=7)
    fig.tight_layout(pad=0.3)
    fig.savefig(FIGS / "fig_gnss.pdf"); fig.savefig(FIGS / "fig_gnss.png", dpi=200)


def gnss_equivalence(dense):
    """GNSS share FastReg needs to match each LIO method at 10 / 25 % (seed 0, log-linear interpolation)."""
    base = dense[(dense.method == "FastReg") & (dense.seed == 0)].sort_values("gnss_pct")
    x, y = base.gnss_pct.to_numpy(float), np.log(base.err_pred_median.to_numpy())
    out = {}
    for label in ("FAST-LIO2", "Faster-LIO"):
        s0 = dense[(dense.method == label) & (dense.seed == 0)].set_index("gnss_pct").err_pred_median
        for p in (10, 25):
            target = np.log(s0[p])
            out[f"{label}@{p}%"] = {"error_m": float(s0[p]),
                                    "fastreg_gnss_pct_needed": round(float(np.interp(-target, -y, x)), 1)}
    return out


def main():
    DATA.mkdir(exist_ok=True); FIGS.mkdir(exist_ok=True)
    det, ref_e, ref_n, poles, bounds = setup()
    drift, curves = drift_analysis()
    print(drift.to_string())
    dense = dense_sweep(det, ref_e, ref_n, poles, bounds)
    # consistency with the published sweep (outputs/scores_comb4)
    for label, _, sname, _ in METHODS:
        pub = pd.read_csv(ROOT / "outputs" / "scores_comb4" / f"{sname}_seed0.csv").set_index("gnss_pct").err_pred_median
        mine = dense[(dense.method == label) & (dense.seed == 0)].set_index("gnss_pct").err_pred_median
        for p in (0, 10, 25, 50):
            assert abs(pub[p] - mine[p]) < 1e-9, (label, p, pub[p], mine[p])
    eq = gnss_equivalence(dense)
    abl = modality_ablation(det, ref_e, ref_n, poles, bounds)
    print(abl.to_string())
    assoc, traces, spacing = association(det, ref_e, ref_n, poles, bounds)
    print(assoc.to_string())
    figures(curves, dense)
    summary = {"gnss_equivalence": eq, "pole_spacing": spacing,
               "dense_seed_range": {f"{m}@{p}": [float(g.min()), float(g.max())] for (m, p), g in
                                    dense.groupby(["method", "gnss_pct"]).err_pred_median}}
    (DATA / "summary.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary["gnss_equivalence"], indent=2), summary["pole_spacing"])


if __name__ == "__main__":
    main()
