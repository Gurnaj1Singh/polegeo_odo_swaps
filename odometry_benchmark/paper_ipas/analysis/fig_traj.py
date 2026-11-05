"""Trajectory figure: GNSS ground truth vs FastReg vs KISS-ICP (YOLOv5 signal detector, as the runtime study).

  ../venv/Scripts/python paper_ipas/analysis/fig_traj.py [--pct 10] [--osm]   (from odometry_benchmark/)
  --osm: OpenStreetMap background (osm_basemap.py), output name gets an _osm suffix
  --sat: satellite background (Esri World Imagery), output name gets a _sat suffix

(a) odometry alone over the whole drive (aligned on the first 20-420 m only, as in scoring);
(b) after snow-pole correction with 10 % GNSS (seed 0), a fixed 400 m stretch centred on the middle of the surveyed
    pole section (not chosen for its result).
The corrected positions come from track(), a copy of scripts/04_gnss_sweep.py run() that also returns the positions;
its median error is checked against run() and results/snowpole_all_methods.csv.
-> paper_ipas/figures/fig_traj.pdf|png (10 %), fig_traj_gnss<pct>.pdf|png otherwise
"""
import json
import pickle
import random
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
import phase2 as P  # noqa: E402

ROOT = P.ROOT
P.sweep.CACHE_DIR = ROOT / "cache"  # YOLOv5 signal detections
METHODS = [("FastReg", "fastreg_run", "FastReg-run", "#2a78d6", "-"),  # reference palette slots 1-2
           ("KISS-ICP", "kiss_icp", "KISS-ICP", "#eb6834", "--")]
PCT, SEED, ZOOM_M = 10, 0, 400.0


def track(csv, pct, det, ref_e, ref_n, poles, bounds, seed=0):
    """sweep.run() returning the corrected position of every frame and the scored errors."""
    s = P.sweep
    fe, fn, headings, translations = s.odometry_increments(csv)
    mn_n, mx_n, mn_e, mx_e = bounds
    random.seed(seed)
    pe = pn = None
    pred_e, pred_n, err = [], [], []
    out_e, out_n = np.full(len(det["frames"]), np.nan), np.full(len(det["frames"]), np.nan)
    for i, dets in enumerate(det["frames"]):
        if i == 0:
            pe, pn = ref_e[0], ref_n[0]
        elif random.uniform(0, 100) < pct:
            pe, pn = ref_e[i], ref_n[i]
        else:
            pe, pn = s.new_position(pe, pn, headings[i], translations[i])
        pred_e.append(pe); pred_n.append(pn)  # pre-correction position: run() takes headings from it
        out_e[i], out_n[i] = pe, pn
        if not (mn_n <= pn <= mx_n and mn_e <= pe <= mx_e):
            continue
        if i > 0:
            for _, dist, az in dets:
                if dist > 5:
                    continue
                with s.quiet():
                    hd = s.gu.calculate_vehicle_heading_from_two_utm(pred_e[i - 1], pred_n[i - 1], pe, pn)
                    la, lo = s.to_wgs84.transform(pe, pn)
                    te, tn, adj = s.gu.local_to_utm33_with_offset_proj(la, lo, hd, az, dist)
                ge, gn = poles[int(np.argmin(np.hypot(poles[:, 0] - te, poles[:, 1] - tn)))]
                glat, glon = s.to_wgs84.transform(ge, gn)
                vlon, vlat, _ = s.geod.fwd(glon, glat, (180 + adj) % 360, dist)
                pe, pn = s.to_utm33n.transform(vlat, vlon)
        out_e[i], out_n[i] = pe, pn  # position after this frame's corrections
        err.append(np.hypot(ref_e[i] - pe, ref_n[i] - pn))
    return out_e, out_n, np.array(err), fe, fn


def main():
    global PCT
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--pct", type=int, default=PCT)
    ap.add_argument("--osm", action="store_true")
    ap.add_argument("--sat", action="store_true")
    args = ap.parse_args()
    PCT = args.pct
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    det = pickle.load(open(P.sweep.CACHE_DIR / "detections.pkl", "rb"))
    with P.sweep.quiet():
        ref_e, ref_n = P.sweep.reference_track(det)
        gt_n, gt_e, mn_n, mx_n, mn_e, mx_e, *_ = P.gu.load_gnss_data_and_calculate_utm_range(
            str(ROOT / "polegeo" / "groundtruth_poles.csv"))
    poles, bounds = np.c_[gt_e, gt_n], (mn_n, mx_n, mn_e, mx_e)
    pub = pd.read_csv(ROOT / "results" / "snowpole_all_methods.csv").set_index("method")
    res = {}
    for label, key, sname, *_ in METHODS:
        csv = str(P.csv_of(key))
        pe, pn, err, fe, fn = track(csv, PCT, det, ref_e, ref_n, poles, bounds, SEED)
        med = round(float(np.median(err)), 2)
        assert med == P.sweep.run(csv, PCT, det, ref_e, ref_n, poles, bounds, SEED)["err_pred_median"]
        assert med == pub.loc[sname, f"gnss{PCT}_median_m"], (label, med)
        n = len(ref_e)  # the CSV has 2 extra GNSS rows; frames index its first len(ref_e) rows, as in run()
        aj = json.loads((ROOT / "outputs" / "csv" / f"incremental_navigation_results_{key}.align.json").read_text())
        res[label] = dict(pe=pe, pn=pn, med=med, fe=fe[:n], fn=fn[:n], drift=aj["raw_vs_gnss_median_m"])
        print(label, "corrected median", med, "m; raw drift median", round(res[label]["drift"], 1), "m")

    e0, n0 = ref_e[0], ref_n[0]
    import matplotlib.patheffects as pe_
    # ground truth: black on a plain/greyscale map, white with a dark outline on satellite imagery
    GT = (dict(color="white", path_effects=[pe_.Stroke(linewidth=3.4, foreground="#0b0b0b"), pe_.Normal()])
          if args.sat else dict(color="#0b0b0b"))
    POLE = "#ffffff" if args.sat else "#8a8984"
    km = lambda e, n: ((np.asarray(e) - e0) / 1000, (np.asarray(n) - n0) / 1000)
    plt.rcParams.update({"font.family": "serif", "font.size": 8, "axes.linewidth": 0.6, "axes.edgecolor": "#52514e",
                         "xtick.color": "#52514e", "ytick.color": "#52514e", "legend.frameon": False,
                         "pdf.fonttype": 42})
    fig, ax = plt.subplots(1, 2, figsize=(7.16, 3.3), gridspec_kw={"width_ratios": [1.25, 1]})

    # (a) odometry alone, whole drive
    a = ax[0]
    a.plot(*km(ref_e, ref_n), lw=2.2, label="GNSS ground truth", zorder=3, **GT)
    for label, _, _, c, ls in METHODS:
        r = res[label]
        a.plot(*km(r["fe"], r["fn"]), color=c, ls=ls, lw=1.3, zorder=4,
               label=f"{label} odometry (median drift {r['drift']:.0f} m)")
    a.scatter(*km(poles[:, 0], poles[:, 1]), s=3, color="#b0afa8", lw=0, zorder=3.5, label="surveyed snow poles")
    a.plot(*km(ref_e[0], ref_n[0]), "o", color="#0b0b0b", mec="white" if args.sat else "#0b0b0b", ms=4, zorder=5)
    a.annotate("start", km(ref_e[0], ref_n[0]), xytext=(-6, 6), textcoords="offset points", fontsize=7, ha="right",
               color="white" if args.sat else "#0b0b0b")
    a.set_title("(a) Odometry only, whole drive (10.3 km)", loc="left", fontsize=8)

    # (b) after pole correction + 10 % GNSS: 400 m stretch centred on the middle of the pole section
    inside = np.where((ref_n >= mn_n) & (ref_n <= mx_n) & (ref_e >= mn_e) & (ref_e <= mx_e))[0]
    trav = np.r_[0, np.cumsum(np.hypot(np.diff(ref_e), np.diff(ref_n)))]
    mid = (trav[inside[0]] + trav[inside[-1]]) / 2
    win = (trav >= mid - ZOOM_M / 2) & (trav <= mid + ZOOM_M / 2)
    b = ax[1]
    b.plot(*km(ref_e[win], ref_n[win]), lw=2.2, zorder=3, label="GNSS ground truth", **GT)
    for label, _, _, c, ls in METHODS:
        r = res[label]
        b.plot(*km(r["pe"][win], r["pn"][win]), color=c, ls=ls, lw=1.3, zorder=4,
               label=f"{label} (median error {r['med']:.2f} m)")
    ze, zn = ref_e[win], ref_n[win]
    near = (poles[:, 0] > ze.min() - 30) & (poles[:, 0] < ze.max() + 30) & \
           (poles[:, 1] > zn.min() - 30) & (poles[:, 1] < zn.max() + 30)
    b.scatter(*km(poles[near, 0], poles[near, 1]), s=10, marker="x", color=POLE, lw=0.8, zorder=2.5,
              label="surveyed snow poles")
    b.set_title(f"(b) + snow poles + {PCT} % GNSS, {ZOOM_M:.0f} m stretch", loc="left", fontsize=8)
    # mark the zoomed stretch in (a)
    (x0, y0), (x1, y1) = km(ze.min() - 60, zn.min() - 60), km(ze.max() + 60, zn.max() + 60)
    a.add_patch(plt.Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, ec="white" if args.sat else "#52514e", lw=0.9,
                              zorder=6))
    a.annotate("(b)", (x1, y0), xytext=(3, -2), textcoords="offset points", fontsize=7, va="top",
               color="white" if args.sat else "#0b0b0b")

    for x in ax:
        x.set_aspect("equal", adjustable="datalim")
        x.set_xlabel("East (km from start)")
        x.grid(color="#e4e3df", lw=0.5); x.set_axisbelow(True)
        for sp in ("top", "right"):
            x.spines[sp].set_visible(False)
    ax[0].set_ylabel("North (km from start)")
    for x in ax:  # on a map, the legend needs its own background
        x.legend(loc="upper left", fontsize=6.5, handlelength=2.2, frameon=args.osm or args.sat, framealpha=0.85,
                 facecolor="white", edgecolor="none")
    fig.tight_layout(pad=0.4, w_pad=1.0)
    name = "fig_traj" if PCT == 10 else f"fig_traj_gnss{PCT}"
    if args.osm or args.sat:
        import osm_basemap
        fig.canvas.draw()  # final equal-aspect limits
        for x, z in ((ax[0], 13), (ax[1], 17)):
            x.set_adjustable("box")
            x.grid(False)
            n = osm_basemap.add(x, z, origin=(e0, n0), scale=1000.0,
                                **(dict(provider="esri_imagery", grey=False, alpha=1.0, credit=False)
                                   if args.sat else {}))
            print("map tiles", n, "at zoom", z)
        if args.sat:  # Esri's required credit is too long for one panel: print it once under the figure
            fig.subplots_adjust(bottom=fig.subplotpars.bottom + 0.03)
            fig.text(0.995, 0.005, osm_basemap.PROVIDERS["esri_imagery"][1], ha="right", va="bottom", fontsize=5.5,
                     color="#3d3d3a")
        name += "_sat" if args.sat else "_osm"
    fig.savefig(P.FIGS / f"{name}.pdf"); fig.savefig(P.FIGS / f"{name}.png", dpi=250)


if __name__ == "__main__":
    main()
