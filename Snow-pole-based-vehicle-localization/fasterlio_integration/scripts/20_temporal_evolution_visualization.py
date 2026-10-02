#!/usr/bin/env python3
"""
Stage 2 (final results): temporal evolution of vehicle motion together with the
localized snow poles.

Renders an animation where, as time advances:
  * the georeferenced snow poles are shown as fixed landmarks (green x),
  * the vehicle tracks grow frame-by-frame:
        - GNSS reference           (blue)
        - Faster-LIO odometry      (orange)   <- replaces FastReg
        - proposed pole-corrected  (black)    [if --results-csv given]
  * snow poles light up red as they are geo-localized                [if results],
  * a side panel shows localization error vs elapsed time with a moving cursor.

Also writes paper-style static summaries: trajectory overlay, error histogram,
error CDF, and error-vs-distance-travelled.

Inputs
------
--fasterlio-csv : output of 10_fasterlio_traj_to_csv.py
                  (Time, easting, northing, gnss_easting, gnss_northing, gnss_error)
--poles         : Groundtruth_pole_location_at_test_site_E39_Hemnekjolen.csv
--results-csv   : (optional) main-pipeline output with the proposed method columns
                  Predicted Vehicle Easting/Northing, Target Easting/Northing,
                  vehicle_easting_original/northing_original

Deps: numpy pandas matplotlib   (all UTM33N already; no pyproj needed)
"""
import argparse
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")                       # headless: write files
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


def load_poles(path):
    d = pd.read_csv(path)
    return d["UTM33-Øst"].values, d["UTM33-Nord"].values      # easting, northing


def try_col(df, *names):
    for n in names:
        if n in df.columns:
            return df[n].values
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--fasterlio-csv", required=True)
    ap.add_argument("--poles", required=True)
    ap.add_argument("--results-csv", default=None)
    ap.add_argument("--outdir", default="fasterlio_integration/output")
    ap.add_argument("--frames", type=int, default=600, help="animation frames (decimated)")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--odom-label", default="Faster-LIO",
                    help="odometry backend name for plot labels (e.g. Faster-LIO, GLIM, Super-LIO)")
    args = ap.parse_args()

    out = Path(args.outdir); out.mkdir(parents=True, exist_ok=True)

    fl = pd.read_csv(args.fasterlio_csv)
    t_abs0 = fl["Time"].values[0]                            # absolute start (epoch s)
    t = (fl["Time"].values - t_abs0) / 60.0                  # minutes elapsed
    fe, fn = fl["easting"].values, fl["northing"].values     # Faster-LIO
    ge, gn = fl["gnss_easting"].values, fl["gnss_northing"].values
    fl_err = fl["gnss_error"].values if "gnss_error" in fl else np.hypot(fe - ge, fn - gn)
    pe, pn = load_poles(args.poles)

    # optional proposed-method results. NOTE: the results CSV is SPARSE (one row per
    # pole detection), so it carries its own timestamps — not frame-aligned with the
    # 5422-row odometry CSV. We key it on its own elapsed time `pred_t` (minutes).
    pred_e = pred_n = ref_e = ref_n = loc_e = loc_n = pred_err = pred_t = None
    if args.results_csv and Path(args.results_csv).exists():
        rr = pd.read_csv(args.results_csv)
        pred_e = try_col(rr, "Predicted Vehicle Easting", "predicted_vehicle_easting")
        pred_n = try_col(rr, "Predicted Vehicle Northing", "predicted_vehicle_northing")
        ref_e = try_col(rr, "vehicle_easting_original", "Vehicle Easting")
        ref_n = try_col(rr, "vehicle_northing_original", "Vehicle Northing")
        loc_e = try_col(rr, "Target Easting", "target_easting")
        loc_n = try_col(rr, "Target Northing", "target_northing")
        rt = try_col(rr, "Timestamp", "Left GNSS Timestamp")
        pred_t = (rt - t_abs0) / 60.0 if rt is not None else None
        if pred_e is not None and ref_e is not None:
            pred_err = np.hypot(pred_e - ref_e, pred_n - ref_n)

    # ---------------- static summary figures (paper-style) ---------------- #
    _static_summaries(out, fe, fn, ge, gn, pe, pn, fl_err, t,
                      pred_e, pred_n, pred_err, pred_t, odom=args.odom_label)

    # ------------------------- animation --------------------------------- #
    idx = np.linspace(0, len(t) - 1, min(args.frames, len(t))).astype(int)

    fig = plt.figure(figsize=(15, 7))
    axm = fig.add_subplot(1, 2, 1)
    axe = fig.add_subplot(1, 2, 2)

    axm.set_title("Temporal evolution of vehicle motion + localized snow poles")
    axm.set_xlabel("Easting (m, UTM33N)"); axm.set_ylabel("Northing (m)")
    axm.scatter(pe, pn, c="green", marker="x", s=25, label="Snow poles (ground truth)", zorder=1)
    (gnss_ln,) = axm.plot([], [], "-", c="royalblue", lw=1.6, label="GNSS reference")
    (fl_ln,) = axm.plot([], [], "-", c="darkorange", lw=1.6, label=f"{args.odom_label} odometry")
    (veh_pt,) = axm.plot([], [], "o", c="darkorange", ms=8)
    if pred_e is not None:
        (pred_ln,) = axm.plot([], [], "-", c="black", lw=1.6, label="Proposed (pole-corrected)")
    if loc_e is not None:
        loc_sc = axm.scatter([], [], c="red", marker="o", s=18, label="Localized poles", zorder=3)
    axm.axis("equal"); axm.legend(loc="upper left", fontsize=8)
    axm.set_xlim(min(pe.min(), ge.min()) - 50, max(pe.max(), ge.max()) + 50)
    axm.set_ylim(min(pn.min(), gn.min()) - 50, max(pn.max(), gn.max()) + 50)

    axe.set_title("Localization error vs time")
    axe.set_xlabel("Elapsed time (min)"); axe.set_ylabel("Error vs GNSS (m)")
    axe.plot(t, fl_err, c="darkorange", lw=1.0, alpha=0.5)
    if pred_err is not None and pred_t is not None:
        axe.plot(pred_t, pred_err, ".", c="black", ms=3, alpha=0.6)   # sparse, own time
    (cursor,) = axe.plot([], [], c="grey", ls="--")
    (fl_dot,) = axe.plot([], [], "o", c="darkorange", label=f"{args.odom_label} odom (med {np.median(fl_err):.1f} m)")
    if pred_err is not None:
        (pred_dot,) = axe.plot([], [], "o", c="black",
                               label=f"Proposed (med {np.nanmedian(pred_err):.1f} m)")
    axe.legend(loc="upper left", fontsize=8); axe.set_ylim(0, np.nanpercentile(fl_err, 99) * 1.2)

    def upd(k):
        j = idx[k]
        tc = t[j]                                             # current elapsed time (min)
        gnss_ln.set_data(ge[:j + 1], gn[:j + 1])
        fl_ln.set_data(fe[:j + 1], fn[:j + 1])
        veh_pt.set_data([fe[j]], [fn[j]])
        arts = [gnss_ln, fl_ln, veh_pt, cursor, fl_dot]
        cursor.set_data([tc, tc], [0, axe.get_ylim()[1]])
        fl_dot.set_data([tc], [fl_err[j]])
        if pred_t is not None:                                # reveal results by THEIR time
            rm = pred_t <= tc
            if pred_e is not None:
                pred_ln.set_data(pred_e[rm], pred_n[rm]); arts.append(pred_ln)
            if loc_e is not None:
                good = rm & ~np.isnan(loc_e)
                loc_sc.set_offsets(np.c_[loc_e[good], loc_n[good]] if good.any()
                                   else np.empty((0, 2))); arts.append(loc_sc)
            if pred_err is not None and rm.any():
                pred_dot.set_data([pred_t[rm][-1]], [pred_err[rm][-1]]); arts.append(pred_dot)
        return arts

    anim = FuncAnimation(fig, upd, frames=len(idx), interval=1000 / args.fps, blit=False)
    mp4 = out / "temporal_evolution.mp4"
    gif = out / "temporal_evolution.gif"
    try:
        anim.save(str(mp4), fps=args.fps, dpi=120)
        print("[viz] wrote", mp4)
    except Exception as e:
        print("[viz] ffmpeg unavailable (", e, ") -> GIF")
        anim.save(str(gif), fps=min(args.fps, 15), dpi=90)
        print("[viz] wrote", gif)
    plt.close(fig)


def _static_summaries(out, fe, fn, ge, gn, pe, pn, fl_err, t,
                      pred_e, pred_n, pred_err, pred_t=None, odom="Faster-LIO"):
    # (1) trajectory overlay
    fig, ax = plt.subplots(figsize=(9, 8))
    ax.scatter(pe, pn, c="green", marker="x", s=25, label="Snow poles (GT)")
    ax.plot(ge, gn, c="royalblue", lw=1.5, label="GNSS reference")
    ax.plot(fe, fn, c="darkorange", lw=1.5, label=f"{odom} odometry")
    if pred_e is not None:
        ax.plot(pred_e, pred_n, c="black", lw=1.2, label="Proposed (pole-corrected)")
    ax.axis("equal"); ax.legend(); ax.set_title("Trajectories + snow poles (UTM33N)")
    ax.set_xlabel("Easting (m)"); ax.set_ylabel("Northing (m)")
    fig.savefig(out / "summary_trajectories.png", dpi=150, bbox_inches="tight"); plt.close(fig)

    # (2) error histogram
    fig, ax = plt.subplots(figsize=(8, 6))
    ax.hist(fl_err, bins=30, color="darkorange", alpha=0.6, label=odom, edgecolor="k")
    if pred_err is not None:
        ax.hist(pred_err[~np.isnan(pred_err)], bins=30, color="royalblue", alpha=0.6,
                label="Proposed", edgecolor="k")
    ax.axvline(np.median(fl_err), c="darkorange", ls="--", label=f"{odom} median {np.median(fl_err):.2f} m")
    if pred_err is not None:
        ax.axvline(np.nanmedian(pred_err), c="royalblue", ls="--",
                   label=f"Proposed median {np.nanmedian(pred_err):.2f} m")
    ax.set_xlabel("Error vs GNSS (m)"); ax.set_ylabel("Frequency"); ax.legend()
    ax.set_title("Error histogram")
    fig.savefig(out / "summary_error_hist.png", dpi=150, bbox_inches="tight"); plt.close(fig)

    # (3) error CDF
    fig, ax = plt.subplots(figsize=(8, 6))
    for err, c, lab in [(fl_err, "darkorange", odom),
                        (pred_err, "royalblue", "Proposed")]:
        if err is None:
            continue
        e = np.sort(err[~np.isnan(err)])
        ax.plot(e, np.linspace(0, 1, len(e)), c=c, label=lab)
    ax.set_xlabel("Error vs GNSS (m)"); ax.set_ylabel("CDF"); ax.grid(True, alpha=.3)
    ax.legend(); ax.set_title("Error CDF")
    fig.savefig(out / "summary_error_cdf.png", dpi=150, bbox_inches="tight"); plt.close(fig)

    # (4) error vs distance travelled
    dist = np.concatenate([[0], np.cumsum(np.hypot(np.diff(ge), np.diff(gn)))]) / 1000.0
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(dist, fl_err, c="darkorange", label=f"{odom} odometry")
    if pred_err is not None:
        # map sparse results onto distance via their own time (pred_t) -> t -> dist
        pdist = np.interp(pred_t, t, dist) if pred_t is not None else dist[:len(pred_err)]
        ax.plot(pdist, pred_err, ".", c="royalblue", ms=4, label="Proposed (pole-corrected)")
    ax.set_xlabel("Distance travelled (km)"); ax.set_ylabel("Error vs GNSS (m)")
    ax.legend(); ax.set_title("Error vs distance")
    fig.savefig(out / "summary_error_vs_distance.png", dpi=150, bbox_inches="tight"); plt.close(fig)

    # console table
    def stats(name, err):
        if err is None:
            return
        e = err[~np.isnan(err)]
        print(f"   {name:12s} mean {e.mean():6.2f}  median {np.median(e):6.2f}  "
              f"p95 {np.percentile(e,95):6.2f}  max {e.max():6.2f}")
    print("[viz] error summary (m):")
    stats(odom, fl_err); stats("Proposed", pred_err)
    print("[viz] wrote 4 summary PNGs ->", out)


if __name__ == "__main__":
    main()
