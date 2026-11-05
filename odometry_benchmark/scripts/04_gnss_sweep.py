"""Snow-pole corrected vehicle-localization error vs. GNSS availability.

A faithful replay of the reference loop in
polegeo_odo_swaps/.../snowpole_based_vehicle_localization_GNSS_percentage.py
(dead-reckoning from odometry headings/translations, random GNSS fixes with
random.seed(GNSS_SEED) drawn once per frame i>0, YOLO pole detections inside the
ground-truth bounding box, 5 m gate, sequential per-box snap to the nearest
ground-truth pole, one error per frame vs. the kriged GNSS track). The only
difference is that the backend-independent YOLO/range-image work is read from
cache/detections.pkl (03_cache_detections.py) instead of being recomputed.

Usage:
  python scripts/04_gnss_sweep.py --csv outputs/csv/incremental_navigation_results_rko_lio.csv \
      --name RKO-LIO --pcts 0 10 25 50
"""
import argparse
import contextlib
import io
import json
import os
import pickle
import random
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from pyproj import CRS, Geod, Transformer

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE / "polegeo"))
os.environ.setdefault("MPL_BACKEND", "Agg")
import geoloc_utils as gu  # noqa: E402

quiet = lambda: contextlib.redirect_stdout(io.StringIO())  # helpers print on every call

utm33n, wgs84 = CRS("EPSG:32633"), CRS("EPSG:4326")
to_wgs84 = Transformer.from_crs(utm33n, wgs84)
to_utm33n = Transformer.from_crs(wgs84, utm33n)
geod = Geod(ellps="WGS84")
P_LL, P_UTM = "EPSG:4326", "EPSG:32633"
CACHE_DIR = HERE / "cache"


def reference_track(det):
    """Kriged GNSS position at every range-image frame (backend independent; cached)."""
    cache = CACHE_DIR / "kriged_gnss.npz"
    if cache.exists():
        d = np.load(cache)
        return d["e"], d["n"]
    left, right = det["left_gnss"], det["right_gnss"]
    lats = [(left[i + 1][0] + right[i + 1][0]) / 2 for i in range(len(det["frames"]))]
    lons = [(left[i + 1][1] + right[i + 1][1]) / 2 for i in range(len(det["frames"]))]
    plat, plon = gu.kriging_interpolation(det["timestamps_range"], det["timestamps_left_gnss"][1:5421],
                                          lats, lons)
    e, n = gu.transform_coordinates(P_LL, P_UTM, plon, plat)
    np.savez(cache, e=np.asarray(e), n=np.asarray(n), lat=plat, lon=plon)
    return np.asarray(e), np.asarray(n)


def odometry_increments(csv):
    data = pd.read_csv(csv)
    fe, fn = data["easting"].values, data["northing"].values
    lat, lon = to_wgs84.transform(fe, fn)
    headings, translations = [np.nan], [np.nan]
    for i in range(1, len(lat)):
        if np.isnan(lat[i]) or np.isnan(lat[i - 1]):
            headings.append(np.nan); translations.append(np.nan)
        else:
            az, _, dist = geod.inv(lon[i - 1], lat[i - 1], lon[i], lat[i])
            headings.append((az + 360) % 360); translations.append(dist)
    return fe, fn, headings, translations


def new_position(e, n, heading, translation):
    lat, lon = to_wgs84.transform(e, n)
    if not np.isnan(heading) and not np.isnan(translation):
        lon, lat, _ = geod.fwd(lon, lat, heading, translation)
    else:
        lat, lon = np.nan, np.nan
    return to_utm33n.transform(lat, lon)


def run(csv, pct, det, ref_e, ref_n, poles, bounds, seed=0):
    fe, fn, headings, translations = odometry_increments(csv)
    min_north, max_north, min_east, max_east = bounds
    random.seed(seed)
    pe = pn = None
    pred_e, pred_n = [], []
    err_pred, err_odom = [], []
    gnss_count = pred_count = 0
    for i, dets in enumerate(det["frames"]):
        if i == 0:
            pe, pn = ref_e[0], ref_n[0]
        elif random.uniform(0, 100) < pct:
            pe, pn = ref_e[i], ref_n[i]
            gnss_count += 1
        else:
            pe, pn = new_position(pe, pn, headings[i], translations[i])
            pred_count += 1
        pred_e.append(pe); pred_n.append(pn)
        if not (min_north <= pn <= max_north and min_east <= pe <= max_east):
            continue
        if i > 0:
            for center_xyz, new_distance, azimuth_gnss in dets:
                if new_distance > 5:
                    continue
                e1, n1 = pred_e[i - 1], pred_n[i - 1]
                e2, n2 = pe, pn
                with quiet():
                    heading = gu.calculate_vehicle_heading_from_two_utm(e1, n1, e2, n2)
                    lat2, lon2 = to_wgs84.transform(e2, n2)
                    te, tn, adj_az = gu.local_to_utm33_with_offset_proj(lat2, lon2, heading,
                                                                        azimuth_gnss, new_distance)
                back_az = (180 + adj_az) % 360
                d = np.hypot(poles[:, 0] - te, poles[:, 1] - tn)
                ge, gn = poles[int(np.argmin(d))]
                glat, glon = to_wgs84.transform(ge, gn)
                vlon, vlat, _ = geod.fwd(glon, glat, back_az, new_distance)
                pe, pn = to_utm33n.transform(vlat, vlon)
        err_pred.append(np.hypot(ref_e[i] - pe, ref_n[i] - pn))
        err_odom.append(np.hypot(ref_e[i] - fe[i], ref_n[i] - fn[i]))
    ep, eo = np.array(err_pred), np.array(err_odom)
    return {"gnss_pct": pct, "err_pred_median": round(float(np.median(ep)), 2),
            "err_pred_mean": round(float(ep.mean()), 2), "err_pred_max": round(float(ep.max()), 2),
            "err_odom_median": round(float(np.median(eo)), 2), "err_odom_mean": round(float(eo.mean()), 2),
            "n_scored_frames": int(len(ep)), "gnss_used": gnss_count, "predictive": pred_count}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", required=True)
    ap.add_argument("--name", required=True)
    ap.add_argument("--pcts", type=float, nargs="+", default=[0, 10, 25, 50])
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out-dir", default=str(HERE / "outputs" / "scores"))
    ap.add_argument("--cache-dir", default=str(HERE / "cache"))
    args = ap.parse_args()

    global CACHE_DIR
    CACHE_DIR = Path(args.cache_dir)
    with open(CACHE_DIR / "detections.pkl", "rb") as f:
        det = pickle.load(f)
    with quiet():
        ref_e, ref_n = reference_track(det)
        (gt_n, gt_e, min_north, max_north, min_east, max_east, *_rest) = \
            gu.load_gnss_data_and_calculate_utm_range(str(HERE / "polegeo" / "groundtruth_poles.csv"))
    poles = np.c_[gt_e, gt_n]

    rows = []
    for pct in args.pcts:
        r = run(args.csv, pct, det, ref_e, ref_n, poles, (min_north, max_north, min_east, max_east), args.seed)
        r["method"] = args.name
        rows.append(r)
        print(json.dumps(r), flush=True)
    out = Path(args.out_dir) / f"{args.name.replace(' ', '_')}_seed{args.seed}.csv"
    out.parent.mkdir(parents=True, exist_ok=True)
    pd.DataFrame(rows).to_csv(out, index=False)
    print(f"-> {out}")


if __name__ == "__main__":
    main()
