#!/usr/bin/env python3
"""
Stage 10 (the drop-in bridge): convert a GLIM odometry run into the CSV the
snow-pole localization pipeline already consumes
(`incremental_navigation_results.csv`), so GLIM transparently replaces
Faster-LIO / FastReg as the LiDAR odometry source.

Adapted 1:1 from fasterlio_integration/scripts/10_fasterlio_traj_to_csv.py — the
alignment maths (GNSS Kabsch start-anchor into UTM33N) is identical; only the
trajectory source (GLIM's TUM dump) and a timestamp-base guard differ.

What it does
------------
1. Reads GLIM's estimated trajectory from `traj_lidar.txt` (TUM: t x y z qx qy qz qw),
   or, optionally, from a recorded ROS2 /glim_ros/odom bag.
2. Reads the ORIGINAL dataset bag for GNSS (both antennas) + the IMU clock, to:
      - build per-frame target timestamps (one row per GNSS frame, like the
        original CSV: 5422 rows),
      - map GLIM's Ouster-sensor-clock stamps onto the GNSS/Unix-epoch timeline
        (affine fit from the IMU, which shares the sensor clock),
      - anchor the local GLIM frame into UTM33N (EPSG:32633).
   GUARD: if GLIM's stamps look relative (not on the sensor clock), re-base them
   to the IMU start so the affine still applies.
3. Rigidly aligns GLIM(local) -> UTM (rotation+translation, NO scale):
      --align start (default): use ONLY a moving band of initial GNSS travel so
        the fit fixes the initial pose + heading and nothing else (drift preserved
        — the fair odometry-only metric, matching the FastReg/Faster-LIO baselines).
      --align full: Kabsch over the whole track (best-fit overlay) — viz only.
4. Resamples onto the GNSS frame timestamps and writes the CSV with the same
   `easting`/`northing` columns the pipeline reads.

Deps:  numpy pandas scipy pyproj rosbags
   e.g.  source ../../.baginspect_venv/bin/activate && pip install pyproj pandas scipy
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd


# --------------------------------------------------------------------------- #
# bag reading (pure-python rosbags; no ROS install needed)
# --------------------------------------------------------------------------- #
def _reader(path):
    from rosbags.highlevel import AnyReader
    return AnyReader([Path(path)])


def read_gnss_frames(dataset_bag):
    """Return per-frame arrays: recv_time(epoch s), lat, lon (mean of L/R antennas)."""
    with _reader(dataset_bag) as r:
        def grab(topic):
            ts, lat, lon = [], [], []
            for c in r.connections:
                if c.topic == topic:
                    for conn, t, raw in r.messages(connections=[c]):
                        m = r.deserialize(raw, conn.msgtype)
                        ts.append(t / 1e9); lat.append(m.latitude); lon.append(m.longitude)
            return np.array(ts), np.array(lat), np.array(lon)
        tL, latL, lonL = grab("/gps_left_position")
        tR, latR, lonR = grab("/gps_right_position")
    n = min(len(tL), len(tR))
    return tL[:n], (latL[:n] + latR[:n]) / 2.0, (lonL[:n] + lonR[:n]) / 2.0


def imu_clock_affine(dataset_bag):
    """Fit epoch = a*sensor_clock + b from the IMU (shares the LiDAR sensor clock).
    Returns (a, b, s_min, s_max) so callers can sanity-check the trajectory base."""
    s, e = [], []
    with _reader(dataset_bag) as r:
        for c in r.connections:
            if c.topic == "/ouster/imu":
                for conn, t, raw in r.messages(connections=[c]):
                    m = r.deserialize(raw, conn.msgtype)
                    s.append(m.header.stamp.sec + m.header.stamp.nanosec / 1e9)
                    e.append(t / 1e9)
    s, e = np.array(s), np.array(e)
    a, b = np.polyfit(s, e, 1)
    resid = np.std(e - (a * s + b))
    print(f"[clock] epoch = {a:.9f}*sensor + {b:.3f}   (fit residual {resid*1e3:.1f} ms)")
    return a, b, float(s.min()), float(s.max())


def read_glim_odom_bag(odom_bag):
    """Return t(sensor s), x, y from a recorded ROS2 /glim_ros/odom bag."""
    t, x, y = [], [], []
    with _reader(odom_bag) as r:
        for c in r.connections:
            if c.topic in ("/glim_ros/odom", "/glim_ros/pose", "/Odometry", "/odom"):
                for conn, _, raw in r.messages(connections=[c]):
                    m = r.deserialize(raw, conn.msgtype)
                    t.append(m.header.stamp.sec + m.header.stamp.nanosec / 1e9)
                    p = m.pose.pose.position
                    x.append(p.x); y.append(p.y)
    if not t:
        raise RuntimeError("no odometry messages found in " + odom_bag)
    return np.array(t), np.array(x), np.array(y)


def read_tum(txt):
    """TUM file `t x y z qx qy qz qw` (sensor-clock timestamps)."""
    d = np.loadtxt(txt)
    return d[:, 0], d[:, 1], d[:, 2]


# --------------------------------------------------------------------------- #
# geometry
# --------------------------------------------------------------------------- #
def kabsch_2d(src, dst):
    """Rigid 2-D transform (R,t), scale fixed to 1, mapping src -> dst (least sq)."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    S, D = src - mu_s, dst - mu_d
    H = S.T @ D
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1.0, d]) @ U.T
    t = mu_d - R @ mu_s
    return R, t


def bearing_from_utm(e, n):
    """Geodesic-style heading (deg, clockwise from north) from consecutive UTM pts."""
    de, dn = np.diff(e), np.diff(n)
    h = (np.degrees(np.arctan2(de, dn)) + 360.0) % 360.0
    return np.concatenate([[np.nan], h])


# --------------------------------------------------------------------------- #
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dataset-bag", required=True,
                    help="original dataset bag (for GNSS + IMU clock)")
    ap.add_argument("--tum", help="GLIM TUM trajectory (default: output/glim_traj_gpu.txt)")
    ap.add_argument("--odom-bag", help="recorded ROS2 /glim_ros/odom bag (alternative to --tum)")
    ap.add_argument("--out", default="incremental_navigation_results_glim.csv")
    ap.add_argument("--align", choices=["start", "full"], default="start")
    ap.add_argument("--start-skip-m", type=float, default=20.0,
                    help="[align=start] skip this much initial GNSS travel (idle)")
    ap.add_argument("--start-window-m", type=float, default=400.0,
                    help="[align=start] metres of travel used to fix pose+heading")
    ap.add_argument("--epsg", default="32633", help="target UTM EPSG (33N for this site)")
    args = ap.parse_args()

    if not args.tum and not args.odom_bag:
        here = Path(__file__).resolve().parent.parent / "output" / "glim_traj_gpu.txt"
        if here.exists():
            args.tum = str(here)
        else:
            sys.exit("provide --tum or --odom-bag (no default output/glim_traj_gpu.txt found)")

    try:
        from pyproj import Transformer
    except ImportError:
        sys.exit("pyproj is required:  pip install pyproj")

    to_utm = Transformer.from_crs("EPSG:4326", f"EPSG:{args.epsg}", always_xy=True)
    to_ll = Transformer.from_crs(f"EPSG:{args.epsg}", "EPSG:4326", always_xy=True)

    # 1) GNSS reference frames (target timeline + ground-truth track for eval/plots)
    t_gnss, lat, lon = read_gnss_frames(args.dataset_bag)
    gnss_e, gnss_n = to_utm.transform(lon, lat)
    gnss_e, gnss_n = np.asarray(gnss_e), np.asarray(gnss_n)
    print(f"[gnss] {len(t_gnss)} frames  E[{gnss_e.min():.0f},{gnss_e.max():.0f}] "
          f"N[{gnss_n.min():.0f},{gnss_n.max():.0f}]")

    # 2) GLIM trajectory (sensor clock) -> epoch
    if args.odom_bag:
        t_g_s, gx_l, gy_l = read_glim_odom_bag(args.odom_bag)
    else:
        t_g_s, gx_l, gy_l = read_tum(args.tum)
    a, b, s_min, s_max = imu_clock_affine(args.dataset_bag)

    # GUARD: GLIM should stamp poses on the Ouster sensor clock (~13214 s). If the
    # stamps instead look relative (start near 0 / far below the IMU clock), re-base
    # the trajectory start to the first IMU sample so the affine still applies.
    if t_g_s.min() < s_min - 100.0:
        shift = s_min - t_g_s.min()
        print(f"[guard] GLIM stamps look relative (min {t_g_s.min():.3f} vs IMU {s_min:.1f}); "
              f"re-basing by +{shift:.3f}s to the sensor clock")
        t_g_s = t_g_s + shift

    t_g = a * t_g_s + b
    order = np.argsort(t_g)
    t_g, gx_l, gy_l = t_g[order], gx_l[order], gy_l[order]
    print(f"[glim] {len(t_g)} poses  span {t_g[-1]-t_g[0]:.1f}s")

    # 3) rigid alignment GLIM(local) -> UTM
    gx = np.interp(t_g, t_gnss, gnss_e)          # GNSS sampled at GLIM times
    gy = np.interp(t_g, t_gnss, gnss_n)
    inside = (t_g >= t_gnss[0]) & (t_g <= t_gnss[-1])
    if args.align == "start":
        gstep = np.hypot(np.diff(gx), np.diff(gy))
        gtrav = np.concatenate([[0], np.cumsum(gstep)])
        lo, hi = args.start_skip_m, args.start_skip_m + args.start_window_m
        sel = inside & (gtrav >= lo) & (gtrav <= hi)
        if sel.sum() < 10:
            sel = inside & (gtrav <= hi)
        print(f"[align] start-anchor on GNSS travel [{lo},{hi}] m "
              f"({sel.sum()} samples, idle skipped) — odometry drift preserved")
    else:
        sel = inside
        print(f"[align] full Kabsch over {sel.sum()} samples — best-fit overlay")
    R, t = kabsch_2d(np.c_[gx_l[sel], gy_l[sel]], np.c_[gx[sel], gy[sel]])
    P = (R @ np.c_[gx_l, gy_l].T).T + t
    fl_e, fl_n = P[:, 0], P[:, 1]

    err = np.hypot(fl_e[inside] - gx[inside], fl_n[inside] - gy[inside])
    print(f"[align] GLIM-vs-GNSS after {args.align}: mean {err.mean():.2f} m  "
          f"median {np.median(err):.2f} m  max {err.max():.2f} m")

    # 4) resample onto the GNSS frame timeline (reproduces original CSV row layout)
    east = np.interp(t_gnss, t_g, fl_e)
    north = np.interp(t_gnss, t_g, fl_n)
    out_lon, out_lat = to_ll.transform(east, north)

    df = pd.DataFrame({
        "Time": t_gnss,
        "latitude": out_lat, "longitude": out_lon,
        "easting": east, "northing": north,          # <-- columns the pipeline reads
        "gnss_easting": gnss_e, "gnss_northing": gnss_n,
        "heading": bearing_from_utm(east, north),
        "gnss_error": np.hypot(east - gnss_e, north - gnss_n),
    })
    df.to_csv(args.out, index=False)
    print(f"[out]  wrote {len(df)} rows -> {args.out}")
    print("       run the pipeline with INCREMENTAL_NAV_CSV pointing at this file.")


if __name__ == "__main__":
    main()
