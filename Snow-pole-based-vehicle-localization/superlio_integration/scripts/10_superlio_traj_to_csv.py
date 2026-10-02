#!/usr/bin/env python3
"""
Stage 1 (the drop-in bridge): convert a Super-LIO odometry run into the CSV that
the snow-pole localization pipeline already consumes
(`incremental_navigation_results.csv`), so Super-LIO transparently replaces
FastReg as the LiDAR odometry source.

This is a 1:1 copy of the Faster-LIO / GLIM bridge maths (GNSS Kabsch start-anchor
-> UTM33N, resample to the 5422 GNSS frames) so the three backends are compared on
an identical footing. The ONLY differences: it reads Super-LIO's `/lio/odom`
(nav_msgs/Odometry, recorded to a ROS2 bag by 00_run_superlio.sh) and defaults its
output to `incremental_navigation_results_superlio.csv`.

What it does
------------
1. Reads the Super-LIO trajectory from the recorded odometry bag (/lio/odom) or,
   as a fallback, from a TUM trajectory text file.
2. Reads the dataset bag for GNSS (both antennas) and the IMU clock, so we can:
      - build the target per-frame timestamps (one row per GNSS frame: 5422 rows),
      - map Super-LIO's Ouster-sensor-clock stamps onto the GNSS/Unix-epoch
        timeline (affine fit from the IMU, which shares the sensor clock),
      - anchor the local Super-LIO frame into UTM33N (EPSG:32633).
3. Aligns the trajectory to UTM with a rigid 2-D transform (rotation + translation,
   NO scale):
      --align start (default): use ONLY the first `start_window_m` metres so the
        alignment fixes the initial pose + heading and nothing else (fair,
        odometry-only comparison; drift preserved), matching the FastReg baseline.
      --align full: Kabsch over the whole track (best-fit overlay) — viz only.
4. Resamples onto the GNSS frame timestamps and writes a CSV with the same
   `easting`/`northing` columns the pipeline reads.

Deps:  numpy pandas scipy pyproj rosbags   (repo-root .baginspect_venv)
   e.g.  source ../../.baginspect_venv/bin/activate && pip install pyproj pandas scipy
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd


# --------------------------------------------------------------------------- #
# bag reading (pure-python rosbags; reads BOTH ROS1 .bag and ROS2 bags)
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
    # pair by index (both 10 Hz, interleaved); average the two antennas
    return tL[:n], (latL[:n] + latR[:n]) / 2.0, (lonL[:n] + lonR[:n]) / 2.0


def imu_clock_affine(dataset_bag):
    """Fit epoch = a*sensor_clock + b from the IMU (shares the LiDAR sensor clock).
    IMU messages are tiny, so this is cheap even on the 39 GB bag."""
    s, e = [], []
    with _reader(dataset_bag) as r:
        for c in r.connections:
            if c.topic == "/ouster/imu":
                for conn, t, raw in r.messages(connections=[c]):
                    m = r.deserialize(raw, conn.msgtype)
                    s.append(m.header.stamp.sec + m.header.stamp.nanosec / 1e9)
                    e.append(t / 1e9)
    s, e = np.array(s), np.array(e)
    a, b = np.polyfit(s, e, 1)          # a ~ 1.0
    resid = np.std(e - (a * s + b))
    print(f"[clock] epoch = {a:.9f}*sensor + {b:.3f}   (fit residual {resid*1e3:.1f} ms)")
    return a, b


def read_odom_bag(odom_bag):
    """Return t(sensor s), x, y from a recorded odometry bag.
    Super-LIO publishes nav_msgs/Odometry on /lio/odom (stamp = sensor clock)."""
    t, x, y = [], [], []
    with _reader(odom_bag) as r:
        for c in r.connections:
            if c.topic in ("/lio/odom", "/Odometry", "/odometry", "/aft_mapped_to_init"):
                for conn, _, raw in r.messages(connections=[c]):
                    m = r.deserialize(raw, conn.msgtype)
                    t.append(m.header.stamp.sec + m.header.stamp.nanosec / 1e9)
                    p = m.pose.pose.position
                    x.append(p.x); y.append(p.y)
    if not t:
        raise RuntimeError("no /lio/odom messages found in " + odom_bag)
    return np.array(t), np.array(x), np.array(y)


def read_tum(txt):
    """Fallback: TUM file `t x y z qx qy qz qw` (sensor-clock timestamps)."""
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
                    help="original dataset bag (for GNSS + IMU clock); the full ROS1 bag")
    ap.add_argument("--odom-bag", help="recorded /lio/odom bag from stage 0")
    ap.add_argument("--tum", help="TUM trajectory file (fallback if no odom bag)")
    ap.add_argument("--out", default="incremental_navigation_results_superlio.csv")
    ap.add_argument("--align", choices=["start", "full"], default="start")
    ap.add_argument("--start-skip-m", type=float, default=20.0,
                    help="[align=start] skip this much initial GNSS travel (idle)")
    ap.add_argument("--start-window-m", type=float, default=400.0,
                    help="[align=start] metres of travel used to fix pose+heading")
    ap.add_argument("--epsg", default="32633", help="target UTM EPSG (33N for this site)")
    args = ap.parse_args()

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

    # 2) Super-LIO trajectory (sensor clock) -> epoch
    if args.odom_bag:
        t_sl_s, sx, sy = read_odom_bag(args.odom_bag)
    elif args.tum:
        t_sl_s, sx, sy = read_tum(args.tum)
    else:
        sys.exit("provide --odom-bag or --tum")
    a, b = imu_clock_affine(args.dataset_bag)
    t_sl = a * t_sl_s + b
    order = np.argsort(t_sl)
    t_sl, sx, sy = t_sl[order], sx[order], sy[order]
    print(f"[slio] {len(t_sl)} poses  span {t_sl[-1]-t_sl[0]:.1f}s")

    # 3) rigid alignment SLIO(local) -> UTM
    gx = np.interp(t_sl, t_gnss, gnss_e)          # GNSS sampled at SLIO times
    gy = np.interp(t_sl, t_gnss, gnss_n)
    inside = (t_sl >= t_gnss[0]) & (t_sl <= t_gnss[-1])
    if args.align == "start":
        # Fix the initial pose + heading using a MOVING band of GNSS travel,
        # skipping any stationary idle at the start (a static blob has no defined
        # heading and would corrupt the rotation estimate).
        gstep = np.hypot(np.diff(gx), np.diff(gy))
        gtrav = np.concatenate([[0], np.cumsum(gstep)])
        lo, hi = args.start_skip_m, args.start_skip_m + args.start_window_m
        sel = inside & (gtrav >= lo) & (gtrav <= hi)
        if sel.sum() < 10:                        # fallback: everything up to hi
            sel = inside & (gtrav <= hi)
        print(f"[align] start-anchor on GNSS travel [{lo},{hi}] m "
              f"({sel.sum()} samples, idle skipped) — odometry drift preserved")
    else:
        sel = inside
        print(f"[align] full Kabsch over {sel.sum()} samples — best-fit overlay")
    R, t = kabsch_2d(np.c_[sx[sel], sy[sel]], np.c_[gx[sel], gy[sel]])
    P = (R @ np.c_[sx, sy].T).T + t
    sl_e, sl_n = P[:, 0], P[:, 1]

    # residual vs GNSS (info only)
    err = np.hypot(sl_e[inside] - gx[inside], sl_n[inside] - gy[inside])
    print(f"[align] SLIO-vs-GNSS after {args.align}: mean {err.mean():.2f} m  "
          f"median {np.median(err):.2f} m  max {err.max():.2f} m")

    # 4) resample onto the GNSS frame timeline (reproduces original CSV row layout)
    east = np.interp(t_gnss, t_sl, sl_e)
    north = np.interp(t_gnss, t_sl, sl_n)
    out_lon, out_lat = to_ll.transform(east, north)

    df = pd.DataFrame({
        "Time": t_gnss,
        "latitude": out_lat, "longitude": out_lon,
        "easting": east, "northing": north,          # <-- columns the pipeline reads
        "gnss_easting": gnss_e, "gnss_northing": gnss_n,   # reference track for plots
        "heading": bearing_from_utm(east, north),
        "gnss_error": np.hypot(east - gnss_e, north - gnss_n),
    })
    df.to_csv(args.out, index=False)
    print(f"[out]  wrote {len(df)} rows -> {args.out}")
    print("       point the pipeline at this file (see superlio_integration/README.md).")


if __name__ == "__main__":
    main()
