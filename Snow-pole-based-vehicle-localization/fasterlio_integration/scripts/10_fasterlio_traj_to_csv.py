#!/usr/bin/env python3
"""
Stage 1 (the drop-in bridge): convert a Faster-LIO odometry run into the CSV
that the snow-pole localization pipeline already consumes
(`incremental_navigation_results.csv`), so Faster-LIO transparently replaces
FastReg as the LiDAR odometry source.

What it does
------------
1. Reads the Faster-LIO trajectory from the recorded odometry bag (/Odometry)
   or, as a fallback, from a TUM trajectory text file.
2. Reads the dataset bag for GNSS (both antennas) and the IMU clock, so we can:
      - build the target per-frame timestamps (one row per GNSS frame, like the
        original CSV: 5422 rows),
      - map Faster-LIO's Ouster-sensor-clock stamps onto the GNSS/Unix-epoch
        timeline (affine fit from the IMU, which shares the sensor clock),
      - anchor the local Faster-LIO frame into UTM33N (EPSG:32633).
3. Aligns the Faster-LIO trajectory to UTM with a rigid 2-D transform
   (rotation + translation, NO scale):
      --align start (default): use ONLY the first `start_window_m` metres so the
        alignment fixes the initial pose + heading and nothing else. This is the
        fair, odometry-only comparison (drift is preserved), matching how the
        FastReg baseline was anchored.
      --align full: Kabsch over the whole track (best-fit overlay) — use for the
        visualization / upper-bound, not for the drift error metric.
4. Resamples onto the GNSS frame timestamps and writes a CSV with the same
   `easting`/`northing` columns the pipeline reads (plus lat/lon/Time and the
   GNSS reference track for the temporal-evolution plots).

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


def read_fasterlio_odom_bag_full(odom_bag):
    """Return full 6-DoF pose arrays (t_sensor_s, x, y, z, qx, qy, qz, qw), sorted
    by time, from a recorded /Odometry bag. This is Faster-LIO's trajectory in its
    local init frame on the Ouster sensor clock — i.e. exactly what the node's
    Savetrajectory() dumps, so we can regenerate a guaranteed TUM file from the
    (authoritative) bag instead of the node's flaky shutdown save."""
    cols = [[] for _ in range(8)]
    with _reader(odom_bag) as r:
        for c in r.connections:
            if c.topic in ("/Odometry", "/odometry", "/aft_mapped_to_init"):
                for conn, _, raw in r.messages(connections=[c]):
                    m = r.deserialize(raw, conn.msgtype)
                    p = m.pose.pose.position
                    o = m.pose.pose.orientation
                    row = (m.header.stamp.sec + m.header.stamp.nanosec / 1e9,
                           p.x, p.y, p.z, o.x, o.y, o.z, o.w)
                    for dst, v in zip(cols, row):
                        dst.append(v)
    if not cols[0]:
        raise RuntimeError("no /Odometry messages found in " + odom_bag)
    arr = [np.asarray(col) for col in cols]
    order = np.argsort(arr[0])
    return [col[order] for col in arr]


def read_fasterlio_odom_bag(odom_bag):
    """Return t(sensor s), x, y from a recorded /Odometry bag."""
    t, x, y = read_fasterlio_odom_bag_full(odom_bag)[:3]
    return t, x, y


def write_tum(path, t, x, y, z, qx, qy, qz, qw):
    """Write a TUM trajectory `timestamp x y z qx qy qz qw` (sensor clock, FL local
    frame). np.savetxt prefixes the header with '# ', which np.loadtxt (the --tum
    fallback reader) skips, so the file round-trips."""
    arr = np.column_stack([t, x, y, z, qx, qy, qz, qw])
    np.savetxt(path, arr, header="timestamp x y z qx qy qz qw",
               fmt=["%.6f"] + ["%.15g"] * 7)
    print(f"[tum]  wrote {len(t)} poses -> {path}")


def read_fasterlio_tum(txt):
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
                    help="original dataset bag (for GNSS + IMU clock)")
    ap.add_argument("--odom-bag", help="recorded /Odometry bag from stage 0")
    ap.add_argument("--tum", help="TUM trajectory file (fallback if no odom bag)")
    ap.add_argument("--tum-out", help="also write a guaranteed TUM trajectory here "
                    "(derived from --odom-bag; replaces the node's flaky Savetrajectory)")
    ap.add_argument("--out", default="incremental_navigation_results_fasterlio.csv")
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

    # 2) Faster-LIO trajectory (sensor clock) -> epoch
    if args.odom_bag:
        t_fl_s, fx, fy, fz, qx, qy, qz, qw = read_fasterlio_odom_bag_full(args.odom_bag)
        if args.tum_out:                          # guaranteed TUM, straight from the bag
            write_tum(args.tum_out, t_fl_s, fx, fy, fz, qx, qy, qz, qw)
    elif args.tum:
        t_fl_s, fx, fy = read_fasterlio_tum(args.tum)
        if args.tum_out:
            sys.exit("--tum-out needs --odom-bag (full 6-DoF pose) as the source")
    else:
        sys.exit("provide --odom-bag or --tum")
    a, b = imu_clock_affine(args.dataset_bag)
    t_fl = a * t_fl_s + b
    order = np.argsort(t_fl)
    t_fl, fx, fy = t_fl[order], fx[order], fy[order]
    print(f"[fl]   {len(t_fl)} poses  span {t_fl[-1]-t_fl[0]:.1f}s")

    # 3) rigid alignment FL(local) -> UTM
    gx = np.interp(t_fl, t_gnss, gnss_e)          # GNSS sampled at FL times
    gy = np.interp(t_fl, t_gnss, gnss_n)
    inside = (t_fl >= t_gnss[0]) & (t_fl <= t_gnss[-1])
    if args.align == "start":
        # Fix the initial pose + heading using a MOVING band of GNSS travel,
        # skipping any stationary idle at the start. A stationary blob has no
        # defined heading and would corrupt the rotation estimate (this is what
        # made an earlier 30 m window fail: ~10 s of idle = a near-static cluster).
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
    R, t = kabsch_2d(np.c_[fx[sel], fy[sel]], np.c_[gx[sel], gy[sel]])
    P = (R @ np.c_[fx, fy].T).T + t
    fl_e, fl_n = P[:, 0], P[:, 1]

    # residual vs GNSS (info only)
    err = np.hypot(fl_e[inside] - gx[inside], fl_n[inside] - gy[inside])
    print(f"[align] FL-vs-GNSS after {args.align}: mean {err.mean():.2f} m  "
          f"median {np.median(err):.2f} m  max {err.max():.2f} m")

    # 4) resample onto the GNSS frame timeline (reproduces original CSV row layout)
    east = np.interp(t_gnss, t_fl, fl_e)
    north = np.interp(t_gnss, t_fl, fl_n)
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
    print("       point the pipeline at this file (see fasterlio_integration/README.md).")


if __name__ == "__main__":
    main()
