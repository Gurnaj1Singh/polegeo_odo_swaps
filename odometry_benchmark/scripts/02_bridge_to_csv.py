"""Convert an odometry trajectory (TUM, Ouster sensor clock) into the per-GNSS-frame
UTM33N CSV consumed by the snow-pole localization pipeline.

Same maths as the reference repo's Faster-LIO / GLIM / Super-LIO bridges
(polegeo_odo_swaps .../10_superlio_traj_to_csv.py), so every backend sits on an
identical footing:
  1. GNSS track = mean of left/right antennas, one row per GNSS frame (5422 rows).
  2. sensor clock -> epoch via an affine fit on /ouster/imu (header stamp vs record time).
  3. rigid 2-D Kabsch (no scale) on GNSS travel [20, 420] m only -> fixes the
     initial pose + heading, odometry drift is preserved.
  4. resample onto the GNSS frame times -> `easting`, `northing` columns.

GNSS frames and the clock fit are read once from the full bag and cached.
"""
import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd
from pyproj import Transformer


def load_reference(bag, cache_file):
    cache_file = Path(cache_file)
    if cache_file.exists():
        d = np.load(cache_file)
        return d["t_gnss"], d["lat"], d["lon"], float(d["a"]), float(d["b"])
    from rosbags.highlevel import AnyReader
    tL, latL, lonL, tR, latR, lonR, s, e = [], [], [], [], [], [], [], []
    with AnyReader([Path(bag)]) as r:
        cons = [c for c in r.connections
                if c.topic in ("/gps_left_position", "/gps_right_position", "/ouster/imu")]
        for c, t, raw in r.messages(connections=cons):
            m = r.deserialize(raw, c.msgtype)
            if c.topic == "/ouster/imu":
                s.append(m.header.stamp.sec + m.header.stamp.nanosec / 1e9); e.append(t / 1e9)
            elif c.topic == "/gps_left_position":
                tL.append(t / 1e9); latL.append(m.latitude); lonL.append(m.longitude)
            else:
                tR.append(t / 1e9); latR.append(m.latitude); lonR.append(m.longitude)
    n = min(len(tL), len(tR))
    t_gnss = np.array(tL[:n])
    lat = (np.array(latL[:n]) + np.array(latR[:n])) / 2.0
    lon = (np.array(lonL[:n]) + np.array(lonR[:n])) / 2.0
    s, e = np.array(s), np.array(e)
    a, b = np.polyfit(s, e, 1)
    print(f"[clock] epoch = {a:.9f}*sensor + {b:.3f}  (residual {np.std(e - (a*s+b))*1e3:.1f} ms)")
    np.savez(cache_file, t_gnss=t_gnss, lat=lat, lon=lon, a=a, b=b)
    return t_gnss, lat, lon, a, b


def gravity_leveling(bag, seconds=1.0):
    """Rotation taking the first LiDAR frame to a gravity-aligned frame.

    LiDAR-only odometry (GenZ-ICP, MAD-ICP) expresses poses in the first scan's
    frame, which is tilted with the sensor mount. The vehicle is stationary at the
    start, so the mean IMU specific force over the first second is the up vector
    (IMU and LiDAR axes coincide on the Ouster, see Trip068.json)."""
    from rosbags.highlevel import AnyReader
    acc, t0 = [], None
    with AnyReader([Path(bag)]) as r:
        cons = [c for c in r.connections if c.topic == "/ouster/imu"]
        for c, t, raw in r.messages(connections=cons):
            t0 = t if t0 is None else t0
            if (t - t0) / 1e9 > seconds:
                break
            m = r.deserialize(raw, c.msgtype).linear_acceleration
            acc.append((m.x, m.y, m.z))
    up = np.mean(acc, axis=0)
    up /= np.linalg.norm(up)
    z = np.array([0.0, 0.0, 1.0])
    v, c = np.cross(up, z), float(up @ z)
    K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    R = np.eye(3) + K + K @ K / (1.0 + c)
    print(f"[level] start tilt {np.degrees(np.arccos(c)):.2f} deg removed (up = {up.round(4)})")
    return R


def kabsch_2d(src, dst):
    mu_s, mu_d = src.mean(0), dst.mean(0)
    H = (src - mu_s).T @ (dst - mu_d)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1.0, d]) @ U.T
    return R, mu_d - R @ mu_s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tum", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--bag", default="bag_full/2024-02-28-12-59-51.bag")
    ap.add_argument("--ref-cache", default="cache/gnss_clock.npz")
    ap.add_argument("--start-skip-m", type=float, default=20.0)
    ap.add_argument("--start-window-m", type=float, default=400.0)
    ap.add_argument("--level-gravity", action="store_true",
                    help="rotate a LiDAR-only trajectory into a gravity-aligned frame first")
    args = ap.parse_args()

    to_utm = Transformer.from_crs("EPSG:4326", "EPSG:32633", always_xy=True)
    to_ll = Transformer.from_crs("EPSG:32633", "EPSG:4326", always_xy=True)

    t_gnss, lat, lon, a, b = load_reference(args.bag, args.ref_cache)
    gnss_e, gnss_n = map(np.asarray, to_utm.transform(lon, lat))

    d = np.loadtxt(args.tum)
    t_od = a * d[:, 0] + b
    order = np.argsort(t_od)
    xyz = d[order, 1:4]
    if args.level_gravity:
        xyz = xyz @ gravity_leveling(args.bag).T
    t_od, sx, sy = t_od[order], xyz[:, 0], xyz[:, 1]

    gx = np.interp(t_od, t_gnss, gnss_e)
    gy = np.interp(t_od, t_gnss, gnss_n)
    inside = (t_od >= t_gnss[0]) & (t_od <= t_gnss[-1])
    gtrav = np.concatenate([[0], np.cumsum(np.hypot(np.diff(gx), np.diff(gy)))])
    lo, hi = args.start_skip_m, args.start_skip_m + args.start_window_m
    sel = inside & (gtrav >= lo) & (gtrav <= hi)
    if sel.sum() < 10:
        sel = inside & (gtrav <= hi)
    R, t = kabsch_2d(np.c_[sx[sel], sy[sel]], np.c_[gx[sel], gy[sel]])
    P = (R @ np.c_[sx, sy].T).T + t

    err = np.hypot(P[inside, 0] - gx[inside], P[inside, 1] - gy[inside])
    print(f"[align] start-anchor [{lo},{hi}] m on {sel.sum()} poses; raw odometry vs GNSS: "
          f"mean {err.mean():.2f} m, median {np.median(err):.2f} m, max {err.max():.2f} m")

    east = np.interp(t_gnss, t_od, P[:, 0])
    north = np.interp(t_gnss, t_od, P[:, 1])
    out_lon, out_lat = to_ll.transform(east, north)
    head = (np.degrees(np.arctan2(np.diff(east), np.diff(north))) + 360) % 360
    pd.DataFrame({
        "Time": t_gnss, "latitude": out_lat, "longitude": out_lon,
        "easting": east, "northing": north,
        "gnss_easting": gnss_e, "gnss_northing": gnss_n,
        "heading": np.r_[np.nan, head],
        "gnss_error": np.hypot(east - gnss_e, north - gnss_n),
    }).to_csv(args.out, index=False)
    stats = {"raw_vs_gnss_mean_m": round(float(err.mean()), 2),
             "raw_vs_gnss_median_m": round(float(np.median(err)), 2),
             "raw_vs_gnss_max_m": round(float(err.max()), 2), "n_poses": int(len(t_od))}
    Path(args.out).with_suffix(".align.json").write_text(json.dumps(stats, indent=2))
    print(f"[out] {len(east)} rows -> {args.out}")


if __name__ == "__main__":
    main()
