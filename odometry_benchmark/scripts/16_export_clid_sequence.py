"""SnowPole scans + IMU -> the sequence layout read by CLID-SLAM (DUTRobot/CLID-SLAM, RA-L 2025).

  python scripts/16_export_clid_sequence.py            # -> cache/clid_seq/{lidar,imu}, pose_ts.txt

Same layout as the repository's converter (dataset/converter/rosbag2dataset_parallel.py):
  lidar/{i}.ply   x, y, z, intensity, timestamp  (the shared 2-150 m scans in cache/scans, Ouster
                  reflectivity from cache/scans_intensity, per-point time; CLID-SLAM normalises
                  per-point times to 0..1 itself before deskewing)
  imu/{i}.csv     header + rows (dt, acc_x..z, gyro_x..z) for the IMU samples between scan i-1 and
                  scan i, i >= 1; dt is the gap to the previous sample (to scan i-1 for the first)
  pose_ts.txt     header + one stamp per scan, which CLID-SLAM writes into its TUM output
The converter pairs IMU with scans by bag record time; here both come from the Ouster sensor clock
(cache/imu.csv and cache/scan_stamps.csv, /ouster/imu and /ouster/points header stamps), which
removes the recording jitter but is otherwise the same split.
"""
import argparse
from pathlib import Path

import numpy as np
import pandas as pd
from plyfile import PlyData, PlyElement

ROOT = Path(__file__).resolve().parent.parent


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cache", default=str(ROOT / "cache"))
    ap.add_argument("--out", default=str(ROOT / "cache" / "clid_seq"))
    args = ap.parse_args()
    cache, out = Path(args.cache), Path(args.out)
    (out / "lidar").mkdir(parents=True, exist_ok=True)
    (out / "imu").mkdir(parents=True, exist_ok=True)

    stamps = pd.read_csv(cache / "scan_stamps.csv")
    imu = pd.read_csv(cache / "imu.csv")
    t_imu = imu.t.to_numpy()
    ts = stamps.sensor_stamp.to_numpy()

    for i, idx in enumerate(stamps.idx):
        scan = np.load(cache / "scans" / f"{idx:06d}.npy")
        refl = np.load(cache / "scans_intensity" / f"{idx:06d}.npy").astype(np.float32)
        v = np.empty(len(scan), dtype=[(k, "f4") for k in ("x", "y", "z", "intensity", "timestamp")])
        v["x"], v["y"], v["z"], v["intensity"], v["timestamp"] = scan[:, 0], scan[:, 1], scan[:, 2], refl, scan[:, 3]
        PlyData([PlyElement.describe(v, "vertex")], text=False).write(str(out / "lidar" / f"{i}.ply"))
        if i > 0:
            sel = (t_imu > ts[i - 1]) & (t_imu <= ts[i])
            seg = imu[sel]
            dt = np.diff(np.concatenate([[ts[i - 1]], seg.t.to_numpy()]))
            pd.DataFrame({"timestamp": dt, "acc_x": seg.ax, "acc_y": seg.ay, "acc_z": seg.az,
                          "gyro_x": seg.gx, "gyro_y": seg.gy, "gyro_z": seg.gz}).to_csv(
                out / "imu" / f"{i}.csv", index=False, float_format="%.9f")
        if i % 500 == 0:
            print(f"{i}/{len(stamps)}", flush=True)
    pd.DataFrame({"timestamp": ts}).to_csv(out / "pose_ts.txt", index=False, float_format="%.9f")
    print(len(stamps), "scans ->", out)


if __name__ == "__main__":
    main()
