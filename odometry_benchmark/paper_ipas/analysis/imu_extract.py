"""Extract /ouster/imu from the SnowPole bag -> paper_ipas/data/imu.npz, and estimate its native noise.

  ../venv/Scripts/python paper_ipas/analysis/imu_extract.py      (from odometry_benchmark/)

Fields: t_bag (bag record time, ns), t (header stamp, s, Ouster sensor clock), gyro (rad/s), acc (m/s^2), frame_id.
Native noise = std over the stationary start (vehicle speed from GNSS < 0.05 m/s).
"""
import json
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "paper_ipas" / "data"


def main():
    t_bag, t, gyro, acc, frame = [], [], [], [], None
    with AnyReader([ROOT / "bag_full" / "2024-02-28-12-59-51.bag"]) as r:
        cons = [c for c in r.connections if c.topic == "/ouster/imu"]
        for c, tb, raw in r.messages(connections=cons):
            m = r.deserialize(raw, c.msgtype)
            frame = m.header.frame_id
            t_bag.append(tb)
            t.append(m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec)
            gyro.append((m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z))
            acc.append((m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z))
    t_bag, t, gyro, acc = np.array(t_bag, np.int64), np.array(t), np.array(gyro), np.array(acc)

    # stationary start: GNSS track (frame times ~ first LiDAR frame) moves < 0.05 m/s
    g = np.load(ROOT / "cache" / "kriged_gnss.npz")
    v = np.hypot(np.diff(g["e"]), np.diff(g["n"])) / 0.1
    n_still = int(np.argmax(v > 0.05))  # LiDAR frames before the vehicle moves
    still = t < t[0] + max(1.0, 0.1 * n_still)
    stats = {"n": int(len(t)), "rate_hz": round(float(1 / np.median(np.diff(t))), 2), "frame_id": frame,
             "stationary_s": round(float(t[still][-1] - t[0]), 2),
             "gyro_std_rad_s": np.std(gyro[still], axis=0).round(5).tolist(),
             "acc_std_m_s2": np.std(acc[still], axis=0).round(5).tolist(),
             "gyro_mean_rad_s": np.mean(gyro[still], axis=0).round(5).tolist(),
             "acc_mean_m_s2": np.mean(acc[still], axis=0).round(4).tolist()}
    np.savez(OUT / "imu.npz", t_bag=t_bag, t=t, gyro=gyro, acc=acc, frame_id=frame)
    (OUT / "imu_native_noise.json").write_text(json.dumps(stats, indent=2))
    print(json.dumps(stats, indent=2))


if __name__ == "__main__":
    main()
