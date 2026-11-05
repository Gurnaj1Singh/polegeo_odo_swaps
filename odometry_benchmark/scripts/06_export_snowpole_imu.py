"""Add imu.csv and extrinsics.yaml to the SnowPole scan cache, so every method (incl.
RKO-LIO through its Python pipeline) can run from the same cache layout as the public
datasets. IMU stamps are /ouster/imu header stamps (Ouster sensor clock, like the scans).
T_lidar_imu = os_imu pose in os_sensor from Trip068.json (identity rotation, mm -> m).
"""
import argparse
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", default="../snow_pole_geo_localization_data/2024-02-28-12-59-51_no_unwanted_topics.bag")
    ap.add_argument("--cache", default="cache")
    args = ap.parse_args()
    rows = []
    with AnyReader([Path(args.bag)]) as r:
        cons = [c for c in r.connections if c.topic == "/ouster/imu"]
        for c, _, raw in r.messages(connections=cons):
            m = r.deserialize(raw, c.msgtype)
            a, g = m.linear_acceleration, m.angular_velocity
            rows.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, a.x, a.y, a.z, g.x, g.y, g.z))
    cache = Path(args.cache)
    np.savetxt(cache / "imu.csv", np.array(rows), fmt="%.9f", delimiter=",",
               header="t,ax,ay,az,gx,gy,gz", comments="")
    T = np.eye(4)
    T[:3, 3] = [0.006253, -0.011775, 0.007645]
    (cache / "extrinsics.yaml").write_text(f"T_lidar_imu: {T.tolist()}\n")
    print(f"wrote {len(rows)} IMU samples -> {cache / 'imu.csv'}")


if __name__ == "__main__":
    main()
