"""Degraded copies of the SnowPole IMU stream as small ROS 1 bags (topic /imu_deg), for the IMU-degradation study.
Runs in fastlio2:noetic (python3 + rosbag):
  python3 /work/scripts/22_make_imu_variants.py /work/paper_ipas/data/imu.npz /work/outputs/imu_variants

  rate100   all samples (control: same replay path as the variants, unchanged data)
  rate50/25/10  block averages of 2/4/10 consecutive samples, stamped at the last sample of each block
                (a lower-rate IMU low-pass filters before sampling)
  noise3/10 white Gaussian noise added so that the total noise std is 3x / 10x the native std measured on the
            stationary start (paper_ipas/data/imu_native_noise.json), per axis, for gyro and accelerometer; seed 0
Bag record times are the original ones, so `rosbag play` interleaves them correctly with /ouster/points.
"""
import json
import sys
from pathlib import Path

import numpy as np
import rosbag
import rospy
from sensor_msgs.msg import Imu

src, out = Path(sys.argv[1]), Path(sys.argv[2])
out.mkdir(parents=True, exist_ok=True)
d = np.load(src)
tb, t, gyro, acc, frame = d["t_bag"], d["t"], d["gyro"], d["acc"], str(d["frame_id"])
noise = json.loads((src.parent / "imu_native_noise.json").read_text())
sg, sa = np.array(noise["gyro_std_rad_s"]), np.array(noise["acc_std_m_s2"])


def write(name, tb_, t_, g_, a_):
    with rosbag.Bag(str(out / f"{name}.bag"), "w") as b:
        for i in range(len(t_)):
            m = Imu()
            m.header.seq, m.header.frame_id = i, frame
            m.header.stamp = rospy.Time.from_sec(float(t_[i]))
            m.orientation_covariance[0] = -1  # no orientation, as the Ouster driver
            m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = map(float, g_[i])
            m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = map(float, a_[i])
            b.write("/imu_deg", m, rospy.Time(nsecs=int(tb_[i])))
    print(name, len(t_), "samples", flush=True)


write("rate100", tb, t, gyro, acc)
for k in (2, 4, 10):
    n = len(t) // k * k
    blk = lambda x: x[:n].reshape(n // k, k, -1).mean(1)
    write(f"rate{100 // k}", tb[k - 1:n:k], t[k - 1:n:k], blk(gyro), blk(acc))
rng = np.random.default_rng(0)
for k in (3, 10):
    s = np.sqrt(k ** 2 - 1)  # added std so that total = k x native
    write(f"noise{k}", tb, t, gyro + rng.normal(0, s * sg, gyro.shape), acc + rng.normal(0, s * sa, acc.shape))
