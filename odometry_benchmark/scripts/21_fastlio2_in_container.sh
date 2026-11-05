#!/usr/bin/env bash
# Runs inside fastlio2:noetic (see run_fastlio2_snowpole.sh). FAST-LIO2 is a live ROS node with no offline
# mapper: replay /ouster/points + /ouster/imu at 1x, record /Odometry, then write a TUM file in the Ouster
# sensor clock (stamp = scan end time, pose = IMU in the first IMU frame, as Faster-LIO's trajectory log).
set -e
source /opt/ros/noetic/setup.bash; source /root/ws_fl2/devel/setup.bash
O=/work/outputs/fastlio2
roscore > $O/roscore.log 2>&1 & sleep 5
roslaunch /cfg/mapping.launch > $O/run.log 2>&1 & LAUNCH=$!
rosbag record -O $O/odom.bag /Odometry __name:=rec > $O/record.log 2>&1 & REC=$!
sleep 5
rosbag play --wait-for-subscribers -d 2 /bag/2024-02-28-12-59-51.bag --topics /ouster/points /ouster/imu \
  > $O/play.log 2>&1
sleep 20                                  # let the node finish its queue
kill -INT $REC; kill -INT $LAUNCH; wait $LAUNCH || true; wait $REC || true
cp /root/ws_fl2/src/FAST_LIO/Log/fast_lio_time_log.csv $O/ || true
python3 - <<'PY'
import json, rosbag
import numpy as np
O = "/work/outputs/fastlio2"
with open(f"{O}/fastlio2_tum.txt", "w") as f:
    f.write("#timestamp x y z q_x q_y q_z q_w\n")
    for _, m, _ in rosbag.Bag(f"{O}/odom.bag").read_messages(topics=["/Odometry"]):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        f.write(f"{m.header.stamp.to_sec():.6f} {p.x:.9f} {p.y:.9f} {p.z:.9f} {q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f}\n")
t = np.genfromtxt(f"{O}/fast_lio_time_log.csv", delimiter=",", skip_header=1)
total, pre = 1000 * t[:, 1], 1000 * t[:, 10]
json.dump({"n_scans": int(len(t)), "ms_per_scan_mean": round(float(total.mean()), 2),
           "ms_per_scan_median": round(float(np.median(total)), 2), "ms_per_scan_p95": round(float(np.percentile(total, 95)), 2),
           "preprocess_ms_mean": round(float(pre.mean()), 2),
           "scope": "FAST-LIO2 'total time' per scan (IMU propagation + undistort + downsample + IEKF + map update; "
                    "excl. point-cloud preprocessing in the ROS callback, reported separately)"},
          open(f"{O}/timing.json", "w"), indent=2)
print("trajectory + timing written")
PY
