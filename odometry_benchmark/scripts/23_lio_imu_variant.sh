#!/usr/bin/env bash
# Runs inside fastlio2:noetic: one LIO method on SnowPole with a replacement IMU stream.
#   bash /work/scripts/23_lio_imu_variant.sh fastlio2|fasterlio <variant>
# /ouster/points is replayed from the original bag and /imu_deg from outputs/imu_variants/<variant>.bag, both at 1x;
# the node subscribes to /imu_deg (configs/imu_degradation/<method>.yaml = the paper's parameters + that topic).
# /Odometry is recorded and written as a TUM file in the Ouster sensor clock.
set -e
M=$1; V=$2
source /opt/ros/noetic/setup.bash
if [ $M = fastlio2 ]; then source /root/ws_fl2/devel/setup.bash; else source /root/ws/devel/setup.bash; fi
O=/work/outputs/imu_degradation/${M}_$V; mkdir -p $O
roscore > $O/roscore.log 2>&1 & sleep 5
roslaunch /cfg/$M.launch > $O/run.log 2>&1 & LAUNCH=$!
rosbag record -O $O/odom.bag /Odometry __name:=rec > $O/record.log 2>&1 & REC=$!
sleep 5
rosbag play --wait-for-subscribers -d 2 /bag/2024-02-28-12-59-51.bag /work/outputs/imu_variants/$V.bag \
  --topics /ouster/points /imu_deg > $O/play.log 2>&1
sleep 20
kill -INT $REC; kill -INT $LAUNCH; wait $LAUNCH || true; wait $REC || true
python3 - $O <<'PY'
import sys, rosbag
O = sys.argv[1]
n = 0
with open(f"{O}/tum.txt", "w") as f:
    f.write("#timestamp x y z q_x q_y q_z q_w\n")
    for _, m, _ in rosbag.Bag(f"{O}/odom.bag").read_messages(topics=["/Odometry"]):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        f.write(f"{m.header.stamp.to_sec():.6f} {p.x:.9f} {p.y:.9f} {p.z:.9f} {q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f}\n")
        n += 1
print(n, "poses")
PY
