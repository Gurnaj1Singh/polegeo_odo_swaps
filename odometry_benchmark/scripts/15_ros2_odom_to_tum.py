"""Export nav_msgs/Odometry from a ROS 2 (Jazzy) bag to TUM, using ROS 2's own rosbag2_py.
(The venv's pure-python `rosbags` 0.9.23 cannot read Jazzy's metadata version 9.)

Runs in the superlio:jazzy image:
  docker run --rm -v D:/Odometry/odometry_benchmark:/work superlio:jazzy bash -c \
    "source /opt/ros/jazzy/setup.bash && python3 /work/scripts/15_ros2_odom_to_tum.py \
       /work/outputs/superlio/superlio_odom /lio/odom /work/outputs/superlio/superlio_tum.txt"
"""
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag, topic, out = sys.argv[1:4]
reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=""), rosbag2_py.ConverterOptions("", ""))
types = {t.name: t.type for t in reader.get_all_topics_and_types()}
msg_type = get_message(types[topic])
rows = []
while reader.has_next():
    name, data, _ = reader.read_next()
    if name != topic:
        continue
    m = deserialize_message(data, msg_type)
    p, q = m.pose.pose.position, m.pose.pose.orientation
    rows.append(f"{m.header.stamp.sec + m.header.stamp.nanosec * 1e-9:.9f} {p.x:.9f} {p.y:.9f} {p.z:.9f} "
                f"{q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f}")
with open(out, "w") as f:
    f.write("\n".join(rows) + "\n")
print(f"{len(rows)} {topic} poses -> {out}")
