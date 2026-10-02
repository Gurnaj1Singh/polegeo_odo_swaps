# Minimal vendored copy of livox_ros_driver2/CustomMsg (interface-only).
# Matches upstream Livox-SDK/livox_ros_driver2 so Super-LIO compiles; unused at
# runtime here (Ouster PointCloud2 path only).
std_msgs/Header header   # ROS standard message header
uint64 timebase          # The time of first point
uint32 point_num         # Total number of pointclouds
uint8 lidar_id           # Lidar device id number
uint8[3] rsvd            # Reserved use
CustomPoint[] points     # Pointcloud data
