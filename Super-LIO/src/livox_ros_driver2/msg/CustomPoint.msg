# Minimal vendored copy of livox_ros_driver2/CustomPoint (interface-only).
# Field layout matches upstream Livox-SDK/livox_ros_driver2 exactly so that
# Super-LIO's ROSWrapper (livoxHandler / livox2pcl) compiles. We only ever feed
# Super-LIO the Ouster PointCloud2 path, so this type is never received at runtime.
uint32 offset_time   # offset time relative to the base time (ns)
float32 x            # X axis, unit:m
float32 y            # Y axis, unit:m
float32 z            # Z axis, unit:m
uint8 reflectivity   # reflectivity, 0~255
uint8 tag            # livox tag
uint8 line           # laser number in lidar
