# =============================================================================
# Super-LIO (CPU IESKF LIO) in ROS 2 Jazzy — portable image, no host ROS 2 needed.
#
# Build context MUST be the vendored Super-LIO source dir (repo-root Super-LIO/):
#   docker build -t superlio:jazzy -f superlio_integration/docker/Dockerfile <repo>/Super-LIO
# The matching ROS 2 Jazzy / Ubuntu 24.04 base means the vendored source (incl. the
# local reliable-QoS mod in src/ros/ROSWrapper.cpp) compiles exactly as on the host.
# =============================================================================
FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# C++ deps (Eigen/PCL/glog/gflags/TBB/pcl-conversions), colcon, rosbag2 CLI +
# storage plugins (sqlite3 to READ the input bag, mcap to WRITE /lio/odom), procps
# (pkill, for the timing-flush shutdown). CPU only — no CUDA.
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake procps \
      python3-colcon-common-extensions \
      libpcl-dev libeigen3-dev libgoogle-glog-dev libgflags-dev libtbb-dev \
      ros-jazzy-pcl-ros ros-jazzy-pcl-conversions \
      ros-jazzy-ros2bag ros-jazzy-rosbag2-transport \
      ros-jazzy-rosbag2-storage-mcap ros-jazzy-rosbag2-storage-sqlite3 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY . /ws/_ctx
# The build context is the Super-LIO workspace root (has src/, build/, install/…).
# Keep only src/ for a clean colcon build.
RUN mkdir -p /ws/src && cp -r /ws/_ctx/src/. /ws/src/ && rm -rf /ws/_ctx \
    && source /opt/ros/jazzy/setup.bash \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf /ws/build /ws/log

CMD ["bash"]
