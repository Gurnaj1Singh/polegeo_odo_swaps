# Reproducible ROS 1 Noetic image that builds Faster-LIO from source.
# Upstream Faster-LIO is ROS 1 / catkin; the dataset bags are ROS 1 .bag files,
# so `rosbag play` works natively here even though the host runs ROS 2 Jazzy.
#
# Build:  docker build -t fasterlio:noetic fasterlio_integration/docker
# (run.sh wires the volumes and network)

FROM ros:noetic-ros-base

SHELL ["/bin/bash", "-lc"]

# --- system deps for Faster-LIO (Eigen, PCL, glog/gflags, TBB, yaml) ---
RUN apt-get update && apt-get install -y --no-install-recommends \
        git build-essential cmake \
        libeigen3-dev libpcl-dev libgoogle-glog-dev libgflags-dev \
        libtbb-dev libyaml-cpp-dev \
        ros-noetic-pcl-ros ros-noetic-tf ros-noetic-rviz \
        ros-noetic-eigen-conversions \
        ros-noetic-rosbag ros-noetic-ros-numpy \
    && rm -rf /var/lib/apt/lists/*

# --- build Faster-LIO in a catkin workspace ---
WORKDIR /root/ws/src
RUN git clone https://github.com/gaoxiang12/faster-lio.git

# Copy our sensor config + launch into the package before building.
# NOTE: build context must be the fasterlio_integration/ dir, i.e.
#   docker build -t fasterlio:noetic -f docker/Dockerfile .
COPY config/ouster_os2_128.yaml            /root/ws/src/faster-lio/config/ouster_os2_128.yaml
COPY config/mapping_ouster_os2_128.launch  /root/ws/src/faster-lio/launch/mapping_ouster_os2_128.launch

# faster-lio vendors a patched TBB; if the apt TBB clashes, the repo's
# thirdparty/ is used. Build Release for speed.
WORKDIR /root/ws
RUN source /opt/ros/noetic/setup.bash && \
    catkin_make -DCMAKE_BUILD_TYPE=Release -j"$(nproc)"

RUN echo "source /opt/ros/noetic/setup.bash"  >> /root/.bashrc && \
    echo "source /root/ws/devel/setup.bash"   >> /root/.bashrc

WORKDIR /work
CMD ["/bin/bash"]
