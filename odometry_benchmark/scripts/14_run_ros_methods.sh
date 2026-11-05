#!/usr/bin/env bash
# Re-run the three reference-repo backends (previously only copied numbers) on SnowPole, with
# the same configs as the reference results, and write TUM trajectories in the Ouster sensor clock:
#   Faster-LIO  ROS 1 offline mapper, config/ouster_os2_128.yaml (point_filter_num 4, surf 1.0)
#   Super-LIO   ROS 2 node fed by `ros2 bag play --rate 3` (reference in_container_run.sh),
#               config/ouster_os2_128_base.yaml (filter_rate 2); /lio/odom recorded to a bag
#   GLIM        ROS 2 glim_rosbag (offline), reference glim_integration/config; both its odometry
#               (odom_lidar.txt) and its pose-graph-optimised trajectory (traj_lidar.txt) are kept
#
#   bash scripts/14_run_ros_methods.sh fasterlio|superlio|glim
set -euo pipefail
cd "$(dirname "$0")/.."
export MSYS_NO_PATHCONV=1
REF=third_party/polegeo_odo_swaps/Snow-pole-based-vehicle-localization
ROS2_BAG=C:/datasets/snowpole/ros2_bag
PY=${PY:-../venv/Scripts/python}
m=$1; O=outputs/$m; mkdir -p "$O"

case $m in
fasterlio)
  # offline mapper reads YAML strictly: use the reference repo's offline twin of ouster_os2_128.yaml
  # (ouster_os2_128_offline_ds.yaml: identical parameters + publish.scan_effect_pub_en, int cube length)
  docker run --rm -v "$PWD:/work" -v "$PWD/bag_full:/bag:ro" -v "$PWD/$REF/fasterlio_integration/config:/cfg:ro" \
    fasterlio:noetic bash -lc "
    source /opt/ros/noetic/setup.bash; source /root/ws/devel/setup.bash
    rosrun faster_lio run_mapping_offline --config_file=/cfg/ouster_os2_128_offline_ds.yaml \
      --bag_file=/bag/2024-02-28-12-59-51.bag --traj_log_file=/work/$O/fasterlio_tum.txt \
      --time_log_file=/work/$O/time.log" > "$O/run.log" 2>&1
  ;;
superlio)
  docker run --rm -v "$ROS2_BAG:/bag:ro" -v "$PWD/$REF/superlio_integration:/integ:ro" -v "$PWD/$O:/out" \
    superlio:jazzy bash /integ/docker/in_container_run.sh /bag /integ/config/ouster_os2_128_base.yaml 3.0 /out \
    > "$O/run.log" 2>&1
  # Jazzy writes rosbag2 metadata v9, which the venv's rosbags 0.9.23 cannot read: export with ROS 2 itself
  docker run --rm -v "$PWD:/work" superlio:jazzy bash -c "source /opt/ros/jazzy/setup.bash && \
    python3 /work/scripts/15_ros2_odom_to_tum.py /work/$O/superlio_odom /lio/odom /work/$O/superlio_tum.txt"
  ;;
glim)
  docker run --rm --gpus all --shm-size=4g -v "$ROS2_BAG:/bag:ro" -v "$PWD/$REF/glim_integration/config:/glim/config:ro" \
    -v "$PWD/$O:/tmp/dump" koide3/glim_ros2@sha256:8e00163cd989faf845134500237cfdcc0ed3b14fb3b33ce45b4952ba29b9c8cc \
    ros2 run glim_ros glim_rosbag /bag --ros-args -p config_path:=/glim/config -p auto_quit:=true \
    > "$O/run.log" 2>&1
  for f in odom_lidar traj_lidar; do [ -f "$O/$f.txt" ] && cp "$O/$f.txt" "$O/glim_${f}_tum.txt"; done
  ;;
*) echo "unknown method $m"; exit 2 ;;
esac
echo "[$m] done: $(ls $O | tr '\n' ' ')"
