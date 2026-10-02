#!/usr/bin/env bash
# Build a 'polegeo' conda env capable of running the snow-pole detection pipeline
# on THIS machine (ROS2 host). We avoid ROS1/bagpy entirely by reading the bag
# with pure-python 'rosbags' (see geoloc_utils.process_ros_bag_data_rosbags).
set -e
CONDA="$HOME/miniconda3/bin/conda"
# Use conda-forge (no Anaconda-channel ToS gate) and only that channel.
"$CONDA" create -y -n polegeo -c conda-forge --override-channels python=3.9
source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda activate polegeo
python -m pip install --upgrade pip
# numpy<2 first (ouster/opencv/torch wheels are built against 1.x here)
pip install "numpy==1.26.4"
# CPU torch (matches environment.yml torch==2.2.0+cpu)
pip install torch==2.2.0 torchvision==0.17.0 --index-url https://download.pytorch.org/whl/cpu
# rest of the stack
pip install "opencv-python==4.9.0.80" pandas matplotlib "pyproj" geopy pykrige \
    scikit-learn contextily "ouster-sdk==0.10.0" rosbags geographiclib utm \
    pyyaml tqdm requests seaborn gitpython psutil pillow
echo "POLEGEO_ENV_READY"
