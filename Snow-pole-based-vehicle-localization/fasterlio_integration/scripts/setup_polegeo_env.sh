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
# YOLOv5 detection stack. ultralytics MUST be installed (geoloc_utils loads the
# pole detector via torch.hub yolov5, whose hubconf imports ultralytics). Pin it
# to the SAME version as environment.yml (8.4.171); 8.4.171 provides
# ultralytics.utils.patches.torch_load, which the pinned yolov5 commit needs, and
# loads as AutoShape with torch 2.2.0+cpu / numpy 1.26.4 (no numpy 2.x pull-in).
pip install "ultralytics==8.4.171" "numpy==1.26.4"
# rest of the stack
pip install "opencv-python==4.9.0.80" pandas matplotlib "pyproj" geopy pykrige \
    scikit-learn contextily "ouster-sdk==0.10.0" rosbags geographiclib utm \
    pyyaml tqdm requests seaborn gitpython psutil pillow
# Guard the reproducibility invariant: ultralytics must not have dragged in numpy 2.x.
python -c "import numpy; assert numpy.__version__.startswith('1.26'), 'numpy drifted to '+numpy.__version__"
echo "POLEGEO_ENV_READY"
