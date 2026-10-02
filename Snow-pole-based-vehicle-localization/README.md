<!-- This section describes the GNSS–LiDAR fusion pipeline -->

# Snow-Pole-Based Vehicle Localization
---
This repository implements a **complete, end-to-end snow-pole-based vehicle localization framework** designed to enable **continuous vehicle positioning in GNSS-limited or GNSS-denied environments**, with a particular emphasis on **harsh Nordic winter conditions** where conventional localization cues are unreliable or unavailable.

The framework builds upon **georeferenced snow poles** and **LiDAR–GNSS data fusion**, treating snow poles as **machine-perceivable roadside infrastructure landmarks**. These landmarks remain visible under snow-covered conditions and can be reliably detected using **LiDAR-based perception**, making them well suited for localization when lane markings, traffic signs, and other visual cues are obscured.

By integrating **snow pole detection, snow pole geo-localization, GNSS–LiDAR fusion, data association, and incremental vehicle navigation**, the framework supports accurate **vehicle pose estimation** even during prolonged GNSS outages. This repository implements the localization framework proposed in the IEEE Transactions on Intelligent Transportation Systems paper:

> *Vehicle Localization Framework Using Georeferenced Snow Poles and LiDAR in GNSS-Limited Environments under Nordic Conditions*.
<p align="center">
  <img width="800" height="500" alt="image" src="https://github.com/user-attachments/assets/3c2ddb5d-c091-4b83-b96f-2b7c6a254106" />
</p>

---


## Overview of the Vehicle Localization Framework

The scope of this repository is to provide a **scalable vehicle localization pipeline** that leverages **georeferenced snow poles as machine-perceivable infrastructure landmarks** to ensure reliable localization in **winter-degraded and infrastructure-sparse environments**.

GNSS measurements provide a global positioning reference when available, while **high-resolution LiDAR data** enable reliable perception of snow poles and continuous motion estimation. By dynamically integrating **snow pole geo-localization** with **LiDAR-based incremental navigation (odometry)**, the framework maintains localization continuity and accuracy across varying GNSS availability conditions.

Together, the components implemented in this repository form a **field-validated vehicle localization framework** suitable for real-world operation in environments where conventional GNSS- or vision-based localization methods often fail.
<p align="center">
                              <img width="500" height="400" alt="image" src="https://github.com/user-attachments/assets/ada77b7a-e92d-47cb-acda-fa97e9791d49" />      
  </p>
  
**Pipeline for vehicle localization using snow poles in GNSS-limited environments**

---

## Framework Components

### 1. Snow Pole Detection and Landmark Extraction

To support landmark-based vehicle localization in **GNSS-degraded or GNSS-denied environments**, snow pole detection is formulated as a **2D object detection problem** using **LiDAR-derived signal images**.

A **YOLOv5s-based convolutional neural network (CNN)** is employed to detect snow poles from LiDAR signal images[3]. The model is trained on a curated dataset of **1,954 manually annotated images**, covering diverse Nordic winter scenes with varying terrain, visibility, and snow coverage. Annotation quality is ensured through refinement in **CVAT**, improving robustness under visually sparse conditions[2].

Each detected bounding box is mapped back to the corresponding **LiDAR range image** to extract spatial information, including distance and angular position relative to the vehicle. These detections serve as **landmark observations** that anchor the vehicle’s pose estimation process and provide correction points for downstream localization modules.

Extended evaluations across multiple YOLO variants and LiDAR input modalities [4] demonstrate that **multimodal LiDAR representations** significantly improve detection robustness and real-time performance, making the approach suitable for near real-time vehicle localization.



### 2. Snow Pole Geo-Localization

Detected snow poles are first localized **relative to the vehicle** by estimating their distance and bearing from LiDAR observations. These relative measurements are then fused with **GNSS positioning** to compute **absolute (map-level) geolocations**, aligned with pre-measured ground-truth snow pole coordinates at the test site.

This snow pole geo-localization module forms a critical bridge between perception and vehicle localization. By anchoring detected poles to known geographic positions, the framework establishes a **stable landmark reference system** that can be reused across vehicle passes and varying GNSS availability conditions.

Experimental validation is performed using **ROS bag recordings** containing synchronized LiDAR and GNSS data collected on Norwegian highways. Pole geolocation accuracy is evaluated by comparing estimated pole positions against measured ground truth, demonstrating high robustness under winter-degraded sensing conditions.



### 3. Incremental Vehicle Localization and GNSS–LiDAR Fusion


Building upon snow pole geo-localization, the proposed framework[1] estimates the vehicle pose through a combination of GNSS-based positioning, LiDAR-based incremental navigation, and snow pole–based pose refinement.

When GNSS signals are reliable, GNSS measurements provide absolute global pose updates. In GNSS-limited or GNSS-denied environments, the framework employs an incremental navigation strategy based on point cloud registration. In this mode, vehicle motion is estimated by aligning consecutive LiDAR frames, where each incoming scan is registered to the previous one to incrementally propagate the vehicle pose over time.

The core component of this LiDAR-based navigation strategy is the FastReg algorithm [9], a feature-based, hierarchical point cloud registration method. FastReg performs coarse-to-fine alignment by first registering downsampled point clouds and subsequently refining the estimated transformation using full-resolution scans. In our framework, FastReg is employed using its default parameter configuration, owing to its favorable trade-off between registration accuracy and computational efficiency, making it suitable for near real-time incremental navigation. Detected snow poles are subsequently leveraged to correct accumulated odometry drift by registering locally observed snow pole point clouds with their corresponding georeferenced map representations, using the same point cloud registration strategy.

The overall system is modular and registration-backend agnostic, allowing alternative point cloud registration methods—such as ICP, NDT, or learning-based registration approaches to be integrated without altering the overall localization pipeline. By dynamically alternating between GNSS-based localization and LiDAR-based navigation with point cloud registration, the proposed fusion strategy ensures continuous, drift-mitigated, and robust vehicle localization under varying GNSS conditions, particularly in challenging Nordic winter environments.



### 4. Evaluation and Analysis

The vehicle localization framework is evaluated using **real-world data collected under Nordic winter conditions**, covering mountainous, forested, and open road environments. Performance is assessed by comparing predicted vehicle trajectories against GNSS reference paths under varying levels of GNSS availability.

Quantitative results demonstrate that the framework:

- Maintains **reasonable localization accuracy under complete GNSS denial**  
- Achieves **sub-meter accuracy with partial GNSS availability**  
- **Significantly outperforms LiDAR-only odometry method** by leveraging snow pole landmarks  

These results validate the effectiveness of snow poles as **machine-sensible infrastructure** and highlight their role in enabling reliable vehicle localization in winter-degraded environments.



### 5. Relation to the Snow Pole Geo-Localization Sub-Moduley

This repository extends the **standalone Snow Pole Geo-Localization Framework[3]** by integrating it into a **complete vehicle localization pipeline[1]**. While the geo-localization sub-module focuses on accurate **snow pole positioning**, this framework leverages those **geolocated snow poles** to:

- **Anchor vehicle pose estimation**  
- **Reduce odometry drift**  
- **Enable vehicle localization** in GNSS-limited or GNSS-denied environments  

For a focused and modular implementation of **snow pole geo-localization**, please refer to the dedicated repository:

👉 https://github.com/bdps1989/snowpole_geolocalization.git

---

## Repository Structure
```text
Snow-pole-based-vehicle-localization/
│
├── snowpole_based_vehicle_localization.py
│   Main pipeline for snow-pole-based vehicle localization
│   (integrates snow pole detection, geo-localization, and vehicle pose estimation)
│
├── snowpole_based_vehicle_localization_GNSS_percentage.py
│   Variant of the main pipeline that evaluates localization performance under
│   different GNSS availability levels (e.g., % GNSS present/denied), enabling
│   robustness analysis across varying GNSS conditions
│
├── geoloc_utils.py
│   Utility functions for coordinate transformation, geodesic calculations,
│   data association, and localization evaluation
│
├── rosbag_utils/
│   Utilities for ROS bag processing and visualization
│   │
│   ├── extract_rosbag_topics.py
│   │   Extracts required LiDAR and GNSS topics from ROS bag files
│   │
│   ├── gnss_and_groundtruth_snowpole_visualization.py
│   │   Visualizes GNSS trajectories alongside ground-truth snow pole locations
│   │
│   ├── lidar_image_visualization.py
│   │   Visualization of LiDAR-derived images (signal / reflectance / NIR)
│   │
│   ├── range_image_to_pointcloud_visualization.py
│   │   Generates and visualizes point clouds from LiDAR range images
│   │
│   └── realpointcloud_vlsulaization.py
│       Visualizes raw LiDAR point clouds from ROS bag data
│
├── model/
│   Directory containing trained snow pole detection models
│   
│
├── Groundtruth_pole_location_at_test_site_E39_Hemnekjølen.csv
│   Georeferenced ground-truth snow pole locations
│   used for localization and evaluation
│
├── Trip068.json
│   Sample trip metadata and configuration file
│
├── incremental_navigation_results.csv
│   Example output file containing incremental vehicle localization estimates
│
├── environment.yml
│   Conda environment specification for reproducibility
│
├── .gitignore
│   Git ignore rules
│
├── __pycache__/
│   Python cache files (not required for execution)
│
└── README.md
│    Project description, setup instructions, and usage notes
│
└──  SYSTEM_CONFIGURATION.txt
Detailed hardware, software, and environment configuration used to conduct all experiments

```
---
## System Configuration

All experiments in this repository were conducted on a Linux-based workstation running **Ubuntu 22.04.5 LTS** with GPU acceleration.
The system includes an **Intel Xeon Platinum CPU**, an **NVIDIA L40 GPU**, and a **Conda-managed Python environment**, ensuring reproducibility of the reported results.

A detailed and complete description of the hardware, operating system, software toolchain, and execution environment is provided in the file:

SYSTEM_CONFIGURATION.txt

---
## Data Description

The dataset used in this repository consists of **real-world LiDAR and GNSS sensor data** collected along Norwegian highways, with a primary focus on the **E39 – Hemnekjølen test site in Norway**. This test site spans approximately **4.2 km** and includes **mountainous terrain, forested regions, and open landscapes**, making it particularly suitable for evaluating vehicle localization performance under **GNSS-limited conditions and harsh Nordic winter environments**.

The dataset includes the following components:

- **High-resolution LiDAR data** captured using a **128-channel Ouster OS2-128 sensor**, providing full 360° environmental coverage  
- **Continuous GNSS measurements** from dual GNSS receivers mounted on the vehicle, used for initialization, reference positioning, and quantitative evaluation  
- **Georeferenced snow pole locations**, manually measured at the infrastructure level and used as **fixed, stable landmarks** for localization  
- **Synchronized ROS bag recordings** containing LiDAR-derived images, point clouds, and GNSS data required to reproduce the snow pole geo-localization and vehicle localization experiments  

Snow poles are distributed along both sides of the roadway and are designed to remain visible under heavy snow. Consequently, they serve as **reliable machine-perceivable infrastructure landmarks** when lane markings, traffic signs, and other visual cues are partially or fully obscured.



### Publicly Available Datasets and Usage

Due to data size limitations and data-sharing constraints, **raw LiDAR point clouds and full ROS bag recordings are not included directly in this repository**. Instead, all datasets required to reproduce the experiments are hosted externally and are publicly available via the following sources:

- **SnowPole Detection Dataset (LiDAR-derived images)**  
  *Mendeley Data, Version 2* [6]  
  https://doi.org/10.17632/tt6rbx7s3h.2  

- **Extended Evaluation of SnowPole Detection Dataset (LiDAR-derived images)**  
  *Mendeley Data, Version 3* [7]  
  https://doi.org/10.17632/tt6rbx7s3h.3  

- **Snow-Pole-Based Vehicle Localization Dataset (ROS bags)**  
  *Kaggle Dataset* [8]  
  https://doi.org/10.34740/KAGGLE/DSV/14311103  

Datasets **[6]** and **[7]** are used to **train the YOLOv5-based snow pole detection model[10]**. The resulting **pretrained snow pole detection model** is then employed—together with the **ROS bag data** from dataset **[8]**—to evaluate both the **snow pole geo-localization framework** and the **end-to-end snow-pole-based vehicle localization framework**.

The Kaggle dataset **[8]** provides the following ROS bag files:

- **`2024-02-28-12-59-51.bag` (41.24 GB)**  
  Contains the **complete raw dataset**, including all recorded sensor streams captured during the data collection campaign.

- **`2024-02-28-12-59-51_no_unwanted_topics.bag` (5.71 GB)**  
  A **reduced and experiment-ready version** containing only the **LiDAR-derived images and GNSS data** required to conduct the snow pole geo-localization and vehicle localization experiments presented in this project.
 

Together, these datasets provide a **complete and reproducible foundation** for training, evaluation, and benchmarking of snow-pole-based localization methods under **GNSS-limited and winter-degraded sensing conditions**.

#### Using ROS Bag Data for Visualization and Analysis

Download **any one of the ROS bag files** associated with this project and place it inside the `Snow-pole-based-vehicle-localization/` directory (or update the file paths in the scripts accordingly). The ROS bag files are publicly available on **Kaggle**[8] under the folder `snow_pole_geo_localization_data`.


To visualize and process various sensor data—such as **raw LiDAR point clouds**, **LiDAR-derived images**, and **GNSS information**—use the utilities provided in the `rosbag_utils/` folder of this repository. These scripts support data inspection, visualization, and preprocessing for reproducing the snow pole geo-localization experiments.


---

## Reproducibility and Environment Setup

To ensure **full reproducibility** of the vehicle localization experiments, a **Conda environment specification** is provided with this repository. The framework has been **tested and validated using Python 3.9.18**, which ensures compatibility across all required dependencies, including **LiDAR data processing, geospatial computation libraries, GNSS handling, and deep learning frameworks**.

Using a controlled Conda environment guarantees consistent behavior of the localization pipeline across different systems and simplifies replication of the experimental results.


### 1. Clone the Repository

Clone the repository and navigate to the project directory:

```bash
git clone https://github.com/bdps1989/Snow-pole-based-vehicle-localization-temporary.git
cd Snow-pole-based-vehicle-localization-temporary
```
### 2. Set Up the Python Environment

A Conda environment specification is provided in environment.yml.

#### Create the Conda Environment
```bash
conda env create -f environment.yml
```
### Activate the Environment
```bash
conda activate polegeo
```
If Conda is not available, inspect the `environment.yml` file and manually install the required dependencies using **pip**.

> **Note:** Python **3.9.18** is recommended to ensure compatibility with all components of the framework.
### 3. Verify Installation

Verify that the correct Python version is active:

```bash
python --version
```
The output should indicate:
```bash
Python 3.9.18
```
---
## Results 
<img width="1369" height="729" alt="image" src="https://github.com/user-attachments/assets/8e1c0b21-cf63-4ce1-aa06-0c2f9825fc58" />

**Fig. 1: Visual quality analysis of vehicle geolocalization: (a) Original GNSS path, (b) Measured pole locations, (c) FastReg GNSS output, (d) Predicted path using the proposed method, and (e) Full route geolocalization with all components.**

<img width="1585" height="573" alt="image" src="https://github.com/user-attachments/assets/0d44abf7-1401-439c-9d25-71a43d29f836" />

**Fig. 2: Error analysis for vehicle localization: (a) Histogram of errors with averages and medians for the proposed and FastReg methods, and (b) Distance traveled vs. prediction errors for both methods.**

<img width="1400" height="900" alt="image" src="https://github.com/user-attachments/assets/8dc86ae4-d28c-4005-a5b7-9caf6b600dcb" />

**Fig. 3: Localization errors (mean and median) vs. GNSS availability percentage.**

**TABLE I: Comparison of median vehicle position errors between FastReg and the proposed pole-based method**

<img width="677" height="115" alt="image" src="https://github.com/user-attachments/assets/180a6b18-0cc9-4624-b67a-f42545328629" />

---
## Demo Video of Vehicle Geo-Localization Using Snow Pole Geo-Localization

[![Demo Video](https://img.youtube.com/vi/MNGnOWTT25Q/maxresdefault.jpg)](https://youtu.be/MNGnOWTT25Q)

***Click the image above to watch the demo on YouTube.  
For better visualization of vehicle motion and localization updates, please play the video at **2× speed**.***

---
## Related Publications and Citation

If you use this code, dataset references, or methodological ideas, please cite the corresponding publications listed below.

### Journal Articles

1. **Bavirisetti, D. P., Berget, G. E., Hanssen Kiss, G., Arnesen, P., Seter, H., & Lindseth, F. (2025). Vehicle localization framework using georeferenced snow poles and LiDAR in GNSS-limited environments under Nordic conditions. IEEE Transactions on Intelligent Transportation Systems, 26(12), 22296–22311. https://doi.org/10.1109/TITS.2025.3608465.**

2. **Bavirisetti, D. P. (2026). SnowPole-GeoLoc: An open-source GNSS–LiDAR snow pole geo-localization framework. SoftwareX, 34, 102609. https://doi.org/10.1016/j.softx.2026.102609.**
3.  **Bavirisetti, D. P., Hanssen Kiss, G., Arnesen, P., Seter, H., Tabassum, S., & Lindseth, F. (2025). SnowPole Detection: A comprehensive dataset for detection and localization using LiDAR imaging in Nordic winter conditions. Data in Brief, 59, 111403. https://doi.org/10.1016/j.dib.2025.111403.**  


### Conference Papers

5. **Bavirisetti, D. P., Kiss, G. H., & Lindseth, F. (2024, July). A pole detection and geospatial localization framework using liDAR-GNSS data fusion. In 2024 27th International Conference on Information Fusion (FUSION) (pp. 1-8). IEEE.**
5. ***Bavirisetti, D. P., Rafiq, M. I., Tabassum, S., Kiss, G. H., & Lindseth, F. (2025, September 18). Extended evaluation of SnowPole detection for machine-perceivable infrastructure for Nordic winter conditions: A comparative study of object detection models. In Proceedings of the FAIEME 2025 Conference, Stavanger, Norway. SSRN. https://doi.org/10.2139/ssrn.5386946**
6. **Bavirisetti, D. P., Berget, G. E., Tabassum, S., Kiss, G. H., Arnesen, P., Seter, H., & Lindseth, F. (2025, March 17–20). Enhancing vehicle navigation in GNSS-limited environments with georeferenced snow poles. 2025 IEEE Symposium Series on Computational Intelligence (SSCI), Trondheim, Norway. (Poster Presentation)**
   
### Datasets
7. **Bavirisetti, Durga Prasad; Kiss, Gabriel Hanssen ; Arnesen, Petter ; Seter, Hanne ; Tabassum, Shaira ; Lindseth, Frank  (2024), “SnowPole Detection: A Comprehensive Dataset for Detection and Localization Using LiDAR Imaging in Nordic Winter Conditions”, Mendeley Data, V2, [https://doi.org/10.17632/tt6rbx7s3h.2](https://doi.org/10.17632/tt6rbx7s3h.2)**
8. **Bavirisetti, Durga Prasad; Rafiq, Muhammad; Kiss, Gabriel Hanssen ; Lindseth, Frank  (2025), “Extended Evaluation of SnowPole Detection for Machine-Perceivable Infrastructure for Nordic Winter Conditions: A Comparative Study of Object Detection Models”, Mendeley Data, V3, [https://doi.org/10.17632/tt6rbx7s3h.3](https://doi.org/10.17632/tt6rbx7s3h.3)**
9. **Bavirisetti Durga Prasad , Gabriel Hanssen Kiss, Frank Lindseth, Petter Arnesen, and Hanne Seter. (2025). Data for the snowpole based vehicle localization [Data set]. Kaggle. https://doi.org/10.34740/KAGGLE/DSV/14311103**
---

## References

10. Arnold, E., Mozaffari, S., & Dianati, M. (2021). Fast and robust registration of partially overlapping point clouds. IEEE Robotics and Automation Letters, 7(2), 1502-1509.
11. Jocher, G. (2020). YOLOv5 by ultralytics (version 7.0)[computer software].


---
##  Project Context & Funding


This research was conducted as part of the project: “Machine Sensible Infrastructure under Nordic Conditions” with Project Number: 333875

---
## Contact

**Durga Prasad Bavirisetti**  

Senior Lecturer - Artificial Intelligence & Computer Vision

Department of Computer Science

University of Gävle, Sweden

For questions, collaborations, or extensions of this work, feel free to get in touch.
