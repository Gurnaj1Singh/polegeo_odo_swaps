import numpy as np
import bagpy    
import cv2  # Import OpenCV for image processing
import torch  # Import PyTorch for using deep learning models
from bagpy import bagreader  # Import bagreader from bagpy to read ROS bag files
import csv  # Import csv module to write data to CSV files
import math  # Import math module for mathematical operations
import json
from io import StringIO
from ouster import client
from ouster import pcap
from contextlib import closing
from more_itertools import nth
import matplotlib.pyplot as plt
import cv2  # type: ignore
import numpy as np
import time
import cv2
import numpy as np
import open3d as o3d
import json

bag = bagreader('2024-02-28-12-59-51.bag')



# Initialize lists to store data from the ROS bag file
signal_image_data, nearir_image_data, reflec_image_data, range_image_data, gnss_data, point_cloud_data = [], [], [], [], [], []

# Read and process data from the ROS bag file
for topic, msg, t in bag.reader.read_messages():
    # Extract and store various types of data based on the topic
    if topic == '/ouster/signal_image':
        signal_image_data.append(np.frombuffer(msg.data, dtype=np.uint16).reshape((msg.height, msg.width)))
    elif topic == '/ouster/nearir_image':
        nearir_image_data.append(np.frombuffer(msg.data, dtype=np.uint16).reshape((msg.height, msg.width)))
    elif topic == '/ouster/reflec_image':
        reflec_image_data.append(np.frombuffer(msg.data, dtype=np.uint16).reshape((msg.height, msg.width)))
    elif topic == '/ouster/range_image':
        range_image_data.append(np.frombuffer(msg.data, dtype=np.uint16).reshape((msg.height, msg.width)))
    elif topic == '/gps_left_position':
        gnss_data.append((msg.latitude, msg.longitude))  # Store GNSS data as tuples of (latitude, longitude)
    elif topic == '/ouster/points':
        point_cloud_data.append((msg.latitude, msg.longitude))  


# Assuming the initialization of bag reading and metadata loading is done correctly

# Load LiDAR configuration
with open('Trip068.json', 'r') as f:
    metadata = client.SensorInfo(f.read())
    print(metadata)

# Assuming range_image_data is populated correctly from the bag file
# Initialize an empty list to accumulate all XYZ points from all frames
all_xyz_points = []

# Assuming you have correctly populated range_image_data from the ROS bag
for i in range(len(range_image_data)):
    # Create an XYZ lookup table from the sensor metadata
    xyzlut = client.XYZLut(metadata)
    #print('range_image_data[i][1, 1]',range_image_data[i][1, 1])
    # Generate XYZ data from the range image
    xyz = xyzlut(range_image_data[i])
    #print('xyz shape',xyz.shape)
    # Flatten the XYZ data and append it to the all_xyz_points list
    #all_xyz_points.extend(xyz.reshape(-1, 3))

print('xyz', xyz)
print('xyz shape',xyz.shape)
print('Starting the visualization')

# Convert the accumulated XYZ points to a NumPy array
xyz_array = np.array(xyz.reshape(-1, 3))
print('xyz_array',xyz_array)
print('xyz_array shape',xyz_array.shape)

# Create an Open3D point cloud object from the XYZ data
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(xyz_array)

# Visualize the point cloud
o3d.visualization.draw_geometries([pcd])