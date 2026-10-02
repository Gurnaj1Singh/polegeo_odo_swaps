import os
import cv2
import numpy as np
import pandas as pd
from pyproj import Geod, CRS, Transformer
import matplotlib
try:
    from bagpy import bagreader           # optional: only used by the old ROS1 reader
except Exception:
    bagreader = None
from ouster import client
import torch.nn as nn
from geopy.distance import distance
from matplotlib.animation import FuncAnimation
import json
import joblib
import contextily as ctx
from geoloc_utils import *
from sklearn.preprocessing import MinMaxScaler
from torch.utils.data import DataLoader, TensorDataset
import pdb

# Set a breakpoint at the start of main execution
# pdb.set_trace()

# Load the custom model
import time # (Faster-LIO integration) run-time instrumentation
from perf_timer import RunTimer # (Faster-LIO integration) timing helper (stdlib only)
RT = RunTimer(label='pipeline') # start the wall-clock before any heavy load
start_time = time.time() # Start time measurement
RT.start('model_load') # time the YOLOv5 model load
model = load_custom_model('./model/pole_best_signal.pt', confidence_threshold=0.6, backend=os.environ.get('MPL_BACKEND', 'TkAgg')) # set MPL_BACKEND=Agg for headless
RT.stop('model_load') # end model-load timing


# Load the GNSS Data from CSV file. The incremental_navigation_results are generated from FastReg (https://github.com/eduardohenriquearnold/fastreg ) and saved as CSV to be used in the pipeline.
# ODOMETRY SOURCE SWITCH (Faster-LIO integration): FastReg by default. Set the
# environment variable INCREMENTAL_NAV_CSV to the Faster-LIO CSV produced by
# fasterlio_integration/scripts/10_fasterlio_traj_to_csv.py to transparently
# replace FastReg with Faster-LIO for LiDAR odometry (same easting/northing schema).
import os
file_path = os.environ.get('INCREMENTAL_NAV_CSV', 'incremental_navigation_results.csv') # Replace with your actual file path
print(f'[odometry] LiDAR odometry source CSV: {file_path}') # self-report which odometry backend (FastReg vs Faster-LIO) is in use
data = pd.read_csv(file_path) # Load the CSV data into a DataFrame

# Extract the Easting and Northing columns into separate variables
fastreg_eastings = data['easting'].values # Extract Easting values
fastreg_northings = data['northing'].values # Extract Northing values

#  Define the CRS for UTM Zone 33N and WGS84
utm33n = CRS("EPSG:32633") # UTM Zone 33N
wgs84 = CRS("EPSG:4326") # WGS84 Latitude/Longitude

#  Create a transformer to convert between UTM and WGS84
transformer_to_wgs84 = Transformer.from_crs(utm33n, wgs84) # UTM → WGS84
transformer_to_utm33n = Transformer.from_crs(wgs84, utm33n) # WGS84 → UTM Zone 33N

# Convert UTM coordinates to Latitude and Longitude
latitudes, longitudes = transformer_to_wgs84.transform(fastreg_eastings, fastreg_northings) # Convert to Latitude and Longitude

#  Initialize the Geod object (using WGS84 ellipsoid)
geod = Geod(ellps='WGS84') # Initialize Geod object for geodesic calculations

# Calculate heading and translation between consecutive points
headings = [] # List to store headings
translations = [] # List to store translations

# Iterate through the coordinates to calculate headings and translations
for i in range(1, len(latitudes)): # Start
    if np.isnan(latitudes[i]) or np.isnan(latitudes[i-1]): # Check for NaN values
        headings.append(np.nan) # Append NaN if any coordinate is NaN
        translations.append(np.nan) # Append NaN for translation as well
    else:
        fwd_azimuth, back_azimuth, dist = geod.inv(longitudes[i-1], latitudes[i-1], longitudes[i], latitudes[i]) # Calculate forward azimuth, back azimuth, and distance
        heading = (fwd_azimuth + 360) % 360 # Normalize heading to [0, 360)
        headings.append(heading) # Append heading to the list
        translations.append(dist) # Append distance to the list

# Add NaN for the first row since there's no previous point to compare
headings.insert(0, np.nan) # Add NaN for the first row
translations.insert(0, np.nan) # Add NaN for the first row

# Add the calculated headings and translations to the DataFrame
data['heading'] = headings # Add headings to DataFrame
data['translation'] = translations # Add translations to DataFrame



# # Load LIDAR configuration and process ROS bag data
metadata, xyzlut = load_lidar_configuration('Trip068.json', client) # Load LIDAR configuration
# Code to Process ROS bag data to extract relevant information such as Signal, Near-IR, Reflectance, Range images, GNSS data, IMU data, and Point Cloud data. However, the bag file '2024-02-28-12-59-51_no_unwanted_topics.bag' only contains the required topics such as lidar images and GNSS data for faster processing. The data can be downloaded from the link provided in the README file.
_bag_path = os.environ.get('BAG_PATH', 'snow_pole_geo_localization_data/2024-02-28-12-59-51_no_unwanted_topics.bag') # override with BAG_PATH
RT.start('bag_load') # (Faster-LIO integration) time the ROS-bag read/decode
signal_image_data, nearir_image_data, reflec_image_data, range_image_data, vehicle_left_gnss_data, vehicle_right_gnss_data, imu_data, point_cloud_data, vehicle_heading_data, timestamps_signal, timestamps_nearir, timestamps_reflec, timestamps_range, timestamps_left_gnss, timestamps_right_gnss, timestamps_imu = process_ros_bag_data(_bag_path)
RT.stop('bag_load') # end bag-load timing

print('length of range_image_data:', len(range_image_data)) 
print('length of signal_image_data:', len(signal_image_data))
print('length of timestamps_range:', len(timestamps_range))
print('length of timestamps_left_gnss:', len(timestamps_left_gnss))
print('length of timestamps_right_gnss:', len(timestamps_right_gnss))
print('length of vehicle_left_gnss_data:', len(vehicle_left_gnss_data))
print('length of vehicle_right_gnss_data:', len(vehicle_right_gnss_data))




# Initialize variables for the loop
predicted_vehicle_easting = None # Initialize predicted vehicle easting
predicted_vehicle_northing = None # Initialize predicted vehicle northing

# Vehicle GNSS Calculation
vehicle_lats = [] # Initialize list to store vehicle latitudes
vehicle_lons = [] # Initialize list to store vehicle longitudes
vehicle_eastings = [] # Initialize list to store vehicle eastings
vehicle_northings = [] # Initialize list to store vehicle northings
proj_latlon = "EPSG:4326" # WGS84
proj_utm33 = "EPSG:32633" # UTM Zone 33N

# csv_data = [] # Initialize list to store CSV data
# Initialize lists to store distances
distances_fastreg_to_original = [] # List to store FastReg to original distances
distances_predicted_to_original = [] # List to store predicted to original distances
min_distance_list = [] # List to store minimum distances

# Process data and calculate vehicle's latitude and longitude
for i in range(len(range_image_data)): # Iterate through range image data associated with GNSS timestamps
    lat = (vehicle_left_gnss_data[i+1][0] + vehicle_right_gnss_data[i+1][0]) / 2 # Average latitude from left and right GNSS
    lon = (vehicle_left_gnss_data[i+1][1] + vehicle_right_gnss_data[i+1][1]) / 2 # Average longitude from left and right GNSS
    vehicle_lats.append(lat) # Append latitude to the list
    vehicle_lons.append(lon) # Append longitude to the list

    
    easting, northing = transform_coordinates(proj_latlon, proj_utm33, lon, lat) # Convert to UTM coordinates
    vehicle_eastings.append(easting) # Append easting to the list
    vehicle_northings.append(northing) # Append northing to the list


# Define GNSS and LIDAR offsets (in meters) based on vehicle configuration              
gnss_offset = np.array([-0.32, 0.0, 1.24]) # GNSS offset from vehicle center
lidar_offset = np.array([0.7, 0.0, 1.8]) # LIDAR offset from vehicle center
gnss_to_lidar_offset = gnss_offset - lidar_offset # Calculate GNSS to LIDAR offset
# lidar_to_gnss_offset = lidar_offset - gnss_offset 
print('gnss_to_lidar_offset', gnss_to_lidar_offset)
# print('lidar_to_gnss_offset', lidar_to_gnss_offset)

predicted_latitude, predicted_longitude = kriging_interpolation(timestamps_range, timestamps_left_gnss[1:5421], vehicle_lats, vehicle_lons) # Perform Kriging interpolation
vehicle_easting_original, vehicle_northing_original = transform_coordinates(proj_latlon, proj_utm33, predicted_longitude, predicted_latitude) # Convert predicted lat/lon to UTM coordinates
print(f'Predicted Latitude: {predicted_latitude[:1]}, Predicted Longitude: {predicted_longitude[:1]}') # Print first predicted lat/lon


# Load reference GNSS data and calculate the UTM range 
dataRef_northings, dataRef_eastings, min_north, max_north, min_east, max_east, dataRef_latitudes, dataRef_longitudes, min_latitude, max_latitude, min_longitude, max_longitude = load_gnss_data_and_calculate_utm_range('Groundtruth_pole_location_at_test_site_E39_Hemnekjølen.csv')
#  Create a list of ground truth pole coordinates
ground_truth_poles = [(dataRef_east, dataRef_north) for dataRef_east, dataRef_north in zip(dataRef_eastings, dataRef_northings)]

###################################################################################################
# # Enable the following to Plot reference GNSS data on the map
# Set up for dynamic plotting.
# Label the odometry track by its actual backend (FastReg vs Faster-LIO).
ODOM_NAME = 'Super-LIO' if 'superlio' in file_path.lower() else ('Faster-LIO' if 'fasterlio' in file_path.lower() else ('GLIM' if 'glim' in file_path.lower() else 'FastReg'))
RT.odom = ODOM_NAME # (Faster-LIO integration) tag timing metrics with the odometry backend
ODOM_LABEL = f'{ODOM_NAME} odometry'
fig, ax = matplotlib.pyplot.subplots(figsize=(10, 8)) # Create a figure and axis for plotting
ax.set_title('Snow-pole vehicle localization') # Set the title of the plot
ax.set_xlabel('Easting (m, UTM33N)') # Set the x-axis label
ax.set_ylabel('Northing (m)') # Set the y-axis label
ax.scatter(dataRef_eastings, dataRef_northings, c='green', label='Snow poles (ground truth)', marker='x')
corresponding_gnss_plot = ax.scatter([], [], c='cornflowerblue', label='Pole-corrected estimate', marker='o')
corresponding_gnss_original_plot = ax.scatter([], [], c='blue', label='GNSS reference', marker='o')
detected_objects_plot = ax.scatter([], [], c='red', label='Detected / localized poles', marker='o', zorder=5)
predicted_fastreg_gnss_plot, = ax.plot([], [], 'ko', label=ODOM_LABEL)

# Basemap: OSM's tile policy rejects contextily's default (random) User-Agent, so
# set a descriptive one that identifies the app + a contact (policy-compliant).
# OSM.Mapnik needs no API key (CartoDB now does). Set BASEMAP=0 to disable, or
# BASEMAP_SRC to another contextily provider. Guarded to degrade gracefully offline.
try:
    ctx.tile.USER_AGENT = 'snowpole-vehicle-localization/1.0 (research; contact gurnajsinghmann@gmail.com)'
    if os.environ.get('BASEMAP', '1') != '0':
        _prov = os.environ.get('BASEMAP_SRC', 'OpenStreetMap.Mapnik').split('.')
        _src = ctx.providers
        for _p in _prov:
            _src = _src[_p]
        ctx.add_basemap(ax, crs='EPSG:32633', source=_src)
except Exception as _e:
    print(f'[warn] skipping basemap (offline/policy/headless ok): {_e}')

ax.legend(loc='upper left', fontsize=9)
fig.canvas.draw()
matplotlib.pyplot.pause(0.001)
###################################################################################################
csv_data = [] # Initialize list to store CSV data
csv_data_2 = [] # Initialize second list to store CSV data
object_gnss_data = [] # Initialize list to store object GNSS data

# # # Initialize with the first known vehicle position (for the first iteration)
# predicted_vehicle_easting, predicted_vehicle_northing = vehicle_eastings[0], vehicle_northings[0]

predicted_vehicle_eastings = [] # List to store predicted vehicle eastings
predicted_vehicle_northings = [] # List to store predicted vehicle northings


# Process data and calculate vehicle's latitude and longitude
RT.start('loop') # (Faster-LIO integration) time the detection + geo-localization loop
for i, (range_image, range_timestamp) in enumerate(zip(range_image_data, timestamps_range)): # Iterate through range image data associated with GNSS timestamps
    
    print(f'Processing frame: {i}')
    if i == 0: # First iteration
        # Initialize with known coordinates for the first iteration
        predicted_vehicle_easting, predicted_vehicle_northing = transform_coordinates(proj_latlon, proj_utm33, predicted_longitude[i], predicted_latitude[i]) # Convert predicted lat/lon to UTM coordinates
        print('Initial predicted_vehicle_easting:', predicted_vehicle_easting) # Print initial predicted vehicle easting
        print('Initial predicted_vehicle_northing:', predicted_vehicle_northing) # Print initial predicted vehicle northing

    else:
        print(f'Processing frame: {i}')
        # Calculate the new predicted positions for this step
        predicted_vehicle_easting, predicted_vehicle_northing = calculate_new_positions(
            predicted_vehicle_easting, predicted_vehicle_northing, headings[i], translations[i]
        ) # Calculate new predicted positions based on previous positions, heading, and translation
        print(f'Predicted Vehicle Easting at step {i}: {predicted_vehicle_easting}')
        print(f'Predicted Vehicle Northing at step {i}: {predicted_vehicle_northing}')

    predicted_vehicle_eastings.append(predicted_vehicle_easting) # Append predicted easting to the list
    predicted_vehicle_northings.append(predicted_vehicle_northing) # Append predicted northing to the list

    vehicle_easting, vehicle_northing = predicted_vehicle_easting, predicted_vehicle_northing
    corresponding_gnss_plot = ax.scatter(vehicle_easting, vehicle_northing, c='cornflowerblue', label='GNSS Data based on pole detection', marker='o') # Plot GNSS data based on pole detection
    corresponding_gnss_original_plot = ax.scatter(vehicle_easting_original[i], vehicle_northing_original[i], c='blue', label='Vehicle GNSS Data', marker='o') # Plot original vehicle GNSS data
    predicted_fastreg_gnss_plot = ax.plot(fastreg_eastings[i], fastreg_northings[i], 'ko', label='_nolegend_') # odometry track (labelled once in setup as ODOM_LABEL)

    # Live map refresh (opt-in): run with LIVE_PLOT=1 and an interactive backend
    # (i.e. do NOT set MPL_BACKEND=Agg). Refreshes every LIVE_EVERY frames so the
    # window actually animates during the loop; no overhead when LIVE_PLOT is unset.
    if os.environ.get('LIVE_PLOT') and i % int(os.environ.get('LIVE_EVERY', '20')) == 0:
        fig.canvas.draw_idle(); plt.pause(0.001)

    if min_north <= vehicle_northing <= max_north and min_east <= vehicle_easting <= max_east: # Check if within bounds
        print(f'Processing frame: {i}')

        if i > 0: # Skip the first frame for processing
            range_image_vis = (range_image - range_image.min()) / (range_image.max() - range_image.min()) # Normalize range image for visualization
            signal_image_vis = (signal_image_data[i] - signal_image_data[i].min()) / (signal_image_data[i].max() - signal_image_data[i].min()) # Normalize signal image for visualization
            # cv2.imshow('range_image', range_image_vis) # Display range image
            # cv2.imshow('signal_image', signal_image_vis) # Display signal image

            xyz = xyzlut(range_image) * 4 # Convert range image to XYZ coordinates
            range_lookup_table, range_vals_scaled_lookup_table = display_range_from_xyz(xyz) # Generate range image from XYZ coordinates
            # cv2.imshow('Range Image from lookup table point cloud', range_lookup_table) # Display range image from lookup table

            rgb_image = np.stack((signal_image_data[i], signal_image_data[i], signal_image_data[i]), axis=-1) # Create RGB image using signal data for all channels for object detection purpose
            
            # rgb_image = np.stack((nearir_image_data[i], signal_image_data[i], reflec_image_data[i]), axis=-1) # Create RGB image using Near-IR, Signal, and Reflectance data
            # cv2.imshow('RGB Image', rgb_image) # Display RGB image
            rgb_image = ((rgb_image - rgb_image.min()) / (rgb_image.max() - rgb_image.min()) * 255).astype(np.uint8) # Normalize RGB image to 0-255 and convert to uint8
            _t_det = RT.tic() # (Faster-LIO integration) time this YOLO inference
            results = model(rgb_image) # Perform object detection using the custom model
            RT.add_detection(RT.toc(_t_det)) # record per-frame detection time
            rgb_annotated_img = results.render()[0] # Get the annotated RGB image with detections
            bboxes = results.xyxy[0] # Extract bounding boxes from detection results

            for bbox in bboxes: # Iterate through each detected bounding box
                print(f'sequence number used for geo localization: {i}') # Print the sequence number
                x_min, y_min, x_max, y_max = map(int, bbox[:4]) # Extract bounding box coordinates
                global_x, global_y, nearest_distance = extract_nearest_point_in_bounding_box_region(x_min, x_max, y_min, y_max, rgb_image, range_image_data, i) # Extract nearest point in bounding box region
                print(f'Object: Global X: {global_x}, Global Y: {global_y} for range_image_data') # Print global coordinates of the nearest point

                # Check if the values are None and skip the rest of the calculations if so 
                if global_x is None or global_y is None or nearest_distance is None: # Check for None values
                  print('Skipping calculations as no valid nearest point found.') # Print skipping message
                  continue  # Skip to the next iteration of the loop

                center_xyz = xyz[int(global_y), int(global_x), :] # Get the XYZ coordinates of the nearest point
                # xyz_distance = np.linalg.norm(center_xyz) # Calculate the Euclidean distance from the LIDAR to the point

                # nearest_distance_range = (range_image_data[i][global_y, global_x] / 1000) * 4 # Calculate the nearest distance from the range image data

                new_distance = calculate_distance(center_xyz, gnss_to_lidar_offset) # Calculate the distance from GNSS to the detected object
                print(f'new_distance from the GNSS location w.r.to gnss: {new_distance}') # Print the distance
                distance_threshold = 5 # Set a distance threshold in meters

                    # If the distance is greater than the threshold, skip to the next bounding box to avoid false detections
                if new_distance > distance_threshold:
                   print(f"Distance {new_distance} is greater than threshold {distance_threshold}, skipping this bounding box.") # Print skipping message
                   continue  # Skip the rest of the loop for this bounding box and move to the next one


                easting1, northing1 = predicted_vehicle_eastings[i - 1], predicted_vehicle_northings[i - 1] # Previous predicted vehicle position
                easting2, northing2 = predicted_vehicle_easting, predicted_vehicle_northing # Current predicted vehicle position
                heading = calculate_vehicle_heading_from_two_utm(easting1, northing1, easting2, northing2) # Calculate vehicle heading based on UTM coordinates
                end_easting, end_northing = calculate_vehicle_heading_direction_utm(easting1, northing1, heading) # Calculate the direction of vehicle heading in UTM coordinates

                # azimuth, elevation = calculate_azimuth_elevation(center_xyz[0], center_xyz[1], center_xyz[2]) # Calculate azimuth and elevation from LIDAR to the detected object
                azimuth_gnss, elevation_gnss = calculate_azimuth_elevation_from_gnss(center_xyz[0], center_xyz[1], center_xyz[2], gnss_to_lidar_offset) # Calculate azimuth and elevation from GNSS to the detected object
                azimuth_gnss = -(azimuth_gnss) # Adjust azimuth to match GNSS coordinate system
                print(f'Azimuth gnss: {azimuth_gnss}') # Print azimuth from GNSS

                # calculate the predicted easting and northing in lat and lon
                predicted_latitude_new, predicted_longitude_new = transformer_to_wgs84.transform(easting2, northing2) # Convert predicted UTM to lat/lon

                target_easting, target_northing, adjusted_azimuth = local_to_utm33_with_offset_proj(predicted_latitude_new, predicted_longitude_new, heading, azimuth_gnss, new_distance) # Convert local coordinates to UTM33N with offset projection
                print(f'Adjusted Azimuth: {adjusted_azimuth}') # Print adjusted azimuth
                back_azimuth_gnss = (180 + adjusted_azimuth) % 360 # Calculate back azimuth from GNSS
                print(f'Back Azimuth GNSS: {back_azimuth_gnss}') # Print back azimuth from GNSS

                object_gnss_data.append((target_easting, target_northing)) # Append target GNSS data to the list
                target_easting_vec, target_northing_vec = zip(*object_gnss_data) # Unzip target GNSS data into separate lists
                ax.scatter(target_easting, target_northing, c='red', marker='o', s=20, label='_nolegend_', zorder=5) # mark the detected/geo-localized snow pole in red

                min_distance = float('inf') # Initialize minimum distance to infinity
                nearest_pole = None # Initialize nearest pole variable
                for pole_easting, pole_northing in ground_truth_poles: # Iterate through each ground truth pole
                    dist = np.sqrt((target_easting - pole_easting) ** 2 + (target_northing - pole_northing) ** 2) # Calculate Euclidean distance to the pole
                    if dist < min_distance: # Check if this pole is the nearest
                        min_distance = dist # Update minimum distance
                        nearest_pole = (pole_easting, pole_northing) # Update nearest pole coordinates
                        distance_fastreg = np.sqrt((predicted_vehicle_easting - pole_easting) ** 2 + (predicted_vehicle_northing - pole_northing) ** 2) # Calculate distance from predicted vehicle GNSS to the pole
                
                ground_truth_easting, ground_truth_northing = nearest_pole # Get ground truth coordinates of the nearest pole
                print(f'Nearest Pole: {nearest_pole}, Distance: {min_distance}') # Print nearest pole and distance
                min_distance_list.append(min_distance) # Append minimum distance to the list
                average_min_distance = np.mean(min_distance_list) # Calculate average minimum distance
                print(f'Average Min Distance: {average_min_distance}') # Print average minimum distance


                ground_truth_latitude, ground_truth_longitude = transformer_to_wgs84.transform(ground_truth_easting, ground_truth_northing) # Convert ground truth UTM to lat/lon
                target_latitude, target_longitude = transformer_to_wgs84.transform(target_easting, target_northing) # Convert target UTM to lat/lon
                geod = Geod(ellps='WGS84') # Initialize Geod object for geodesic calculations
                vehicle_predicted_lon, vehicle_predicted_lat, _ = geod.fwd(ground_truth_longitude, ground_truth_latitude, back_azimuth_gnss, new_distance) # Calculate predicted vehicle lat/lon using geodesic forward calculation
                print(f'ground truth easting: {ground_truth_easting}, ground truth northing: {ground_truth_northing}') # Print ground truth UTM coordinates
                print(f'back_azimuth_gnss: {back_azimuth_gnss}') # Print back azimuth from GNSS
                print(f'new_distance: {new_distance}') # Print new distance
                predicted_vehicle_easting, predicted_vehicle_northing = transformer_to_utm33n.transform(vehicle_predicted_lat, vehicle_predicted_lon) # Convert predicted vehicle lat/lon to UTM coordinates
                print(f'Predicted Vehicle Easting2: {predicted_vehicle_easting}, Predicted Vehicle Northing: {predicted_vehicle_northing}') # Print predicted vehicle UTM coordinates


                csv_data.append([range_timestamp, timestamps_left_gnss[i], new_distance, adjusted_azimuth, vehicle_easting_original[i], vehicle_northing_original[i], vehicle_easting, vehicle_northing, target_easting, target_northing, ground_truth_easting, ground_truth_northing, min_distance, predicted_vehicle_easting, predicted_vehicle_northing, fastreg_eastings[i], fastreg_northings[i], back_azimuth_gnss, distance_fastreg])

                ###################################################################################################
                # Enable the following code to plot dynamic GNSS data on the map

                # corresponding_gnss_plot = ax.scatter(vehicle_easting, vehicle_northing, c='cornflowerblue', label='Initial Vehicle GNSS Data', marker='o')
                # # corresponding_gnss_original_plot = ax.scatter(vehicle_easting_original[i], vehicle_northing_original[i], c='blue', label='Vehicle GNSS Data', marker='o')
                # detected_objects_plot = ax.scatter(target_easting, target_northing, c='red', marker='o')
                # # predicted_vehicle_gnss_plot = ax.scatter(predicted_vehicle_easting, predicted_vehicle_northing, c='black', marker='o')

                # fig.canvas.draw_idle()
                # matplotlib.pyplot.pause(0.001)
                ###################################################################################################
                # Enable the following code to display annotated RGB image with distance and GNSS information

                # annotation_text_distance = f"Distance: {new_distance:.6f} m"
                # annotation_text_angle = f"Azimuth: {adjusted_azimuth:.2f} deg"
                # cv2.circle(rgb_annotated_img, (global_x, global_y), radius=1, color=(0, 0, 255), thickness=3)
                # cv2.putText(rgb_annotated_img, annotation_text_distance, (x_max, y_max - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 100, 255), 2)
                # cv2.putText(rgb_annotated_img, annotation_text_angle, (x_max, y_max - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                # annotation_text = f"GNSS: ({target_easting:.6f}, {target_northing:.6f})"
                # cv2.putText(rgb_annotated_img, annotation_text, (x_max, y_max - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 100), 2)

                # new_size = (1024, 128)
                # # new_size = (2000, 300)
                # rgb_annotated_img = cv2.resize(rgb_annotated_img, new_size)
                
                # cv2.imshow("Geo referenced poles", rgb_annotated_img)               

                # cv2.waitKey(1)
            # csv_data_2.append([range_timestamp, timestamps_left_gnss[i], vehicle_easting_original[i], vehicle_northing_original[i], headings[i+1], translations[i+1], vehicle_easting, vehicle_northing, predicted_vehicle_easting, predicted_vehicle_northing, fastreg_eastings[i], fastreg_northings[i]])  # Save the data to a CSV file    
        # calculate the error between the predicted vehicle position and vehicle original position
        error_predicted = np.sqrt((vehicle_easting_original[i] - predicted_vehicle_easting) ** 2 + (vehicle_northing_original[i] - predicted_vehicle_northing) ** 2) # Calculate error between predicted and original vehicle position
        print(f'Error predicted: {error_predicted}') # Print predicted error
        distances_predicted_to_original.append(error_predicted) # Append predicted error to the list

        average_error_predicted = np.mean(distances_predicted_to_original) # Calculate average predicted error
        
        print(f'Average Error between orignal and predicted gnss: {average_error_predicted}') # Print average predicted error

        # calculate the error between the fastreg and vehicle original position
        error_fastreg = np.sqrt((vehicle_easting_original[i] - fastreg_eastings[i]) ** 2 + (vehicle_northing_original[i] - fastreg_northings[i]) ** 2) # Calculate error between FastReg and original vehicle position
        print(f'Error Fastreg: {error_fastreg}') # Print FastReg error
        # calculate the average error
        distances_fastreg_to_original.append(error_fastreg) # Append FastReg error to the list
        average_error_fastreg = np.mean(distances_fastreg_to_original) # Calculate average FastReg error

        print(f'Average Error between orignal and fastreg gnss: {average_error_fastreg}') # Print average FastReg error


# (Faster-LIO integration) Save the per-detection pole-corrected results so the
# temporal-evolution visualization can overlay the proposed method + localized
# poles. Written right after the loop (before the blocking plt.show()). Columns
# match fasterlio_integration/scripts/20_temporal_evolution_visualization.py.
import os as _os
_results_csv = _os.environ.get('RESULTS_CSV', 'snowpole_results_fasterlio.csv')
with open(_results_csv, 'w', newline='') as _f:
    _w = csv.writer(_f)
    _w.writerow(['Timestamp', 'Left GNSS Timestamp', 'New Distance', 'Adjusted Azimuth GNSS', 'vehicle_easting_original', 'vehicle_northing_original', 'Vehicle Easting', 'Vehicle Northing', 'Target Easting', 'Target Northing', 'Ground Truth Easting', 'Ground Truth Northing', 'Distance to Ground Truth Pole from predicted pole', 'Predicted Vehicle Easting', 'Predicted Vehicle Northing', 'Fastreg Easting', 'Fastreg Northing', 'back_azimuth_gnss', 'distance_fastreg_to_ground_truth'])
    _w.writerows(csv_data)
print(f'[results] wrote {len(csv_data)} rows -> {_results_csv}')

# (Faster-LIO integration) stop the loop timer and emit the run-time + accuracy
# summary (JSON per run + one row appended to timing_comparison.csv) so FastReg
# and Faster-LIO runs can be compared on speed and error together.
RT.stop('loop')
_pred = np.asarray(distances_predicted_to_original, dtype=float)
_odom = np.asarray(distances_fastreg_to_original, dtype=float)
RT.finish(
    n_frames=len(range_image_data),
    n_pole_events=len(csv_data),
    extra={
        'err_pred_median': round(float(np.median(_pred)), 2) if _pred.size else None,
        'err_pred_mean': round(float(_pred.mean()), 2) if _pred.size else None,
        'err_pred_max': round(float(_pred.max()), 2) if _pred.size else None,
        'err_odom_median': round(float(np.median(_odom)), 2) if _odom.size else None,
        'err_odom_mean': round(float(_odom.mean()), 2) if _odom.size else None,
        'err_odom_max': round(float(_odom.max()), 2) if _odom.size else None,
    },
)

# Plotting the histogram of both errors with average and median lines
plt.figure(figsize=(8, 6))  # Set the size of the figure

# Histogram for distances predicted to original
n_predicted, bins_predicted, patches_predicted = plt.hist(distances_predicted_to_original, bins=10, color='blue', alpha=0.6, edgecolor='black', label='Predicted to Original') #  Plot histogram for predicted distances    

# Histogram for distances fastreg to original
n_fastreg, bins_fastreg, patches_fastreg = plt.hist(distances_fastreg_to_original, bins=10, color='orange', alpha=0.6, edgecolor='black', label=f'{ODOM_NAME} (odometry only) to Original') # Plot histogram for odometry distances

# Adding bin labels for Predicted to Original
for i in range(len(patches_predicted)): # Iterate through each patch in the predicted histogram
    bin_height_pred = patches_predicted[i].get_height() # Get the height of the bin
    plt.text(patches_predicted[i].get_x() + patches_predicted[i].get_width() / 2, bin_height_pred, f'{int(bin_height_pred)}', # Add text label for the bin
             ha='center', va='bottom', fontsize=10, color='blue') # Center align the text

# Adding bin labels for FastReg to Original
for i in range(len(patches_fastreg)): # Iterate through each patch in the FastReg histogram
    bin_height_fastreg = patches_fastreg[i].get_height() # Get the height of the bin
    plt.text(patches_fastreg[i].get_x() + patches_fastreg[i].get_width() / 2, bin_height_fastreg, f'{int(bin_height_fastreg)}', 
             ha='center', va='bottom', fontsize=10, color='orange') # Center align the text

# Calculate and plot the average line for predicted distances
avg_predicted = np.mean(distances_predicted_to_original) # Calculate average predicted distance
plt.axvline(avg_predicted, color='blue', linestyle='dashed', linewidth=2, label=f'Avg Predicted: {avg_predicted:.2f}') # Plot average line for predicted distances

# Calculate and plot the average line for fastreg distances
avg_fastreg = np.mean(distances_fastreg_to_original) # Calculate average FastReg distance
plt.axvline(avg_fastreg, color='orange', linestyle='dashed', linewidth=2, label=f'Avg {ODOM_NAME}: {avg_fastreg:.2f}') # Plot average line for odometry distances

# Calculate and plot the median line for predicted distances
median_predicted = np.median(distances_predicted_to_original) # Calculate median predicted distance
plt.axvline(median_predicted, color='blue', linestyle='solid', linewidth=2, label=f'Median Predicted: {median_predicted:.2f}') # Plot median line for predicted distances

# Calculate and plot the median line for fastreg distances
median_fastreg = np.median(distances_fastreg_to_original) # Calculate median FastReg distance
plt.axvline(median_fastreg, color='orange', linestyle='solid', linewidth=2, label=f'Median {ODOM_NAME}: {median_fastreg:.2f}') # Plot median line for odometry distances

# Adding titles and labels
plt.title('Histogram of Errors with Averages and Medians', fontsize=16) # Set the title of the plot
plt.xlabel(f'Error (Predicted / {ODOM_NAME} - Original) [m]', fontsize=14) # Set the x-axis label
plt.ylabel('Frequency', fontsize=14) # Set the y-axis label

# Display the legend
plt.legend()

# Show the plot
plt.show()




# Assuming we have 'timestamps_left_gnss' available and it's a list or array of timestamp data.
# Replace X-axis with time elapsed.

# Convert timestamps to time elapsed from the first timestamp
timestamps_left_gnss = np.array(timestamps_left_gnss) # Convert to numpy array for easier manipulation
print(timestamps_left_gnss[:10]) # Print first 10 timestamps for verification
time_elapsed = (timestamps_left_gnss - timestamps_left_gnss[0])  # stamps are already epoch seconds -> elapsed seconds (matches the 'seconds' x-axis label)
print(time_elapsed[:10]) # Print first 10 time elapsed values for verification
 
# Truncate the time_elapsed array to match the distance arrays' length if needed
time_elapsed = time_elapsed[:len(distances_predicted_to_original)] # Truncate to match lengths

# Plotting the distances_predicted_to_original and distances_fastreg_to_original against time elapsed
plt.figure(figsize=(8, 6)) # Set the size of the figure

# Plot distances_predicted_to_original
plt.plot(time_elapsed, distances_predicted_to_original, label='Predicted to Original Distance', color='blue', marker='o') # Plot predicted distances

# Plot distances_fastreg_to_original
plt.plot(time_elapsed, distances_fastreg_to_original, label=f'{ODOM_NAME} (odometry only) to Original', color='orange', marker='x') # Plot odometry distances

# Calculate mean and median for distances_predicted_to_original
mean_predicted = np.mean(distances_predicted_to_original) # Calculate mean for predicted distances
median_predicted = np.median(distances_predicted_to_original) # Calculate median for predicted distances

# Calculate mean and median for distances_fastreg_to_original
mean_fastreg = np.mean(distances_fastreg_to_original) # Calculate mean for FastReg distances
median_fastreg = np.median(distances_fastreg_to_original) # Calculate median for FastReg distances

# Plot mean and median lines for predicted distances
plt.axhline(mean_predicted, color='blue', linestyle='dashed', linewidth=2, label=f'Mean Predicted: {mean_predicted:.2f}') # Plot mean line for predicted distances
plt.axhline(median_predicted, color='blue', linestyle='solid', linewidth=2, label=f'Median Predicted: {median_predicted:.2f}') # Plot median line for predicted distances

# Plot mean and median lines for fastreg distances
plt.axhline(mean_fastreg, color='orange', linestyle='dashed', linewidth=2, label=f'Mean {ODOM_NAME}: {mean_fastreg:.2f}') # Plot mean line for odometry distances
plt.axhline(median_fastreg, color='orange', linestyle='solid', linewidth=2, label=f'Median {ODOM_NAME}: {median_fastreg:.2f}') # Plot median line for odometry distances

# Adding titles and labels
plt.title('Time Elapsed vs. Prediction Errors with Mean and Median Lines', fontsize=16) # Set the title of the plot
plt.xlabel('Time Elapsed (seconds)', fontsize=14) # Set the x-axis label
plt.ylabel('Error Distance (meters)', fontsize=14) # Set the y-axis label

# Display the legend
plt.legend()

# Show the plot
plt.show()


# Save  data to CSV file
# with open('gnss_data_with_kriging_interpolation_utm33_ground_pole_assignment_fastreg_vehicle_initial_position_esimation__pyproj_final_position_estimation_pyproj_new_17_10_2024.csv', 'w', newline='') as file:
#     writer = csv.writer(file)
#     writer.writerow(['Timestamp', 'Left GNSS Timestamp', 'New Distance', 'Adjusted Azimuth GNSS', 'vehicle_easting_original', 'vehicle_northing_original', 'Vehicle Easting', 'Vehicle Northing', 'Target Easting', 'Target Northing', 'Ground Truth Easting', 'Ground Truth Northing', 'Distance to Ground Truth Pole from predicted pole', 'Predicted Vehicle Easting', 'Predicted Vehicle Northing', 'Fastreg Easting', 'Fastreg Northing', 'back_azimuth_gnss', 'distance_fastreg_to_ground_truth'])
#     writer.writerows(csv_data)
# # save the data to a CSV 2 file
# with open('gnss_data_with_kriging_interpolation_utm33_ground_pole_assignment_fastreg_initial_position_esimation__pyproj_final_position_estimation_pyproj_new_17_10_2024.csv', 'w', newline='') as file:
#     writer = csv.writer(file)
#     writer.writerow(['LiDAR_Timestamp', 'Left GNSS Timestamp', 'vehicle_easting_original', 'vehicle_northing_original', 'Heading', 'Translation', 'Vehicle Easting', 'Vehicle Northing', 'Predicted Vehicle Easting', 'Predicted Vehicle Northing', 'Fastreg Easting', 'Fastreg Northing'])
#     writer.writerows(csv_data_2)


# Save the final localization map (works headless too) so the figure is captured
# regardless of backend. Path overridable via MAP_FIG.
try:
    _map_fig = os.environ.get('MAP_FIG', 'fasterlio_integration/output/live_map_final.png')
    fig.savefig(_map_fig, dpi=140, bbox_inches='tight')
    print(f'[map] saved final localization map -> {_map_fig}')
except Exception as _e:
    print(f'[warn] could not save map figure: {_e}')

cv2.destroyAllWindows()
matplotlib.pyplot.ioff()
matplotlib.pyplot.show(block=True)
