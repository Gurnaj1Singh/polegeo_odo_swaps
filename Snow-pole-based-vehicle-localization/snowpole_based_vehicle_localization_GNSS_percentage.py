import os # (Faster-LIO integration) env-var switches + headless backend
import cv2 # OpenCV for image processing
import numpy as np # NumPy for numerical operations
import pandas as pd # Pandas for data manipulation
from pyproj import Geod, CRS, Transformer # pyproj for coordinate transformations
import matplotlib # Matplotlib for plotting
try:
    from bagpy import bagreader # optional: replaced by rosbags reader in geoloc_utils
except Exception:
    bagreader = None
from ouster import client # Ouster client for LiDAR data
import torch.nn as nn # PyTorch for neural networks
from geopy.distance import distance # Geopy for distance calculations
from matplotlib.animation import FuncAnimation # For dynamic plotting
import json # JSON for configuration file handling
import joblib # Joblib for model loading
import contextily as ctx # Contextily for basemaps
from geoloc_utils import * # Custom geolocation utilities
from sklearn.preprocessing import MinMaxScaler # Scaler for feature normalization
from torch.utils.data import DataLoader, TensorDataset # PyTorch data utilities
import random # Random for random number generation
import time # (Faster-LIO integration) run-time instrumentation
from perf_timer import RunTimer # (Faster-LIO integration) timing helper (stdlib only)

# (Faster-LIO integration) start the wall-clock before anything heavy loads.
RT = RunTimer(label='gnss_percentage')

# Load models
RT.start('model_load') # time the YOLOv5 model load
model = load_custom_model(os.environ.get('POLE_MODEL', './model/pole_best_signal.pt'), confidence_threshold=0.6, backend=os.environ.get('MPL_BACKEND', 'TkAgg')) # Load the custom YOLOv5 model (local path; set MPL_BACKEND=Agg for headless)
RT.stop('model_load') # end model-load timing




#  Load the GNSS Data from CSV
# ODOMETRY SOURCE SWITCH (Faster-LIO integration): set INCREMENTAL_NAV_CSV to the
# Faster-LIO CSV to replace FastReg odometry; defaults to the original FastReg CSV.
import os
file_path = os.environ.get('INCREMENTAL_NAV_CSV', 'incremental_navigation_results.csv') # load incremental navigation results CSV file
print(f'[odometry] LiDAR odometry source CSV: {file_path}') # self-report FastReg vs Faster-LIO
RT.odom = 'Super-LIO' if 'superlio' in file_path.lower() else ('Faster-LIO' if 'fasterlio' in file_path.lower() else ('GLIM' if 'glim' in file_path.lower() else 'FastReg')) # (Faster-LIO/GLIM/Super-LIO integration) tag timing metrics with the odometry backend
data = pd.read_csv(file_path) # Read the CSV file into a DataFrame

# Extract the Easting and Northing columns into separate variables
fastreg_eastings = data['easting'].values  # Extract Easting values
fastreg_northings = data['northing'].values # Extract Northing values

# Define the CRS for UTM Zone 33N and WGS84
utm33n = CRS("EPSG:32633") # UTM Zone 33N
wgs84 = CRS("EPSG:4326") # WGS84 Latitude/Longitude

#  Create a transformer to convert between UTM and WGS84
transformer_to_wgs84 = Transformer.from_crs(utm33n, wgs84) # UTM to WGS84
transformer_to_utm33n = Transformer.from_crs(wgs84, utm33n) # WGS84 to UTM

#  Convert UTM coordinates to Latitude and Longitude
latitudes, longitudes = transformer_to_wgs84.transform(fastreg_eastings, fastreg_northings) # Convert to lat/lon

# Step 6: Initialize the Geod object (using WGS84 ellipsoid)
geod = Geod(ellps='WGS84') # WGS84 ellipsoid

# Step 7: Calculate heading and translation between consecutive points
headings = [] # List to store headings
translations = [] # List to store translations

for i in range(1, len(latitudes)): # Start from the second point
    if np.isnan(latitudes[i]) or np.isnan(latitudes[i-1]): # Check for NaN values
        headings.append(np.nan) # append Nan for heading
        translations.append(np.nan) # append Nan for translation
    else:
        fwd_azimuth, back_azimuth, dist = geod.inv(longitudes[i-1], latitudes[i-1], longitudes[i], latitudes[i]) # Calculate forward azimuth, back azimuth, and distance
        heading = (fwd_azimuth + 360) % 360 # Normalize heading to [0, 360)
        headings.append(heading) # Append heading to the list
        translations.append(dist) # Append distance to the list

# Add NaN for the first row since there's no previous point to compare
headings.insert(0, np.nan) # Add NaN for the first heading
translations.insert(0, np.nan) # Add NaN for the first translation

# Add the calculated headings and translations to the DataFrame
data['heading'] = headings # Add headings to DataFrame
data['translation'] = translations # Add translations to DataFrame
# Prepare scalers based on training
scaler_features = MinMaxScaler() # Scaler for features
scaler_labels = MinMaxScaler() # Scaler for labels

# Fit the scalers on available data to ensure consistency
features = data[['easting', 'northing', 'heading', 'translation']].values # Feature columns
labels = data[['easting', 'northing']].values # Label columns
scaler_features.fit(features) # Fit feature scaler
scaler_labels.fit(labels) # Fit label scaler

#  Function to Calculate new positions
def calculate_new_positions(easting, northing, heading, translation): # Calculate new UTM positions based on heading and translation
    latitude, longitude = transformer_to_wgs84.transform(easting, northing) # Convert UTM to lat/lon
    if not np.isnan(heading) and not np.isnan(translation): # Check for valid heading and translation
        lon, lat, back_azimuth = geod.fwd(longitude, latitude, heading, translation) # Calculate new lat/lon using forward geodetic calculation
    else:
        lat, lon = np.nan, np.nan # Assign NaN if heading or translation is invalid

    new_easting, new_northing = transformer_to_utm33n.transform(lat, lon) # Convert new lat/lon back to UTM
    return new_easting, new_northing # Return new UTM coordinates

# Predict new positions and integrate with LiDAR data processing
metadata, xyzlut = load_lidar_configuration('Trip068.json', client) # Load LiDAR configuration
_bag_path = os.environ.get('BAG_PATH', 'snow_pole_geo_localization_data/2024-02-28-12-59-51_no_unwanted_topics.bag') # override with BAG_PATH
RT.start('bag_load') # (Faster-LIO integration) time the ROS-bag read/decode
signal_image_data, nearir_image_data, reflec_image_data, range_image_data, vehicle_left_gnss_data, vehicle_right_gnss_data, imu_data, point_cloud_data, vehicle_heading_data, timestamps_signal, timestamps_nearir, timestamps_reflec, timestamps_range, timestamps_left_gnss, timestamps_right_gnss, timestamps_imu = process_ros_bag_data(_bag_path) # Process ROS bag data
RT.stop('bag_load') # end bag-load timing


# Set up for dynamic plotting
fig, ax = matplotlib.pyplot.subplots(figsize=(10, 8)) # Create a figure and axis for plotting
ax.set_title('GNSS Data Visualization') # Set the title of the plot
ax.set_xlabel('Easting') # Set the x-axis label
ax.set_ylabel('Northing') # Set the y-axis label

# Initialize variables for the loop
predicted_vehicle_easting = None # Initialize predicted vehicle easting
predicted_vehicle_northing = None # Initialize predicted vehicle northing

# Vehicle GNSS Calculation
vehicle_lats = [] # List to store vehicle latitudes
vehicle_lons = [] # List to store vehicle longitudes
vehicle_eastings = [] # List to store vehicle eastings
vehicle_northings = [] # List to store vehicle northings
proj_latlon = "EPSG:4326" # WGS84 Latitude/Longitude
proj_utm33 = "EPSG:32633" # UTM Zone 33N
                
# Initialize lists to store distances
distances_fastreg_to_original = [] # List for FastReg to original distances
distances_predicted_to_original = [] # List for predicted to original distances
min_distance_list = [] # List for minimum distances

# Process data and calculate vehicle's latitude and longitude
for i in range(len(range_image_data)): # Iterate over range image data
    lat = (vehicle_left_gnss_data[i+1][0] + vehicle_right_gnss_data[i+1][0]) / 2 # Average latitude from left and right GNSS
    lon = (vehicle_left_gnss_data[i+1][1] + vehicle_right_gnss_data[i+1][1]) / 2 # Average longitude from left and right GNSS
    vehicle_lats.append(lat) # Append latitude to the list
    vehicle_lons.append(lon) # Append longitude to the list
    easting, northing = transform_coordinates(proj_latlon, proj_utm33, lon, lat) # Transform to UTM coordinates
    vehicle_eastings.append(easting) # Append easting to the list
    vehicle_northings.append(northing) # Append northing to the list

gnss_offset = np.array([-0.32, 0.0, 1.24]) # GNSS sensor offset from vehicle center
lidar_offset = np.array([0.7, 0.0, 1.8]) # LiDAR sensor offset from vehicle center
gnss_to_lidar_offset = gnss_offset - lidar_offset # Calculate GNSS to LiDAR offset
# lidar_to_gnss_offset = lidar_offset - gnss_offset 
print('gnss_to_lidar_offset', gnss_to_lidar_offset) 
# print('lidar_to_gnss_offset', lidar_to_gnss_offset)

predicted_latitude, predicted_longitude = kriging_interpolation(timestamps_range, timestamps_left_gnss[1:5421], vehicle_lats, vehicle_lons) # Perform Kriging interpolation for latitude and longitude
vehicle_easting_original, vehicle_northing_original = transform_coordinates(proj_latlon, proj_utm33, predicted_longitude, predicted_latitude) # Transform predicted lat/lon to UTM coordinates
print(f'Predicted Latitude: {predicted_latitude[:10]}, Predicted Longitude: {predicted_longitude[:10]}') # Print first 10 predicted latitudes and longitudes

# Load reference GNSS data and calculate the UTM range
dataRef_northings, dataRef_eastings, min_north, max_north, min_east, max_east, dataRef_latitudes, dataRef_longitudes, min_latitude, max_latitude, min_longitude, max_longitude = load_gnss_data_and_calculate_utm_range('Groundtruth_pole_location_at_test_site_E39_Hemnekjølen.csv') # Load reference GNSS data

ground_truth_poles = [(dataRef_east, dataRef_north) for dataRef_east, dataRef_north in zip(dataRef_eastings, dataRef_northings)] # Create a list of ground truth pole coordinates
ax.scatter(dataRef_eastings, dataRef_northings, c='green', label='Poles GNSS Data', marker='x') # Plot ground truth poles

corresponding_gnss_plot = ax.scatter([], [], c='cornflowerblue', label='GNSS Data based on pole detection', marker='o') # Initialize scatter plot for GNSS data based on pole detection
corresponding_gnss_original_plot = ax.scatter([], [], c='blue', label='Vehicle GNSS Data_original', marker='o') # Initialize scatter plot for original vehicle GNSS data
detected_objects_plot, = ax.plot([], [], 'ro', label='Detected Objects GNSS Data') # Initialize plot for detected objects GNSS data
# predicted_vehicle_gnss_plot, = ax.plot([], [], 'ko', label='Predicted vehicle GNSS Data') # Initialize plot for predicted vehicle GNSS data
predicted_fastreg_gnss_plot, = ax.plot([], [], 'ko', label='Fastreg GNSS Data') # Initialize plot for FastReg GNSS data

try:
    ctx.add_basemap(ax, crs='EPSG:32633', source=ctx.providers.OpenStreetMap.Mapnik) # needs internet tiles
except Exception as _e:
    print(f'[warn] skipping basemap (offline/headless ok): {_e}')

ax.legend() # Add legend to the plot
fig.canvas.draw() # Draw the canvas
matplotlib.pyplot.pause(0.001) # Pause to update the plot


object_gnss_data = [] # To store the GNSS data of detected objects


predicted_vehicle_eastings = [] # To store predicted vehicle eastings
predicted_vehicle_northings = [] # To store predicted vehicle northings
sequence_data = []  # To store the sequence for LSTM input

# sequence_length = 1  # As used in training
gnss_percentage = float(os.environ.get('GNSS_PERCENTAGE', '0'))  # % GNSS used (0=none .. 100=all). Set env GNSS_PERCENTAGE to sweep.
random.seed(int(os.environ.get('GNSS_SEED', '0')))  # (Faster-LIO integration) reproducible GNSS-vs-odometry choice across the sweep
RT.gnss_pct = gnss_percentage # (Faster-LIO integration) tag timing metrics with the GNSS fraction
# Calculate the number of results that used GNSS versus predictive logic
gnss_count = 0 # Count of GNSS usage
predictive_count = 0 # Count of predictive logic usage

def use_gnss_randomly(gnss_percentage): 
    """
    Decide randomly whether to use GNSS data based on the input percentage.
    """
    return random.uniform(0, 100) < gnss_percentage

# Process data and calculate vehicle's latitude and longitude
RT.start('loop') # (Faster-LIO integration) time the detection + geo-localization loop
for i, (range_image, range_timestamp) in enumerate(zip(range_image_data, timestamps_range)): # Iterate over range image data
    if i == 0: 
        # Initialize the predicted vehicle position with the first GNSS data point
        predicted_vehicle_easting, predicted_vehicle_northing = transform_coordinates(
            proj_latlon, proj_utm33, predicted_longitude[i], predicted_latitude[i]
        ) # Initial position from GNSS
    elif use_gnss_randomly(gnss_percentage):  # Randomly decide to use GNSS data based on the specified percentage
        print(f'Using available GNSS data for frame: {i}') # Log GNSS usage
        lon = predicted_longitude[i] # Get predicted longitude
        lat = predicted_latitude[i] # Get predicted latitude
        predicted_vehicle_easting, predicted_vehicle_northing = transform_coordinates(proj_latlon, proj_utm33, lon, lat) # Update position from GNSS
        gnss_count += 1  # Count GNSS usage
    else:
        # Use the existing logic for prediction when GNSS data is not chosen
        predicted_vehicle_easting, predicted_vehicle_northing = calculate_new_positions(
            predicted_vehicle_easting, predicted_vehicle_northing, headings[i], translations[i]
        ) # Predict new position
        predictive_count += 1  # Count predictive logic usage
    
    gnss_percentage_actual = (gnss_count / (gnss_count + predictive_count)) * 100 if (gnss_count + predictive_count) > 0 else 0 # Calculate actual GNSS usage percentage

    print(f'GNSS Count: {gnss_count}, Predictive Count: {predictive_count}, GNSS Percentage: {gnss_percentage_actual:.2f}%') # Log counts and percentage


    predicted_vehicle_eastings.append(predicted_vehicle_easting) # Append predicted easting
    predicted_vehicle_northings.append(predicted_vehicle_northing) # Append predicted northing

    vehicle_easting, vehicle_northing = predicted_vehicle_easting, predicted_vehicle_northing # Current vehicle position
    corresponding_gnss_plot = ax.scatter(vehicle_easting, vehicle_northing, c='cornflowerblue', label='GNSS Data based on pole detection', marker='o') # Plot GNSS data based on pole detection
    corresponding_gnss_original_plot = ax.scatter(vehicle_easting_original[i], vehicle_northing_original[i], c='blue', label='Vehicle GNSS Data', marker='o') # Plot original vehicle GNSS data
    predicted_fastreg_gnss_plot = ax.plot(fastreg_eastings[i], fastreg_northings[i], 'ko', label='Fastreg GNSS Data') # Plot FastReg GNSS data

    if min_north <= vehicle_northing <= max_north and min_east <= vehicle_easting <= max_east: # Check if within bounds
        print(f'Processing frame: {i}') # Log frame processing

        if i > 0: # Skip the first frame for processing
            range_image_vis = (range_image - range_image.min()) / (range_image.max() - range_image.min()) # Normalize range image for visualization
            signal_image_vis = (signal_image_data[i] - signal_image_data[i].min()) / (signal_image_data[i].max() - signal_image_data[i].min()) # Normalize signal image for visualization
            # cv2.imshow('range_image', range_image_vis) # Display range image
            # cv2.imshow('signal_image', signal_image_vis) # Display signal image

            xyz = xyzlut(range_image) * 4 # Convert range image to XYZ coordinates
            range_lookup_table, range_vals_scaled_lookup_table = display_range_from_xyz(xyz) # Generate range image from XYZ coordinates
            # cv2.imshow('Range Image from lookup table point cloud', range_lookup_table) # Display range image from lookup table

            # rgb_image = np.stack((signal_image_data[i], nearir_image_data[i], reflec_image_data[i]), axis=-1) # Create RGB image from signal, near-IR, and reflectance data
            rgb_image = np.stack((signal_image_data[i], signal_image_data[i], signal_image_data[i]), axis=-1) # Create RGB image from signal data only

            rgb_image = ((rgb_image - rgb_image.min()) / (rgb_image.max() - rgb_image.min()) * 255).astype(np.uint8) # Normalize RGB image to 0-255 range
            _t_det = RT.tic() # (Faster-LIO integration) time this YOLO inference
            results = model(rgb_image) # Perform object detection using the loaded model
            RT.add_detection(RT.toc(_t_det)) # record per-frame detection time
            rgb_annotated_img = results.render()[0] # Get the annotated image with detections
            bboxes = results.xyxy[0] # Extract bounding boxes from detection results

            for bbox in bboxes: # Iterate over detected bounding boxes
                print(f'sequence number used for geo localization: {i}') # Log sequence number
                x_min, y_min, x_max, y_max = map(int, bbox[:4]) # Extract bounding box coordinates
                global_x, global_y, nearest_distance = extract_nearest_point_in_bounding_box_region(x_min, x_max, y_min, y_max, rgb_image, range_image_data, i) # Extract nearest point in bounding box region
                print(f'Object: Global X: {global_x}, Global Y: {global_y} for range_image_data') # Log global coordinates

                # Check if the values are None and skip the rest of the calculations if so
                if global_x is None or global_y is None or nearest_distance is None: # Check for valid nearest point
                  print('Skipping calculations as no valid nearest point found.') # Log skipping
                  continue  # Skip to the next iteration of the loop

                center_xyz = xyz[int(global_y), int(global_x), :] # Get the XYZ coordinates of the nearest point
                xyz_distance = np.linalg.norm(center_xyz) # Calculate the Euclidean distance from the LiDAR to the point

                nearest_distance_range = (range_image_data[i][global_y, global_x] / 1000) * 4 # Calculate the nearest distance in meters

                new_distance = calculate_distance(center_xyz, gnss_to_lidar_offset)  # Calculate new distance from GNSS location
                print(f'new_distance from the GNSS location w.r.to gnss: {new_distance}') # Log new distance

                distance_threshold = 5 # Define distance threshold in meters

                    # If the distance is greater than the threshold, skip to the next bounding box
                if new_distance > distance_threshold: # Check distance threshold
                   print(f"Distance {new_distance} is greater than threshold {distance_threshold}, skipping this bounding box.") # Log skipping due to distance threshold
                   continue  # Skip the rest of the loop for this bounding box and move to the next one


                easting1, northing1 = predicted_vehicle_eastings[i - 1], predicted_vehicle_northings[i - 1] # Previous vehicle position
                easting2, northing2 = predicted_vehicle_easting, predicted_vehicle_northing # Current vehicle position
                heading = calculate_vehicle_heading_from_two_utm(easting1, northing1, easting2, northing2) # Calculate vehicle heading
                end_easting, end_northing = calculate_vehicle_heading_direction_utm(easting1, northing1, heading) # Calculate heading direction
                print(f'Heading: {heading}, End Easting: {end_easting}, End Northing: {end_northing}') # Log heading and end coordinates

                azimuth_gnss, elevation_gnss = calculate_azimuth_elevation_from_gnss(center_xyz[0], center_xyz[1], center_xyz[2], gnss_to_lidar_offset) # Calculate azimuth and elevation from GNSS
                azimuth_gnss = -(azimuth_gnss) # Adjust azimuth to match vehicle heading convention
                print(f'Azimuth gnss: {azimuth_gnss}')  # Log azimuth

                # calculate the predicted easting and northing in lat and lon
                predicted_latitude_new, predicted_longitude_new = transformer_to_wgs84.transform(easting2, northing2)   # Convert predicted UTM to lat/lon
                print(f'Predicted Latitude New: {predicted_latitude_new}, Predicted Longitude New: {predicted_longitude_new}')  # Log predicted lat/lon   

                # convert back to utm with offset projection
                target_easting, target_northing, adjusted_azimuth = local_to_utm33_with_offset_proj(predicted_latitude_new, predicted_longitude_new, heading, azimuth_gnss, new_distance) # Convert local to UTM with offset projection
                print(f'Adjusted Azimuth: {adjusted_azimuth}') # Log adjusted azimuth
                back_azimuth_gnss = (180 + adjusted_azimuth) % 360 # Calculate back azimuth
                print(f'Back Azimuth GNSS: {back_azimuth_gnss}') # Log back azimuth

                object_gnss_data.append((target_easting, target_northing)) # Append target GNSS data
                target_easting_vec, target_northing_vec = zip(*object_gnss_data) # Unzip target GNSS data

                min_distance = float('inf') # Initialize minimum distance
                nearest_pole = None # Initialize nearest pole
                for pole_easting, pole_northing in ground_truth_poles: # Iterate over ground truth poles
                    dist = np.sqrt((target_easting - pole_easting) ** 2 + (target_northing - pole_northing) ** 2) # Calculate distance to pole
                    if dist < min_distance: # Check for minimum distance
                        min_distance = dist # Update minimum distance
                        nearest_pole = (pole_easting, pole_northing) # Update nearest pole
                        distance_fastreg = np.sqrt((predicted_vehicle_easting - pole_easting) ** 2 + (predicted_vehicle_northing - pole_northing) ** 2) # Distance from FastReg position to pole
                

                ground_truth_easting, ground_truth_northing = nearest_pole # Get ground truth coordinates of nearest pole
                print(f'Nearest Pole: {nearest_pole}, Distance: {min_distance}') # Log nearest pole and distance
                # append the min_distance to the list
                
                min_distance_list.append(min_distance) # Append minimum distance to the list
                # calculate the avegage of min_distance
                average_min_distance = np.mean(min_distance_list) # Calculate average minimum distance
                print(f'Average Min Distance: {average_min_distance}') # Log average minimum distance


                # calculate the predicted vehicle position based on the ground truth pole location

                ground_truth_latitude, ground_truth_longitude = transformer_to_wgs84.transform(ground_truth_easting, ground_truth_northing) # Convert ground truth UTM to lat/lon
                target_latitude, target_longitude = transformer_to_wgs84.transform(target_easting, target_northing)  # Convert target UTM to lat/lon
                geod = Geod(ellps='WGS84') # Initialize Geod object
                vehicle_predicted_lon, vehicle_predicted_lat, _ = geod.fwd(ground_truth_longitude, ground_truth_latitude, back_azimuth_gnss, new_distance) # Calculate predicted vehicle position using inverse Haversine
                print(f'ground truth easting: {ground_truth_easting}, ground truth northing: {ground_truth_northing}') # Log ground truth UTM
                print(f'back_azimuth_gnss: {back_azimuth_gnss}') # Log back azimuth
                print(f'new_distance: {new_distance}') # Log new distance
                predicted_vehicle_easting, predicted_vehicle_northing = transformer_to_utm33n.transform(vehicle_predicted_lat, vehicle_predicted_lon) # Convert predicted lat/lon back to UTM
                print(f'Predicted Vehicle Easting2: {predicted_vehicle_easting}, Predicted Vehicle Northing: {predicted_vehicle_northing}') # Log predicted vehicle UTM
                
                # (Faster-LIO integration) DE-DUPLICATED: the per-DETECTION error
                # accounting that used to live here appended to distances_*_to_original
                # once per bounding box, on TOP of the per-FRAME append after this loop —
                # inflating the corrected-error sample. The pole correction has already
                # updated predicted_vehicle_easting/northing above; the single, clean
                # per-frame error is computed after the bbox loop (matching
                # snowpole_based_vehicle_localization.py). See SPEED_AND_RELIABILITY_COMPARISON.md §4.

                ################################################################################################################
                # uncomment the following lines to enable dynamic plotting to visualize the new data updates

                # corresponding_gnss_plot = ax.scatter(vehicle_easting, vehicle_northing, c='cornflowerblue', label='Initial Vehicle GNSS Data', marker='o')
                # corresponding_gnss_original_plot = ax.scatter(vehicle_easting_original[i], vehicle_northing_original[i], c='blue', label='Vehicle GNSS Data', marker='o')
                # detected_objects_plot = ax.scatter(target_easting, target_northing, c='red', marker='o')
                # predicted_vehicle_gnss_plot = ax.scatter(predicted_vehicle_easting, predicted_vehicle_northing, c='black', marker='o')

                # fig.canvas.draw_idle()
                # matplotlib.pyplot.pause(10)
                ################################################################################################################
                # # uncomment the following lines to enable image annotation and visualization 
                # # Annotate the image with GNSS coordinates and distance
                # annotation_text_distance = f"Distance: {nearest_distance_range:.6f} m"
                # cv2.circle(rgb_annotated_img, (global_x, global_y), radius=1, color=(0, 0, 255), thickness=3)
                # cv2.putText(rgb_annotated_img, annotation_text_distance, (x_max, y_max - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 100, 255), 2)
                # annotation_text = f"GNSS: ({target_easting:.6f}, {target_northing:.6f})"
                # cv2.putText(rgb_annotated_img, annotation_text, (x_max, y_max - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 100), 2)

                # # new_size = (1024, 128)
                # new_size = (2000, 300)
                # rgb_annotated_img = cv2.resize(rgb_annotated_img, new_size)
                
                # cv2.imshow("Geo referenced poles", rgb_annotated_img)

                # cv2.imshow('range_image', range_image_vis)
                ################################################################################################################

                cv2.waitKey(1)
            
        # calculate the error between the predicted vehicle position and vehicle original position
        error_predicted = np.sqrt((vehicle_easting_original[i] - predicted_vehicle_easting) ** 2 + (vehicle_northing_original[i] - predicted_vehicle_northing) ** 2) # Calculate error between predicted and original position
        print(f'Error predicted: {error_predicted}') # Log error
        # calculate the average error
        distances_predicted_to_original.append(error_predicted) # Append error to the list

        

        average_error_predicted = np.mean(distances_predicted_to_original) # Calculate average error
        print(f'Average Error between orignal and predicted gnss: {average_error_predicted}') # Log average error

        # calculate the error between the fastreg and vehicle original position
        error_fastreg = np.sqrt((vehicle_easting_original[i] - fastreg_eastings[i]) ** 2 + (vehicle_northing_original[i] - fastreg_northings[i]) ** 2) # Calculate error between FastReg and original position
        print(f'Error Fastreg: {error_fastreg}') # Log error
        # calculate the average error
        distances_fastreg_to_original.append(error_fastreg) # Append error to the list
        average_error_fastreg = np.mean(distances_fastreg_to_original) # Calculate average fastreg error

        print(f'Average Error between orignal and fastreg gnss: {average_error_fastreg}') # Log average fastreg error

        


# (Faster-LIO integration) loop finished: stop the loop timer and emit the
# run-time + accuracy summary (JSON per run + one row appended to
# fasterlio_integration/output/timing_comparison.csv) so FastReg and Faster-LIO
# runs can be compared on speed and error together. Done before the (headless
# no-op) plt.show() calls so metrics are always written.
RT.stop('loop')
_pred = np.asarray(distances_predicted_to_original, dtype=float)
_odom = np.asarray(distances_fastreg_to_original, dtype=float)
RT.finish(
    n_frames=len(range_image_data),
    n_pole_events=None,  # GNSS-% variant doesn't build csv_data, so no discrete pole-event count
    extra={
        'err_pred_median': round(float(np.median(_pred)), 2) if _pred.size else None,
        'err_pred_mean': round(float(_pred.mean()), 2) if _pred.size else None,
        'err_pred_max': round(float(_pred.max()), 2) if _pred.size else None,
        'err_odom_median': round(float(np.median(_odom)), 2) if _odom.size else None,
        'err_odom_mean': round(float(_odom.mean()), 2) if _odom.size else None,
        'err_odom_max': round(float(_odom.max()), 2) if _odom.size else None,
        'gnss_used_count': gnss_count,
        'predictive_count': predictive_count,
    },
)

# Plotting the histogram of both errors with average and median lines
plt.figure(figsize=(8, 6))  # Set the size of the figure

# Histogram for distances predicted to original
n_predicted, bins_predicted, patches_predicted = plt.hist(distances_predicted_to_original, bins=10, color='blue', alpha=0.6, edgecolor='black', label='Predicted to Original')

# Histogram for distances fastreg to original
n_fastreg, bins_fastreg, patches_fastreg = plt.hist(distances_fastreg_to_original, bins=10, color='orange', alpha=0.6, edgecolor='black', label='FastReg to Original')

# Adding bin labels for Predicted to Original
for i in range(len(patches_predicted)):
    bin_height_pred = patches_predicted[i].get_height()
    plt.text(patches_predicted[i].get_x() + patches_predicted[i].get_width() / 2, bin_height_pred, f'{int(bin_height_pred)}', 
             ha='center', va='bottom', fontsize=10, color='blue')

# Adding bin labels for FastReg to Original
for i in range(len(patches_fastreg)):
    bin_height_fastreg = patches_fastreg[i].get_height()
    plt.text(patches_fastreg[i].get_x() + patches_fastreg[i].get_width() / 2, bin_height_fastreg, f'{int(bin_height_fastreg)}', 
             ha='center', va='bottom', fontsize=10, color='orange')

# Calculate and plot the average line for predicted distances
avg_predicted = np.mean(distances_predicted_to_original)
plt.axvline(avg_predicted, color='blue', linestyle='dashed', linewidth=2, label=f'Avg Predicted: {avg_predicted:.2f}')

# Calculate and plot the average line for fastreg distances
avg_fastreg = np.mean(distances_fastreg_to_original)
plt.axvline(avg_fastreg, color='orange', linestyle='dashed', linewidth=2, label=f'Avg FastReg: {avg_fastreg:.2f}')

# Calculate and plot the median line for predicted distances
median_predicted = np.median(distances_predicted_to_original)
plt.axvline(median_predicted, color='blue', linestyle='solid', linewidth=2, label=f'Median Predicted: {median_predicted:.2f}')

# Calculate and plot the median line for fastreg distances
median_fastreg = np.median(distances_fastreg_to_original)
plt.axvline(median_fastreg, color='orange', linestyle='solid', linewidth=2, label=f'Median FastReg: {median_fastreg:.2f}')

# Adding titles and labels
plt.title('Histogram of Errors with Averages and Medians', fontsize=16)
plt.xlabel('Error (Predicted/FastReg - Original)', fontsize=14)
plt.ylabel('Frequency', fontsize=14)

# Display the legend
plt.legend()

# Show the plot
plt.show()




# Assuming we have 'timestamps_left_gnss' available and it's a list or array of timestamp data.
# Replace X-axis with time elapsed.

# Convert timestamps to time elapsed from the first timestamp
timestamps_left_gnss = np.array(timestamps_left_gnss)
print(timestamps_left_gnss[:10])
time_elapsed = (timestamps_left_gnss - timestamps_left_gnss[0])  # stamps are already epoch seconds -> elapsed seconds (matches the 'seconds' x-axis label)
print(time_elapsed[:10])

# Truncate the time_elapsed array to match the distance arrays' length if needed
time_elapsed = time_elapsed[:len(distances_predicted_to_original)]

# Plotting the distances_predicted_to_original and distances_fastreg_to_original against time elapsed
plt.figure(figsize=(10, 6))

# Plot distances_predicted_to_original
plt.plot(time_elapsed, distances_predicted_to_original, label='Predicted to Original Distance', color='blue', marker='o')

# Plot distances_fastreg_to_original
plt.plot(time_elapsed, distances_fastreg_to_original, label='FastReg to Original Distance', color='orange', marker='x')

# Calculate mean and median for distances_predicted_to_original
mean_predicted = np.mean(distances_predicted_to_original)
median_predicted = np.median(distances_predicted_to_original)

# Calculate mean and median for distances_fastreg_to_original
mean_fastreg = np.mean(distances_fastreg_to_original)
median_fastreg = np.median(distances_fastreg_to_original)

# Plot mean and median lines for predicted distances
plt.axhline(mean_predicted, color='blue', linestyle='dashed', linewidth=2, label=f'Mean Predicted: {mean_predicted:.2f}')
plt.axhline(median_predicted, color='blue', linestyle='solid', linewidth=2, label=f'Median Predicted: {median_predicted:.2f}')

# Plot mean and median lines for fastreg distances
plt.axhline(mean_fastreg, color='orange', linestyle='dashed', linewidth=2, label=f'Mean FastReg: {mean_fastreg:.2f}')
plt.axhline(median_fastreg, color='orange', linestyle='solid', linewidth=2, label=f'Median FastReg: {median_fastreg:.2f}')

# Adding titles and labels
plt.title('Time Elapsed vs. Prediction Errors with Mean and Median Lines', fontsize=16)
plt.xlabel('Time Elapsed (seconds)', fontsize=14)
plt.ylabel('Error Distance (meters)', fontsize=14)

# Display the legend
plt.legend()

# Show the plot
plt.show()


cv2.destroyAllWindows()
matplotlib.pyplot.ioff()
matplotlib.pyplot.show(block=True)
