
import numpy as np
import math
from geographiclib.geodesic import Geodesic
from pyproj import Proj, transform, CRS
# --- Optional ROS1 / heavy deps (Faster-LIO integration) -------------------
# On a non-ROS1 host these are unavailable. They are only needed for the raw
# ROS1 bag reader (replaced below by a pure-python `rosbags` reader) and for
# offline open3d point-cloud visualisation, neither of which the main pipeline
# path requires. Guard them so `from geoloc_utils import *` works anywhere.
try:
    from bagpy import bagreader
except Exception:
    bagreader = None
try:
    import open3d as o3d
except Exception:
    o3d = None
try:
    from cv_bridge import CvBridgeError
    import cv_bridge
except Exception:
    CvBridgeError = Exception
    cv_bridge = None
try:
    import sensor_msgs.point_cloud2 as pc2
except Exception:
    pc2 = None
# ---------------------------------------------------------------------------
import torch
import matplotlib
import pandas as pd
from pyproj import Transformer
import csv
from geopy.distance import distance as geopy_distance
from geopy.point import Point
import utm
import time
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, ConstantKernel as C
from pykrige.ok import OrdinaryKriging
from pyproj import Geod
import matplotlib.pyplot as plt
from datetime import datetime
from scipy.interpolate import interp1d
from sklearn.metrics import mean_absolute_error, mean_squared_error


def load_lidar_configuration(file_path, client):
    """
    Loads LiDAR sensor configuration from a specified JSON file and creates an XYZ lookup table based on the loaded metadata.

    Args:
    file_path (str): Path to the JSON file containing the LiDAR configuration data.
    client (object): An instance of a client class that has SensorInfo and XYZLut methods capable of processing LiDAR data.

    Returns:
    tuple: Returns a tuple containing:
           - metadata: A metadata object created from the JSON configuration data.
           - xyzlut: An XYZ lookup table object created based on the metadata.

    Description:
    The function opens and reads a JSON configuration file for a LiDAR sensor. It then uses the client's SensorInfo
    method to parse the configuration data into a metadata object. After parsing, the function uses the metadata
    to create an XYZ coordinate lookup table using the client's XYZLut method. Both the metadata and the lookup
    table are returned for further processing.
    """
    # Load the JSON data from the specified file
    with open(file_path, 'r') as f:
        json_data = f.read()
        metadata = client.SensorInfo(json_data)
        print(metadata)  # Optionally print the metadata for verification or debugging

    # Create the XYZ lookup table using the loaded metadata
    xyzlut = client.XYZLut(metadata)

    return metadata, xyzlut

def load_custom_model(model_path, confidence_threshold=0.7, backend='TkAgg'):
    """
    Loads a custom YOLOv5 model from a specified path and sets the confidence threshold for object detection.
    Also, sets the matplotlib backend as specified.

    Args:
    model_path (str): Path to the custom model file (.pt file).
    confidence_threshold (float): Confidence threshold for object detection (default is 0.4).
    backend (str): The matplotlib backend to use for plotting (default is 'TkAgg'). Other options include 'Qt5Agg',
                   'GTK3Agg', 'WXAgg', and 'agg'.

    Returns:
    model: The loaded YOLOv5 model with the confidence threshold set.
    """
    # Load the custom model using the specified path.
    # Pin the yolov5 hub ref to a specific commit so a fresh clone is reproducible:
    # an unpinned 'ultralytics/yolov5' floats to master, whose hubconf imports
    # symbols (e.g. ultralytics.utils.patches.torch_load) that must match the
    # installed ultralytics. This commit's hubconf needs ultralytics>=8.4.65 and
    # loads as AutoShape with the pinned ultralytics==8.4.171 (see environment.yml
    # / setup_polegeo_env.sh). trust_repo/skip_validation keep the download
    # unattended (no interactive trust prompt; commit SHAs fail torch's branch/tag
    # fork-check otherwise).
    model = torch.hub.load('ultralytics/yolov5:0675a31b96ad01746f8f29fe7f08d6a05200b370',
                           'custom', path=model_path,
                           trust_repo=True, skip_validation=True)
    
    # Change the matplotlib backend as per the argument
    matplotlib.use(backend)
    
    # Set the model's confidence threshold
    model.conf = confidence_threshold
    
    return model




def evaluate_interpolation_methods(
    range_timestamps,
    left_gnss_timestamps,
    right_gnss_timestamps,
    latitude,
    longitude,
    verbose=True
):
    """
    Evaluate multiple 1D interpolation methods for resampling GNSS latitude/longitude onto a target
    timestamp sequence (e.g., LiDAR/range sensor timestamps), and export per-method predictions.

    This function:
      1) Converts UNIX timestamps (range, left GNSS, right GNSS) to datetime, then to seconds relative
         to the first range timestamp (start_time).
      2) Averages left and right GNSS time bases (element-wise) to form a single GNSS time vector.
      3) Truncates range_time_sec, gnss_time_sec, latitude, and longitude to the first 5420 samples and
         flattens them into 1D NumPy arrays.
      4) For each interpolation kind in ['linear', 'nearest', 'zero', 'slinear', 'previous']:
           - Builds interpolation functions (scipy.interpolate.interp1d) for latitude and longitude
             using GNSS time as x and coordinates as y (with extrapolation enabled).
           - Predicts latitude/longitude at the range timestamps.
           - Computes MAE and MSE for latitude and longitude.
           - Saves a CSV file 'predicted_coordinates_<kind>.csv'.
      5) Aggregates error metrics into a pandas DataFrame and visualizes them.
      6) Returns linear-interpolation predictions and the error summary.

    Parameters
    ----------
    range_timestamps : array-like
    left_gnss_timestamps : array-like
    right_gnss_timestamps : array-like
    latitude : array-like
    longitude : array-like
    verbose : bool

    Returns
    -------
    predicted_latitude : np.ndarray or None
    predicted_longitude : np.ndarray or None
    error_df : pandas.DataFrame or None
    """
    try:
        # Convert the range timestamps to datetime objects
        range_datetime = [datetime.fromtimestamp(ts) for ts in range_timestamps]
        print('range_datetime', range_datetime)
        # Convert the left and right GNSS timestamps to datetime objects
        left_gnss_datetime = [datetime.fromtimestamp(ts) for ts in left_gnss_timestamps]
        print('left_gnss_datetime', left_gnss_datetime)
        right_gnss_datetime =  [datetime.fromtimestamp(ts) for ts in right_gnss_timestamps]
        print('right_gnss_datetime', right_gnss_datetime)

        # Get the start time (first timestamp in the range)
        start_time = range_datetime[0]
        print('start_time', start_time)
        
        # Calculate the time in seconds since the start time for range timestamps
        range_time_sec = [(dt - start_time).total_seconds() for dt in range_datetime]
        print('range_time_sec', range_time_sec[:10])
        
        # Calculate the time in seconds since the start time for left and right GNSS timestamps
        left_gnss_time_sec = [(dt - start_time).total_seconds() for dt in left_gnss_datetime]
        print('left_gnss_time_sec', left_gnss_time_sec[:10])
        right_gnss_time_sec = [(dt - start_time).total_seconds() for dt in right_gnss_datetime]
        print('right_gnss_time_sec', right_gnss_time_sec[:10])
        
        # Average the left and right GNSS times
        gnss_time_sec = np.mean([left_gnss_time_sec, right_gnss_time_sec], axis=0)
        print('gnss_time_sec', gnss_time_sec[:10])

        # # Average the left and right latitude and longitude
        # latitude = np.mean([left_latitude, right_latitude], axis=0)
        # longitude = np.mean([left_longitude, right_longitude], axis=0)

        # Convert lists to numpy arrays for efficient numerical operations
        range_time_sec = np.array(range_time_sec)[:5420].flatten()
        gnss_time_sec = np.array(gnss_time_sec)[:5420].flatten()
        latitude = np.array(latitude)[:5420].flatten()
        longitude = np.array(longitude)[:5420].flatten()

        # Set print options to show full precision
        np.set_printoptions(precision=17)

        if verbose:
            # Print the first 10 values of each array for verification
            print('range_time_sec', range_time_sec[:10])
            print('range_time_sec size', range_time_sec.size)
            print('gnss_time_sec', gnss_time_sec[:10])
            print('gnss_time_sec size', gnss_time_sec.size)
            print('latitude', latitude[:10])
            print('longitude', longitude[:10])

        # Define different interpolation methods to evaluate
        interpolation_kinds = ['linear', 'nearest', 'zero', 'slinear', 'previous']
        # List to store error metrics for each interpolation method
        errors = []

        for kind in interpolation_kinds:
            # Create interpolation functions for latitude using GNSS timestamps and specified method
            lat_interp_display = interp1d(gnss_time_sec, latitude, kind=kind, fill_value='extrapolate')
            # Create interpolation functions for longitude using GNSS timestamps and specified method
            lon_interp_display = interp1d(gnss_time_sec, longitude, kind=kind, fill_value='extrapolate')

            # Predict latitude values at range timestamps using the interpolation function
            predicted_latitude_display = lat_interp_display(range_time_sec)
            # Predict longitude values at range timestamps using the interpolation function
            predicted_longitude_display= lon_interp_display(range_time_sec)

            # Calculate Mean Absolute Error (MAE) for latitude predictions
            mae_lat = mean_absolute_error(latitude, predicted_latitude_display)
            # Calculate Mean Squared Error (MSE) for latitude predictions
            mse_lat = mean_squared_error(latitude, predicted_latitude_display)
            # Calculate Mean Absolute Error (MAE) for longitude predictions
            mae_lon = mean_absolute_error(longitude, predicted_longitude_display)
            # Calculate Mean Squared Error (MSE) for longitude predictions
            mse_lon = mean_squared_error(longitude, predicted_longitude_display)

            print(f"Method: {kind}, MAE Lat: {mae_lat}, MSE Lat: {mse_lat}, MAE Lon: {mae_lon}, MSE Lon: {mse_lon}")

            # Append the error metrics for the current interpolation method to the errors list
            errors.append({
                'method': kind,
                'mae_lat': mae_lat,
                'mse_lat': mse_lat,
                'mae_lon': mae_lon,
                'mse_lon': mse_lon
            })

            if verbose:
                # Print error metrics for the current interpolation method
                print(f"Method: {kind}, MAE Lat: {mae_lat}, MSE Lat: {mse_lat}, MAE Lon: {mae_lon}, MSE Lon: {mse_lon}")

            # Combine predicted and actual values into a DataFrame for further analysis and saving
            predictions = pd.DataFrame({
                'range time (seconds since start)': range_time_sec,
                'predicted_latitude': predicted_latitude_display,
                'predicted_longitude': predicted_longitude_display,
                'gnss time (seconds since start)': gnss_time_sec,
                'latitude': latitude,
                'longitude': longitude
            })

            # print the first 10 rows of the predictions DataFrame for verification
            # improve the precision of the DataFrame
            pd.set_option('display.float_format', lambda x: '%.17f' % x)
            print('predictions.head(10)', predictions.head(10))
            

            # Save the predictions to a new CSV file for each interpolation method
            output_file = f'predicted_coordinates_{kind}.csv'
            predictions.to_csv(output_file, index=False)

            if verbose:
                # Print confirmation that the predictions have been saved
                print(f"Predicted coordinates using {kind} interpolation saved to '{output_file}'")

        # # Convert the errors list to a DataFrame for easier plotting
        error_df = pd.DataFrame(errors)

        # Create interpolation functions for latitude using GNSS timestamps and specified method
        lat_interp = interp1d(gnss_time_sec, latitude, kind='linear', fill_value='extrapolate')
        # Create interpolation functions for longitude using GNSS timestamps and specified method
        lon_interp = interp1d(gnss_time_sec, longitude, kind='linear', fill_value='extrapolate')

        print('gnss time sec', gnss_time_sec)

        # Predict latitude values at range timestamps using the interpolation function
        predicted_latitude = lat_interp(range_time_sec)
        print('predicted_latitude', predicted_latitude)
        # print length of predicted latitude
        # print('predicted_latitude size', predicted_latitude.size)
        # # Predict longitude values at range timestamps using the interpolation function
        predicted_longitude = lon_interp(range_time_sec)
        print('predicted_longitude', predicted_longitude)
        # # print length of predicted longitude
        # print('predicted_longitude size', predicted_longitude.size)


        # Create a figure for plotting error metrics
        plt.figure(figsize=(14, 12))

        # Plot Mean Absolute Error for Latitude for each interpolation method
        plt.subplot(2, 2, 1)
        plt.bar(error_df['method'], error_df['mae_lat'], color='b', alpha=0.7)
        plt.title('MAE for Latitude')
        plt.xlabel('Interpolation Method')
        plt.ylabel('Mean Absolute Error')

        # Plot Mean Squared Error for Latitude for each interpolation method
        plt.subplot(2, 2, 2)
        plt.bar(error_df['method'], error_df['mse_lat'], color='r', alpha=0.7)
        plt.title('MSE for Latitude')
        plt.xlabel('Interpolation Method')
        plt.ylabel('Mean Squared Error')

        # Plot Mean Absolute Error for Longitude for each interpolation method
        plt.subplot(2, 2, 3)
        plt.bar(error_df['method'], error_df['mae_lon'], color='g', alpha=0.7)
        plt.title('MAE for Longitude')
        plt.xlabel('Interpolation Method')
        plt.ylabel('Mean Absolute Error')

        # Plot Mean Squared Error for Longitude for each interpolation method
        plt.subplot(2, 2, 4)
        plt.bar(error_df['method'], error_df['mse_lon'], color='m', alpha=0.7)
        plt.title('MSE for Longitude')
        plt.xlabel('Interpolation Method')
        plt.ylabel('Mean Squared Error')

        # Adjust the layout to prevent overlap
        plt.tight_layout()
        # Show the plots
        plt.show()

        # Return the interpolation functions for latitude and longitude
        return predicted_latitude, predicted_longitude, error_df
        # return lat_interp, lon_interp, errors

    except Exception as e:
        # Print any exception that occurs during the process
        print(f"An error occurred: {e}")
        return None, None, None

 # Define projections
proj_latlon = Proj(init='epsg:4326')  # WGS84
proj_utm33 = Proj(init='epsg:32633')  # UTM Zone 33N

# Convert lat/lon to UTM
def latlon_to_utm(latitudes, longitudes):
    """
    Convert geographic coordinates (latitude, longitude in WGS84) to projected UTM coordinates
    (Easting, Northing) in UTM Zone 33 (EPSG:32633 or equivalent).

    Parameters
    ----------
    latitudes : array-like
        Latitudes in degrees (WGS84).
    longitudes : array-like
        Longitudes in degrees (WGS84).

    Returns
    -------
    eastings : array-like
        UTM Easting coordinates (meters) in UTM Zone 33.
    northings : array-like
        UTM Northing coordinates (meters) in UTM Zone 33.

    Notes
    -----
    - Uses `transform(proj_latlon, proj_utm33, longitudes, latitudes)` where proj_latlon is
      a lat/lon CRS and proj_utm33 is UTM Zone 33 CRS.
    - Ensure proj_latlon and proj_utm33 are defined consistently (e.g., WGS84 → EPSG:32633).
    """
    eastings, northings = transform(proj_latlon, proj_utm33, longitudes, latitudes)
    return eastings, northings

# Convert UTM to lat/lon
def utm_to_latlon(eastings, northings):
    """
    Convert projected UTM coordinates (Easting, Northing in UTM Zone 33) back to geographic
    coordinates (latitude, longitude in WGS84).

    Parameters
    ----------
    eastings : array-like
        UTM Easting coordinates (meters) in UTM Zone 33.
    northings : array-like
        UTM Northing coordinates (meters) in UTM Zone 33.

    Returns
    -------
    latitudes : array-like
        Latitudes in degrees (WGS84).
    longitudes : array-like
        Longitudes in degrees (WGS84).

    Notes
    -----
    - Uses `transform(proj_utm33, proj_latlon, eastings, northings)`.
    - Output order is returned as (latitudes, longitudes) for convenience.
    """
    longitudes, latitudes = transform(proj_utm33, proj_latlon, eastings, northings)
    return latitudes, longitudes

# Kriging interpolation function
def kriging_interpolation_utm33(lidar_timestamps, gnss_timestamps, gnss_latitudes, gnss_longitudes):
    """
    Interpolate GNSS positions to LiDAR timestamps using Ordinary Kriging in the UTM Zone 33
    coordinate system and return interpolated UTM Easting and Northing coordinates.

    The function first converts GNSS latitude/longitude coordinates (WGS84) into UTM Zone 33
    Easting and Northing coordinates (meters). Ordinary Kriging is then applied independently
    to the Easting and Northing components as functions of time, using GNSS timestamps as the
    interpolation domain. The resulting interpolated UTM coordinates are evaluated at the
    LiDAR timestamps.

    Parameters
    ----------
    lidar_timestamps : array-like
        Target timestamps at which UTM positions are required (e.g., LiDAR or range sensor
        timestamps). Must be numeric and in the same time unit as `gnss_timestamps`
        (typically UNIX time in seconds).
    gnss_timestamps : array-like
        GNSS timestamps corresponding to the provided latitude and longitude measurements.
    gnss_latitudes : array-like
        GNSS latitude samples in degrees (WGS84), aligned with `gnss_timestamps`.
    gnss_longitudes : array-like
        GNSS longitude samples in degrees (WGS84), aligned with `gnss_timestamps`.

    Returns
    -------
    predicted_eastings : np.ndarray
        Interpolated UTM Easting coordinates (meters) at `lidar_timestamps`.
    predicted_northings : np.ndarray
        Interpolated UTM Northing coordinates (meters) at `lidar_timestamps`.

    Notes
    -----
    - A linear variogram model is used for Ordinary Kriging.
    - Kriging is performed over time by providing (timestamp, 0) as the 2D input coordinates,
      where the second dimension is a dummy zero vector required by the kriging API.
    - Working in UTM space ensures interpolation is performed in a metric coordinate system,
      which is more physically meaningful than interpolating directly in latitude/longitude.
    - UTM Zone 33 is appropriate only if the trajectory lies within or close to that zone;
      otherwise, the correct UTM zone or a suitable local projected CRS should be used.
    """
    # Convert GNSS latitudes and longitudes to UTM
    gnss_eastings, gnss_northings = latlon_to_utm(gnss_latitudes, gnss_longitudes)
    
    # Perform Kriging on Eastings
    OK_east = OrdinaryKriging(
        np.array(gnss_timestamps), np.zeros_like(gnss_timestamps), np.array(gnss_eastings),
        variogram_model='linear', verbose=False, enable_plotting=False
    )
    predicted_eastings, _ = OK_east.execute('points', np.array(lidar_timestamps), np.zeros_like(lidar_timestamps))

    # Perform Kriging on Northings
    OK_north = OrdinaryKriging(
        np.array(gnss_timestamps), np.zeros_like(gnss_timestamps), np.array(gnss_northings),
        variogram_model='linear', verbose=False, enable_plotting=False
    )
    predicted_northings, _ = OK_north.execute('points', np.array(lidar_timestamps), np.zeros_like(lidar_timestamps))

    
    return predicted_eastings, predicted_northings
   
def kriging_interpolation(lidar_timestamps, gnss_timestamps, gnss_latitudes, gnss_longitudes):
    """
    Interpolate GNSS latitude and longitude directly (in degrees) to LiDAR timestamps using Ordinary Kriging.

    This method applies Ordinary Kriging separately to latitude and longitude as functions of timestamp,
    treating timestamps as the interpolation axis. It does not convert coordinates to a metric projection.

    Parameters
    ----------
    lidar_timestamps : array-like
        Target timestamps (e.g., LiDAR frames) where lat/lon estimates are needed.
    gnss_timestamps : array-like
        GNSS timestamps aligned with the input latitude/longitude samples.
    gnss_latitudes : array-like
        GNSS latitude samples in degrees.
    gnss_longitudes : array-like
        GNSS longitude samples in degrees.

    Returns
    -------
    predicted_latitudes : np.ndarray
        Interpolated latitude values at lidar_timestamps.
    predicted_longitudes : np.ndarray
        Interpolated longitude values at lidar_timestamps.

    Notes
    -----
    - Uses a 'linear' variogram model in Ordinary Kriging.
    - Because latitude/longitude are angular units, direct interpolation in degrees may be less physically
      meaningful than interpolating in a local metric projection (e.g., UTM), especially over larger areas.
    - Kriging is applied over time by using (timestamp, 0) as the 2D kriging input; the second dimension
      is a dummy zero vector required by the kriging API.
    """
    OK_lat = OrdinaryKriging(
        np.array(gnss_timestamps), np.zeros_like(gnss_timestamps), np.array(gnss_latitudes),
        variogram_model='linear', verbose=False, enable_plotting=False
    )
    predicted_latitudes, _ = OK_lat.execute('points', np.array(lidar_timestamps), np.zeros_like(lidar_timestamps))
    OK_lon = OrdinaryKriging(
        np.array(gnss_timestamps), np.zeros_like(gnss_timestamps), np.array(gnss_longitudes),
        variogram_model='linear', verbose=False, enable_plotting=False
    )
    predicted_longitudes, _ = OK_lon.execute('points', np.array(lidar_timestamps), np.zeros_like(lidar_timestamps))
    return predicted_latitudes, predicted_longitudes
    
def gpr_interpolation(lidar_timestamps, gnss_timestamps, gnss_latitudes, gnss_longitudes):
    """
    Interpolate GNSS latitude/longitude to LiDAR timestamps using Gaussian Process Regression (GPR).

    This method fits two independent Gaussian Process models:
      - latitude  = f(time)
      - longitude = f(time)
    using GNSS timestamps as the input feature. Predictions are then produced at LiDAR timestamps.
    An RBF kernel with a scaling constant is used, and the optimizer is restarted multiple times to
    improve kernel hyperparameter fitting.

    Parameters
    ----------
    lidar_timestamps : array-like
        Target timestamps where position estimates are required (e.g., LiDAR frames).
    gnss_timestamps : array-like
        GNSS timestamps aligned with GNSS latitude/longitude samples.
    gnss_latitudes : array-like
        GNSS latitude samples in degrees.
    gnss_longitudes : array-like
        GNSS longitude samples in degrees.

    Returns
    -------
    predicted_latitudes : np.ndarray
        GPR-predicted latitudes at lidar_timestamps.
    predicted_longitudes : np.ndarray
        GPR-predicted longitudes at lidar_timestamps.

    Notes
    -----
    - Kernel: ConstantKernel * RBF, with relatively wide bounds for hyperparameter search.
    - alpha=1e-2 adds noise regularization (helps with noisy GNSS).
    - return_std=True is used internally; only the mean predictions are returned. If needed, you can
      return the uncertainty (standard deviation) as well for downstream filtering/quality checks.
    """
    # Define a kernel with increased bounds
    kernel = C(1.0, (1e-4, 1e2)) * RBF(1, (1e-4, 1e2))

    gp_lat = GaussianProcessRegressor(kernel=kernel, n_restarts_optimizer=10, alpha=1e-2)
    gp_lat.fit(np.array(gnss_timestamps).reshape(-1, 1), np.array(gnss_latitudes))
    predicted_latitudes, _ = gp_lat.predict(np.array(lidar_timestamps).reshape(-1, 1), return_std=True)

    gp_lon = GaussianProcessRegressor(kernel=kernel, n_restarts_optimizer=10, alpha=1e-2)
    gp_lon.fit(np.array(gnss_timestamps).reshape(-1, 1), np.array(gnss_longitudes))
    predicted_longitudes, _ = gp_lon.predict(np.array(lidar_timestamps).reshape(-1, 1), return_std=True)

    return predicted_latitudes, predicted_longitudes


def load_and_transform_gnss_data_and_calculate_latlon_range(file_path):
    """
    Reads GNSS data from a CSV file, transforms it from UTM33 to WGS84 coordinates,
    and extracts the minimum and maximum longitude and latitude.

    Args:
    file_path (str): Path to the CSV file containing GNSS coordinates.

    Returns:
    tuple: Returns a tuple containing:
           - dataRef_gnss_lon: Array of transformed longitudes.
           - dataRef_gnss_lat: Array of transformed latitudes.
           - min_lon: Minimum longitude.
           - min_lat: Minimum latitude.
           - max_lon: Maximum longitude.
           - max_lat: Maximum latitude.

    Description:
    This function loads GNSS data from a specified CSV file, transforms the coordinates from UTM33 (EPSG:32633) to WGS84 (EPSG:4326),
    and calculates the geographic extents by finding the minimum and maximum values for longitude and latitude.
    The transformation ensures that the data can be used for geographic applications that require standard latitude and longitude coordinates.
    """
    # Read the reference GNSS data from the CSV file
    dataRefPd = pd.read_csv(file_path)
    dataRef = dataRefPd.loc[:, ["UTM33-Nord", "UTM33-Øst"]].to_numpy()

    # Initialize the transformer from UTM33 to WGS84
    transformer = Transformer.from_crs("epsg:32633", "epsg:4326", always_xy=True)
    # Perform the transformation
    dataRef_gnss_lon, dataRef_gnss_lat = transformer.transform(dataRef[:, 1], dataRef[:, 0])

    # Calculate the minimum and maximum longitude and latitude
    min_lon, max_lon = min(dataRef_gnss_lon), max(dataRef_gnss_lon)
    min_lat, max_lat = min(dataRef_gnss_lat), max(dataRef_gnss_lat)

    return (dataRef_gnss_lon, dataRef_gnss_lat, min_lon, min_lat, max_lon, max_lat)

def load_gnss_data_and_calculate_utm_range(file_path):
    """
    Reads GNSS data from a CSV file and extracts the minimum and maximum easting and northing coordinates.
    
    Args:
    file_path (str): Path to the CSV file containing GNSS coordinates in UTM format.

    Returns:
    tuple: Returns a tuple containing:
           - dataRef_north: Array of UTM northing coordinates.
           - dataRef_east: Array of UTM easting coordinates.
           - min_north: Minimum northing.
           - max_north: Maximum northing.
           - min_east: Minimum easting.
           - max_east: Maximum easting.
           - dataRef_latitude: Array of latitude coordinates.
           - dataRef_longitude: Array of longitude coordinates.
           - min_latitude: Minimum latitude.
           - max_latitude: Maximum latitude.
           - min_longitude: Minimum longitude.
           - max_longitude: Maximum longitude.

    Description:
    This function loads GNSS data from a specified CSV file containing coordinates in UTM33 (EPSG:32633).
    It calculates the geographic extents by finding the minimum and maximum values for northing and easting.
    The data is also converted to latitude and longitude coordinates using the WGS84 datum.
    """
    # Read the reference GNSS data from the CSV file
    dataRefPd = pd.read_csv(file_path)
    dataRef_north = dataRefPd["UTM33-Nord"].values
    dataRef_east = dataRefPd["UTM33-Øst"].values

    # Calculate the minimum and maximum northing and easting
    min_north, max_north = min(dataRef_north), max(dataRef_north)
    min_east, max_east = min(dataRef_east), max(dataRef_east)

    # Define the CRS for UTM Zone 33N and WGS84
    utm33n = CRS("EPSG:32633")
    wgs84 = CRS("EPSG:4326")

    # Create a transformer to convert between UTM and WGS84
    transformer = Transformer.from_crs(utm33n, wgs84)

    # Convert UTM coordinates to latitude and longitude
    dataRef_longitude, dataRef_latitude = transformer.transform(dataRef_east, dataRef_north)

    # Calculate the minimum and maximum latitude and longitude
    min_latitude, max_latitude = min(dataRef_latitude), max(dataRef_latitude)
    min_longitude, max_longitude = min(dataRef_longitude), max(dataRef_longitude)

    return (dataRef_north, dataRef_east, min_north, max_north, min_east, max_east,
            dataRef_latitude, dataRef_longitude, min_latitude, max_latitude, min_longitude, max_longitude)


def transform_coordinates(proj_latlon, proj_utm33, lon, lat):
    """
    Transform geographic coordinates (latitude, longitude) to UTM Zone 33N coordinates (easting, northing).

    Parameters:
    proj_latlon (str): The projection string for latitude/longitude coordinates.
    proj_utm33 (str): The projection string for UTM Zone 33N coordinates.
    lon (float): The longitude in degrees.
    lat (float): The latitude in degrees.

    Returns:
    tuple: Easting and northing in UTM Zone 33N.
    """
    # Initialize a transformer from WGS84 to UTM Zone 33N
    
    transformer = Transformer.from_crs(proj_latlon, proj_utm33, always_xy=True)
    # Perform the transformation
    easting, northing = transformer.transform(lon, lat)

    return easting, northing
# Define CRS for UTM Zone 33N and WGS84
utm33n = CRS("EPSG:32633")
wgs84 = CRS("EPSG:4326")
geod = Geod(ellps='WGS84')
transformer_to_wgs84 = Transformer.from_crs(utm33n, wgs84) # UTM → WGS84
transformer_to_utm33n = Transformer.from_crs(wgs84, utm33n)# WGS84 → UTM Zone 33N
def calculate_new_positions(easting, northing, heading, translation):
    """
    Compute a new UTM position by propagating an initial position using heading
    and translation distance.

    This function performs the following steps:
    1. Converts the input UTM coordinates (easting, northing) to geographic
       coordinates (latitude, longitude) in WGS84.
    2. Uses forward geodesic computation to move the position by a given
       translation distance along the specified heading (azimuth).
    3. Converts the resulting geographic coordinates back to UTM Zone 33N.

    Parameters
    ----------
    easting : float
        Initial UTM easting coordinate (meters).
    northing : float
        Initial UTM northing coordinate (meters).
    heading : float
        Heading (azimuth) in degrees, measured clockwise from true north.
    translation : float
        Translation distance in meters along the heading direction.

    Returns
    -------
    new_easting : float
        Updated UTM easting coordinate (meters).
    new_northing : float
        Updated UTM northing coordinate (meters).

    Notes
    -----
    - If `heading` or `translation` is NaN, the function returns NaN for both
      easting and northing.
    - The function assumes:
        * `transformer_to_wgs84` converts UTM → WGS84 (lat/lon).
        * `transformer_to_utm33n` converts WGS84 → UTM Zone 33N.
        * `geod` is a pyproj.Geod object configured for the WGS84 ellipsoid.
    """
    latitude, longitude = transformer_to_wgs84.transform(easting, northing)

    if not np.isnan(heading) and not np.isnan(translation):
        lon, lat, back_azimuth = geod.fwd(
            longitude, latitude, heading, translation
        )
    else:
        lat, lon = np.nan, np.nan

    new_easting, new_northing = transformer_to_utm33n.transform(lat, lon)
    return new_easting, new_northing

def save_to_csv(filename, data, headers):
    """
    Saves data to a CSV file.

    Args:
    filename (str): The name of the CSV file.
    data (list): The data to be saved.
    headers (list): The headers for the CSV file.
    """
    with open(filename, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(headers)
        writer.writerows(data)




def process_ros_bag_data(bag_file_path):
    """
    Processes data from a ROS bag file, extracting various types of sensor data including image, GNSS, and LiDAR data.

    Args:
    bag_file_path (str): The path to the ROS bag file.

    Returns:
    tuple of lists: Each containing data for different sensors:
        - Signal images from the LIDAR sensor.
        - Near-infrared images from the LIDAR sensor.
        - Reflectivity images from the LIDAR sensor.
        - Range images from the LIDAR sensor.
        - GNSS data from the left sensor on the vehicle.
        - GNSS data from the right sensor on the vehicle.
        - IMU data including orientation, angular velocity, and linear acceleration.
        - Point cloud data from the LIDAR sensor.
        - Vehicle heading data.
        - Timestamps for signal images.
        - Timestamps for near-infrared images.
        - Timestamps for reflectivity images.
        - Timestamps for range images.
        - Timestamps for left GNSS data.
        - Timestamps for right GNSS data.
        - Timestamps for IMU data.
    """
    # Pure-python ROS1 bag reader via `rosbags` (Faster-LIO integration): replaces
    # bagpy so the pipeline runs on a non-ROS1 host. Same return contract and the
    # same record-time timestamps (t_ns/1e9 == ROS1 t.to_sec()) and per-topic
    # ordering as the original bagpy loop. /ouster/points and /vehicle_enu_heading
    # are absent from the reduced bag and unused by the main path.
    from rosbags.highlevel import AnyReader
    from pathlib import Path

    # Initialize lists to store data from different topics
    signal_image_data = []
    nearir_image_data = []
    reflec_image_data = []
    range_image_data = []
    vehicle_left_gnss_data = []
    vehicle_right_gnss_data = []
    point_cloud_data = []
    imu_data = []
    timestamps_signal = []
    timestamps_nearir = []
    timestamps_reflec = []
    timestamps_range = []
    timestamps_left_gnss = []
    timestamps_right_gnss = []
    timestamps_imu = []
    vehicle_heading_data = []

    def _img16(m):
        # rosbags exposes uint8[] `data` as an ndarray; reinterpret as uint16 image
        return np.frombuffer(np.ascontiguousarray(m.data).tobytes(),
                             dtype=np.uint16).reshape((m.height, m.width))

    with AnyReader([Path(bag_file_path)]) as reader:
        for conn, t_ns, raw in reader.messages():
            topic = conn.topic
            t = t_ns / 1e9  # record time in seconds (== ROS1 t.to_sec())
            if topic == '/ouster/signal_image':
                signal_image_data.append(_img16(reader.deserialize(raw, conn.msgtype)))
                timestamps_signal.append(t)
            elif topic == '/ouster/nearir_image':
                nearir_image_data.append(_img16(reader.deserialize(raw, conn.msgtype)))
                timestamps_nearir.append(t)
            elif topic == '/ouster/reflec_image':
                reflec_image_data.append(_img16(reader.deserialize(raw, conn.msgtype)))
                timestamps_reflec.append(t)
            elif topic == '/ouster/range_image':
                range_image_data.append(_img16(reader.deserialize(raw, conn.msgtype)))
                timestamps_range.append(t)
            elif topic == '/gps_left_position':
                m = reader.deserialize(raw, conn.msgtype)
                vehicle_left_gnss_data.append((m.latitude, m.longitude))
                timestamps_left_gnss.append(t)
            elif topic == '/gps_right_position':
                m = reader.deserialize(raw, conn.msgtype)
                vehicle_right_gnss_data.append((m.latitude, m.longitude))
                timestamps_right_gnss.append(t)
            elif topic == '/ouster/imu':
                m = reader.deserialize(raw, conn.msgtype)
                imu_data.append({
                    'timestamp': t,
                    'orientation': {'x': m.orientation.x, 'y': m.orientation.y,
                                    'z': m.orientation.z, 'w': m.orientation.w},
                    'angular_velocity': {'x': m.angular_velocity.x, 'y': m.angular_velocity.y,
                                         'z': m.angular_velocity.z},
                    'linear_acceleration': {'x': m.linear_acceleration.x, 'y': m.linear_acceleration.y,
                                            'z': m.linear_acceleration.z},
                })
                timestamps_imu.append(t)

    # Return the collected data as a tuple of lists
    return (signal_image_data, nearir_image_data, reflec_image_data, range_image_data, vehicle_left_gnss_data, vehicle_right_gnss_data, imu_data, point_cloud_data, vehicle_heading_data,
            timestamps_signal, timestamps_nearir, timestamps_reflec, timestamps_range, timestamps_left_gnss, timestamps_right_gnss, timestamps_imu)


def process_imu_data(imu_data):
    """
    Processes IMU data to extract orientation, angular velocity, and linear acceleration.

    Args:
    imu_data (list): A list of dictionaries containing IMU data.

    Returns:
    dict: A dictionary containing separate lists for orientation, angular velocity, and linear acceleration.
    """
    orientation_data = []
    angular_velocity_data = []
    linear_acceleration_data = []
    imu_timestamps = []

    for entry in imu_data:
        imu_timestamps.append(entry['timestamp'])
        orientation_data.append(entry['orientation'])
        angular_velocity_data.append(entry['angular_velocity'])
        linear_acceleration_data.append(entry['linear_acceleration'])

    return {
        'timestamps': imu_timestamps,
        'orientation': orientation_data,
        'angular_velocity': angular_velocity_data,
        'linear_acceleration': linear_acceleration_data
    }

def calculate_vehicle_heading_from_two_gnss(lat1, lon1, lat2, lon2):
    """
    Calculate the heading between two geographical points provided by GNSS sensors using the Haversine formula.
    This function determines the vehicle's heading by calculating the angle from an initial point to a final point,
    which represents consecutive readings from GNSS sensors positioned on a vehicle.

    Args:
    lat1 (float): Latitude of the first GNSS sensor reading (degrees).
    lon1 (float): Longitude of the first GNSS sensor reading (degrees).
    lat2 (float): Latitude of the second GNSS sensor reading (degrees).
    lon2 (float): Longitude of the second GNSS sensor reading (degrees).

    Returns:
    float: The calculated heading from the first point to the second point in degrees.
    """
    # Convert all latitude and longitude values from degrees to radians for calculation
    lat1, lon1, lat2, lon2 = map(np.radians, [lat1, lon1, lat2, lon2])
    dlon = lon2 - lon1  # Calculate the difference in longitude
    
    # Calculate the x and y components of the direction vector
    x = np.sin(dlon) * np.cos(lat2)
    y = np.cos(lat1) * np.sin(lat2) - (np.sin(lat1) * np.cos(lat2) * np.cos(dlon))
    
    # Calculate the arctangent of y and x which gives the heading in radians, then convert to degrees
    heading = np.degrees(np.arctan2(x, y))
    
    # Normalize the heading to ensure it is between 0 and 360 degrees
    heading = (heading + 360) % 360
    
    print(f"Calculated GPS Heading: {heading} degrees")  # Output the calculated heading to the console
    return heading  # Return the calculated heading


def calculate_vehicle_heading_from_two_utm(easting1, northing1, easting2, northing2):
    """
    Calculate the heading between two geographical points provided by UTM coordinates.
    This function determines the vehicle's heading by calculating the angle from an initial point
    to a final point, which represents consecutive readings from GNSS sensors positioned on a vehicle.

    Args:
    easting1 (float): Easting of the first GNSS sensor reading (meters).
    northing1 (float): Northing of the first GNSS sensor reading (meters).
    easting2 (float): Easting of the second GNSS sensor reading (meters).
    northing2 (float): Northing of the second GNSS sensor reading (meters).

    Returns:
    float: The calculated heading from the first point to the second point in degrees.
    """
    # Calculate the difference in easting and northing
    delta_easting = easting2 - easting1
    delta_northing = northing2 - northing1
    
    # Calculate the heading using the arctangent of the delta values
    heading_rad = np.arctan2(delta_easting, delta_northing)
    
    # Convert the heading from radians to degrees
    heading = np.degrees(heading_rad)
    
    # Normalize the heading to ensure it is between 0 and 360 degrees
    heading = (heading + 360) % 360
    
    print(f"Calculated UTM Heading: {heading} degrees")  # Output the calculated heading to the console
    return heading  # Return the calculated heading


def calculate_vehicle_heading_direction(lat1, lon1, heading):
    """
    Calculate end coordinates for an arrow showing the vehicle's heading.
    
    Parameters:
    - lat1, lon1: The starting latitude and longitude.
    - heading: The calculated heading angle in degrees.

    Returns:
    - end_lon, end_lat: End coordinates for the heading arrow.
    """
    # Convert heading to radians
    heading_rad = np.radians(heading)
    # Define the length of the arrow
    arrow_length = 0.005
    # Calculate the end point of the arrow
    end_lon = lon1 + arrow_length * np.sin(heading_rad)
    end_lat = lat1 + arrow_length * np.cos(heading_rad)
    return end_lon, end_lat

def inverse_haversine(lat, lon, distance, bearing):
    """
    Compute the destination geographic coordinates given a start point, distance,
    and bearing using the inverse Haversine (great-circle) formula.

    This function calculates the latitude and longitude of a point reached by
    traveling a specified distance from an initial geographic location along
    a given bearing, assuming a spherical Earth model.

    Parameters
    ----------
    lat : float
        Latitude of the starting point in degrees (WGS84).
    lon : float
        Longitude of the starting point in degrees (WGS84).
    distance : float
        Distance to travel from the starting point in meters.
    bearing : float
        Bearing (azimuth) in degrees, measured clockwise from true North.

    Returns
    -------
    lat2 : float
        Latitude of the destination point in degrees (WGS84).
    lon2 : float
        Longitude of the destination point in degrees (WGS84).

    Notes
    -----
    - Assumes a spherical Earth with radius 6,371,000 meters.
    - The bearing is defined as:
        0°   → North  
        90°  → East  
        180° → South  
        270° → West
        """
    R = 6371000  # Radius of Earth in meters
    
    bearing_rad = math.radians(bearing)

    lat1 = math.radians(lat)
    lon1 = math.radians(lon)

    lat2 = math.asin(math.sin(lat1) * math.cos(distance / R) +
                     math.cos(lat1) * math.sin(distance / R) * math.cos(bearing_rad))

    lon2 = lon1 + math.atan2(math.sin(bearing_rad) * math.sin(distance / R) * math.cos(lat1),
                             math.cos(distance / R) - math.sin(lat1) * math.sin(lat2))

    lat2 = math.degrees(lat2)
    lon2 = math.degrees(lon2)

    return lat2, lon2

def calculate_azimuth_elevation_with_two_points(x1, y1, z1, x2, y2, z2):
    """
    Calculate the azimuth and elevation angles between two points.
    
    Parameters:
    - (x1, y1, z1): Coordinates of the first point.
    - (x2, y2, z2): Coordinates of the second point.
    
    Returns:
    - azimuth: Azimuth angle in degrees.
    - elevation: Elevation angle in degrees.
    """
    dx = x2 - x1
    dy = y2 - y1
    dz = z2 - z1
    
    azimuth = np.arctan2(dy, dx) * (180 / np.pi)
    elevation = np.arcsin(dz / np.sqrt(dx**2 + dy**2 + dz**2)) * (180 / np.pi)
    
    return azimuth, elevation

def calculate_azimuth_elevation(x, y, z):
    """
    Compute azimuth and elevation angles from a 3D Cartesian vector.

    Given a point or direction vector expressed in a local Cartesian
    coordinate system (x, y, z), this function computes:
      - Azimuth: horizontal angle in the x–y plane
      - Elevation: vertical angle relative to the horizontal plane

    Parameters
    ----------
    x : float or np.ndarray
        X-coordinate in the local frame.
    y : float or np.ndarray
        Y-coordinate in the local frame.
    z : float or np.ndarray
        Z-coordinate in the local frame.

    Returns
    -------
    azimuth : float or np.ndarray
        Azimuth angle in degrees, measured counter-clockwise from the
        positive x-axis in the x–y plane.
    elevation : float or np.ndarray
        Elevation angle in degrees, measured from the horizontal plane
        toward the positive z-axis.

   """

    azimuth = np.arctan2(y, x) * (180 / np.pi)
    elevation = np.arcsin(z / np.sqrt(x**2 + y**2 + z**2)) * (180 / np.pi)
    return azimuth, elevation

def inverse_haversine(lat, lon, azimuth, distance):
    """
    Compute the destination latitude and longitude from a start point,
    azimuth, and distance using an ellipsoidal Earth model (WGS84).

    This function uses GeographicLib's direct geodesic solution to
    propagate a point along a given azimuth for a specified distance
    on the WGS84 ellipsoid.

    Parameters
    ----------
    lat : float
        Latitude of the starting point in degrees (WGS84).
    lon : float
        Longitude of the starting point in degrees (WGS84).
    azimuth : float
        Forward azimuth (bearing) in degrees, measured clockwise
        from true North.
    distance : float
        Travel distance from the starting point in meters.

    Returns
    -------
    lat2 : float
        Latitude of the destination point in degrees (WGS84).
    lon2 : float
        Longitude of the destination point in degrees (WGS84).
    """

    geod = Geodesic.WGS84
    result = geod.Direct(lat, lon, azimuth, distance)
    return result['lat2'], result['lon2']


def local_xyz_to_utm33_with_heading_inverse_haversine(vehicle_easting, vehicle_northing, heading, x, y, z, gnss_to_lidar_offset):
    """
    Transform a local 3D point (e.g., LiDAR measurement) into global
    UTM Zone 33 coordinates using vehicle position and heading.

    The function projects a local Cartesian point (x, y, z), expressed
    in the vehicle/LiDAR frame, onto the Earth's surface by:
      1) Converting the vehicle's UTM position to latitude/longitude.
      2) Computing the horizontal distance and relative angle of the
         local point in the vehicle frame.
      3) Adjusting the vehicle heading with the local point angle.
      4) Propagating the position geodesically using the inverse
         geodesic (ellipsoidal) formulation.
      5) Converting the resulting latitude/longitude back to UTM
         coordinates.

    Parameters
    ----------
    vehicle_easting : float
        Vehicle UTM Easting coordinate (meters), UTM Zone 33.
    vehicle_northing : float
        Vehicle UTM Northing coordinate (meters), UTM Zone 33.
    heading : float
        Vehicle heading in degrees, measured clockwise from true North.
    x : float
        Local x-coordinate of the point (meters) in the vehicle/LiDAR frame.
    y : float
        Local y-coordinate of the point (meters) in the vehicle/LiDAR frame.
    z : float
        Local z-coordinate of the point (meters) in the vehicle/LiDAR frame.
        (Currently not used in the horizontal geodesic projection.)
    gnss_to_lidar_offset : float or array-like
        Offset between GNSS antenna and LiDAR sensor (currently not applied,
        reserved for future extension).

    Returns
    -------
    target_easting : float
        UTM Easting coordinate (meters) of the projected point.
    target_northing : float
        UTM Northing coordinate (meters) of the projected point.
        """
    # Convert UTM to latitude and longitude
    lat, lon = utm.to_latlon(vehicle_easting, vehicle_northing, 33, 'N')

    # Calculate the distance and adjusted heading
    distance = math.sqrt(x**2 + y**2)
    angle = math.atan2(y, x)
    adjusted_heading = (heading + math.degrees(angle)) % 360

    # Calculate new latitude and longitude using inverse Haversine
    new_lat, new_lon = inverse_haversine(lat, lon, distance, adjusted_heading)

    # Convert the new latitude and longitude back to UTM
    target_easting, target_northing, _, _ = utm.from_latlon(new_lat, new_lon)

    return target_easting, target_northing

def calculate_vehicle_heading_direction_utm(easting1, northing1, heading):
    """
    Calculate end coordinates for an arrow showing the vehicle's heading in UTM coordinates.
    
    Parameters:
    - easting1, northing1: The starting easting and northing in UTM coordinates.
    - heading: The calculated heading angle in degrees from North.

    Returns:
    - end_easting, end_northing: End coordinates for the heading arrow in UTM coordinates.
    """
    # Convert heading to radians
    heading_rad = np.radians(heading)
    # Define the length of the arrow
    arrow_length = 0.005 # 0.005 meters for visualization, adjust as needed for the scale of the map or application

    # Calculate the end point of the arrow using simple trigonometry on a flat plane
    end_easting = easting1 + arrow_length * np.sin(heading_rad)
    end_northing = northing1 + arrow_length * np.cos(heading_rad)
    
    return end_easting, end_northing



def transform_lidar_to_sensor(x, y, z):
    """
    Transforms a point from the LiDAR coordinate frame to the Sensor coordinate frame.

    Parameters:
    - x, y, z: Coordinates of the point in the LiDAR frame.

    Returns:
    - Transformed coordinates in the Sensor frame.
    """
    # Transformation matrix from LiDAR to Sensor coordinate frame
    # M_lidar_to_sensor = np.array([
    #     [-1, 0, 0, 0],
    #     [0, -1, 0, 0],
    #     [0, 0, 1, 38.195],
    #     [0, 0, 0, 1]
    M_lidar_to_sensor = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    
    # Point in LiDAR coordinate frame
    point_lidar = np.array([x, y, z, 1]) # Homogeneous coordinates
    
    # Transform point to Sensor coordinate frame
    point_sensor = np.dot(M_lidar_to_sensor, point_lidar)
    
    return point_sensor[:3] # Return the x, y, z coordinates; ignore the homogeneous coordinate

def display_range_from_xyz(xyz):
    """
    Computes and displays the range image from xyz values.

    Parameters:
    - xyz: Numpy array of shape (height, width, 3) where each entry [i, j]
            contains the X, Y, Z coordinates of a point.
    """
    # Reshape the structured XYZ array to a flat (N, 3) array
    xyz_flat = xyz.reshape(-1, 3)  # Flatten the array maintaining 3 columns for X, Y, Z
    print(f'xyz_flat shape: {xyz_flat.shape}')
    # Compute range from xyz values
    range_vals = np.sqrt(np.sum(xyz_flat**2, axis=1))  # Sum along the last axis and take the square root

    # A bit of scaling
    # range_vals_scaled = range_vals / 4 * 1000
    range_vals_scaled = (range_vals / 1000)*4
    # reshape to 2D
    range_vals_scaled = range_vals_scaled.reshape(128, 1024)
    print(f'range_vals_scaled shape: {range_vals_scaled[:10]}')
        # Normalize and convert to uint8
    img_data_range = (range_vals_scaled / range_vals_scaled.max() * 255).astype(np.uint8)


    # Reshape to correct size (assuming the original array is already correctly sized, this line may be unnecessary)
    img_data_range = img_data_range.reshape(128, 1024)

    return img_data_range, range_vals_scaled




def display_xyz_as_pointcloud(xyz):
    """
    Visualizes XYZ coordinates as a point cloud using Open3D on a black background,
    includes a coordinate frame for orientation, and custom view settings.

    Parameters:
    - xyz: Numpy array of shape (N, 3) where N is the number of points and
      each row contains the X, Y, Z coordinates of a point.
    """
    if xyz is None or xyz.size == 0:
        print("No XYZ data provided or empty array.")
        return

    # Reshape the structured XYZ array to a flat (N, 3) array
    xyz_flat = xyz.reshape(-1, 3)  # Flatten the array maintaining 3 columns for X, Y, Z

    # Create a point cloud object from the flattened XYZ data
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz_flat)

    # Setup visualization
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="XYZ Point Cloud", width=800, height=600)

    # Set the background color to black
    opt = vis.get_render_option()
    opt.background_color = np.asarray([0, 0, 0])

    # Add the point cloud to the visualizer
    vis.add_geometry(pcd)

    # Add coordinate frame for orientation
    vis.add_geometry(o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3))

    # Configure view to optimize visibility
    view_ctl = vis.get_view_control()
    view_ctl.set_front([0, -1, -0.5])  # Adjust based on the specific point cloud orientation
    view_ctl.set_lookat([0, 0, 0])
    view_ctl.set_up([0, 0, 1])
    view_ctl.set_zoom(0.8)

    # Run the visualization
    vis.run()
    vis.destroy_window()

def extract_nearest_point_in_bounding_box_region(x_min, x_max, y_min, y_max, rgb_image, range_image_data, i):
    """
    This function extracts the nearest point in the region around the center of the bounding box. It calculates
    the center coordinates of the bounding box, determines the window size, and handles 360-degree image
    wrap-around to ensure the region is correctly analyzed even if it crosses the image boundary. The function
    also finds the nearest non-zero distance within that region and calculates its global position in the
    entire range image, then returns these global coordinates.

    Args:
    x_min (float): The minimum x-coordinate of the bounding box.
    x_max (float): The maximum x-coordinate of the bounding box.
    y_min (float): The minimum y-coordinate of the bounding box.
    y_max (float): The maximum y-coordinate of the bounding box.
    rgb_image (ndarray): The RGB image from which the dimensions are extracted. Must be a 2D array.
    range_image_data (list of ndarrays): A list of 2D arrays, each representing range image data at different times.
    i (int): Index of the specific image in `range_image_data` to analyze.

    Returns:
    tuple: A tuple containing the global coordinates (global_x, global_y) of the nearest non-zero distance point.
    """
   
    
    # Calculate center of bounding box
    x_center, y_center = (x_min + x_max) / 2, (y_min + y_max) / 2

    # Define the window size based on the bounding box width
    window_size = x_max - x_min

    # Obtain the height and width of the RGB image
    image_height, image_width = rgb_image.shape[:2]

    # Calculate vertical bounds within the image
    region_y_min = max(int(y_center - window_size / 2), 0)
    region_y_max = min(int(y_center + window_size / 2), image_height - 1)

    # Handle 360-degree wrap-around for horizontal bounds
    region_x_min_left = int(x_center - window_size / 2) % image_width
    region_x_max_right = int(x_center + window_size / 2) % image_width

    # Determine if the region is contiguous or needs to be split due to wrap-around
    if region_x_min_left < region_x_max_right:
        center_region = range_image_data[i][region_y_min:region_y_max, region_x_min_left:region_x_max_right]
        offset_x = region_x_min_left
    else:
        center_region_left = range_image_data[i][region_y_min:region_y_max, region_x_min_left:]
        center_region_right = range_image_data[i][region_y_min:region_y_max, :region_x_max_right]
        center_region = np.concatenate((center_region_left, center_region_right), axis=1)
        offset_x = region_x_min_left
    # print center_region
    print('center region around the center', center_region)


    # Find indices of non-zero distances in the center region
    non_zero_indices = np.where(center_region > 0)
    # print maximum and minimum non zero distance


    if non_zero_indices[0].size > 0:
        non_zero_values = center_region[non_zero_indices]
        print('non_zero_values', non_zero_values)

        min_non_zero_idx_flat = np.argmin(non_zero_values) # Find the index of the minimum non-zero distance
        
        nearest_distance_idx = (non_zero_indices[0][min_non_zero_idx_flat], non_zero_indices[1][min_non_zero_idx_flat]) # Get the index of the nearest non-zero distance
        nearest_distance = center_region[nearest_distance_idx] / 1000  # Convert to meters

        # Calculate global position of the nearest distance within the entire range image
        global_y = region_y_min + nearest_distance_idx[0]
        if region_x_min_left < region_x_max_right:
            global_x = offset_x + nearest_distance_idx[1]
        else:
            if nearest_distance_idx[1] >= center_region_left.shape[1]:
                global_x = nearest_distance_idx[1] - center_region_left.shape[1]
            else:
                global_x = offset_x + nearest_distance_idx[1]

        return (global_x, global_y, nearest_distance)

    else:
        # Return None or suitable defaults if no non-zero distances found
        return (None, None, None)
    

def local_xyz_to_latlon_with_heading(vehicle_lat, vehicle_lon, vehicle_heading, x, y, z, gnss_offset, lidar_offset):
    """
    Convert local Cartesian coordinates (x, y, z) of a point on an object with respect
    to the vehicle's GNSS location and heading into geographical coordinates (latitude, longitude),
    considering the offsets of the vehicle's sensors.

    Parameters:
    - vehicle_lat: Latitude of the vehicle (degrees).
    - vehicle_lon: Longitude of the vehicle (degrees).
    - vehicle_heading: Heading of the vehicle (degrees from North).
    - x, y, z: Local X, Y, Z coordinates relative to the LiDAR sensor (meters).
    - gnss_offset: GNSS sensor offset from vehicle's reference point [x, y, z] (meters).
    - lidar_offset: LiDAR sensor offset from vehicle's reference point [x, y, z] (meters).

    Returns:
    - Tuple of (latitude, longitude) for the point.
    """
    geod = Geodesic.WGS84

    # Calculate the offset from LiDAR to GNSS
    lidar_to_gnss_offset = [lidar_offset[i] - gnss_offset[i] for i in range(3)]

    # Adjust local coordinates by the LiDAR to GNSS offset
    x_adj = x + lidar_to_gnss_offset[0]
    y_adj = y + lidar_to_gnss_offset[1]
    z_adj = z + lidar_to_gnss_offset[2]  # Generally ignored for lat/lon calculations

    # Convert vehicle heading to radians for calculation
    heading_rad = math.radians(vehicle_heading)

    # Rotate the adjusted local coordinates (x_adj, y_adj) by the vehicle's heading
    x_rotated = x_adj * math.cos(heading_rad) - y_adj * math.sin(heading_rad)
    y_rotated = x_adj * math.sin(heading_rad) + y_adj * math.cos(heading_rad)

    # Calculate the new position by applying the rotated distances
    intermediate_result = geod.Direct(vehicle_lat, vehicle_lon, 0, y_rotated)
    final_result = geod.Direct(intermediate_result['lat2'], intermediate_result['lon2'], 90, x_rotated)

    return final_result['lat2'], final_result['lon2']



def local_to_utm33_without_heading(easting, northing, x, y, z, lidar_to_gnss_offset):
    """
    Transform a local Cartesian point into UTM Zone 33 coordinates using
    direct planar translation without considering vehicle heading.

    This function applies a fixed LiDAR-to-GNSS offset to a local point
    (x, y, z) and directly adds the resulting horizontal displacement
    to the vehicle's UTM Easting and Northing coordinates. The transformation
    assumes that the local coordinate frame is already aligned with the
    global UTM axes and ignores Earth curvature and heading effects.

    Parameters
    ----------
    easting : float
        Vehicle UTM Easting coordinate (meters), UTM Zone 33.
    northing : float
        Vehicle UTM Northing coordinate (meters), UTM Zone 33.
    x : float
        Local x-coordinate of the point (meters), assumed to align with
        the UTM Easting direction.
    y : float
        Local y-coordinate of the point (meters), assumed to align with
        the UTM Northing direction.
    z : float
        Local z-coordinate of the point (meters). This parameter is
        currently not used in the horizontal transformation.
    lidar_to_gnss_offset : tuple or array-like of length 3
        Fixed offset (x, y, z) in meters between the LiDAR sensor and
        the GNSS reference point, expressed in the same local frame.

    Returns
    -------
    final_easting : float
        UTM Easting coordinate (meters) of the transformed point.
    final_northing : float
        UTM Northing coordinate (meters) of the transformed point.
        """

    
    # Apply the LiDAR to GNSS offset to the displacement
    x_offset, y_offset, z_offset = lidar_to_gnss_offset
    x += x_offset
    y += y_offset
    # Optionally use z_offset if z displacement needs to be considered
  
    
    # Apply displacement in meters directly in the UTM coordinate system
    final_easting = easting + x
    final_northing = northing + y
    
    # Return the final UTM33 coordinates with applied offset
    return final_easting, final_northing

def local_xyz_to_utm33_with_heading(vehicle_easting, vehicle_northing, vehicle_heading, x, y, z, lidar_to_gnss_offset):
    """
    Convert local Cartesian coordinates (x, y, z) of a point on an object with respect
    to the vehicle's UTM33 coordinates and heading into UTM Zone 33N coordinates,
    considering sensor offsets.

    Parameters:
    - vehicle_easting: Easting of the vehicle in UTM Zone 33N (meters).
    - vehicle_northing: Northing of the vehicle in UTM Zone 33N (meters).
    - vehicle_heading: Heading of the vehicle (degrees from North).
    - x, y, z: Local coordinates relative to the LiDAR sensor (meters).
    - gnss_offset: Array [x, y, z] offset of the GNSS sensor from the vehicle's reference point (meters).
    - lidar_offset: Array [x, y, z] offset of the LiDAR sensor from the vehicle's reference point (meters).

    Returns:
    - Tuple of (easting, northing) for the point in UTM Zone 33N.
    """
  

    # Adjust local coordinates by LiDAR to GNSS offset
    x_adj = x + lidar_to_gnss_offset[0]
    y_adj = y + lidar_to_gnss_offset[1]
    z_adj = z + lidar_to_gnss_offset[2]  # This might be ignored depending on use-case

    # Convert vehicle heading to radians for calculation
    heading_rad = math.radians(vehicle_heading)

    # Rotate the adjusted local coordinates (x_adj, y_adj) by the vehicle's heading
    x_rotated = x_adj * math.cos(heading_rad) - y_adj * math.sin(heading_rad)
    y_rotated = x_adj * math.sin(heading_rad) + y_adj * math.cos(heading_rad)

    # Apply rotated coordinates to the vehicle's UTM coordinates
    final_easting = vehicle_easting + x_rotated
    final_northing = vehicle_northing + y_rotated

    return final_easting, final_northing

def calculate_distance(center_xyz, gnss_offset):
    """
    Compute the Euclidean distance between two points in 3D Cartesian space.

    This function calculates the straight-line (L2) distance between a
    reference point (e.g., a LiDAR point or object center) and a GNSS-related
    offset expressed in the same local Cartesian coordinate frame.

    Parameters
    ----------
    center_xyz : tuple or array-like of length 3
        Reference point coordinates (x, y, z) in meters.
    gnss_offset : tuple or array-like of length 3
        Offset coordinates (x, y, z) in meters, typically representing the
        displacement between a GNSS reference point and another sensor or
        object.

    Returns
    -------
    distance : float
        Euclidean distance between the two points in meters.
        """

    # Unpack the coordinates
    x1, y1, z1 = center_xyz
    x2, y2, z2 = gnss_offset
    
    # Calculate the Euclidean distance
    distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)
    return distance


def local_to_utm33_with_offset_proj(lat, lon, vehicle_heading, azimuth, distance):
    """
    Project a local displacement into global UTM Zone 33 coordinates using
    heading-aware geodesic propagation on the WGS84 ellipsoid.

    This function computes a global position by:
      1) Adjusting a local azimuth with the vehicle heading.
      2) Propagating the position geodesically from a given latitude/longitude
         using the adjusted azimuth and travel distance.
      3) Converting the resulting geographic coordinates to UTM Zone 33
         Easting/Northing.

    Parameters
    ----------
    lat : float
        Latitude of the vehicle or reference point in degrees (WGS84).
    lon : float
        Longitude of the vehicle or reference point in degrees (WGS84).
    vehicle_heading : float
        Vehicle heading in degrees, measured clockwise from true North.
    azimuth : float
        Local azimuth angle in degrees, typically derived from a local
        Cartesian vector (e.g., LiDAR detection direction).
    distance : float
        Travel distance in meters to be projected from the reference point.

    Returns
    -------
    new_easting : float
        UTM Easting coordinate (meters) of the projected point in UTM Zone 33.
    new_northing : float
        UTM Northing coordinate (meters) of the projected point in UTM Zone 33.
    adjusted_azimuth : float
        Global azimuth (degrees) after combining vehicle heading and local
        azimuth, wrapped to [0, 360).
        """


    # Initialize Geod object with WGS84 ellipsoid
    geod = Geod(ellps='WGS84')
    print(f'vehicle_heading: {vehicle_heading}')
    print(f'azimuth: {azimuth}')
    
    # Adjust azimuth by the vehicle's heading
    adjusted_azimuth = (vehicle_heading + azimuth) % 360
    print(f'adjusted_azimuth: {adjusted_azimuth}')
    
    # Calculate the new position in lat/lon
    new_lon, new_lat, back_azimuth = geod.fwd(lon, lat, adjusted_azimuth, distance)
    
    # Transformer for converting between geographic and UTM coordinates (UTM zone 33N)
    transformer_to_proj = Transformer.from_crs('epsg:4326', 'epsg:32633', always_xy=True)
    
    # Convert new lat/lon to easting/northing
    new_easting, new_northing = transformer_to_proj.transform(new_lon, new_lat)
    
    return new_easting, new_northing, adjusted_azimuth


def utm33_to_orig_with_back_azimuth(new_easting, new_northing, vehicle_heading, back_azimuth, distance):
    """
    Calculate the original UTM coordinates from the new UTM coordinates, vehicle heading, back azimuth, and distance.

    Parameters:
    - new_easting: Easting of the new position in UTM coordinates
    - new_northing: Northing of the new position in UTM coordinates
    - vehicle_heading: Heading of the vehicle in degrees
    - back_azimuth: Back azimuth angle in degrees
    - distance: Distance to the original point in meters

    Returns:
    - orig_easting: Easting of the original position in UTM coordinates
    - orig_northing: Northing of the original position in UTM coordinates
    """
    # Transformer for converting between UTM coordinates (UTM zone 33N) and geographic coordinates
    transformer_to_latlon = Transformer.from_crs('epsg:32633', 'epsg:4326', always_xy=True)
    transformer_to_utm33 = Transformer.from_crs('epsg:4326', 'epsg:32633', always_xy=True)
    
    # Convert new easting/northing to lat/lon
    new_lat, new_lon = transformer_to_latlon.transform(new_easting, new_northing)
    
    # Initialize Geod object with WGS84 ellipsoid
    geod = Geod(ellps='WGS84')
    
    if -90 < back_azimuth < 0:
        adjusted_azimuth = (vehicle_heading + back_azimuth) % 360
    else:
        adjusted_azimuth = (vehicle_heading + back_azimuth) % 360
  
    print(f'adjusted_azimuth: {adjusted_azimuth}')
    orig_lon, orig_lat, back_azimuth_pole = geod.fwd(new_lon, new_lat, adjusted_azimuth, distance)
    
    # Convert the original lat/lon to UTM33 coordinates
    orig_easting, orig_northing = transformer_to_utm33.transform(orig_lat, orig_lon)
    
    return orig_easting, orig_northing, back_azimuth_pole



def local_xyz_to_utm33_with_heading_inverse_haversine(vehicle_easting, vehicle_northing, heading, x, y, z, gnss_to_lidar_offset):
    """
    Project a local (x, y, z) displacement into global UTM Zone 33 coordinates using the
    vehicle GNSS position and heading, via a forward geodesic (“inverse haversine”) step.

    The function:
      1) Converts the vehicle UTM position (Zone 33N) to latitude/longitude.
      2) Computes the horizontal range `distance = sqrt(x^2 + y^2)` of the local point.
      3) Computes the local direction `angle = atan2(y, x)` (radians) and combines it with
         the vehicle heading to obtain a global bearing:
             adjusted_heading = (heading + deg(angle)) mod 360
      4) Moves from (lat, lon) by `distance` meters along `adjusted_heading` degrees using
         `inverse_haversine`.
      5) Converts the resulting (new_lat, new_lon) back to UTM (easting, northing).

    Parameters
    ----------
    vehicle_easting : float
        Vehicle UTM Easting (meters) in UTM Zone 33N.
    vehicle_northing : float
        Vehicle UTM Northing (meters) in UTM Zone 33N.
    heading : float
        Vehicle heading in degrees, measured clockwise from true North.
    x : float
        Local x displacement (meters) in the sensor/vehicle frame (used for horizontal projection).
    y : float
        Local y displacement (meters) in the sensor/vehicle frame (used for horizontal projection).
    z : float
        Local z displacement (meters). Currently not used in the geodesic projection.
    gnss_to_lidar_offset : tuple or array-like
        Offset between GNSS and LiDAR frames. Currently not applied (reserved for extension).

    Returns
    -------
    target_easting : float
        Projected UTM Easting (meters) of the local point in UTM Zone 33N.
    target_northing : float
        Projected UTM Northing (meters) of the local point in UTM Zone 33N.
        """


    # Convert UTM to latitude and longitude
    lat, lon = utm.to_latlon(vehicle_easting, vehicle_northing, 33, 'N')

    # Calculate the distance and adjusted heading
    distance = math.sqrt(x**2 + y**2)
    angle = math.atan2(y, x)
    adjusted_heading = (heading + math.degrees(angle)) % 360

    # Calculate new latitude and longitude using inverse Haversine
    new_lat, new_lon = inverse_haversine(lat, lon, distance, adjusted_heading)

    # Convert the new latitude and longitude back to UTM
    target_easting, target_northing, _, _ = utm.from_latlon(new_lat, new_lon)

    return target_easting, target_northing


def draw_vehicle_heading(lat1, lon1, heading):
    """
    Calculate end coordinates for an arrow showing the vehicle's heading.
    
    Parameters:
    - lat1, lon1: The starting latitude and longitude.
    - heading: The calculated heading angle in degrees.

    Returns:
    - end_lon, end_lat: End coordinates for the heading arrow.
    """
    # Convert heading to radians
    heading_rad = np.radians(heading)
    # Define the length of the arrow
    arrow_length = 0.001
    # Calculate the end point of the arrow
    end_lon = lon1 + arrow_length * np.sin(heading_rad)
    end_lat = lat1 + arrow_length * np.cos(heading_rad)
    return end_lon, end_lat


def calculate_destination_point_haversine(lat, lon, distance, bearing):
    """
    Compute a destination latitude/longitude from a start point, distance, and bearing
    using a spherical-Earth great-circle approximation (Haversine-style forward formula).

    Parameters
    ----------
    lat : float
        Starting latitude in degrees (WGS84).
    lon : float
        Starting longitude in degrees (WGS84).
    distance : float
        Travel distance from the start point in kilometers (km).
        (Note: this function uses Earth radius R=6371.0 km, so distance must be in km.)
    bearing : float
        Bearing (azimuth) in degrees, measured clockwise from true North.

    Returns
    -------
    lat2 : float
        Destination latitude in degrees (WGS84).
    lon2 : float
        Destination longitude in degrees (WGS84).
    """


    # Earth radius in kilometers
    R = 6371.0
    
    # Convert latitude and longitude from degrees to radians
    lat = math.radians(lat)
    lon = math.radians(lon)
    
    # Convert bearing to radians
    bearing = math.radians(bearing)
    
    # Calculate destination point
    lat2 = math.asin(math.sin(lat) * math.cos(distance / R) +
                     math.cos(lat) * math.sin(distance / R) * math.cos(bearing))
    lon2 = lon + math.atan2(math.sin(bearing) * math.sin(distance / R) * math.cos(lat),
                            math.cos(distance / R) - math.sin(lat) * math.sin(lat2))
    
    # Convert back to degrees
    lat2 = math.degrees(lat2)
    lon2 = math.degrees(lon2)
    
    return lat2, lon2

def calculate_destination_point_vincenty(lat, lon, distance_m, bearing):
    """
    Compute a destination latitude/longitude from a start point, distance, and bearing
    using an ellipsoidal Earth model via geopy's geodesic distance computation.

    Parameters
    ----------
    lat : float
        Starting latitude in degrees (WGS84).
    lon : float
        Starting longitude in degrees (WGS84).
    distance_m : float
        Travel distance from the start point in meters (m).
    bearing : float
        Bearing (azimuth) in degrees, measured clockwise from true North.

    Returns
    -------
    destination_point.latitude : float
        Destination latitude in degrees (WGS84).
    destination_point.longitude : float
        Destination longitude in degrees (WGS84).
    """


    # Create a geopy Point for the starting position
    start_point = Point(lat, lon)
    
    # Calculate the destination point using distance in meters
    destination_point = geopy_distance(meters=distance_m).destination(start_point, bearing)
    
    return destination_point.latitude, destination_point.longitude

def calculate_azimuth_elevation_from_gnss(x, y, z, gnss_to_lidar_offset):
    """
    Compute azimuth and elevation angles of a 3D point relative to the GNSS origin,
    accounting for a fixed GNSS-to-LiDAR (or GNSS-to-sensor) translation offset.

    The input point (x, y, z) is first translated by subtracting `gnss_to_lidar_offset`,
    producing an adjusted vector from the GNSS reference point to the target. Azimuth
    and elevation are then computed from this adjusted vector.

    Parameters
    ----------
    x : float
        X-coordinate of the point in the local sensor/vehicle frame (meters).
    y : float
        Y-coordinate of the point in the local sensor/vehicle frame (meters).
    z : float
        Z-coordinate of the point in the local sensor/vehicle frame (meters).
    gnss_to_lidar_offset : tuple or array-like of length 3
        Translation offset (dx, dy, dz) in meters from the GNSS reference point
        to the LiDAR/sensor origin, expressed in the same local frame.

    Returns
    -------
    azimuth : float
        Azimuth angle in degrees, computed as atan2(y_adj, x_adj).
        Range is typically (-180°, 180°].
    elevation : float
        Elevation angle in degrees above the horizontal plane, computed as
        asin(z_adj / ||[x_adj, y_adj, z_adj]||).
    """


    # Adjust the point by the origin offset
    x_adj = x - gnss_to_lidar_offset[0]
    y_adj = y - gnss_to_lidar_offset[1]
    z_adj = z - gnss_to_lidar_offset[2]
    
    # Calculate azimuth
    azimuth = np.arctan2(y_adj, x_adj) * (180 / np.pi)
    
    # Calculate elevation
    elevation = np.arcsin(z_adj / np.sqrt(x_adj**2 + y_adj**2 + z_adj**2)) * (180 / np.pi)
    
    print(f"Azimuth: {azimuth} degrees, Elevation: {elevation} degrees")
    
    return azimuth, elevation

def convert_azimuth_angle(azimuth_angle):
    """
    Converts azimuth angles:
    - From the range of 0 to 180 degrees (where North indicates 0 degrees)
      to the range of 360 to 180 degrees (where 0 indicates true North).
    - From the range of 0 to -180 degrees (where North indicates 0 degrees)
      to the range of 0 to 180 degrees (where 0 indicates true North).

    Parameters:
    - azimuth_angle: Azimuth angle in degrees (0 to 180 or 0 to -180).

    Returns:
    - Converted azimuth angle in degrees.
    """
    if 0 <= azimuth_angle <= 180:
        converted_angle = 360 - azimuth_angle if azimuth_angle != 0 else 0
    elif -180 <= azimuth_angle < 0:
        converted_angle = -azimuth_angle
    else:
        raise ValueError("Input angle should be in the range of 0 to 180 or 0 to -180 degrees.")

    return converted_angle

def calculate_easting_northing(easting_ref, northing_ref, distance, azimuth_angle):
    """
    Calculate the easting and northing of a point in UTM coordinates.

    Parameters:
    - easting_ref: Easting of the reference point (meters).
    - northing_ref: Northing of the reference point (meters).
    - distance: Distance from the reference point to the target point (meters).
    - azimuth_angle: Azimuth angle in degrees (0 to 360, where 0 indicates North).

    Returns:
    - (easting, northing): Tuple with the calculated easting and northing (meters).
    """
    # Convert azimuth angle to radians
    azimuth_rad = np.radians(azimuth_angle)

    # Calculate easting and northing
    easting = easting_ref + distance * np.cos(azimuth_rad)
    northing = northing_ref + distance * np.sin(azimuth_rad)

    return easting, northing


def write_gnss_data_to_csv(filename, data):
    """
    Writes GNSS data to a CSV file.

    Parameters:
    - filename: str, the name of the file to write to.
    - data: list of tuples, where each tuple contains (latitude, longitude).
    """
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['Latitude', 'Longitude'])
        writer.writerows(data)

def save_data_to_csv(timestamps, distances, azimuths, elevations, filename):
    """
    Save timestamped distance, azimuth, and elevation measurements to a CSV file.

    This function writes synchronized sensor-derived quantities—timestamps,
    distances, azimuth angles, and elevation angles—into a CSV file with a
    predefined header. Each row corresponds to one timestamped observation.

    Parameters
    ----------
    timestamps : array-like
        Sequence of timestamps (e.g., UNIX time or sensor time) associated
        with each measurement.
    distances : array-like
        Distances in meters corresponding to each timestamp.
    azimuths : array-like
        Azimuth angles in degrees corresponding to each timestamp.
    elevations : array-like
        Elevation angles in degrees corresponding to each timestamp.
    filename : str
        Path to the output CSV file.
    """
    with open(filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Timestamp', 'Distance (m)', 'Azimuth', 'Elevation'])
            for i in range(len(timestamps)):
                writer.writerow([timestamps[i], distances[i], azimuths[i], elevations[i]])
