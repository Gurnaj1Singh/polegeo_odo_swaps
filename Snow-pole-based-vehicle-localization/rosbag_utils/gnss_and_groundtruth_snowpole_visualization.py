
import pandas as pd
import matplotlib.pyplot as plt
from pyproj import Transformer
import bagpy
from bagpy import bagreader
from mpl_toolkits.axes_grid1.inset_locator import inset_axes

# Reading and transforming Reference GNSS data
dataRefPd = pd.read_csv('Groundtruth_pole_location_at_test_site_E39_Hemnekjølen.csv')
transformer = Transformer.from_crs("epsg:32633", "epsg:4326", always_xy=True)
dataRef_gnss = transformer.transform(dataRefPd["UTM33-Øst"].to_numpy(), dataRefPd["UTM33-Nord"].to_numpy())

# Initialize the bag reader for Vehicle GNSS Data
bag = bagpy.bagreader('snow_pole_geo_localization_data/2024-02-28-12-59-51.bag')

# Initialize lists to store latitude and longitude data for Vehicle GNSS
latitude_data = []
longitude_data = []

# Iterate through the messages in the bag file for a specific topic
for topic, msg, t in bag.reader.read_messages(topics=['/gps_left_position']):
    latitude = msg.latitude
    longitude = msg.longitude
    latitude_data.append(latitude)
    longitude_data.append(longitude)

# Create a figure and axis for the plot
fig, ax = plt.subplots(figsize=(12, 8))

# Plot Reference GNSS data statically
ax.scatter(dataRef_gnss[0], dataRef_gnss[1], c='green', label='Measured Pole Locations', marker='x', s=50)

# Plot Vehicle GNSS data from ROS bag
ax.scatter(longitude_data, latitude_data, marker='o', linestyle='-', color='blue', label="Vehicle's GNSS Track", s=50)

# Adding labels, title, and grid
ax.set_xlabel('Longitude', fontsize=20)
ax.set_ylabel('Latitude', fontsize=20)
ax.tick_params(axis='both', which='major', labelsize=14)
ax.set_title("GNSS Data: Reference Poles vs Vehicle Track",
             fontsize=24, fontweight='bold')

ax.grid(True)

# Adding a legend
ax.legend(fontsize=20)

# Define the bounds for the zoomed-in area
# Adjust these values to focus on your area of interest
# x1, x2, y1, y2 = -0.01, 0.01, 63.24, 63.245
# Define the bounds for the zoomed-in area based on the provided values
x1, x2, y1, y2 = 9.5609, 9.5620, 63.2400, 63.2415

# Add inset of the axes
axins = inset_axes(ax, width="30%", height="30%", loc='lower left',
                   bbox_to_anchor=(0.5, 0.1, 1, 1),
                   bbox_transform=ax.transAxes)
axins.scatter(dataRef_gnss[0], dataRef_gnss[1], c='green', marker='x', s=40)
axins.scatter(longitude_data, latitude_data, marker='o', linestyle='-', color='blue', s=40)
axins.set_xlim(x1, x2)  # Set the x-axis limits for the zoomed-in area
axins.set_ylim(y1, y2)  # Set the y-axis limits for the zoomed-in area

# Optional: Add grid to the inset for better readability
axins.grid(True)

# Save the plot to a PDF file
plt.savefig('GNSS_Data_Visualization_with_Zoom.PNG', dpi=300, bbox_inches='tight')

# Show the plot
plt.show()