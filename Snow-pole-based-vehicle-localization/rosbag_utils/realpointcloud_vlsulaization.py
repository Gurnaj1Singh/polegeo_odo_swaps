from bagpy import bagreader
import pandas as pd
import numpy as np
import open3d as o3d

# Path to ROS bag
bag_path = './2024-02-28-12-59-51.bag'

# Read the bag
b = bagreader(bag_path)

# Extract point cloud topic to CSV
pc_csv = b.message_by_topic('/ouster/points')

# Load CSV
df = pd.read_csv(pc_csv)

# Check required fields
required_fields = {'x', 'y', 'z'}
if not required_fields.issubset(df.columns):
    raise ValueError(f"CSV does not contain required fields {required_fields}")

# Initialize Open3D point cloud and visualizer
o3d_cloud = o3d.geometry.PointCloud()
vis = o3d.visualization.Visualizer()
vis.create_window()

# Visualization settings
opt = vis.get_render_option()
opt.background_color = np.array([0, 0, 0])
opt.point_size = 2.0
opt.mesh_shade_option = o3d.visualization.MeshShadeOption.Color

# Add coordinate frame
coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
    size=0.5, origin=[0, 0, 0]
)
vis.add_geometry(coord_frame)

# Convert CSV to numpy array
points = df[['x', 'y', 'z']].values.astype(np.float64)

# Assign points
o3d_cloud.points = o3d.utility.Vector3dVector(points)

# Optional voxel downsampling
voxel_down_pcd = o3d_cloud.voxel_down_sample(voxel_size=0.05)

# Add geometry
vis.add_geometry(voxel_down_pcd)

# Visualization loop (static frame)
while True:
    vis.poll_events()
    vis.update_renderer()
