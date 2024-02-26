import open3d as o3d
import numpy as np

# Generate a simulated point cloud
num_points = 1000
points = np.random.rand(num_points, 3) * 10  # Random 3D points in a 10x10x10 cube

# Create a PointCloud object
lidar_cloud = o3d.geometry.PointCloud()
lidar_cloud.points = o3d.utility.Vector3dVector(points)

# Visualize the point cloud
o3d.visualization.draw_geometries([lidar_cloud])
