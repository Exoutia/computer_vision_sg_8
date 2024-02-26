import open3d as o3d
import numpy as np

# Load point cloud
pcd = o3d.io.read_point_cloud('./apple.ply')

# Voxel Grid Downsampling
voxel_size = 0.01
pcd_downsampled = pcd.voxel_down_sample(voxel_size)

# Visualize the original point clouds
o3d.visualization.draw_geometries([pcd])

# Visualize the downsampled point clouds
o3d.visualization.draw_geometries([pcd_downsampled])

