import numpy as np
import open3d as o3d
from sklearn.cluster import DBSCAN

ply_point_cloud = './airplane.ply'
pcd = o3d.io.read_point_cloud(ply_point_cloud)

# Convert point cloud to NumPy array
points = np.asarray(pcd.points)

# Cluster points using DBSCAN for object recognition
dbscan = DBSCAN(eps=0.1, min_samples=50)
labels = dbscan.fit_predict(points)

# Visualize the clustered objects
colors = labels % 10  # Assign different colors to different clusters
pcd.colors = o3d.utility.Vector3dVector(np.array([colors, np.zeros_like(colors), np.zeros_like(colors)]).T / 10.0)
o3d.visualization.draw_geometries([pcd])
