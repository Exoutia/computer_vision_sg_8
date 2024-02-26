import numpy as np
import open3d as o3d

print("Load a ply point cloud, print it, and render it")
ply_point_cloud = './airplane.ply'
pcd = o3d.io.read_point_cloud(ply_point_cloud)
print(pcd)
print(np.asarray(pcd.points))
o3d.visualization.draw_geometries([pcd])