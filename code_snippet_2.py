import open3d as o3d
import numpy as np

# Load the first PLY file
ply_file_1 = "./apple.ply"
point_cloud_1 = o3d.io.read_point_cloud(ply_file_1)

o3d.visualization.draw_geometries([point_cloud_1])

alpha = 0.05
print(f"alpha={alpha:.3f}")
mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(point_cloud_1, alpha)
mesh.compute_vertex_normals()
o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)
