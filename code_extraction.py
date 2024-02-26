import copy

import numpy as np
import open3d as o3d

# Load a point cloud from a file
point_cloud = o3d.io.read_point_cloud("./ant.ply")


# Compute curvature
def calculate_surface_curvature(pcd, radius=0.1, max_nn=30):
    pcd_n = copy.deepcopy(pcd)
    pcd_n.estimate_covariances(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=max_nn)
    )
    covs = np.asarray(pcd_n.covariances)
    vals, vecs = np.linalg.eig(covs)
    curvature = np.min(vals, axis=1) / np.sum(vals, axis=1)
    return curvature


curvature = calculate_surface_curvature(point_cloud)


# Compute edges
def calcuate_edges_and_line_set(pcd, radius=0.02):
    point_cloud.estimate_normals()

    # Create a KDTree for efficient nearest neighbor search
    kdtree = o3d.geometry.KDTreeFlann(point_cloud)

    # Set the radius for edge computation
    radius = 0.02

    # List to store lines (edges)
    edges = []

    # Iterate through each point in the point cloud
    for i in range(len(point_cloud.points)):
        # Find neighbors within the specified radius
        [k, idx, _] = kdtree.search_radius_vector_3d(point_cloud.points[i], radius)

        # Connect the current point to its neighbors with lines
        for j in range(1, len(idx)):
            edges.append([i, idx[j]])

    # Create a LineSet for visualization
    line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(np.asarray(point_cloud.points)),
        lines=o3d.utility.Vector2iVector(edges),
    )
    return edges, line_set

edges, line_set = calcuate_edges_and_line_set(point_cloud)


print(edges, curvature)

# Display the results (visualization)
o3d.visualization.draw_geometries([point_cloud])
