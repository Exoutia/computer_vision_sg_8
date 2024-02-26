import open3d as o3d
import numpy as np

# Load a point cloud from a file
point_cloud = o3d.io.read_point_cloud("./ant.ply")

# Estimate normals for the point cloud
point_cloud.estimate_normals()

# Create a KDTree for efficient nearest neighbor search
kdtree = o3d.geometry.KDTreeFlann(point_cloud)

# Set the radius for edge computation
radius = 0.02

# List to store lines (edges)
lines = []

# Iterate through each point in the point cloud
for i in range(len(point_cloud.points)):
    # Find neighbors within the specified radius
    [k, idx, _] = kdtree.search_radius_vector_3d(point_cloud.points[i], radius)
    
    # Connect the current point to its neighbors with lines
    for j in range(1, len(idx)):
        lines.append([i, idx[j]])

# Create a LineSet for visualization
line_set = o3d.geometry.LineSet(
    points=o3d.utility.Vector3dVector(np.asarray(point_cloud.points)),
    lines=o3d.utility.Vector2iVector(lines),
)

# Display the point cloud and edges
print(lines)
