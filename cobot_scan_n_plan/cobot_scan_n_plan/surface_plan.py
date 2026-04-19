#!/usr/bin/env python

###############################################
"""
 @maintainer: Azeez Adebayo
 @about: ROS1 mcfly_scan_n_plan package
"""
###############################################

# v17

###################////////////////////////////////////////////////////////
# Restored filter_yellow_chain call with skip_count=1 and connectedness_radii=[0.0001] to minimize clustering.
# Retained IndexError fix from v16 in create_yellow_point_chain for robust path construction.
###################////////////////////////////////////////////////////////

import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
from scipy.interpolate import splprep, splev
from scipy.spatial.transform import Rotation as R
from sklearn.neighbors import NearestNeighbors
import time

VERBOSE: bool = False

# Set a fixed random seed for reproducibility
np.random.seed(50) 

# ----------------------
# frame viz
# ----------------------

def plot_coordinate_frame(ax, position, quaternion, scale=0.01):
    """
    Plot a tiny coordinate frame at the given position with the specified orientation.

    Args:
        ax: Matplotlib 3D axes object.
        position (np.ndarray): Position of the frame, shape (3,).
        quaternion (list): Quaternion [x, y, z, w] for orientation.
        scale (float): Length of the frame axes.
    """
    rotation = R.from_quat(quaternion)
    rotation_matrix = rotation.as_matrix()
    x_axis = rotation_matrix @ np.array([scale, 0, 0])
    y_axis = rotation_matrix @ np.array([0, scale, 0])
    z_axis = rotation_matrix @ np.array([0, 0, scale])
    origin = position
    ax.quiver(origin[0], origin[1], origin[2], x_axis[0], x_axis[1], x_axis[2], color='r', linewidth=1, label='X' if origin is position else '')
    ax.quiver(origin[0], origin[1], origin[2], y_axis[0], y_axis[1], y_axis[2], color='g', linewidth=1, label='Y' if origin is position else '')
    ax.quiver(origin[0], origin[1], origin[2], z_axis[0], z_axis[1], z_axis[2], color='b', linewidth=1, label='Z' if origin is position else '')
    ax.text(origin[0] + x_axis[0], origin[1] + x_axis[1], origin[2] + x_axis[2], 'X', color='r', fontsize=8)
    ax.text(origin[0] + y_axis[0], origin[1] + y_axis[1], origin[2] + y_axis[2], 'Y', color='g', fontsize=8)
    ax.text(origin[0] + z_axis[0], origin[1] + z_axis[1], origin[2] + z_axis[2], 'Z', color='b', fontsize=8)

# ----------------------
# 2d path gen.
# ----------------------

def generate_path(x1, y1, x2, y2, spacing, path_type="boustrophedon", grid_x_offset=0.0, grid_y_offset=0.0):
    """
    Generate a 2D spiral or boustrophedon path over a rectangular grid defined by diagonal points,
    with optional offsets to expand the grid.

    Args:
        x1, y1, x2, y2 (float): Diagonal corners of the grid.
        spacing (float): Distance between parallel lines in the path.
        path_type (str): Type of path, either "spiral" or "boustrophedon".
        grid_x_offset (float): Offset to add to both left and right sides of the grid (default 0.0).
        grid_y_offset (float): Offset to add to both bottom and top sides of the grid (default 0.0).

    Returns:
        list: List of 2D points [x, y] defining the path.
    """
    if spacing <= 0:
        raise ValueError("[generate_path] Spacing must be positive.")
    if path_type not in ["spiral", "boustrophedon"]:
        raise ValueError("[generate_path] path_type must be 'spiral' or 'boustrophedon'.")
    if grid_x_offset < 0 or grid_y_offset < 0:
        raise ValueError("[generate_path] Grid offsets must be non-negative.")
    left = min(x1, x2) - grid_x_offset
    right = max(x1, x2) + grid_x_offset
    bottom = min(y1, y2) - grid_y_offset
    top = max(y1, y2) + grid_y_offset
    if right <= left or top <= bottom:
        raise ValueError("[generate_path] Invalid rectangle: x1, x2 and y1, y2 with offsets must define a non-zero area.")
    width = right - left
    height = top - bottom
    path = []
    EPSILON = 1e-6
    if VERBOSE:
        print(f"[generate_path] Debug: Grid with offsets: left={left:.3f}, right={right:.3f}, bottom={bottom:.3f}, top={top:.3f}")
        print(f"[generate_path] Debug: Grid dimensions: width={width:.3f}, height={height:.3f}")
    if path_type == "spiral":
        current_left, current_right, current_bottom, current_top = left, right, bottom, top
        if width < spacing or height < spacing:
            if VERBOSE:
                print(f"[generate_path] Warning: Grid too small for spacing={spacing}. Generating center point.")
            path.append([(left + right) / 2, (bottom + top) / 2])
        else:
            while current_left + spacing < current_right and current_bottom + spacing < current_top:
                for px in np.arange(current_left, current_right + EPSILON, spacing):
                    path.append([px, current_top])
                current_top -= spacing
                if current_top <= current_bottom:
                    break
                for py in np.arange(current_top, current_bottom - EPSILON, -spacing):
                    path.append([current_right, py])
                current_right -= spacing
                if current_right <= current_left:
                    break
                for px in np.arange(current_right, current_left - EPSILON, -spacing):
                    path.append([px, current_bottom])
                current_bottom += spacing
                if current_bottom >= current_top:
                    break
                for py in np.arange(current_bottom, current_top + EPSILON, spacing):
                    path.append([current_left, py])
                current_left += spacing
                if current_left >= current_right:
                    break
            if current_left <= current_right and current_bottom <= current_top:
                path.append([(current_left + current_right) / 2, (bottom + top) / 2])
    else:  # boustrophedon
        num_lines = max(1, int(np.ceil(height / spacing)))
        for i in range(num_lines):
            py = min(bottom + i * spacing, top)
            if i % 2 == 0:
                for px in np.arange(left, right + EPSILON, max(spacing, width / 10)):
                    path.append([px, py])
            else:
                for px in np.arange(right, left - EPSILON, -max(spacing, width / 10)):
                    path.append([px, py])
            if i < num_lines - 1 and path:
                next_y = min(bottom + (i + 1) * spacing, top)
                path.append([path[-1][0], next_y])
    if VERBOSE:
        print(f"[generate_path] Debug: Generated {path_type} path with {len(path)} points")
    return path

# ----------------------
# kdtree
# ----------------------

def compute_kdtree(points):
    """Build a KDTree from a point cloud."""
    return KDTree(points)

# ----------------------
# PC downsample
# ----------------------

def voxel_downsample(points, voxel_size):
    """
    Downsample a point cloud by averaging points within each voxel.

    Args:
        points (np.ndarray): Point cloud, shape (N, 3).
        voxel_size (float): Size of the voxel grid.

    Returns:
        np.ndarray: Downsampled points, shape (M, 3).
    """
    indices = np.floor(points / voxel_size).astype(int)
    voxel_dict = {}
    for index, point in zip(indices, points):
        index_t = tuple(index)
        voxel_dict.setdefault(index_t, []).append(point)
    centroids = [np.mean(voxel_points, axis=0) for voxel_points in voxel_dict.values()]
    return np.array(centroids)

# ----------------------
# frame orientation
# ----------------------

def compute_orientation_surface_normal(point, prev_point, next_point, ds_obj_points, is_first=False,
    is_last=False, camera_pos=None, k=30, angle_lower_limit_deg=0.0, angle_upper_limit_deg=180.0
):
    """
    Compute orientation quaternion with Z-axis aligned to the local surface normal.
    Adjusts normals outside [angle_lower_limit_deg, angle_upper_limit_deg] relative to global +Z.

    Args:
        point (np.ndarray): Current 3D point [x, y, z].
        prev_point, next_point: Previous/next points (may be None).
        ds_obj_points (np.ndarray): Downsampled point cloud, shape (N, 3).
        is_first (bool): Whether this is the first point.
        is_last (bool): Whether this is the last point.
        camera_pos (np.ndarray or None): Camera position for outward-facing normals.
        k (int): Number of neighbors for PCA.
        angle_lower_limit_deg (float): Minimum angle with global +Z.
        angle_upper_limit_deg (float): Maximum angle with global +Z.

    Returns:
        list: Quaternion [x, y, z, w].
    """
    point = np.asarray(point, dtype=float)
    if len(ds_obj_points) < k:
        k = len(ds_obj_points)
    nbrs = NearestNeighbors(n_neighbors=k).fit(ds_obj_points)
    _, idx = nbrs.kneighbors([point])
    local_pts = ds_obj_points[idx[0]]
    local_pts -= np.mean(local_pts, axis=0)
    cov = np.cov(local_pts.T)
    eigvals, eigvecs = np.linalg.eigh(cov)
    normal = eigvecs[:, 0]
    normal /= np.linalg.norm(normal)
    if camera_pos is not None and np.dot(normal, camera_pos - point) < 0:
        normal = -normal
    z_ref = np.array([0.0, 0.0, 1.0])
    dot = np.clip(np.dot(normal, z_ref), -1.0, 1.0)
    angle_deg = np.degrees(np.arccos(dot))
    if angle_deg < angle_lower_limit_deg:
        angle_rad = np.radians(angle_lower_limit_deg)
        axis = np.cross(normal, z_ref)
        if np.linalg.norm(axis) < 1e-6:
            axis = np.array([1.0, 0.0, 0.0])
        axis /= np.linalg.norm(axis)
        R_rot = R.from_rotvec(angle_rad * axis)
        normal = R_rot.apply(z_ref)
    elif angle_deg > angle_upper_limit_deg:
        angle_rad = np.radians(angle_upper_limit_deg)
        axis = np.cross(normal, z_ref)
        if np.linalg.norm(axis) < 1e-6:
            axis = np.array([1.0, 0.0, 0.0])
        axis /= np.linalg.norm(axis)
        R_rot = R.from_rotvec(angle_rad * axis)
        normal = R_rot.apply(z_ref)
    z_axis = normal
    arbitrary = np.array([1.0, 0.0, 0.0])
    if np.abs(np.dot(z_axis, arbitrary)) > 0.99:
        arbitrary = np.array([0.0, 1.0, 0.0])
    x_axis = np.cross(z_axis, arbitrary)
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    Rm = np.column_stack([x_axis, y_axis, z_axis])
    quat = R.from_matrix(Rm).as_quat()
    return quat.tolist()

# ----------------------
# 3d projection
# ----------------------

def generate_3d_path_from_pointcloud(ds_obj_points, spacing=None, path_type="boustrophedon", max_search_length=0.2, neighbourhood_radius=0.05, grid_x_offset=0.0, grid_y_offset=0.0):
    """
    Generate a 3D spiral or boustrophedon path from a downsampled point cloud.
    Projects 2D path points to the closest point in a cylindrical region.

    Args:
        ds_obj_points (np.ndarray): Downsampled point cloud, shape (N, 3).
        spacing (float or None): Distance between parallel lines in the 2D path.
        path_type (str): Type of path, either "spiral" or "boustrophedon".
        max_search_length (float): Maximum Z distance for projection.
        neighbourhood_radius (float): XY radius for cylindrical search.
        grid_x_offset (float): Offset for grid expansion.
        grid_y_offset (float): Offset for grid expansion.

    Returns:
        tuple: (path_3d, orientations, projected_flags, path_2d, spacing)
    """
    if ds_obj_points.size == 0:
        raise ValueError("[generate_3d_path_from_pointcloud] Point cloud is empty.")
    if path_type not in ["spiral", "boustrophedon"]:
        raise ValueError("[generate_3d_path_from_pointcloud] path_type must be 'spiral' or 'boustrophedon'.")
    
    xy_points = ds_obj_points[:, :2]
    min_xy = np.min(xy_points, axis=0)
    max_xy = np.max(xy_points, axis=0)
    x1, y1 = min_xy
    x2, y2 = max_xy
    width = x2 - x1
    height = y2 - y1
    if VERBOSE:
        print(f"[generate_3d_path_from_pointcloud] Debug: Diagonal extremes: (x1, y1)=({x1:.3f}, {y1:.3f}), (x2, y2)=({x2:.3f}, {y2:.3f})")
        print(f"[generate_3d_path_from_pointcloud] Debug: Grid dimensions (before offsets): width={width:.3f}, height={height:.3f}")
    if spacing is None:
        spacing = min(width, height) * 0.1
        if VERBOSE:
            print(f"[generate_3d_path_from_pointcloud] Debug: Auto-set spacing to {spacing:.3f}")
    if spacing <= 0:
        raise ValueError("[generate_3d_path_from_pointcloud] Spacing must be positive.")
    path_2d = generate_path(x1, y1, x2, y2, spacing, path_type, grid_x_offset, grid_y_offset)
    path_2d = np.array(path_2d)
    if VERBOSE:
        print(f"[generate_3d_path_from_pointcloud] Debug: 2D path length: {len(path_2d)} points")
    tree = compute_kdtree(ds_obj_points)
    _, idx1 = tree.query([x1, y1, ds_obj_points[:, 2].mean()])
    _, idx2 = tree.query([x2, y2, ds_obj_points[:, 2].mean()])
    p1_3d = ds_obj_points[idx1]
    p2_3d = ds_obj_points[idx2]
    if VERBOSE:
        print(f"[generate_3d_path_from_pointcloud] Debug: 3D extreme points: p1={p1_3d}, p2={p2_3d}")
    path_3d = []
    orientations = []
    projected_flags = []
    default_z = np.min(ds_obj_points[:, 2])
    for point_2d in path_2d:
        point_3d = np.array([point_2d[0], point_2d[1], default_z])
        projected = False
        distances, indices = tree.query([point_2d[0], point_2d[1], default_z], k=len(ds_obj_points), distance_upper_bound=neighbourhood_radius)
        valid_indices = indices[distances < neighbourhood_radius]
        if len(valid_indices) > 0:
            candidate_points = ds_obj_points[valid_indices]
            z_valid = (candidate_points[:, 2] >= default_z) & (candidate_points[:, 2] <= default_z + max_search_length)
            if np.any(z_valid):
                valid_candidates = candidate_points[z_valid]
                distances_3d = np.sqrt(np.sum((valid_candidates - [point_2d[0], point_2d[1], default_z])**2, axis=1))
                closest_idx = np.argmin(distances_3d)
                point_3d = valid_candidates[closest_idx]
                projected = True
                if VERBOSE:
                    print(f"[generate_3d_path_from_pointcloud] Debug: Projected point {point_2d} to {point_3d}")
            else:
                if VERBOSE:
                    print(f"[generate_3d_path_from_pointcloud] Debug: No point found for {point_2d} within Z range, using default Z={default_z}")
        else:
            if VERBOSE:
                print(f"[generate_3d_path_from_pointcloud] Debug: No point found for {point_2d} within XY radius, using default Z={default_z}")
        prev_idx = max(0, len(path_3d) - 1)
        next_idx = min(len(path_2d) - 1, len(path_3d) + 1)
        prev_point = path_3d[-1] if path_3d else None
        next_point = path_2d[next_idx] if next_idx < len(path_2d) else None
        if next_point is not None:
            next_point_3d = np.array([next_point[0], next_point[1], default_z])
            distances, indices = tree.query([next_point[0], next_point[1], default_z], k=len(ds_obj_points), distance_upper_bound=neighbourhood_radius)
            valid_indices = indices[distances < neighbourhood_radius]
            if len(valid_indices) > 0:
                candidate_points = ds_obj_points[valid_indices]
                z_valid = (candidate_points[:, 2] >= default_z) & (candidate_points[:, 2] <= default_z + max_search_length)
                if np.any(z_valid):
                    valid_candidates = candidate_points[z_valid]
                    distances_3d = np.sqrt(np.sum((valid_candidates - [next_point[0], point_2d[1], default_z])**2, axis=1))
                    closest_idx = np.argmin(distances_3d)
                    next_point_3d = valid_candidates[closest_idx]
        quat = compute_orientation_surface_normal(point_3d, prev_point, next_point_3d, ds_obj_points)
        path_3d.append(point_3d)
        orientations.append(quat)
        projected_flags.append(projected)
    return np.array(path_3d), orientations, projected_flags, path_2d, spacing

# ----------------------
# path chain gen.
# ----------------------

def create_yellow_point_chain(path_3d, path_2d, projected_flags, ds_obj_points, connectedness_radius=0.2, stretch_limit=0.5):
    """
    Create a continuous chain of yellow (projected) points by clustering points within connectedness_radius
    in 3D, then connecting clusters into a single path where no connection exceeds stretch_limit.
    Points are renumbered sequentially based on the new path order.

    Args:
        path_3d (np.ndarray): 3D path points, shape (M, 3).
        path_2d (np.ndarray): 2D path points, shape (M, 2).
        projected_flags (list): Boolean list indicating projected (True) or unprojected (False) points.
        ds_obj_points (np.ndarray): Downsampled point cloud, shape (N, 3).
        connectedness_radius (float): Radius for clustering points in 3D.
        stretch_limit (float): Maximum allowed distance for connections between points.

    Returns:
        tuple: (chained_path, chained_orientations, neighbor_list)
    """
    yellow_indices = [i for i, flag in enumerate(projected_flags) if flag]
    if len(yellow_indices) < 2:
        if VERBOSE:
            print("[create_yellow_point_chain] Warning: Fewer than 2 yellow points, returning empty chain.")
        return np.array([]), [], []
    
    yellow_path_3d = path_3d[yellow_indices]
    if VERBOSE:
        print(f"[create_yellow_point_chain] Debug: {len(yellow_path_3d)} yellow points extracted")
    
    # Step 1: Cluster yellow points greedily in original order using connectedness_radius
    points = yellow_path_3d.copy()
    used = np.zeros(len(points), dtype=bool)
    clusters = []
    for i in range(len(points)):
        if used[i]:
            continue
        cluster_indices = [i]
        used[i] = True
        for j in range(i + 1, len(points)):
            if not used[j] and np.linalg.norm(points[i] - points[j]) <= connectedness_radius:
                cluster_indices.append(j)
                used[j] = True
        cluster_points = points[cluster_indices]
        centroid = np.mean(cluster_points, axis=0)
        clusters.append(centroid)
    if VERBOSE:
        print(f"[create_yellow_point_chain] Debug: Clustered {len(yellow_path_3d)} yellow points into {len(clusters)} clusters with connectedness_radius={connectedness_radius}")

    chained_path = np.array(clusters)
    if len(chained_path) == 0:
        if VERBOSE:
            print("[create_yellow_point_chain] Warning: No clusters formed, returning empty chain.")
        return np.array([]), [], []

    # Step 2: Build a continuous path using a greedy nearest-neighbor approach
    tree = KDTree(chained_path)
    n_points = len(chained_path)
    visited = np.zeros(n_points, dtype=bool)
    new_order = [0]  # Start from the first cluster
    visited[0] = True
    current_idx = 0
    if VERBOSE:
        print(f"[create_yellow_point_chain] Debug: Starting continuous path construction from cluster {current_idx}")

    # Greedily connect to the nearest unvisited point within stretch_limit
    while len(new_order) < n_points:
        current_point = chained_path[current_idx]
        distances, indices = tree.query([current_point], k=n_points)
        found = False
        for j, dist in zip(indices[0], distances[0]):
            if j >= n_points:  # Skip invalid indices
                continue
            if not visited[j] and dist <= stretch_limit and j != current_idx:
                new_order.append(j)
                visited[j] = True
                current_idx = j
                if VERBOSE:
                    print(f"[create_yellow_point_chain] Debug: Connected cluster {new_order[-2]} to {j} (distance={dist:.3f})")
                found = True
                break
        if not found:
            if VERBOSE:
                print(f"[create_yellow_point_chain] Warning: No unvisited points within stretch_limit={stretch_limit} from cluster {current_idx}. Path may be incomplete.")
            break

    if len(new_order) < n_points:
        if VERBOSE:
            print(f"[create_yellow_point_chain] Error: Could not connect all {n_points} points into a continuous path. Connected {len(new_order)} points.")
        chained_path = chained_path[new_order]
    else:
        if VERBOSE:
            print(f"[create_yellow_point_chain] Debug: Successfully connected all {n_points} points into a continuous path.")
        chained_path = chained_path[new_order]

    # Step 3: Build new neighbor list based on the continuous path
    neighbor_list = []
    for i in range(len(chained_path)):
        neighbors = []
        if i > 0:
            neighbors.append(i - 1)  # Previous point
        if i < len(chained_path) - 1:
            neighbors.append(i + 1)  # Next point
        neighbor_list.append(neighbors)
    if VERBOSE:
        print(f"[create_yellow_point_chain] Debug: New neighbor list after renumbering: {neighbor_list}")

    # Step 4: Verify connectivity and stretch_limit
    for i in range(len(chained_path)):
        for n in neighbor_list[i]:
            distance = np.linalg.norm(chained_path[i] - chained_path[n])
            if distance > stretch_limit:
                if VERBOSE:
                    print(f"[create_yellow_point_chain] Warning: Connection between {i} and {n} exceeds stretch_limit ({distance:.3f} > {stretch_limit})")
        num_neighbors = len(neighbor_list[i])
        if i == 0 or i == len(chained_path) - 1:
            if num_neighbors > 1:
                if VERBOSE:
                    print(f"[create_yellow_point_chain] Warning: Cluster {i} ({'start' if i == 0 else 'end'}) has {num_neighbors} neighbors, expected at most 1")
        else:
            if num_neighbors > 2:
                if VERBOSE:
                    print(f"[create_yellow_point_chain] Warning: Cluster {i} has {num_neighbors} neighbors, expected at most 2")

    # Step 5: Compute orientations for the new path
    chained_orientations = []
    camera_pos = np.array([0, 0, 0])
    for i in range(len(chained_path)):
        is_first = (i == 0)
        is_last = (i == len(chained_path) - 1)
        prev_point = chained_path[neighbor_list[i][0]] if neighbor_list[i] else None
        next_point = chained_path[neighbor_list[i][1]] if len(neighbor_list[i]) > 1 else None
        quat = compute_orientation_surface_normal(
            chained_path[i], prev_point, next_point, ds_obj_points, is_first, is_last, camera_pos=camera_pos, k=30
        )
        chained_orientations.append(quat)

    if VERBOSE:
        print(f"[create_yellow_point_chain] Debug: Chained path length: {len(chained_path)} points")
    return chained_path, chained_orientations, neighbor_list

# ----------------------
# path filter
# ----------------------

def filter_yellow_chain(chained_path, chained_orientations, neighbor_list, skip_count):
    """
    Filter the yellow point chain by keeping every (skip_count + 1)-th point, including the first and last points.

    Args:
        chained_path (np.ndarray): Array of 3D points in the yellow chain, shape (K, 3).
        chained_orientations (list): List of quaternions [x, y, z, w] for each point.
        neighbor_list (list): List of neighbor indices for each point.
        skip_count (int): Number of points to skip between kept points.

    Returns:
        tuple: (filtered_path, filtered_orientations, filtered_neighbor_list)
    """
    if skip_count < 0:
        raise ValueError("[filter_yellow_chain] skip_count must be non-negative.")
    if len(chained_path) != len(chained_orientations) or len(chained_path) != len(neighbor_list):
        raise ValueError("[filter_yellow_chain] chained_path, chained_orientations, and neighbor_list must have the same length.")
    if len(chained_path) < 2:
        if VERBOSE:
            print(f"[filter_yellow_chain] Debug: Chain has {len(chained_path)} points, returning unchanged.")
        return np.array(chained_path), chained_orientations, neighbor_list
    if skip_count == 0:
        if VERBOSE:
            print(f"[filter_yellow_chain] Debug: skip_count=0, returning original chain with {len(chained_path)} points.")
        return np.array(chained_path), chained_orientations, neighbor_list
    indices = list(range(0, len(chained_path), skip_count + 1))
    if indices[-1] != len(chained_path) - 1:
        indices.append(len(chained_path) - 1)
    filtered_path = np.array(chained_path)[indices]
    filtered_orientations = [chained_orientations[i] for i in indices]
    # Rebuild neighbor_list for filtered points
    original_indices = {i: idx for i, idx in enumerate(indices)}
    filtered_neighbor_list = []
    for i, idx in enumerate(indices):
        old_neighbors = neighbor_list[idx]
        new_neighbors = []
        for n in old_neighbors:
            for j, orig_idx in enumerate(indices):
                if n == orig_idx:
                    new_neighbors.append(j)
        filtered_neighbor_list.append(new_neighbors)
    if VERBOSE:
        print(f"[filter_yellow_chain] Debug: Filtered chain from {len(chained_path)} to {len(filtered_path)} points with skip_count={skip_count}.")
    return filtered_path, filtered_orientations, filtered_neighbor_list

# ----------------------
# reconnect/merge path
# ----------------------

def smooth_path(path_3d, num_points=None):
    """
    Smooth a 3D path using spline interpolation with chord-length parameterization, with fallback for short or problematic paths.

    Args:
        path_3d (np.ndarray): Array of 3D points to smooth, shape (N, 3).
        num_points (int or None): Number of points in the smoothed path. If None, use input length.

    Returns:
        np.ndarray: Smoothed path, shape (M, 3).
    """
    if len(path_3d) < 2:
        if VERBOSE:
            print(f"[smooth_path] Debug: Path has {len(path_3d)} points, too few for smoothing. Returning unchanged.")
        return path_3d
    if num_points is None:
        num_points = len(path_3d)
    if len(path_3d) < 4:
        if VERBOSE:
            print(f"[smooth_path] Debug: Path has {len(path_3d)} points, too few for spline smoothing. Returning unchanged.")
        return path_3d

    try:
        # Compute chord-length parameterization
        distances = np.sqrt(np.sum(np.diff(path_3d, axis=0)**2, axis=1))
        t = np.zeros(len(path_3d))
        t[1:] = np.cumsum(distances)
        t /= t[-1]  # Normalize to [0, 1]
        t_new = np.linspace(0, 1, num_points)

        # Ensure points are unique
        unique_indices = np.unique(path_3d, axis=0, return_index=True)[1]
        if len(unique_indices) < 4:
            if VERBOSE:
                print(f"[smooth_path] Debug: Only {len(unique_indices)} unique points after clustering, skipping smoothing.")
            return path_3d
        path_3d_unique = path_3d[unique_indices]
        t_unique = t[unique_indices]

        # Check for small path variations
        path_range = np.ptp(path_3d_unique, axis=0)
        if np.any(path_range < 1e-6):
            if VERBOSE:
                print(f"[smooth_path] Debug: Path range too small ({path_range}), skipping smoothing to avoid numerical issues.")
            return path_3d

        # Perform spline interpolation with increased smoothing factor
        tck, _ = splprep([path_3d_unique[:, 0], path_3d_unique[:, 1], path_3d_unique[:, 2]], u=t_unique, s=1.0)
        smoothed = np.array(splev(t_new, tck)).T
        if VERBOSE:
            print(f"[smooth_path] Debug: Successfully smoothed path with {len(smoothed)} points.")
        return smoothed
    except Exception as e:
        if VERBOSE:
            print(f"[smooth_path] Warning: Spline smoothing failed with error: {e}. Falling back to linear interpolation.")
        # ---- Fallback: linear interpolation ----
        try:
            t = np.linspace(0, 1, len(path_3d))
            t_new = np.linspace(0, 1, num_points)
            smoothed = np.zeros((num_points, 3))
            for i in range(3):  # Interpolate each dimension
                smoothed[:, i] = np.interp(t_new, t, path_3d[:, i])
            if VERBOSE:
                print(f"[smooth_path] Debug: Linear interpolation succeeded with {len(smoothed)} points.")
            return smoothed
        except Exception as e2:
            if VERBOSE:
                print(f"[smooth_path] Warning: Linear interpolation also failed with error: {e2}. Returning original path.")
            return path_3d

# ----------------------
# Visualization
# ----------------------

def visualization(ds_obj_points, path_3d, path_2d, projected_flags, chained_path, chained_orientations, path_type, connectedness_radius, stretch_limit, skip_count, show_frames):
    """
    Visualize 2D/3D paths and clustered yellow chains for the specified path type.

    Args:
        ds_obj_points (np.ndarray): Downsampled point cloud, shape (N, 3).
        path_3d (np.ndarray): 3D path points, shape (M, 3).
        path_2d (np.ndarray): 2D path points, shape (M, 2).
        projected_flags (list): Boolean list indicating projected (True) or unprojected (False) points.
        chained_path (np.ndarray): Filtered chained path, shape (K, 3).
        chained_orientations (list): Orientations for chained path points, shape (K, 4).
        path_type (str): Type of path, either "spiral" or "boustrophedon".
        connectedness_radius (float): Radius for clustering points.
        stretch_limit (float): Maximum allowed distance for connections.
        skip_count (int): Number of points to skip in filtering.
        show_frames (bool): Whether to show coordinate frames for orientations.
    """
    min_coords = np.min(ds_obj_points, axis=0)
    max_coords = np.max(ds_obj_points, axis=0)
    max_range = np.max(max_coords - min_coords)
    padding = max_range * 0.1
    frame_scale = max_range * 0.02

    # Initialize figures
    fig1 = plt.figure(figsize=(12, 5))  # 2D and 3D paths
    fig2 = plt.figure(figsize=(12, 5))  # Clustered and filtered chain

    # 3D Path Plot
    ax_3d = fig1.add_subplot(121, projection='3d')
    ax_3d.scatter(ds_obj_points[:, 0], ds_obj_points[:, 1], ds_obj_points[:, 2], c='gray', s=10, alpha=0.5, label='Point Cloud')
    ax_3d.plot(path_3d[:, 0], path_3d[:, 1], path_3d[:, 2], 'b-', linewidth=2, label=f'{path_type.capitalize()} Path')
    projected_indices = [j for j, flag in enumerate(projected_flags) if flag]
    unprojected_indices = [j for j, flag in enumerate(projected_flags) if not flag]
    if projected_indices:
        ax_3d.scatter(path_3d[projected_indices, 0], path_3d[projected_indices, 1], path_3d[projected_indices, 2],
                     c='yellow', s=50, label='Projected Points')
    if unprojected_indices:
        ax_3d.scatter(path_3d[unprojected_indices, 0], path_3d[unprojected_indices, 1], path_3d[unprojected_indices, 2],
                     c='red', s=50, label='Unprojected Points')
    ax_3d.set_xlim(min_coords[0] - padding, max_coords[0] + padding)
    ax_3d.set_ylim(min_coords[1] - padding, max_coords[1] + padding)
    ax_3d.set_zlim(min_coords[2] - padding, max_coords[2] + padding)
    ax_3d.set_box_aspect([1, 1, 1])
    ax_3d.set_xlabel('X')
    ax_3d.set_ylabel('Y')
    ax_3d.set_zlabel('Z')
    ax_3d.set_title(f'{path_type.capitalize()} 3D Path')
    ax_3d.legend()

    # 2D Path Plot
    ax_2d = fig1.add_subplot(122, projection='3d')
    ax_2d.plot(path_2d[:, 0], path_2d[:, 1], np.full_like(path_2d[:, 0], min_coords[2] - padding), 'b-', linewidth=2, label=f'{path_type.capitalize()} Path')
    ax_2d.scatter(path_2d[:, 0], path_2d[:, 1], np.full_like(path_2d[:, 0], min_coords[2] - padding), c=['yellow' if flag else 'red' for flag in projected_flags], s=50, label='Path Points')
    ax_2d.scatter(ds_obj_points[:, 0], ds_obj_points[:, 1], np.full_like(ds_obj_points[:, 0], min_coords[2] - padding), c='gray', s=10, alpha=0.5, label='Point Cloud XY')
    ax_2d.set_xlim(min_coords[0] - padding, max_coords[0] + padding)
    ax_2d.set_ylim(min_coords[1] - padding, max_coords[1] + padding)
    ax_2d.set_zlim(min_coords[2] - padding, max_coords[2] + padding)
    ax_2d.set_box_aspect([1, 1, 1])
    ax_2d.set_xlabel('X')
    ax_2d.set_ylabel('Y')
    ax_2d.set_zlabel('Z')
    ax_2d.set_title(f'{path_type.capitalize()} 2D Path (Projected to Z={min_coords[2] - padding:.3f})')
    ax_2d.legend()

    # Chained Path Plot
    ax_chained = fig2.add_subplot(111, projection='3d')
    ax_chained.scatter(ds_obj_points[:, 0], ds_obj_points[:, 1], ds_obj_points[:, 2], c='gray', s=10, alpha=0.5, label='Point Cloud')
    if len(chained_path) > 0:
        # chained_path = smooth_path(chained_path, num_points=len(chained_path))
        ax_chained.plot(chained_path[:, 0], chained_path[:, 1], chained_path[:, 2], 'g-', linewidth=2, label='Continuous Path')
        ax_chained.scatter(chained_path[:, 0], chained_path[:, 1], chained_path[:, 2], c='yellow', s=50, label='Clustered Points')
        for j, point in enumerate(chained_path):
            ax_chained.text(point[0], point[1], point[2], str(j), color='black', fontsize=8,
                           bbox=dict(facecolor='white', alpha=0.8, edgecolor='none', boxstyle='round,pad=0.2'))
        if show_frames:
            for j, (point, quat) in enumerate(zip(chained_path, chained_orientations)):
                plot_coordinate_frame(ax_chained, point, quat, scale=frame_scale)
    ax_chained.set_xlim(min_coords[0] - padding, max_coords[0] + padding)
    ax_chained.set_ylim(min_coords[1] - padding, max_coords[1] + padding)
    ax_chained.set_zlim(min_coords[2] - padding, max_coords[2] + padding)
    ax_chained.set_box_aspect([1, 1, 1])
    ax_chained.set_xlabel('X')
    ax_chained.set_ylabel('Y')
    ax_chained.set_zlabel('Z')
    ax_chained.set_title(f'{path_type.capitalize()} Continuous Chain (radius={connectedness_radius}, stretch_limit={stretch_limit}, skip_count={skip_count}, frames={show_frames})')
    handles, labels = ax_chained.get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    ax_chained.legend(unique.values(), unique.keys())

    # Finalize plots
    fig1.tight_layout()
    fig2.tight_layout()
    plt.show()

# ----------------------
# Plan
# ----------------------

def plan(ply_path, 
         dist_threshold=0.55,  # 2.0 --> red3
         downsample_voxel_size=0.015,  # 0.01 --> red3 
         path_choice=1,  # ["spiral", "boustrophedon"]
         max_search_length=0.6,  # max travel distance to consider projection hit
         neighbourhood_radius=0.2,  # radius to find/consider projection hit
         grid_x_offset=0.03,  # offset by which to extend grid size in x
         grid_y_offset=0.03,  # offset by which to extend grid size in y
         connectedness_radius=0.0001,  # radius of ball around each point to determine the collapsibility of other points
         stretch_limit=0.2,  # max length to consider next path a neighbor
         skip_count=1,  # Filter to keep every n steps to reduce point
         spacing=None,  # none implies algorithm estimates by itself
         show_frames=True,  # Show coordinate frames for orientations in visualization
         show_visualization=True):   # plots are saved if true, Display visualizations is controlled within function
    """
    Generate a 3D path from a point cloud with a continuous chain of clustered and filtered points.
    Optionally visualize 2D/3D paths and clustered yellow chains.
    Excludes points near the camera origin and applies filter_yellow_chain to reduce points while preserving path continuity.

    Args:
        ply_path (str): Path to the input point cloud (.ply file).
        dist_threshold (float): Distance threshold for filtering points.
        downsample_voxel_size (float): Voxel size for downsampling the point cloud.
        path_choice (int): 0 for "spiral", 1 for "boustrophedon".
        max_search_length (float): Maximum Z distance for projection.
        neighbourhood_radius (float): XY radius for cylindrical search.
        grid_x_offset (float): Offset for grid expansion in x.
        grid_y_offset (float): Offset for grid expansion in y.
        connectedness_radius (float): Radius for clustering points in 3D.
        stretch_limit (float): Maximum allowed distance for connections.
        skip_count (int): Number of points to skip in filtering.
        spacing (float or None): Distance between parallel lines in the 2D path.
        show_frames (bool): Whether to show coordinate frames in visualization.
        show_visualization (bool): Whether to display visualizations.
        VERBOSE (bool): Whether to print debug information.

    Returns:
        tuple: (chained_path, chained_orientations)
            - chained_path (np.ndarray): Filtered and smoothed 3D path points, shape (K, 3).
            - chained_orientations (list): Corresponding orientations [x, y, z, w], length K.
    """
    t0 = time.time()
    print("[plan] Start time:", time.strftime("%H:%M:%S", time.localtime(t0)))

    # Load point cloud
    pcd = o3d.io.read_point_cloud(ply_path)
    init_obj_points = np.asarray(pcd.points)
    if VERBOSE:
        print(f"[plan] Initial point cloud shape: {init_obj_points.shape}")

    # Filter points near camera origin
    camera_threshold = 1e-3
    camera_distances = np.linalg.norm(init_obj_points, axis=1)
    camera_mask = camera_distances > camera_threshold
    camera_filtered_points = init_obj_points[camera_mask]
    if VERBOSE:
        print(f"[plan] Camera-filtered point cloud shape: {camera_filtered_points.shape} (removed {init_obj_points.shape[0] - camera_filtered_points.shape[0]} points near origin)")
    if camera_filtered_points.shape[0] == 0:
        raise ValueError("[plan] Error: No points remain after camera position filtering.")

    # Config Parameters
    path_types = [["spiral", "boustrophedon"][path_choice]]

    # Threshold pointcloud based on distance to camera
    distances = np.sqrt(np.sum(camera_filtered_points**2, axis=1))
    max_distance = np.max(distances)
    normalized_distances = distances / max_distance
    mask = normalized_distances < dist_threshold
    filtered_points = camera_filtered_points[mask]
    if VERBOSE:
        print(f"[plan] Distance-filtered point cloud shape: {filtered_points.shape}")

    # Downsample point cloud
    ds_obj_points = voxel_downsample(filtered_points, downsample_voxel_size)
    if VERBOSE:
        print(f"[plan] Downsampled point cloud shape: {ds_obj_points.shape}")
        print(f"[plan] Point cloud ranges: X={np.ptp(ds_obj_points[:, 0]):.3f}, Y={np.ptp(ds_obj_points[:, 1]):.3f}, Z={np.ptp(ds_obj_points[:, 2]):.3f}")

    # Process path type
    for path_type in path_types:
        # Generate 3D path
        path_3d, _, projected_flags, path_2d, spacing_used = generate_3d_path_from_pointcloud(
            ds_obj_points, spacing, path_type, max_search_length, neighbourhood_radius, grid_x_offset, grid_y_offset
        )
        if VERBOSE:
            print(f"[plan] Debug: {path_type} path: spacing_used={spacing_used}, {sum(projected_flags)} projected points, {len(projected_flags) - sum(projected_flags)} unprojected points")

        # Create continuous chain
        chained_path, chained_orientations, neighbor_list = create_yellow_point_chain(
            path_3d, path_2d, projected_flags, ds_obj_points, connectedness_radius, stretch_limit
        )
        # Filter the chain with skip_count
        chained_path, chained_orientations, neighbor_list = filter_yellow_chain(
            chained_path, chained_orientations, neighbor_list, skip_count
        )
        if VERBOSE:
            print(f"[plan] Debug: {path_type} chained path (connectedness_radius={connectedness_radius}, stretch_limit={stretch_limit}) length: {len(chained_path)} points")
            # print(f"[plan] Debug: {path_type} neighbor list (connectedness_radius={connectedness_radius}, stretch_limit={stretch_limit}): {neighbor_list}")

        # smooth final filtered path for consistency
        chained_path = smooth_path(chained_path, num_points=len(chained_path))
        if VERBOSE:
            print(f"[plan] Debug: {path_type} chained path smoothed")
                
        # Visualize if requested
        if show_visualization:
            visualization(ds_obj_points, path_3d, path_2d, projected_flags, chained_path, chained_orientations,
                          path_type, connectedness_radius, stretch_limit, skip_count, show_frames)

    # After processing and visualization
    t1 = time.time()
    duration = t1 - t0
    print(f"[plan] End time: {time.strftime('%H:%M:%S', time.localtime(t1))}")
    print(f"[plan] Duration: {duration:.2f} seconds \n")

    return chained_path, chained_orientations

# ----------------------
# main
# ----------------------

if __name__ == "__main__":
    pkg_path = r"C:\Users\MCFLY\env_ideahub\ideahub\mcfly\mcfly_ros1_s_n_p\mcfly_scan_n_plan"
    ply_path = pkg_path + "/config/surface.ply"  # red3.ply

    # Config Parameters
    path, orientations = plan(
        ply_path,
        dist_threshold=0.55,  # 2.0 --> red3
        downsample_voxel_size=0.015,  # 0.01 --> red3 
        path_choice=0,  # ["spiral", "boustrophedon"]
        max_search_length=0.6,  # max travel distance to consider projection hit
        neighbourhood_radius=0.2,  # radius to find/consider projection hit
        grid_x_offset=0.03,  # offset by which to extend grid size in x
        grid_y_offset=0.03,  # offset by which to extend grid size in y
        connectedness_radius=0.0001,  # radius of ball around each point to determine the collapsibility of other points
        stretch_limit=0.2,  # max length to consider next path a neighbor
        skip_count=1,  # Filter to keep every n steps to reduce point
        spacing=None,  # none implies algorithm estimates by itself
        show_frames=True,  # Show coordinate frames for orientations in visualization
        show_visualization=True  # plots are saved if true, Display visualizations is controlled within function
    )

    # Assertions: Ensure path and orientations are not empty and have matching lengths
    assert path is not None and len(path) > 0, "Path is empty"
    assert orientations is not None and len(orientations) > 0, "Orientations are empty"
    assert len(path) == len(orientations), f"Length mismatch: path={len(path)}, orientations={len(orientations)}"

    # validate output
    if len(orientations) > 3:
        print("first 3 path: ", path[:3]) 
        # sample coordinate output x y z:
        #  [ [-0.28637828 -0.21336537  0.49535714]
        #    [-0.29240263 -0.18743184  0.48682351]
        #    [-0.2324327  -0.22300702  0.49597266] ... ]
        print("first 3 orientations: ", orientations[:3])
        # sample orientation output q1 q2 q3 q4:
        #  [ [-0.6869060151751537, 0.7012878601872029, -0.09052810146056349, -0.16780979207481128], 
        #    [-0.677371573183293, 0.7031382268039938, -0.07481065436777896, -0.2028983781186799], 
        #    [-0.6903639963498369, 0.694768816377791, -0.13151536712112163, -0.15296258543801602] ...]

