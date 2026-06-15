#!/usr/bin/env python3
"""
Map Preprocessor: RANSAC ground alignment + wall-based XY axis correction.

Usage:
  python map_preprocessor.py <input.pcd> <output.pcd> [options]

Options:
  --visualize    Show Open3D 3D preview
  --no-align     Skip coordinate alignment (default: alignment ON)
  --infill       Enable multi-floor ground infill (default: OFF)
  --voxel_size N Voxel downsampling grid size (m), default: disabled

Steps (when alignment enabled):
  1. RANSAC extracts ground plane → compute rotation to align ground with XOY
  2. Detect dominant vertical planes (walls) → rotate around Z to align XY with walls
  3. Translate so ground center = origin
  4. (optional) Ground infill → fill sparse floor gaps
  5. Save result
"""

import argparse
import sys
import numpy as np
import open3d as o3d
from scipy import ndimage


def ransac_ground_plane(pcd, distance_threshold=0.15, num_iterations=2000):
    """Extract ground plane using RANSAC. Returns (plane_model, inlier_indices)."""
    plane_model, inliers = pcd.segment_plane(
        distance_threshold=distance_threshold,
        ransac_n=3,
        num_iterations=num_iterations
    )
    return plane_model, inliers


def compute_gravity_alignment(plane_normal):
    """
    Compute rotation matrix that aligns plane_normal with +Z axis.
    plane_normal: the ground plane normal (should roughly point up).
    """
    n = np.array(plane_normal, dtype=np.float64)
    n = n / np.linalg.norm(n)

    # Ensure normal points "up" (positive Z component)
    if n[2] < 0:
        n = -n

    target = np.array([0.0, 0.0, 1.0])

    # Rotation axis = cross product, angle = arccos(dot)
    v = np.cross(n, target)
    s = np.linalg.norm(v)
    c = np.dot(n, target)

    if s < 1e-8:
        # Already aligned
        return np.eye(3)

    # Skew-symmetric matrix
    vx = np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])

    R = np.eye(3) + vx + vx @ vx * ((1 - c) / (s * s))
    return R


def find_wall_direction_from_normals(pcd, z_threshold=0.3):
    """
    Estimate dominant wall direction using point normal estimation.
    More robust than RANSAC plane extraction for sparse wall points.

    Returns dominant angle (mod pi/2) of wall normals in XY plane, or None.
    """
    # Estimate normals
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=30))
    normals = np.asarray(pcd.normals)

    # Select points with mostly-horizontal normals (wall-like surfaces)
    horizontal_mask = np.abs(normals[:, 2]) < z_threshold
    h_normals = normals[horizontal_mask]

    if len(h_normals) < 100:
        return None

    # Project to XY and normalize
    n_xy = h_normals[:, :2]
    n_xy_mag = np.linalg.norm(n_xy, axis=1)
    valid = n_xy_mag > 0.5
    n_xy = n_xy[valid]
    n_xy = n_xy / np.linalg.norm(n_xy, axis=1, keepdims=True)

    if len(n_xy) < 50:
        return None

    # Compute angles mod pi/2 (walls come in perpendicular pairs)
    angles = np.arctan2(n_xy[:, 1], n_xy[:, 0]) % (np.pi / 2)

    # Histogram to find peak
    n_bins = 90
    hist, edges = np.histogram(angles, bins=n_bins, range=(0, np.pi / 2))

    # Smooth histogram with wrap-around
    kernel = np.array([0.1, 0.2, 0.4, 0.2, 0.1])
    hist_ext = np.concatenate([hist[-2:], hist, hist[:2]])
    hist_smooth = np.convolve(hist_ext, kernel, mode='same')[2:-2]

    peak_bin = np.argmax(hist_smooth)
    peak_angle = (edges[peak_bin] + edges[peak_bin + 1]) / 2

    # Confidence: peak should be significantly above average
    mean_count = hist_smooth.mean()
    peak_count = hist_smooth[peak_bin]
    confidence = peak_count / max(mean_count, 1)

    print(f"    Horizontal-normal points: {len(n_xy)}")
    print(f"    Peak angle (mod 90): {np.degrees(peak_angle):.2f} deg")
    print(f"    Peak strength: {peak_count:.0f} (avg={mean_count:.0f}, ratio={confidence:.1f}x)")

    if confidence < 1.5:
        print(f"    Low confidence ({confidence:.1f}x), skipping XY alignment")
        return None

    return peak_angle



def compute_yaw_rotation(yaw_angle):
    """Rotation matrix around Z axis by -yaw_angle (to align walls with axes)."""
    c = np.cos(-yaw_angle)
    s = np.sin(-yaw_angle)
    R = np.array([
        [c, -s, 0],
        [s, c, 0],
        [0, 0, 1]
    ])
    return R


def floor_infill(points, resolution=0.2, min_floor_density=0.15, neighbor_threshold=3):
    """
    Detect floor layers and fill XY gaps via morphological closing.
    Compatible with multi-story buildings.

    Algorithm:
      1. Discretize all points into Z slices (1 voxel thick)
      2. For each Z slice, compute XY occupancy grid
      3. If a slice has high XY density (floor-like), apply morphological closing
      4. New occupied cells become infill points

    Args:
        points: Nx3 numpy array
        resolution: voxel size (meters)
        min_floor_density: minimum ratio of occupied cells to consider a layer as floor
        neighbor_threshold: minimum occupied neighbors (out of 8) to fill a gap cell

    Returns:
        infill_points: Mx3 numpy array of new points to add
    """
    if len(points) == 0:
        return np.empty((0, 3))

    res = resolution
    # Discretize to grid indices
    ix = np.floor(points[:, 0] / res).astype(np.int32)
    iy = np.floor(points[:, 1] / res).astype(np.int32)
    iz = np.floor(points[:, 2] / res).astype(np.int32)

    ix_min, ix_max = ix.min(), ix.max()
    iy_min, iy_max = iy.min(), iy.max()
    iz_min, iz_max = iz.min(), iz.max()

    nx = ix_max - ix_min + 1
    ny = iy_max - iy_min + 1

    # Shift to 0-based
    ix_shifted = ix - ix_min
    iy_shifted = iy - iy_min
    iz_shifted = iz - iz_min

    # Group points by Z layer
    z_layers = {}
    for i in range(len(points)):
        zk = iz_shifted[i]
        if zk not in z_layers:
            z_layers[zk] = []
        z_layers[zk].append((ix_shifted[i], iy_shifted[i]))

    infill_list = []
    floors_found = 0

    for zk, cells in z_layers.items():
        # Build 2D occupancy grid for this Z layer
        grid = np.zeros((nx, ny), dtype=np.uint8)
        for (cx, cy) in cells:
            grid[cx, cy] = 1

        # Check if this is a floor-like layer
        occupied_count = grid.sum()
        total_possible = nx * ny
        density = occupied_count / total_possible

        if density < min_floor_density:
            continue

        floors_found += 1

        # Morphological closing: dilate then erode to fill small gaps
        # Use a 3x3 structuring element
        struct = ndimage.generate_binary_structure(2, 2)  # 8-connectivity
        closed = ndimage.binary_closing(grid, structure=struct, iterations=2)

        # Also: fill cells that have >= neighbor_threshold occupied neighbors
        # Count neighbors for each empty cell
        neighbor_count = ndimage.convolve(grid.astype(np.int32),
                                          np.ones((3, 3), dtype=np.int32),
                                          mode='constant', cval=0)
        # Fill where enough neighbors exist (excluding already-occupied)
        fill_by_neighbors = (neighbor_count >= neighbor_threshold) & (grid == 0)

        # Combine both methods
        new_cells = (closed | fill_by_neighbors) & (grid == 0)

        # Convert new cells back to world coordinates
        new_ix, new_iy = np.where(new_cells)
        if len(new_ix) == 0:
            continue

        world_x = (new_ix + ix_min + 0.5) * res
        world_y = (new_iy + iy_min + 0.5) * res
        world_z = np.full(len(new_ix), (zk + iz_min + 0.5) * res)

        infill_list.append(np.column_stack([world_x, world_y, world_z]))

    if floors_found > 0:
        print(f"    Detected {floors_found} floor-like layers")

    if infill_list:
        return np.vstack(infill_list)
    return np.empty((0, 3))


def voxel_down_sample_with_min_points(pcd, voxel_size, min_points=1):
    """
    Custom voxel downsampling that discards voxels with fewer than min_points.

    Args:
        pcd: Open3D PointCloud
        voxel_size: voxel grid size (m)
        min_points: keep only voxels with >= this many points (default 1 = all)

    Returns:
        filtered Open3D PointCloud
    """
    points = np.asarray(pcd.points)
    if len(points) == 0:
        return pcd

    res = voxel_size
    ix = np.floor(points[:, 0] / res).astype(np.int64)
    iy = np.floor(points[:, 1] / res).astype(np.int64)
    iz = np.floor(points[:, 2] / res).astype(np.int64)

    # Group points by voxel index
    from collections import defaultdict
    voxels = defaultdict(list)
    for i in range(len(points)):
        voxels[(ix[i], iy[i], iz[i])].append(i)

    # Only keep voxels with enough points, output centroid
    out_points = []
    for voxel_idx, pt_indices in voxels.items():
        if len(pt_indices) < min_points:
            continue
        centroid = points[pt_indices].mean(axis=0)
        out_points.append(centroid)

    out_pcd = o3d.geometry.PointCloud()
    out_pcd.points = o3d.utility.Vector3dVector(np.array(out_points))
    return out_pcd


def preprocess_map(input_path, output_path, visualize=False,
                   do_align=True, do_infill=False, voxel_size=None,
                   voxel_min_points=1):
    """Full preprocessing pipeline."""
    print(f"Loading: {input_path}")
    pcd = o3d.io.read_point_cloud(input_path)
    points = np.asarray(pcd.points)
    print(f"  Points: {len(points)}")
    print(f"  Bounds: X[{points[:,0].min():.2f}, {points[:,0].max():.2f}] "
          f"Y[{points[:,1].min():.2f}, {points[:,1].max():.2f}] "
          f"Z[{points[:,2].min():.2f}, {points[:,2].max():.2f}]")

    # --- Voxel downsampling (optional, runs first) ---
    if voxel_size is not None and voxel_size > 0:
        n_before = len(pcd.points)
        pcd = voxel_down_sample_with_min_points(pcd, voxel_size, voxel_min_points)
        points = np.asarray(pcd.points)
        n_after = len(pcd.points)
        print(f"\n[Voxel Downsampling] voxel_size={voxel_size:.3f}, min_points={voxel_min_points}: "
              f"{n_before} -> {n_after} points "
              f"({100 * n_after / max(n_before, 1):.1f}%)")

    yaw_correction = 0.0
    R_gravity = np.eye(3)
    ground_center_x = 0.0
    ground_center_y = 0.0
    ground_z = 0.0

    if do_align:
        # Step 1: RANSAC ground plane extraction
        print("\n[Step 1] RANSAC ground plane extraction...")
        plane_model, ground_inliers = ransac_ground_plane(pcd)
        a, b, c, d = plane_model
        print(f"  Plane: {a:.4f}x + {b:.4f}y + {c:.4f}z + {d:.4f} = 0")
        print(f"  Ground inliers: {len(ground_inliers)} ({100*len(ground_inliers)/len(points):.1f}%)")
        print(f"  Normal: ({a:.4f}, {b:.4f}, {c:.4f})")

        # Step 2: Compute gravity alignment rotation
        print("\n[Step 2] Gravity alignment...")
        R_gravity = compute_gravity_alignment([a, b, c])
        angle_deg = np.degrees(np.arccos(np.clip(np.dot([a, b, c] / np.linalg.norm([a, b, c]), [0, 0, 1]), -1, 1)))
        print(f"  Tilt angle: {angle_deg:.2f} degrees")

        # Apply gravity rotation
        pcd.rotate(R_gravity, center=(0, 0, 0))
        points = np.asarray(pcd.points)

        # Step 3: Wall-based XY alignment via normal histogram
        print("\n[Step 3] Wall direction detection (normal histogram)...")
        dominant_angle = find_wall_direction_from_normals(pcd)

        if dominant_angle is not None:
            yaw_correction = dominant_angle
            print(f"  Applying yaw correction: -{np.degrees(yaw_correction):.2f} degrees")
            R_yaw = compute_yaw_rotation(yaw_correction)
            pcd.rotate(R_yaw, center=(0, 0, 0))
            points = np.asarray(pcd.points)
        else:
            print("  Could not determine wall direction, skipping XY alignment")

        # Step 4: Translate origin to ground center
        print("\n[Step 4] Origin → ground center...")
        plane_model2, ground_inliers2 = ransac_ground_plane(pcd, distance_threshold=0.1)
        ground_points = points[ground_inliers2]
        ground_center_x = np.median(ground_points[:, 0])
        ground_center_y = np.median(ground_points[:, 1])
        ground_z = np.median(ground_points[:, 2])
        print(f"  Ground center: ({ground_center_x:.3f}, {ground_center_y:.3f}, {ground_z:.3f})")

        points[:, 0] -= ground_center_x
        points[:, 1] -= ground_center_y
        points[:, 2] -= ground_z
        pcd.points = o3d.utility.Vector3dVector(points)
    else:
        print("\n[Coordinate alignment] SKIPPED")
        points = np.asarray(pcd.points)

    # --- Ground infill (optional, default OFF) ---
    if do_infill:
        print("\n[Step 5] Ground infill (multi-floor compatible)...")
        points = np.asarray(pcd.points)
        infill_points = floor_infill(points, resolution=0.2)
        if len(infill_points) > 0:
            print(f"  Added {len(infill_points)} infill points")
            all_points = np.vstack([points, infill_points])
            pcd.points = o3d.utility.Vector3dVector(all_points)
        else:
            print("  No infill needed")
    else:
        print("\n[Ground infill] SKIPPED")

    # Final stats
    points = np.asarray(pcd.points)
    print(f"\n[Result]")
    print(f"  Points: {len(points)}")
    print(f"  Bounds: X[{points[:,0].min():.2f}, {points[:,0].max():.2f}] "
          f"Y[{points[:,1].min():.2f}, {points[:,1].max():.2f}] "
          f"Z[{points[:,2].min():.2f}, {points[:,2].max():.2f}]")

    # Build transform info
    T_full = np.eye(4)
    R_combined = compute_yaw_rotation(yaw_correction) @ R_gravity if yaw_correction != 0 else R_gravity
    T_full[:3, :3] = R_combined
    T_full[0, 3] = -ground_center_x
    T_full[1, 3] = -ground_center_y
    T_full[2, 3] = -ground_z
    print(f"\n  Combined rotation matrix:")
    print(f"    {T_full[0,:3]}")
    print(f"    {T_full[1,:3]}")
    print(f"    {T_full[2,:3]}")
    print(f"  Z offset: {-ground_z:.4f}")

    # Save
    print(f"\nSaving: {output_path}")
    o3d.io.write_point_cloud(output_path, pcd, write_ascii=False)
    print("Done.")

    if visualize:
        # Color ground green, rest gray
        colors = np.full((len(points), 3), 0.6)
        ground_mask = np.abs(points[:, 2]) < 0.15
        colors[ground_mask] = [0.2, 0.8, 0.3]
        pcd.colors = o3d.utility.Vector3dVector(colors)

        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=2.0)
        o3d.visualization.draw_geometries([pcd, coord_frame],
                                          window_name="Map Preprocessor Result")

    return T_full


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Map Preprocessor')
    parser.add_argument('input', help='Input PCD file')
    parser.add_argument('output', help='Output PCD file')
    parser.add_argument('--visualize', action='store_true', help='Show Open3D visualization')
    parser.add_argument('--no-align', action='store_true',
                        help='Skip coordinate alignment (default: alignment ON)')
    parser.add_argument('--infill', action='store_true',
                        help='Enable ground infill (default: OFF)')
    parser.add_argument('--voxel_size', type=float, default=None,
                        help='Voxel downsampling grid size in meters (default: disabled)')
    parser.add_argument('--voxel_min_points', type=int, default=10,
                        help='Min points per voxel; voxels below this are discarded (default: 1)')
    args = parser.parse_args()

    preprocess_map(args.input, args.output, args.visualize,
                   do_align=not args.no_align,
                   do_infill=args.infill,
                   voxel_size=args.voxel_size,
                   voxel_min_points=args.voxel_min_points)
