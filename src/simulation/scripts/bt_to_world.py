#!/usr/bin/env python3
"""
Convert an OctoMap .bt file to a Gazebo .world file.

Reads occupied voxels from .bt, greedily merges them into axis-aligned boxes,
and writes SDF box collisions for Gazebo.

Usage:
  python3 bt_to_world.py input.bt output.world [--resolution 0.3] [--max-boxes 3000]
  python3 bt_to_world.py input.bt output.world [--resolution 0.3] [--min-z 0.0]
"""
import argparse
import sys
import tempfile
import subprocess
import numpy as np
from pathlib import Path


def load_octomap_bt(filepath, resolution=0.2):
    """Load .bt OctoMap binary, extract occupied leaf centers."""
    # Try python-octomap bindings first
    try:
        import octomap
        tree = octomap.OcTree(resolution)
        tree.readBinary(str(filepath).encode())
        points = []
        it = tree.begin_leafs()
        while it != tree.end_leafs():
            if tree.isNodeOccupied(it):
                coord = it.getCoordinate()
                points.append([coord.x(), coord.y(), coord.z()])
            it.__next__()
        return np.array(points, dtype=np.float64)
    except ImportError:
        pass

    # Fallback: octomap CLI (octomap-tools)
    with tempfile.NamedTemporaryFile(suffix='.pcd', delete=False) as tmp:
        tmp_path = tmp.name
    try:
        subprocess.run(['octomap_to_pcd', str(filepath), tmp_path],
                       check=True, capture_output=True)
        points = _load_pcd_xyz(tmp_path)
        return np.array(points, dtype=np.float64)
    except (FileNotFoundError, subprocess.CalledProcessError):
        print("Error: cannot load .bt file. Install python-octomap or octomap-tools.",
              file=sys.stderr)
        sys.exit(1)
    finally:
        Path(tmp_path).unlink(missing_ok=True)


def _load_pcd_xyz(filepath):
    """Load PCD file, return list of [x, y, z]."""
    import struct
    points = []
    with open(filepath, 'rb') as f:
        header_lines = []
        while True:
            line = f.readline().decode('ascii', errors='ignore').strip()
            header_lines.append(line)
            if line.startswith('DATA'):
                break

        fields = []
        num_points = 0
        data_type = 'ascii'
        for line in header_lines:
            if line.startswith('FIELDS'):
                fields = line.split()[1:]
            elif line.startswith('POINTS'):
                num_points = int(line.split()[1])
            elif line.startswith('DATA'):
                data_type = line.split()[1].lower()

        x_idx = fields.index('x') if 'x' in fields else 0
        y_idx = fields.index('y') if 'y' in fields else 1
        z_idx = fields.index('z') if 'z' in fields else 2

        if data_type == 'ascii':
            for _ in range(num_points):
                parts = f.readline().decode('ascii', errors='ignore').strip().split()
                if len(parts) > max(x_idx, y_idx, z_idx):
                    points.append([float(parts[x_idx]), float(parts[y_idx]), float(parts[z_idx])])
        else:
            point_size = len(fields) * 4
            for _ in range(num_points):
                data = f.read(point_size)
                if len(data) < point_size:
                    break
                vals = struct.unpack(f'<{len(fields)}f', data)
                points.append([vals[x_idx], vals[y_idx], vals[z_idx]])

    return points


def voxelize(points, resolution):
    if len(points) == 0:
        return points
    indices = np.round(points / resolution).astype(int)
    unique_indices = np.unique(indices, axis=0)
    return unique_indices.astype(float) * resolution


def greedy_merge_boxes(voxels, resolution):
    if len(voxels) == 0:
        return []

    voxel_set = set(map(tuple, np.round(voxels / resolution).astype(int)))
    visited = set()
    boxes = []

    for v in sorted(voxel_set):
        if v in visited:
            continue
        # Extend along X
        x_end = v[0]
        while (x_end + 1, v[1], v[2]) in voxel_set and (x_end + 1, v[1], v[2]) not in visited:
            x_end += 1
        # Extend along Y
        y_end = v[1]
        can_extend_y = True
        while can_extend_y:
            for xi in range(v[0], x_end + 1):
                if (xi, y_end + 1, v[2]) not in voxel_set or (xi, y_end + 1, v[2]) in visited:
                    can_extend_y = False
                    break
            if can_extend_y:
                y_end += 1
        # Extend along Z
        z_end = v[2]
        can_extend_z = True
        while can_extend_z:
            for xi in range(v[0], x_end + 1):
                for yi in range(v[1], y_end + 1):
                    if (xi, yi, z_end + 1) not in voxel_set or (xi, yi, z_end + 1) in visited:
                        can_extend_z = False
                        break
                if not can_extend_z:
                    break
            if can_extend_z:
                z_end += 1
        # Mark visited
        for xi in range(v[0], x_end + 1):
            for yi in range(v[1], y_end + 1):
                for zi in range(v[2], z_end + 1):
                    visited.add((xi, yi, zi))
        # Box center and size
        cx = (v[0] + x_end) / 2.0 * resolution
        cy = (v[1] + y_end) / 2.0 * resolution
        cz = (v[2] + z_end) / 2.0 * resolution
        sx = (x_end - v[0] + 1) * resolution
        sy = (y_end - v[1] + 1) * resolution
        sz = (z_end - v[2] + 1) * resolution
        boxes.append((cx, cy, cz, sx, sy, sz))

    return boxes


def generate_world_sdf(boxes):
    models_sdf = ""
    for i, (cx, cy, cz, sx, sy, sz) in enumerate(boxes):
        models_sdf += f"""
    <model name="obstacle_{i}">
      <static>true</static>
      <pose>{cx} {cy} {cz} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.6 0.6 0.6 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <world name="bt_world">
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <physics type="ode">
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>500</real_time_update_rate>
    </physics>

    <gravity>0 0 -9.81</gravity>
{models_sdf}
  </world>
</sdf>
"""


def main():
    parser = argparse.ArgumentParser(description='Convert OctoMap .bt to Gazebo .world')
    parser.add_argument('input', help='Input .bt file')
    parser.add_argument('output', help='Output .world file')
    parser.add_argument('--resolution', type=float, default=0.3,
                        help='Voxel resolution in meters (default: 0.3)')
    parser.add_argument('--min-z', type=float, default=None,
                        help='Exclude voxels below this Z (auto-detect ground if omitted)')
    parser.add_argument('--max-z', type=float, default=None,
                        help='Exclude voxels above this Z (meters)')
    parser.add_argument('--max-boxes', type=int, default=3000,
                        help='Maximum number of boxes (default: 3000)')
    args = parser.parse_args()

    input_path = Path(args.input)
    if not input_path.exists():
        print(f"Error: input file not found: {args.input}", file=sys.stderr)
        sys.exit(1)
    if input_path.suffix.lower() != '.bt':
        print(f"Error: expected .bt file, got: {args.input}", file=sys.stderr)
        sys.exit(1)

    print(f"Loading OctoMap: {args.input}")
    points = load_octomap_bt(args.input, args.resolution)
    if len(points) == 0:
        print("Error: no occupied voxels found", file=sys.stderr)
        sys.exit(1)
    print(f"Extracted {len(points)} occupied voxels")

    voxels = voxelize(points, args.resolution)
    print(f"Voxelized to {len(voxels)} voxels at {args.resolution}m")

    # Auto-detect ground: exclude lowest Z layer
    if args.min_z is None:
        min_z = voxels[:, 2].min()
        mask = voxels[:, 2] > min_z + args.resolution * 0.5
        voxels = voxels[mask]
        print(f"Auto-excluded ground layer at z={min_z:.2f}")
    else:
        voxels = voxels[voxels[:, 2] >= args.min_z]
    if args.max_z is not None:
        voxels = voxels[voxels[:, 2] <= args.max_z]

    print(f"After filtering: {len(voxels)} obstacle voxels")

    boxes = greedy_merge_boxes(voxels, args.resolution)
    print(f"Merged into {len(boxes)} boxes")

    if len(boxes) > args.max_boxes:
        print(f"Truncating to {args.max_boxes} boxes (sorted by volume)")
        boxes.sort(key=lambda b: b[3] * b[4] * b[5], reverse=True)
        boxes = boxes[:args.max_boxes]

    world_sdf = generate_world_sdf(boxes)
    Path(args.output).write_text(world_sdf)
    print(f"Written: {args.output}")


if __name__ == '__main__':
    main()
