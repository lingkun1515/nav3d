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
import numpy as np
from pathlib import Path


def load_octomap_bt(filepath, resolution=0.2):
    """Load .bt OctoMap binary, extract occupied leaf centers.

    Pure Python implementation — no external OctoMap dependency needed.
    OctoMap binary format: 2 bytes per node, encoding 8 children (2 bits each):
      00 = no child, 01 = free leaf, 10 = occupied leaf, 11 = inner node.
    Depth-first traversal; tree_depth defaults to 16.
    """
    import struct

    with open(filepath, 'rb') as f:
        data = f.read()

    header_end = data.find(b'data\n') + 5
    binary = data[header_end:]
    pos = [0]  # list for mutable closure

    def _read_u16():
        if pos[0] + 2 > len(binary):
            return None
        lo, hi = binary[pos[0]], binary[pos[0] + 1]
        pos[0] += 2
        return lo, hi

    tree_depth = 16
    root_size = resolution * (1 << tree_depth)
    occupied = []

    def _parse(depth, cx, cy, cz, size):
        node = _read_u16()
        if node is None:
            return
        b1, b2 = node
        half = size / 2.0
        quarter = size / 4.0

        for child_i in range(8):
            shift = (child_i % 4) * 2
            code = ((b1 if child_i < 4 else b2) >> shift) & 0x03

            dx = quarter if (child_i & 1) else -quarter
            dy = quarter if (child_i & 2) else -quarter
            dz = quarter if (child_i & 4) else -quarter

            if code == 0x02:  # occupied leaf
                occupied.append([cx + dx, cy + dy, cz + dz])
            elif code == 0x03:  # inner node
                if depth + 1 < tree_depth:
                    _parse(depth + 1, cx + dx, cy + dy, cz + dz, half)

    _parse(0, 0.0, 0.0, 0.0, root_size)

    if not occupied:
        print("Error: no occupied voxels found in .bt file", file=sys.stderr)
        sys.exit(1)

    return np.array(occupied, dtype=np.float64)


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
