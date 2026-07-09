#!/usr/bin/env python3
"""
将 jie_3d_nav 的地图包 NPZ 文件转换为 octomap .bt 二进制格式。

jie_3d_nav 的 NPZ 格式来自 map_package_manager.py，存储了
octomap_msgs::msg::Octomap 的序列化字段：
  - binary:   bool (是否为 binary 编码)
  - id:       string (tree type, 如 "OcTree")
  - resolution: float64
  - data:     int8[] (writeBinaryData 输出的原始节点二进制数据)

.bt 文件格式 (来自 octomap::AbstractOccupancyOcTree::writeBinary):
  text:   "# Octomap OcTree binary\n"
  text:   "{tree_type}\n"
  binary: uint32 (tree size, 节点总数)
  binary: double (resolution)
  binary: [writeBinaryData 输出的节点数据]

用法:
  python3 npz_to_bt.py <input.npz> [output.bt]
  python3 npz_to_bt.py --package-dir <jie_map_package_dir> [output.bt]
"""

import struct
import sys
import os
import argparse
import numpy as np


def count_nodes(data: bytes) -> int:
    """从 writeBinaryData 的二进制数据中统计节点数。

    格式: 每个节点 2 bytes (child1to4, child5to8).
    每字节中每 2 bits 表示一个子节点状态:
      00 = unknown (leaf)
      01 = occupied (leaf)
      10 = free (leaf)
      11 = has children (inner node, 需递归)
    只统计有实际内容的节点（非全零）。
    """
    if len(data) < 2:
        return 0

    total = 0
    stack = [0]  # offset stack, 进入一个 inner node

    while stack:
        offset = stack.pop()
        if offset + 2 > len(data):
            continue
        total += 1
        child1to4 = data[offset]
        child5to8 = data[offset + 1]

        children = []
        for byte_val in (child1to4, child5to8):
            for shift in (0, 2, 4, 6):
                bits = (byte_val >> shift) & 0x03
                children.append(bits)

        # 收集有子节点的子节点（bits == 3），加入栈中（后进先出）
        for child_bits in reversed(children):
            if child_bits == 3:  # has children
                # 子节点从当前节点之后开始，但实际编码是深度优先
                # 我们需要维护一个全局偏移量
                pass

    # 上面这种方法行不通，因为字节偏移不简单。
    # 改用不同的策略：递归解析并计数。
    return _count_nodes_recursive(data, 0)[0]


def _count_nodes_recursive(data: bytes, offset: int):
    """递归解析节点，返回 (count, new_offset)."""
    if offset + 2 > len(data):
        return 0, offset

    count = 1
    child1to4 = data[offset]
    child5to8 = data[offset + 1]
    offset += 2

    children_bits = []
    for byte_val in (child1to4, child5to8):
        for shift in (0, 2, 4, 6):
            children_bits.append((byte_val >> shift) & 0x03)

    for bits in children_bits:
        if bits == 3:  # inner node, recurse
            sub_count, offset = _count_nodes_recursive(data, offset)
            count += sub_count
    return count, offset


def make_bt_file(output_path: str, tree_type: str, tree_size: int,
                 resolution: float, node_data: bytes):
    """构造并写入 .bt 文件。"""
    with open(output_path, 'wb') as f:
        f.write(b"# Octomap OcTree binary\n")
        f.write(tree_type.encode('utf-8') + b"\n")
        f.write(struct.pack('<I', tree_size))
        f.write(struct.pack('<d', resolution))
        f.write(node_data)
    return output_path


def npz_to_bt(npz_path: str, output_path: str):
    """从 NPZ 文件生成 .bt 文件。"""
    data = np.load(npz_path, allow_pickle=True)

    resolution = float(data['resolution'])
    tree_type = str(data['octomap_id'])
    node_data = data['data'].tobytes()

    # 若 NPZ 中有 binary 字段且为 False，说明是 full 格式（.ot），不能直接转换
    if 'binary' in data and not bool(data['binary']):
        print("WARNING: NPZ contains full (non-binary) octomap data. "
              "The resulting .bt file may not be valid.\n"
              "  Consider re-saving the map from jie_octomap with binary encoding.")

    tree_size = _count_nodes_recursive(node_data, 0)[0]

    print(f"NPZ: {npz_path}")
    print(f"  Tree type:   {tree_type}")
    print(f"  Resolution:  {resolution}")
    print(f"  Node count:  {tree_size}")
    print(f"  Data size:   {len(node_data)} bytes")

    make_bt_file(output_path, tree_type, tree_size, resolution, node_data)
    print(f"Saved: {output_path}")


def convert_package_dir(package_dir: str, output_path: str):
    """从 jie 地图包目录转换（查找 octomap_msg.npz）。"""
    npz_path = os.path.join(package_dir, "octomap_msg.npz")
    if not os.path.isfile(npz_path):
        print(f"ERROR: {npz_path} not found.", file=sys.stderr)
        sys.exit(1)
    npz_to_bt(npz_path, output_path)


def main():
    parser = argparse.ArgumentParser(
        description="Convert jie_3d_nav NPZ map package to octomap .bt format")
    parser.add_argument(
        "input", nargs="?", default=None,
        help="Path to octomap_msg.npz file")
    parser.add_argument(
        "output", nargs="?", default=None,
        help="Output .bt file path (default: <input>.bt)")
    parser.add_argument(
        "--package-dir", "-d", default=None,
        help="Path to jie map package directory (looks for octomap_msg.npz inside)")
    args = parser.parse_args()

    if args.package_dir:
        output = args.output or os.path.join(args.package_dir, "map.bt")
        convert_package_dir(args.package_dir, output)
    elif args.input:
        output = args.output or os.path.splitext(args.input)[0] + ".bt"
        npz_to_bt(args.input, output)
    else:
        parser.print_help()
        sys.exit(1)


if __name__ == "__main__":
    main()
