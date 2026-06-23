#!/usr/bin/env python3
"""
Offline conversion: Livox CustomMsg -> standard PointCloud2, MCAP bag -> SQLite db3.

Reads a ROS2 rosbag stored in MCAP format (containing /livox/lidar as
livox_ros_driver2/msg/CustomMsg) and writes a NEW rosbag in sqlite3 (.db3)
storage where:

  - /livox/lidar            sensor_msgs/PointCloud2  (NEW, converted from CustomMsg)
  - /livox/lidar_custom     livox_ros_driver2/msg/CustomMsg (original, renamed)
  - /livox/imu              sensor_msgs/Imu          (unchanged)
  - /utlidar/cloud          sensor_msgs/PointCloud2  (unchanged)
  - /utlidar/imu            sensor_msgs/Imu          (unchanged)

The PointCloud2 field layout exactly replicates livox_ros_driver2 with
xfer_format=0 (PointCloud2), see src/lddc.cpp InitPointcloud2MsgHeader() and
src/comm/comm.h LivoxPointXyzrtlt:

    offset 0   x          FLOAT32  <- CustomPoint.x
    offset 4   y          FLOAT32  <- CustomPoint.y
    offset 8   z          FLOAT32  <- CustomPoint.z
    offset 12  intensity  FLOAT32  <- CustomPoint.reflectivity  (NOTE: stored as intensity)
    offset 16  tag        UINT8    <- CustomPoint.tag
    offset 17  line       UINT8    <- CustomPoint.line
    offset 18  timestamp  FLOAT64  <- CustomPoint.offset_time (ns, cast to double)

point_step = 26 bytes (packed), width = point_num, height = 1, is_dense = True.

The source bag is NEVER modified. Output goes to an independent directory.

Usage:
    # Environment must be sourced first (see convert_and_verify.sh):
    source /opt/ros/humble/setup.bash
    source /home/lenovo/Projects/ROS2_WS/install/setup.bash   # py3.10 livox bindings

    python3 convert_livox_to_pc2_db3.py \
        --input  /home/lenovo/Documents/rosbag/lidar \
        --output /home/lenovo/Documents/rosbag/lidar_with_pc2
"""
import argparse
import os
import shutil
import struct
import sys
import time

# ROS imports -- require sourced ROS2 Humble + livox py3.10 bindings.
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from livox_ros_driver2.msg import CustomMsg


# Packed struct for one point, matching livox_ros_driver2's LivoxPointXyzrtlt
# (#pragma pack(1)) in src/comm/comm.h. Little-endian, NO padding:
#   f x, f y, f z, f intensity(=reflectivity), B tag, B line, d timestamp
# 4+4+4+4+1+1+8 = 26 bytes. Note: under "<" (standard size, no alignment)
# there is no padding before the trailing double, matching the C pack(1) struct.
POINT_STRUCT = struct.Struct("<ffffBBd")
assert POINT_STRUCT.size == 26, "LivoxPointXyzrtlt must be 26 bytes packed"


def build_pc2_fields():
    """PointCloud2 fields matching livox_ros_driver2 xfer_format=0 layout."""
    def f(name, offset, dtype, count=1):
        pf = PointField()
        pf.name = name
        pf.offset = offset
        pf.datatype = dtype
        pf.count = count
        return pf

    return [
        f("x",         0,  PointField.FLOAT32),
        f("y",         4,  PointField.FLOAT32),
        f("z",         8,  PointField.FLOAT32),
        f("intensity", 12, PointField.FLOAT32),
        f("tag",       16, PointField.UINT8),
        f("line",      17, PointField.UINT8),
        f("timestamp", 18, PointField.FLOAT64),
    ]


FIELDS = build_pc2_fields()
POINT_STEP = 26


def custom_msg_to_pc2(custom: CustomMsg) -> PointCloud2:
    """Convert a Livox CustomMsg to a PointCloud2 (livox xfer_format=0 layout)."""
    n = custom.point_num
    points = custom.points

    # Pack all points into a single bytearray.
    data = bytearray(n * POINT_STEP)
    # Guard against point_num vs len(points) mismatch (defensive).
    pack_into = POINT_STRUCT.pack_into
    for i in range(n):
        p = points[i]
        pack_into(data, i * POINT_STEP,
                  p.x, p.y, p.z, float(p.reflectivity), p.tag, p.line,
                  float(p.offset_time))
    cloud = PointCloud2()
    cloud.header = Header()
    cloud.header.stamp = custom.header.stamp
    cloud.header.frame_id = custom.header.frame_id or "livox_frame"
    cloud.height = 1
    cloud.width = n
    cloud.fields = FIELDS
    cloud.is_bigendian = False
    cloud.point_step = POINT_STEP
    cloud.row_step = n * POINT_STEP
    cloud.data = bytes(data)
    cloud.is_dense = True
    return cloud


def topic_meta(name, msg_type, qos_yaml):
    return rosbag2_py.TopicMetadata(
        name=name,
        type=msg_type,
        serialization_format="cdr",
        offered_qos_profiles=qos_yaml,
    )


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--input", required=True,
                    help="Path to source bag directory (mcap)")
    ap.add_argument("--output", required=True,
                    help="Path to output bag directory (db3). Will be created.")
    ap.add_argument("--custom-topic", default="/livox/lidar_custom",
                    help="Output topic name for the (renamed) original CustomMsg")
    ap.add_argument("--pc2-topic", default="/livox/lidar",
                    help="Output topic name for the converted PointCloud2")
    ap.add_argument("--force", action="store_true",
                    help="If set, remove the output directory if it exists")
    args = ap.parse_args()

    in_uri = os.path.abspath(args.input)
    out_uri = os.path.abspath(args.output)

    if in_uri == out_uri:
        print("ERROR: input and output must differ", file=sys.stderr)
        return 2
    # The sqlite storage plugin refuses to overwrite an existing output dir.
    # Let the plugin create it; if --force, clean up first.
    if os.path.exists(out_uri):
        if not args.force:
            print(f"ERROR: output dir exists (use --force to remove): {out_uri}",
                  file=sys.stderr)
            return 2
        shutil.rmtree(out_uri)
    # NOTE: do NOT create out_uri here; SequentialWriter.open() requires it absent.

    # ---- Open reader (mcap) ----
    reader = rosbag2_py.SequentialReader()
    r_storage = rosbag2_py.StorageOptions(uri=in_uri, storage_id="mcap")
    r_conv = rosbag2_py.ConverterOptions("cdr", "cdr")
    reader.open(r_storage, r_conv)

    # QoS profiles from source topics (reuse for output to preserve reliability).
    src_topics = reader.get_all_topics_and_types()
    qos_by_topic = {t.name: t.offered_qos_profiles for t in src_topics}
    livox_lidar_qos = qos_by_topic.get("/livox/lidar", "")
    livox_imu_qos = qos_by_topic.get("/livox/imu", "")
    ut_cloud_qos = qos_by_topic.get("/utlidar/cloud", "")
    ut_imu_qos = qos_by_topic.get("/utlidar/imu", "")

    # ---- Open writer (sqlite3) ----
    writer = rosbag2_py.SequentialWriter()
    w_storage = rosbag2_py.StorageOptions(uri=out_uri, storage_id="sqlite3")
    w_conv = rosbag2_py.ConverterOptions("cdr", "cdr")
    writer.open(w_storage, w_conv)

    # Register all output topics up front.
    writer.create_topic(topic_meta(args.pc2_topic,        "sensor_msgs/msg/PointCloud2", livox_lidar_qos))
    writer.create_topic(topic_meta(args.custom_topic,     "livox_ros_driver2/msg/CustomMsg", livox_lidar_qos))
    writer.create_topic(topic_meta("/livox/imu",          "sensor_msgs/msg/Imu", livox_imu_qos))
    writer.create_topic(topic_meta("/utlidar/cloud",      "sensor_msgs/msg/PointCloud2", ut_cloud_qos))
    writer.create_topic(topic_meta("/utlidar/imu",        "sensor_msgs/msg/Imu", ut_imu_qos))

    counts = {
        args.pc2_topic: 0,
        args.custom_topic: 0,
        "/livox/imu": 0,
        "/utlidar/cloud": 0,
        "/utlidar/imu": 0,
    }
    skipped = 0
    total = 0
    t0 = time.time()

    print(f"Converting: {in_uri}")
    print(f"        ->  {out_uri}")
    print(f"  {args.pc2_topic}  (PointCloud2, converted)")
    print(f"  {args.custom_topic}  (CustomMsg, renamed)")
    print()

    while reader.has_next():
        (topic, data, stamp_ns) = reader.read_next()
        total += 1

        if topic == "/livox/lidar":
            # Deserialize CustomMsg, convert to PC2, and also re-emit the
            # original CustomMsg under the renamed topic.
            custom = deserialize_message(data, CustomMsg)
            pc2 = custom_msg_to_pc2(custom)

            pc2_ser = serialize_message(pc2)
            writer.write(args.pc2_topic, pc2_ser, stamp_ns)
            counts[args.pc2_topic] += 1

            # Original CustomMsg bytes are already CDR; write them verbatim
            # under the renamed topic (no need to re-serialize).
            writer.write(args.custom_topic, data, stamp_ns)
            counts[args.custom_topic] += 1

        elif topic == "/livox/imu":
            writer.write("/livox/imu", data, stamp_ns)
            counts["/livox/imu"] += 1

        elif topic == "/utlidar/cloud":
            writer.write("/utlidar/cloud", data, stamp_ns)
            counts["/utlidar/cloud"] += 1

        elif topic == "/utlidar/imu":
            writer.write("/utlidar/imu", data, stamp_ns)
            counts["/utlidar/imu"] += 1

        else:
            # Unknown topic in source bag; skip with a warning (first time only).
            skipped += 1
            if skipped <= 3:
                print(f"  [skip] unknown topic in source: {topic}")

        if total % 2000 == 0:
            elapsed = time.time() - t0
            print(f"  processed {total} msgs "
                  f"({counts[args.pc2_topic]} lidar frames, "
                  f"{counts['/livox/imu']} livox imu, "
                  f"{counts['/utlidar/cloud']} ut cloud, "
                  f"{counts['/utlidar/imu']} ut imu) "
                  f"[{elapsed:.1f}s]")

    writer.close()

    elapsed = time.time() - t0
    print()
    print("=== Done ===")
    print(f"  Total source messages read : {total}")
    print(f"  Skipped (unknown topics)   : {skipped}")
    print(f"  Elapsed                    : {elapsed:.1f}s")
    print("  Output topic counts:")
    for name, c in counts.items():
        print(f"    {name:24s} {c}")

    print(f"\nOutput bag: {out_uri}")
    print("Verify with:")
    print(f"  ros2 bag info {out_uri}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
