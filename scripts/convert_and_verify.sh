#!/usr/bin/env bash
#
# Convert the Livox mcap rosbag into a db3 rosbag with both the original
# CustomMsg (renamed) and a freshly-converted standard PointCloud2.
#
# Then runs verification (ros2 bag info + sample readback of /livox/lidar).
#
# Usage:
#   ./convert_and_verify.sh [INPUT_BAG_DIR] [OUTPUT_BAG_DIR]
#
# Defaults point at the bag described in the project task.

# NOTE: -u is intentionally omitted: ROS2 setup.bash files reference unset
# vars (e.g. AMENT_TRACE_SETUP_FILES), so `set -u` breaks sourcing.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

INPUT_BAG="${1:-/home/lenovo/Documents/rosbag/lidar}"
OUTPUT_BAG="${2:-/home/lenovo/Documents/rosbag/lidar_with_pc2}"

# ---- source ROS env (order matters) ----
# 1. ROS2 Humble
source /opt/ros/humble/setup.bash
# 2. ROS2_WS install provides the working cpython-3.10 livox_ros_driver2
#    Python bindings (Dog3DNav's install is py3.8 and cannot be loaded by
#    the system python3.10 used for this conversion).
if [ -f /home/lenovo/Projects/ROS2_WS/install/setup.bash ]; then
    source /home/lenovo/Projects/ROS2_WS/install/setup.bash
else
    echo "ERROR: /home/lenovo/Projects/ROS2_WS/install/setup.bash not found." >&2
    echo "       The working py3.10 livox bindings live there." >&2
    exit 1
fi

echo "ROS_DISTRO=$ROS_DISTRO  python=$(which python3)"
python3 -c "import livox_ros_driver2.msg; import rosbag2_py; print('imports OK')"

echo
echo "=== Converting ==="
python3 "${SCRIPT_DIR}/convert_livox_to_pc2_db3.py" \
    --input  "${INPUT_BAG}" \
    --output "${OUTPUT_BAG}" \
    --force

echo
echo "=== ros2 bag info ==="
ros2 bag info "${OUTPUT_BAG}"

echo
echo "=== sample readback of first /livox/lidar PointCloud2 ==="
python3 - "${OUTPUT_BAG}" <<'PY'
import sys, rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
out = sys.argv[1]
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=out, storage_id="sqlite3"),
       rosbag2_py.ConverterOptions("cdr","cdr"))
seen = 0
while r.has_next():
    topic, data, ts = r.read_next()
    if topic == "/livox/lidar":
        msg = deserialize_message(data, PointCloud2)
        print(f"topic={topic} width={msg.width} point_step={msg.point_step} "
              f"frame={msg.header.frame_id} data_len={len(msg.data)} "
              f"fields={[ (f.name, f.offset, f.datatype) for f in msg.fields ]}")
        seen += 1
        if seen >= 2:
            break
print("OK")
PY

echo
echo "=== output files ==="
ls -lh "${OUTPUT_BAG}"
