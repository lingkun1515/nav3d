#!/bin/bash
set -e

source /opt/ros/foxy/setup.bash
# Workspace overlay is optional: it only exists after the first colcon build.
# Tolerate its absence so a fresh container can boot before any build.
if [ -f /ros2_ws/install/setup.bash ]; then
  source /ros2_ws/install/setup.bash
fi

exec "$@"
