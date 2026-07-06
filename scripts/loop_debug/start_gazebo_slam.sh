#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
source /opt/ros/humble/setup.bash
source "$PROJECT_DIR/install/setup.bash"
exec ros2 launch simulation gazebo.launch.py robot_model:=a1 slam_mode:=true gui:=false
