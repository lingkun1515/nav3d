#!/bin/bash
export ROS_DOMAIN_ID=0
export FASTRTPS_BUILTIN_TRANSPORTS=UDPv4
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/foxy/setup.bash
source /ros2_ws/install/setup.bash
ros2 daemon stop 2>/dev/null
sleep 1
ros2 daemon start 2>/dev/null
sleep 1
exec ros2 launch simulation gazebo.launch.py robot_model:=a1 slam_mode:=true gui:=false
