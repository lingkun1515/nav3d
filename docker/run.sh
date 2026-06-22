#!/bin/bash
# 运行 Dog3DNav 容器（GPU + 网络透传 + 图形显示）
# 使用方法: bash docker/run.sh

set -e

xhost +local:docker 2>/dev/null || true

docker run -it --rm \
  --gpus all \
  --net=host \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/lenovo/Projects/NavProject/Dog3DNav:/ros2_ws \
  --name dog3dnav \
  dog3dnav:foxy-gpu
