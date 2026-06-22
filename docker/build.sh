#!/bin/bash
# 构建 Dog3DNav Docker 镜像
# 使用方法: bash docker/build.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

cd "$PROJECT_DIR"

docker build \
  -f docker/Dockerfile \
  -t dog3dnav:foxy-gpu \
  .
