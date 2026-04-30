#!/usr/bin/env bash
# Post-install: NGC 로그인 안내 + Docker 이미지 빌드.
set -e

PROJECT_DIR="${EASYTRAINER_DATA_DIR:-/opt/easytrainer}/project"
ISAACLAB_DIR="$PROJECT_DIR/isaaclab"

echo "================================================================"
echo " Isaac Lab Module Installation"
echo "================================================================"
echo ""
echo "Isaac Lab requires NVIDIA NGC login (https://ngc.nvidia.com)."
echo "After getting an NGC API key, run:"
echo "  docker login nvcr.io"
echo ""
echo "Then build the images:"
echo "  cd $ISAACLAB_DIR/docker && docker compose --profile ros2 build"
echo "  cd $ISAACLAB_DIR/curobo && docker build -t curobo-ros2:latest ."
echo ""
echo "Once built, the launcher can start the simulation."
echo "================================================================"
