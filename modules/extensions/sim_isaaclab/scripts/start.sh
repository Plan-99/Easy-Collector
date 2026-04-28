#!/usr/bin/env bash
# Start Isaac Lab + cuRobo containers.
# 런처가 호출. ROS_DOMAIN_ID를 EasyTrainer config.json과 공유.
set -e

MODULE_DIR="$(cd "$(dirname "$0")/.." && pwd)"
COMPOSE_FILE="$MODULE_DIR/docker-compose.isaaclab.yml"
CONFIG_FILE="${EASYTRAINER_CONFIG_PATH:-/opt/easytrainer/config.json}"

# config.json에서 ros_domain_id 읽기 (없으면 0)
if [ -f "$CONFIG_FILE" ]; then
    DOMAIN_ID=$(python3 -c "import json; print(json.load(open('$CONFIG_FILE')).get('ros_domain_id', 0))" 2>/dev/null || echo 0)
else
    DOMAIN_ID=0
fi

export ROS_DOMAIN_ID="$DOMAIN_ID"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

echo "[sim_isaaclab] Starting Isaac Lab + cuRobo (ROS_DOMAIN_ID=$ROS_DOMAIN_ID)..."

# 이미지 존재 확인
if ! docker image inspect isaac-lab-ros2:latest >/dev/null 2>&1; then
    echo "[sim_isaaclab][ERROR] Docker image 'isaac-lab-ros2:latest' not found."
    echo "[sim_isaaclab] Build it first via: cd /opt/easytrainer/project/isaaclab/docker && docker compose --profile ros2 build"
    exit 1
fi
if ! docker image inspect curobo-ros2:latest >/dev/null 2>&1; then
    echo "[sim_isaaclab][WARN] Docker image 'curobo-ros2:latest' not found. cuRobo will be skipped."
fi

docker compose -f "$COMPOSE_FILE" up -d
echo "[sim_isaaclab] Started."
