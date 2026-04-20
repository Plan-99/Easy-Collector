#!/usr/bin/env bash
# ===================================================================
# EasyTrainer ROS2 Bridge Container Entrypoint
# gRPC 서버 + WebRTC 스트리밍 시작
# ===================================================================
set -e

echo "============================================"
echo " EasyTrainer ROS2 Bridge Starting..."
echo "============================================"

# --- Python stdout 버퍼링 해제 (pipe를 통해 tee로 전달 시 필수) ---
export PYTHONUNBUFFERED=1

# --- 공유 로그 파일 설정 (메인 컨테이너에서 읽을 수 있도록) ---
ROS2_LOG_FILE="/dev/shm/easytrainer/ros2.log"
: > "$ROS2_LOG_FILE"  # 시작 시 초기화
exec > >(tee -a "$ROS2_LOG_FILE") 2>&1

# --- ROS2 환경 소싱 ---
source /opt/ros/humble/setup.bash

if [ -f /root/ros2_ws/install/setup.bash ]; then
    source /root/ros2_ws/install/setup.bash
    echo "[ROS2 Bridge] ROS2 workspace sourced"
else
    echo "[ROS2 Bridge] WARNING: ros2_ws/install/setup.bash not found, building..."
    cd /root/ros2_ws
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    source /root/ros2_ws/install/setup.bash
fi

# --- 환경 변수 설정 ---
export PYTHONPATH="/root/src:/root/src/backend/lerobot/src:/opt/openrobots/lib/python3.10/site-packages:${PYTHONPATH}"
export LD_LIBRARY_PATH="/opt/openrobots/lib:${LD_LIBRARY_PATH}"
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

# ROS Domain ID from config
if [ -f /opt/easytrainer/config.json ]; then
    DOMAIN_ID=$(python3 -c "import json; print(json.load(open('/opt/easytrainer/config.json')).get('ros_domain_id', 0))" 2>/dev/null || echo "0")
    export ROS_DOMAIN_ID=$DOMAIN_ID
fi

echo "[ROS2 Bridge] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "[ROS2 Bridge] RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"

# --- 공유 메모리 디렉토리 생성 ---
mkdir -p /dev/shm/easytrainer
chmod 777 /dev/shm/easytrainer

# --- 로그 디렉토리 ---
LOG_DIR="${EASYTRAINER_LOG_DIR:-/tmp/easytrainer/logs}"
mkdir -p "$LOG_DIR"

# --- gRPC 서버 시작 ---
echo "[ROS2 Bridge] Starting gRPC server on port 50051..."
python3 -m ros2_bridge.server &
GRPC_PID=$!

# --- WebRTC 스트리밍 서버 시작 ---
echo "[ROS2 Bridge] Starting WebRTC streaming server on port 5002..."
python3 -m backend.api.streaming &
STREAM_PID=$!

echo "[ROS2 Bridge] All services started"
echo "[ROS2 Bridge] gRPC PID=$GRPC_PID, Streaming PID=$STREAM_PID"

# --- Graceful shutdown ---
cleanup() {
    echo "[ROS2 Bridge] Shutting down..."
    kill $GRPC_PID $STREAM_PID 2>/dev/null
    wait $GRPC_PID $STREAM_PID 2>/dev/null
    echo "[ROS2 Bridge] Shutdown complete"
    exit 0
}

trap cleanup INT TERM

# 메인 프로세스 대기
wait -n $GRPC_PID $STREAM_PID
EXIT_CODE=$?
echo "[ROS2 Bridge] A service exited with code $EXIT_CODE"
cleanup
