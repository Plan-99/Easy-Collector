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
mkdir -p "$(dirname "$ROS2_LOG_FILE")"
: > "$ROS2_LOG_FILE"  # 시작 시 초기화
exec > >(tee -a "$ROS2_LOG_FILE") 2>&1

# --- ROS2 환경 소싱 ---
source /opt/ros/humble/setup.bash

if [ -f /root/ros2_ws/install/setup.bash ]; then
    source /root/ros2_ws/install/setup.bash
    echo "[ROS2 Bridge] ROS2 workspace sourced"
elif [ -d /root/ros2_ws/src ] && [ "$(ls -A /root/ros2_ws/src 2>/dev/null)" ]; then
    echo "[ROS2 Bridge] Building ROS2 workspace..."
    cd /root/ros2_ws
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release || true
    [ -f /root/ros2_ws/install/setup.bash ] && source /root/ros2_ws/install/setup.bash
fi

# --- 모듈 의존성 복구 (project/modules/*.json = single source of truth) ---
MODULES_DIR="/opt/easytrainer/project/modules"
if [ -d "$MODULES_DIR" ]; then
    MISSING_INFO=$(python3 -c "
import json, subprocess, importlib, glob

apt_missing = []
pip_missing = []

for mj_path in glob.glob('$MODULES_DIR/*.json'):
    meta = json.load(open(mj_path))
    deps = meta.get('dependencies', {})
    for pkg in deps.get('apt', []):
        r = subprocess.run(['dpkg-query','-W','-f=\${Status}',pkg], capture_output=True, text=True)
        if 'install ok installed' not in r.stdout:
            apt_missing.append(pkg)
    for pkg in deps.get('pip', []):
        mod = pkg.split('>=')[0].split('==')[0].replace('-','_')
        try:
            importlib.import_module(mod)
        except ImportError:
            pip_missing.append(pkg)

apt_missing = list(dict.fromkeys(apt_missing))
pip_missing = list(dict.fromkeys(pip_missing))
print('APT:' + ' '.join(apt_missing))
print('PIP:' + ' '.join(pip_missing))
" 2>/dev/null)

    APT_PKGS=$(echo "$MISSING_INFO" | grep '^APT:' | sed 's/^APT://')
    PIP_PKGS=$(echo "$MISSING_INFO" | grep '^PIP:' | sed 's/^PIP://')

    if [ -n "$APT_PKGS" ]; then
        echo "[ROS2 Bridge] Installing missing apt packages: $APT_PKGS"
        apt-get update -qq 2>/dev/null || true
        apt-get install -y --no-install-recommends $APT_PKGS 2>/dev/null || true
    fi
    if [ -n "$PIP_PKGS" ]; then
        echo "[ROS2 Bridge] Installing missing pip packages: $PIP_PKGS"
        python3 -m pip install --quiet $PIP_PKGS 2>/dev/null || true
    fi
    if [ -z "$APT_PKGS" ] && [ -z "$PIP_PKGS" ]; then
        echo "[ROS2 Bridge] All module dependencies satisfied."
    else
        echo "[ROS2 Bridge] Module dependencies installed."
    fi
fi

# --- SDK 모듈 자동 설치 (robot_sdk/*/setup.py) ---
SDK_DIR="/root/robot_sdk"
if [ -d "$SDK_DIR" ]; then
    for sdk_setup in "$SDK_DIR"/*/setup.py; do
        [ -f "$sdk_setup" ] || continue
        sdk_path=$(dirname "$sdk_setup")
        sdk_name=$(basename "$sdk_path")
        # setup.py의 패키지명으로 import 체크
        PKG_NAME=$(python3 -c "
import ast, sys
try:
    tree = ast.parse(open('$sdk_setup').read())
    for node in ast.walk(tree):
        if isinstance(node, ast.keyword) and node.arg == 'name':
            print(ast.literal_eval(node.value))
            sys.exit(0)
except: pass
print('${sdk_name}_sdk')
" 2>/dev/null)
        IMPORT_NAME=$(echo "$PKG_NAME" | tr '-' '_')
        if python3 -c "import $IMPORT_NAME" 2>/dev/null; then
            echo "[ROS2 Bridge] SDK already installed: $sdk_name ($IMPORT_NAME)"
        else
            echo "[ROS2 Bridge] Installing SDK: $sdk_name..."
            python3 -m pip install --quiet "setuptools==70.0.0" 2>/dev/null || true
            python3 -m pip install -e "$sdk_path" 2>&1 || echo "[ROS2 Bridge] WARNING: SDK install failed: $sdk_name"
        fi
    done
fi

# --- 환경 변수 설정 ---
export PYTHONPATH="/root/ros2:/opt/openrobots/lib/python3.10/site-packages:${PYTHONPATH}"
export LD_LIBRARY_PATH="/opt/openrobots/lib:${LD_LIBRARY_PATH}"
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

# MuJoCo: 컨테이너에는 X11 디스플레이가 보장되지 않으므로 GL 백엔드를
# 명시한다. egl(NVIDIA GPU 헤드리스) 기본 → 실패 시 osmesa(소프트웨어
# 렌더링)로 폴백 가능하도록 외부 override 허용.
export MUJOCO_GL=${MUJOCO_GL:-egl}
# PyOpenGL 백엔드도 같이 맞춰주면 일부 드라이버 조합에서 안정적
export PYOPENGL_PLATFORM=${PYOPENGL_PLATFORM:-$MUJOCO_GL}

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

echo "[ROS2 Bridge] All services started (gRPC PID=$GRPC_PID)"

# --- Graceful shutdown ---
cleanup() {
    echo "[ROS2 Bridge] Shutting down..."
    kill $GRPC_PID 2>/dev/null
    wait $GRPC_PID 2>/dev/null
    echo "[ROS2 Bridge] Shutdown complete"
    exit 0
}

trap cleanup INT TERM

# 메인 프로세스 대기
wait $GRPC_PID
EXIT_CODE=$?
echo "[ROS2 Bridge] gRPC server exited with code $EXIT_CODE"
cleanup
