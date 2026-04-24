#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash

WS="/root/ros2_ws"

# --- Install module dependencies from manifests (single source of truth) ---
# project/modules/*.json contains all installed module manifests.
# On container restart, apt/pip packages are lost — this restores them.
DEPS_MARKER="/tmp/.ros2_deps_installed"
DATA_DIR="${EASYTRAINER_DATA_DIR:-/opt/easytrainer}"
MANIFEST_DIR="$DATA_DIR/project/modules"

if [ ! -f "$DEPS_MARKER" ]; then
    # 1) Restore deps from manifest dir (covers ALL modules including apt/pip-only)
    if [ -d "$MANIFEST_DIR" ]; then
        for mj in "$MANIFEST_DIR"/*.json; do
            [ -f "$mj" ] || continue
            mod_id=$(python3 -c "import json; print(json.load(open('$mj')).get('id',''))" 2>/dev/null)
            [ -z "$mod_id" ] && continue

            pip_deps=$(python3 -c "import json; deps=json.load(open('$mj')).get('dependencies',{}).get('pip',[]); print(' '.join(deps))" 2>/dev/null)
            if [ -n "$pip_deps" ]; then
                echo "[ros2] Restoring pip deps for $mod_id..."
                python3 -m pip install --quiet $pip_deps 2>/dev/null || true
            fi

            apt_deps=$(python3 -c "import json; deps=json.load(open('$mj')).get('dependencies',{}).get('apt',[]); print(' '.join(deps))" 2>/dev/null)
            if [ -n "$apt_deps" ]; then
                echo "[ros2] Restoring apt deps for $mod_id..."
                apt-get install -y --no-install-recommends $apt_deps 2>/dev/null || true
            fi
        done
    fi

    # 2) Custom install scripts from ros2_ws/src (need source files, not just manifest)
    if [ -d "$WS/src" ] && [ "$(ls -A $WS/src 2>/dev/null)" ]; then
        for mj in "$WS"/src/*/module.json; do
            [ -f "$mj" ] || continue
            install_script=$(python3 -c "import json; print(json.load(open('$mj')).get('install_script',''))" 2>/dev/null)
            if [ -n "$install_script" ]; then
                script_path="$(dirname $mj)/$install_script"
                if [ -f "$script_path" ]; then
                    pkg_name=$(basename $(dirname "$mj"))
                    echo "[ros2] Running install script for $pkg_name..."
                    bash "$script_path" || true
                fi
            fi
        done
    fi

    # 3) Install SDK modules (robot_sdk/*/setup.py)
    SDK_DIR="/root/robot_sdk"
    if [ -d "$SDK_DIR" ]; then
        for sdk_setup in "$SDK_DIR"/*/setup.py; do
            [ -f "$sdk_setup" ] || continue
            sdk_path=$(dirname "$sdk_setup")
            sdk_name=$(basename "$sdk_path")
            if ! python3 -c "import ${sdk_name}_sdk" 2>/dev/null && \
               ! python3 -c "import ${sdk_name}" 2>/dev/null; then
                echo "[ros2] Installing SDK: $sdk_name..."
                python3 -m pip install -e "$sdk_path" 2>&1 | tail -1 || true
            fi
        done
    fi

    touch "$DEPS_MARKER"
fi

# --- Build ROS 2 workspace (only if install/ is missing) ---
if [ -d "$WS/src" ] && [ "$(ls -A $WS/src 2>/dev/null)" ] && [ ! -f "$WS/install/setup.bash" ]; then
    echo "[ros2] Building workspace..."
    cd "$WS"
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release || true
fi
if [ -f "$WS/install/setup.bash" ]; then
    source "$WS/install/setup.bash"
fi

echo "[ros2] ROS 2 Humble ready. ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"

# Keep container alive — robot/sensor nodes are launched dynamically
exec bash -c "echo '[ros2] Waiting for commands...'; sleep infinity"
