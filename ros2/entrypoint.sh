#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash

WS="/root/ros2_ws"

# --- Install module dependencies (skip if already done) ---
DEPS_MARKER="/tmp/.ros2_deps_installed"
if [ -d "$WS/src" ] && [ "$(ls -A $WS/src 2>/dev/null)" ] && [ ! -f "$DEPS_MARKER" ]; then
    for mj in "$WS"/src/*/module.json; do
        [ -f "$mj" ] || continue
        pkg_name=$(basename $(dirname "$mj"))
        # pip deps (check if already importable)
        pip_deps=$(python3 -c "import json; deps=json.load(open('$mj')).get('dependencies',{}).get('pip',[]); print(' '.join(deps))" 2>/dev/null)
        if [ -n "$pip_deps" ]; then
            echo "[ros2] Checking pip deps for $pkg_name..."
            python3 -m pip install --quiet $pip_deps 2>/dev/null || true
        fi
        # apt deps
        apt_deps=$(python3 -c "import json; deps=json.load(open('$mj')).get('dependencies',{}).get('apt',[]); print(' '.join(deps))" 2>/dev/null)
        if [ -n "$apt_deps" ]; then
            echo "[ros2] Checking apt deps for $pkg_name..."
            apt-get install -y --no-install-recommends $apt_deps 2>/dev/null || true
        fi
        # custom install script
        install_script=$(python3 -c "import json; print(json.load(open('$mj')).get('install_script',''))" 2>/dev/null)
        if [ -n "$install_script" ]; then
            script_path="$(dirname $mj)/$install_script"
            if [ -f "$script_path" ]; then
                echo "[ros2] Running install script for $pkg_name..."
                bash "$script_path" || true
            fi
        fi
    done
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
