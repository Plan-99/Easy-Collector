xhost +

DATA_ROOT=${EASYTRAINER_DATA_DIR:-/opt/easytrainer}
LOG_DIR="/tmp/easytrainer/logs"
CONFIG_PATH=${EASYTRAINER_CONFIG_PATH:-${DATA_ROOT}/config.json}

if ! mkdir -p "$LOG_DIR"; then
    sudo mkdir -p "$LOG_DIR"
fi

docker stop easy_collector_container 2>/dev/null || true

docker run -it --rm \
    --privileged \
    -v /dev:/dev \
    -v "./src":"/root/src" \
    -v "./ros2":"/root/ros2" \
    -v "./python_pkgs":"/root/python_pkgs" \
    -v "./requirements.txt":"/root/requirements.txt" \
    -v "${DATA_ROOT}":"${DATA_ROOT}" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=$DISPLAY \
    -e ROS_DOMAIN_ID=30 \
    --env "XAUTHORITY=$HOME/.Xauthority" \
    -e EASYTRAINER_DATA_DIR="${DATA_ROOT}" \
    -e EASYTRAINER_LOG_DIR="${LOG_DIR}" \
    -e EASYTRAINER_CONFIG_PATH="${CONFIG_PATH}" \
    --net=host \
    --name easy_collector_container easy-collector:latest
# To run a custom command instead of the default, append it to the line above.
