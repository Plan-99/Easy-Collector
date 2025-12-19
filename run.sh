xhost +

mkdir -p ./logs

docker stop easy_collector_container

docker run -it --rm \
    --privileged \
    -v /dev:/dev \
    -v "./src":"/root/src" \
    -v "./ros2_ws":"/root/ros2_ws" \
    -v "./python_pkgs":"/root/python_pkgs" \
    -v "./requirements.txt":"/root/requirements.txt" \
    -v "./logs":"/root/easytrainer/logs" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=$DISPLAY \
    -e ROS_DOMAIN_ID=30 \
    --env "XAUTHORITY=$HOME/.Xauthority" \
    --net=host \
    --name easy_collector_container easy-collector:latest \
    # bash -c "cd .. && bash run_backend.sh"
