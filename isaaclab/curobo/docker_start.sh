docker run -it \
  --rm \
  --net=host \
  --privileged \
  --shm-size=4gb \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --env="NVIDIA_VISIBLE_DEVICES=all" \
  --env="NVIDIA_DRIVER_CAPABILITIES=all" \
  --env="RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" \
  --env="ROS_DOMAIN_ID=31" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$(pwd)/ws:/root/ws:rw" \
  --volume="$(dirname $(pwd))/isaac_control_core:/root/ws/src/isaac_control_core:rw" \
  --volume="$(dirname $(pwd))/ros_pkgs/piper_description:/root/ws/src/piper_description:rw" \
  --runtime=nvidia \
  --gpus all \
  --name curobo_dev \
  curobo-ros2:latest
