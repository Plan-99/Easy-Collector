KEEP_BACKEND=0
if [[ "${EC_KILL_BACKEND:-1}" == "0" ]]; then
  KEEP_BACKEND=1
fi
if [[ "${1:-}" == "--keep-backend" ]]; then
  KEEP_BACKEND=1
fi

if [[ "${KEEP_BACKEND}" != "1" ]]; then
  pkill -9 -f backend.api
fi
pkill -9 -f rosbridge
pkill -9 -f realsense
pkill -9 -f piper
pkill -9 -f rosapi_node
pkill -9 -f ros2
pkill -9 -f robot_state_publisher
pkill -9 -f joint_state_publisher
pkill -9 -f rviz2

ros2 daemon stop
