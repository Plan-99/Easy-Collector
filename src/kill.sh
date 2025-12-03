pkill -9 -f backend.api
pkill -9 -f rosbridge
pkill -9 -f realsense
pkill -9 -f piper
pkill -9 -f rosapi_node
pkill -9 -f ros2
pkill -9 -f robot_state_publisher
pkill -9 -f joint_state_publisher
pkill -9 -f rviz2

ros2 daemon stop