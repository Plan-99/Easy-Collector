cmake_minimum_required(VERSION 3.8)
project(custom_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED) # msg 내부에서 std_msgs를 쓴다면 필요

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DynamixelData.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()