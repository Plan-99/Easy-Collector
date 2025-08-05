cmake_minimum_required(VERSION 3.8)
project(dynamixel_ros)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DynamixelData.msg"
)

install(
    PROGRAMS
    "dynamixel_ros/dynamixel_node.py"
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)


ament_export_dependencies(rclpy std_msgs rosidl_default_runtime)
ament_package()