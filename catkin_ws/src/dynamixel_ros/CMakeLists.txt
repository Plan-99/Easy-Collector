cmake_minimum_required(VERSION 3.0.2)
project(dynamixel_ros)

## (중요!) find_package에 message_generation이 포함되어야 합니다.
find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
)

## (중요!) add_message_files 섹션
add_message_files(
  FILES
  DynamixelData.msg
)

## (중요!) generate_messages 섹션
generate_messages(
  DEPENDENCIES
  std_msgs
)

## (중요!) catkin_package에 CATKIN_DEPENDS로 message_runtime이 포함되어야 합니다.
catkin_package(
  CATKIN_DEPENDS message_runtime rospy std_msgs
)

## 실행 파일들을 포함시킵니다.
catkin_install_python(PROGRAMS
  scripts/dynamixel_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)