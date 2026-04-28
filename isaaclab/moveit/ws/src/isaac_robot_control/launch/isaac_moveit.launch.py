"""
MoveIt2 + IsaacSim 연동 Launch 파일 (Franka Panda).

사용법 (MoveIt2 Docker 컨테이너 내부):
    ros2 launch isaac_robot_control isaac_moveit.launch.py
"""

from moveit_configs_utils import MoveItConfigsBuilder
from isaac_robot_control.launch_common import generate_isaac_moveit_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .robot_description_semantic(file_path="config/panda.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    return generate_isaac_moveit_launch(
        moveit_config=moveit_config,
        controller_yaml="config/moveit_controllers.yaml",
        bridge_executable="trajectory_bridge",
        rviz_config="config/moveit.rviz",
    )
