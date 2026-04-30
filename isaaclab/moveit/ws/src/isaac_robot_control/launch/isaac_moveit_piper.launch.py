import os
import yaml
from collections.abc import Mapping
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def load_yaml(package_name, file_path):
    package_dir = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_dir, file_path)
    with open(absolute_file_path, "r") as f:
        return yaml.safe_load(f)

def _sanitize(obj):
    """모든 tuple을 list로, MappingProxy를 dict로 변환하여 ROS 2 파라미터 호환성을 보장합니다."""
    if isinstance(obj, Mapping):
        return {str(k): _sanitize(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_sanitize(v) for v in obj]
    return obj

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("piper", package_name="piper_with_gripper_moveit")
        .robot_description(file_path="config/piper.urdf.xacro")
        .robot_description_semantic(file_path="config/piper.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    our_pkg_share = get_package_share_directory("isaac_robot_control")
    our_controllers = load_yaml("isaac_robot_control", "config/piper_moveit_controllers.yaml")

    # SRDF 오버라이드 (end_effector 정의 추가)
    srdf_path = os.path.join(our_pkg_share, "config", "piper.srdf")
    with open(srdf_path, "r") as f:
        srdf_content = f.read()

    robot_description = _sanitize(moveit_config.robot_description)
    robot_description_semantic = {"robot_description_semantic": srdf_content}
    robot_description_kinematics = _sanitize(moveit_config.robot_description_kinematics)
    # moveit_config.planning_pipelines를 직접 사용 (planning_plugins 복수형 포함)
    planning_params = _sanitize(moveit_config.planning_pipelines)
    joint_limits = _sanitize(moveit_config.joint_limits)

    move_group_params = [
        robot_description,
        robot_description_semantic,
        robot_description_kinematics,
        joint_limits,
        planning_params,
        {
            "use_sim_time": True,
            "moveit_simple_controller_manager": _sanitize(our_controllers["moveit_simple_controller_manager"]),
            "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
            "trajectory_execution.allowed_start_tolerance": 0.05,
        },
    ]

    # IsaacSim 토픽은 /simulation 네임스페이스 아래 퍼블리시됨
    sim_remappings = [
        ("/joint_states", "/simulation/joint_states"),
        ("/tf", "/simulation/tf"),
        ("/clock", "/simulation/clock"),
    ]

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description, {"use_sim_time": True}],
            remappings=sim_remappings,
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=move_group_params,
            remappings=sim_remappings,
        ),
        Node(
            package="isaac_robot_control",
            executable="trajectory_bridge_piper",
            output="screen",
            parameters=[{"use_sim_time": True}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", os.path.join(our_pkg_share, "config", "piper_moveit.rviz")],
            parameters=[
                robot_description,
                robot_description_semantic,
                robot_description_kinematics,
                {"use_sim_time": True},
            ],
            remappings=sim_remappings,
        ),
    ]

    return LaunchDescription(nodes)