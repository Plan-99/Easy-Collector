"""Launch file for the MuJoCo dual-arm test world (single-topic / humanoid-style).

Hosts assets/dual_arm_scene.xml as ONE robot (14 joints) on one topic prefix.
The joint_names list IS the contract for the role='dual_arm' robot row:
  [ L1..L6, Lgrip, R1..R6, Rgrip ]  (mid=7, tool_inner per arm)

Unlike mujoco_world.launch.py, this forwards joint_names/cameras to the node
(the generic launch file does not), so no node code change is needed.
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


DUAL_ARM_JOINT_NAMES = [
    "left_joint1", "left_joint2", "left_joint3",
    "left_joint4", "left_joint5", "left_joint6", "left_gripper",
    "right_joint1", "right_joint2", "right_joint3",
    "right_joint4", "right_joint5", "right_joint6", "right_gripper",
]
DUAL_ARM_CAMERAS = ["top_cam", "front_cam"]


def generate_launch_description():
    scene_default = os.path.join(
        get_package_share_directory("mujoco_world"), "assets", "dual_arm_scene.xml"
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "topic_prefix", default_value="/dual_arm_test",
            description="Prefix for all topics of the dual-arm robot."),
        DeclareLaunchArgument(
            "scene_xml", default_value=scene_default,
            description="MJCF scene. Defaults to the packaged dual_arm_scene.xml."),
        DeclareLaunchArgument("image_width", default_value="640"),
        DeclareLaunchArgument("image_height", default_value="480"),
        DeclareLaunchArgument("state_publish_hz", default_value="50.0"),
        DeclareLaunchArgument("image_publish_hz", default_value="20.0"),
        DeclareLaunchArgument("realtime_factor", default_value="1.0"),
        DeclareLaunchArgument(
            "show_viewer", default_value="true",
            description="Open native MuJoCo viewer; falls back to headless on X11 failure."),

        Node(
            package="mujoco_world",
            executable="mujoco_world_node",
            name="mujoco_world_node",
            output="screen",
            parameters=[{
                "topic_prefix": LaunchConfiguration("topic_prefix"),
                "scene_xml": LaunchConfiguration("scene_xml"),
                "joint_names": DUAL_ARM_JOINT_NAMES,
                "cameras": DUAL_ARM_CAMERAS,
                "image_width": LaunchConfiguration("image_width"),
                "image_height": LaunchConfiguration("image_height"),
                "state_publish_hz": LaunchConfiguration("state_publish_hz"),
                "image_publish_hz": LaunchConfiguration("image_publish_hz"),
                "realtime_factor": LaunchConfiguration("realtime_factor"),
                "show_viewer": LaunchConfiguration("show_viewer"),
            }],
        ),
    ])
