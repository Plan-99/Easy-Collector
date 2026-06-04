"""Launch file for the MuJoCo dual-arm ASSEMBLY test world.

ONE physics sim (assets/dual_arm_scene.xml), TWO topic namespaces so two
independent role='single_arm' robots can be assembled:
    /da_asm_left   <- left_joint1..6 + left_gripper   (+ top_cam)
    /da_asm_right  <- right_joint1..6 + right_gripper  (+ front_cam)

Both groups feed the same SimRunner via the node's `groups` parameter.
reset/reset_world services live under topic_prefix (/dual_arm_assembly_test).
"""
import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


LEFT_JOINTS = ["left_joint1", "left_joint2", "left_joint3",
               "left_joint4", "left_joint5", "left_joint6", "left_gripper"]
RIGHT_JOINTS = ["right_joint1", "right_joint2", "right_joint3",
                "right_joint4", "right_joint5", "right_joint6", "right_gripper"]

GROUPS = [
    {"topic_prefix": "/da_asm_left",  "joint_names": LEFT_JOINTS,  "cameras": ["top_cam"]},
    {"topic_prefix": "/da_asm_right", "joint_names": RIGHT_JOINTS, "cameras": ["front_cam"]},
]


def generate_launch_description():
    scene_default = os.path.join(
        get_package_share_directory("mujoco_world"), "assets", "dual_arm_scene.xml"
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "topic_prefix", default_value="/dual_arm_assembly_test",
            description="Prefix for the shared reset/reset_world services."),
        DeclareLaunchArgument(
            "scene_xml", default_value=scene_default,
            description="MJCF scene. Defaults to the packaged dual_arm_scene.xml."),
        DeclareLaunchArgument(
            "groups", default_value=json.dumps(GROUPS),
            description="JSON list of {topic_prefix, joint_names, cameras} groups."),
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
                # Force str so launch_ros doesn't try to auto-parse the JSON
                # array (its items are dicts -> "uniform type" error).
                "groups": ParameterValue(LaunchConfiguration("groups"), value_type=str),
                "image_width": LaunchConfiguration("image_width"),
                "image_height": LaunchConfiguration("image_height"),
                "state_publish_hz": LaunchConfiguration("state_publish_hz"),
                "image_publish_hz": LaunchConfiguration("image_publish_hz"),
                "realtime_factor": LaunchConfiguration("realtime_factor"),
                "show_viewer": LaunchConfiguration("show_viewer"),
            }],
        ),
    ])
