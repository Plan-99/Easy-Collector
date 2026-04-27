"""Launch file for the MuJoCo tutorial world ROS2 node."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "topic_prefix", default_value="/tutorial",
            description="Prefix for all topics published/subscribed by the sim."),
        DeclareLaunchArgument(
            "scene_xml", default_value="",
            description="Override path to MJCF scene. Empty = use packaged default."),
        DeclareLaunchArgument(
            "image_width", default_value="640"),
        DeclareLaunchArgument(
            "image_height", default_value="480"),
        DeclareLaunchArgument(
            "state_publish_hz", default_value="50.0"),
        DeclareLaunchArgument(
            "image_publish_hz", default_value="20.0"),
        DeclareLaunchArgument(
            "realtime_factor", default_value="1.0"),
        DeclareLaunchArgument(
            "show_viewer", default_value="true",
            description="Open the native MuJoCo viewer window (requires X/OpenGL)."),

        Node(
            package="mujoco_world",
            executable="mujoco_world_node",
            name="mujoco_world_node",
            output="screen",
            parameters=[{
                "topic_prefix": LaunchConfiguration("topic_prefix"),
                "scene_xml": LaunchConfiguration("scene_xml"),
                "image_width": LaunchConfiguration("image_width"),
                "image_height": LaunchConfiguration("image_height"),
                "state_publish_hz": LaunchConfiguration("state_publish_hz"),
                "image_publish_hz": LaunchConfiguration("image_publish_hz"),
                "realtime_factor": LaunchConfiguration("realtime_factor"),
                "show_viewer": LaunchConfiguration("show_viewer"),
            }],
        ),
    ])
