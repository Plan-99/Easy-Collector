"""Combined launch for the peg-in-hole tutorial benchmark: MuJoCo world
(scene_peg.xml) + tutorial_planner_node.

Used by Model Tester so both nodes come up under a single ROS daemon and
the planner has its joint_states / object_poses subscriptions ready before
the data collector starts triggering ``/tutorial/run_episode``.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = FindPackageShare("mujoco_world")

    scene_default = PathJoinSubstitution([pkg_share, "assets", "scene_peg.xml"])

    return LaunchDescription([
        DeclareLaunchArgument(
            "scene_xml", default_value=scene_default,
            description="MJCF scene file (defaults to peg-in-hole)."),
        DeclareLaunchArgument(
            "randomize_ranges",
            default_value='{"peg":{"x":[0.25,0.40],"y":[-0.05,0.10]},'
                          '"hole_base":{"x":[0.25,0.40],"y":[-0.15,-0.05],"yaw":[-0.3,0.3]}}',
            description="Per-body sampling JSON. See mujoco_world.launch.py."),
        DeclareLaunchArgument(
            "show_viewer", default_value="false",
            description="Open native MuJoCo viewer? Default false for headless eval."),
        DeclareLaunchArgument("realtime_factor", default_value="1.0"),
        DeclareLaunchArgument("plan_hz", default_value="20.0"),
        DeclareLaunchArgument("max_step_rad", default_value="0.05"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_share, "launch", "mujoco_world.launch.py"])
            ),
            launch_arguments={
                "scene_xml": LaunchConfiguration("scene_xml"),
                "randomize_ranges": LaunchConfiguration("randomize_ranges"),
                "show_viewer": LaunchConfiguration("show_viewer"),
                "realtime_factor": LaunchConfiguration("realtime_factor"),
                # peg-in-hole 데이터는 320×240 으로 모은다 (학습 시간 단축).
                # 실시간 publish 해상도가 이미 작아져서 collector / eval bridge
                # 모두 동일한 stream 을 그대로 쓸 수 있다.
                "image_width": "320",
                "image_height": "240",
                # round_1: 2-cam wrist-only mount. wrist_cam (forward+down) +
                # wrist_cam_down (straight-down at grasp footprint). Forces the
                # policy to triangulate XY from the gripper-relative views
                # instead of relying on a fixed top_cam frame.
                "cameras": "wrist_cam,wrist_cam_down",
            }.items(),
        ),

        Node(
            package="mujoco_world",
            executable="tutorial_planner_node",
            name="tutorial_planner_node",
            output="screen",
            parameters=[{
                "topic_prefix": "/tutorial",
                "plan_hz": LaunchConfiguration("plan_hz"),
                "max_step_rad": LaunchConfiguration("max_step_rad"),
            }],
        ),
    ])
