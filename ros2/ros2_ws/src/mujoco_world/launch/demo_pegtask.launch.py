"""Demo peg-in-hole launch — wrist-only 2-camera setup for the demo/ catalog.

Differs from tutorial_pegtask.launch.py: publishes ONLY the two wrist cameras
(wrist_cam, wrist_cam_down) instead of top/front/wrist. This matches the demo
workspace (tutorial_wrist_cam + tutorial_wrist_cam_2) and — because it renders
two cameras instead of three-plus-depth — keeps the single-threaded sim loop
from starving the physics step, so the motion planner tracks reliably.

Launches mujoco_world_node directly (cameras passed as a real list) + the
tutorial_planner_node that serves /tutorial/run_episode.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("mujoco_world")
    scene_default = os.path.join(pkg_share, "assets", "scene_peg.xml")

    return LaunchDescription([
        DeclareLaunchArgument("scene_xml", default_value=scene_default),
        DeclareLaunchArgument(
            "show_viewer", default_value="false",
            description="Open native MuJoCo viewer? Default false (headless)."),
        DeclareLaunchArgument("realtime_factor", default_value="1.0"),
        DeclareLaunchArgument("plan_hz", default_value="20.0"),
        DeclareLaunchArgument("max_step_rad", default_value="0.01"),

        Node(
            package="mujoco_world",
            executable="mujoco_world_node",
            name="mujoco_world_node",
            output="screen",
            parameters=[{
                "topic_prefix": "/tutorial",
                "scene_xml": LaunchConfiguration("scene_xml"),
                # Wrist-only 2-cam setup (passed as a real list).
                "cameras": ["wrist_cam", "wrist_cam_down"],
                # peg-in-hole 데이터는 200×150 으로 모은다 (학습 시간 단축 + 손목캠 근접뷰).
                "image_width": 200,
                "image_height": 150,
                # Lower render rate so the single-threaded sim loop keeps physics
                # at real time under data-collection load (the planner's joint
                # ramp must not out-run the arm — see demo/RESULTS.md).
                "image_publish_hz": 15.0,
                "show_viewer": LaunchConfiguration("show_viewer"),
                "realtime_factor": LaunchConfiguration("realtime_factor"),
                "randomize_ranges":
                    '{"peg":{"x":[0.25,0.40],"y":[-0.05,0.10]},'
                    '"hole_base":{"x":[0.25,0.40],"y":[-0.15,-0.05],"yaw":[-0.3,0.3]}}',
            }],
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
