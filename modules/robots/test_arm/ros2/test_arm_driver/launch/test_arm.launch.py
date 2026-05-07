"""Virtual test arm launch file.

ros2 launch test_arm_driver test_arm.launch.py namespace:=ec_robot_42 publish_rate:=200.0
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('publish_rate', default_value='200.0'),
        Node(
            package='test_arm_driver',
            executable='virtual_arm_node',
            name='virtual_arm_node',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[
                {'publish_rate': LaunchConfiguration('publish_rate')},
            ],
        ),
    ])
