from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        Node(
            package='test_arm',
            executable='test_arm_node',
            name='test_arm_node',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[
                {'publish_rate': 50.0},
            ],
        ),
    ])
