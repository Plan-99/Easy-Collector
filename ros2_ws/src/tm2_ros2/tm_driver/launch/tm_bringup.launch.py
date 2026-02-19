from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.1.10',
            description='IP address of the robot'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for the node'
        ),
        Node(
            package='tm_driver',
            executable='tm_driver',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            arguments=[['robot_ip:=', LaunchConfiguration('robot_ip')]],
        )
    ])