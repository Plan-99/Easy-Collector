from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for the node.'
        ),
        DeclareLaunchArgument(
            'device_index',
            default_value='0',
            description='Index of the webcam device.'
        ),
        Node(
            package='webcam_publisher',
            executable='webcam_publisher_node',
            name='webcam_publisher_node',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[{
                'device_index': LaunchConfiguration('device_index')
            }]
        ),
    ])
