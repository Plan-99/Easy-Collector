import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare a launch argument for the robot namespace
    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='jaka_zu12',  # Default namespace for Jaka
        description='Namespace for the robot nodes (e.g., jaka_zu12)'
    )

    # Declare the 'ip' argument
    ip_arg = DeclareLaunchArgument('ip', default_value='127.0.0.1', description='IP address')

    return LaunchDescription([
        robot_namespace_arg,
        ip_arg,

        # Print the IP and namespace to the log for debugging
        LogInfo(
            msg=["The IP address is: ", LaunchConfiguration('ip')]
        ),
        LogInfo(
            msg=["The robot namespace is: ", LaunchConfiguration('robot_namespace')]
        ),

        # Launch the 'jaka_driver' node with the provided 'ip' parameter and namespace
        Node(
            package='jaka_driver',
            executable='jaka_driver',  # the executable to run
            name='jaka_driver',
            namespace=LaunchConfiguration('robot_namespace'), # Set the node's namespace
            output='screen',
            parameters=[{'ip': LaunchConfiguration('ip')}],
        ),
    ])
