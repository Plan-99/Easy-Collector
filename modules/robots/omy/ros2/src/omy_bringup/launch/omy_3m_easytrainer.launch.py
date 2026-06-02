#!/usr/bin/env python3
#
# EasyTrainer launch wrapper for ROBOTIS OMY-3M (6-DOF arm, no gripper).
#
# Differences from the upstream omy_3m.launch.py:
#   - Spawns a single `joint_group_position_controller` (Float64MultiArray
#     over /joint_group_position_controller/commands) covering joint1-6, so
#     EasyTrainer can drive the whole arm from one topic.
#   - Drops the joint_trajectory_executor home-pose helper (EasyTrainer
#     handles homing on its own).
#   - Drops rviz/gazebo plumbing.
#

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'prefix',
            default_value='""',
            description='Prefix of the joint and link names',
        ),
        DeclareLaunchArgument(
            'use_mock_hardware',
            default_value='false',
            description='Use mock hardware mirroring command.',
        ),
        DeclareLaunchArgument(
            'port_name',
            default_value='/dev/ttyAMA2',
            description='Serial port of the OMY arm Dynamixel bus.',
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='ROS namespace for this robot instance.',
        ),
    ]

    prefix = LaunchConfiguration('prefix')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    port_name = LaunchConfiguration('port_name')
    namespace = LaunchConfiguration('namespace')

    urdf_file = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('omy_description'),
            'urdf',
            'omy_3m',
            'omy_3m.urdf.xacro',
        ]),
        ' ',
        'prefix:=',
        prefix,
        ' ',
        'use_sim:=false',
        ' ',
        'use_mock_hardware:=',
        use_mock_hardware,
        ' ',
        'mock_sensor_commands:=false',
        ' ',
        'port_name:=',
        port_name,
        ' ',
        'ros2_control_type:=omy_3m_position',
    ])

    controller_manager_config = PathJoinSubstitution([
        FindPackageShare('omy_bringup'),
        'config',
        'omy_3m',
        'easytrainer_controller_manager.yaml',
    ])

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace=namespace,
        parameters=[{'robot_description': urdf_file}, controller_manager_config],
        output='both',
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        parameters=[{'robot_description': urdf_file}],
        output='both',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        arguments=['joint_state_broadcaster'],
        output='both',
    )

    joint_group_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        arguments=['joint_group_position_controller'],
        output='both',
    )

    return LaunchDescription(
        declared_arguments
        + [
            control_node,
            robot_state_publisher_node,
            joint_state_broadcaster_spawner,
            joint_group_position_controller_spawner,
        ]
    )
