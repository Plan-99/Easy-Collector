#!/usr/bin/env python3
#
# EasyTrainer bringup for ROBOTIS AI Worker FFW-SG2 (dual 7-DOF arm + gripper).
#
# Runs INSIDE the AI Worker's onboard docker container (`ai_worker`). Deployed to
# the robot via the module's driver.remote.payload (scp), then launched over SSH.
#
# Differs from the stock ffw_bg2_follower_ai.launch.py:
#   - Spawns a SINGLE `easytrainer_arm_controller` (JointTrajectoryController over
#     both arms, 16 joints) instead of separate arm_l/arm_r/head/lift controllers,
#     so EasyTrainer can drive the whole robot from one /…/joint_trajectory topic.
#   - Drops rviz / cameras / homing executors / head-eef-tracker.
# Reuses the stock `ffw_description` URDF (present in the container image).
#
# The controller yaml is loaded as a sibling file (both are scp'd into the same
# directory on the robot).

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

_HERE = os.path.dirname(os.path.realpath(__file__))
_CONTROLLER_YAML = os.path.join(_HERE, 'ffw_sg2_easytrainer_controller.yaml')


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('use_sim', default_value='false'),
        DeclareLaunchArgument('use_mock_hardware', default_value='false',
                              description='Use mock hardware mirroring command.'),
        DeclareLaunchArgument('mock_sensor_commands', default_value='false'),
        DeclareLaunchArgument('port_name', default_value='/dev/follower',
                              description='Hardware port for the FFW follower bus.'),
        DeclareLaunchArgument('model', default_value='ffw_sg2_rev1_follower',
                              description='Robot model name (ffw_description urdf folder).'),
        DeclareLaunchArgument('ros2_control_type', default_value='ffw_sg2_follower'),
        DeclareLaunchArgument('init_position_file',
                              default_value='ffw_sg2_follower_initial_positions.yaml'),
    ]

    use_sim = LaunchConfiguration('use_sim')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    mock_sensor_commands = LaunchConfiguration('mock_sensor_commands')
    port_name = LaunchConfiguration('port_name')
    model = LaunchConfiguration('model')
    ros2_control_type = LaunchConfiguration('ros2_control_type')
    init_position_file = LaunchConfiguration('init_position_file')

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([FindPackageShare('ffw_description'),
                              'urdf', model, 'ffw_sg2_follower.urdf.xacro']),
        ' ', 'use_sim:=', use_sim,
        ' ', 'use_mock_hardware:=', use_mock_hardware,
        ' ', 'mock_sensor_commands:=', mock_sensor_commands,
        ' ', 'port_name:=', port_name,
        ' ', 'model:=', model,
        ' ', 'init_position_file:=', init_position_file,
        ' ', 'ros2_control_type:=', ros2_control_type,
    ])
    robot_description = {'robot_description': robot_description_content}

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, _CONTROLLER_YAML],
        output='both',
        condition=UnlessCondition(use_sim),
    )

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': use_sim}],
        output='both',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='both',
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['easytrainer_arm_controller'],
        output='both',
    )

    return LaunchDescription(
        declared_arguments + [
            control_node,
            robot_state_pub_node,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
        ]
    )
