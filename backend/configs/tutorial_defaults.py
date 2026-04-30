# -*- coding: utf-8 -*-
"""
Tutorial mode defaults.

The tutorial mode bundles a MuJoCo world (ros2/ros2_ws/src/mujoco_world) that
publishes ROS2 topics for a virtual robot and camera. Tutorial robot/sensor
DB rows are stored with `type='custom'` so the rest of EasyTrainer treats
them like any other external-topic device — no driver process is launched
on its behalf, the MuJoCo node is the source.

`is_tutorial` is added to settings so the frontend can render a distinct
badge / disable destructive actions for these seeded rows.
"""

# ROS2 launch coordinates for the bundled sim
TUTORIAL_LAUNCH_PROCESS_ID = 'tutorial_sim'
TUTORIAL_LAUNCH_PACKAGE = 'mujoco_world'
TUTORIAL_LAUNCH_FILE = 'mujoco_world.launch.py'
TUTORIAL_TOPIC_PREFIX = '/tutorial'


# Robot row: 6DOF arm + 1DOF gripper, joint convention matches Piper.
TUTORIAL_ROBOT = {
    'name': 'tutorial_arm',
    'type': 'custom',
    'homepose': [],
    'settings': {
        'is_tutorial': True,
        'role': 'single_arm',
        'read_topic': f'{TUTORIAL_TOPIC_PREFIX}/joint_states',
        'read_topic_msg': 'sensor_msgs/JointState',
        'write_type': 'topic',
        'write_topic': f'{TUTORIAL_TOPIC_PREFIX}/joint_command',
        'write_topic_msg': 'sensor_msgs/JointState',
        'joint_names': ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper'],
        'joint_lower_bounds': [-3.14, -1.57, -2.40, -3.14, -1.40, -3.14, 0.0],
        'joint_upper_bounds': [3.14, 1.57, 0.50, 3.14, 1.40, 3.14, 0.04],
        'tool_index': [6],
        'tool_inner': True,
        'interpolation': False,
        # Task-space (IK) support: kinematics mirror assets/scene.xml.
        # End-effector frame = link6 + (0.045 + 0.05) along x = 0.095 from joint6.
        'urdf_path': '/root/ros2_ws/src/mujoco_world/urdf/tutorial_arm.urdf',
        'urdf_package_dir': '/root/ros2_ws/src/mujoco_world/',
        'ik_setting': {
            'joints_to_lock': ['gripper', 'gripper_mirror'],
            'ee_definitions': [['ee', 'joint6', [0.095, 0.0, 0.0]]],
        },
    },
}


# Sensor row: overhead RGB camera looking at the table.
TUTORIAL_SENSOR = {
    'name': 'tutorial_camera',
    'type': 'custom',
    'settings': {
        'is_tutorial': True,
        'read_topic': f'{TUTORIAL_TOPIC_PREFIX}/camera/image_raw/compressed',
        'read_topic_msg': 'sensor_msgs/CompressedImage',
        'resolution': [640, 480],
    },
}
