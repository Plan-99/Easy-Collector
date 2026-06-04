# -*- coding: utf-8 -*-
"""
dual_arm_test mode defaults.

A single MuJoCo robot with role='dual_arm' (14 joints) published on ONE topic
prefix (/dual_arm_test), backed by ros2_ws/src/mujoco_world/assets/
dual_arm_scene.xml + urdf/dual_arm.urdf. Verifies the humanoid-style양팔
control path (one agent, midpoint joint split, dual-EE IK).

14-joint contract (matches dual_arm_world.launch.py joint_names):
    [ left_joint1..6, left_gripper,  right_joint1..6, right_gripper ]
mid=7, tool_inner=True, tool_index=[6,13] -> per-arm gripper as tool.
"""

_PREFIX = '/dual_arm_test'
_ENV = 'dual_arm_test'

_ARM_LOWER = [-3.14, -1.57, -2.40, -3.14, -1.40, -3.14]
_ARM_UPPER = [3.14, 1.57, 0.50, 3.14, 1.40, 3.14]
_GRIP_LOWER = [0.0]
_GRIP_UPPER = [0.04]

_JOINT_NAMES = [
    'left_joint1', 'left_joint2', 'left_joint3',
    'left_joint4', 'left_joint5', 'left_joint6', 'left_gripper',
    'right_joint1', 'right_joint2', 'right_joint3',
    'right_joint4', 'right_joint5', 'right_joint6', 'right_gripper',
]

_ARM_HOME = [0.0, 1.21, -1.70, 0.0, -1.08, 0.0]
_HOME_14 = _ARM_HOME + [0.04] + _ARM_HOME + [0.04]

SPEC = {
    'env': _ENV,
    'launch': {
        'process_id': 'dual_arm_test_sim',
        'package': 'mujoco_world',
        'launch_file': 'dual_arm_world.launch.py',
        'topic_prefix': _PREFIX,
        'reset_prefix': _PREFIX,
        'extra_args': {},
    },
    'robots': [
        {
            'name': 'dual_arm_test_robot',
            'type': 'custom',
            'homepose': [],
            'settings': {
                'sim_test_env': _ENV,
                'sim_test_slug': 'dual_arm',
                'is_sim': True,
                'role': 'dual_arm',
                'read_topic': f'{_PREFIX}/joint_states',
                'read_topic_msg': 'sensor_msgs/JointState',
                'write_type': 'topic',
                'write_topic': f'{_PREFIX}/joint_command',
                'write_topic_msg': 'sensor_msgs/JointState',
                'joint_names': _JOINT_NAMES,
                'joint_lower_bounds': _ARM_LOWER + _GRIP_LOWER + _ARM_LOWER + _GRIP_LOWER,
                'joint_upper_bounds': _ARM_UPPER + _GRIP_UPPER + _ARM_UPPER + _GRIP_UPPER,
                'tool_index': [6, 13],
                'tool_inner': True,
                'interpolation': False,
                'urdf_path': '/root/ros2_ws/src/mujoco_world/urdf/dual_arm.urdf',
                'urdf_package_dir': '/root/ros2_ws/src/mujoco_world/',
                'ik_setting': {
                    'joints_to_lock': [
                        'left_gripper', 'left_gripper_mirror',
                        'right_gripper', 'right_gripper_mirror',
                    ],
                    'ee_definitions': [
                        ['L_ee', 'left_joint6', [0.095, 0.0, 0.0]],
                        ['R_ee', 'right_joint6', [0.095, 0.0, 0.0]],
                    ],
                },
            },
        },
    ],
    'sensors': [
        {
            'name': 'dual_arm_test_top_cam',
            'type': 'custom',
            'settings': {
                'sim_test_env': _ENV,
                'sim_test_slug': 'top_cam',
                'read_topic': f'{_PREFIX}/top_cam/image_raw/compressed',
                'read_topic_msg': 'sensor_msgs/CompressedImage',
                'resolution': [640, 480],
            },
        },
        {
            'name': 'dual_arm_test_front_cam',
            'type': 'custom',
            'settings': {
                'sim_test_env': _ENV,
                'sim_test_slug': 'front_cam',
                'read_topic': f'{_PREFIX}/front_cam/image_raw/compressed',
                'read_topic_msg': 'sensor_msgs/CompressedImage',
                'resolution': [640, 480],
            },
        },
    ],
    # role='dual_arm' robot fills BOTH assembly slots with the same robot id
    # (mirrors AssemblyForm.vue behavior for a single dual-arm robot).
    'assembly': {
        'name': 'dual_arm_test_assembly',
        'left_slug': 'dual_arm',
        'right_slug': 'dual_arm',
        'left_tool_slug': None,
        'right_tool_slug': None,
    },
    'workspace': {
        'name': 'dual_arm_test',
        'episode_len': 200,
        'home_poses': {'dual_arm': _HOME_14},
        'default_sensor_cfg': {
            'img_size': [320, 240],
            'cropped_area': [0, 0, 640, 480],
            'rotate': 0,
        },
    },
}
