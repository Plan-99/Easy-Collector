# -*- coding: utf-8 -*-
"""
dual_arm_assembly_test mode defaults.

TWO independent role='single_arm' robots on SEPARATE topic prefixes, sharing
ONE MuJoCo physics sim (mujoco_world dual_arm_assembly_world.launch.py hosts
both via the node's `groups` param). They are combined with an Assembly
(left_arm + right_arm), verifying the multi-robot 양팔 control path
(two agents, two single-EE IK solvers, coordinated keyboard / pendant control).

  left  robot -> /da_asm_left   joints [left_joint1..6, left_gripper]   + top_cam
  right robot -> /da_asm_right  joints [right_joint1..6, right_gripper]  + front_cam
"""

_ENV = 'dual_arm_assembly_test'
_RESET_PREFIX = '/dual_arm_assembly_test'
_LEFT = '/da_asm_left'
_RIGHT = '/da_asm_right'

_ARM_LOWER = [-3.14, -1.57, -2.40, -3.14, -1.40, -3.14, 0.0]
_ARM_UPPER = [3.14, 1.57, 0.50, 3.14, 1.40, 3.14, 0.04]
_HOME_7 = [0.0, 1.21, -1.70, 0.0, -1.08, 0.0, 0.04]


def _arm_robot(slug, prefix, joint_prefix, urdf):
    jn = [f'{joint_prefix}_joint{i}' for i in range(1, 7)] + [f'{joint_prefix}_gripper']
    return {
        'name': f'dual_arm_assembly_{slug}',
        'type': 'custom',
        'homepose': [],
        'settings': {
            'sim_test_env': _ENV,
            'sim_test_slug': slug,
            'is_sim': True,
            'role': 'single_arm',
            'read_topic': f'{prefix}/joint_states',
            'read_topic_msg': 'sensor_msgs/JointState',
            'write_type': 'topic',
            'write_topic': f'{prefix}/joint_command',
            'write_topic_msg': 'sensor_msgs/JointState',
            'joint_names': jn,
            'joint_lower_bounds': _ARM_LOWER,
            'joint_upper_bounds': _ARM_UPPER,
            'tool_index': [6],
            'tool_inner': True,
            'interpolation': False,
            'urdf_path': f'/root/ros2_ws/src/mujoco_world/urdf/{urdf}',
            'urdf_package_dir': '/root/ros2_ws/src/mujoco_world/',
            'ik_setting': {
                'joints_to_lock': [f'{joint_prefix}_gripper', f'{joint_prefix}_gripper_mirror'],
                'ee_definitions': [['ee', f'{joint_prefix}_joint6', [0.095, 0.0, 0.0]]],
            },
        },
    }


SPEC = {
    'env': _ENV,
    'launch': {
        'process_id': 'dual_arm_assembly_test_sim',
        'package': 'mujoco_world',
        'launch_file': 'dual_arm_assembly_world.launch.py',
        'topic_prefix': _RESET_PREFIX,
        'reset_prefix': _RESET_PREFIX,
        'extra_args': {},
    },
    'robots': [
        _arm_robot('left_arm', _LEFT, 'left', 'dual_arm_left.urdf'),
        _arm_robot('right_arm', _RIGHT, 'right', 'dual_arm_right.urdf'),
    ],
    'sensors': [
        {
            'name': 'dual_arm_assembly_top_cam',
            'type': 'custom',
            'settings': {
                'sim_test_env': _ENV,
                'sim_test_slug': 'top_cam',
                'read_topic': f'{_LEFT}/top_cam/image_raw/compressed',
                'read_topic_msg': 'sensor_msgs/CompressedImage',
                'resolution': [640, 480],
            },
        },
        {
            'name': 'dual_arm_assembly_front_cam',
            'type': 'custom',
            'settings': {
                'sim_test_env': _ENV,
                'sim_test_slug': 'front_cam',
                'read_topic': f'{_RIGHT}/front_cam/image_raw/compressed',
                'read_topic_msg': 'sensor_msgs/CompressedImage',
                'resolution': [640, 480],
            },
        },
    ],
    'assembly': {
        'name': 'dual_arm_assembly_test_assembly',
        'left_slug': 'left_arm',
        'right_slug': 'right_arm',
        'left_tool_slug': None,
        'right_tool_slug': None,
    },
    'workspace': {
        'name': 'dual_arm_assembly_test',
        'episode_len': 200,
        'home_poses': {'left_arm': _HOME_7, 'right_arm': _HOME_7},
        'default_sensor_cfg': {
            'img_size': [320, 240],
            'cropped_area': [0, 0, 640, 480],
            'rotate': 0,
        },
    },
}
