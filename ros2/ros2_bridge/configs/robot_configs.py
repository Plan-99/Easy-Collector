# -*- coding: utf-8 -*-
"""
ROS2 컨테이너 전용 로봇 설정.
URDF 경로, IK 설정, SDK 설정 등 로봇 제어에 필요한 정보만 포함.
UI/API 관련 정보 (company, module_id 등)는 backend/configs/global_configs.py에 있음.
"""
import numpy as np

ROBOT_CONFIGS = {
    'test_arm': {
        'urdf_path': '/root/ros2_ws/src/piper/piper_description/urdf/piper_description.urdf',
        'urdf_package_dir': '/root/ros2_ws/src/piper/piper_description/',
        'ik_setting': {
            'joints_to_lock': ['joint6_to_gripper_base', 'joint7', 'joint8'],
            'ee_definitions': [('ee', 'joint7', None)],
        },
    },
    'piper': {
        'urdf_path': '/root/ros2_ws/src/piper/piper_description/urdf/piper_description.urdf',
        'urdf_package_dir': '/root/ros2_ws/src/piper/piper_description/',
        'ik_setting': {
            'joints_to_lock': ['joint6_to_gripper_base', 'joint7', 'joint8'],
            'ee_definitions': [('ee', 'joint7', None)],
        },
    },
    'piper(no gripper)': {
        'urdf_path': '/root/ros2_ws/src/piper/piper_description/urdf/piper_no_gripper_description.urdf',
        'urdf_package_dir': '/root/ros2_ws/src/piper/piper_description/',
        'ik_setting': {
            'joints_to_lock': [],
            'ee_definitions': [('ee', 'joint6', np.array([0.20, 0, 0]).T)],
        },
    },
    'tm_12': {
        'urdf_path': '/root/ros2_ws/src/tm2_ros2/tm_description/urdf/tm12s.urdf',
        'urdf_package_dir': '/root/ros2_ws/src/tm2_ros2/tm_description/',
        'ik_setting': {
            'joints_to_lock': [],
            'ee_definitions': [('ee', 'joint_6', np.array([0, 0, 0.15]).T)],
        },
    },
    'tm_12s': {
        'urdf_path': '/root/ros2_ws/src/tm2_ros2/tm_description/urdf/tm12s.urdf',
        'urdf_package_dir': '/root/ros2_ws/src/tm2_ros2/tm_description/',
        'ik_setting': {
            'joints_to_lock': [],
            'ee_definitions': [('ee', 'joint_6', np.array([0, 0, 0.15]).T)],
        },
    },
    'tm_12_robotiq': {
        'urdf_path': '/root/ros2_ws/src/tm2_ros2/tm_description/urdf/tm12s.urdf',
        'urdf_package_dir': '/root/ros2_ws/src/tm2_ros2/tm_description/',
        'ik_setting': {
            'joints_to_lock': [],
            'ee_definitions': [('ee', 'joint_6', np.array([0, 0, 0.15]).T)],
        },
    },
    'rb3_730es_u': {
        'urdf_path': '/root/ros2_ws/src/rbpodo_ros2/rbpodo_description/robots/rb3_730es_u.urdf',
        'urdf_package_dir': '/root/ros2_ws/src/rbpodo_ros2/rbpodo_description/',
        'ik_setting': {
            'joints_to_lock': [],
            'ee_definitions': [('ee', 'tcp_joint', None)],
        },
    },
    'kinova_gen3_7dof_robotiq_2f_85': {
        'urdf_path': '/root/ros2_ws/src/ros2_kortex/kortex_description/robots/gen3_7dof.urdf',
        'urdf_package_dir': '/root/ros2_ws/src/ros2_kortex/kortex_description/',
        'ik_setting': {
            'joints_to_lock': [],
            'ee_definitions': [('ee', 'joint_7', np.array([0, 0, 0]).T)],
            'gravity_compensate': 0.00003,
        },
    },
    'fairino_fr5': {
        'urdf_path': '/root/ros2_ws/src/frcobot_ros2/fairino_description/urdf/fairino5_v6.urdf',
        'urdf_package_dir': '/root/ros2_ws/src/frcobot_ros2/fairino_description/',
        'ik_setting': {
            'joints_to_lock': [],
            'ee_definitions': [('ee', 'j6', np.array([0, 0, 0.1]).T)],
        },
    },
    'jaka_zu12': {
        'urdf_path': '/root/ros2_ws/src/jaka_ros2/src/jaka_description/urdf/jaka_zu12.urdf',
        'urdf_package_dir': '/root/ros2_ws/src/jaka_ros2/src/jaka_description/',
        'ik_setting': {
            'joints_to_lock': [],
            'ee_definitions': [('ee', 'joint_6', None)],
        },
    },
}


def get_robot_config(robot_type: str) -> dict | None:
    """로봇 타입으로 제어 설정을 조회."""
    return ROBOT_CONFIGS.get(robot_type)
