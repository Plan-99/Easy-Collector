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
#
# 표준 sim 디바이스 `sim_arm` (sim_device_slug='arm'). 설치 시 로드되는 커스텀
# 로봇이며, 모든 단일팔 시뮬 환경이 이 디바이스를 리포인트해서 쓴다. 기본 바인딩은
# tutorial 환경(/tutorial 토픽, is_tutorial=True).
TUTORIAL_ROBOT = {
    'name': 'sim_arm',
    'type': 'custom',
    'homepose': [],
    'settings': {
        'sim_device_slug': 'arm',
        'is_tutorial': True,
        # is_sim=True를 줘야 Agent.move_to가 단발 publish + 짧은 sleep으로 끝나는
        # sim 분기로 떨어진다 (agent.py:906). 기본값(False)이면 step_size=0.0005로
        # 매우 느린 step-by-step 이동이 되어 record_episode의 home pose 이동이
        # 사실상 안 움직이는 것처럼 보이고, 그 사이 gRPC가 블록되어 stop 신호도
        # 즉시 반영되지 않는다.
        'is_sim': True,
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
        'is_sim': True,
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


# 두 번째 표준 sim 암 `sim_arm_2` (sim_device_slug='arm_2'). sim_arm 과 동일한
# 단일팔 7DOF. 양팔 시뮬 환경에서 sim_arm=왼팔, sim_arm_2=오른팔로 쓰인다.
# 설치 시 함께 로드되며, 기본 바인딩은 dual_arm 오른팔(/da_asm_right). 단일팔
# 환경(tutorial 등)에서는 사용되지 않고, 환경 활성화 시 토픽이 리포인트된다.
SIM_ARM_2 = {
    'name': 'sim_arm_2',
    'type': 'custom',
    'homepose': [],
    'settings': {
        'sim_device_slug': 'arm_2',
        'is_tutorial': False,
        'is_sim': True,
        'role': 'single_arm',
        'read_topic': '/da_asm_right/joint_states',
        'read_topic_msg': 'sensor_msgs/JointState',
        'write_type': 'topic',
        'write_topic': '/da_asm_right/joint_command',
        'write_topic_msg': 'sensor_msgs/JointState',
        'joint_names': ['right_joint1', 'right_joint2', 'right_joint3',
                        'right_joint4', 'right_joint5', 'right_joint6', 'right_gripper'],
        'joint_lower_bounds': [-3.14, -1.57, -2.40, -3.14, -1.40, -3.14, 0.0],
        'joint_upper_bounds': [3.14, 1.57, 0.50, 3.14, 1.40, 3.14, 0.04],
        'tool_index': [6],
        'tool_inner': True,
        'interpolation': False,
        'urdf_path': '/root/ros2_ws/src/mujoco_world/urdf/dual_arm_right.urdf',
        'urdf_package_dir': '/root/ros2_ws/src/mujoco_world/',
        'ik_setting': {
            'joints_to_lock': ['right_gripper', 'right_gripper_mirror'],
            'ee_definitions': [['ee', 'right_joint6', [0.095, 0.0, 0.0]]],
        },
    },
}


# Sensor rows: top-down (slightly diagonal) and front view RGB cameras.
# 토픽 이름은 mujoco_world_node가 발행하는 것과 1:1 매칭되어야 한다.
# 새 카메라를 추가하려면 (1) scene.xml에 카메라 정의 추가, (2) 노드의
# DEFAULT_CAMERA_NAMES 갱신, (3) 아래 리스트에 row 정의 추가.
TUTORIAL_SENSORS = [
    {
        'name': 'cam',
        'type': 'custom',
        'settings': {
            'sim_device_slug': 'cam',
            'is_tutorial': True,
            'tutorial_camera_slug': 'top_cam',
            'read_topic': f'{TUTORIAL_TOPIC_PREFIX}/top_cam/image_raw/compressed',
            'read_topic_msg': 'sensor_msgs/CompressedImage',
            'resolution': [640, 480],
        },
    },
    {
        'name': 'cam_2',
        'type': 'custom',
        'settings': {
            'sim_device_slug': 'cam_2',
            'is_tutorial': True,
            'tutorial_camera_slug': 'front_cam',
            'read_topic': f'{TUTORIAL_TOPIC_PREFIX}/front_cam/image_raw/compressed',
            'read_topic_msg': 'sensor_msgs/CompressedImage',
            'resolution': [640, 480],
        },
    },
    {
        # Wrist depth camera (gripper-mounted) for the `visual_reach` planner block.
        # RGB is streamed live; depth is fetched on demand via /tutorial/wrist_rgbd.
        'name': 'cam_3',
        'type': 'custom',
        'settings': {
            'sim_device_slug': 'cam_3',
            'is_tutorial': True,
            'tutorial_camera_slug': 'wrist_cam',
            'read_topic': f'{TUTORIAL_TOPIC_PREFIX}/wrist_cam/image_raw/compressed',
            'read_topic_msg': 'sensor_msgs/CompressedImage',
            'resolution': [640, 480],
            'has_depth': True,
            'rgbd_service': f'{TUTORIAL_TOPIC_PREFIX}/wrist_rgbd',
        },
    },
]


# Assembly row: tutorial_arm을 single-arm 으로 묶은 "에이전트".
# Assembly는 settings 컬럼이 없으므로 name으로 식별한다.
TUTORIAL_AGENT_NAME = 'tutorial_agent'


# Task row: tutorial_agent + 두 개의 tutorial 카메라를 가지는 기본 워크스페이스.
# `is_tutorial` 플래그를 settings에 넣어 robot/sensor와 동일하게 식별 가능하게 한다.
TUTORIAL_WORKSPACE_NAME = 'tutorial_env'
TUTORIAL_WORKSPACE_EPISODE_LEN = 200
TUTORIAL_WORKSPACE_HOMEPOSE = [0.0, 0.5, -0.5, 0.0, 0.5, 0.0, 0.04]
TUTORIAL_WORKSPACE_DEFAULT_SENSOR_CFG = {
    'img_size': [320, 240],
    'cropped_area': [0, 0, 640, 480],
    'rotate': 0,
}
