# -*- coding: utf-8 -*-
"""
dual_arm_plank demo defaults.

TWO independent role='single_arm' robots on SEPARATE topic prefixes, sharing
ONE MuJoCo physics sim (mujoco_world dual_arm_plank_world.launch.py). Unlike
dual_arm_assembly (one Assembly combining both arms into one workspace), here
each arm drives its OWN workspace:

  left  arm -> /da_plank_left   joints [left_joint1..6, left_gripper]
               cams: left_wrist_cam (DEPTH) + left_wrist_cam2
  right arm -> /da_plank_right  joints [right_joint1..6, right_gripper]
               cams: right_wrist_cam (DEPTH) + right_wrist_cam2

The two workspaces feed a single planner whose two plans (one per arm) run
simultaneously: each goes to a home pose (wrist cam looking down at the floor)
then a wrist_view_reach (visual_reach) block drives the tool to that arm's
handle on the shared plank.

These specs are consumed by:
  - configs/sim_devices.py  -> SIM_ENV_BINDINGS['dual_arm_plank'] (device repoint)
  - api/routes/dual_arm_plank.py -> assembly/workspace seeding + sim control
"""

_ENV = 'dual_arm_plank'
_RESET_PREFIX = '/dual_arm_plank_test'
_LEFT = '/da_plank_left'
_RIGHT = '/da_plank_right'

_ARM_LOWER = [-3.14, -1.57, -2.40, -3.14, -1.40, -3.14, 0.0]
_ARM_UPPER = [3.14, 1.57, 0.50, 3.14, 1.40, 3.14, 0.04]

# Home / observe pose (per arm): arm folded so the primary wrist camera looks
# straight DOWN at this arm's handle on the plank, giving the visual_reach block
# a clean top-down depth read. Tuned against the sim (see dual_arm_plank_scene).
#   left  -> green handle  at world (0.34,  0.16, ~0.084)
#   right -> magenta handle at world (0.34, -0.16, ~0.084)
_HOME_LEFT = [-0.12, 1.05, -1.25, 0.0, -1.40, 0.0, 0.04]
_HOME_RIGHT = [0.12, 1.05, -1.25, 0.0, -1.40, 0.0, 0.04]


def _arm_settings(prefix, joint_prefix, urdf):
    """role='single_arm' robot settings for a dual_arm_plank arm. No sim_test
    markers (it repoints the standard sim_arm/sim_arm_2 devices)."""
    jn = [f'{joint_prefix}_joint{i}' for i in range(1, 7)] + [f'{joint_prefix}_gripper']
    return {
        'is_sim': True,
        'is_tutorial': False,
        'role': 'single_arm',
        'read_topic': f'{prefix}/joint_states',
        'read_topic_msg': 'sensor_msgs/JointState',
        'write_type': 'topic',
        'write_topic': f'{prefix}/joint_command',
        'write_topic_msg': 'sensor_msgs/JointState',
        'joint_names': jn,
        'joint_lower_bounds': list(_ARM_LOWER),
        'joint_upper_bounds': list(_ARM_UPPER),
        'tool_index': [6],
        'tool_inner': True,
        'interpolation': False,
        'urdf_path': f'/root/ros2_ws/src/mujoco_world/urdf/{urdf}',
        'urdf_package_dir': '/root/ros2_ws/src/mujoco_world/',
        'ik_setting': {
            'joints_to_lock': [f'{joint_prefix}_gripper', f'{joint_prefix}_gripper_mirror'],
            'ee_definitions': [['ee', f'{joint_prefix}_joint6', [0.095, 0.0, 0.0]]],
        },
    }


def _cam_settings(cam_name, prefix, depth=False):
    """Sensor settings for one wrist camera. `depth` cameras additionally expose
    the per-arm wrist_rgbd service used by the visual_reach block."""
    s = {
        'is_sim': True,
        'is_tutorial': False,
        'sim_camera_slug': cam_name,
        'read_topic': f'{prefix}/{cam_name}/image_raw/compressed',
        'read_topic_msg': 'sensor_msgs/CompressedImage',
        'resolution': [640, 480],
    }
    if depth:
        s['has_depth'] = True
        s['rgbd_service'] = f'{prefix}/wrist_rgbd'
    return s


LEFT_ARM_SETTINGS = _arm_settings(_LEFT, 'left', 'dual_arm_left.urdf')
RIGHT_ARM_SETTINGS = _arm_settings(_RIGHT, 'right', 'dual_arm_right.urdf')

# Keyed by standard device slug -> settings (left=cam/cam_2, right=cam_3/cam_4).
CAMERAS = {
    'cam':   _cam_settings('left_wrist_cam',  _LEFT,  depth=True),
    'cam_2': _cam_settings('left_wrist_cam2', _LEFT,  depth=False),
    'cam_3': _cam_settings('right_wrist_cam',  _RIGHT, depth=True),
    'cam_4': _cam_settings('right_wrist_cam2', _RIGHT, depth=False),
}

LAUNCH = {
    'process_id': 'dual_arm_plank_test_sim',
    'package': 'mujoco_world',
    'launch_file': 'dual_arm_plank_world.launch.py',
    'topic_prefix': _RESET_PREFIX,
    'reset_prefix': _RESET_PREFIX,
}

ENV = _ENV
RESET_PREFIX = _RESET_PREFIX
EPISODE_LEN = 200
DEFAULT_SENSOR_CFG = {
    'img_size': [320, 240],
    'cropped_area': [0, 0, 640, 480],
    'rotate': 0,
}

# Two separate single-arm workspaces. `arm_slug` is the standard device slug the
# workspace's single-arm assembly points at; `sensor_slugs` are that arm's cams.
WORKSPACES = [
    {
        'name': 'dual_arm_plank_left',
        'assembly_name': 'dual_arm_plank_left_assembly',
        'arm_slug': 'arm',
        'sensor_slugs': ['cam', 'cam_2'],
        'home_pose': list(_HOME_LEFT),
    },
    {
        'name': 'dual_arm_plank_right',
        'assembly_name': 'dual_arm_plank_right_assembly',
        'arm_slug': 'arm_2',
        'sensor_slugs': ['cam_3', 'cam_4'],
        'home_pose': list(_HOME_RIGHT),
    },
]
