# -*- coding: utf-8 -*-
"""
Demo simulation device defaults.

Demo simulations (repo-root ``demo/``) run the SAME bundled MuJoCo world as
tutorial mode (same ``/tutorial`` ROS topics), but they are **not** the tutorial
scene: a curriculum / planner owns the scene and restores the peg itself
(``put_peg`` + randomize). A device flagged ``is_tutorial`` makes
``record_episode`` create the env with ``tutorial=True``, which makes
``env.reset()`` call ``/tutorial/reset_world`` and snap the movable objects back
to the home keyframe on every episode — wiping the planner's restoration and
killing peg-position diversity in the collected data.

So demo-sim devices are an **independent device set**: identical in form to the
tutorial devices (same topics, IK, etc.) but ``is_tutorial=False`` (still
``is_sim=True`` so ``Agent.move_to`` takes the single-publish sim branch).

Identification: a sim device carries ``is_sim=True`` AND ``is_tutorial`` is
falsy in its settings. Created on demand when the user activates a demo sim
(see ``backend/api/routes/demo.py``).
"""
import copy

from .tutorial_defaults import (
    TUTORIAL_ROBOT,
    TUTORIAL_TOPIC_PREFIX,
    TUTORIAL_WORKSPACE_DEFAULT_SENSOR_CFG,
    TUTORIAL_WORKSPACE_HOMEPOSE,
)

# Demo sims reuse the bundled MuJoCo world, so the topic prefix is the same.
SIM_TOPIC_PREFIX = TUTORIAL_TOPIC_PREFIX


# Unambiguous marker for "demo simulation" devices. `is_sim` alone is NOT
# enough to identify them — dual_arm_test / other custom sim robots also carry
# is_sim. This marker is unique to the demo-sim device set.
SIM_DEVICE_MARKER = 'demo_sim_device'


def _sim_robot_settings():
    """tutorial_arm settings 그대로(토픽/IK/joint bounds) 가져오되 tutorial 표식만 제거.

    단일 진실원천(tutorial_defaults)에서 파생 → 토픽/IK 가 바뀌어도 자동 동기화.
    """
    s = copy.deepcopy(TUTORIAL_ROBOT['settings'])
    s['is_tutorial'] = False
    s['is_sim'] = True
    s[SIM_DEVICE_MARKER] = True
    return s


# Robot row: same 6DOF arm + gripper as the tutorial robot, but a demo-sim device.
SIM_ROBOT = {
    'name': 'sim_arm',
    'type': 'custom',
    'homepose': [],
    'settings': _sim_robot_settings(),
}


# Sensor rows: the demo wrist 2-cam setup (peg_in_hole manifest cameras
# wrist_cam + wrist_cam_down). Same topics as the bundled sim; flagged is_sim
# (NOT is_tutorial). `sim_camera_slug` identifies each row idempotently.
SIM_SENSORS = [
    {
        'name': 'sim_wrist_cam',
        'type': 'custom',
        'settings': {
            'is_tutorial': False,
            'is_sim': True,
            'demo_sim_device': True,
            'sim_camera_slug': 'wrist_cam',
            'read_topic': f'{SIM_TOPIC_PREFIX}/wrist_cam/image_raw/compressed',
            'read_topic_msg': 'sensor_msgs/CompressedImage',
            'resolution': [640, 480],
            'has_depth': True,
            'rgbd_service': f'{SIM_TOPIC_PREFIX}/wrist_rgbd',
        },
    },
    {
        'name': 'sim_wrist_cam_down',
        'type': 'custom',
        'settings': {
            'is_tutorial': False,
            'is_sim': True,
            'demo_sim_device': True,
            'sim_camera_slug': 'wrist_cam_down',
            'read_topic': f'{SIM_TOPIC_PREFIX}/wrist_cam_down/image_raw/compressed',
            'read_topic_msg': 'sensor_msgs/CompressedImage',
            'resolution': [640, 480],
        },
    },
]


# Assembly (single-arm, sim_arm). Assembly has no settings column → name-keyed.
SIM_AGENT_NAME = 'sim_agent'

# Workspace defaults reused from tutorial (img crop/resize, home pose).
SIM_WORKSPACE_DEFAULT_SENSOR_CFG = dict(TUTORIAL_WORKSPACE_DEFAULT_SENSOR_CFG)
SIM_WORKSPACE_HOMEPOSE = list(TUTORIAL_WORKSPACE_HOMEPOSE)
