# -*- coding: utf-8 -*-
"""
표준(canonical) 시뮬레이션 디바이스 + 환경별 바인딩.

설치 시 로드되는 표준 디바이스:
  - 로봇 2개: sim_arm(slug 'arm'), sim_arm_2(slug 'arm_2')  — 둘 다 single_arm 7DOF
  - 카메라 3개: cam('cam'), cam_2('cam_2'), cam_3('cam_3')

(디바이스 자체의 행 정의/시딩은 tutorial_defaults.py + tutorial.py 가 담당한다.
 이 모듈은 "각 시뮬 환경이 표준 디바이스를 어떤 토픽/설정으로 쓰는지"의 단일
 진실원천이다.)

모든 시뮬 환경은 자기만의 로봇/센서 행을 따로 만들지 않고, 활성화 시
``repoint_sim_devices(env)`` 로 표준 디바이스의 토픽/설정을 그 환경에 맞게
갈아끼운다(리포인트). 한 번에 한 환경만 활성화된다는 전제.

바인딩 형식:
  SIM_ENV_BINDINGS[env] = {
    'robots':  { slug: <robot settings dict> , ... },
    'cameras': { slug: <sensor settings dict> , ... },
  }
나열되지 않은 slug 는 해당 환경에서 사용되지 않는다(리포인트 대상 아님).
"""
import copy

from .dual_arm_assembly_defaults import SPEC as _DA_ASM_SPEC
from .dual_arm_plank_defaults import (
    CAMERAS as _PLANK_CAMERAS,
    LEFT_ARM_SETTINGS as _PLANK_LEFT_ARM,
    RIGHT_ARM_SETTINGS as _PLANK_RIGHT_ARM,
)
from .tutorial_defaults import (
    TUTORIAL_ROBOT,
    TUTORIAL_SENSORS,
    TUTORIAL_TOPIC_PREFIX,
)

_P = TUTORIAL_TOPIC_PREFIX  # '/tutorial' — 번들 MuJoCo world 공통 prefix


def _arm_from_tutorial(is_tutorial):
    """tutorial single-arm 설정(토픽/IK/joint)을 그대로 쓰되 is_tutorial 만 분기.

    데모는 is_tutorial=False 여야 record_episode 의 env.reset 이 peg 를 매 에피소드
    홈으로 스냅하지 않는다(플래너의 peg 복원 보존)."""
    s = copy.deepcopy(TUTORIAL_ROBOT['settings'])
    s['is_tutorial'] = is_tutorial
    return s


def _tutorial_cam(slug):
    """TUTORIAL_SENSORS 에서 sim_device_slug 로 카메라 설정을 가져온다."""
    for spec in TUTORIAL_SENSORS:
        if spec['settings'].get('sim_device_slug') == slug:
            return copy.deepcopy(spec['settings'])
    raise KeyError(slug)


# 데모(peg_in_hole) wrist 2-cam — cam=wrist_cam(depth), cam_2=wrist_cam_down.
def _demo_cam(slug, cam_slug, depth=False):
    s = {
        'sim_device_slug': slug,
        'is_tutorial': False,
        'sim_camera_slug': cam_slug,
        'read_topic': f'{_P}/{cam_slug}/image_raw/compressed',
        'read_topic_msg': 'sensor_msgs/CompressedImage',
        'resolution': [640, 480],
    }
    if depth:
        s['has_depth'] = True
        s['rgbd_service'] = f'{_P}/wrist_rgbd'
    return s


def _da_robot(sim_test_slug):
    """dual_arm_assembly SPEC 의 left_arm/right_arm 로봇 설정을 표준 디바이스용으로
    변환(sim_test 마커 제거, is_tutorial=False)."""
    for r in _DA_ASM_SPEC['robots']:
        if r['settings'].get('sim_test_slug') == sim_test_slug:
            s = copy.deepcopy(r['settings'])
            s.pop('sim_test_env', None)
            s.pop('sim_test_slug', None)
            s['is_tutorial'] = False
            return s
    raise KeyError(sim_test_slug)


def _da_cam(sim_test_slug):
    """dual_arm_assembly SPEC 의 카메라 설정 변환. sim_camera_slug 를 부여해
    블록 remap(visual_reach)이 따라가게 한다."""
    for c in _DA_ASM_SPEC['sensors']:
        if c['settings'].get('sim_test_slug') == sim_test_slug:
            s = copy.deepcopy(c['settings'])
            s.pop('sim_test_env', None)
            s.pop('sim_test_slug', None)
            s['is_tutorial'] = False
            s['sim_camera_slug'] = sim_test_slug
            return s
    raise KeyError(sim_test_slug)


SIM_ENV_BINDINGS = {
    # 기본 tutorial 환경: 단일팔 + 3카메라(top/front/wrist). is_tutorial=True.
    'tutorial': {
        'robots': {
            'arm': _arm_from_tutorial(True),
        },
        'cameras': {
            'cam': _tutorial_cam('cam'),      # top_cam
            'cam_2': _tutorial_cam('cam_2'),  # front_cam
            'cam_3': _tutorial_cam('cam_3'),  # wrist_cam (depth)
        },
    },
    # 데모 peg_in_hole: 같은 번들 world, 단일팔, wrist 2-cam. is_tutorial=False
    # (플래너가 scene 의 peg 를 직접 복원하므로 매 에피소드 reset 금지).
    'peg_in_hole': {
        'robots': {
            'arm': _arm_from_tutorial(False),
        },
        'cameras': {
            'cam': _demo_cam('cam', 'wrist_cam', depth=True),
            'cam_2': _demo_cam('cam_2', 'wrist_cam_down'),
        },
    },
    # 양팔: sim_arm=왼팔(/da_asm_left), sim_arm_2=오른팔(/da_asm_right). 두 단일팔이
    # 한 MuJoCo sim 을 공유(dual_arm_assembly_world.launch.py). cam=top(좌), cam_2=front(우).
    'dual_arm': {
        'robots': {
            'arm': _da_robot('left_arm'),
            'arm_2': _da_robot('right_arm'),
        },
        'cameras': {
            'cam': _da_cam('top_cam'),
            'cam_2': _da_cam('front_cam'),
        },
    },
    # 양팔 plank 데모: 두 단일팔이 각자 워크스페이스를 구동(어셈블리로 합치지 않음).
    # 한 MuJoCo sim 공유(dual_arm_plank_world.launch.py). 팔마다 손목캠 2개:
    #   왼팔 cam=left_wrist_cam(depth)/cam_2=left_wrist_cam2  -> /da_plank_left
    #   오른팔 cam_3=right_wrist_cam(depth)/cam_4=right_wrist_cam2 -> /da_plank_right
    'dual_arm_plank': {
        'robots': {
            'arm': copy.deepcopy(_PLANK_LEFT_ARM),
            'arm_2': copy.deepcopy(_PLANK_RIGHT_ARM),
        },
        'cameras': {
            'cam': copy.deepcopy(_PLANK_CAMERAS['cam']),
            'cam_2': copy.deepcopy(_PLANK_CAMERAS['cam_2']),
            'cam_3': copy.deepcopy(_PLANK_CAMERAS['cam_3']),
            'cam_4': copy.deepcopy(_PLANK_CAMERAS['cam_4']),
        },
    },
}
