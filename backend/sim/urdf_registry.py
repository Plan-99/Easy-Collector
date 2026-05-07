"""
Backend 시뮬레이션용 URDF 경로 레지스트리.
backend 컨테이너에서 접근 가능한 경로 (/opt/easytrainer/project/...) 기준.
"""
import os

_DATA_ROOT = os.environ.get('EASYTRAINER_DATA_DIR', '/opt/easytrainer')
_ROS2_SRC = os.path.join(_DATA_ROOT, 'project', 'ros2', 'ros2_ws', 'src')

URDF_REGISTRY = {
    'piper': f'{_ROS2_SRC}/piper_description/urdf/piper_description.urdf',
    'piper(no gripper)': f'{_ROS2_SRC}/piper_description/urdf/piper_no_gripper_description.urdf',
    'tm_12': f'{_ROS2_SRC}/tm2_ros2/tm_description/urdf/tm12s.urdf',
    'tm_12s': f'{_ROS2_SRC}/tm2_ros2/tm_description/urdf/tm12s.urdf',
    'tm_12_robotiq': f'{_ROS2_SRC}/tm2_ros2/tm_description/urdf/tm12s.urdf',
    'rb3_730es_u': f'{_ROS2_SRC}/rbpodo_ros2/rbpodo_description/robots/rb3_730es_u.urdf',
    'kinova_gen3_7dof_robotiq_2f_85': f'{_ROS2_SRC}/ros2_kortex/kortex_description/robots/gen3_7dof.urdf',
    'fairino_fr5': f'{_ROS2_SRC}/frcobot_ros2/fairino_description/urdf/fairino5_v6.urdf',
    'jaka_zu12': f'{_ROS2_SRC}/jaka_ros2/src/jaka_description/urdf/jaka_zu12.urdf',
}


def get_urdf_path(robot_type: str) -> str | None:
    """로봇 타입으로 backend에서 접근 가능한 URDF 경로를 반환."""
    return URDF_REGISTRY.get(robot_type)
