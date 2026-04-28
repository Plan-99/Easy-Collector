"""Backend mirror of ros2_bridge/configs/robot_configs.py ee_definitions defaults.

ROS2 컨테이너의 robot_configs.py는 backend 컨테이너에서 import 불가능하므로
default ee_definitions만 frontend 표시용으로 여기에 미러링한다.
값을 수정할 때는 ros2/ros2_bridge/configs/robot_configs.py도 같이 갱신해야 한다.

각 항목 형식: (ee_name, parent_joint_or_frame, offset_xyz_list_or_None)
"""

ROBOT_EE_DEFINITIONS = {
    'test_arm': [('ee', 'joint7', None)],
    'piper': [('ee', 'joint7', None)],
    'piper(no gripper)': [('ee', 'joint6', [0.20, 0.0, 0.0])],
    'tm_12': [('ee', 'joint_6', [0.0, 0.0, 0.15])],
    'tm_12s': [('ee', 'joint_6', [0.0, 0.0, 0.15])],
    'tm_12_robotiq': [('ee', 'joint_6', [0.0, 0.0, 0.15])],
    'rb3_730es_u': [('ee', 'tcp_joint', None)],
    'kinova_gen3_7dof_robotiq_2f_85': [('ee', 'joint_7', [0.0, 0.0, 0.0])],
    'fairino_fr5': [('ee', 'j6', [0.0, 0.0, 0.1])],
    'jaka_zu12': [('ee', 'joint_6', None)],
}


def get_default_ee_definitions(robot_type):
    """주어진 robot_type의 default ee_definitions를 반환.

    각 정의는 dict로 직렬화: {name, parent, offset}
    offset은 list 또는 None.
    """
    raw = ROBOT_EE_DEFINITIONS.get(robot_type, [])
    return [{'name': n, 'parent': p, 'offset': o} for (n, p, o) in raw]
