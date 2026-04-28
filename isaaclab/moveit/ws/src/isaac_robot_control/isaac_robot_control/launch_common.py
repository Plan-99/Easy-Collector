"""
isaac_robot_control Launch 공통 유틸리티.

로봇별 launch 파일에서 이 모듈의 함수를 사용하여 공통 노드를 생성합니다.
"""

import os
import yaml
from collections.abc import Mapping
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


OUR_PACKAGE = "isaac_robot_control"


def load_yaml(package_name: str, file_path: str):
    """패키지 내 YAML 파일을 로드합니다."""
    package_dir = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_dir, file_path)
    with open(absolute_file_path, "r") as f:
        return yaml.safe_load(f)


def _sanitize(obj):
    """
    MoveIt config 값에서 tuple→list, MappingProxy→dict 등을 재귀적으로 변환합니다.
    ROS 2 파라미터 서버가 허용하지 않는 'tuple' 타입을 완전히 제거합니다.
    """
    if isinstance(obj, Mapping):
        # 키가 문자열이 아닌 경우를 대비해 str(k) 처리
        return {str(k): _sanitize(v) for k, v in obj.items()}
    
    if isinstance(obj, (list, tuple)):
        # 모든 시퀀스 타입을 list로 변환 (중요: () -> [])
        return [_sanitize(v) for v in obj]
    
    # RViz 설정 등에서 Class: "" 처럼 빈 문자열이 오는 경우 그대로 유지
    return obj


def _safe_param(param) -> dict | None:
    """
    moveit_config 속성을 안전한 dict로 변환합니다. 
    데이터가 없으면 None을 반환하여 파라미터 리스트에 추가되지 않게 합니다.
    """
    if param is None:
        return None
    
    sanitized = _sanitize(param)
    
    # 비어있는 리스트나 딕셔너리인 경우 ROS 2 파라미터로 전달하지 않음
    if isinstance(sanitized, (list, dict)) and not sanitized:
        return None
        
    return sanitized


def generate_isaac_moveit_launch(
    moveit_config,
    controller_yaml: str,
    bridge_executable: str,
    rviz_config: str,
    srdf_override: str | None = None,
) -> LaunchDescription:
    """MoveIt2 + IsaacSim 연동에 필요한 공통 LaunchDescription을 생성합니다.

    Args:
        moveit_config: MoveItConfigsBuilder로 빌드된 MoveIt config.
        controller_yaml: 컨트롤러 YAML 파일 경로 (config/ 내).
        bridge_executable: trajectory bridge executable 이름.
        rviz_config: RViz 설정 파일 경로 (config/ 내).
        srdf_override: 우리 패키지의 SRDF 파일 경로 (config/ 내). None이면 원본 사용.
    """
    our_pkg_share = get_package_share_directory(OUR_PACKAGE)
    our_controllers = load_yaml(OUR_PACKAGE, controller_yaml)

    # SRDF 오버라이드: end_effector 등 추가 정의가 필요한 경우
    if srdf_override:
        srdf_path = os.path.join(our_pkg_share, srdf_override)
        with open(srdf_path, "r") as f:
            srdf_content = f.read()
        semantic_param = {"robot_description_semantic": srdf_content}
    else:
        semantic_param = _safe_param(moveit_config.robot_description_semantic)

    # move_group 파라미터: 유효한 것만 포함
    move_group_params = [
        p for p in [
            _safe_param(moveit_config.robot_description),
            semantic_param,
            _safe_param(moveit_config.robot_description_kinematics),
            _safe_param(moveit_config.planning_pipelines),
            _safe_param(moveit_config.trajectory_execution),
            _safe_param(moveit_config.joint_limits),
        ] if p is not None
    ] + [
        {
            "moveit_simple_controller_manager": our_controllers["moveit_simple_controller_manager"],
            "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
            # 3D sensor 플러그인 비활성화 (headless/Docker 환경에서 GL 크래시 방지)
            "sensors": [],
            "use_sim_time": True,
        },
    ]

    # RViz 파라미터
    rviz_params = [
        p for p in [
            _safe_param(moveit_config.robot_description),
            semantic_param,
            _safe_param(moveit_config.robot_description_kinematics),
            _safe_param(moveit_config.planning_pipelines),
        ] if p is not None
    ] + [
        {"use_sim_time": True},
    ]

    # IsaacSim 토픽은 /simulation 네임스페이스 아래 퍼블리시됨
    sim_remappings = [
        ("/joint_states", "/simulation/joint_states"),
        ("/tf", "/simulation/tf"),
        ("/clock", "/simulation/clock"),
    ]

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            _safe_param(moveit_config.robot_description),
            {"use_sim_time": True},
        ],
        remappings=sim_remappings,
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params,
        remappings=sim_remappings,
    )

    trajectory_bridge = Node(
        package=OUR_PACKAGE,
        executable=bridge_executable,
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    rviz_config_path = os.path.join(our_pkg_share, rviz_config)
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=rviz_params,
        remappings=sim_remappings,
    )

    return LaunchDescription([
        robot_state_publisher,
        move_group,
        trajectory_bridge,
        rviz,
    ])
