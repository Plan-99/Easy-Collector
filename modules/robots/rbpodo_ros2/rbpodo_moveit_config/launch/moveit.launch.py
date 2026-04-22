import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    declared_arguments = []
    
    # Namespace 인자 추가
    declared_arguments.append(
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Namespace for all nodes, topics, services and actions",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config",
            default_value="moveit.rviz",
            description="RViz configuration file",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="10.0.2.7",
            description="RB Cobot Control Box IP Address",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="True if there's no RB Cobot Control Box",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description="True when use fake sensor commands",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "cb_simulation",
            default_value="Simulation",
            description="Select RB Control Box mode, Simulation or Real",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "model_id",
            default_value="rb5_850e",
            description="RB Series currently using",
        )
    )
    
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )


def launch_setup(context, *args, **kwargs):
    # LaunchConfiguration 불러오기
    namespace = LaunchConfiguration("namespace")
    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    model_id = LaunchConfiguration("model_id")
    cb_simulation = LaunchConfiguration("cb_simulation")

    mappings = {
        "robot_ip": robot_ip,
        "use_fake_hardware": use_fake_hardware,
        "fake_sensor_commands": fake_sensor_commands,
        "model_id": model_id,
        "cb_simulation": cb_simulation,
    }

    # MoveIt Config 설정 (MoveItConfigsBuilder에도 네임스페이스가 고려되어야 할 수 있음)
    moveit_config = (
        MoveItConfigsBuilder("rbpodo")
        .robot_description(file_path="config/rbpodo.urdf.xacro", mappings=mappings)
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    # 1. Move Group Node
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace=namespace, # 네임스페이스 추가
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    rviz_base = LaunchConfiguration("rviz_config")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("rbpodo_moveit_config"), "config", rviz_base]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        # namespace=namespace,  <-- 이 부분을 주석 처리하거나 삭제하세요.
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            # RViz 플러그인에 네임스페이스 직접 명시
            {"move_group_namespace": namespace},
            {"use_sim_time": True}
        ],
        # 토픽들을 네임스페이스가 붙은 쪽으로 연결
        remappings=[
            ("/robot_description", [namespace, "/robot_description"]),
            ("/robot_description_semantic", [namespace, "/robot_description_semantic"]),
            ("/joint_states", [namespace, "/joint_states"]),
            ("/display_planned_path", [namespace, "/display_planned_path"]),
            ("/monitored_planning_scene", [namespace, "/monitored_planning_scene"]),
            ("/clicked_point", [namespace, "/clicked_point"]),
            # TF의 경우 로봇이 여러 대라면 아래와 같이 리매핑이 필요할 수 있습니다.
            # ("/tf", [namespace, "/tf"]),
            # ("/tf_static", [namespace, "/tf_static"]),
        ]
    )

    # 3. Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        namespace=namespace, # 네임스페이스 추가
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "link0"],
    )

    # 4. Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=namespace, # 네임스페이스 추가
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # 5. ros2_control node
    ros2_controllers_path = os.path.join(
        get_package_share_directory("rbpodo_bringup"),
        "config",
        "controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=namespace, # 네임스페이스 추가
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace, # 이미 여기서 ec_robot_1이 적용됨
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "controller_manager", # 상대 경로로 지정
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager", "controller_manager",
        ],
    )

    nodes_to_start = [
        rviz_node,
        static_tf,
        robot_state_publisher,
        run_move_group_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
    ]

    return nodes_to_start