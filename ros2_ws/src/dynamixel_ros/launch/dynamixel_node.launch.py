import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. 런치 인자(Launch Argument) 선언
    # ROS 1의 <arg> 태그에 해당합니다.
    device_port_arg = DeclareLaunchArgument(
        'device_port',
        default_value='/dev/ttyUSB0',
        description='Device port for Dynamixel'
    )

    # 2. 노드(Node) 실행 액션 정의
    dynamixel_node = Node(
        package='dynamixel_ros',
        # ROS 1의 'type'은 ROS 2에서 'executable'에 해당합니다.
        # setup.py의 entry_points에 등록된 실행 파일 이름을 사용합니다.
        executable='dynamixel_node.py',
        name='dynamixel_node',
        output='screen',
        # <env> 태그는 env 딕셔너리로 변환됩니다.
        emulate_tty=True, # output='screen'과 함께 사용하면 출력이 안정적입니다.
        parameters=[
            # <param> 태그는 parameters 리스트로 변환됩니다.
            {'baudrate': 57600},
            # $(arg device_port)는 LaunchConfiguration을 사용해 값을 가져옵니다.
            {'device_port': LaunchConfiguration('device_port')}
        ]
    )

    # 3. LaunchDescription 객체 생성 및 반환
    # 선언된 인자와 노드 액션을 리스트에 담아 반환합니다.
    return LaunchDescription([
        device_port_arg,
        dynamixel_node
    ])