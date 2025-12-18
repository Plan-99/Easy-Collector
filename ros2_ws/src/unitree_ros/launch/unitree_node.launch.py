import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # --- 1. 런치 인자 선언 (Launch Arguments) ---
    
    # 'type' 파라미터에 전달될 값을 런치 인자로 선언
    # 터미널에서 robot_type:=<값> 으로 설정 가능
    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value='h1_2', # 노드의 기본값과 일치시킴
        description="Robot type (e.g., 'h1_2')"
    )
    
    # 'pub_frequency' 파라미터에 전달될 값을 런치 인자로 선언
    # 터미널에서 freq:=<값> 으로 설정 가능
    pub_frequency_arg = DeclareLaunchArgument(
        'freq', # 런치 인자 이름 (파라미터 이름과 달라도 됨)
        default_value='50.0', # 노드의 기본값과 일치시킴
        description='Joint state publish frequency'
    )

    # --- 2. 런치 인자 값 가져오기 (Launch Configurations) ---
    
    # 런치 인자 'robot_type'의 값을 가져옴
    robot_type_launch_config = LaunchConfiguration('robot_type')
    
    # 런치 인자 'freq'의 값을 가져옴
    pub_frequency_launch_config = LaunchConfiguration('freq')


    # --- 3. 노드 정의 (Node Definition) ---
    
    unitree_node = Node(
        # [필수] 이 런치 파일이 포함된 패키지 이름을 적어주세요.
        package='unitree_ros', 
        
        # [필수] 실행할 파이썬 노드 파일의 이름을 적어주세요.
        executable='unitree_node.py', 
        
        name='unitree_node', # ROS 그래프에 표시될 노드 이름
        output='screen',     # 노드의 로그(get_logger)를 터미널에 출력
        
        # 노드에 파라미터 전달
        parameters=[
            {
                'type': robot_type_launch_config,
                'pub_frequency': pub_frequency_launch_config
            }
        ]
    )

    # --- 4. 런치 설명 반환 ---
    
    return LaunchDescription([
        # 1번에서 선언한 런치 인자들을 런치 설명에 추가
        robot_type_arg,
        pub_frequency_arg,
        
        # 3번에서 정의한 노드를 런치 설명에 추가
        unitree_node
    ])