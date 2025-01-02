# ~/go2_ws/src/unitree-go2-ros2/launch/combined_launch.py

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():


    
    # costmap_generator 노드 실행
    costmap_generator = Node(
        package='local_path_planner',
        executable='costmap',
        name='costmap_generator',
        output='screen',
        parameters=[
            {'use_sim_time': True}  # 시뮬레이션 시간 사용 설정

        ]
    )
    
    # local_planner 노드 실행
    local_planner = Node(
        package='local_path_planner',
        executable='dwa_planner',
        name='dwa_planner',
        output='screen',
        parameters=[
            {'use_sim_time': True}  # 시뮬레이션 시간 사용 설정

        ]
    )
    
    # LaunchDescription에 노드 추가
    ld = LaunchDescription()
    
    # 비용 맵 생성 및 로컬 플래너 노드 추가
    ld.add_action(costmap_generator)
    ld.add_action(local_planner)
    
    return ld
