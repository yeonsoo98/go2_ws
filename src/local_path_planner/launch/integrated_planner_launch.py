#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='local_path_planner',
            executable='costmap',
            name='costmap_generator',
            output='screen',
            parameters=[
                {'lidar_topic': '/velodyne_points'},
                {'imu_topic': '/imu/data'},
                {'odom_topic': '/odom'},
                {'map_width': 100},  
                {'map_height': 100}, 
                {'resolution': 0.05},
                {'use_sim_time': True}
            ]
        ),
        Node(
            package='local_path_planner',
            executable='path_planner',
            name='path_planner',
            output='screen',
            parameters=[
                {'algorithm_name': 'dwa'},
                {'use_sim_time': True}
            ]
        ),
    ])
