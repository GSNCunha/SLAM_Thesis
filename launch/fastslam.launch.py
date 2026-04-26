import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    fastslam_node = Node(
        package='fastslam_thesis',
        executable='fastslam_node',
        name='fastslam_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'particle_count': 300,
            'map_resolution': 0.05,
            'map_width': 400,
            'map_height': 400,
            'linear_update': 0.2,
            'angular_update': 0.2
        }]
    )
    return LaunchDescription([fastslam_node])
