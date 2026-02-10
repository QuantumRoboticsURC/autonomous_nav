#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Directorios
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # ========== RVIZ ==========
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': False}],
        output='screen'
    )

    
    return LaunchDescription([
   
        
        # RViz
        rviz_node,
    ])