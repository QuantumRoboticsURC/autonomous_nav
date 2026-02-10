#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Directorios

    gps_node_dir = get_package_share_directory('rtk_rover')
    imu_node_dir = get_package_share_directory('bno055')
    
    matrix_node = Node(
        package='matrix_implementation3',
        executable='matrixsignalreceiver',
        name='matrixsignalreceiver',
        output='screen'
    )

    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(imu_node_dir, 'launch', 'bno055.launch.py')  # o el nombre de tu launch
        )
    )

    gps_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gps_node_dir, 'launch', 'rtk_rover_launch.py')  # o el nombre de tu launch
        )
    )   

    return LaunchDescription([
        
         # LIDAR
        matrix_node,
        imu_launch,
        gps_launch     
        
     
    ])
