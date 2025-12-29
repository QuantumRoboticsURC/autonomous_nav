#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import RegisterEventHandler, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node


def generate_launch_description():
    # Directorios
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    autonomous_nav_dir = get_package_share_directory('autonomous_nav')
    sllidar_dir = get_package_share_directory('sllidar_ros2')
    
    # Archivo de parámetros Nav2
    params_file = os.path.join(autonomous_nav_dir, 'config', 'config_nav2.yaml')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # ========== STATIC TRANSFORMS (siempre activos) ==========
    
    static_tf_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    static_tf_base_footprint_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_base_link',
        arguments=['0', '0', '0.05', '0', '0', '0', 'base_footprint', 'base_link'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    static_tf_base_link_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser',
        arguments=['0.1', '0', '0.05', '0', '0', '0', 'base_link', 'laser_frame'],
        parameters=[{'use_sim_time': use_sim_time}]
    )


      # ========== LIDAR Y FILTRO ==========
    
    # 1. LIDAR Launch (incluye el launch original)
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sllidar_dir, 'launch', 'sllidar_a1_launch.py')  # o el nombre de tu launch
        ),
        launch_arguments={
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': '115200',
            'frame_id': 'laser_frame',
            'inverted': 'false',
            'angle_compensate': 'true',
            'scan_mode': 'Sensitivity'
        }.items()
    )
    
    laser_filter_node = Node(
        package='autonomous_nav',
        executable='laser_filter_180',
        name='laser_filter_180',
        parameters=[{
            'lower_angle': -1.5708,  # -90 grados
            'upper_angle': 1.5708,   # +90 grados
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )
    
    # ========== NODOS DE TU PAQUETE ==========
    
    # 1. GPS Odometry (publica /gps_origin y /odom)
    odom_by_gps_node = Node(
        package='autonomous_nav',
        executable='odom_by_gps',
        name='odom_by_gps',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # 2. Nav2 Controller (convierte targets y coordina con Nav2)
    nav2_controller_node = Node(
        package='autonomous_nav',
        executable='nav2_controller',
        name='nav2_controller',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # 3. Direct Motion Controller (CENTER_AND_APPROACH y SEARCH)
    direct_motion_node = Node(
        package='autonomous_nav',
        executable='move',  
        name='direct_motion_controller',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # 4. Main Controller (FSM - máquina de estados)
    main_controller_node = Node(
        package='autonomous_nav',
        executable='main_controller',
        name='main_controller',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # ========== NAV2 STACK ==========
    
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true'
        }.items()
    )
    
    # ========== RVIZ (OPCIONAL) ==========
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # ========== SECUENCIA DE LANZAMIENTO ==========
    
    # odom_by_gps arranca después de los static transforms
    start_odom = RegisterEventHandler(
        OnProcessStart(
            target_action=static_tf_map_to_odom,
            on_start=[odom_by_gps_node]
        )
    )
    
    # nav2_controller arranca después de odom_by_gps
    start_nav2_ctrl = RegisterEventHandler(
        OnProcessStart(
            target_action=odom_by_gps_node,
            on_start=[nav2_controller_node]
        )
    )
    
    # direct_motion arranca después de nav2_controller
    start_direct_motion = RegisterEventHandler(
        OnProcessStart(
            target_action=nav2_controller_node,
            on_start=[direct_motion_node]
        )
    )
    
    # main_controller (FSM) arranca después de direct_motion
    start_main_ctrl = RegisterEventHandler(
        OnProcessStart(
            target_action=direct_motion_node,
            on_start=[main_controller_node]
        )
    )
    
    return LaunchDescription([
        # Arguments
        declare_use_sim_time_cmd,
        
        # Static transforms (arrancan primero)
        static_tf_map_to_odom,
        static_tf_base_footprint_to_base_link,
        static_tf_base_link_to_laser,

         # LIDAR
        lidar_launch,
        
        # Laser filter
        laser_filter_node,
        
        
        # Secuencia de nodos
        start_odom,
        start_nav2_ctrl,
        start_direct_motion,
        start_main_ctrl,
        
        # Nav2 stack
        nav2_bringup,
        
        # RViz (opcional - comenta si no lo necesitas)
        rviz_node,
    ])
