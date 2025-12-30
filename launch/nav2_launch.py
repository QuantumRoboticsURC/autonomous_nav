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
    
    # Archivos de configuración
    params_file = os.path.join(autonomous_nav_dir, 'config', 'config_nav2.yaml')
    ekf_params_file = os.path.join(autonomous_nav_dir, 'config', 'ekf.yaml')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # ========== STATIC TRANSFORMS ==========
  
    static_tf_base_footprint_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_base_link',
        arguments=['0', '0', '0.40', '0', '0', '0', 'base_footprint', 'base_link'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    static_tf_base_link_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser',
        arguments=['0.525', '0', '0.05', '0', '0', '0', 'base_link', 'laser_frame'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ========== LIDAR Y FILTRO ==========
    
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sllidar_dir, 'launch', 'sllidar_a1_launch.py')
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
            'lower_angle': -1.5708,
            'upper_angle': 1.5708,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )
    
    # ========== ODOMETRÍA ==========
    
    # 1. Odometría de ruedas (publica /wheel/odom)
    wheel_odometry_node = Node(
        package='autonomous_nav',
        executable='odometry',
        name='wheel_odometry',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # 2. GPS Odometry (publica /gps/odom)
    gps_odometry_node = Node(
        package='autonomous_nav',
        executable='odom_by_gps',
        name='gps_odometry',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # ========== FUSIÓN DE SENSORES (EKF) ==========
    
    # 3. EKF Local (ruedas + IMU → /odometry/local)
    # Publica TF: odom → base_footprint
    ekf_local_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local',
        parameters=[ekf_params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('odometry/filtered', 'odometry/local')
        ],
        output='screen'
    )
    
    # 4. EKF Global (GPS + ruedas + IMU → /odometry/global)
    # Publica TF: map → odom
    ekf_global_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global',
        parameters=[ekf_params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('odometry/filtered', 'odometry/global')
        ],
        output='screen'
    )

    # ========== CONTROLADORES DE NAVEGACIÓN ==========

    nav2_controller_node = Node(
        package='autonomous_nav',
        executable='nav2_controller',
        name='nav2_controller',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    direct_motion_node = Node(
        package='autonomous_nav',
        executable='move',  
        name='direct_motion_controller',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    main_controller_node = Node(
        package='autonomous_nav',
        executable='main_controller',
        name='main_controller',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # ========== DETECCIÓN ==========

    detection_aruco_node = Node(
        package='autonomous_nav',
        executable='detection',
        name='detection_aruco',
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
    
    # ========== RVIZ ==========
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # ========== SECUENCIA DE LANZAMIENTO ==========
    
    # 1. Wheel y GPS odometry arrancan después de static TFs
    start_wheel_odom = RegisterEventHandler(
        OnProcessStart(
            target_action=static_tf_base_footprint_to_base_link,
            on_start=[wheel_odometry_node]
        )
    )
    
    start_gps_odom = RegisterEventHandler(
        OnProcessStart(
            target_action=static_tf_base_footprint_to_base_link,
            on_start=[gps_odometry_node]
        )
    )
    
    # 2. EKF Local arranca después de wheel odometry
    start_ekf_local = RegisterEventHandler(
        OnProcessStart(
            target_action=wheel_odometry_node,
            on_start=[ekf_local_node]
        )
    )
    
    # 3. EKF Global arranca después de GPS odometry
    start_ekf_global = RegisterEventHandler(
        OnProcessStart(
            target_action=gps_odometry_node,
            on_start=[ekf_global_node]
        )
    )
    
    # 4. Nav2 controller arranca después de EKF local
    start_nav2_ctrl = RegisterEventHandler(
        OnProcessStart(
            target_action=ekf_local_node,
            on_start=[nav2_controller_node]
        )
    )
    
    # 5. Direct motion arranca después de nav2_controller
    start_direct_motion = RegisterEventHandler(
        OnProcessStart(
            target_action=nav2_controller_node,
            on_start=[direct_motion_node]
        )
    )
    
    # 6. Main controller arranca después de direct_motion
    start_main_ctrl = RegisterEventHandler(
        OnProcessStart(
            target_action=direct_motion_node,
            on_start=[main_controller_node]
        )
    )

    # 7. Detection arranca después de main_controller
    start_detection = RegisterEventHandler(
        OnProcessStart(
            target_action=main_controller_node,
            on_start=[detection_aruco_node]
        )
    )
    
    return LaunchDescription([
        # Arguments
        declare_use_sim_time_cmd,
        
        # Static transforms
        static_tf_base_footprint_to_base_link,
        static_tf_base_link_to_laser,

        # LIDAR
        lidar_launch,
        laser_filter_node,
        
        # Secuencia de odometrías
        start_wheel_odom,      # ← Wheel odometry
        start_gps_odom,        # ← GPS odometry
        start_ekf_local,       # ← EKF local (publica odom→base_footprint)
        start_ekf_global,      # ← EKF global (publica map→odom)
        
        # Secuencia de controladores
        start_nav2_ctrl,
        start_direct_motion,
        start_main_ctrl,
        start_detection,
        
        # Nav2 stack
        nav2_bringup,
        
        # RViz
        rviz_node,
    ])