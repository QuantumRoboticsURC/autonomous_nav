import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    params_file = os.path.expanduser('~/nav2_mapless_real.yaml')
    
    return LaunchDescription([
        # 1. TF: map -> odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),
        
        # 2. TF: base_footprint -> base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_to_base_link',
            arguments=['0', '0', '0.05', '0', '0', '0', 'base_footprint', 'base_link']
        ),
        
        # 3. TF: base_link -> laser_frame (AJUSTAR SEGÚN TU ROBOT)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=['0.1', '0', '0.05', '0', '0', '0', 'base_link', 'laser_frame']
            # ↑ Cambiar estos valores según tu robot
        ),
        
        # 4. Tu nodo de odometría (NUEVO CÓDIGO)
        Node(
            package='tu_paquete',  # ← Cambiar por tu paquete
            executable='odometry.py',  # ← Nombre de tu archivo
            name='odometry',
            output='screen'
        ),
        
        # 5. Driver del IMU (si no lo tienes ya)
        # Node(
        #     package='bno055',
        #     executable='bno055',
        #     name='bno055',
        #     parameters=[{'topic_prefix': '/bno055'}]
        # ),
        
        # 6. Driver del LIDAR
        # Node(
        #     package='rplidar_ros',
        #     executable='rplidar_node',
        #     name='rplidar_node',
        #     parameters=[{
        #         'serial_port': '/dev/ttyUSB0',
        #         'frame_id': 'laser_frame'
        #     }]
        # ),
        
        # 7. Driver de motores (tu código que lee /cmd_vel)
        # Node(...),
        
        # 8. Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'false',
                'params_file': params_file,
                'autostart': 'true'
            }.items()
        ),
    ])