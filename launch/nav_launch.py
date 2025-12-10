from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

def generate_launch_description():

    node1 = Node(
        package='autonomous_nav',
        executable='move',
        name="move",
    )

    node2 = Node(
        package='autonomous_nav',
        executable='odom_by_gps',
        name="odom_by_gps",
    )

    node3 = Node(
        package='autonomous_nav',
        executable='odometry',
        name="odometry",
    )

    node4 = Node(
        package='autonomous_nav',
        executable='kalman2D',
        name="kalman2D",
    )

    node5 = Node(
        package='autonomous_nav',
        executable='main_controller',
        name="main_controller",
    )

    # node2 se lanza cuando node1 ARRANCA
    start_node2 = RegisterEventHandler(
        OnProcessStart(
            target_action=node1,
            on_start=[node2],
        )
    )

    # node3 se lanza cuando node2 ARRANCA
    start_node3 = RegisterEventHandler(
        OnProcessStart(
            target_action=node2,
            on_start=[node3],
        )
    )

    # node4 se lanza cuando node3 ARRANCA
    start_node4 = RegisterEventHandler(
        OnProcessStart(
            target_action=node3,
            on_start=[node4],
        )
    )

    # node5 se lanza cuando node4 ARRANCA
    start_node5 = RegisterEventHandler(
        OnProcessStart(
            target_action=node4,
            on_start=[node5],
        )
    )

    return LaunchDescription([
        node1,        # el primero sí va directo
        start_node2,
        start_node3,
        start_node4,
        start_node5,
    ])
