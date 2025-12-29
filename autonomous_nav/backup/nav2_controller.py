#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Int8, Bool


class Nav2Controller(Node):
    def __init__(self):
        super().__init__("nav2_controller")
        self.get_logger().info("Nav2 Controller node started")

        latching_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        # Publisher para Nav2
        self.pub_goal = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Subscribers
        self.create_subscription(Point, "/target_point", self.callback_target, latching_qos)
        self.create_subscription(Int8, '/state_command', self.callback_state_command, 10)
        
        # Estado actual
        self.current_state = None
        self.last_goal_sent = None

    def callback_state_command(self, msg: Int8):
        """
        0 = STOP
        1 = POINTTOPOINT (usar Nav2)
        2 = CENTERANDAPPROACH (no usar Nav2)
        3 = SEARCH (no usar Nav2)
        """
        old_state = self.current_state
        self.current_state = msg.data
        
        self.get_logger().info(f"Estado cambiado: {old_state} → {self.current_state}")
        
        # Si dejamos de estar en modo Nav2, enviar goal STOP (pose actual)
        # Esto cancela la navegación en curso
        if old_state == 1 and self.current_state != 1:
            self.cancel_navigation()

    def callback_target(self, msg: Point):
        """
        Recibe target (lat, lon en Point.x, Point.y)
        Solo envía a Nav2 si estamos en estado POINTTOPOINT
        """
        if self.current_state != 1:  # Solo en POINTTOPOINT
            self.get_logger().debug(
                f"Estado {self.current_state} no usa Nav2, ignorando target"
            )
            return
        
        self.send_nav2_goal(msg.x, msg.y)

    def send_nav2_goal(self, x: float, y: float):
        """
        Envía goal a Nav2 vía /goal_pose
        Asume que x,y ya están en el frame 'map'
        """
        goal_msg = PoseStamped()
        
        # Header
        goal_msg.header.frame_id = "odom"
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Posición (asumiendo que ya vienen convertidas a map frame)
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.position.z = 0.0
        
        # Orientación (w=1 significa "sin rotación preferida")
        # Nav2 calculará la orientación óptima
        goal_msg.pose.orientation.x = 0.0
        goal_msg.pose.orientation.y = 0.0
        goal_msg.pose.orientation.z = 0.0
        goal_msg.pose.orientation.w = 1.0
        
        self.get_logger().info(f"📍 Enviando goal a Nav2: ({x:.2f}, {y:.2f})")
        self.pub_goal.publish(goal_msg)
        self.last_goal_sent = goal_msg

    def cancel_navigation(self):
        """
        Para cancelar navegación en Nav2, simplemente no hay que hacer nada.
        El behavior tree de Nav2 se encargará cuando cambiemos a control directo.
        Opcionalmente, podrías enviar el goal a la pose actual.
        """
        self.get_logger().info("Saliendo de modo Nav2")
        # Nav2 se detendrá cuando otro nodo tome control de /cmd_vel


def main(args=None):
    rclpy.init(args=args)
    node = Nav2Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()