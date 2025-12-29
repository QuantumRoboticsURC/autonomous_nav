#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from .submodules.alvinxy import *
from geometry_msgs.msg import Point
from std_msgs.msg import Int8, Bool, Float64MultiArray
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


class Nav2Controller(Node):
    def __init__(self):
        super().__init__("nav2_controller")

        # Action client para Nav2
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # QoS para targets que deben persistir
        latching_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # Publishers
        self.pub_arrived = self.create_publisher(Bool, "/nav2_arrived", 10)
        
        # Subscribers
        self.create_subscription(Point, "/target_point", self.callback_target, latching_qos)
        self.create_subscription(Int8, '/state_command', self.callback_state_command, 10)
        self.create_subscription(Float64MultiArray, '/gps_origin', self.callback_gps_origin, latching_qos)
        
        # Estado
        self.active_goal = None
        self.current_state = None

        # GPS origin
        self.lat_origin = None
        self.long_origin = None
        
        # Buffer para target que llega antes del origin
        self.pending_target = None
        
        self.get_logger().info("Esperando servidor Nav2...")
        self._action_client.wait_for_server()
        self.get_logger().info("✓ Nav2 servidor listo!")

    def callback_gps_origin(self, msg: Float64MultiArray):
        self.lat_origin = msg.data[0]
        self.long_origin = msg.data[1]
        self.get_logger().info(
            f"GPS origin: lat={self.lat_origin:.6f}, lon={self.long_origin:.6f}"
        )
        
        # Procesar target pendiente si existe
        if self.pending_target is not None:
            self.get_logger().info("Procesando target pendiente...")
            self.callback_target(self.pending_target)
            self.pending_target = None

    def callback_state_command(self, msg: Int8):
        """
        0 = STOP
        1 = POINTTOPOINT (usar Nav2)
        2 = CENTERANDAPPROACH (no usar Nav2)
        3 = SEARCH (no usar Nav2)
        """
        old_state = self.current_state
        self.current_state = msg.data
        
        # Si cambiamos a un estado que NO usa Nav2, cancelar goal activo
        if old_state == 1 and self.current_state in [0, 2, 3]:
            self.get_logger().info(
                f"Estado {old_state}→{self.current_state}, cancelando Nav2 goal"
            )
            if self.active_goal is not None:
                self.cancel_current_goal()

    def callback_target(self, msg: Point):
        """
        Recibe target en lat/lon
        Point.x = latitud, Point.y = longitud
        """
        if self.current_state != 1:  # Solo POINTTOPOINT
            self.get_logger().debug(
                f"Estado {self.current_state} no usa Nav2, ignorando target"
            )
            return
        
        # Verificar GPS origin
        if self.lat_origin is None or self.long_origin is None:
            self.get_logger().warn("GPS origin no disponible. Target buffered.")
            self.pending_target = msg
            return
        
        # Convertir lat/lon a coordenadas locales
        x_d, y_d = ll2xy(msg.x, msg.y, self.lat_origin, self.long_origin)
        
        self.get_logger().info(
            f"Target: lat/lon({msg.x:.6f},{msg.y:.6f}) → "
            f"xy({x_d:.2f},{y_d:.2f})m"
        )
        
        self.send_nav2_goal(x_d, y_d)

    def send_nav2_goal(self, x: float, y: float):
        """
        Envía goal a Nav2 via action
        x, y en coordenadas del frame 'odom' (metros)
        """
        goal_msg = NavigateToPose.Goal()
        
        # Crear PoseStamped
        goal_msg.pose.header.frame_id = "odom" 
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Posición en frame odom
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        
        # Orientación (w=1 permite que Nav2 elija la mejor)
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0
        
        self.get_logger().info(f"Enviando goal a Nav2: ({x:.2f}, {y:.2f})m [odom]")
        
        # Cancelar goal anterior si existe
        if self.active_goal is not None:
            self.get_logger().warn("Cancelando goal anterior...")
            self.cancel_current_goal()
        
        # Enviar nuevo goal
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 rechazó el goal!')
            # Publicar arrived=False
            arrived_msg = Bool()
            arrived_msg.data = False
            self.pub_arrived.publish(arrived_msg)
            return

        self.get_logger().info('Nav2 aceptó el goal')
        self.active_goal = goal_handle
        
        # Esperar resultado
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """Callback para feedback de Nav2"""
        feedback = feedback_msg.feedback
        # Descomentar para debug:
        # distance = feedback.distance_remaining
        # self.get_logger().debug(f'Distancia restante: {distance:.2f}m')

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        
        arrived_msg = Bool()
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Nav2 COMPLETADO!')
            arrived_msg.data = True
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn('Nav2 goal ABORTADO')
            arrived_msg.data = False
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Nav2 goal CANCELADO')
            arrived_msg.data = False
        else:
            self.get_logger().warn(f'Nav2 status: {status}')
            arrived_msg.data = False
        
        self.pub_arrived.publish(arrived_msg)
        self.active_goal = None

    def cancel_current_goal(self):
        if self.active_goal is not None:
            self.get_logger().info("Cancelando goal de Nav2...")
            cancel_future = self.active_goal.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done)
            self.active_goal = None 

    def cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal cancelado')
        else:
            self.get_logger().warn('Fallo al cancelar goal')


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