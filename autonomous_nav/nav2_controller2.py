#!/usr/bin/env python3
import rclpy
import math
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
        
        # ========================================
        # NUEVO: Suscripción a detección (para detener búsqueda)
        # ========================================
        self.create_subscription(Bool, '/object_detected', self.callback_object_detected, 10)
        
        # Estado
        self.active_goal = None
        self.current_state = None

        # GPS origin
        self.lat_origin = None
        self.long_origin = None
        
        # Buffer para target que llega antes del origin
        self.pending_target = None
        
        # ========================================
        # NUEVO: Variables para búsqueda en espiral
        # ========================================
        self.search_waypoints = []
        self.current_waypoint_index = 0
        self.search_center = None  # (x, y) del centro de búsqueda
        self.search_active = False
        self.object_found = False
        
        # Parámetros de búsqueda (configurables)
        self.declare_parameter('search_step_size', 2.0)      # Distancia entre pasos (metros)
        self.declare_parameter('search_max_radius', 20.0)    # Radio máximo de búsqueda (metros)
        self.declare_parameter('search_pattern', 'circle')   # 'square', 'circle', 'grid'
        
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

    # ========================================
    # NUEVO: Callback para detección de objeto
    # ========================================
    def callback_object_detected(self, msg: Bool):
        """
        Cuando se detecta el objeto, detener búsqueda
        """
        if msg.data and self.search_active:
            self.get_logger().info("🎯 ¡Objeto detectado! Deteniendo búsqueda")
            self.object_found = True
            self.stop_search()

    def callback_state_command(self, msg: Int8):
        """
        0 = STOP
        1 = POINTTOPOINT (usar Nav2)
        2 = CENTERANDAPPROACH (no usar Nav2)
        3 = SEARCH (búsqueda en espiral)
        """
        old_state = self.current_state
        self.current_state = msg.data
        
        # Log del cambio de estado
        self.get_logger().info(f"Estado: {old_state} → {self.current_state}")
        
        # ========================================
        # CAMBIO 1: Detener búsqueda al salir de estado SEARCH
        # ========================================
        if old_state == 3 and self.current_state != 3:
            self.get_logger().info("Saliendo de modo BÚSQUEDA")
            self.stop_search()
        
        # ========================================
        # CAMBIO 2: Iniciar búsqueda al entrar a estado SEARCH
        # ========================================
        if self.current_state == 3:  # SEARCH
            self.get_logger().info("🔍 Iniciando modo BÚSQUEDA")
            self.start_search()
        
        # ========================================
        # CAMBIO 3: Cancelar goals activos cuando cambiamos a estados no-Nav2
        # ========================================
        elif self.current_state in [0, 2]:  # STOP o CENTER_AND_APPROACH
            if self.active_goal is not None:
                self.get_logger().info(
                    f"Estado {old_state}→{self.current_state}, cancelando Nav2 goal"
                )
                self.cancel_current_goal()

    def callback_target(self, msg: Point):
        """
        Recibe target en lat/lon
        Point.x = latitud, Point.y = longitud
        """
        # ========================================
        # MODIFICADO: También guardar centro para búsqueda
        # ========================================
        
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
        
        # Guardar como centro potencial de búsqueda
        self.search_center = (x_d, y_d)
        
        # Solo enviar goal directo si estado es POINTTOPOINT
        if self.current_state == 1:
            self.send_nav2_goal(x_d, y_d)
        elif self.current_state == 3:
            # En modo búsqueda, ya se inició con start_search()
            pass
        else:
            self.get_logger().debug(
                f"Estado {self.current_state} no requiere nav2_goal inmediato"
            )

    # ========================================
    # NUEVO: Sistema de búsqueda en espiral
    # ========================================
    
    def start_search(self):
        """Inicia patrón de búsqueda desde posición actual o target"""
        
        if self.search_center is None:
            self.get_logger().error("No hay centro de búsqueda definido!")
            return
        
        # Reiniciar variables
        self.search_active = True
        self.object_found = False
        self.current_waypoint_index = 0
        
        # Obtener parámetros
        step_size = self.get_parameter('search_step_size').value
        max_radius = self.get_parameter('search_max_radius').value
        pattern = self.get_parameter('search_pattern').value
        
        # Generar waypoints según patrón
        if pattern == 'square':
            self.search_waypoints = self.generate_square_spiral(
                self.search_center, step_size, max_radius
            )
        elif pattern == 'circle':
            self.search_waypoints = self.generate_circular_spiral(
                self.search_center, step_size, max_radius
            )
        elif pattern == 'grid':
            self.search_waypoints = self.generate_grid_search(
                self.search_center, step_size, max_radius
            )
        else:
            self.get_logger().error(f"Patrón desconocido: {pattern}")
            return
        
        self.get_logger().info(
            f"🔍 Búsqueda iniciada: {len(self.search_waypoints)} waypoints, "
            f"patrón={pattern}, centro=({self.search_center[0]:.1f}, {self.search_center[1]:.1f})"
        )
        
        # Navegar primer waypoint
        self.navigate_next_search_waypoint()
    
    def generate_square_spiral(self, center, step_size, max_radius):
        """
        Genera espiral cuadrada desde el centro
        
        Patrón:
            5--6--7--8
            |        |
            4  1--2  9
            |  |  | |
            3--0  3 10
               |____|
        """
        waypoints = []
        cx, cy = center
        
        x, y = cx, cy
        waypoints.append((x, y))  # Centro
        
        direction = 0  # 0=Este, 1=Norte, 2=Oeste, 3=Sur
        steps_in_direction = 1
        steps_taken = 0
        direction_changes = 0
        
        while True:
            # Calcular siguiente posición
            if direction == 0:    # Este
                x += step_size
            elif direction == 1:  # Norte
                y += step_size
            elif direction == 2:  # Oeste
                x -= step_size
            elif direction == 3:  # Sur
                y -= step_size
            
            # Verificar si está dentro del radio máximo
            distance = math.sqrt((x - cx)**2 + (y - cy)**2)
            if distance > max_radius:
                break
            
            waypoints.append((x, y))
            steps_taken += 1
            
            # Cambiar dirección si completamos los pasos
            if steps_taken >= steps_in_direction:
                steps_taken = 0
                direction = (direction + 1) % 4
                direction_changes += 1
                
                # Aumentar pasos cada 2 cambios de dirección
                if direction_changes % 2 == 0:
                    steps_in_direction += 1
        
        return waypoints
    
    def generate_circular_spiral(self, center, step_size, max_radius):
        """Genera espiral circular (aproximada con segmentos)"""
        waypoints = []
        cx, cy = center
        
        angle = 0.0
        radius = 0.0
        angle_increment = math.radians(30)  # 30 grados por paso
        radius_increment = step_size / 6    # Radio crece gradualmente
        
        while radius <= max_radius:
            x = cx + radius * math.cos(angle)
            y = cy + radius * math.sin(angle)
            waypoints.append((x, y))
            
            angle += angle_increment
            radius += radius_increment
        
        return waypoints
    
    def generate_grid_search(self, center, step_size, max_radius):
        """Genera patrón de barrido en grid (líneas paralelas)"""
        waypoints = []
        cx, cy = center
        
        # Determinar límites del grid
        half_size = max_radius
        
        # Barrido horizontal (ida y vuelta)
        y = cy - half_size
        going_right = True
        
        while y <= cy + half_size:
            if going_right:
                x_start = cx - half_size
                x_end = cx + half_size
            else:
                x_start = cx + half_size
                x_end = cx - half_size
            
            # Generar puntos en esta línea
            x = x_start
            while (going_right and x <= x_end) or (not going_right and x >= x_end):
                waypoints.append((x, y))
                x += step_size if going_right else -step_size
            
            y += step_size
            going_right = not going_right
        
        return waypoints
    
    def navigate_next_search_waypoint(self):
        """Navega al siguiente waypoint de búsqueda"""
        
        if not self.search_active:
            return
        
        if self.object_found:
            self.get_logger().info("Objeto encontrado, búsqueda completada")
            self.stop_search()
            return
        
        if self.current_waypoint_index >= len(self.search_waypoints):
            self.get_logger().warn(
                "⚠️ Búsqueda completada sin encontrar objeto"
            )
            self.stop_search()
            
            # Publicar arrived=False (no encontrado)
            arrived_msg = Bool()
            arrived_msg.data = False
            self.pub_arrived.publish(arrived_msg)
            return
        
        # Obtener siguiente waypoint
        x, y = self.search_waypoints[self.current_waypoint_index]
        
        self.get_logger().info(
            f"🔍 Waypoint {self.current_waypoint_index + 1}/"
            f"{len(self.search_waypoints)}: ({x:.1f}, {y:.1f})"
        )
        
        # Enviar goal
        self.send_nav2_goal(x, y, is_search=True)
    
    def stop_search(self):
        """Detiene búsqueda activa"""
        self.search_active = False
        self.search_waypoints = []
        self.current_waypoint_index = 0
        
        if self.active_goal is not None:
            self.cancel_current_goal()

    # ========================================
    # Método send_nav2_goal MODIFICADO
    # ========================================
    
    def send_nav2_goal(self, x: float, y: float, is_search=False):
        """
        Envía goal a Nav2 via action
        x, y en coordenadas del frame 'odom' (metros)
        is_search: True si es parte de búsqueda (auto-continúa al siguiente)
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
        
        prefix = "🔍" if is_search else "📍"
        self.get_logger().info(
            f"{prefix} Enviando goal: ({x:.2f}, {y:.2f})m [odom]"
        )
        
        # Cancelar goal anterior si existe
        if self.active_goal is not None:
            self.cancel_current_goal()
        
        # Enviar nuevo goal
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(
            lambda future: self.goal_response_callback(future, is_search)
        )

    def goal_response_callback(self, future, is_search=False):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 rechazó el goal!')
            
            # Si es búsqueda, intentar siguiente waypoint
            if is_search and self.search_active:
                self.get_logger().warn("Saltando al siguiente waypoint de búsqueda")
                self.current_waypoint_index += 1
                self.navigate_next_search_waypoint()
            else:
                arrived_msg = Bool()
                arrived_msg.data = False
                self.pub_arrived.publish(arrived_msg)
            return

        self.get_logger().info('Nav2 aceptó el goal')
        self.active_goal = goal_handle
        
        # Esperar resultado
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(
            lambda future: self.get_result_callback(future, is_search)
        )

    def feedback_callback(self, feedback_msg):
        """Callback para feedback de Nav2"""
        # Descomentar para debug:
        # distance = feedback_msg.feedback.distance_remaining
        # self.get_logger().debug(f'Distancia restante: {distance:.2f}m')
        pass

    def get_result_callback(self, future, is_search=False):
        result = future.result().result
        status = future.result().status
        
        arrived_msg = Bool()
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('✅ Nav2 COMPLETADO!')
            
            # Si es búsqueda, continuar al siguiente waypoint
            if is_search and self.search_active:
                self.current_waypoint_index += 1
                self.navigate_next_search_waypoint()
            else:
                arrived_msg.data = True
                self.pub_arrived.publish(arrived_msg)
        
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn('⚠️ Nav2 goal ABORTADO')
            
            # Si es búsqueda, intentar siguiente
            if is_search and self.search_active:
                self.get_logger().warn("Saltando al siguiente waypoint")
                self.current_waypoint_index += 1
                self.navigate_next_search_waypoint()
            else:
                arrived_msg.data = False
                self.pub_arrived.publish(arrived_msg)
        
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Nav2 goal CANCELADO')
            if not is_search:
                arrived_msg.data = False
                self.pub_arrived.publish(arrived_msg)
        
        else:
            self.get_logger().warn(f'Nav2 status desconocido: {status}')
            if not is_search:
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