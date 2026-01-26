#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64, Float64MultiArray, Int8
from .submodules.alvinxy import *
from enum import Enum, auto
import time


class MotionState(Enum):
    STOP = auto()
    PASSTHROUGH = auto()
    CENTERANDAPPROACH = auto()
    SEARCH = auto()


class PointToPointMotion(Node):
    def __init__(self):
        super().__init__("direct_motion_controller")
        self.get_logger().info("Direct Motion Controller started")

        # Publishers
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 1)
        self.pub_done_aruco = self.create_publisher(Bool, "/done_aruco", 1)

        # Subscribers
        self.create_subscription(Float64MultiArray, '/object_offset', self.callback_object_offset, 10)
        self.create_subscription(Float64, '/distance_to_object', self.callback_distance, 10)
        self.create_subscription(Int8, '/state_command', self.callback_state_command, 10)

        # Timer de control (50 Hz)
        self.create_timer(0.02, self.control_loop)

        # Máquina de estados
        self.state = MotionState.STOP
        
        # ========================================
        # Parámetros de control (más conservadores)
        # ========================================
        self.kv_approach = 0.15       # ← Reducido de 0.2 (más suave)
        self.kw = 1.5                 # ← Reducido de 2.5 (menos agresivo)
        self.max_linear_speed = 0.25  # ← NUEVO: Límite lineal
        self.max_ang_speed = 0.4
        
        # ========================================
        # Tolerancias con HISTÉRESIS
        # ========================================
        # Offset lateral (centrado)
        self.offset_center_tolerance = 0.05      # Umbral para considerar centrado
        self.offset_center_hysteresis = 0.08     # ← NUEVO: Umbral para descentrarse
        
        # Distancia
        self.distance_tolerance = 0.10           # Umbral para considerar llegada
        self.distance_hysteresis = 0.15          # ← NUEVO: Umbral para reiniciar avance
        self.distance_target = 0.5
        
        # ========================================
        # Variables de estado con filtrado
        # ========================================
        # Offset del objeto (con filtro paso bajo)
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.offset_x_filtered = 0.0  # ← NUEVO: Valor filtrado
        self.filter_alpha = 0.3       # ← NUEVO: Factor de suavizado (0-1)
        
        # Distancia
        self.distance_to_object = 0.0
        self.distance_filtered = 0.0  # ← NUEVO: Distancia filtrada
        
        # ========================================
        # Control suavizado de velocidades
        # ========================================
        self.cmd_linear_x = 0.0       # ← NUEVO: Velocidad lineal actual
        self.cmd_angular_z = 0.0      # ← NUEVO: Velocidad angular actual
        self.accel_limit = 0.5        # ← NUEVO: m/s² (aceleración máxima)
        self.angular_accel_limit = 2.0 # ← NUEVO: rad/s² 
        
        # ========================================
        # Estados internos para histéresis
        # ========================================
        self.is_centered = False      # ← NUEVO: Bandera de centrado
        self.is_at_distance = False   # ← NUEVO: Bandera de distancia alcanzada
        
        # Control de mensajes
        self.arrived_aruco_msg = Bool()
        self.last_log_time = time.time()
        self.log_interval = 0.5       # ← NUEVO: Log cada 0.5s (no cada ciclo)
        
        # ========================================
        # Timeout de detección
        # ========================================
        self.last_offset_time = time.time()
        self.last_distance_time = time.time()
        self.detection_timeout = 2.0  # ← NUEVO: 2s sin detección = fallo

    # ========================================
    # Callbacks
    # ========================================
    
    def callback_state_command(self, msg: Int8):
        new_state = None
        
        if msg.data == 0:
            new_state = MotionState.STOP
        elif msg.data == 1 or msg.data == 3:
            new_state = MotionState.PASSTHROUGH
        elif msg.data == 2:
            new_state = MotionState.CENTERANDAPPROACH
        else:
            self.get_logger().warn(f"Received unknown state command: {msg.data}")
            return
        
        # Solo log si cambia el estado
        if new_state != self.state:
            self.get_logger().info(f"Motion state changed to: {new_state.name}")
            self.state = new_state
            
            # Resetear banderas al entrar en CENTER_AND_APPROACH
            if self.state == MotionState.CENTERANDAPPROACH:
                self.is_centered = False
                self.is_at_distance = False

    def callback_distance(self, msg: Float64):
        self.distance_to_object = msg.data
        self.last_distance_time = time.time()
        
        # ========================================
        # Filtro paso bajo para distancia
        # ========================================
        self.distance_filtered = (self.filter_alpha * msg.data + 
                                  (1 - self.filter_alpha) * self.distance_filtered)

    def callback_object_offset(self, msg: Float64MultiArray):
        if len(msg.data) >= 2:
            self.offset_x = msg.data[0]
            self.offset_y = msg.data[1]
            self.last_offset_time = time.time()
            
            # ========================================
            # Filtro paso bajo para offset
            # ========================================
            self.offset_x_filtered = (self.filter_alpha * msg.data[0] + 
                                      (1 - self.filter_alpha) * self.offset_x_filtered)

    # ========================================
    # Control Loop
    # ========================================
    
    def control_loop(self):
        if self.state == MotionState.STOP:
            self.stop_robot()
        elif self.state == MotionState.PASSTHROUGH:
            pass  # Nav2 controla
        elif self.state == MotionState.CENTERANDAPPROACH:
            self.center_and_approach()

    # ========================================
    # Center and Approach con HISTÉRESIS y SUAVIZADO
    # ========================================
    
    def center_and_approach(self):
        # Verificar timeout de detección
        current_time = time.time()
        if (current_time - self.last_offset_time > self.detection_timeout or
            current_time - self.last_distance_time > self.detection_timeout):
            self.get_logger().warn("[CENTER] ⚠️ Detección perdida, deteniendo")
            self.stop_robot()
            self.arrived_aruco_msg.data = False
            self.pub_done_aruco.publish(self.arrived_aruco_msg)
            return
        
        # Usar valores filtrados
        lateral_error = abs(self.offset_x_filtered)
        distance_error = abs(self.distance_filtered - self.distance_target)
        
        # ========================================
        # HISTÉRESIS para estado de centrado
        # ========================================
        if self.is_centered:
            # Ya estaba centrado, usar umbral más amplio para descentrarse
            if lateral_error > self.offset_center_hysteresis:
                self.is_centered = False
        else:
            # No estaba centrado, usar umbral más estricto para centrarse
            if lateral_error < self.offset_center_tolerance:
                self.is_centered = True
        
        # ========================================
        # HISTÉRESIS para distancia
        # ========================================
        if self.is_at_distance:
            # Ya estaba a distancia, usar umbral más amplio
            if distance_error > self.distance_hysteresis:
                self.is_at_distance = False
        else:
            # No estaba a distancia, usar umbral más estricto
            if distance_error < self.distance_tolerance:
                self.is_at_distance = True
        
        # ========================================
        # Calcular comandos deseados
        # ========================================
        desired_linear = 0.0
        desired_angular = 0.0
        
        if not self.is_centered:
            # FASE 1: Centrar (sin avanzar)
            desired_linear = 0.0
            desired_angular = -self.kw * self.offset_x_filtered
            status = "CENTERING"
            
        elif not self.is_at_distance:
            # FASE 2: Avanzar (ya centrado)
            if self.distance_filtered > self.distance_target:
                desired_linear = self.kv_approach * distance_error
            else:
                desired_linear = -self.kv_approach * 0.5 * distance_error
            
            # ========================================
            # MEJORA: Corrección angular suave mientras avanza
            # ========================================
            desired_angular = -self.kw * 0.3 * self.offset_x_filtered  # 30% de corrección
            status = "APPROACHING"
            
        else:
            # FASE 3: Objetivo alcanzado
            desired_linear = 0.0
            desired_angular = 0.0
            status = "REACHED"
        
        # ========================================
        # Aplicar límites de velocidad
        # ========================================
        desired_linear = max(-self.max_linear_speed, 
                            min(self.max_linear_speed, desired_linear))
        desired_angular = max(-self.max_ang_speed, 
                             min(self.max_ang_speed, desired_angular))
        
        # ========================================
        # SUAVIZADO con límites de aceleración
        # ========================================
        dt = 0.02  # 50 Hz
        
        # Suavizar velocidad lineal
        linear_diff = desired_linear - self.cmd_linear_x
        max_linear_change = self.accel_limit * dt
        if abs(linear_diff) > max_linear_change:
            linear_diff = max_linear_change if linear_diff > 0 else -max_linear_change
        self.cmd_linear_x += linear_diff
        
        # Suavizar velocidad angular
        angular_diff = desired_angular - self.cmd_angular_z
        max_angular_change = self.angular_accel_limit * dt
        if abs(angular_diff) > max_angular_change:
            angular_diff = max_angular_change if angular_diff > 0 else -max_angular_change
        self.cmd_angular_z += angular_diff
        
        # ========================================
        # Publicar comandos
        # ========================================
        cmd = Twist()
        cmd.linear.x = self.cmd_linear_x
        cmd.angular.z = self.cmd_angular_z
        self.pub_cmd.publish(cmd)
        
        # ========================================
        # Publicar estado de llegada
        # ========================================
        self.arrived_aruco_msg.data = self.is_at_distance and self.is_centered
        self.pub_done_aruco.publish(self.arrived_aruco_msg)
        
        # ========================================
        # Log con throttling (no 50 veces por segundo)
        # ========================================
        if current_time - self.last_log_time > self.log_interval:
            self.get_logger().info(
                f"[{status}] offset={lateral_error:.3f}m, "
                f"dist={self.distance_filtered:.2f}m, "
                f"v={self.cmd_linear_x:.2f}, w={self.cmd_angular_z:.2f}"
            )
            self.last_log_time = current_time

    # ========================================
    # Helpers
    # ========================================
    
    def stop_robot(self):
        # Suavizar detención (no frenar abruptamente)
        cmd = Twist()
        
        # Desacelerar gradualmente
        dt = 0.02
        
        # Velocidad lineal
        if abs(self.cmd_linear_x) > 0.01:
            decel = -self.accel_limit * dt if self.cmd_linear_x > 0 else self.accel_limit * dt
            self.cmd_linear_x += decel
        else:
            self.cmd_linear_x = 0.0
        
        # Velocidad angular
        if abs(self.cmd_angular_z) > 0.01:
            decel = -self.angular_accel_limit * dt if self.cmd_angular_z > 0 else self.angular_accel_limit * dt
            self.cmd_angular_z += decel
        else:
            self.cmd_angular_z = 0.0
        
        cmd.linear.x = self.cmd_linear_x
        cmd.angular.z = self.cmd_angular_z
        self.pub_cmd.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PointToPointMotion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()