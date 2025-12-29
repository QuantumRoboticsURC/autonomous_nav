#!/usr/bin/env python3
import rclpy
import math
import time
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster  # ← NUEVO: Para publicar TF

def euler_from_quaternion(x, y, z, w):
    """Converts quaternion to Euler angles (roll, pitch, yaw)."""
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = max(min(t2, 1.0), -1.0)  # Clamping
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z  # in radians

class OdometryClass(Node):
    def __init__(self):
        super().__init__("odometry")
        self.get_logger().info("Robot pose estimation by odometry node.")
        
        # Timer para publicar odometría
        self.create_timer(0.01, self.odometry_callback)  # 100 Hz
        
        # Publishers
        self.pub = self.create_publisher(Odometry, 'odom', 10)
        self.pub_an = self.create_publisher(Float64, 'angle', 10)
        
        # TF Broadcaster ← NUEVO
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribers
        self.create_subscription(Twist, "/cmd_vel", self.call_vel, 10)
        self.create_subscription(Imu, "/bno055/imu", self.callback_imu, 10)
        
        # Variables de estado
        self.vel_linear = 0.0
        self.vel_angular = 0.0  # ← NUEVO: para guardarlo
        self.x = 0.0 
        self.y = 0.0 
        self.theta = 0.0  # Mejor nombre que 'q'
        self.t0 = time.time() 
        
        # Quaternion actual (desde IMU)
        self.quat_x = 0.0
        self.quat_y = 0.0
        self.quat_z = 0.0
        self.quat_w = 1.0
        
        self.pub_angle = Float64()
        
        self.get_logger().info("Odometry node initialized")
    
    def call_vel(self, data):
        """Callback para guardar velocidades comandadas"""
        self.vel_linear = data.linear.x
        self.vel_angular = data.angular.z  # ← NUEVO
    
    def callback_imu(self, data):
        """Callback del IMU - obtener orientación"""
        # Guardar quaternion completo ← NUEVO
        self.quat_x = data.orientation.x
        self.quat_y = data.orientation.y
        self.quat_z = data.orientation.z
        self.quat_w = data.orientation.w
        
        # Convertir a yaw para integración
        _, _, self.theta = euler_from_quaternion(
            self.quat_x, self.quat_y, self.quat_z, self.quat_w
        )
    
    def odometry_callback(self):
        """Publicar odometría y TF"""
        # Calcular tiempo transcurrido
        current_time = time.time()
        elapsed_time = current_time - self.t0 
        self.t0 = current_time
        
        # Integrar velocidad para calcular posición
        v = self.vel_linear
        self.x += v * math.cos(self.theta) * elapsed_time
        self.y += v * math.sin(self.theta) * elapsed_time
        
        # Timestamp actual
        current_stamp = self.get_clock().now().to_msg()
        
        # ============================================
        # PUBLICAR TRANSFORMACIÓN TF ← NUEVO
        # ============================================
        t = TransformStamped()
        t.header.stamp = current_stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"  # ← Cambiar a base_footprint
        
        # Posición
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        # Orientación (del IMU)
        t.transform.rotation.x = self.quat_x
        t.transform.rotation.y = self.quat_y
        t.transform.rotation.z = self.quat_z
        t.transform.rotation.w = self.quat_w
        
        # Publicar TF
        self.tf_broadcaster.sendTransform(t)
        
        # ============================================
        # PUBLICAR MENSAJE ODOMETRY ← MEJORADO
        # ============================================
        odom_msg = Odometry()
        odom_msg.header.stamp = current_stamp
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"  # ← Cambiar
        
        # Pose (posición y orientación)
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        # Orientación (del IMU) ← NUEVO
        odom_msg.pose.pose.orientation.x = self.quat_x
        odom_msg.pose.pose.orientation.y = self.quat_y
        odom_msg.pose.pose.orientation.z = self.quat_z
        odom_msg.pose.pose.orientation.w = self.quat_w
        
        # Velocidades ← NUEVO
        odom_msg.twist.twist.linear.x = self.vel_linear
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = self.vel_angular
        
        # Covarianza (opcional pero recomendado)
        # Valores bajos = alta confianza, valores altos = baja confianza
        odom_msg.pose.covariance[0] = 0.001  # x
        odom_msg.pose.covariance[7] = 0.001  # y
        odom_msg.pose.covariance[35] = 0.01  # theta
        
        odom_msg.twist.covariance[0] = 0.001  # vx
        odom_msg.twist.covariance[35] = 0.01  # vtheta
        
        # Publicar odometría
        self.pub.publish(odom_msg)
        
        # Publicar ángulo (para debug)
        self.pub_angle.data = self.theta
        self.pub_an.publish(self.pub_angle)

def main(args=None):
    rclpy.init(args=args)
    nodeh = OdometryClass()
    try:
        rclpy.spin(nodeh)
    except KeyboardInterrupt:
        print("\nNode terminated by user")
    finally:
        nodeh.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()