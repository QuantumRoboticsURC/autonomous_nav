#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LaserFilter180(Node):
    def __init__(self):
        super().__init__('laser_filter_180')
        
        # Parámetros
        self.declare_parameter('lower_angle', -math.pi/2)  # -90 grados
        self.declare_parameter('upper_angle', math.pi/2)   # +90 grados
        
        self.lower_angle = self.get_parameter('lower_angle').value
        self.upper_angle = self.get_parameter('upper_angle').value
        
        # Subscriber y Publisher
        self.sub = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.scan_callback, 
            10
        )
        self.pub = self.create_publisher(LaserScan, '/scan_filtered', 10)
        
        self.get_logger().info(f'Laser Filter 180 started')
        self.get_logger().info(f'Filtering from {self.lower_angle:.2f} to {self.upper_angle:.2f} rad')
        self.get_logger().info(f'({math.degrees(self.lower_angle):.1f}° to {math.degrees(self.upper_angle):.1f}°)')
        
    def scan_callback(self, msg):
        # Crear mensaje filtrado
        filtered_msg = LaserScan()
        filtered_msg.header = msg.header
        filtered_msg.range_min = msg.range_min
        filtered_msg.range_max = msg.range_max
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.time_increment = msg.time_increment
        
        # Definir nuevos límites angulares
        filtered_msg.angle_min = self.lower_angle
        filtered_msg.angle_max = self.upper_angle
        
        # Filtrar puntos
        filtered_ranges = []
        filtered_intensities = []
        
        for i in range(len(msg.ranges)):
            angle = msg.angle_min + i * msg.angle_increment
            
            # Normalizar ángulo a [-pi, pi]
            while angle > math.pi:
                angle -= 2 * math.pi
            while angle < -math.pi:
                angle += 2 * math.pi
            
            # Mantener solo si está dentro del rango deseado
            if self.lower_angle <= angle <= self.upper_angle:
                filtered_ranges.append(msg.ranges[i])
                if len(msg.intensities) > i:
                    filtered_intensities.append(msg.intensities[i])
        
        # Si no hay puntos filtrados, no publicar
        if len(filtered_ranges) == 0:
            return
        
        filtered_msg.ranges = filtered_ranges
        filtered_msg.intensities = filtered_intensities
        filtered_msg.angle_increment = (filtered_msg.angle_max - filtered_msg.angle_min) / max(len(filtered_ranges) - 1, 1)
        
        self.pub.publish(filtered_msg)
     
def main(args=None):
    rclpy.init(args=args)
    node = LaserFilter180()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()