#!/usr/bin/env python3
"""
lidar_timestamp_fixer.py

Nodo que corrige timestamps del LIDAR para sincronizar con el reloj de la laptop.
Solo usar para desarrollo/testing remoto desde laptop.
NO usar en producción (usar NTP/chrony en su lugar).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarTimestampFixer(Node):
    def __init__(self):
        super().__init__('lidar_timestamp_fixer')
        
        # Parámetro: topic original del LIDAR
        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_fixed')
        self.declare_parameter('show_stats', True)
        
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        show_stats = self.get_parameter('show_stats').value
        
        # Subscriber al LIDAR original
        self.sub = self.create_subscription(
            LaserScan,
            input_topic,
            self.callback,
            10
        )
        
        # Publisher con timestamps corregidos
        self.pub = self.create_publisher(
            LaserScan,
            output_topic,
            10
        )
        
        # Estadísticas
        self.scan_count = 0
        self.first_scan = True
        self.original_time = None
        self.show_stats = show_stats
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("LIDAR Timestamp Fixer Started")
        self.get_logger().info(f"  Input:  {input_topic}")
        self.get_logger().info(f"  Output: {output_topic}")
        self.get_logger().info("  Mode: LAPTOP TESTING ONLY")
        self.get_logger().info("=" * 60)
    
    def callback(self, msg):
        # Guardar timestamp original (solo para stats)
        original_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        
        # ========================================
        # CORRECCIÓN: Reemplazar con tiempo actual
        # ========================================
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Timestamp corregido
        fixed_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        
        # Publicar scan corregido
        self.pub.publish(msg)
        
        # ========================================
        # Estadísticas (primera vez y cada 50 scans)
        # ========================================
        if self.show_stats:
            if self.first_scan:
                self.original_time = original_stamp
                self.get_logger().info("")
                self.get_logger().info("First scan received:")
                self.get_logger().info(f"  Original timestamp: {original_stamp:.3f}s")
                self.get_logger().info(f"  Fixed timestamp:    {fixed_stamp:.3f}s")
                self.get_logger().info(f"  Difference:         {fixed_stamp - original_stamp:.3f}s")
                self.get_logger().info("")
                self.first_scan = False
            
            elif self.scan_count % 50 == 0:
                diff = fixed_stamp - original_stamp
                self.get_logger().info(
                    f"Scan #{self.scan_count}: "
                    f"Original={original_stamp:.1f}s, "
                    f"Fixed={fixed_stamp:.1f}s, "
                    f"Diff={diff:.1f}s"
                )
        
        self.scan_count += 1


def main(args=None):
    rclpy.init(args=args)
    node = LidarTimestampFixer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down timestamp fixer...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()