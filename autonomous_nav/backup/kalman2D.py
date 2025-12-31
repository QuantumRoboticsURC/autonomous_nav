#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np

from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray


class KalmanFusion2D(Node):
    def __init__(self):
        super().__init__("kalman_fusion_2d")

        self.get_logger().info("Kalman fusion node (odom + gps_odom)")

        self.x_est = np.zeros((2, 1))
        self.P = np.eye(2) * 1.0  # covarianza inicial

        # Aumenta si hay mucho patinaje
        self.Q = np.eye(2) * 0.05

        # Aumenta si el GPS es muy ruidoso
        self.R = np.eye(2) * 0.5

        self.has_odom = False
        self.has_gps = False
        self.last_odom_xy = None

        self.sub_odom = self.create_subscription(Odometry, "odom", self.odom_callback, 10)

        self.sub_gps = self.create_subscription(Float64MultiArray, "gps_odom", self.gps_callback, 10)

        self.pub_odom_fused = self.create_publisher(Odometry, "odom_fused", 10)

        self.last_odom_msg = None

    def odom_callback(self, msg: Odometry):
        self.last_odom_msg = msg

        x_odom = msg.pose.pose.position.x
        y_odom = msg.pose.pose.position.y

        if not self.has_odom:
            self.last_odom_xy = (x_odom, y_odom)
            self.x_est[0, 0] = x_odom
            self.x_est[1, 0] = y_odom
            self.has_odom = True
            self.get_logger().info("Odom initialized in KF")
            self.publish_fused(msg)  
            return

        dx = x_odom - self.last_odom_xy[0]
        dy = y_odom - self.last_odom_xy[1]
        self.last_odom_xy = (x_odom, y_odom)

        # Modelo de transición: x_k^- = x_{k-1} + delta
        F = np.eye(2)
        u = np.array([[dx], [dy]])  # entrada = delta odom

        # Predicción de estado
        self.x_est = F @ self.x_est + u

        # Predicción de covarianza
        self.P = F @ self.P @ F.T + self.Q

        self.publish_fused(msg)

    def gps_callback(self, msg: Float64MultiArray):
        if not self.has_odom:
            self.get_logger().warn("GPS received but no odom yet, skipping")
            return

        if len(msg.data) < 2:
            self.get_logger().warn("gps_odom message has less than 2 elements")
            return

        x_gps = msg.data[0]
        y_gps = msg.data[1]

        if not self.has_gps:
            self.has_gps = True
            # self.x_est[0, 0] = x_gps
            # self.x_est[1, 0] = y_gps
            self.get_logger().info("GPS initialized in KF")

        z = np.array([[x_gps], [y_gps]])

        H = np.eye(2)

        # Innovación
        y = z - H @ self.x_est
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        # Corrección de estado
        self.x_est = self.x_est + K @ y

        # Corrección de covarianza
        I = np.eye(2)
        self.P = (I - K @ H) @ self.P

    def publish_fused(self, odom_base: Odometry):
        fused = Odometry()
        fused.header = odom_base.header
        fused.child_frame_id = odom_base.child_frame_id

        fused.pose.pose.position.x = float(self.x_est[0, 0])
        fused.pose.pose.position.y = float(self.x_est[1, 0])
        fused.pose.pose.position.z = 0.0

        fused.pose.pose.orientation = odom_base.pose.pose.orientation

        self.pub_odom_fused.publish(fused)


def main(args=None):
    rclpy.init(args=args)
    node = KalmanFusion2D()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
