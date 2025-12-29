#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64, String, Float64MultiArray, Int8
from .submodules.alvinxy import *
from enum import Enum, auto
from rclpy.qos import QoSProfile, QoSDurabilityPolicy


class MotionState(Enum):
    STOP = auto()
    PASSTHROUGH = auto()  # Nav2 al mando
    CENTERANDAPPROACH = auto()
    SEARCH = auto()


class PointToPointMotion(Node):
    def __init__(self):
        super().__init__("direct_motion_controller")
        self.get_logger().info("Direct Motion Controller started")

        latching_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )       

        # Publishers
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 1)
        self.pub_arrived = self.create_publisher(Bool, "/arrived", 1)
        self.pub_done_aruco = self.create_publisher(Bool, "/done_aruco", 1)

    
        #self.create_subscription(Float64MultiArray, '/gps_origin', self.callback_gps_origin, latching_qos)

        self.create_subscription(Float64MultiArray, '/object_offset', self.callback_object_offset, 10)
        self.create_subscription(Float64, '/distance_to_object', self.callback_distance, 10)
        self.create_subscription(Int8, '/state_command', self.callback_state_command, 10)

        # Timer de control (50 Hz)
        self.create_timer(0.02, self.control_loop)

        # Máquina de estados interna
        self.state = MotionState.STOP


        # # Estado actual del robot
        # self.x = None
        # self.y = None
        # self.theta = None  

        # # Target actual
        # self.x_d = None
        # self.y_d = None

        
        # Parámetros de control
        self.kv_approach = 0.2          # ganancia lineal
        self.kw = 2.5          # ganancia angular
        self.max_ang_speed = 0.4     # rad/s


        self.distance_tolerance = 0.1
        self.offset_center_tolerance = 0.05
        self.distance_target = 0.5

        # # Origen GPS
        # self.long_origin = None
        # self.lat_origin = None

        # Offset del objeto detectado
        self.offset_x = 0.0
        self.offset_y = 0.0

        # Distancia al objeto
        self.distance_to_object = 0.0

 
        self.arrived_aruco_msg = Bool()

    
    # ---------- Callbacks ----------

    def callback_state_command(self, msg: Int8):

        if msg.data == 0:
            self.state = MotionState.STOP
        elif msg.data == 1:
            self.state = MotionState.PASSTHROUGH
        elif msg.data == 2:
            self.state = MotionState.CENTERANDAPPROACH
        elif msg.data == 3:
            self.state = MotionState.SEARCH
       
        else:
            self.get_logger().warn(f"Received unknown state command: {msg.data}")
            return

        self.get_logger().info(f"Motion state changed to: {self.state.name}")
        
        
    def callback_distance(self, msg: Float64):
        self.distance_to_object = msg.data


    def callback_object_offset(self, msg: Float64MultiArray):
        if len(msg.data) >= 2:
            self.offset_x = msg.data[0]
            self.offset_y = msg.data[1]

    # def callback_target(self, msg: Point):

    #     if self.lat_origin is None or self.long_origin is None:
    #         self.get_logger().warn("GPS origin not set yet. Cannot process target.")
    #         return
    #     self.x_d, self.y_d = ll2xy(msg.x, msg.y, self.lat_origin, self.long_origin)
  
    #     self.get_logger().info( f"[P2P] Nuevo target recibido: ({self.x_d:.2f}, {self.y_d:.2f}) → estado POINTTOPOINT")

    # def callback_gps_origin(self, msg: Float64MultiArray):
    #     self.lat_origin = msg.data[0]
    #     self.long_origin = msg.data[1]
    

    # def callback_odom(self, msg: Float64MultiArray):
    #     self.x = msg.data[0]
    #     self.y = msg.data[1]

    # def callback_angle(self, msg: Float64):
    #     self.theta = msg.data

    # ---------- Bucle de control ----------

    def control_loop(self):
   
        if self.state == MotionState.STOP:
            self.stop_robot()
        elif self.state == MotionState.PASSTHROUGH:
            pass
        elif self.state == MotionState.CENTERANDAPPROACH:
            self.center_and_approach()
        elif self.state == MotionState.SEARCH:
            self.search_routine()
        

    def search_routine(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.3 
        self.pub_cmd.publish(cmd)


    def center_and_approach(self):
        cmd = Twist()

        lateral_error = abs(self.offset_x)
        
        # Error de distancia
        distance_error = abs(self.distance_to_object - self.distance_target)

        self.get_logger().info("[CENTER] Centrándose en el objeto...")

        if lateral_error > self.offset_center_tolerance:
            cmd.linear.x = 0.0
            cmd.angular.z = -self.kw * self.offset_x
            cmd.angular.z = max(-self.max_ang_speed, 
                               min(self.max_ang_speed, cmd.angular.z))
            # cmd.angular.z = -max(-self.max_ang_speed, min(self.max_ang_speed, self.offset_x * self.kw))
            self.arrived_aruco_msg.data = False
        
        elif distance_error > self.distance_tolerance:

            if self.distance_to_object > self.distance_target:
                cmd.linear.x = self.kv_approach
            else:
                cmd.linear.x = -self.kv_approach * 0.5  

            cmd.angular.z = 0.0
            self.arrived_aruco_msg.data = False

        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.arrived_aruco_msg.data = True
            self.get_logger().info("[CENTER] Objeto centrado y alcanzado.")
            
        
        self.pub_done_aruco.publish(self.arrived_aruco_msg)
        self.pub_cmd.publish(cmd)


    # ---------- Helpers ----------

    def stop_robot(self):

        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
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
