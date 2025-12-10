#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64, String, Float64MultiArray, Int8
from .submodules.alvinxy import *
from enum import Enum, auto



def normalize_angle(angle: float) -> float:

    return math.atan2(math.sin(angle), math.cos(angle))


class MotionState(Enum):
    STOP = auto()
    POINTTOPOINT = auto()
    CENTERANDAPPROACH = auto()
    SEARCH = auto()
    AVOIDOBSTACLE = auto() 


class PointToPointMotion(Node):
    def __init__(self):
        super().__init__("point_to_point_motion")
        self.get_logger().info("Point-to-Point Motion node started")

        # Publishers
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 1)
        self.pub_arrived = self.create_publisher(Bool, "/arrived", 1)
        self.pub_done_aruco = self.create_publisher(Bool, "/done_aruco", 1)

        # Subscribers
        self.create_subscription(Point, "/target_point", self.callback_target, 10)
        self.create_subscription(Float64MultiArray, "/gps_odom", self.callback_odom, 10)
        self.create_subscription(Float64, "/angle", self.callback_angle, 10)
        self.create_subscription(Float64MultiArray, '/gps_origin', self.callback_gps_origin, 10)
        self.create_subscription(Float64MultiArray, '/object_offset', self.callback_object_offset, 10)
        self.create_subscription(Float64, '/distance_to_object', self.callback_distance, 10)
        self.create_subscription(Int8, '/state_command', self.callback_state_command, 10)

        # Timer de control (50 Hz)
        self.create_timer(0.02, self.control_loop)

        # Estado actual del robot
        self.x = None
        self.y = None
        self.theta = None  

        # Target actual
        self.x_d = None
        self.y_d = None

        # Máquina de estados interna
        self.state = MotionState.STOP

        # Parámetros de control
        self.kv = 0.3          # ganancia lineal
        self.kw = 1.0          # ganancia angular
        self.dist_tolerance = 0.10   # distancia para considerar "llegué"
        self.max_ang_speed = 0.3     # rad/s
        self.offset_center_tolerance = 0.05  # metros

        # Origen GPS
        self.long_origin = None
        self.lat_origin = None

        # Offset del objeto detectado
        self.offset_x = 0.0
        self.offset_y = 0.0

        # Distancia al objeto
        self.distance_to_object = 0.0

        self.arrived_msg = Bool()
        self.arrived_aruco_msg = Bool()
        self.already_arrived = False

    
    # ---------- Callbacks ----------

    def callback_state_command(self, msg: Int8):

        if msg.data == 0:
            self.state = MotionState.STOP
        elif msg.data == 1:
            self.state = MotionState.POINTTOPOINT
        elif msg.data == 2:
            self.state = MotionState.CENTERANDAPPROACH
        elif msg.data == 3:
            self.state = MotionState.SEARCH
        elif msg.data == 4:
            self.state = MotionState.AVOIDOBSTACLE
        else:
            self.get_logger().warn(f"Received unknown state command: {msg.data}")
            return

        self.get_logger().info(f"Motion state changed to: {self.state.name}")
        
        
    def callback_distance(self, msg: Float64):
        self.distance_to_object = msg.data


    def callback_object_offset(self, msg: Float64MultiArray):
        self.offset_x = msg.data[0]
        self.offset_y = msg.data[1]

    def callback_target(self, msg: Point):

        if self.lat_origin is None or self.long_origin is None:
            self.get_logger().warn("GPS origin not set yet. Cannot process target.")
            return
        self.x_d, self.y_d = ll2xy(msg.x, msg.y, self.lat_origin, self.long_origin)
  
        self.get_logger().info( f"[P2P] Nuevo target recibido: ({self.x_d:.2f}, {self.y_d:.2f}) → estado POINTTOPOINT")

    def callback_gps_origin(self, msg: Float64MultiArray):
        self.lat_origin = msg.data[0]
        self.long_origin = msg.data[1]
    

    def callback_odom(self, msg: Float64MultiArray):
        self.x = msg.data[0]
        self.y = msg.data[1]

    def callback_angle(self, msg: Float64):
        self.theta = msg.data

    # ---------- Bucle de control ----------

    def control_loop(self):
        if None in (self.x, self.y, self.theta):
            return
        
        if self.state == MotionState.STOP:
            self.stop_robot()
        
        elif self.state == MotionState.POINTTOPOINT:
            if None in (self.x_d, self.y_d):
                self.get_logger().warn("[P2P] No target yet.")
                self.stop_robot()
                return
            self.point_to_point()
        elif self.state == MotionState.CENTERANDAPPROACH:
            self.center_and_approach()
        elif self.state == MotionState.SEARCH:
            self.search_routine()
        elif self.state == MotionState.AVOIDOBSTACLE:
            self.avoid_obstacle()
        

    def search_routine(self):
        pass  # todo

    def avoid_obstacle(self):
        pass  # todo


    def center_and_approach(self):
        cmd = Twist()

        self.get_logger().info("[CENTER] Centrándose en el objeto...")

        if abs(self.offset_x) > self.offset_center_tolerance:
            cmd.linear.x = 0.0
            cmd.angular.z = -max(-self.max_ang_speed, min(self.max_ang_speed, self.offset_x * self.kw))
            self.arrived_aruco_msg.data = False
            self.pub_done_aruco.publish(self.arrived_aruco_msg)
        
        elif self.distance_to_object > 1.0:
            cmd.linear.x = self.kv
            cmd.angular.z = 0.0
            self.arrived_aruco_msg.data = False
            self.pub_done_aruco.publish(self.arrived_aruco_msg)

        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.arrived_aruco_msg.data = True
            self.pub_done_aruco.publish(self.arrived_aruco_msg)
            self.get_logger().info("[CENTER] Objeto centrado y alcanzado.")
            

        self.pub_cmd.publish(cmd)



    def point_to_point(self):
        # Necesitamos pose + target
        if None in (self.x, self.y, self.theta, self.x_d, self.y_d):
            return

        cmd = Twist()

        Dx = self.x_d - self.x
        Dy = self.y_d - self.y

        distance = math.sqrt(Dx**2 + Dy**2)

        self.get_logger().info(
            f"[P2P] dist={distance:.3f}, pose=({self.x:.2f},{self.y:.2f}), "
            f"goal=({self.x_d:.2f},{self.y_d:.2f})")
       
        if distance > self.dist_tolerance:
            
            angle_to_goal = math.atan2(Dy, Dx)
            angle_error = normalize_angle(angle_to_goal - self.theta)

            if abs(angle_error) > 0.2:

                cmd.linear.x = 0.0
                cmd.angular.z = max(-self.max_ang_speed, min(self.max_ang_speed, self.kw * angle_error))
            else:
                
                cmd.linear.x = self.kv
                cmd.angular.z = max(-self.max_ang_speed, min(self.max_ang_speed, self.kw * angle_error))

            self.arrived_msg.data = False
            self.pub_arrived.publish(self.arrived_msg)

        else:
            
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

            self.arrived_msg.data = True
            self.pub_arrived.publish(self.arrived_msg)

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
