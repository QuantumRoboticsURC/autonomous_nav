#!/usr/bin/env python3
import rclpy, math, time
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import math
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Float64MultiArray, Float64
from rclpy.qos import QoSProfile, QoSDurabilityPolicy


def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts Euler angles (roll, pitch, yaw) to quaternion (x, y, z, w).
    Angles must be in radians.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return x, y, z, w

class OdometryClass(Node):
    def __init__(self):
        super().__init__("odometry")
        latching_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.get_logger().info("Robot pose estimation by odometry node.")
        self.create_timer(0.01, self.odometry_callback)
        self.pub = self.create_publisher(Odometry, '/odom', 1)

        self.create_subscription(Float64, "/latitude",  self.update_coords_latitude,  10)
        self.create_subscription(Float64, "/longitude", self.update_coords_longitude, 10)
        self.pub_origin = self.create_publisher(Float64MultiArray, "/gps_origin", latching_qos)

        self.create_subscription(Twist, "/cmd_vel", self.call_vel, 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.STARTED = True          # todavía no hemos fijado origen
        self.have_lat = False
        self.have_long = False

        self.current_lat  = 0.0
        self.current_long = 0.0
        
        # Initialize variables
        self.vel_linear = 0.0
        self.vel_angular = 0.0
        self.x = 0.0 
        self.y = 0.0 
        self.q = 0.0 
        self.t0 = time.time() 

        self.angular_correction = 0.8  # Factor de corrección para la velocidad angular

        self.org_lat  = 0.0
        self.org_long = 0.0

        self.origin_msg = Float64MultiArray()
        
        # Initialize the odometry message
        self.odom_msg = Odometry()

    def update_coords_latitude(self, msg: Float64):
        """
        /latitude: asumimos que msg.data viene en grados * 1e-7 (como antes con UBX).
        Si ya lo convertiste antes a grados "normales", quita el * 1e-7.
        """
        # Pasa a grados
        self.current_lat = msg.data
        self.have_lat = True

        # Si aún no hemos fijado origen y ya tenemos lat+lon, fijarlo
        if self.STARTED and self.have_lat and self.have_long:
            self.set_origin()

        # Si ya hay origen, publica posición local cuando tengamos lat+lon
        if not self.STARTED and self.have_lat and self.have_long:
            pass

    def update_coords_longitude(self, msg: Float64):
        """
        /longitude: mismo caso que latitud.
        """
        self.current_long = msg.data
        self.have_long = True   

        if self.STARTED and self.have_lat and self.have_long:
            self.set_origin()

        if not self.STARTED and self.have_lat and self.have_long:
            pass

    def set_origin(self):
        """Fija el origen GPS con la primera pareja lat/lon."""
        self.org_lat  = self.current_lat
        self.org_long = self.current_long
        self.STARTED = False

        self.origin_msg.data = [self.org_lat, self.org_long]
        self.pub_origin.publish(self.origin_msg)

        self.get_logger().info(
            f"Origin set to lat: {self.org_lat}, long: {self.org_long}"
        )


    def call_vel(self, data):
        self.vel_linear = data.linear.x
        self.vel_angular = data.angular.z * self.angular_correction


    def odometry_callback(self):
        elapsed_time = time.time() - self.t0 


        if elapsed_time > 0.5:
            self.get_logger().warn(f'Large dt: {elapsed_time:.3f}s, skipping')
            self.t0 = time.time()
            return
        

        self.t0 = time.time()  
        v = self.vel_linear
        w = self.vel_angular

        
        if abs(w) > 1e-6:
            dx = (v / w) * (math.sin(self.q + w * elapsed_time) - math.sin(self.q))
            dy = (v / w) * (-math.cos(self.q + w * elapsed_time) + math.cos(self.q))
        else:
            dx = v * math.cos(self.q) * elapsed_time
            dy = v * math.sin(self.q) * elapsed_time

        self.x += dx
        self.y += dy
        self.q += w * elapsed_time

        self.q = math.atan2(math.sin(self.q), math.cos(self.q))

        current_stamp = self.get_clock().now().to_msg()

        qx, qy, qz, qw = quaternion_from_euler(0, 0, self.q)

        t = TransformStamped()
        t.header.stamp = current_stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"  

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)


        # Create odometry message
        self.odom_msg.header.stamp = current_stamp
        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.child_frame_id = "base_footprint"
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.position.z = 0.0

        self.odom_msg.pose.pose.orientation.x = qx
        self.odom_msg.pose.pose.orientation.y = qy
        self.odom_msg.pose.pose.orientation.z = qz
        self.odom_msg.pose.pose.orientation.w = qw
        self.odom_msg.twist.twist.linear.x = v
        self.odom_msg.twist.twist.angular.z = w
        self.pub.publish(self.odom_msg)
 

def main(args=None):
    rclpy.init(args=args)
    nodeh = OdometryClass()
    try: rclpy.spin(nodeh)
    except Exception as error: print(error)
    except KeyboardInterrupt: print("\nNode terminated by user")
    finally:
        nodeh.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
