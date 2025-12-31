#!/usr/bin/env python3
import rclpy, math, time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


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

        self.create_timer(0.01, self.odometry_callback)

        self.pub = self.create_publisher(Odometry, '/wheel/odom', 1)
        self.create_subscription(Twist, "/cmd_vel", self.call_vel, 10)
        self.create_subscription(Imu, "/bno055/imu", self.callback_imu, 10)
        
        # Initialize variables
        self.vel_linear = 0.0
        self.x = 0.0 
        self.y = 0.0 
        self.q = 0.0 
        self.t0 = time.time() 
        
        # Initialize the odometry message
        self.odom_msg = Odometry()
        self.imu_data = Imu()

    def callback_imu(self, data):

        self.imu_data = data
        _, _, angle_z = euler_from_quaternion(self.imu_data.orientation.x,
                                             self.imu_data.orientation.y,
                                             self.imu_data.orientation.z,
                                             self.imu_data.orientation.w)
        self.q = angle_z


    def call_vel(self, data):
        self.vel_linear = data.linear.x

    def odometry_callback(self):
        elapsed_time = time.time() - self.t0 


        if elapsed_time > 0.5:
            self.get_logger().warn(f'Large dt: {elapsed_time:.3f}s, skipping')
            self.t0 = time.time()
            return
        

        self.t0 = time.time()  
        v = self.vel_linear

        
        self.x += v * math.cos(self.q) * elapsed_time
        self.y += v * math.sin(self.q) * elapsed_time


        current_stamp = self.get_clock().now().to_msg()

        # Create odometry message
        self.odom_msg.header.stamp = current_stamp
        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.child_frame_id = "base_footprint"

        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.position.z = 0.0

        self.odom_msg.pose.pose.orientation.x = self.imu_data.orientation.x
        self.odom_msg.pose.pose.orientation.y = self.imu_data.orientation.y
        self.odom_msg.pose.pose.orientation.z = self.imu_data.orientation.z
        self.odom_msg.pose.pose.orientation.w = self.imu_data.orientation.w

        self.odom_msg.pose.covariance[0] = 0.05   # x
        self.odom_msg.pose.covariance[7] = 0.05   # y
        self.odom_msg.pose.covariance[35] = 0.01   


        self.odom_msg.twist.twist.linear.x = v
        self.odom_msg.twist.twist.angular.z = self.imu_data.angular_velocity.z

        self.odom_msg.twist.covariance[0] = 0.05
        self.odom_msg.twist.covariance[35] = 0.01 

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
