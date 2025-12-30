#!/usr/bin/env python3
from .submodules.alvinxy import *
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from sensor_msgs.msg import Imu


class OdomByGPS(Node):
    def __init__(self):
        super().__init__("odom_by_gps")

        latching_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self.get_logger().info("Robot pose estimation by GPS node (lat/long topics).")

        # Suscripciones a latitud y longitud (Float64)
        # Suponemos que data.data viene en unidades "grados * 1e-7" como antes.
        self.create_subscription(Float64, "/latitude",  self.update_coords_latitude,  10)
        self.create_subscription(Float64, "/longitude", self.update_coords_longitude, 10)
        self.create_subscription(Imu, "/bno055/imu", self.imu_callback, 10)
        self.create_subscription(Twist, "/cmd_vel", self.call_vel, 10)

        # Publicaciones
        self.pub_odom   = self.create_publisher(Odometry, "/gps/odom", 10)
        self.pub_origin = self.create_publisher(Float64MultiArray, "/gps_origin", latching_qos)

        # Flags y estado
        self.STARTED = True          # todavía no hemos fijado origen
        self.have_lat = False
        self.have_long = False

        # Origen en grados
        self.org_lat  = 0.0
        self.org_long = 0.0

        # Coordenadas actuales en grados
        self.current_lat  = 0.0
        self.current_long = 0.0

        # Coordenadas locales en metros
        self.x_rover = 0.0
        self.y_rover = 0.0
        self.last_x = 0.0
        self.last_y = 0.0

        # Mensajes que vamos a reutilizar
        self.origin_msg = Float64MultiArray()

        self.imu_data = Imu()
        self.odom_msg = Odometry()

        self.vel_linear = 0.0
        self.last_time = None

    # ---------- Callbacks ----------


    def call_vel(self, data):
        
        self.vel_linear = data.linear.x
     

    def imu_callback(self, msg: Imu):
    
        self.imu_data = msg


    def update_coords_latitude(self, msg: Float64):
        """
        /latitude: asumimos que msg.data viene en grados * 1e-7 (como antes con UBX).
        Si ya lo convertiste antes a grados "normales", quita el * 1e-7.
        """
        # Pasa a grados
        self.current_lat = msg.data * 1e-7
        self.have_lat = True

        # Si aún no hemos fijado origen y ya tenemos lat+lon, fijarlo
        if self.STARTED and self.have_lat and self.have_long:
            self.set_origin()

        # Si ya hay origen, publica posición local cuando tengamos lat+lon
        if not self.STARTED and self.have_lat and self.have_long:
            self.publish_odometry()

    def update_coords_longitude(self, msg: Float64):
        """
        /longitude: mismo caso que latitud.
        """
        self.current_long = msg.data * 1e-7
        self.have_long = True   

        if self.STARTED and self.have_lat and self.have_long:
            self.set_origin()

        if not self.STARTED and self.have_lat and self.have_long:
            self.publish_odometry()

    # ---------- Helpers ----------

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

    def publish_odometry(self):
        """Convierte lat/lon actuales a (x,y) metros y publica /gps_odom."""
        self.x_rover, self.y_rover = ll2xy(
            self.current_lat,
            self.current_long,
            self.org_lat,
            self.org_long
        )

        # self.get_logger().info(
        #     f"Current position - X: {self.x_rover:.2f} m, Y: {self.y_rover:.2f} m"
        # )

        current_stamp = self.get_clock().now().to_msg()


        current_time = self.get_clock().now()
        if self.last_time is not None:
            dt = (current_time - self.last_time).nanoseconds * 1e-9
            if dt > 0 and dt < 1.0:  # Solo si dt es razonable
                vx = (self.x_rover - self.last_x) / dt
                vy = (self.y_rover - self.last_y) / dt
            else:
                vx = 0.0
                vy = 0.0
        else:
            vx = 0.0
            vy = 0.0

        self.last_x = self.x_rover
        self.last_y = self.y_rover
        self.last_time = current_time


        self.odom_msg.header.stamp = current_stamp
        self.odom_msg.header.frame_id = "map"
        self.odom_msg.child_frame_id = "base_footprint"
        self.odom_msg.pose.pose.position.x = self.x_rover
        self.odom_msg.pose.pose.position.y = self.y_rover
        self.odom_msg.pose.pose.position.z = 0.0
        self.odom_msg.pose.pose.orientation = self.imu_data.orientation


        self.odom_msg.pose.covariance[0] = 1.5    # x
        self.odom_msg.pose.covariance[7] = 1.5    # y
        self.odom_msg.pose.covariance[35] = 0.05  # yaw (IMU preciso)
        
        self.odom_msg.twist.twist.linear.x = vx
        self.odom_msg.twist.twist.linear.y = vy
        self.odom_msg.twist.twist.angular.z = self.imu_data.angular_velocity.z

        self.odom_msg.twist.covariance[0] = 0.5   # vx
        self.odom_msg.twist.covariance[7] = 0.5   # vy
        self.odom_msg.twist.covariance[35] = 0.05 # vyaw

        self.pub_odom.publish(self.odom_msg)


def main(args=None):
    rclpy.init(args=args)
    nodeh = OdomByGPS()
    try:
        rclpy.spin(nodeh)
    except Exception as error:
        print(error)
    except KeyboardInterrupt:
        print("\nNode terminated by user")
    finally:
        nodeh.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
