import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int8, Float64
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import time
import cv2
import pyzed.sl as sl
import math


class Detect(Node):
   
	def __init__(self):
		super().__init__("Aruco_node")
		self.publisher_ = self.create_publisher(Image, 'camera/image', 10)
		self.found = self.create_publisher(Bool, "detected_aruco", 1)
		self.state_pub = self.create_publisher(Int8, "state", 1)
		self.create_subscription(Int8, "/state", self.update_state, 1)
		
		# Add publishers for distance and offset
		self.distance_pub = self.create_publisher(Float64, "/aruco_distance", 10)
		self.offset_pub = self.create_publisher(Point, "/aruco_offset", 10)
		
		self.bridge = CvBridge()
		self.twist = Twist()
		
		self.vel_x = 0.33
		self.vel_y = 0
		self.vel_theta = 0.1
		
		self.x = 0
		self.y = 0
		self.distance = None
		self.contador = 0
		self.aruco_dis = False
		self.is_center = False
		self.state = -1

		self.ARUCO_DICT = {
			"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
			"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
			"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
			"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
			"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
			"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
			"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
			"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
			"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
			"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
			"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
			"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
			"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
			"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
			"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
			"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
			"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
			"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
			"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
			"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
			"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
		}
		self.aruco_type = "DICT_4X4_50"
		self.arucoDict = cv2.aruco.Dictionary_get(self.ARUCO_DICT[self.aruco_type])
		self.arucoParams = cv2.aruco.DetectorParameters_create()
		
		self.zed = sl.Camera()

		# Create a InitParameters object and set configuration parameters
		self.init_params = sl.InitParameters()
		self.init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # Use ULTRA depth mode
		self.init_params.coordinate_units = sl.UNIT.MILLIMETER  # Use millimeter units (for depth measurements)

		# Open the camera
		status = self.zed.open(self.init_params)
		if status != sl.ERROR_CODE.SUCCESS:
			print("Camera Open : "+repr(status)+". Exit program.")
			exit()

		# Create and set RuntimeParameters after opening the camera
		self.runtime_parameters = sl.RuntimeParameters()
		self.image = sl.Mat()
		self.depth = sl.Mat()
		self.point_cloud = sl.Mat()
		
		self.timer = self.create_timer(0.1, self.detect)  # Check every 0.1 seconds
		
	def update_state(self, msg):
		self.state = msg.data

	def detect(self):
		if self.state == 4 and self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
			self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
			self.zed.retrieve_measure(self.depth, sl.MEASURE.DEPTH)
			self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA)
		
			grayimg = cv2.cvtColor(self.image.get_data(), cv2.COLOR_BGR2GRAY)
			corners, ids, _ = cv2.aruco.detectMarkers(grayimg, self.arucoDict, parameters=self.arucoParams)
			
			if ids is not None and len(corners) > 0:
				self.aruco_dis = True
				for markerCorner, markerID in zip(corners, ids.flatten()):
					corners = markerCorner.reshape((4, 2))
					(topLeft, _, bottomRight, _) = corners
					
					cX = int((topLeft[0] + bottomRight[0]) / 2.0)
					cY = int((topLeft[1] + bottomRight[1]) / 2.0)
					self.x, self.y = cX, cY  # Marker center

					err, point_cloud_value = self.point_cloud.get_value(cX, cY)
					if math.isfinite(point_cloud_value[2]):
						self.distance = math.sqrt(
							point_cloud_value[0]**2 + point_cloud_value[1]**2 + point_cloud_value[2]**2)
						offset = Point()  # Offset from image center
						offset.x = cX - (self.image.get_width() / 2)
						offset.y = cY - (self.image.get_height() / 2)
						offset.z = 0

						self.distance_pub.publish(Float64(data=self.distance))  # Publish distance
						self.offset_pub.publish(offset)  # Publish offset
			else:
				self.aruco_dis = False
				self.distance = None

def main(args=None):
	rclpy.init(args=args)
	detect = Detect()
	rclpy.spin(detect)
	detect.destroy_node()
	rclpy.shutdown()
	
if __name__ == "__main__":
	main()
