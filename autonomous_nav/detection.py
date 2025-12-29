#!/usr/bin/env python3

#pip uninstall -y numpy
#sudo apt install --reinstall python3-numpy

import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Float64, Bool, Float64MultiArray
from cv_bridge import CvBridge

import numpy as np
import cv2
import pyzed.sl as sl


class ZedArucoPyZedNode(Node):
    def __init__(self):
        super().__init__("zed2i_aruco_pyzed_node")

        # ---------- Parámetros ----------
        self.declare_parameter("fps", 30)
        self.declare_parameter("resolution", "HD720")   # HD720, HD1080, VGA
        self.declare_parameter("depth_mode", "ULTRA")   # ULTRA, QUALITY, PERFORMANCE
        self.declare_parameter("min_depth_m", 0.2)
        self.declare_parameter("max_depth_m", 10.0)
        self.declare_parameter("dictionary", "DICT_4X4_50")
        self.declare_parameter("publish_debug_image", True)

        self.fps = int(self.get_parameter("fps").value)
        self.resolution = str(self.get_parameter("resolution").value)
        self.depth_mode = str(self.get_parameter("depth_mode").value)
        self.min_depth_m = float(self.get_parameter("min_depth_m").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)
        self.dict_name = str(self.get_parameter("dictionary").value)
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)

        # ---------- ROS ----------
        self.bridge = CvBridge()

        self.pub_img = self.create_publisher(Image, "/zed2i/left/image_raw", 10)
        self.pub_debug = self.create_publisher(Image, "/aruco/debug_image", 10)

        # NUEVOS pubs:
        self.pub_distance = self.create_publisher(Float64, "/distance_to_object", 10)
        self.pub_detected = self.create_publisher(Bool, "/detect_aruco", 10)
        self.pub_offset = self.create_publisher(Float64MultiArray, "/object_offset", 10)

        # ---------- ArUco ----------
        self.aruco_dict = self._get_aruco_dict(self.dict_name)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # ---------- ZED ----------
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = self._map_resolution(self.resolution)
        init_params.camera_fps = self.fps
        init_params.depth_mode = self._map_depth_mode(self.depth_mode)
        init_params.coordinate_units = sl.UNIT.METER
        init_params.depth_minimum_distance = self.min_depth_m

        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"No pude abrir la ZED: {repr(err)}")

        self.runtime_params = sl.RuntimeParameters()

        self.mat_left = sl.Mat()
        self.mat_pointcloud = sl.Mat()

        period = 1.0 / max(self.fps, 1)
        self.timer = self.create_timer(period, self.loop)

        self.get_logger().info("ZED2i + pyzed OK. Publicando distancia(Float64), detected(Bool) y offset_norm(Float64MultiArray).")

    def destroy_node(self):
        try:
            self.zed.close()
        except Exception:
            pass
        super().destroy_node()

    # ----------------- Mapeos ZED -----------------
    def _map_resolution(self, s: str):
        s = s.upper()
        if s == "HD1080":
            return sl.RESOLUTION.HD1080
        if s == "VGA":
            return sl.RESOLUTION.VGA
        return sl.RESOLUTION.HD720

    def _map_depth_mode(self, s: str):
        s = s.upper()
        if s == "PERFORMANCE":
            return sl.DEPTH_MODE.PERFORMANCE
        if s == "QUALITY":
            return sl.DEPTH_MODE.QUALITY
        return sl.DEPTH_MODE.ULTRA

    # ----------------- ArUco dict -----------------
    def _get_aruco_dict(self, name: str):
        mapping = {
            "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
            "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
            "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
            "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
            "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
            "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
        }
        if name not in mapping:
            self.get_logger().warn(f"Diccionario '{name}' no soportado. Usando DICT_4X4_50.")
            return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        return cv2.aruco.getPredefinedDictionary(mapping[name])

    # ----------------- Utilidades -----------------
    @staticmethod
    def _slmat_to_bgr(sl_mat: sl.Mat) -> np.ndarray:
        arr = sl_mat.get_data()  # (H,W,4) BGRA
        if arr is None:
            return None
        bgra = np.array(arr, copy=True)
        bgr = cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR)
        return bgr

    @staticmethod
    def _clip_uv(u: int, v: int, w: int, h: int):
        u = max(0, min(w - 1, u))
        v = max(0, min(h - 1, v))
        return u, v

    def _depth_from_pointcloud(self, u: int, v: int) -> float:
        err, point = self.mat_pointcloud.get_value(u, v)
        if err != sl.ERROR_CODE.SUCCESS:
            return float("nan")
        x, y, z = float(point[0]), float(point[1]), float(point[2])
        if not np.isfinite(x) or not np.isfinite(y) or not np.isfinite(z):
            return float("nan")
        return math.sqrt(x * x + y * y + z * z)

    def _robust_depth_center(self, u: int, v: int, w: int, h: int, r: int = 3) -> float:
        vals = []
        for dv in range(-r, r + 1):
            for du in range(-r, r + 1):
                uu, vv = self._clip_uv(u + du, v + dv, w, h)
                d = self._depth_from_pointcloud(uu, vv)
                if np.isfinite(d) and self.min_depth_m <= d <= self.max_depth_m:
                    vals.append(d)
        if not vals:
            return float("nan")
        return float(np.median(vals))

    @staticmethod
    def _offset_norm(cx: float, cy: float, w: int, h: int):
        # Normaliza a [-1, 1] respecto al centro
        ox = (cx - (w / 2.0)) / (w / 2.0)
        oy = (cy - (h / 2.0)) / (h / 2.0)
        # Clamping por seguridad
        ox = max(-1.0, min(1.0, float(ox)))
        oy = max(-1.0, min(1.0, float(oy)))
        return ox, oy

    # ----------------- Loop principal -----------------
    def loop(self):
        if self.zed.grab(self.runtime_params) != sl.ERROR_CODE.SUCCESS:
            return

        self.zed.retrieve_image(self.mat_left, sl.VIEW.LEFT)
        frame_bgr = self._slmat_to_bgr(self.mat_left)
        if frame_bgr is None:
            return

        h, w = frame_bgr.shape[:2]

        # Publicar imagen raw
        stamp = self.get_clock().now().to_msg()
        img_msg = self.bridge.cv2_to_imgmsg(frame_bgr, encoding="bgr8")
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = "zed2i_left_camera"
        self.pub_img.publish(img_msg)

        # Detectar ArUco (primero)
        corners, ids, _ = self.aruco_detector.detectMarkers(frame_bgr)

        # Publicar detected SIEMPRE
        detected_msg = Bool()
        detected_msg.data = (ids is not None and len(ids) > 0)
        self.pub_detected.publish(detected_msg)

        debug = frame_bgr.copy() if self.publish_debug_image else None

        # Si no hay ArUco:
        if not detected_msg.data:
            # Mientras no detecte: offset = (0,0), distancia = NaN
            off = Float64MultiArray()
            off.data = [0.0, 0.0]
            self.pub_offset.publish(off)

            dist_msg = Float64()
            dist_msg.data = float("nan")
            self.pub_distance.publish(dist_msg)

            if self.publish_debug_image:
                dbg_msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
                dbg_msg.header = img_msg.header
                self.pub_debug.publish(dbg_msg)
            return

        # Con ArUco: para distancia necesitamos pointcloud
        self.zed.retrieve_measure(self.mat_pointcloud, sl.MEASURE.XYZRGBA)

        if self.publish_debug_image:
            cv2.aruco.drawDetectedMarkers(debug, corners, ids)

        best_i = None
        best_dist = float("inf")
        best_center = None

        for i in range(len(ids)):
            c = corners[i][0]  # (4,2)
            cx = float(np.mean(c[:, 0]))
            cy = float(np.mean(c[:, 1]))
            u = int(round(cx))
            v = int(round(cy))
            u, v = self._clip_uv(u, v, w, h)

            dist = self._robust_depth_center(u, v, w, h, r=3)

            # escoger el más cercano con distancia válida
            if np.isfinite(dist) and dist < best_dist:
                best_dist = dist
                best_i = i
                best_center = (cx, cy)

            if self.publish_debug_image:
                cv2.circle(debug, (u, v), 4, (0, 255, 0), -1)
                txt = f"id:{int(ids[i])} d:{dist:.2f}m" if np.isfinite(dist) else f"id:{int(ids[i])} d:NaN"
                cv2.putText(debug, txt, (u + 8, v - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Publicar distancia y offset (si hay un marcador con distancia válida)
        dist_msg = Float64()
        off_msg = Float64MultiArray()

        if best_i is not None and best_center is not None:
            cx, cy = best_center
            ox, oy = self._offset_norm(cx, cy, w, h)
            off_msg.data = [ox, oy]
            dist_msg.data = float(best_dist)
        else:
            # Se detectó aruco pero no hubo depth válido (muy cerca, reflejos, etc.)
            off_msg.data = [0.0, 0.0]
            dist_msg.data = float("nan")

        self.pub_offset.publish(off_msg)
        self.pub_distance.publish(dist_msg)

        if self.publish_debug_image:
            dbg_msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
            dbg_msg.header = img_msg.header
            self.pub_debug.publish(dbg_msg)


def main():
    rclpy.init()
    node = ZedArucoPyZedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()