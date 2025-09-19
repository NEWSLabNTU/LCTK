import json5
import math
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from cv_bridge import CvBridge


def read_extrinsic_4x4(path: str) -> np.ndarray:
    with open(path, 'r') as f:
        data = json5.load(f)
    mat = np.asarray(data['matrix'], dtype=np.float64)
    if mat.shape == (4, 4):
        return mat
    raise ValueError('extrinsic JSON5 must contain key "matrix" as 4x4 array')


def pointcloud2_to_xyz(pc2: PointCloud2) -> np.ndarray:
    import struct
    if pc2.point_step == 0 or len(pc2.data) == 0:
        return np.zeros((0, 3), dtype=np.float32)
    offset = {f.name: f.offset for f in pc2.fields}
    step = pc2.point_step
    xyz = []
    for i in range(0, len(pc2.data), step):
        x = struct.unpack_from('f', pc2.data, i + offset['x'])[0]
        y = struct.unpack_from('f', pc2.data, i + offset['y'])[0]
        z = struct.unpack_from('f', pc2.data, i + offset['z'])[0]
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            xyz.append((x, y, z))
    if not xyz:
        return np.zeros((0, 3), dtype=np.float32)
    return np.asarray(xyz, dtype=np.float32)


class OverlayNode(Node):
    def __init__(self):
        super().__init__('pointcloud_image_overlay')
        self.bridge = CvBridge()

        # Parameters
        self.declare_parameter('extrinsic_json5', '')
        self.declare_parameter('image_topic', '/sensing/camera/front_center/synchronized_image')
        self.declare_parameter('pointcloud_topic', '/sensing/lidar/top/synchronized_pointcloud')
        self.declare_parameter('camera_info_topic', '/sensing/camera/front_center/camera_info')

        extr_path = self.get_parameter('extrinsic_json5').get_parameter_value().string_value
        try:
            self.T_lidar_cam = read_extrinsic_4x4(extr_path) if extr_path else None
            if self.T_lidar_cam is not None:
                self.get_logger().info(f'Loaded extrinsic from {extr_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to read extrinsic: {e}')
            self.T_lidar_cam = None

        # State
        self.K: Optional[np.ndarray] = None
        self.dist: Optional[np.ndarray] = None
        self.last_image: Optional[Image] = None
        self.last_pc: Optional[PointCloud2] = None

        # IO
        img_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        pc_topic = self.get_parameter('pointcloud_topic').get_parameter_value().string_value
        info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value

        self.sub_img = self.create_subscription(Image, img_topic, self.on_image, 10)
        self.sub_pc = self.create_subscription(PointCloud2, pc_topic, self.on_pointcloud, 10)
        self.sub_info = self.create_subscription(CameraInfo, info_topic, self.on_caminfo, 10)
        self.pub = self.create_publisher(Image, '/calibration/pointcloud_overlay', 10)

    def on_caminfo(self, msg: CameraInfo):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        # Distortion may be empty; if so use zeros
        if msg.d:
            self.dist = np.array(msg.d, dtype=np.float64).reshape(-1)
        else:
            self.dist = np.zeros((5,), dtype=np.float64)
        self.get_logger().info('Camera intrinsics/distortion loaded')

    def on_image(self, msg: Image):
        self.last_image = msg
        self.try_publish()

    def on_pointcloud(self, msg: PointCloud2):
        self.last_pc = msg
        self.try_publish()

    def try_publish(self):
        if self.last_image is None or self.last_pc is None or self.K is None or self.T_lidar_cam is None:
            return
        cv_img = self.bridge.imgmsg_to_cv2(self.last_image, desired_encoding='bgr8')
        h, w = cv_img.shape[:2]
        xyz = pointcloud2_to_xyz(self.last_pc)
        if xyz.shape[0] == 0:
            self.pub.publish(self.bridge.cv2_to_imgmsg(cv_img, encoding='bgr8'))
            return

        # Use OpenCV projectPoints with extrinsic from LiDAR to camera
        R = self.T_lidar_cam[:3, :3]
        t = self.T_lidar_cam[:3, 3]
        rvec, _ = cv2.Rodrigues(R)
        tvec = t.reshape(3, 1)

        # Filter out points behind the camera using Z after transform
        X_cam = (R @ xyz.astype(np.float64).T).T + t.reshape(1, 3)
        positive_z_mask = X_cam[:, 2] > 1e-6
        if not np.any(positive_z_mask):
            self.pub.publish(self.bridge.cv2_to_imgmsg(cv_img, encoding='bgr8'))
            return
        xyz = xyz[positive_z_mask]

        image_points, _ = cv2.projectPoints(
            xyz.astype(np.float64), rvec, tvec, self.K, self.dist if self.dist is not None else None
        )
        image_points = image_points.reshape(-1, 2)

        # Draw points
        for ui, vi in image_points:
            if 0 <= ui < w and 0 <= vi < h:
                cv2.circle(cv_img, (int(ui), int(vi)), 1, (0, 0, 255), -1)

        out = self.bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')
        out.header = self.last_image.header
        self.pub.publish(out)


def main():
    rclpy.init()
    node = OverlayNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

import json5
import math
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from cv_bridge import CvBridge


def read_extrinsic_4x4(path: str) -> np.ndarray:
    with open(path, 'r') as f:
        data = json5.load(f)
    mat = np.asarray(data['matrix'], dtype=np.float64)
    if mat.shape == (4, 4):
        return mat
    raise ValueError('extrinsic JSON5 must contain key "matrix" as 4x4 array')


def pointcloud2_to_xyz(pc2: PointCloud2) -> np.ndarray:
    import struct
    if pc2.point_step == 0 or len(pc2.data) == 0:
        return np.zeros((0, 3), dtype=np.float32)
    offset = {f.name: f.offset for f in pc2.fields}
    step = pc2.point_step
    xyz = []
    for i in range(0, len(pc2.data), step):
        x = struct.unpack_from('f', pc2.data, i + offset['x'])[0]
        y = struct.unpack_from('f', pc2.data, i + offset['y'])[0]
        z = struct.unpack_from('f', pc2.data, i + offset['z'])[0]
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            xyz.append((x, y, z))
    if not xyz:
        return np.zeros((0, 3), dtype=np.float32)
    return np.asarray(xyz, dtype=np.float32)


class OverlayNode(Node):
    def __init__(self):
        super().__init__('pointcloud_image_overlay')
        self.bridge = CvBridge()

        # Parameters
        self.declare_parameter('extrinsic_json5', '')
        self.declare_parameter('image_topic', '/sensing/camera/front_center/synchronized_image')
        self.declare_parameter('pointcloud_topic', '/sensing/lidar/top/synchronized_pointcloud')
        self.declare_parameter('camera_info_topic', '/sensing/camera/front_center/camera_info')

        extr_path = self.get_parameter('extrinsic_json5').get_parameter_value().string_value
        try:
            self.T_lidar_cam = read_extrinsic_4x4(extr_path) if extr_path else None
            if self.T_lidar_cam is not None:
                self.get_logger().info(f'Loaded extrinsic from {extr_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to read extrinsic: {e}')
            self.T_lidar_cam = None

        # State
        self.K: Optional[np.ndarray] = None
        self.last_image: Optional[Image] = None
        self.last_pc: Optional[PointCloud2] = None

        # IO
        img_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        pc_topic = self.get_parameter('pointcloud_topic').get_parameter_value().string_value
        info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value

        self.sub_img = self.create_subscription(Image, img_topic, self.on_image, 10)
        self.sub_pc = self.create_subscription(PointCloud2, pc_topic, self.on_pointcloud, 10)
        self.sub_info = self.create_subscription(CameraInfo, info_topic, self.on_caminfo, 10)
        self.pub = self.create_publisher(Image, '/calibration/pointcloud_overlay', 10)

    def on_caminfo(self, msg: CameraInfo):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.get_logger().info('Camera intrinsics loaded')

    def on_image(self, msg: Image):
        self.last_image = msg
        self.try_publish()

    def on_pointcloud(self, msg: PointCloud2):
        self.last_pc = msg
        self.try_publish()

    def try_publish(self):
        if self.last_image is None or self.last_pc is None or self.K is None or self.T_lidar_cam is None:
            return
        cv_img = self.bridge.imgmsg_to_cv2(self.last_image, desired_encoding='bgr8')
        h, w = cv_img.shape[:2]
        xyz = pointcloud2_to_xyz(self.last_pc)
        if xyz.shape[0] == 0:
            self.pub.publish(self.bridge.cv2_to_imgmsg(cv_img, encoding='bgr8'))
            return

        # Transform LiDAR to camera frame: X_cam = T * X_lidar
        ones = np.ones((xyz.shape[0], 1), dtype=np.float64)
        xyz_h = np.hstack([xyz.astype(np.float64), ones])
        X_cam = (self.T_lidar_cam @ xyz_h.T).T[:, :3]
        z = X_cam[:, 2]
        valid = z > 1e-6
        X_cam = X_cam[valid]
        if X_cam.shape[0] == 0:
            self.pub.publish(self.bridge.cv2_to_imgmsg(cv_img, encoding='bgr8'))
            return
        uvw = (self.K @ X_cam.T).T
        u = uvw[:, 0] / uvw[:, 2]
        v = uvw[:, 1] / uvw[:, 2]

        # Draw points
        for ui, vi in zip(u, v):
            if 0 <= ui < w and 0 <= vi < h:
                cv2.circle(cv_img, (int(ui), int(vi)), 1, (0, 0, 255), -1)

        out = self.bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')
        out.header = self.last_image.header
        self.pub.publish(out)


def main():
    rclpy.init()
    node = OverlayNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


