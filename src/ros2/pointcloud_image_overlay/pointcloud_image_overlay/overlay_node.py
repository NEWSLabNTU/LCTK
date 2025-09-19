#!/usr/bin/env python3
"""
Simple pointcloud to image overlay node.
Projects LiDAR points onto camera image using extrinsic calibration.
"""

import json5
import math
from typing import Optional
import struct

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from cv_bridge import CvBridge


def read_extrinsic_4x4(path: str) -> np.ndarray:
    """Read extrinsic matrix from JSON5 file."""
    with open(path, "r") as f:
        data = json5.load(f)
    mat = np.asarray(data["matrix"], dtype=np.float64)
    if mat.shape == (4, 4):
        return mat
    raise ValueError('extrinsic JSON5 must contain key "matrix" as 4x4 array')


def pointcloud2_to_xyz(pc2: PointCloud2) -> np.ndarray:
    """Convert PointCloud2 message to xyz numpy array."""
    if pc2.point_step == 0 or len(pc2.data) == 0:
        return np.zeros((0, 3), dtype=np.float32)

    # Find field offsets
    offset = {f.name: f.offset for f in pc2.fields}
    if "x" not in offset or "y" not in offset or "z" not in offset:
        return np.zeros((0, 3), dtype=np.float32)

    step = pc2.point_step
    xyz = []

    # Extract xyz coordinates
    for i in range(0, len(pc2.data), step):
        x = struct.unpack_from("f", pc2.data, i + offset["x"])[0]
        y = struct.unpack_from("f", pc2.data, i + offset["y"])[0]
        z = struct.unpack_from("f", pc2.data, i + offset["z"])[0]
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            xyz.append((x, y, z))

    if not xyz:
        return np.zeros((0, 3), dtype=np.float32)
    return np.asarray(xyz, dtype=np.float32)


class OverlayNode(Node):
    def __init__(self):
        super().__init__("pointcloud_image_overlay")
        self.bridge = CvBridge()

        # Parameters
        self.declare_parameter("extrinsic_json5", "")
        self.declare_parameter("use_best_effort_qos", True)

        # Load extrinsic calibration
        extr_path = (
            self.get_parameter("extrinsic_json5").get_parameter_value().string_value
        )
        try:
            self.T_lidar_cam = read_extrinsic_4x4(extr_path) if extr_path else None
            if self.T_lidar_cam is not None:
                self.get_logger().info(f"Loaded extrinsic from {extr_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to read extrinsic: {e}")
            self.T_lidar_cam = None

        # State
        self.K: Optional[np.ndarray] = None
        self.dist: Optional[np.ndarray] = None
        self.last_image: Optional[Image] = None
        self.last_pc: Optional[PointCloud2] = None

        # Configure QoS
        use_best_effort = (
            self.get_parameter("use_best_effort_qos").get_parameter_value().bool_value
        )
        if use_best_effort:
            # Best effort for live sensors
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                depth=10,
            )
        else:
            # Reliable for rosbag playback
            qos = QoSProfile(depth=10)

        # Subscriptions (using base topic names that will be remapped)
        self.sub_img = self.create_subscription(Image, "image", self.on_image, qos)
        self.sub_pc = self.create_subscription(
            PointCloud2, "pointcloud", self.on_pointcloud, qos
        )

        # Camera info topic - derive from image topic namespace
        self.sub_info = self.create_subscription(
            CameraInfo, "camera_info", self.on_caminfo, qos
        )

        # Publisher
        self.pub = self.create_publisher(Image, "/calibration/pointcloud_overlay", 10)

        self.get_logger().info("Pointcloud image overlay node started")
        self.get_logger().info(
            f"QoS: {'Best Effort' if use_best_effort else 'Reliable'}"
        )

    def on_caminfo(self, msg: CameraInfo):
        """Handle camera info message."""
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        # Distortion may be empty; if so use zeros
        if msg.d:
            self.dist = np.array(msg.d, dtype=np.float64).reshape(-1)
        else:
            self.dist = np.zeros((5,), dtype=np.float64)
        self.get_logger().info("Camera intrinsics loaded", once=True)

    def on_image(self, msg: Image):
        """Handle image message."""
        self.last_image = msg
        self.try_publish()

    def on_pointcloud(self, msg: PointCloud2):
        """Handle pointcloud message."""
        self.last_pc = msg
        self.try_publish()

    def try_publish(self):
        """Try to publish overlay if all data is available."""
        if (
            self.last_image is None
            or self.last_pc is None
            or self.K is None
            or self.T_lidar_cam is None
        ):
            return

        try:
            # Convert image
            cv_img = self.bridge.imgmsg_to_cv2(self.last_image, desired_encoding="bgr8")
            h, w = cv_img.shape[:2]

            # Extract points from pointcloud
            xyz = pointcloud2_to_xyz(self.last_pc)
            if xyz.shape[0] == 0:
                # No points, publish original image
                out = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
                out.header = self.last_image.header
                self.pub.publish(out)
                return

            # Use OpenCV projectPoints with extrinsic from LiDAR to camera
            R = self.T_lidar_cam[:3, :3]
            t = self.T_lidar_cam[:3, 3]
            rvec, _ = cv2.Rodrigues(R)
            tvec = t.reshape(3, 1)

            # Filter points behind camera
            X_cam = (R @ xyz.astype(np.float64).T).T + t.reshape(1, 3)
            positive_z_mask = X_cam[:, 2] > 0.1  # Keep points at least 10cm in front
            if not np.any(positive_z_mask):
                # All points behind camera
                out = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
                out.header = self.last_image.header
                self.pub.publish(out)
                return

            xyz_filtered = xyz[positive_z_mask]

            # Project points to image
            image_points, _ = cv2.projectPoints(
                xyz_filtered.astype(np.float64),
                rvec,
                tvec,
                self.K,
                self.dist if self.dist is not None else None,
            )
            image_points = image_points.reshape(-1, 2)

            # Draw projected points on image
            for ui, vi in image_points:
                if 0 <= ui < w and 0 <= vi < h:
                    cv2.circle(cv_img, (int(ui), int(vi)), 1, (0, 0, 255), -1)

            # Publish overlay image
            out = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
            out.header = self.last_image.header
            self.pub.publish(out)

        except Exception as e:
            self.get_logger().error(
                f"Error creating overlay: {e}", throttle_duration_sec=1.0
            )


def main():
    rclpy.init()
    node = OverlayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
