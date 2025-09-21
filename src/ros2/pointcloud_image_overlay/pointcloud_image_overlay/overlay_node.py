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
        self.extrinsic_error = None
        try:
            if not extr_path:
                self.T_lidar_cam = None
                self.extrinsic_error = "No extrinsic file path provided"
            else:
                self.T_lidar_cam = read_extrinsic_4x4(extr_path)
                self.get_logger().info(f"Loaded extrinsic from {extr_path}")
        except FileNotFoundError:
            self.get_logger().error(f"Extrinsic file not found: {extr_path}")
            self.T_lidar_cam = None
            self.extrinsic_error = f"Extrinsic file not found: {extr_path}"
        except Exception as e:
            self.get_logger().error(f"Failed to read extrinsic: {e}")
            self.T_lidar_cam = None
            self.extrinsic_error = f"Failed to parse extrinsic file: {str(e)}"

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

        # Derive camera_info topic from resolved image topic (following image_pipeline convention)
        resolved_image_topic = self.sub_img.topic_name
        if "/" in resolved_image_topic:
            # Find the last slash and replace the last component with "camera_info"
            base_path = resolved_image_topic.rsplit("/", 1)[0]
            camera_info_topic = f"{base_path}/camera_info"
        else:
            camera_info_topic = "camera_info"

        self.get_logger().info(f"Derived camera_info topic: {camera_info_topic}")

        # Subscribe to derived camera_info topic
        self.sub_info = self.create_subscription(
            CameraInfo, camera_info_topic, self.on_caminfo, qos
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
        # Always try to publish when we get an image
        self.publish_overlay()

    def on_pointcloud(self, msg: PointCloud2):
        """Handle pointcloud message."""
        self.last_pc = msg
        # Only publish if we have a recent image
        if self.last_image is not None:
            self.publish_overlay()

    def draw_error_on_image(self, cv_img: np.ndarray, error_text: str):
        """Draw error text on the image."""
        h, w = cv_img.shape[:2]

        # Choose font and scale based on image size
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = min(w, h) / 800.0  # Scale font based on image size
        thickness = max(1, int(font_scale * 2))

        # Split long text into multiple lines
        max_chars_per_line = max(20, w // 25)
        lines = []
        words = error_text.split()
        current_line = ""

        for word in words:
            if len(current_line + " " + word) <= max_chars_per_line:
                current_line = current_line + " " + word if current_line else word
            else:
                if current_line:
                    lines.append(current_line)
                current_line = word
        if current_line:
            lines.append(current_line)

        # Draw semi-transparent background
        overlay = cv_img.copy()
        line_height = int(30 * font_scale)
        text_height = len(lines) * line_height + 20
        cv2.rectangle(overlay, (10, 10), (w - 10, 10 + text_height), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, cv_img, 0.3, 0, cv_img)

        # Draw text lines
        for i, line in enumerate(lines):
            y_pos = 30 + i * line_height
            cv2.putText(
                cv_img, line, (20, y_pos), font, font_scale, (0, 0, 255), thickness
            )

    def publish_overlay(self):
        """Always publish an overlay image when image data is available."""
        if self.last_image is None:
            return

        try:
            # Convert image
            cv_img = self.bridge.imgmsg_to_cv2(self.last_image, desired_encoding="bgr8")
            h, w = cv_img.shape[:2]

            # Check for extrinsic parameter errors
            if self.extrinsic_error is not None:
                self.draw_error_on_image(cv_img, f"ERROR: {self.extrinsic_error}")
            elif self.T_lidar_cam is None:
                self.draw_error_on_image(
                    cv_img, "ERROR: No extrinsic calibration loaded"
                )
            elif self.K is None:
                self.draw_error_on_image(
                    cv_img, "ERROR: No camera intrinsics available"
                )
            elif self.last_pc is None:
                self.draw_error_on_image(
                    cv_img, "WARNING: No point cloud data available"
                )
            else:
                # All data available, try to create overlay
                try:
                    # Extract points from pointcloud
                    xyz = pointcloud2_to_xyz(self.last_pc)
                    if xyz.shape[0] == 0:
                        self.draw_error_on_image(
                            cv_img, "WARNING: Point cloud is empty"
                        )
                    else:
                        # Use OpenCV projectPoints with extrinsic from LiDAR to camera
                        R = self.T_lidar_cam[:3, :3]
                        t = self.T_lidar_cam[:3, 3]
                        rvec, _ = cv2.Rodrigues(R)
                        tvec = t.reshape(3, 1)

                        # Filter points behind camera
                        X_cam = (R @ xyz.astype(np.float64).T).T + t.reshape(1, 3)
                        positive_z_mask = (
                            X_cam[:, 2] > 0.1
                        )  # Keep points at least 10cm in front
                        if not np.any(positive_z_mask):
                            self.draw_error_on_image(
                                cv_img, "WARNING: All LiDAR points behind camera"
                            )
                        else:
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
                            points_drawn = 0
                            for ui, vi in image_points:
                                if 0 <= ui < w and 0 <= vi < h:
                                    cv2.circle(
                                        cv_img, (int(ui), int(vi)), 1, (0, 255, 0), -1
                                    )
                                    points_drawn += 1

                            # Show status overlay
                            status_text = (
                                f"Points: {points_drawn}/{len(image_points)} visible"
                            )
                            cv2.putText(
                                cv_img,
                                status_text,
                                (10, h - 20),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.5,
                                (0, 255, 0),
                                1,
                            )

                except Exception as overlay_error:
                    self.draw_error_on_image(
                        cv_img, f"ERROR in overlay: {str(overlay_error)}"
                    )

            # Always publish the image (with or without overlay)
            out = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
            out.header = self.last_image.header
            self.pub.publish(out)

        except Exception as e:
            self.get_logger().error(
                f"Error publishing overlay: {e}", throttle_duration_sec=1.0
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
