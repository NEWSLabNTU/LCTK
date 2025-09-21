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

        # Debug counters
        self.image_count = 0
        self.pointcloud_count = 0
        self.caminfo_count = 0
        self.publish_count = 0

        # Parameters
        self.declare_parameter("extrinsic_json5", "")
        self.declare_parameter("use_best_effort_qos", True)
        self.declare_parameter("filter_config_file", "")

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
                self.get_logger().info(f"Extrinsic matrix:\n{self.T_lidar_cam}")

                # Validate extrinsic matrix
                self._validate_extrinsic_matrix()

                # Check if it's an identity matrix (no transformation)
                if np.allclose(self.T_lidar_cam, np.eye(4)):
                    self.get_logger().warn(
                        "WARNING: Extrinsic matrix is identity - no transformation applied!"
                    )
                    self.get_logger().warn(
                        "This means LiDAR and camera are assumed to be at the same position."
                    )
        except FileNotFoundError:
            self.get_logger().error(f"Extrinsic file not found: {extr_path}")
            self.T_lidar_cam = None
            self.extrinsic_error = f"Extrinsic file not found: {extr_path}"
        except Exception as e:
            self.get_logger().error(f"Failed to read extrinsic: {e}")
            self.T_lidar_cam = None
            self.extrinsic_error = f"Failed to parse extrinsic file: {str(e)}"

        # Load filter configuration
        filter_config_path = (
            self.get_parameter("filter_config_file").get_parameter_value().string_value
        )
        self._load_filter_config(filter_config_path)

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
        self.get_logger().info("Subscribed to topics:")
        self.get_logger().info("  - image (will be remapped to camera topic)")
        self.get_logger().info("  - pointcloud (will be remapped to lidar topic)")
        self.get_logger().info("  - camera_info")
        self.get_logger().info("Publishing to: /calibration/pointcloud_overlay")

    def _validate_extrinsic_matrix(self):
        """Validate the extrinsic matrix and check units."""
        try:
            self.get_logger().info("=== EXTRINSIC MATRIX VALIDATION ===")

            R = self.T_lidar_cam[:3, :3]
            t = self.T_lidar_cam[:3, 3]

            # Check rotation matrix properties
            det_R = np.linalg.det(R)
            if abs(det_R - 1.0) > 0.01:
                self.get_logger().warn(
                    f"WARNING: Rotation matrix determinant is {det_R:.6f} (should be 1.0)"
                )

            # Check if R is orthogonal
            should_be_identity = R @ R.T
            identity_error = np.linalg.norm(should_be_identity - np.eye(3))
            if identity_error > 0.01:
                self.get_logger().warn(
                    f"WARNING: Rotation matrix is not orthogonal (error: {identity_error:.6f})"
                )

            # Analyze translation vector
            self.get_logger().info(
                f"Translation vector (LiDAR to Camera): [{t[0]:.6f}, {t[1]:.6f}, {t[2]:.6f}]"
            )

            # Check if translation values are reasonable for meters
            translation_magnitude = np.linalg.norm(t)
            self.get_logger().info(
                f"Translation magnitude: {translation_magnitude:.3f} meters"
            )

            # Check individual components
            if abs(t[0]) > 10.0:  # X translation > 10m
                self.get_logger().warn(
                    f"WARNING: Large X translation ({t[0]:.3f}m) - check units!"
                )
            if abs(t[1]) > 10.0:  # Y translation > 10m
                self.get_logger().warn(
                    f"WARNING: Large Y translation ({t[1]:.3f}m) - check units!"
                )
            if abs(t[2]) > 10.0:  # Z translation > 10m
                self.get_logger().warn(
                    f"WARNING: Large Z translation ({t[2]:.3f}m) - check units!"
                )

            # Check if values look like they might be in wrong units
            if abs(t[0]) > 100.0 or abs(t[1]) > 100.0 or abs(t[2]) > 100.0:
                self.get_logger().warn(
                    "WARNING: Very large translation values detected!"
                )
                self.get_logger().warn(
                    "This might indicate the translation is in centimeters or millimeters instead of meters."
                )
                self.get_logger().warn(
                    "If so, divide translation values by 100 (cm->m) or 1000 (mm->m)"
                )
                self.get_logger().warn(
                    "Please fix the extrinsic calibration file manually - automatic conversion is disabled."
                )

            # Interpret the translation
            self.get_logger().info("Translation interpretation:")
            self.get_logger().info(
                f"  - LiDAR is {abs(t[0]):.3f}m {'behind' if t[0] < 0 else 'in front of'} camera (X)"
            )
            self.get_logger().info(
                f"  - LiDAR is {abs(t[1]):.3f}m {'left' if t[1] < 0 else 'right'} of camera (Y)"
            )
            self.get_logger().info(
                f"  - LiDAR is {abs(t[2]):.3f}m {'below' if t[2] < 0 else 'above'} camera (Z)"
            )

            # Check if the setup makes sense
            if abs(t[0]) < 0.01 and abs(t[1]) < 0.01 and abs(t[2]) < 0.01:
                self.get_logger().warn(
                    "WARNING: Very small translation values - LiDAR and camera might be at same position!"
                )

            self.get_logger().info("=== END EXTRINSIC VALIDATION ===")

        except Exception as e:
            self.get_logger().warn(f"Extrinsic matrix validation failed: {e}")

    def _load_filter_config(self, config_path: str):
        """Load pointcloud filter configuration from JSON5 file."""
        # Default filter configuration
        self.filter_config = {
            "filtering_enabled": True,
            "z_filter": {"enabled": True, "min_distance": 0.1, "max_distance": 50.0},
            "x_filter": {"enabled": True, "max_range": 20.0},
            "y_filter": {"enabled": True, "max_range": 20.0},
            "logging": {
                "log_stats_every_n_frames": 30,
                "enable_filter_breakdown": True,
            },
        }

        if not config_path:
            self.get_logger().info(
                "No filter config file provided, using default filtering parameters"
            )
            return

        try:
            with open(config_path, "r") as f:
                loaded_config = json5.load(f)
                self.filter_config.update(loaded_config)

            self.get_logger().info(f"Loaded filter config from {config_path}")
            self.get_logger().info(
                f"Filtering enabled: {self.filter_config['filtering_enabled']}"
            )

            if self.filter_config["filtering_enabled"]:
                z_cfg = self.filter_config["z_filter"]
                x_cfg = self.filter_config["x_filter"]
                y_cfg = self.filter_config["y_filter"]

                self.get_logger().info(f"Filter ranges:")
                self.get_logger().info(
                    f"  - Z (depth): {z_cfg['min_distance']}m to {z_cfg['max_distance']}m (enabled: {z_cfg['enabled']})"
                )
                self.get_logger().info(
                    f"  - X (horizontal): ±{x_cfg['max_range']}m (enabled: {x_cfg['enabled']})"
                )
                self.get_logger().info(
                    f"  - Y (vertical): ±{y_cfg['max_range']}m (enabled: {y_cfg['enabled']})"
                )
            else:
                self.get_logger().info("Pointcloud filtering is DISABLED")

        except FileNotFoundError:
            self.get_logger().warn(
                f"Filter config file not found: {config_path}, using defaults"
            )
        except Exception as e:
            self.get_logger().error(
                f"Failed to load filter config: {e}, using defaults"
            )

    def _apply_pointcloud_filters(
        self, X_cam: np.ndarray, total_points: int
    ) -> np.ndarray:
        """Apply configurable pointcloud filtering."""
        if not self.filter_config["filtering_enabled"]:
            # Return all points as valid
            return np.ones(X_cam.shape[0], dtype=bool)

        # Initialize mask with all points valid
        valid_mask = np.ones(X_cam.shape[0], dtype=bool)

        z_cfg = self.filter_config["z_filter"]
        x_cfg = self.filter_config["x_filter"]
        y_cfg = self.filter_config["y_filter"]
        log_cfg = self.filter_config["logging"]

        # Z-axis filtering (depth)
        if z_cfg["enabled"]:
            z_mask = (X_cam[:, 2] > z_cfg["min_distance"]) & (
                X_cam[:, 2] < z_cfg["max_distance"]
            )
            valid_mask = valid_mask & z_mask
        else:
            z_mask = np.ones(X_cam.shape[0], dtype=bool)

        # X-axis filtering (horizontal)
        if x_cfg["enabled"]:
            x_mask = np.abs(X_cam[:, 0]) < x_cfg["max_range"]
            valid_mask = valid_mask & x_mask
        else:
            x_mask = np.ones(X_cam.shape[0], dtype=bool)

        # Y-axis filtering (vertical)
        if y_cfg["enabled"]:
            y_mask = np.abs(X_cam[:, 1]) < y_cfg["max_range"]
            valid_mask = valid_mask & y_mask
        else:
            y_mask = np.ones(X_cam.shape[0], dtype=bool)

        # Logging
        points_valid = np.sum(valid_mask)
        if self.publish_count % log_cfg["log_stats_every_n_frames"] == 0:
            self.get_logger().info(
                f"Points in valid range: {points_valid}/{total_points} ({points_valid/total_points*100:.1f}%)"
            )

            if (
                log_cfg["enable_filter_breakdown"]
                and self.filter_config["filtering_enabled"]
            ):
                self.get_logger().info(f"Filter breakdown:")
                if z_cfg["enabled"]:
                    self.get_logger().info(
                        f"  - Z filter ({z_cfg['min_distance']}-{z_cfg['max_distance']}m): {np.sum(z_mask)} points"
                    )
                if x_cfg["enabled"]:
                    self.get_logger().info(
                        f"  - X filter (±{x_cfg['max_range']}m): {np.sum(x_mask)} points"
                    )
                if y_cfg["enabled"]:
                    self.get_logger().info(
                        f"  - Y filter (±{y_cfg['max_range']}m): {np.sum(y_mask)} points"
                    )
                self.get_logger().info(f"  - Combined: {points_valid} points")

        return valid_mask

    def on_caminfo(self, msg: CameraInfo):
        """Handle camera info message."""
        self.caminfo_count += 1
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)

        # Check for potential coordinate system issue
        fx, fy = self.K[0, 0], self.K[1, 1]
        if fx > 1000 or fy > 1000:  # Likely coordinate system issue
            self.get_logger().warn(
                f"WARNING: Large focal length detected (fx={fx:.1f}, fy={fy:.1f})"
            )
            self.get_logger().warn("This suggests a coordinate system or scale issue!")
            self.get_logger().warn(
                "Please check and fix the camera intrinsics manually - automatic scaling is disabled."
            )

        # Distortion may be empty; if so use zeros
        if msg.d:
            self.dist = np.array(msg.d, dtype=np.float64).reshape(-1)
        else:
            self.dist = np.zeros((5,), dtype=np.float64)

        self.get_logger().info(
            f"Camera intrinsics loaded (count: {self.caminfo_count})", once=True
        )
        self.get_logger().info(f"Camera matrix K:\n{self.K}")
        self.get_logger().info(f"Distortion coefficients: {self.dist}")
        self.get_logger().info(f"Image size: {msg.width}x{msg.height}")

    def on_image(self, msg: Image):
        """Handle image message."""
        self.image_count += 1
        self.last_image = msg
        if self.image_count % 30 == 0:  # Log every 30th image to avoid spam
            self.get_logger().info(
                f"Received image {self.image_count}: {msg.width}x{msg.height}, encoding: {msg.encoding}"
            )
        # Always try to publish when we get an image
        self.publish_overlay()

    def on_pointcloud(self, msg: PointCloud2):
        """Handle pointcloud message."""
        self.pointcloud_count += 1
        self.last_pc = msg
        if self.pointcloud_count % 30 == 0:  # Log every 30th pointcloud to avoid spam
            self.get_logger().info(
                f"Received pointcloud {self.pointcloud_count}: {len(msg.data)} bytes, {msg.point_step} step, {len(msg.fields)} fields"
            )
            field_names = [f.name for f in msg.fields]
            self.get_logger().info(f"Pointcloud fields: {field_names}")
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
            self.publish_count += 1

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
                        if self.publish_count % 30 == 0:
                            self.get_logger().info(
                                f"Processing {xyz.shape[0]} points for overlay"
                            )

                        # Use OpenCV projectPoints with extrinsic from LiDAR to camera
                        R = self.T_lidar_cam[:3, :3]
                        t = self.T_lidar_cam[:3, 3]
                        rvec, _ = cv2.Rodrigues(R)
                        tvec = t.reshape(3, 1)

                        # Transform all points to camera frame for analysis
                        X_cam = (R @ xyz.astype(np.float64).T).T + t.reshape(1, 3)

                        # Apply configurable filtering
                        valid_mask = self._apply_pointcloud_filters(X_cam, xyz.shape[0])

                        if not np.any(valid_mask):
                            self.draw_error_on_image(
                                cv_img, "WARNING: All LiDAR points outside valid range"
                            )
                        else:
                            xyz_filtered = xyz[valid_mask]

                            # Project points to image
                            image_points, _ = cv2.projectPoints(
                                xyz_filtered.astype(np.float64),
                                rvec,
                                tvec,
                                self.K,
                                self.dist if self.dist is not None else None,
                            )
                            image_points = image_points.reshape(-1, 2)

                            # Draw projected points on image with improved visibility
                            points_drawn = 0
                            for ui, vi in image_points:
                                if 0 <= ui < w and 0 <= vi < h:
                                    cv2.circle(
                                        cv_img, (int(ui), int(vi)), 2, (0, 255, 0), -1
                                    )  # Green points
                                    cv2.circle(
                                        cv_img, (int(ui), int(vi)), 3, (0, 0, 255), 1
                                    )  # Red border
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

                            # Add debug validation every 100th frame
                            if self.publish_count % 100 == 0:
                                self._test_projection_accuracy(
                                    cv_img, R, t, self.K, self.dist, w, h
                                )
                                self._validate_coordinate_system(R, t, self.K, w, h)

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
            import traceback

            self.get_logger().error(f"Traceback: {traceback.format_exc()}")

    def _test_projection_accuracy(self, cv_img, R, t, K, dist, w, h):
        """Test projection accuracy with known test points."""
        try:
            # Create test points at known distances in front of camera
            test_distances = [1.0, 2.0, 3.0, 5.0]  # meters
            test_points = []

            for dist in test_distances:
                # Test points at different positions relative to camera
                test_points.extend(
                    [
                        [0, 0, dist],  # Center
                        [1, 0, dist],  # Right
                        [-1, 0, dist],  # Left
                        [0, 1, dist],  # Up
                        [0, -1, dist],  # Down
                    ]
                )

            test_points = np.array(test_points, dtype=np.float64)

            # Transform to camera frame
            test_cam = (R @ test_points.T).T + t.reshape(1, 3)

            # Project test points
            rvec, _ = cv2.Rodrigues(R)
            tvec = t.reshape(3, 1)
            test_projected, _ = cv2.projectPoints(
                test_points,
                rvec,
                tvec,
                K,
                dist if dist is not None else None,
            )
            test_projected = test_projected.reshape(-1, 2)

            # Draw test points in blue
            for i, (ui, vi) in enumerate(test_projected):
                if 0 <= ui < w and 0 <= vi < h:
                    cv2.circle(
                        cv_img, (int(ui), int(vi)), 8, (255, 0, 0), -1
                    )  # Blue test points
                    cv2.circle(
                        cv_img, (int(ui), int(vi)), 10, (255, 255, 255), 2
                    )  # White border

            self.get_logger().info(
                f"Projection test: {len(test_points)} test points projected"
            )

        except Exception as e:
            self.get_logger().warn(f"Projection test failed: {e}")

    def _validate_coordinate_system(self, R, t, K, w, h):
        """Validate coordinate system and transformation."""
        try:
            self.get_logger().info("=== COORDINATE SYSTEM VALIDATION ===")

            # Check camera matrix
            fx, fy = K[0, 0], K[1, 1]
            cx, cy = K[0, 2], K[1, 2]
            self.get_logger().info(
                f"Camera intrinsics: fx={fx:.1f}, fy={fy:.1f}, cx={cx:.1f}, cy={cy:.1f}"
            )

            # Check if focal length is reasonable
            if fx > 10000 or fy > 10000:
                self.get_logger().warn(
                    f"WARNING: Very large focal length detected! This might indicate a coordinate system issue."
                )

            # Check principal point
            if cx < 0 or cx > w or cy < 0 or cy > h:
                self.get_logger().warn(
                    f"WARNING: Principal point ({cx:.1f}, {cy:.1f}) is outside image bounds ({w}x{h})"
                )

            # Check rotation matrix properties
            det_R = np.linalg.det(R)
            if abs(det_R - 1.0) > 0.01:
                self.get_logger().warn(
                    f"WARNING: Rotation matrix determinant is {det_R:.6f} (should be 1.0)"
                )

            # Check if R is orthogonal
            should_be_identity = R @ R.T
            identity_error = np.linalg.norm(should_be_identity - np.eye(3))
            if identity_error > 0.01:
                self.get_logger().warn(
                    f"WARNING: Rotation matrix is not orthogonal (error: {identity_error:.6f})"
                )

            # Test transformation with known points
            test_points = np.array(
                [
                    [0, 0, 1],  # 1m in front of LiDAR
                    [1, 0, 1],  # 1m right, 1m forward
                    [0, 1, 1],  # 1m up, 1m forward
                    [0, 0, 5],  # 5m in front
                ],
                dtype=np.float64,
            )

            # Transform to camera frame
            test_cam = (R @ test_points.T).T + t.reshape(1, 3)

            # Project using simple pinhole model
            rvec, _ = cv2.Rodrigues(R)
            tvec = t.reshape(3, 1)
            test_proj, _ = cv2.projectPoints(test_points, rvec, tvec, K, None)
            test_proj = test_proj.reshape(-1, 2)

            self.get_logger().info("Test point projections:")
            for i, (orig, cam, proj) in enumerate(
                zip(test_points, test_cam, test_proj)
            ):
                self.get_logger().info(
                    f"  Point {i}: LiDAR{tuple(orig)} -> Cam{tuple(cam)} -> Proj({proj[0]:.1f},{proj[1]:.1f})"
                )

                # Check if projection is reasonable
                if abs(proj[0]) > w * 2 or abs(proj[1]) > h * 2:
                    self.get_logger().warn(
                        f"    WARNING: Projection is way outside image bounds!"
                    )

            self.get_logger().info("=== END VALIDATION ===")

        except Exception as e:
            self.get_logger().warn(f"Coordinate system validation failed: {e}")


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
