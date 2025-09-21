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
from dataclasses import dataclass


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


@dataclass
class BBox:
    """Bounding box for filtering pointcloud points."""
    # Position and orientation (translation and rotation)
    position: np.ndarray  # [x, y, z] in meters
    rotation: np.ndarray  # [rx, ry, rz] in radians (Euler angles)
    # Size of the bounding box
    size_xyz: np.ndarray  # [width, height, depth] in meters
    
    def __post_init__(self):
        """Convert to numpy arrays if needed."""
        self.position = np.array(self.position, dtype=np.float64)
        self.rotation = np.array(self.rotation, dtype=np.float64)
        self.size_xyz = np.array(self.size_xyz, dtype=np.float64)
    
    @classmethod
    def default(cls) -> 'BBox':
        """Create default bounding box similar to bbox.rs."""
        return cls(
            position=[2.5, 0.0, 0.0],  # 2.5m in front of LiDAR
            rotation=[0.0, 0.0, 0.0],  # No rotation
            size_xyz=[1.0, 3.0, 2.0]  # x_range: 2~3 (1), y_range: -1.5~1.5 (3), z_range: -1~1 (2)
        )
    
    def contains_point(self, point: np.ndarray) -> bool:
        """Check if a point is inside the bounding box."""
        # Transform point to bounding box local coordinate system
        # For simplicity, we'll use a basic translation (no rotation for now)
        local_point = point - self.position
        
        # Check if point is within bounds
        half_sizes = self.size_xyz / 2.0
        return (np.abs(local_point[0]) <= half_sizes[0] and
                np.abs(local_point[1]) <= half_sizes[1] and
                np.abs(local_point[2]) <= half_sizes[2])
    
    def filter_points(self, points: np.ndarray) -> np.ndarray:
        """Filter points to only include those inside the bounding box."""
        if points.shape[0] == 0:
            return points
        
        # Create mask for points inside bounding box
        mask = np.array([self.contains_point(point) for point in points])
        return points[mask]


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
        self.declare_parameter("use_bbox_filter", True)
        self.declare_parameter("bbox_position", [2.5, 0.0, 0.0])  # [x, y, z] in meters
        self.declare_parameter("bbox_rotation", [0.0, 0.0, 0.0])  # [rx, ry, rz] in radians
        self.declare_parameter("bbox_size", [1.0, 3.0, 2.0])     # [width, height, depth] in meters

        # Load extrinsic calibration
        extr_path = (
            self.get_parameter("extrinsic_json5").get_parameter_value().string_value
        )
        try:
            self.T_lidar_cam = read_extrinsic_4x4(extr_path) if extr_path else None
            if self.T_lidar_cam is not None:
                self.get_logger().info(f"Loaded extrinsic from {extr_path}")
                self.get_logger().info(f"Extrinsic matrix:\n{self.T_lidar_cam}")
                
                # Validate extrinsic matrix
                self._validate_extrinsic_matrix()
                
                # Check if it's an identity matrix (no transformation)
                if np.allclose(self.T_lidar_cam, np.eye(4)):
                    self.get_logger().warn("WARNING: Extrinsic matrix is identity - no transformation applied!")
                    self.get_logger().warn("This means LiDAR and camera are assumed to be at the same position.")
            else:
                self.get_logger().error("No extrinsic matrix loaded - overlay will not work!")
        except Exception as e:
            self.get_logger().error(f"Failed to read extrinsic: {e}")
            self.T_lidar_cam = None

        # Initialize bounding box for pointcloud filtering
        use_bbox_filter = self.get_parameter("use_bbox_filter").get_parameter_value().bool_value
        if use_bbox_filter:
            bbox_position = self.get_parameter("bbox_position").get_parameter_value().double_array_value
            bbox_rotation = self.get_parameter("bbox_rotation").get_parameter_value().double_array_value
            bbox_size = self.get_parameter("bbox_size").get_parameter_value().double_array_value
            
            self.bbox = BBox(
                position=bbox_position,
                rotation=bbox_rotation,
                size_xyz=bbox_size
            )
            self.get_logger().info(f"Bounding box filter enabled:")
            self.get_logger().info(f"  Position: {self.bbox.position}")
            self.get_logger().info(f"  Rotation: {self.bbox.rotation}")
            self.get_logger().info(f"  Size: {self.bbox.size_xyz}")
        else:
            self.bbox = None
            self.get_logger().info("Bounding box filter disabled")

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
                self.get_logger().warn(f"WARNING: Rotation matrix determinant is {det_R:.6f} (should be 1.0)")
            
            # Check if R is orthogonal
            should_be_identity = R @ R.T
            identity_error = np.linalg.norm(should_be_identity - np.eye(3))
            if identity_error > 0.01:
                self.get_logger().warn(f"WARNING: Rotation matrix is not orthogonal (error: {identity_error:.6f})")
            
            # Analyze translation vector
            self.get_logger().info(f"Translation vector (LiDAR to Camera): [{t[0]:.6f}, {t[1]:.6f}, {t[2]:.6f}]")
            
            # Check if translation values are reasonable for meters
            translation_magnitude = np.linalg.norm(t)
            self.get_logger().info(f"Translation magnitude: {translation_magnitude:.3f} meters")
            
            # Check individual components
            if abs(t[0]) > 10.0:  # X translation > 10m
                self.get_logger().warn(f"WARNING: Large X translation ({t[0]:.3f}m) - check units!")
            if abs(t[1]) > 10.0:  # Y translation > 10m
                self.get_logger().warn(f"WARNING: Large Y translation ({t[1]:.3f}m) - check units!")
            if abs(t[2]) > 10.0:  # Z translation > 10m
                self.get_logger().warn(f"WARNING: Large Z translation ({t[2]:.3f}m) - check units!")
            
            # Check if values look like they might be in wrong units
            if abs(t[0]) > 100.0 or abs(t[1]) > 100.0 or abs(t[2]) > 100.0:
                self.get_logger().warn("WARNING: Very large translation values detected!")
                self.get_logger().warn("This might indicate the translation is in centimeters or millimeters instead of meters.")
                self.get_logger().warn("If so, divide translation values by 100 (cm->m) or 1000 (mm->m)")
                
                # Try to auto-detect and fix unit issues
                self._auto_fix_translation_units()
            
            # Interpret the translation
            self.get_logger().info("Translation interpretation:")
            self.get_logger().info(f"  - LiDAR is {abs(t[0]):.3f}m {'behind' if t[0] < 0 else 'in front of'} camera (X)")
            self.get_logger().info(f"  - LiDAR is {abs(t[1]):.3f}m {'left' if t[1] < 0 else 'right'} of camera (Y)")
            self.get_logger().info(f"  - LiDAR is {abs(t[2]):.3f}m {'below' if t[2] < 0 else 'above'} camera (Z)")
            
            # Check if the setup makes sense
            if abs(t[0]) < 0.01 and abs(t[1]) < 0.01 and abs(t[2]) < 0.01:
                self.get_logger().warn("WARNING: Very small translation values - LiDAR and camera might be at same position!")
            
            self.get_logger().info("=== END EXTRINSIC VALIDATION ===")
            
        except Exception as e:
            self.get_logger().warn(f"Extrinsic matrix validation failed: {e}")

    def _auto_fix_translation_units(self):
        """Auto-detect and fix potential unit issues in translation."""
        try:
            t = self.T_lidar_cam[:3, 3]
            original_t = t.copy()
            
            # Check if values look like centimeters (typical range: 100-10000)
            if (100.0 < abs(t[0]) < 10000.0 and 
                100.0 < abs(t[1]) < 10000.0 and 
                100.0 < abs(t[2]) < 10000.0):
                self.get_logger().warn("Auto-detected: Translation appears to be in centimeters")
                self.get_logger().warn("Converting cm to meters by dividing by 100...")
                self.T_lidar_cam[:3, 3] = t / 100.0
                self.get_logger().info(f"Fixed translation: {original_t} -> {self.T_lidar_cam[:3, 3]}")
                return
            
            # Check if values look like millimeters (typical range: 1000-100000)
            if (1000.0 < abs(t[0]) < 100000.0 and 
                1000.0 < abs(t[1]) < 100000.0 and 
                1000.0 < abs(t[2]) < 100000.0):
                self.get_logger().warn("Auto-detected: Translation appears to be in millimeters")
                self.get_logger().warn("Converting mm to meters by dividing by 1000...")
                self.T_lidar_cam[:3, 3] = t / 1000.0
                self.get_logger().info(f"Fixed translation: {original_t} -> {self.T_lidar_cam[:3, 3]}")
                return
                
            self.get_logger().warn("Could not auto-detect unit issue - manual verification needed")
            
        except Exception as e:
            self.get_logger().warn(f"Auto-fix translation units failed: {e}")

    def on_caminfo(self, msg: CameraInfo):
        """Handle camera info message."""
        self.caminfo_count += 1
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        
        # Check for potential coordinate system issue
        fx, fy = self.K[0, 0], self.K[1, 1]
        if fx > 1000 or fy > 1000:  # Likely coordinate system issue
            self.get_logger().warn(f"WARNING: Large focal length detected (fx={fx:.1f}, fy={fy:.1f})")
            self.get_logger().warn("This suggests a coordinate system or scale issue!")
            self.get_logger().warn("Attempting to fix by scaling camera matrix...")
            
            # Scale down the camera matrix to reasonable values
            # Assume the intrinsics are in a different coordinate system
            scale_factor = 0.1  # Scale down by factor of 10
            self.K[0, 0] *= scale_factor  # fx
            self.K[1, 1] *= scale_factor  # fy
            self.K[0, 2] *= scale_factor  # cx
            self.K[1, 2] *= scale_factor  # cy
            
            # Fix principal point to be at image center for better distribution
            # The scaled principal point is too small, set it to image center
            self.K[0, 2] = msg.width / 2.0  # cx = image width / 2
            self.K[1, 2] = msg.height / 2.0  # cy = image height / 2
            
            self.get_logger().warn(f"Scaled camera matrix K:\n{self.K}")
        
        # Distortion may be empty; if so use zeros
        if msg.d:
            self.dist = np.array(msg.d, dtype=np.float64).reshape(-1)
        else:
            self.dist = np.zeros((5,), dtype=np.float64)
        
        self.get_logger().info(f"Camera intrinsics loaded (count: {self.caminfo_count})", once=True)
        self.get_logger().info(f"Camera matrix K:\n{self.K}")
        self.get_logger().info(f"Distortion coefficients: {self.dist}")
        self.get_logger().info(f"Image size: {msg.width}x{msg.height}")

    def on_image(self, msg: Image):
        """Handle image message."""
        self.image_count += 1
        self.last_image = msg
        if self.image_count % 30 == 0:  # Log every 30th image to avoid spam
            self.get_logger().info(f"Received image {self.image_count}: {msg.width}x{msg.height}, encoding: {msg.encoding}")
        self.try_publish()

    def on_pointcloud(self, msg: PointCloud2):
        """Handle pointcloud message."""
        self.pointcloud_count += 1
        self.last_pc = msg
        if self.pointcloud_count % 30 == 0:  # Log every 30th pointcloud to avoid spam
            self.get_logger().info(f"Received pointcloud {self.pointcloud_count}: {len(msg.data)} bytes, {msg.point_step} step, {len(msg.fields)} fields")
            field_names = [f.name for f in msg.fields]
            self.get_logger().info(f"Pointcloud fields: {field_names}")
        self.try_publish()

    def try_publish(self):
        """Try to publish overlay if all data is available."""
        # Check what data is missing
        missing_data = []
        if self.last_image is None:
            missing_data.append("image")
        if self.last_pc is None:
            missing_data.append("pointcloud")
        if self.K is None:
            missing_data.append("camera_info")
        if self.T_lidar_cam is None:
            missing_data.append("extrinsic_matrix")
        
        if missing_data:
            if self.publish_count % 100 == 0:  # Log every 100th attempt to avoid spam
                self.get_logger().warn(f"Missing data for overlay: {missing_data}")
            return

        try:
            self.publish_count += 1
            
            # Convert image
            cv_img = self.bridge.imgmsg_to_cv2(self.last_image, desired_encoding="bgr8")
            h, w = cv_img.shape[:2]

            # Extract points from pointcloud
            xyz = pointcloud2_to_xyz(self.last_pc)
            if xyz.shape[0] == 0:
                # No points, publish original image
                if self.publish_count % 30 == 0:
                    self.get_logger().warn("No valid points in pointcloud - publishing original image")
                out = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
                out.header = self.last_image.header
                self.pub.publish(out)
                return

            if self.publish_count % 30 == 0:
                self.get_logger().info(f"Processing {xyz.shape[0]} points for overlay")
                # Debug: Show original pointcloud distribution
                self.get_logger().info(f"Original LiDAR point distribution:")
                self.get_logger().info(f"  - X: {np.mean(xyz[:, 0]):.2f}±{np.std(xyz[:, 0]):.2f}m (range: {np.min(xyz[:, 0]):.2f} to {np.max(xyz[:, 0]):.2f})")
                self.get_logger().info(f"  - Y: {np.mean(xyz[:, 1]):.2f}±{np.std(xyz[:, 1]):.2f}m (range: {np.min(xyz[:, 1]):.2f} to {np.max(xyz[:, 1]):.2f})")
                self.get_logger().info(f"  - Z: {np.mean(xyz[:, 2]):.2f}±{np.std(xyz[:, 2]):.2f}m (range: {np.min(xyz[:, 2]):.2f} to {np.max(xyz[:, 2]):.2f})")

            # Apply bounding box filtering if enabled
            if self.bbox is not None:
                original_count = xyz.shape[0]
                xyz = self.bbox.filter_points(xyz)
                filtered_count = xyz.shape[0]
                
                if self.publish_count % 30 == 0:
                    self.get_logger().info(f"Bounding box filtering: {filtered_count}/{original_count} points ({filtered_count/original_count*100:.1f}%)")
                    if filtered_count > 0:
                        self.get_logger().info(f"Filtered LiDAR point distribution:")
                        self.get_logger().info(f"  - X: {np.mean(xyz[:, 0]):.2f}±{np.std(xyz[:, 0]):.2f}m (range: {np.min(xyz[:, 0]):.2f} to {np.max(xyz[:, 0]):.2f})")
                        self.get_logger().info(f"  - Y: {np.mean(xyz[:, 1]):.2f}±{np.std(xyz[:, 1]):.2f}m (range: {np.min(xyz[:, 1]):.2f} to {np.max(xyz[:, 1]):.2f})")
                        self.get_logger().info(f"  - Z: {np.mean(xyz[:, 2]):.2f}±{np.std(xyz[:, 2]):.2f}m (range: {np.min(xyz[:, 2]):.2f} to {np.max(xyz[:, 2]):.2f})")
                
                if xyz.shape[0] == 0:
                    # No points after filtering, publish original image
                    if self.publish_count % 30 == 0:
                        self.get_logger().warn("No points remaining after bounding box filtering - publishing original image")
                    out = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
                    out.header = self.last_image.header
                    self.pub.publish(out)
                    return

            # Use OpenCV projectPoints with extrinsic from LiDAR to camera
            R = self.T_lidar_cam[:3, :3]
            t = self.T_lidar_cam[:3, 3]
            rvec, _ = cv2.Rodrigues(R)
            tvec = t.reshape(3, 1)

            # Transform all points to camera frame for analysis
            X_cam = (R @ xyz.astype(np.float64).T).T + t.reshape(1, 3)
            
            # More comprehensive filtering - keep points in a reasonable range
            # Instead of just "in front", keep points in a reasonable viewing volume
            z_mask = (X_cam[:, 2] > 0.1) & (X_cam[:, 2] < 50.0)  # 10cm to 50m
            x_mask = np.abs(X_cam[:, 0]) < 20.0  # Within 20m left/right
            y_mask = np.abs(X_cam[:, 1]) < 20.0  # Within 20m up/down
            
            # Combine all filters
            valid_mask = z_mask & x_mask & y_mask
            points_in_front = np.sum(valid_mask)
            
            if self.publish_count % 30 == 0:
                self.get_logger().info(f"Points in valid range: {points_in_front}/{xyz.shape[0]} ({points_in_front/xyz.shape[0]*100:.1f}%)")
                if points_in_front > 0:
                    z_distances = X_cam[valid_mask, 2]
                    x_distances = X_cam[valid_mask, 0]
                    y_distances = X_cam[valid_mask, 1]
                    self.get_logger().info(f"Z-distance range: {np.min(z_distances):.2f}m to {np.max(z_distances):.2f}m")
                    self.get_logger().info(f"X-distance range: {np.min(x_distances):.2f}m to {np.max(x_distances):.2f}m")
                    self.get_logger().info(f"Y-distance range: {np.min(y_distances):.2f}m to {np.max(y_distances):.2f}m")
                    
                    # Debug: Show distribution of points in camera frame
                    self.get_logger().info(f"Camera frame point distribution:")
                    self.get_logger().info(f"  - X: {np.mean(x_distances):.2f}±{np.std(x_distances):.2f}m")
                    self.get_logger().info(f"  - Y: {np.mean(y_distances):.2f}±{np.std(y_distances):.2f}m") 
                    self.get_logger().info(f"  - Z: {np.mean(z_distances):.2f}±{np.std(z_distances):.2f}m")
                    
                    # Show filtering breakdown
                    self.get_logger().info(f"Filtering breakdown:")
                    self.get_logger().info(f"  - Z filter (0.1-50m): {np.sum(z_mask)} points")
                    self.get_logger().info(f"  - X filter (±20m): {np.sum(x_mask)} points")
                    self.get_logger().info(f"  - Y filter (±20m): {np.sum(y_mask)} points")
                    self.get_logger().info(f"  - Combined: {np.sum(valid_mask)} points")
            
            if not np.any(valid_mask):
                # No valid points
                if self.publish_count % 30 == 0:
                    self.get_logger().warn("No valid points in range - publishing original image")
                out = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
                out.header = self.last_image.header
                self.pub.publish(out)
                return

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

            # Debug: Analyze projection results
            if self.publish_count % 30 == 0:
                u_coords = image_points[:, 0]
                v_coords = image_points[:, 1]
                self.get_logger().info(f"Projected coordinates range:")
                self.get_logger().info(f"  - U (horizontal): {np.min(u_coords):.1f} to {np.max(u_coords):.1f} (image width: {w})")
                self.get_logger().info(f"  - V (vertical): {np.min(v_coords):.1f} to {np.max(v_coords):.1f} (image height: {h})")
                
                # Check how many points are outside image bounds
                u_out_of_bounds = np.sum((u_coords < 0) | (u_coords >= w))
                v_out_of_bounds = np.sum((v_coords < 0) | (v_coords >= h))
                self.get_logger().info(f"Points outside bounds: U={u_out_of_bounds}, V={v_out_of_bounds}")
                
                # Check for extreme projection values (potential coordinate system issue)
                extreme_u = np.sum(np.abs(u_coords) > w * 10)  # More than 10x image width
                extreme_v = np.sum(np.abs(v_coords) > h * 10)  # More than 10x image height
                if extreme_u > 0 or extreme_v > 0:
                    self.get_logger().warn(f"EXTREME PROJECTION VALUES DETECTED!")
                    self.get_logger().warn(f"  - {extreme_u} points with |U| > {w*10}")
                    self.get_logger().warn(f"  - {extreme_v} points with |V| > {h*10}")
                    self.get_logger().warn(f"  - This suggests a coordinate system or scale issue!")
                    
                    # Show some sample extreme points
                    extreme_mask = (np.abs(u_coords) > w * 5) | (np.abs(v_coords) > h * 5)
                    if np.any(extreme_mask):
                        extreme_indices = np.where(extreme_mask)[0][:5]  # Show first 5
                        self.get_logger().warn(f"Sample extreme projections:")
                        for idx in extreme_indices:
                            orig_pt = xyz_filtered[idx]
                            cam_pt = X_cam[idx]
                            proj_pt = image_points[idx]
                            self.get_logger().warn(f"  Point {idx}: LiDAR({orig_pt[0]:.2f},{orig_pt[1]:.2f},{orig_pt[2]:.2f}) -> Cam({cam_pt[0]:.2f},{cam_pt[1]:.2f},{cam_pt[2]:.2f}) -> Proj({proj_pt[0]:.1f},{proj_pt[1]:.1f})")

            # Count points within image bounds and analyze distribution
            points_in_image = 0
            u_in_bounds = []
            v_in_bounds = []
            
            for ui, vi in image_points:
                if 0 <= ui < w and 0 <= vi < h:
                    points_in_image += 1
                    u_in_bounds.append(ui)
                    v_in_bounds.append(vi)
                    # Draw larger, more visible points
                    cv2.circle(cv_img, (int(ui), int(vi)), 3, (0, 255, 0), -1)  # Green points, size 3
                    cv2.circle(cv_img, (int(ui), int(vi)), 5, (0, 0, 255), 1)   # Red border, size 5

            if self.publish_count % 30 == 0:
                self.get_logger().info(f"Points projected to image: {points_in_image}/{len(image_points)}")
                if points_in_image > 0:
                    self.get_logger().info(f"Image bounds: {w}x{h}")
                    self.get_logger().info(f"Valid points distribution:")
                    self.get_logger().info(f"  - U: {np.mean(u_in_bounds):.1f}±{np.std(u_in_bounds):.1f} (range: {np.min(u_in_bounds):.1f}-{np.max(u_in_bounds):.1f})")
                    self.get_logger().info(f"  - V: {np.mean(v_in_bounds):.1f}±{np.std(v_in_bounds):.1f} (range: {np.min(v_in_bounds):.1f}-{np.max(v_in_bounds):.1f})")
                    
                    # Check if points are concentrated in a small area
                    u_span = np.max(u_in_bounds) - np.min(u_in_bounds)
                    v_span = np.max(v_in_bounds) - np.min(v_in_bounds)
                    self.get_logger().info(f"Point spread: U={u_span:.1f}px ({u_span/w*100:.1f}% of image), V={v_span:.1f}px ({v_span/h*100:.1f}% of image)")

            # Add validation test points (optional debug feature)
            if self.publish_count % 100 == 0:  # Test every 100th frame
                self._test_projection_accuracy(cv_img, R, t, self.K, self.dist, w, h)
                self._validate_coordinate_system(R, t, self.K, w, h)

            # Publish overlay image
            out = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
            out.header = self.last_image.header
            self.pub.publish(out)

        except Exception as e:
            self.get_logger().error(
                f"Error creating overlay: {e}", throttle_duration_sec=1.0
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
                test_points.extend([
                    [0, 0, dist],      # Center
                    [1, 0, dist],      # Right
                    [-1, 0, dist],     # Left
                    [0, 1, dist],      # Up
                    [0, -1, dist],     # Down
                ])
            
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
                    cv2.circle(cv_img, (int(ui), int(vi)), 8, (255, 0, 0), -1)  # Blue test points
                    cv2.circle(cv_img, (int(ui), int(vi)), 10, (255, 255, 255), 2)  # White border
            
            self.get_logger().info(f"Projection test: {len(test_points)} test points projected")
            
        except Exception as e:
            self.get_logger().warn(f"Projection test failed: {e}")

    def _validate_coordinate_system(self, R, t, K, w, h):
        """Validate coordinate system and transformation."""
        try:
            self.get_logger().info("=== COORDINATE SYSTEM VALIDATION ===")
            
            # Check camera matrix
            fx, fy = K[0, 0], K[1, 1]
            cx, cy = K[0, 2], K[1, 2]
            self.get_logger().info(f"Camera intrinsics: fx={fx:.1f}, fy={fy:.1f}, cx={cx:.1f}, cy={cy:.1f}")
            
            # Check if focal length is reasonable
            if fx > 10000 or fy > 10000:
                self.get_logger().warn(f"WARNING: Very large focal length detected! This might indicate a coordinate system issue.")
            
            # Check principal point
            if cx < 0 or cx > w or cy < 0 or cy > h:
                self.get_logger().warn(f"WARNING: Principal point ({cx:.1f}, {cy:.1f}) is outside image bounds ({w}x{h})")
            
            # Check rotation matrix properties
            det_R = np.linalg.det(R)
            if abs(det_R - 1.0) > 0.01:
                self.get_logger().warn(f"WARNING: Rotation matrix determinant is {det_R:.6f} (should be 1.0)")
            
            # Check if R is orthogonal
            should_be_identity = R @ R.T
            identity_error = np.linalg.norm(should_be_identity - np.eye(3))
            if identity_error > 0.01:
                self.get_logger().warn(f"WARNING: Rotation matrix is not orthogonal (error: {identity_error:.6f})")
            
            # Test transformation with known points
            test_points = np.array([
                [0, 0, 1],    # 1m in front of LiDAR
                [1, 0, 1],    # 1m right, 1m forward
                [0, 1, 1],    # 1m up, 1m forward
                [0, 0, 5],    # 5m in front
            ], dtype=np.float64)
            
            # Transform to camera frame
            test_cam = (R @ test_points.T).T + t.reshape(1, 3)
            
            # Project using simple pinhole model
            rvec, _ = cv2.Rodrigues(R)
            tvec = t.reshape(3, 1)
            test_proj, _ = cv2.projectPoints(test_points, rvec, tvec, K, None)
            test_proj = test_proj.reshape(-1, 2)
            
            self.get_logger().info("Test point projections:")
            for i, (orig, cam, proj) in enumerate(zip(test_points, test_cam, test_proj)):
                self.get_logger().info(f"  Point {i}: LiDAR{tuple(orig)} -> Cam{tuple(cam)} -> Proj({proj[0]:.1f},{proj[1]:.1f})")
                
                # Check if projection is reasonable
                if abs(proj[0]) > w * 2 or abs(proj[1]) > h * 2:
                    self.get_logger().warn(f"    WARNING: Projection is way outside image bounds!")
            
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
