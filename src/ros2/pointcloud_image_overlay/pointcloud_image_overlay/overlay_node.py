#!/usr/bin/env python3
"""
Educational Pointcloud Image Overlay Node

This educational implementation demonstrates the core concepts of LiDAR-camera sensor fusion:
1. Real-time extrinsic calibration from live calibration solver
2. 3D coordinate transformation (LiDAR → Camera frame)
3. Camera projection (3D → 2D image coordinates)
4. Visual overlay of LiDAR points on camera images

Key Educational Concepts:
- Coordinate system transformations using homogeneous matrices
- Camera intrinsic parameters and projection model
- Real-time sensor fusion and synchronization
- ROS2 multi-topic subscription patterns

Author: Educational Version for LCTK
Version: Simplified for learning purposes (~250 lines vs 740 original)
Compatible with: OpenCV 4.5.4, NumPy 1.21.5 (Ubuntu 22.04)
"""

import math
import struct
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
# ROS 2 message types
from sensor_msgs.msg import CameraInfo, Image, PointCloud2


def pointcloud2_to_xyz(pc2: PointCloud2) -> np.ndarray:
    """
    Educational Function: Convert PointCloud2 message to NumPy array

    This function demonstrates how to extract 3D coordinates from ROS PointCloud2 messages.
    PointCloud2 uses a binary format where each point contains x,y,z coordinates as floats.

    Args:
        pc2: ROS PointCloud2 message containing LiDAR scan data

    Returns:
        np.ndarray: Nx3 array of [x,y,z] coordinates in LiDAR frame

    Educational Notes:
        - PointCloud2 data is stored as packed binary (struct format)
        - Field offsets tell us where x,y,z are located in each point
        - We filter out invalid points (NaN, infinity) for robust processing
    """
    if pc2.point_step == 0 or len(pc2.data) == 0:
        return np.zeros((0, 3), dtype=np.float32)

    # Find byte offsets for x,y,z coordinates in the binary data
    offset = {f.name: f.offset for f in pc2.fields}
    if "x" not in offset or "y" not in offset or "z" not in offset:
        return np.zeros((0, 3), dtype=np.float32)

    step = pc2.point_step  # Bytes per point
    xyz = []

    # Extract xyz coordinates from binary data
    for i in range(0, len(pc2.data), step):
        # Unpack 32-bit float from binary data at specific offset
        x = struct.unpack_from("f", pc2.data, i + offset["x"])[0]
        y = struct.unpack_from("f", pc2.data, i + offset["y"])[0]
        z = struct.unpack_from("f", pc2.data, i + offset["z"])[0]

        # Only keep valid finite coordinates
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            xyz.append((x, y, z))

    if not xyz:
        return np.zeros((0, 3), dtype=np.float32)
    return np.asarray(xyz, dtype=np.float32)


def transform_to_rvec_tvec(
    transform: TransformStamped,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Educational Function: Convert ROS Transform to OpenCV rotation and translation vectors

    This demonstrates how to convert ROS geometry_msgs/Transform to the rotation vector (rvec)
    and translation vector (tvec) format used by OpenCV's projectPoints function.

    Args:
        transform: ROS TransformStamped message (from extrinsic_solver_node)

    Returns:
        tuple: (rvec, tvec) - rotation vector and translation vector for OpenCV

    Educational Notes:
        - OpenCV uses Rodrigues rotation vectors (3x1) instead of matrices
        - Translation vector (3x1) represents spatial offset
        - projectPoints will apply both extrinsic and intrinsic transformations
        - This approach lets OpenCV handle the full transformation pipeline
    """
    # Extract translation vector (3x1) for OpenCV
    t = transform.transform.translation
    tvec = np.array([t.x, t.y, t.z], dtype=np.float64)

    # Extract rotation quaternion (w + x*i + y*j + z*k)
    q = transform.transform.rotation
    qx, qy, qz, qw = q.x, q.y, q.z, q.w

    # Convert quaternion to 3x3 rotation matrix
    # This is the mathematical transformation from quaternion to rotation matrix
    rotation_matrix = np.array(
        [
            [1 - 2 * (qy**2 + qz**2), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx**2 + qz**2), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx**2 + qy**2)],
        ],
        dtype=np.float64,
    )

    # Convert rotation matrix to Rodrigues rotation vector for OpenCV
    # cv2.Rodrigues converts between 3x3 rotation matrix and 3x1 rotation vector
    rvec, _ = cv2.Rodrigues(rotation_matrix)

    return rvec.reshape(3), tvec.reshape(3)


class EducationalOverlayNode(Node):
    """
    Educational LiDAR-Camera Overlay Node

    This class demonstrates the essential components of sensor fusion between
    LiDAR and camera systems using real-time extrinsic calibration.

    Educational Architecture:
    1. Subscribe to live extrinsic calibration (geometry_msgs/TransformStamped)
    2. Subscribe to sensor data (Image, PointCloud2, CameraInfo)
    3. Transform LiDAR points to camera coordinate system
    4. Project 3D camera points to 2D image pixels
    5. Visualize overlay by drawing points on image

    Key Learning Points:
    - Real-time calibration vs static configuration files
    - Coordinate system transformations in robotics
    - Camera projection model and intrinsic parameters
    - Multi-sensor data synchronization patterns
    """

    def __init__(self):
        super().__init__("educational_pointcloud_overlay")

        # OpenCV bridge for ROS Image ↔ numpy array conversion
        self.bridge = CvBridge()

        # Educational counters for monitoring data flow
        self.message_counts = {
            "images": 0,
            "pointclouds": 0,
            "camera_info": 0,
            "extrinsics": 0,
            "overlays_published": 0,
        }

        # === EDUCATIONAL STATE VARIABLES ===
        # These demonstrate what data we need for sensor fusion

        # Camera intrinsic parameters (from camera_info topic)
        self.camera_matrix: Optional[np.ndarray] = None  # 3x3 K matrix
        self.distortion: Optional[np.ndarray] = None  # Distortion coefficients

        # Extrinsic calibration (from live calibration solver)
        self.extrinsic_rvec: Optional[np.ndarray] = None  # Rotation vector (3x1)
        self.extrinsic_tvec: Optional[np.ndarray] = None  # Translation vector (3x1)

        # Latest sensor data (for overlay generation)
        self.latest_image: Optional[Image] = None
        self.latest_pointcloud: Optional[PointCloud2] = None

        # === ROS 2 QUALITY OF SERVICE CONFIGURATION ===
        # Educational note: QoS affects message delivery reliability
        self.declare_parameter("use_best_effort_qos", True)
        use_best_effort = (
            self.get_parameter("use_best_effort_qos").get_parameter_value().bool_value
        )

        if use_best_effort:
            # Best effort: Fast, may lose messages (good for live sensors)
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                depth=10,
            )
        else:
            # Reliable: Guaranteed delivery (good for recorded data/rosbags)
            qos = QoSProfile(depth=10)

        # === SENSOR DATA SUBSCRIPTIONS ===
        # Educational pattern: subscribe to all required sensor streams

        # Camera image stream (sensor_msgs/Image)
        self.image_subscription = self.create_subscription(
            Image, "image", self.on_image_received, qos
        )

        # LiDAR pointcloud stream (sensor_msgs/PointCloud2)
        self.pointcloud_subscription = self.create_subscription(
            PointCloud2, "pointcloud", self.on_pointcloud_received, qos
        )

        # Camera intrinsic parameters (sensor_msgs/CameraInfo)
        # Auto-derive topic name from image topic (image_pipeline convention)
        resolved_image_topic = self.image_subscription.topic_name
        if "/" in resolved_image_topic:
            base_path = resolved_image_topic.rsplit("/", 1)[0]
            camera_info_topic = f"{base_path}/camera_info"
        else:
            camera_info_topic = "camera_info"

        self.camera_info_subscription = self.create_subscription(
            CameraInfo, camera_info_topic, self.on_camera_info_received, qos
        )

        # === LIVE EXTRINSIC CALIBRATION SUBSCRIPTION ===
        # Educational highlight: Real-time calibration from solver instead of static file
        self.extrinsic_subscription = self.create_subscription(
            TransformStamped,
            "/calibration/extrinsic_solver/extrinsic_transform",  # Live calibration output
            self.on_extrinsic_received,
            qos,
        )

        # === OUTPUT PUBLISHER ===
        # Publish overlay visualization (sensor_msgs/Image)
        self.overlay_publisher = self.create_publisher(
            Image, "/calibration/pointcloud_overlay", 10
        )

        # Educational logging
        self.get_logger().info("Educational Pointcloud Overlay Node Started!")
        self.get_logger().info("Learning Objectives:")
        self.get_logger().info("   - Real-time extrinsic calibration integration")
        self.get_logger().info("   - 3D to 2D coordinate transformation pipeline")
        self.get_logger().info("   - Multi-sensor data synchronization")
        self.get_logger().info("   - Camera projection model application")
        self.get_logger().info(
            f"QoS Mode: {'Best Effort' if use_best_effort else 'Reliable'}"
        )

    def on_extrinsic_received(self, msg: TransformStamped):
        """
        Educational Callback: Handle live extrinsic calibration updates

        This demonstrates real-time calibration - instead of loading a static file,
        we receive live updates from the extrinsic_solver_node as it refines the
        calibration between LiDAR and camera coordinate systems.

        Args:
            msg: Live extrinsic transform (LiDAR frame → Camera frame)
        """
        self.message_counts["extrinsics"] += 1

        # Convert ROS transform to OpenCV rvec and tvec for projectPoints
        self.extrinsic_rvec, self.extrinsic_tvec = transform_to_rvec_tvec(msg)

        # Educational logging (every 10th message to avoid spam)
        if self.message_counts["extrinsics"] % 10 == 0:
            self.get_logger().info(
                f"Live Extrinsic Update #{self.message_counts['extrinsics']}: "
                f"Translation = [{msg.transform.translation.x:.3f}, "
                f"{msg.transform.translation.y:.3f}, {msg.transform.translation.z:.3f}]"
            )

    def on_camera_info_received(self, msg: CameraInfo):
        """
        Educational Callback: Handle camera intrinsic parameters

        Camera intrinsics define how 3D points project to 2D image coordinates.
        This includes focal length, principal point, and distortion coefficients.

        Args:
            msg: Camera calibration parameters (sensor_msgs/CameraInfo)
        """
        self.message_counts["camera_info"] += 1

        # Extract 3x3 camera matrix K from ROS message
        # K = [fx  0  cx]  where (fx,fy) = focal lengths, (cx,cy) = principal point
        #     [0  fy  cy]
        #     [0   0   1]
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)

        # Extract distortion coefficients (lens distortion correction)
        if msg.d:
            self.distortion = np.array(msg.d, dtype=np.float64)
        else:
            self.distortion = np.zeros(5, dtype=np.float64)  # No distortion

        # Educational logging (first time only)
        if self.message_counts["camera_info"] == 1:
            fx, fy = self.camera_matrix[0, 0], self.camera_matrix[1, 1]
            cx, cy = self.camera_matrix[0, 2], self.camera_matrix[1, 2]
            self.get_logger().info(
                f"Camera Intrinsics Loaded: "
                f"Focal=[{fx:.1f}, {fy:.1f}], Principal=[{cx:.1f}, {cy:.1f}], "
                f"Image={msg.width}x{msg.height}"
            )

    def on_image_received(self, msg: Image):
        """
        Educational Callback: Handle camera image updates

        Images provide the visual context for overlay. We always try to generate
        an overlay when new images arrive to maintain real-time visualization.

        Args:
            msg: Camera image (sensor_msgs/Image)
        """
        self.message_counts["images"] += 1
        self.latest_image = msg

        # Educational logging (every 30th image to avoid spam)
        if self.message_counts["images"] % 30 == 0:
            self.get_logger().info(
                f"Image #{self.message_counts['images']}: "
                f"{msg.width}x{msg.height}, encoding={msg.encoding}"
            )

        # Always attempt overlay generation when new image arrives
        self.generate_overlay()

    def on_pointcloud_received(self, msg: PointCloud2):
        """
        Educational Callback: Handle LiDAR pointcloud updates

        Pointclouds provide 3D spatial information to overlay on 2D images.
        We store the latest pointcloud and trigger overlay if image is available.

        Args:
            msg: LiDAR scan data (sensor_msgs/PointCloud2)
        """
        self.message_counts["pointclouds"] += 1
        self.latest_pointcloud = msg

        # Educational logging (every 30th pointcloud to avoid spam)
        if self.message_counts["pointclouds"] % 30 == 0:
            self.get_logger().info(
                f"Pointcloud #{self.message_counts['pointclouds']}: "
                f"{len(msg.data)} bytes, {len(msg.fields)} fields"
            )

        # Only generate overlay if we have a recent image
        if self.latest_image is not None:
            self.generate_overlay()

    def generate_overlay(self):
        """
        Educational Core Function: Generate LiDAR-Camera Overlay

        This is the main educational demonstration of sensor fusion pipeline:
        1. Check all required data is available
        2. Extract 3D points from LiDAR pointcloud
        3. Transform points from LiDAR coordinate system to camera coordinate system
        4. Project 3D camera points to 2D image pixel coordinates
        5. Draw projected points on camera image
        6. Publish overlay for visualization

        Educational Value:
        - Shows complete 3D→2D transformation pipeline
        - Demonstrates coordinate system transformations
        - Illustrates real-time sensor fusion concepts
        """
        # === STEP 1: VALIDATE ALL REQUIRED DATA ===
        # Educational pattern: check prerequisites before processing
        if self.latest_image is None:
            return  # No image to overlay on

        if self.camera_matrix is None:
            self._draw_status_message("No camera intrinsics available")
            return

        if self.extrinsic_rvec is None or self.extrinsic_tvec is None:
            self._draw_status_message("No extrinsic calibration available")
            return

        if self.latest_pointcloud is None:
            self._draw_status_message("No LiDAR data available")
            return

        try:
            # === STEP 2: EXTRACT 3D POINTS FROM LIDAR ===
            # Convert ROS PointCloud2 binary format to NumPy array
            lidar_points = pointcloud2_to_xyz(self.latest_pointcloud)

            if lidar_points.shape[0] == 0:
                self._draw_status_message("Empty pointcloud")
                return

            # === STEP 3: DIRECT 3D → 2D PROJECTION WITH FULL TRANSFORMATION ===
            # Educational highlight: OpenCV's projectPoints handles the complete pipeline
            # - Applies extrinsic transformation (LiDAR → Camera) using rvec and tvec
            # - Applies intrinsic projection (3D Camera → 2D Image) using camera matrix
            # - Applies lens distortion correction if provided
            # This single function call replaces manual matrix multiplication

            # OpenCV projectPoints performs the full transformation pipeline:
            # 1. Rotate points using rvec (Rodrigues rotation)
            # 2. Translate points using tvec
            # 3. Project to 2D using camera intrinsics
            # 4. Apply distortion correction
            projected_points, _ = cv2.projectPoints(
                lidar_points.reshape(-1, 1, 3),  # LiDAR points in original frame
                self.extrinsic_rvec,  # Rotation from LiDAR to camera
                self.extrinsic_tvec,  # Translation from LiDAR to camera
                self.camera_matrix,  # Intrinsic parameters [fx,fy,cx,cy]
                self.distortion,  # Distortion coefficients
            )

            # === STEP 4: FILTER POINTS FOR VISIBILITY ===
            # Educational note: We need to filter points that are behind the camera
            # After projection, we check which points would be visible
            # Transform points to camera frame for Z-check (points behind camera have negative Z)

            # Compute camera frame points for filtering (only for Z-check)
            # Apply rotation
            R, _ = cv2.Rodrigues(self.extrinsic_rvec)
            camera_points = (R @ lidar_points.T).T + self.extrinsic_tvec.reshape(1, 3)

            # Filter points behind camera (negative Z in camera frame)
            front_mask = camera_points[:, 2] > 0.1  # At least 10cm in front

            if not np.any(front_mask):
                self._draw_status_message("No points in front of camera")
                return

            # Filter projected points to only include those in front
            projected_points_front = projected_points[front_mask]

            # Reshape to simple 2D array: [[u1,v1], [u2,v2], ...]
            image_points = projected_points_front.reshape(-1, 2)

            # === STEP 5: VISUAL OVERLAY ===
            # Draw projected LiDAR points on camera image
            overlay_image = self._create_visual_overlay(image_points)

            # === STEP 6: PUBLISH RESULT ===
            # Convert back to ROS Image message and publish
            self._publish_overlay(overlay_image)

            # Educational statistics logging
            self.message_counts["overlays_published"] += 1
            if self.message_counts["overlays_published"] % 30 == 0:
                self.get_logger().info(
                    f"Overlay #{self.message_counts['overlays_published']}: "
                    f"{np.sum(front_mask)}/{len(lidar_points)} points visible"
                )

        except Exception as e:
            self.get_logger().error(f"Overlay generation failed: {str(e)}")
            self._draw_status_message(f"Processing error: {str(e)}")

    def _create_visual_overlay(self, image_points: np.ndarray) -> np.ndarray:
        """
        Educational Helper: Create visual overlay of LiDAR points on camera image

        Args:
            image_points: Projected 2D pixel coordinates [[u1,v1], [u2,v2], ...]

        Returns:
            np.ndarray: Camera image with LiDAR points overlaid
        """
        # Convert ROS Image to OpenCV format (BGR8)
        cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding="bgr8")
        h, w = cv_image.shape[:2]

        # Draw each projected point as a colored circle
        points_drawn = 0
        for u, v in image_points:
            # Only draw points that fall within image boundaries
            if 0 <= u < w and 0 <= v < h:
                # Draw filled green circle (LiDAR point)
                cv2.circle(cv_image, (int(u), int(v)), 2, (0, 255, 0), -1)
                # Draw red border for better visibility
                cv2.circle(cv_image, (int(u), int(v)), 3, (0, 0, 255), 1)
                points_drawn += 1

        # Add educational status overlay
        status_text = f"LiDAR Points: {points_drawn}/{len(image_points)} visible"
        cv2.putText(
            cv_image,
            status_text,
            (10, h - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
        )

        return cv_image

    def _draw_status_message(self, message: str):
        """
        Educational Helper: Draw status message on image when data is unavailable

        Args:
            message: Status message to display
        """
        if self.latest_image is None:
            return

        # Convert image and draw status message
        cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding="bgr8")
        h, w = cv_image.shape[:2]

        # Draw semi-transparent background for text
        overlay = cv_image.copy()
        cv2.rectangle(overlay, (10, 10), (w - 10, 60), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, cv_image, 0.3, 0, cv_image)

        # Draw status message
        cv2.putText(
            cv_image, message, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2
        )

        self._publish_overlay(cv_image)

    def _publish_overlay(self, cv_image: np.ndarray):
        """
        Educational Helper: Publish overlay image as ROS message

        Args:
            cv_image: OpenCV image with overlay visualization
        """
        try:
            # Convert OpenCV image back to ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            ros_image.header = self.latest_image.header  # Preserve timestamp and frame

            # Publish for visualization in RViz, image viewers, etc.
            self.overlay_publisher.publish(ros_image)

        except Exception as e:
            self.get_logger().error(f"Failed to publish overlay: {str(e)}")


def main():
    """
    Educational Main Function: ROS 2 Node Lifecycle

    This demonstrates the standard ROS 2 Python node lifecycle pattern.
    """
    # Initialize ROS 2 Python client library
    rclpy.init()

    # Create our educational node instance
    node = EducationalOverlayNode()

    try:
        # Enter ROS 2 event loop (handles callbacks, message processing)
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Graceful shutdown on Ctrl+C
        node.get_logger().info("Educational node shutting down...")
    finally:
        # Clean up resources
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
