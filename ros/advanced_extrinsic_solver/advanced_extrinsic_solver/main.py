#!/usr/bin/env python3
"""
Advanced Extrinsic Calibration Node

This ROS2 node performs high-quality LiDAR-camera extrinsic calibration
using buffered multi-pose calibration with the Perspective-n-Point (PnP) algorithm.

Key Features:
1. Buffer-based detection storage for multi-pose calibration
2. Service-driven workflow for user-controlled data selection
3. Continuous transform publishing after successful calibration
4. Enhanced accuracy through aggregated point correspondences

Workflow:
1. Play rosbag with calibration data
2. Call add_detection service to buffer good detection pairs
3. Repeat for multiple board poses
4. Node automatically re-solves using all buffered data
5. Continuous publishing of optimized extrinsic transform

Author: LCTK Team
License: MIT
"""

import threading
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from lctk_interfaces.srv import (
    AddDetectionToBuffer,
    ClearDetectionBuffer,
    GetBufferStatus,
    ListDetectionBuffer,
    RemoveDetectionFromBuffer,
)
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header
from vision_msgs.msg import Detection2DArray, Detection3DArray


@dataclass
class ArUcoMarker:
    """Represents an ArUco marker detection in image coordinates."""

    id: int
    corners: List[Tuple[float, float]]  # 4 corners in pixel coordinates
    center: Tuple[float, float]  # Center point in pixels


@dataclass
class BoardDetection:
    """Represents a calibration board detection in 3D LiDAR coordinates."""

    position: Tuple[float, float, float]  # x, y, z in meters (LiDAR frame)
    orientation: Tuple[float, float, float, float]  # quaternion x, y, z, w


class AdvancedExtrinsicSolver(Node):
    """
    Advanced ROS2 node for multi-pose LiDAR-camera extrinsic calibration.

    This node uses a buffer-based approach for collecting detection data from
    multiple calibration board poses, then solves a single optimized PnP problem
    using all accumulated correspondences for improved accuracy.

    Services:
    - add_detection: Add current detection pair to buffer and re-solve
    - clear_buffer: Clear buffer and stop publishing
    - get_status: Query buffer status and calibration state

    Topics:
    - Subscribes: aruco_detections, calibration_board_detections, camera_info
    - Publishes: extrinsic_transform (continuous at 10Hz when solved)
    """

    def __init__(self):
        super().__init__("advanced_extrinsic_solver")

        # Essential parameter declarations
        self.declare_parameter("parent_frame", "lidar")
        self.declare_parameter("child_frame", "camera")
        self.declare_parameter("camera_topic", "")
        self.declare_parameter("aruco_config_file", "")
        self.declare_parameter("debug_mode", True)
        self.declare_parameter("publishing_rate", 10.0)
        self.declare_parameter("min_poses_required", 2)

        # Get parameters
        self.parent_frame = (
            self.get_parameter("parent_frame").get_parameter_value().string_value
        )
        self.child_frame = (
            self.get_parameter("child_frame").get_parameter_value().string_value
        )
        aruco_config_file = (
            self.get_parameter("aruco_config_file").get_parameter_value().string_value
        )
        publishing_rate = (
            self.get_parameter("publishing_rate").get_parameter_value().double_value
        )
        self.min_poses_required = (
            self.get_parameter("min_poses_required").get_parameter_value().integer_value
        )

        # Load ArUco pattern configuration
        self.aruco_pattern_config = self._load_aruco_pattern_config(aruco_config_file)

        # Detection buffer for multi-pose calibration
        self.detection_buffer: List[Tuple[Detection2DArray, Detection3DArray]] = []

        # Latest detections (cached for service calls)
        self.latest_aruco_detection: Optional[Detection2DArray] = None
        self.latest_board_detection: Optional[Detection3DArray] = None
        self.camera_info: Optional[CameraInfo] = None

        # Calibration state
        self.last_transform: Optional[TransformStamped] = None
        self.publishing_enabled = False
        self.last_solve_status = "No calibration performed yet"
        self.total_correspondences = 0

        # Thread safety
        self.lock = threading.Lock()

        # QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Publishers
        self.transform_publisher = self.create_publisher(
            TransformStamped, "extrinsic_transform", qos_profile
        )

        # Publishing timer (10Hz continuous publishing when enabled)
        self.publishing_timer = self.create_timer(
            1.0 / publishing_rate, self._publishing_timer_callback
        )

        # Subscribers
        self.aruco_subscription = self.create_subscription(
            Detection2DArray, "aruco_detections", self.aruco_callback, qos_profile
        )

        self.board_subscription = self.create_subscription(
            Detection3DArray,
            "calibration_board_detections",
            self.board_callback,
            qos_profile,
        )

        # Derive camera_info topic from camera_topic parameter
        camera_topic = (
            self.get_parameter("camera_topic").get_parameter_value().string_value
        )
        if camera_topic:
            if "/" in camera_topic:
                base_path = camera_topic.rsplit("/", 1)[0]
                camera_info_topic = f"{base_path}/camera_info"
            else:
                camera_info_topic = "camera_info"
            self.get_logger().info(
                f"Deriving camera_info topic: '{camera_topic}' -> '{camera_info_topic}'"
            )
        else:
            camera_info_topic = "camera_info"

        self.camera_info_subscription = self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback, qos_profile
        )

        # Services
        self.add_detection_service = self.create_service(
            AddDetectionToBuffer,
            "~/add_detection",
            self.add_detection_callback,
        )

        self.clear_buffer_service = self.create_service(
            ClearDetectionBuffer,
            "~/clear_buffer",
            self.clear_buffer_callback,
        )

        self.get_status_service = self.create_service(
            GetBufferStatus,
            "~/get_status",
            self.get_status_callback,
        )

        self.list_buffer_service = self.create_service(
            ListDetectionBuffer,
            "~/list_buffer",
            self.list_buffer_callback,
        )

        self.remove_detection_service = self.create_service(
            RemoveDetectionFromBuffer,
            "~/remove_detection",
            self.remove_detection_callback,
        )

        self.get_logger().info(
            f"Advanced Extrinsic Solver initialized\n"
            f"Mode: Multi-pose buffered calibration\n"
            f"Minimum poses required: {self.min_poses_required}\n"
            f"Subscribing to: aruco_detections, calibration_board_detections, {camera_info_topic}\n"
            f"Publishing to: extrinsic_transform (at {publishing_rate}Hz when enabled)\n"
            f"Transform: {self.parent_frame} -> {self.child_frame}\n"
            f"Services: ~/add_detection, ~/clear_buffer, ~/get_status, ~/list_buffer, ~/remove_detection"
        )

    def camera_info_callback(self, msg: CameraInfo):
        """Cache camera info for PnP solving."""
        with self.lock:
            self.camera_info = msg
            self.get_logger().debug(f"Camera info received: {msg.width}x{msg.height}")

    def aruco_callback(self, msg: Detection2DArray):
        """Cache ArUco detections without solving."""
        self.get_logger().debug(
            f"ArUco detection: {len(msg.detections)} markers at "
            f"t={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}"
        )

        if msg.detections:
            with self.lock:
                self.latest_aruco_detection = msg
        else:
            self.get_logger().debug("Ignoring empty ArUco detection")

    def board_callback(self, msg: Detection3DArray):
        """Cache board detections without solving."""
        self.get_logger().debug(
            f"Board detection: {len(msg.detections)} boards at "
            f"t={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}"
        )

        if msg.detections:
            with self.lock:
                self.latest_board_detection = msg
        else:
            self.get_logger().warn("Received empty board detection")

    def _publishing_timer_callback(self):
        """Continuously publish the last solved transform when enabled."""
        with self.lock:
            if self.publishing_enabled and self.last_transform:
                # Update timestamp to current time
                self.last_transform.header.stamp = self.get_clock().now().to_msg()
                self.transform_publisher.publish(self.last_transform)

    def add_detection_callback(self, request, response):
        """
        Service callback: Add latest detection pair to buffer and re-solve.

        This triggers a complete re-solve using all buffered detections,
        potentially improving calibration accuracy with more data.
        """
        with self.lock:
            aruco_msg = self.latest_aruco_detection
            board_msg = self.latest_board_detection

        # Validate prerequisites
        if not self.camera_info:
            response.success = False
            response.message = "No camera info available"
            response.buffer_size = len(self.detection_buffer)
            self.get_logger().error(response.message)
            return response

        if not aruco_msg or not board_msg:
            response.success = False
            missing = []
            if not aruco_msg:
                missing.append("ArUco")
            if not board_msg:
                missing.append("Board")
            response.message = f"Missing detections: {', '.join(missing)}"
            response.buffer_size = len(self.detection_buffer)
            self.get_logger().error(response.message)
            return response

        if not aruco_msg.detections or not board_msg.detections:
            response.success = False
            response.message = "Empty detection messages"
            response.buffer_size = len(self.detection_buffer)
            self.get_logger().error(response.message)
            return response

        # Check for duplicate/similar board poses
        new_board_pos = np.array([
            board_msg.detections[0].results[0].pose.pose.position.x,
            board_msg.detections[0].results[0].pose.pose.position.y,
            board_msg.detections[0].results[0].pose.pose.position.z
        ])
        
        new_board_quat = np.array([
            board_msg.detections[0].results[0].pose.pose.orientation.x,
            board_msg.detections[0].results[0].pose.pose.orientation.y,
            board_msg.detections[0].results[0].pose.pose.orientation.z,
            board_msg.detections[0].results[0].pose.pose.orientation.w
        ])
        
        # Check against all existing poses in buffer
        duplicate_found = False
        min_position_distance = 0.1  # 10cm minimum movement
        min_rotation_distance = 0.087  # ~5 degrees (in quaternion space)
        
        for idx, (existing_aruco, existing_board) in enumerate(self.detection_buffer):
            existing_pos = np.array([
                existing_board.detections[0].results[0].pose.pose.position.x,
                existing_board.detections[0].results[0].pose.pose.position.y,
                existing_board.detections[0].results[0].pose.pose.position.z
            ])
            
            existing_quat = np.array([
                existing_board.detections[0].results[0].pose.pose.orientation.x,
                existing_board.detections[0].results[0].pose.pose.orientation.y,
                existing_board.detections[0].results[0].pose.pose.orientation.z,
                existing_board.detections[0].results[0].pose.pose.orientation.w
            ])
            
            pos_distance = np.linalg.norm(new_board_pos - existing_pos)
            # Quaternion distance: min(||q1-q2||, ||q1+q2||) due to double cover
            quat_dist1 = np.linalg.norm(new_board_quat - existing_quat)
            quat_dist2 = np.linalg.norm(new_board_quat + existing_quat)
            rot_distance = min(quat_dist1, quat_dist2)
            
            if pos_distance < min_position_distance and rot_distance < min_rotation_distance:
                duplicate_found = True
                self.get_logger().warn(
                    f"Duplicate pose detected! Too similar to pose #{idx+1} in buffer:\n"
                    f"  Position distance: {pos_distance:.4f}m (threshold: {min_position_distance}m)\n"
                    f"  Rotation distance: {rot_distance:.4f} (threshold: {min_rotation_distance})\n"
                    f"  Existing pose: ({existing_pos[0]:.4f}, {existing_pos[1]:.4f}, {existing_pos[2]:.4f})\n"
                    f"  New pose:      ({new_board_pos[0]:.4f}, {new_board_pos[1]:.4f}, {new_board_pos[2]:.4f})"
                )
                break
        
        if duplicate_found:
            response.success = False
            response.message = (
                f"Rejected: Board pose too similar to existing pose #{idx+1}. "
                f"Move board at least {min_position_distance}m or rotate {np.degrees(2*np.arcsin(min_rotation_distance/2)):.1f}° before adding."
            )
            response.buffer_size = len(self.detection_buffer)
            self.get_logger().error(response.message)
            return response

        # Add to buffer
        with self.lock:
            self.detection_buffer.append((aruco_msg, board_msg))
            buffer_size = len(self.detection_buffer)

        # Log successful addition with diversity metrics
        if buffer_size == 1:
            self.get_logger().info(
                f"✓ Added detection pair #1 to buffer (initial pose)\n"
                f"  Board position: ({new_board_pos[0]:.4f}, {new_board_pos[1]:.4f}, {new_board_pos[2]:.4f})"
            )
        else:
            # Calculate distance to nearest existing pose
            min_dist = float('inf')
            for existing_aruco, existing_board in self.detection_buffer[:-1]:  # Exclude the one we just added
                existing_pos = np.array([
                    existing_board.detections[0].results[0].pose.pose.position.x,
                    existing_board.detections[0].results[0].pose.pose.position.y,
                    existing_board.detections[0].results[0].pose.pose.position.z
                ])
                dist = np.linalg.norm(new_board_pos - existing_pos)
                min_dist = min(min_dist, dist)
            
            self.get_logger().info(
                f"✓ Added detection pair #{buffer_size} to buffer\n"
                f"  Board position: ({new_board_pos[0]:.4f}, {new_board_pos[1]:.4f}, {new_board_pos[2]:.4f})\n"
                f"  Distance to nearest pose: {min_dist:.4f}m\n"
                f"  Pose diversity: Good ({'>' if min_dist >= 0.2 else '>='} {min_position_distance}m threshold)"
            )

        # Re-solve calibration from entire buffer
        success = self._solve_from_buffer()

        if success:
            response.success = True
            response.message = (
                f"Added detection pair and solved calibration successfully "
                f"({self.total_correspondences} correspondences from {buffer_size} poses)"
            )
            response.buffer_size = buffer_size
            self.get_logger().info(response.message)
        else:
            # Check if we just need more poses (not an error, just waiting)
            if buffer_size < self.min_poses_required:
                response.success = True  # Detection was added successfully
                response.message = (
                    f"Detection buffered ({buffer_size}/{self.min_poses_required} poses). "
                    f"Add {self.min_poses_required - buffer_size} more to solve calibration."
                )
                response.buffer_size = buffer_size
                self.get_logger().info(response.message)
            else:
                response.success = False
                response.message = (
                    f"Added to buffer but calibration failed: {self.last_solve_status}"
                )
                response.buffer_size = buffer_size
                self.get_logger().error(response.message)

        return response

    def clear_buffer_callback(self, request, response):
        """Service callback: Clear buffer and stop publishing."""
        with self.lock:
            buffer_size = len(self.detection_buffer)
            self.detection_buffer.clear()
            self.publishing_enabled = False
            self.last_transform = None
            self.last_solve_status = "Buffer cleared"
            self.total_correspondences = 0

        response.success = True
        response.message = f"Cleared {buffer_size} detection pairs from buffer"
        self.get_logger().info(response.message)

        return response

    def get_status_callback(self, request, response):
        """Service callback: Return buffer status."""
        with self.lock:
            response.buffer_size = len(self.detection_buffer)
            response.total_correspondences = self.total_correspondences
            response.is_publishing = self.publishing_enabled
            response.last_solve_status = self.last_solve_status

        return response

    def list_buffer_callback(self, request, response):
        """Service callback: List all detection pairs with details."""
        with self.lock:
            buffer_size = len(self.detection_buffer)
            aruco_counts = []
            board_counts = []
            timestamps_sec = []
            timestamps_nanosec = []

            for aruco_msg, board_msg in self.detection_buffer:
                aruco_counts.append(len(aruco_msg.detections))
                board_counts.append(len(board_msg.detections))
                # Use ArUco message timestamp (both should be synchronized)
                timestamps_sec.append(aruco_msg.header.stamp.sec)
                timestamps_nanosec.append(aruco_msg.header.stamp.nanosec)

        response.success = True
        response.message = f"Buffer contains {buffer_size} detection pairs"
        response.buffer_size = buffer_size
        response.aruco_counts = aruco_counts
        response.board_counts = board_counts
        response.timestamps_sec = timestamps_sec
        response.timestamps_nanosec = timestamps_nanosec

        self.get_logger().debug(
            f"Listed buffer: {buffer_size} pairs, "
            f"ArUco counts: {aruco_counts}, Board counts: {board_counts}"
        )

        return response

    def remove_detection_callback(self, request, response):
        """Service callback: Remove detection pair by index."""
        index = request.index

        with self.lock:
            buffer_size = len(self.detection_buffer)

            if index < 0 or index >= buffer_size:
                response.success = False
                response.message = (
                    f"Invalid index {index}. Buffer size is {buffer_size}"
                )
                response.buffer_size = buffer_size
                self.get_logger().error(response.message)
                return response

            # Remove the detection pair at the specified index
            removed_aruco, removed_board = self.detection_buffer.pop(index)
            new_buffer_size = len(self.detection_buffer)

        self.get_logger().info(
            f"Removed detection pair at index {index} "
            f"({len(removed_aruco.detections)} ArUco, {len(removed_board.detections)} boards). "
            f"New buffer size: {new_buffer_size}"
        )

        # Re-solve calibration if buffer is not empty
        if new_buffer_size > 0:
            success = self._solve_from_buffer()
            if success:
                response.success = True
                response.message = (
                    f"Removed detection at index {index} and re-solved calibration successfully "
                    f"({self.total_correspondences} correspondences from {new_buffer_size} poses)"
                )
            else:
                response.success = True  # Removal succeeded even if solve failed
                response.message = f"Removed detection at index {index} but calibration failed: {self.last_solve_status}"
        else:
            # Buffer is now empty, stop publishing
            with self.lock:
                self.publishing_enabled = False
                self.last_transform = None
                self.last_solve_status = "Buffer empty after removal"
                self.total_correspondences = 0

            response.success = True
            response.message = f"Removed last detection pair. Buffer is now empty."

        response.buffer_size = new_buffer_size
        self.get_logger().info(response.message)

        return response

    def _solve_from_buffer(self) -> bool:
        """
        Solve extrinsic calibration using all buffered detection pairs.

        This accumulates 3D-2D correspondences from all buffered poses
        and solves a single PnP problem for optimal accuracy.
        """
        with self.lock:
            buffer_size = len(self.detection_buffer)

        if buffer_size == 0:
            self.last_solve_status = "Empty buffer"
            self.get_logger().error("Cannot solve with empty buffer")
            return False

        # Check minimum poses requirement for multi-pose calibration
        if buffer_size < self.min_poses_required:
            self.last_solve_status = (
                f"Insufficient poses: {buffer_size}/{self.min_poses_required} required"
            )
            self.get_logger().warn(
                f"Buffered {buffer_size} pose(s), need {self.min_poses_required} minimum. "
                f"Add {self.min_poses_required - buffer_size} more pose(s) to start calibration."
            )
            return False

        self.get_logger().info(
            f"\n{'#'*80}\n"
            f"{'#'*80}\n"
            f"  SOLVING MULTI-POSE CALIBRATION FROM {buffer_size} BUFFERED POSES\n"
            f"{'#'*80}\n"
            f"{'#'*80}\n"
        )

        # Accumulate all correspondences from buffer
        all_object_points = []
        all_image_points = []

        for pose_idx, (aruco_msg, board_msg) in enumerate(self.detection_buffer, 1):
            self.get_logger().info(
                f"\n{'-'*80}\n"
                f"Processing Pose #{pose_idx}/{buffer_size}\n"
                f"{'-'*80}"
            )
            
            # Convert messages to internal format
            aruco_markers = self._detection2d_to_aruco_markers(aruco_msg)
            board_detection = self._detection3d_to_board_detection(
                board_msg.detections[0]
            )

            # Create correspondences for this pose
            object_points, image_points = self._create_point_correspondences(
                aruco_markers, board_detection
            )

            self.get_logger().info(
                f"Pose #{pose_idx}: Added {len(object_points)} correspondences"
            )

            all_object_points.extend(object_points)
            all_image_points.extend(image_points)

        # Convert to numpy arrays
        all_object_points = np.array(all_object_points, dtype=np.float32)
        all_image_points = np.array(all_image_points, dtype=np.float32)

        num_correspondences = len(all_object_points)

        if num_correspondences < 4:
            self.last_solve_status = (
                f"Insufficient correspondences: {num_correspondences} < 4"
            )
            self.get_logger().error(self.last_solve_status)
            return False

        self.get_logger().info(
            f"Accumulated {num_correspondences} correspondences from {buffer_size} poses"
        )

        # Solve PnP
        self.get_logger().info(
            f"\n{'='*80}\n"
            f"Solving PnP with {num_correspondences} total correspondences...\n"
            f"{'='*80}"
        )
        success, rvec, tvec = self._solve_pnp(all_object_points, all_image_points)

        if not success:
            self.last_solve_status = "PnP solver failed"
            self.get_logger().error(self.last_solve_status)
            return False

        # Convert rvec to rotation matrix for logging
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        euler_angles = R.from_matrix(rotation_matrix).as_euler('xyz', degrees=True)

        # Create transform message
        transform_msg = self._create_transform_message(rvec, tvec)

        # Store result and enable publishing
        with self.lock:
            self.last_transform = transform_msg
            self.publishing_enabled = True
            self.total_correspondences = num_correspondences
            self.last_solve_status = "Calibration successful"

        self.get_logger().info(
            f"\n{'#'*80}\n"
            f"{'#'*80}\n"
            f"  CALIBRATION SOLVED SUCCESSFULLY!\n"
            f"{'#'*80}\n"
            f"  Poses: {buffer_size}\n"
            f"  Correspondences: {num_correspondences}\n"
            f"\n"
            f"  Extrinsic Transform (LiDAR → Camera):\n"
            f"  -------------------------------------\n"
            f"  Translation (m):\n"
            f"    x: {tvec.flatten()[0]:+.6f}\n"
            f"    y: {tvec.flatten()[1]:+.6f}\n"
            f"    z: {tvec.flatten()[2]:+.6f}\n"
            f"\n"
            f"  Rotation (Rodrigues vector):\n"
            f"    rx: {rvec.flatten()[0]:+.6f}\n"
            f"    ry: {rvec.flatten()[1]:+.6f}\n"
            f"    rz: {rvec.flatten()[2]:+.6f}\n"
            f"\n"
            f"  Rotation (Euler angles XYZ, degrees):\n"
            f"    Roll:  {euler_angles[0]:+.3f}°\n"
            f"    Pitch: {euler_angles[1]:+.3f}°\n"
            f"    Yaw:   {euler_angles[2]:+.3f}°\n"
            f"\n"
            f"  Rotation Matrix:\n"
            f"    [{rotation_matrix[0,0]:+.6f}, {rotation_matrix[0,1]:+.6f}, {rotation_matrix[0,2]:+.6f}]\n"
            f"    [{rotation_matrix[1,0]:+.6f}, {rotation_matrix[1,1]:+.6f}, {rotation_matrix[1,2]:+.6f}]\n"
            f"    [{rotation_matrix[2,0]:+.6f}, {rotation_matrix[2,1]:+.6f}, {rotation_matrix[2,2]:+.6f}]\n"
            f"\n"
            f"  Quaternion (x, y, z, w):\n"
            f"    ({transform_msg.transform.rotation.x:+.6f}, "
            f"{transform_msg.transform.rotation.y:+.6f}, "
            f"{transform_msg.transform.rotation.z:+.6f}, "
            f"{transform_msg.transform.rotation.w:+.6f})\n"
            f"{'#'*80}\n"
            f"{'#'*80}\n"
        )

        return True

    def _detection2d_to_aruco_markers(
        self, detection_msg: Detection2DArray
    ) -> List[ArUcoMarker]:
        """Convert ROS Detection2DArray to ArUcoMarker objects."""
        markers = []

        for detection in detection_msg.detections:
            bbox = detection.bbox
            center_x = bbox.center.position.x
            center_y = bbox.center.position.y
            size_x = bbox.size_x
            size_y = bbox.size_y

            # Convert bounding box to 4 corner points
            corners = [
                (center_x - size_x / 2.0, center_y - size_y / 2.0),  # Top-left
                (center_x + size_x / 2.0, center_y - size_y / 2.0),  # Top-right
                (center_x + size_x / 2.0, center_y + size_y / 2.0),  # Bottom-right
                (center_x - size_x / 2.0, center_y + size_y / 2.0),  # Bottom-left
            ]

            marker_id = detection.id if hasattr(detection, "id") else 0

            markers.append(
                ArUcoMarker(id=marker_id, corners=corners, center=(center_x, center_y))
            )

        return markers

    def _detection3d_to_board_detection(self, detection) -> BoardDetection:
        """Convert ROS Detection3D to BoardDetection object."""
        if not detection.results:
            raise ValueError("No detection results available")

        pose = detection.results[0].pose.pose
        return BoardDetection(
            position=(pose.position.x, pose.position.y, pose.position.z),
            orientation=(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ),
        )

    def _load_aruco_pattern_config(self, config_file_path: str) -> dict:
        """Load ArUco pattern configuration from JSON5 file."""
        if not config_file_path:
            raise ValueError("aruco_config_file parameter is required")

        self.get_logger().info(f"Loading ArUco pattern config from: {config_file_path}")

        import json5

        with open(config_file_path, "r") as f:
            config = json5.load(f)

        self.get_logger().info(
            f"Loaded ArUco config: {config['num_squares_per_side']}x{config['num_squares_per_side']} grid, "
            f"board_size={config['board_size']}, "
            f"marker IDs={config['marker_ids']}"
        )

        return config

    def _parse_dimension(self, dim_str: str) -> float:
        """Parse dimension string like '500mm' or '10mm' to meters."""
        if dim_str.endswith("mm"):
            return float(dim_str[:-2]) / 1000.0
        elif dim_str.endswith("m"):
            return float(dim_str[:-1])
        else:
            return float(dim_str)

    def _compute_multi_marker_corners(
        self,
    ) -> Dict[int, List[Tuple[float, float, float]]]:
        """Compute 3D corner positions for all ArUco markers in board frame."""
        config = self.aruco_pattern_config

        board_size = self._parse_dimension(config["board_size"])
        board_border_size = self._parse_dimension(config["board_border_size"])
        marker_square_size_ratio = config["marker_square_size_ratio"]
        num_squares = config["num_squares_per_side"]
        marker_ids = config["marker_ids"]

        square_size = (board_size - 2.0 * board_border_size) / num_squares
        marker_size = square_size * marker_square_size_ratio
        marker_border = (square_size - marker_size) / 2.0

        self.get_logger().debug(
            f"Board geometry: square_size={square_size*1000:.1f}mm, "
            f"marker_size={marker_size*1000:.1f}mm, "
            f"marker_border={marker_border*1000:.1f}mm"
        )

        def make_corners(
            base_x: float, base_y: float
        ) -> List[Tuple[float, float, float]]:
            """Create 4 corner points for a marker in board-local coordinates."""
            bottom = (base_x, base_y, 0.0)
            left = (base_x + marker_size, base_y, 0.0)
            right = (base_x, base_y + marker_size, 0.0)
            top = (base_x + marker_size, base_y + marker_size, 0.0)
            return [right, top, left, bottom]

        origin_x = board_border_size + marker_border
        origin_y = board_border_size + marker_border

        marker_corners = {}
        marker_corners[marker_ids[0]] = make_corners(origin_x, origin_y)
        marker_corners[marker_ids[1]] = make_corners(origin_x + square_size, origin_y)
        marker_corners[marker_ids[2]] = make_corners(origin_x, origin_y + square_size)
        marker_corners[marker_ids[3]] = make_corners(
            origin_x + square_size, origin_y + square_size
        )

        self.get_logger().debug(
            f"Computed corners for {len(marker_corners)} markers in board frame"
        )

        return marker_corners

    def _create_point_correspondences(
        self, aruco_markers: List[ArUcoMarker], board_detection: BoardDetection
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Create 3D-2D point correspondences for PnP solving."""
        object_points = []
        image_points = []

        # Enhanced debug logging for board detection
        self.get_logger().info(
            f"\n{'='*70}\n"
            f"Creating Point Correspondences\n"
            f"{'='*70}\n"
            f"Board Detection (LiDAR frame):\n"
            f"  Position (x, y, z): ({board_detection.position[0]:.4f}, "
            f"{board_detection.position[1]:.4f}, {board_detection.position[2]:.4f}) m\n"
            f"  Orientation (quat x,y,z,w): ({board_detection.orientation[0]:.4f}, "
            f"{board_detection.orientation[1]:.4f}, {board_detection.orientation[2]:.4f}, "
            f"{board_detection.orientation[3]:.4f})\n"
            f"ArUco Markers Detected: {len(aruco_markers)}"
        )

        board_rotation = (
            R.from_quat(board_detection.orientation).as_matrix().astype(np.float32)
        )
        board_position = np.array(board_detection.position, dtype=np.float32)

        # Log rotation matrix
        self.get_logger().info(
            f"Board Rotation Matrix:\n{board_rotation}"
        )

        board_frame_corners = self._compute_multi_marker_corners()

        for marker in aruco_markers:
            marker_id_str = marker.id
            if isinstance(marker_id_str, str) and marker_id_str.startswith("aruco_"):
                marker_id = int(marker_id_str.split("_")[1])
            else:
                marker_id = (
                    int(marker_id_str)
                    if isinstance(marker_id_str, str)
                    else marker_id_str
                )

            if marker_id not in board_frame_corners:
                self.get_logger().warn(
                    f"Marker ID {marker_id} not found in ArUco pattern config, skipping"
                )
                continue

            local_corners = np.array(board_frame_corners[marker_id], dtype=np.float32)
            world_corners = (board_rotation @ local_corners.T).T + board_position
            
            image_corners = np.array(marker.corners, dtype=np.float32)
            
            # Detailed logging for each marker
            self.get_logger().info(
                f"\nMarker ID {marker_id}:\n"
                f"  Image Corners (pixels):\n"
                f"    Corner 0: ({image_corners[0][0]:.2f}, {image_corners[0][1]:.2f})\n"
                f"    Corner 1: ({image_corners[1][0]:.2f}, {image_corners[1][1]:.2f})\n"
                f"    Corner 2: ({image_corners[2][0]:.2f}, {image_corners[2][1]:.2f})\n"
                f"    Corner 3: ({image_corners[3][0]:.2f}, {image_corners[3][1]:.2f})\n"
                f"  Board Frame (local) Corners:\n"
                f"    Corner 0: ({local_corners[0][0]:.4f}, {local_corners[0][1]:.4f}, {local_corners[0][2]:.4f})\n"
                f"    Corner 1: ({local_corners[1][0]:.4f}, {local_corners[1][1]:.4f}, {local_corners[1][2]:.4f})\n"
                f"    Corner 2: ({local_corners[2][0]:.4f}, {local_corners[2][1]:.4f}, {local_corners[2][2]:.4f})\n"
                f"    Corner 3: ({local_corners[3][0]:.4f}, {local_corners[3][1]:.4f}, {local_corners[3][2]:.4f})\n"
                f"  World Frame (LiDAR) Corners:\n"
                f"    Corner 0: ({world_corners[0][0]:.4f}, {world_corners[0][1]:.4f}, {world_corners[0][2]:.4f})\n"
                f"    Corner 1: ({world_corners[1][0]:.4f}, {world_corners[1][1]:.4f}, {world_corners[1][2]:.4f})\n"
                f"    Corner 2: ({world_corners[2][0]:.4f}, {world_corners[2][1]:.4f}, {world_corners[2][2]:.4f})\n"
                f"    Corner 3: ({world_corners[3][0]:.4f}, {world_corners[3][1]:.4f}, {world_corners[3][2]:.4f})"
            )
            
            object_points.extend(world_corners)
            image_points.extend(image_corners)

        if len(object_points) == 0:
            self.get_logger().error("No valid marker correspondences found")
        else:
            self.get_logger().info(
                f"\n{'='*70}\n"
                f"Total Correspondences Created: {len(object_points)} points\n"
                f"{'='*70}\n"
            )

        return np.array(object_points, dtype=np.float32), np.array(
            image_points, dtype=np.float32
        )

    def _solve_pnp(
        self, object_points: np.ndarray, image_points: np.ndarray
    ) -> Tuple[bool, Optional[np.ndarray], Optional[np.ndarray]]:
        """Solve the Perspective-n-Point problem using OpenCV."""
        if len(object_points) < 4:
            self.get_logger().error("PnP requires at least 4 point correspondences")
            return False, None, None

        K = np.array(self.camera_info.k, dtype=np.float32).reshape(3, 3)
        dist_coeffs = np.zeros(5, dtype=np.float32)

        self.get_logger().info(
            f"Solving PnP with {len(object_points)} correspondences\n"
            f"Camera matrix K:\n{K}"
        )

        try:
            success, rvec, tvec = cv2.solvePnP(
                object_points,
                image_points,
                K,
                dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )

            if success:
                self.get_logger().info(
                    f"PnP solved successfully!\n"
                    f"Rotation vector: {rvec.flatten()}\n"
                    f"Translation vector: {tvec.flatten()}"
                )
                return True, rvec, tvec
            else:
                self.get_logger().error("PnP solver failed to converge")
                return False, None, None

        except cv2.error as e:
            self.get_logger().error(f"OpenCV PnP error: {e}")
            return False, None, None

    def _create_transform_message(
        self, rvec: np.ndarray, tvec: np.ndarray
    ) -> TransformStamped:
        """Create ROS TransformStamped message from PnP solution."""
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        quaternion = self._rotation_matrix_to_quaternion(rotation_matrix)

        transform_msg = TransformStamped()
        transform_msg.header = Header()
        transform_msg.header.stamp = self.get_clock().now().to_msg()
        transform_msg.header.frame_id = self.parent_frame
        transform_msg.child_frame_id = self.child_frame

        t = tvec.flatten()
        transform_msg.transform.translation = Vector3(
            x=float(t[0]), y=float(t[1]), z=float(t[2])
        )

        transform_msg.transform.rotation = Quaternion(
            x=float(quaternion[0]),
            y=float(quaternion[1]),
            z=float(quaternion[2]),
            w=float(quaternion[3]),
        )

        return transform_msg

    def _rotation_matrix_to_quaternion(self, rotation_matrix: np.ndarray) -> np.ndarray:
        """Convert 3x3 rotation matrix to quaternion."""
        rvec, _ = cv2.Rodrigues(rotation_matrix)
        angle = np.linalg.norm(rvec)

        if angle < 1e-6:
            return np.array([0.0, 0.0, 0.0, 1.0])

        axis = rvec.flatten() / angle
        half_angle = angle / 2.0

        qx = axis[0] * np.sin(half_angle)
        qy = axis[1] * np.sin(half_angle)
        qz = axis[2] * np.sin(half_angle)
        qw = np.cos(half_angle)

        return np.array([qx, qy, qz, qw])


def main(args=None):
    """Main function to run the advanced extrinsic solver node."""
    rclpy.init(args=args)

    node = AdvancedExtrinsicSolver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down advanced extrinsic solver")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
