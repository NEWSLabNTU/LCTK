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
5. Save/load detections to/from files
6. Manual transform adjustment
7. Axis arrow visualization

Workflow:
1. Play rosbag with calibration data
2. Call add_detection service to buffer good detection pairs
3. Repeat for multiple board poses
4. Node automatically re-solves using all buffered data
5. Continuous publishing of optimized extrinsic transform

Author: LCTK Team
License: MIT
"""

import json
import threading
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from conflux_py import DropPolicy, ROS2Synchronizer, SyncGroup
from geometry_msgs.msg import Point, Quaternion, TransformStamped, Vector3
from lctk_interfaces.srv import (
    AddDetectionToBuffer,
    AdjustTransform,
    ClearDetectionBuffer,
    DumpDetections,
    GetBufferStatus,
    GetPoseInfo,
    ListDetectionBuffer,
    LoadDetections,
    RemoveDetectionFromBuffer,
    ResetTransform,
)
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import ColorRGBA, Header
from vision_msgs.msg import Detection2DArray, Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray


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
    - dump_detections: Save all buffered detections to a file
    - load_detections: Load detections from a file
    - adjust_transform: Manually adjust the extrinsic transform

    Topics:
    - Subscribes: aruco_detections, calibration_board_detections, camera_info
    - Publishes: extrinsic_transform (continuous at 10Hz when solved)
    - Publishes: axis_markers (visualization of coordinate axes)
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
        self.declare_parameter("axis_length", 0.3)  # Length of axis arrows in meters
        self.declare_parameter("axis_diameter", 0.02)  # Diameter of axis arrows
        self.declare_parameter("use_best_effort_qos", True)
        self.declare_parameter("sync_tolerance_ms", 50.0)
        self.declare_parameter("sync_queue_size", 10)
        self.declare_parameter("sync_drop_policy", "reject_new")  # reject_new or drop_oldest

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
        self.axis_length = (
            self.get_parameter("axis_length").get_parameter_value().double_value
        )
        self.axis_diameter = (
            self.get_parameter("axis_diameter").get_parameter_value().double_value
        )
        use_best_effort_qos = (
            self.get_parameter("use_best_effort_qos").get_parameter_value().bool_value
        )
        sync_tolerance_ms = (
            self.get_parameter("sync_tolerance_ms").get_parameter_value().double_value
        )
        sync_queue_size = (
            self.get_parameter("sync_queue_size").get_parameter_value().integer_value
        )
        sync_drop_policy_str = (
            self.get_parameter("sync_drop_policy").get_parameter_value().string_value
        )
        sync_drop_policy = (
            DropPolicy.DROP_OLDEST if sync_drop_policy_str == "drop_oldest" else DropPolicy.REJECT_NEW
        )

        # Load ArUco pattern configuration
        self.aruco_pattern_config = self._load_aruco_pattern_config(aruco_config_file)

        # Detection buffer for multi-pose calibration
        self.detection_buffer: List[Tuple[Detection2DArray, Detection3DArray]] = []

        # Latest synchronized detection pair (cached for service calls)
        # Uses conflux_py for time synchronization
        self.latest_sync_pair: Optional[Tuple[Detection2DArray, Detection3DArray]] = None
        self.camera_info: Optional[CameraInfo] = None

        # Calibration state
        self.last_transform: Optional[TransformStamped] = None
        self.publishing_enabled = False
        self.last_solve_status = "No calibration performed yet"
        self.total_correspondences = 0

        # Pose state: solved (from PnP) and current (with manual adjustments)
        self.solved_rvec: Optional[np.ndarray] = None
        self.solved_tvec: Optional[np.ndarray] = None
        self.current_rvec: Optional[np.ndarray] = None
        self.current_tvec: Optional[np.ndarray] = None

        # Thread safety
        self.lock = threading.Lock()

        # QoS profile configuration based on mode:
        # - BEST_EFFORT (realtime): Low latency, may drop messages
        # - RELIABLE (offline): No message drops, suitable for rosbag playback
        reliability = ReliabilityPolicy.BEST_EFFORT if use_best_effort_qos else ReliabilityPolicy.RELIABLE
        qos_profile = QoSProfile(
            reliability=reliability,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.get_logger().info(
            f"Using {'BEST_EFFORT' if use_best_effort_qos else 'RELIABLE'} QoS"
        )

        # Publishers
        self.transform_publisher = self.create_publisher(
            TransformStamped, "extrinsic_transform", qos_profile
        )

        # Axis marker publisher for visualization
        self.axis_marker_publisher = self.create_publisher(
            MarkerArray, "axis_markers", qos_profile
        )

        # Publishing timer (10Hz continuous publishing when enabled)
        self.publishing_timer = self.create_timer(
            1.0 / publishing_rate, self._publishing_timer_callback
        )

        # Create synchronizer for ArUco and board detections
        # This ensures we only cache detection pairs that are time-synchronized
        self.get_logger().info(
            f"Using Conflux synchronization (window={int(sync_tolerance_ms)}ms, buffer={sync_queue_size})"
        )
        self.sync = ROS2Synchronizer(
            self,
            window_size_ms=int(sync_tolerance_ms) if sync_tolerance_ms > 0 else None,
            buffer_size=sync_queue_size,
            drop_policy=sync_drop_policy,
            qos=qos_profile,
        )
        self.sync.add_subscription(Detection2DArray, "aruco_detections")
        self.sync.add_subscription(Detection3DArray, "calibration_board_detections")

        @self.sync.on_synchronized
        def on_sync(group: SyncGroup):
            self._handle_synchronized_detections(group)

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

        self.dump_detections_service = self.create_service(
            DumpDetections,
            "~/dump_detections",
            self.dump_detections_callback,
        )

        self.load_detections_service = self.create_service(
            LoadDetections,
            "~/load_detections",
            self.load_detections_callback,
        )

        self.adjust_transform_service = self.create_service(
            AdjustTransform,
            "~/adjust_transform",
            self.adjust_transform_callback,
        )

        self.reset_transform_service = self.create_service(
            ResetTransform,
            "~/reset_transform",
            self.reset_transform_callback,
        )

        self.get_pose_info_service = self.create_service(
            GetPoseInfo,
            "~/get_pose_info",
            self.get_pose_info_callback,
        )

        self.get_logger().info(
            f"Advanced Extrinsic Solver initialized\n"
            f"Mode: Multi-pose buffered calibration\n"
            f"Using conflux_py for time-synchronized detection pairs\n"
            f"Minimum poses required: {self.min_poses_required}\n"
            f"Subscribing to: aruco_detections, calibration_board_detections, {camera_info_topic}\n"
            f"Publishing to: extrinsic_transform (at {publishing_rate}Hz when enabled), axis_markers\n"
            f"Transform: {self.parent_frame} -> {self.child_frame}\n"
            f"Services: ~/add_detection, ~/clear_buffer, ~/get_status, ~/list_buffer, ~/remove_detection, ~/dump_detections, ~/load_detections, ~/adjust_transform, ~/reset_transform, ~/get_pose_info"
        )

    def camera_info_callback(self, msg: CameraInfo):
        """Cache camera info for PnP solving."""
        with self.lock:
            self.camera_info = msg
            self.get_logger().debug(f"Camera info received: {msg.width}x{msg.height}")

    def _handle_synchronized_detections(self, group: SyncGroup):
        """
        Handle synchronized ArUco and board detections.

        This callback is invoked by conflux_py when both ArUco and board
        detections are available within the time window. The synchronized
        pair is cached for use by the add_detection service.
        """
        # Debug: log available keys in the sync group
        available_keys = group.topics()
        self.get_logger().debug(f"SyncGroup keys: {available_keys}")

        # Safely get messages with error handling
        aruco_msg = group.get("aruco_detections")
        board_msg = group.get("calibration_board_detections")

        if aruco_msg is None or board_msg is None:
            self.get_logger().warn(
                f"Incomplete sync group received. Available keys: {available_keys}, "
                f"aruco={aruco_msg is not None}, board={board_msg is not None}"
            )
            return

        self.get_logger().debug(
            f"Synchronized detection pair at t={group.timestamp:.6f}s: "
            f"{len(aruco_msg.detections)} ArUco markers, "
            f"{len(board_msg.detections)} boards"
        )

        # Only cache non-empty detection pairs
        if aruco_msg.detections and board_msg.detections:
            with self.lock:
                self.latest_sync_pair = (aruco_msg, board_msg)
        else:
            if not aruco_msg.detections:
                self.get_logger().debug("Ignoring sync group with empty ArUco detection")
            if not board_msg.detections:
                self.get_logger().warn("Ignoring sync group with empty board detection")

    def _publishing_timer_callback(self):
        """Continuously publish the last solved transform when enabled."""
        with self.lock:
            if self.publishing_enabled and self.last_transform:
                # Update timestamp to current time
                self.last_transform.header.stamp = self.get_clock().now().to_msg()
                self.transform_publisher.publish(self.last_transform)

                # Also publish axis markers
                self._publish_axis_markers()

    def _publish_axis_markers(self):
        """Publish axis arrow markers for transform visualization."""
        if self.last_transform is None:
            return

        tf = self.last_transform.transform
        markers = MarkerArray()

        # Get position
        pos = np.array([tf.translation.x, tf.translation.y, tf.translation.z])

        # Get rotation matrix from quaternion
        quat = [tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w]
        rot = R.from_quat(quat)
        rot_matrix = rot.as_matrix()

        # Colors for X (red), Y (green), Z (blue) axes
        colors = [
            ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),  # X - Red
            ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),  # Y - Green
            ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0),  # Z - Blue
        ]

        for i, (axis_col, color) in enumerate(zip(rot_matrix.T, colors)):
            marker = Marker()
            marker.header.frame_id = self.parent_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "extrinsic_axes"
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD

            # Arrow from origin to axis tip
            start = Point(x=pos[0], y=pos[1], z=pos[2])
            end_pos = pos + axis_col * self.axis_length
            end = Point(x=end_pos[0], y=end_pos[1], z=end_pos[2])
            marker.points = [start, end]

            # Arrow dimensions
            marker.scale.x = self.axis_diameter  # Shaft diameter
            marker.scale.y = self.axis_diameter * 1.5  # Head diameter
            marker.scale.z = self.axis_length * 0.15  # Head length

            marker.color = color
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 200000000  # 200ms

            markers.markers.append(marker)

        self.axis_marker_publisher.publish(markers)

    def add_detection_callback(self, request, response):
        """
        Service callback: Add latest synchronized detection pair to buffer and re-solve.

        This triggers a complete re-solve using all buffered detections,
        potentially improving calibration accuracy with more data.
        Note: Only synchronized detection pairs (from conflux_py) are used.
        """
        with self.lock:
            sync_pair = self.latest_sync_pair

        # Validate prerequisites
        if not self.camera_info:
            response.success = False
            response.message = "No camera info available"
            response.buffer_size = len(self.detection_buffer)
            self.get_logger().error(response.message)
            return response

        if not sync_pair:
            response.success = False
            response.message = "No synchronized detection pair available. Waiting for time-synchronized ArUco and board detections."
            response.buffer_size = len(self.detection_buffer)
            self.get_logger().error(response.message)
            return response

        aruco_msg, board_msg = sync_pair

        if not aruco_msg.detections or not board_msg.detections:
            response.success = False
            response.message = "Empty detection messages in synchronized pair"
            response.buffer_size = len(self.detection_buffer)
            self.get_logger().error(response.message)
            return response

        # Add to buffer (no similarity check - allow multiple detections to average out)
        with self.lock:
            self.detection_buffer.append((aruco_msg, board_msg))
            buffer_size = len(self.detection_buffer)

        # Log successful addition
        board_pos = board_msg.detections[0].results[0].pose.pose.position
        self.get_logger().info(
            f"Added detection pair #{buffer_size} to buffer\n"
            f"  Board position: ({board_pos.x:.4f}, {board_pos.y:.4f}, {board_pos.z:.4f})"
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
            self.latest_sync_pair = None
            self.publishing_enabled = False
            self.last_transform = None
            self.last_solve_status = "Buffer cleared"
            self.total_correspondences = 0
            self.solved_rvec = None
            self.solved_tvec = None
            self.current_rvec = None
            self.current_tvec = None

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
            response.message = "Removed last detection pair. Buffer is now empty."

        response.buffer_size = new_buffer_size
        self.get_logger().info(response.message)

        return response

    def dump_detections_callback(self, request, response):
        """Service callback: Save all buffered detections and manual adjustments to a JSON file."""
        file_path = request.file_path

        with self.lock:
            buffer_size = len(self.detection_buffer)

            if buffer_size == 0 and self.current_rvec is None:
                response.success = False
                response.message = "Buffer is empty and no transform available, nothing to save"
                response.num_detections = 0
                return response

            # Serialize detections to JSON-compatible format
            detections_data = []
            for aruco_msg, board_msg in self.detection_buffer:
                detection_pair = {
                    "aruco": self._serialize_detection2d_array(aruco_msg),
                    "board": self._serialize_detection3d_array(board_msg),
                }
                detections_data.append(detection_pair)

            # Serialize manual adjustments (current transform)
            transform_data = None
            if self.current_rvec is not None and self.current_tvec is not None:
                transform_data = {
                    "rvec": self.current_rvec.flatten().tolist(),
                    "tvec": self.current_tvec.flatten().tolist(),
                }

        try:
            save_data = {
                "version": 2,  # Bumped version for new format with transform
                "num_detections": buffer_size,
                "detections": detections_data,
            }
            if transform_data:
                save_data["transform"] = transform_data

            with open(file_path, 'w') as f:
                json.dump(save_data, f, indent=2)

            msg_parts = [f"Saved {buffer_size} detection pairs"]
            if transform_data:
                msg_parts.append("with manual adjustments")
            msg_parts.append(f"to {file_path}")

            response.success = True
            response.message = " ".join(msg_parts)
            response.num_detections = buffer_size
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Failed to save detections: {str(e)}"
            response.num_detections = 0
            self.get_logger().error(response.message)

        return response

    def load_detections_callback(self, request, response):
        """Service callback: Load detections and manual adjustments from a JSON file."""
        file_path = request.file_path
        append = request.append

        try:
            with open(file_path, 'r') as f:
                data = json.load(f)

            version = data.get("version", 0)
            if version not in [1, 2]:
                response.success = False
                response.message = "Invalid or unsupported file format"
                response.num_detections = 0
                response.buffer_size = len(self.detection_buffer)
                return response

            loaded_detections = []
            for detection_pair in data.get("detections", []):
                aruco_msg = self._deserialize_detection2d_array(detection_pair["aruco"])
                board_msg = self._deserialize_detection3d_array(detection_pair["board"])
                loaded_detections.append((aruco_msg, board_msg))

            # Check for saved transform (version 2+)
            has_transform = "transform" in data and data["transform"] is not None
            loaded_rvec = None
            loaded_tvec = None
            if has_transform:
                loaded_rvec = np.array(data["transform"]["rvec"], dtype=np.float64).reshape(3, 1)
                loaded_tvec = np.array(data["transform"]["tvec"], dtype=np.float64).reshape(3, 1)

            with self.lock:
                if not append:
                    self.detection_buffer.clear()
                self.detection_buffer.extend(loaded_detections)
                buffer_size = len(self.detection_buffer)

                # If we have a saved transform, use it directly instead of re-solving
                if has_transform:
                    self.current_rvec = loaded_rvec
                    self.current_tvec = loaded_tvec
                    self.last_transform = self._create_transform_message(loaded_rvec, loaded_tvec)
                    self.publishing_enabled = True
                    self.last_solve_status = "Loaded from file (with manual adjustments)"
                    self.get_logger().info("Restored manual transform adjustments from file")
                elif buffer_size >= self.min_poses_required:
                    # No saved transform, re-solve from detections
                    self._solve_from_buffer()

            msg_parts = [f"Loaded {len(loaded_detections)} detection pairs"]
            if has_transform:
                msg_parts.append("with manual adjustments")
            msg_parts.append(f"from {file_path}")

            response.success = True
            response.message = " ".join(msg_parts)
            response.num_detections = len(loaded_detections)
            response.buffer_size = buffer_size
            self.get_logger().info(response.message)

        except FileNotFoundError:
            response.success = False
            response.message = f"File not found: {file_path}"
            response.num_detections = 0
            response.buffer_size = len(self.detection_buffer)
            self.get_logger().error(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Failed to load detections: {str(e)}"
            response.num_detections = 0
            response.buffer_size = len(self.detection_buffer)
            self.get_logger().error(response.message)

        return response

    def adjust_transform_callback(self, request, response):
        """Service callback: Manually adjust the extrinsic transform."""
        with self.lock:
            if self.current_rvec is None or self.current_tvec is None:
                response.success = False
                response.message = "No transform available to adjust. Solve calibration first."
                return response

            # Apply translation adjustment
            self.current_tvec[0, 0] += request.delta_x
            self.current_tvec[1, 0] += request.delta_y
            self.current_tvec[2, 0] += request.delta_z

            # Apply rotation adjustment (as Euler angle deltas in XYZ order)
            if request.delta_roll != 0 or request.delta_pitch != 0 or request.delta_yaw != 0:
                # Get current rotation matrix
                current_rot_matrix, _ = cv2.Rodrigues(self.current_rvec)
                current_rot = R.from_matrix(current_rot_matrix)

                # Create delta rotation from Euler angles
                delta_rot = R.from_euler('xyz', [request.delta_roll, request.delta_pitch, request.delta_yaw])

                # Apply delta rotation (delta * current)
                new_rot = delta_rot * current_rot
                new_rot_matrix = new_rot.as_matrix()

                # Convert back to Rodrigues vector
                self.current_rvec, _ = cv2.Rodrigues(new_rot_matrix)

            # Update transform message
            self.last_transform = self._create_transform_message(self.current_rvec, self.current_tvec)

            # Get updated Euler angles for logging
            rot_matrix, _ = cv2.Rodrigues(self.current_rvec)
            euler = R.from_matrix(rot_matrix).as_euler('xyz', degrees=True)

            response.success = True
            response.message = (
                f"Transform adjusted: t=({self.current_tvec[0,0]:.4f}, {self.current_tvec[1,0]:.4f}, {self.current_tvec[2,0]:.4f}), "
                f"rpy=({euler[0]:.2f}, {euler[1]:.2f}, {euler[2]:.2f}) deg"
            )
            self.get_logger().info(response.message)

        return response

    def reset_transform_callback(self, request, response):
        """Service callback: Reset manual adjustments and re-solve from buffered detections."""
        with self.lock:
            buffer_size = len(self.detection_buffer)

            if buffer_size < self.min_poses_required:
                response.success = False
                response.message = f"Cannot reset: need at least {self.min_poses_required} poses in buffer (have {buffer_size})"
                return response

        # Re-solve from buffer (this will reset current_rvec/current_tvec to the solved values)
        success = self._solve_from_buffer()

        if success:
            response.success = True
            response.message = f"Reset transform to solved values ({self.total_correspondences} correspondences from {buffer_size} poses)"
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = f"Reset failed: {self.last_solve_status}"
            self.get_logger().error(response.message)

        return response

    def get_pose_info_callback(self, request, response):
        """Service callback: Get detailed pose information."""
        with self.lock:
            if self.solved_rvec is None or self.current_rvec is None:
                response.has_pose = False
                return response

            response.has_pose = True

            # Get solved pose as Euler angles
            solved_rot_matrix, _ = cv2.Rodrigues(self.solved_rvec)
            solved_euler = R.from_matrix(solved_rot_matrix).as_euler('xyz')
            response.solved_x = float(self.solved_tvec[0, 0])
            response.solved_y = float(self.solved_tvec[1, 0])
            response.solved_z = float(self.solved_tvec[2, 0])
            response.solved_roll = float(solved_euler[0])
            response.solved_pitch = float(solved_euler[1])
            response.solved_yaw = float(solved_euler[2])

            # Get current pose as Euler angles
            current_rot_matrix, _ = cv2.Rodrigues(self.current_rvec)
            current_euler = R.from_matrix(current_rot_matrix).as_euler('xyz')
            response.current_x = float(self.current_tvec[0, 0])
            response.current_y = float(self.current_tvec[1, 0])
            response.current_z = float(self.current_tvec[2, 0])
            response.current_roll = float(current_euler[0])
            response.current_pitch = float(current_euler[1])
            response.current_yaw = float(current_euler[2])

            # Compute adjustments (delta)
            response.adjust_x = response.current_x - response.solved_x
            response.adjust_y = response.current_y - response.solved_y
            response.adjust_z = response.current_z - response.solved_z
            response.adjust_roll = response.current_roll - response.solved_roll
            response.adjust_pitch = response.current_pitch - response.solved_pitch
            response.adjust_yaw = response.current_yaw - response.solved_yaw

        return response

    def _serialize_detection2d_array(self, msg: Detection2DArray) -> dict:
        """Serialize Detection2DArray to JSON-compatible dict."""
        return {
            "header": {
                "stamp": {"sec": msg.header.stamp.sec, "nanosec": msg.header.stamp.nanosec},
                "frame_id": msg.header.frame_id,
            },
            "detections": [
                {
                    "id": d.id if hasattr(d, 'id') else "",
                    "bbox": {
                        "center": {"x": d.bbox.center.position.x, "y": d.bbox.center.position.y},
                        "size_x": d.bbox.size_x,
                        "size_y": d.bbox.size_y,
                    },
                }
                for d in msg.detections
            ],
        }

    def _serialize_detection3d_array(self, msg: Detection3DArray) -> dict:
        """Serialize Detection3DArray to JSON-compatible dict."""
        return {
            "header": {
                "stamp": {"sec": msg.header.stamp.sec, "nanosec": msg.header.stamp.nanosec},
                "frame_id": msg.header.frame_id,
            },
            "detections": [
                {
                    "results": [
                        {
                            "pose": {
                                "position": {
                                    "x": r.pose.pose.position.x,
                                    "y": r.pose.pose.position.y,
                                    "z": r.pose.pose.position.z,
                                },
                                "orientation": {
                                    "x": r.pose.pose.orientation.x,
                                    "y": r.pose.pose.orientation.y,
                                    "z": r.pose.pose.orientation.z,
                                    "w": r.pose.pose.orientation.w,
                                },
                            }
                        }
                        for r in d.results
                    ]
                }
                for d in msg.detections
            ],
        }

    def _deserialize_detection2d_array(self, data: dict) -> Detection2DArray:
        """Deserialize Detection2DArray from JSON-compatible dict."""
        from vision_msgs.msg import Detection2D, BoundingBox2D

        msg = Detection2DArray()
        msg.header.stamp.sec = data["header"]["stamp"]["sec"]
        msg.header.stamp.nanosec = data["header"]["stamp"]["nanosec"]
        msg.header.frame_id = data["header"]["frame_id"]

        for d_data in data["detections"]:
            detection = Detection2D()
            if "id" in d_data:
                detection.id = d_data["id"]
            detection.bbox = BoundingBox2D()
            detection.bbox.center.position.x = d_data["bbox"]["center"]["x"]
            detection.bbox.center.position.y = d_data["bbox"]["center"]["y"]
            detection.bbox.size_x = d_data["bbox"]["size_x"]
            detection.bbox.size_y = d_data["bbox"]["size_y"]
            msg.detections.append(detection)

        return msg

    def _deserialize_detection3d_array(self, data: dict) -> Detection3DArray:
        """Deserialize Detection3DArray from JSON-compatible dict."""
        from vision_msgs.msg import Detection3D, ObjectHypothesisWithPose
        from geometry_msgs.msg import PoseWithCovariance, Pose

        msg = Detection3DArray()
        msg.header.stamp.sec = data["header"]["stamp"]["sec"]
        msg.header.stamp.nanosec = data["header"]["stamp"]["nanosec"]
        msg.header.frame_id = data["header"]["frame_id"]

        for d_data in data["detections"]:
            detection = Detection3D()
            for r_data in d_data["results"]:
                result = ObjectHypothesisWithPose()
                result.pose = PoseWithCovariance()
                result.pose.pose = Pose()
                result.pose.pose.position.x = r_data["pose"]["position"]["x"]
                result.pose.pose.position.y = r_data["pose"]["position"]["y"]
                result.pose.pose.position.z = r_data["pose"]["position"]["z"]
                result.pose.pose.orientation.x = r_data["pose"]["orientation"]["x"]
                result.pose.pose.orientation.y = r_data["pose"]["orientation"]["y"]
                result.pose.pose.orientation.z = r_data["pose"]["orientation"]["z"]
                result.pose.pose.orientation.w = r_data["pose"]["orientation"]["w"]
                detection.results.append(result)
            msg.detections.append(detection)

        return msg

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
            f"  SOLVING MULTI-POSE CALIBRATION FROM {buffer_size} BUFFERED POSES\n"
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

        # Store solved and current rvec/tvec
        with self.lock:
            self.solved_rvec = rvec.copy()
            self.solved_tvec = tvec.copy()
            self.current_rvec = rvec.copy()
            self.current_tvec = tvec.copy()

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
            f"  CALIBRATION SOLVED SUCCESSFULLY!\n"
            f"{'#'*80}\n"
            f"  Poses: {buffer_size}\n"
            f"  Correspondences: {num_correspondences}\n"
            f"\n"
            f"  Extrinsic Transform (LiDAR -> Camera):\n"
            f"  -------------------------------------\n"
            f"  Translation (m):\n"
            f"    x: {tvec.flatten()[0]:+.6f}\n"
            f"    y: {tvec.flatten()[1]:+.6f}\n"
            f"    z: {tvec.flatten()[2]:+.6f}\n"
            f"\n"
            f"  Rotation (Euler angles XYZ, degrees):\n"
            f"    Roll:  {euler_angles[0]:+.3f}\n"
            f"    Pitch: {euler_angles[1]:+.3f}\n"
            f"    Yaw:   {euler_angles[2]:+.3f}\n"
            f"\n"
            f"  Quaternion (x, y, z, w):\n"
            f"    ({transform_msg.transform.rotation.x:+.6f}, "
            f"{transform_msg.transform.rotation.y:+.6f}, "
            f"{transform_msg.transform.rotation.z:+.6f}, "
            f"{transform_msg.transform.rotation.w:+.6f})\n"
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

        board_rotation = (
            R.from_quat(board_detection.orientation).as_matrix().astype(np.float32)
        )
        board_position = np.array(board_detection.position, dtype=np.float32)

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

            object_points.extend(world_corners)
            image_points.extend(image_corners)

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
            f"Solving PnP with {len(object_points)} correspondences"
        )

        try:
            success, rvec, tvec = cv2.solvePnP(
                object_points,
                image_points,
                K,
                dist_coeffs,
                flags=cv2.SOLVEPNP_SQPNP,
            )

            if success:
                self.get_logger().info(
                    f"PnP solved successfully!\n"
                    f"Translation: {tvec.flatten()}"
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
    import time

    rclpy.init(args=args)

    node = AdvancedExtrinsicSolver()

    # Brief delay to allow DDS discovery to complete before spinning
    # This helps avoid race conditions with entity creation
    time.sleep(0.1)

    try:
        # Use explicit executor for better control
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)

        try:
            executor.spin()
        finally:
            executor.shutdown()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down advanced extrinsic solver")
    except Exception as e:
        # Handle RCLError and other exceptions gracefully
        node.get_logger().error(f"Error during spin: {e}")
    finally:
        # Log final synchronization statistics
        try:
            stats = node.sync.statistics
            node.get_logger().info(
                f"Final sync statistics: "
                f"received={stats.total_received()}, "
                f"rejected={stats.total_rejected()}, "
                f"groups={stats.groups_synchronized}, "
                f"rejection_rate={stats.rejection_rate():.1%}"
            )
            for topic in stats.messages_received:
                topic_rate = stats.rejection_rate(topic)
                node.get_logger().info(
                    f"  {topic}: received={stats.messages_received[topic]}, "
                    f"rejected={stats.messages_rejected[topic]}, "
                    f"rejection_rate={topic_rate:.1%}"
                )
        except Exception:
            pass  # Ignore errors during statistics logging

        try:
            node.destroy_node()
        except Exception:
            pass  # Ignore errors during cleanup
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass  # Ignore errors if context is already invalid


if __name__ == "__main__":
    main()
