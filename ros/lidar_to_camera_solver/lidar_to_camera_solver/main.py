#!/usr/bin/env python3
"""
LiDAR-to-Camera Extrinsic Calibration Node

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
import sys
import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from conflux_py import DropPolicy, ROS2Synchronizer, SyncGroup
from lidar_to_camera_solver.board_geometry import (
    BOARD_FRAME_CONVENTION,
    BOARD_FRAME_CONVENTION_TOPIC,
    ArUcoMarker,
    compute_multi_marker_corners,
    detection2d_to_aruco_markers,
    frame_convention_error,
    load_aruco_pattern_config,
    marker_geometry_summary,
    parse_dimension,
    rotation_matrix_to_quaternion,
)
from lidar_to_camera_solver.detection_format import (
    FORMAT_VERSION,
    deserialize_detection2d_array,
    deserialize_detection3d_array,
    format_version_error,
    serialize_detection2d_array,
    serialize_detection3d_array,
)
from geometry_msgs.msg import Point, Quaternion, TransformStamped, Vector3
from lctk_quality import build_report, distinct_placements
from lctk_quality.resampling import MIN_PLACEMENTS_FOR_SPREAD
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
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import ColorRGBA, Header, String
from vision_msgs.msg import Detection2DArray, Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray


@dataclass
class BoardDetection:
    """Represents a calibration board detection in 3D LiDAR coordinates."""

    position: Tuple[float, float, float]  # x, y, z in meters (LiDAR frame)
    orientation: Tuple[float, float, float, float]  # quaternion x, y, z, w

    # M-13: the board-pose covariance from the detector's ICP fit, 6x6 row-major in the ROS
    # PoseWithCovariance order [x, y, z, rx, ry, rz]. It is strongly ANISOTROPIC: interior LiDAR
    # returns constrain only the out-of-plane DoF, so a board with few edge/hole-rim returns is
    # tightly determined along its normal and nearly free within its own plane.
    #
    # This used to be discarded, and the solver treated every board pose as exact -- which is the
    # errors-in-variables bias: solvePnP assumes the 3D points are noiseless, while they are model
    # corners pushed through this very pose.
    covariance: Optional[np.ndarray] = (
        None  # 6x6, or None for a detector that does not publish it
    )

    @property
    def position_sigma_m(self) -> Optional[np.ndarray]:
        """Per-axis translation standard deviation, metres. None if no covariance was published."""
        if self.covariance is None:
            return None
        return np.sqrt(np.abs(np.diag(self.covariance)[:3]))

    @property
    def rotation_sigma_deg(self) -> Optional[np.ndarray]:
        """Per-axis rotation standard deviation, degrees."""
        if self.covariance is None:
            return None
        return np.degrees(np.sqrt(np.abs(np.diag(self.covariance)[3:])))


class LidarToCameraSolver(Node):
    """
    ROS 2 node for multi-pose LiDAR-camera extrinsic calibration.

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
        super().__init__("lidar_to_camera_solver")

        # Essential parameter declarations
        self.declare_parameter("parent_frame", "lidar")
        self.declare_parameter("child_frame", "camera")
        self.declare_parameter("camera_topic", "")
        self.declare_parameter("aruco_config_file", "")
        self.declare_parameter("debug_mode", True)
        self.declare_parameter("publishing_rate", 10.0)
        # H-07: this is a minimum number of buffered FRAMES before a solve is attempted. It is
        # NOT a measure of how well-constrained the calibration is -- twenty frames of a board that
        # never moved are one board placement and cannot determine the extrinsic. The constraint
        # that matters is the number of DISTINCT placements, which is measured and reported by
        # lctk_quality (H-09) rather than gated on here.
        self.declare_parameter("min_frames_required", 2)

        # H-07: pose-diversity gate. The PnP correspondences all lie on one 500 mm
        # coplanar ArUco patch, so a near-coplanar / single-depth buffer leaves a
        # rotation about the correspondence centroid almost unconstrained (the board
        # overlay stays glued while the background tilts). Gate the solve on the
        # GEOMETRY of the accumulated poses, not just their count.
        self.declare_parameter("min_normal_spread_deg", 20.0)
        self.declare_parameter("min_depth_range_m", 1.0)
        # Warn by default (so the sample single-placement demo still solves), refuse
        # only when explicitly enforced.
        self.declare_parameter("enforce_pose_diversity", False)
        self.declare_parameter("axis_length", 0.3)  # Length of axis arrows in meters
        self.declare_parameter("axis_diameter", 0.02)  # Diameter of axis arrows
        self.declare_parameter("use_best_effort_qos", True)
        self.declare_parameter("sync_tolerance_ms", 50.0)
        self.declare_parameter("sync_queue_size", 10)
        self.declare_parameter(
            "sync_drop_policy", "reject_new"
        )  # reject_new or drop_oldest

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
        self.min_frames_required = (
            self.get_parameter("min_frames_required")
            .get_parameter_value()
            .integer_value
        )
        self.min_normal_spread_deg = (
            self.get_parameter("min_normal_spread_deg")
            .get_parameter_value()
            .double_value
        )
        self.min_depth_range_m = (
            self.get_parameter("min_depth_range_m").get_parameter_value().double_value
        )
        self.enforce_pose_diversity = (
            self.get_parameter("enforce_pose_diversity")
            .get_parameter_value()
            .bool_value
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
            DropPolicy.DROP_OLDEST
            if sync_drop_policy_str == "drop_oldest"
            else DropPolicy.REJECT_NEW
        )

        # H-11: refuse to run against a detector that disagrees about the board frame,
        # BEFORE any geometry is loaded or any transform can be published.
        self.declare_parameter("frame_convention_timeout_s", 10.0)
        self._await_board_frame_convention(
            self.get_parameter("frame_convention_timeout_s")
            .get_parameter_value()
            .double_value
        )

        # Load ArUco pattern configuration
        self.aruco_pattern_config = self._load_aruco_pattern_config(aruco_config_file)

        # Detection buffer for multi-pose calibration
        self.detection_buffer: List[Tuple[Detection2DArray, Detection3DArray]] = []

        # Latest synchronized detection pair (cached for service calls)
        # Uses conflux_py for time synchronization
        self.latest_sync_pair: Optional[Tuple[Detection2DArray, Detection3DArray]] = (
            None
        )
        self.camera_info: Optional[CameraInfo] = None

        # Calibration state
        self.last_transform: Optional[TransformStamped] = None
        self.publishing_enabled = False
        self.last_solve_status = "No calibration performed yet"

        # H-09: quality report for the last solve (None until one succeeds), plus the per-frame
        # arrays it needs. Kept per-frame, not concatenated, because N is the number of DISTINCT
        # board placements -- buffering one placement a hundred times is duplication, not
        # information.
        self.last_quality = None
        # M-12: pose-granularity outlier rejection. On by default -- one bad pose silently
        # corrupting the extrinsic is the failure this exists to prevent -- but switchable,
        # because with very few poses the honest answer is to capture more, not to drop any.
        self.declare_parameter("reject_outlier_poses", True)
        self.reject_outlier_poses = (
            self.get_parameter("reject_outlier_poses").get_parameter_value().bool_value
        )
        self.declare_parameter("outlier_pose_mad_k", 4.0)
        self.outlier_pose_mad_k = (
            self.get_parameter("outlier_pose_mad_k").get_parameter_value().double_value
        )

        self._per_frame_object_points = []
        self._per_frame_image_points = []
        self._per_frame_board_poses = []
        # M-13: per-frame weight from the board-pose covariance. A pose whose ICP fit is loose
        # contributes less to the solve than one that is tight.
        self._per_frame_weights = []
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
        reliability = (
            ReliabilityPolicy.BEST_EFFORT
            if use_best_effort_qos
            else ReliabilityPolicy.RELIABLE
        )
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
            f"LiDAR-to-camera solver initialized\n"
            f"Mode: Multi-pose buffered calibration\n"
            f"Using conflux_py for time-synchronized detection pairs\n"
            f"Minimum frames before solving: {self.min_frames_required}\n"
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
                self.get_logger().debug(
                    "Ignoring sync group with empty ArUco detection"
                )
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

        # H-07: accept the frame, but tell the operator NOW whether it is a new board placement or
        # a duplicate of one already buffered. This is the only moment the feedback can change what
        # they do — by the time they read the solve log, they have already put the board down.
        #
        # Duplicates are still buffered (they do average down the per-frame noise), but they add no
        # geometry, and the pipeline used to count them as "poses" and report success.
        with self.lock:
            self.detection_buffer.append((aruco_msg, board_msg))
            buffer_size = len(self.detection_buffer)
            placements_before = self._count_placements(exclude_last=True)
            placements_after = self._count_placements()

        board_pos = board_msg.detections[0].results[0].pose.pose.position
        is_new_placement = placements_after > placements_before

        if is_new_placement:
            self.get_logger().info(
                f"Added detection #{buffer_size}: NEW board placement "
                f"#{placements_after} at "
                f"({board_pos.x:.2f}, {board_pos.y:.2f}, {board_pos.z:.2f}) m"
            )
        else:
            self.get_logger().warn(
                f"Added detection #{buffer_size}, but it is a DUPLICATE of a board placement "
                f"already buffered — still {placements_after} distinct placement(s).\n"
                "  Repeated frames of the same board placement average down the noise but add no "
                "geometry, and cannot constrain the extrinsic.\n"
                "  MOVE THE BOARD: a new distance, or a new yaw/pitch."
            )

        # Re-solve calibration from entire buffer
        success = self._solve_from_buffer()

        response.buffer_size = buffer_size

        if success:
            # H-07: the response is what the operator actually reads when they hit Add. It used to
            # say "solved calibration successfully (320 correspondences from 20 poses)" — and both
            # of those numbers are the exact lies H-09 disproved: 20 frames of one placement are
            # not 20 poses, and 320 correspondences are not 320 pieces of information. Report the
            # quality verdict instead, so the operator cannot collect a degenerate buffer while
            # being congratulated twenty times.
            response.success = True
            response.message = self.last_solve_status

            if self.last_quality is not None and self.last_quality.is_degenerate:
                self.get_logger().warn(response.message)
            else:
                self.get_logger().info(response.message)
        elif placements_after < MIN_PLACEMENTS_FOR_SPREAD:
            # Not an error — just not enough distinct geometry yet to say anything.
            response.success = True
            response.message = (
                f"Buffered: {buffer_size} frame(s), {placements_after} distinct placement(s). "
                f"Need {MIN_PLACEMENTS_FOR_SPREAD} distinct placements before the uncertainty can "
                "be estimated. Move the board and add more."
            )
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = (
                f"Added to buffer but calibration failed: {self.last_solve_status}"
            )
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
        """Service callback: Return buffer status.

        H-07: `last_solve_status` now carries the quality verdict from lctk_quality, so the
        interactive controller shows "DEGENERATE | 2 placements (18 frames) | ..." rather than
        "Calibration successful". `buffer_size` and `total_correspondences` are still frame counts,
        and are still misleading on their own — the placement count is the number that matters, so
        it goes into the status string where a reader cannot miss it.
        """
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
                response.message = (
                    "Buffer is empty and no transform available, nothing to save"
                )
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
                # v3 (H-10): 2D detections persist the real ArUco corners in
                # `results`, not just the axis-aligned bbox. v1/v2 files carry no
                # corners and reload into a biased (C-01) solve.
                # v4 (H-11): the file records the board-frame convention that produced
                # it, and keeps the board pose's 6x6 covariance.
                "version": FORMAT_VERSION,
                "board_frame_convention": BOARD_FRAME_CONVENTION,
                "num_detections": buffer_size,
                "detections": detections_data,
            }
            if transform_data:
                save_data["transform"] = transform_data

            # H-09: a saved calibration carries its own quality record, so it can be judged
            # later without re-deriving it. Unknown keys are ignored by older loaders.
            if self.last_quality is not None:
                q = self.last_quality
                save_data["quality"] = {
                    "status": q.status_line(),
                    "is_degenerate": q.is_degenerate,
                    "n_frames": q.n_frames,
                    "n_distinct_placements": q.n_placements,
                    "reprojection_rms_px": q.residuals.rms_px,
                    "reprojection_max_px": q.residuals.max_px,
                    "per_pose_rms_px": q.residuals.per_pose_rms_px,
                    "cond_JtJ": q.conditioning.cond,
                    "diversity": {
                        "normal_span_deg": q.diversity.normal_span_deg,
                        "depth_range_m": q.diversity.depth_range_m,
                        "lateral_span_m": q.diversity.lateral_span_m,
                    },
                    "uncertainty": (
                        {
                            "rot_deg": q.spread.rot_deg,
                            "trans_mm": q.spread.trans_mm,
                            "n_subsets": q.spread.n_subsets,
                        }
                        if q.spread is not None
                        else None
                    ),
                    "warnings": q.warnings(),
                }

            with open(file_path, "w") as f:
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
            with open(file_path, "r") as f:
                data = json.load(f)

            # H-11: a stale-convention file must be rejected, not silently
            # reinterpreted. Migrating on load would make a file's meaning depend on
            # which build opened it.
            version_error = format_version_error(data)
            if version_error is not None:
                response.success = False
                response.message = version_error
                response.num_detections = 0
                response.buffer_size = len(self.detection_buffer)
                self.get_logger().error(version_error)
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
                loaded_rvec = np.array(
                    data["transform"]["rvec"], dtype=np.float64
                ).reshape(3, 1)
                loaded_tvec = np.array(
                    data["transform"]["tvec"], dtype=np.float64
                ).reshape(3, 1)

            with self.lock:
                if not append:
                    self.detection_buffer.clear()
                self.detection_buffer.extend(loaded_detections)
                buffer_size = len(self.detection_buffer)

                # If we have a saved transform, use it directly instead of re-solving
                if has_transform:
                    self.current_rvec = loaded_rvec
                    self.current_tvec = loaded_tvec
                    self.last_transform = self._create_transform_message(
                        loaded_rvec, loaded_tvec
                    )
                    self.publishing_enabled = True
                    self.last_solve_status = (
                        "Loaded from file (with manual adjustments)"
                    )
                    self.get_logger().info(
                        "Restored manual transform adjustments from file"
                    )
                elif buffer_size >= self.min_frames_required:
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
                response.message = (
                    "No transform available to adjust. Solve calibration first."
                )
                return response

            # Apply translation adjustment
            self.current_tvec[0, 0] += request.delta_x
            self.current_tvec[1, 0] += request.delta_y
            self.current_tvec[2, 0] += request.delta_z

            # Apply rotation adjustment (as Euler angle deltas in XYZ order)
            if (
                request.delta_roll != 0
                or request.delta_pitch != 0
                or request.delta_yaw != 0
            ):
                # Get current rotation matrix
                current_rot_matrix, _ = cv2.Rodrigues(self.current_rvec)
                current_rot = R.from_matrix(current_rot_matrix)

                # Create delta rotation from Euler angles
                delta_rot = R.from_euler(
                    "xyz", [request.delta_roll, request.delta_pitch, request.delta_yaw]
                )

                # Apply delta rotation (delta * current)
                new_rot = delta_rot * current_rot
                new_rot_matrix = new_rot.as_matrix()

                # Convert back to Rodrigues vector
                self.current_rvec, _ = cv2.Rodrigues(new_rot_matrix)

            # Update transform message
            self.last_transform = self._create_transform_message(
                self.current_rvec, self.current_tvec
            )

            # Get updated Euler angles for logging
            rot_matrix, _ = cv2.Rodrigues(self.current_rvec)
            euler = R.from_matrix(rot_matrix).as_euler("xyz", degrees=True)

            response.success = True
            response.message = (
                f"Transform adjusted: t=({self.current_tvec[0, 0]:.4f}, {self.current_tvec[1, 0]:.4f}, {self.current_tvec[2, 0]:.4f}), "
                f"rpy=({euler[0]:.2f}, {euler[1]:.2f}, {euler[2]:.2f}) deg"
            )
            self.get_logger().info(response.message)

        return response

    def reset_transform_callback(self, request, response):
        """Service callback: Reset manual adjustments and re-solve from buffered detections."""
        with self.lock:
            buffer_size = len(self.detection_buffer)

            if buffer_size < self.min_frames_required:
                response.success = False
                response.message = f"Cannot reset: need at least {self.min_frames_required} frames in buffer (have {buffer_size})"
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
            solved_euler = R.from_matrix(solved_rot_matrix).as_euler("xyz")
            response.solved_x = float(self.solved_tvec[0, 0])
            response.solved_y = float(self.solved_tvec[1, 0])
            response.solved_z = float(self.solved_tvec[2, 0])
            response.solved_roll = float(solved_euler[0])
            response.solved_pitch = float(solved_euler[1])
            response.solved_yaw = float(solved_euler[2])

            # Get current pose as Euler angles
            current_rot_matrix, _ = cv2.Rodrigues(self.current_rvec)
            current_euler = R.from_matrix(current_rot_matrix).as_euler("xyz")
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
        return serialize_detection2d_array(msg)

    def _serialize_detection3d_array(self, msg: Detection3DArray) -> dict:
        return serialize_detection3d_array(msg)

    def _deserialize_detection2d_array(self, data: dict) -> Detection2DArray:
        return deserialize_detection2d_array(data)

    def _deserialize_detection3d_array(self, data: dict) -> Detection3DArray:
        return deserialize_detection3d_array(data)

    def _count_placements(self, exclude_last: bool = False) -> int:
        """How many DISTINCT board placements are in the buffer.

        H-07/H-09: this — not `len(self.detection_buffer)` — is the number that says how much the
        capture actually constrains. Twenty frames of a board that never moved are one placement.
        Caller must hold `self.lock`.
        """
        buffer = self.detection_buffer[:-1] if exclude_last else self.detection_buffer
        if not buffer:
            return 0

        poses = []
        for _, board_msg in buffer:
            if not board_msg.detections:
                continue
            pose = board_msg.detections[0].results[0].pose.pose
            poses.append(
                (
                    (pose.position.x, pose.position.y, pose.position.z),
                    (
                        pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z,
                        pose.orientation.w,
                    ),
                )
            )
        return len(distinct_placements(poses)) if poses else 0

    def _assess_quality(self, rvec: np.ndarray, tvec: np.ndarray) -> None:
        """Compute the H-09 quality report and surface it.

        Reports and warns; never rejects. C-04 was a gate whose threshold was unreachable, and it
        silently discarded every detection for months — quality thresholds get validated against
        field data before anything is allowed to refuse.
        """
        K = np.array(self.camera_info.k, dtype=np.float64).reshape(3, 3)

        self.last_quality = build_report(
            self._per_frame_object_points,
            self._per_frame_image_points,
            self._per_frame_board_poses,
            K,
            rvec,
            tvec,
        )
        if self.last_quality is None:
            return

        # `last_solve_status` is already a string, so the metric reaches the operator and the
        # interactive controller with no .srv change and no lctk_interfaces rebuild.
        self.last_solve_status = self.last_quality.status_line()

        warnings = self.last_quality.warnings()
        if warnings:
            self.get_logger().warn("\n".join(warnings))
        else:
            self.get_logger().info(f"Calibration quality: {self.last_solve_status}")

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
        if buffer_size < self.min_frames_required:
            self.last_solve_status = f"Insufficient frames: {buffer_size}/{self.min_frames_required} required"
            self.get_logger().warn(
                f"Buffered {buffer_size} frame(s), need {self.min_frames_required} minimum. "
                f"Add {self.min_frames_required - buffer_size} more to start calibration."
            )
            return False

        self.get_logger().info(
            f"\n{'#' * 80}\n"
            f"  SOLVING MULTI-POSE CALIBRATION FROM {buffer_size} BUFFERED POSES\n"
            f"{'#' * 80}\n"
        )

        # Accumulate all correspondences from buffer.
        #
        # H-09: keep the PER-FRAME arrays and the board poses as well as the concatenated set. The
        # quality metrics need them, because `N` is the number of DISTINCT board placements, not the
        # number of buffered frames — buffering one placement a hundred times is duplication, not
        # information, and a metric that cannot tell the difference reports its highest confidence
        # exactly when the calibration is worst.
        all_object_points = []
        all_image_points = []
        pose_board_detections = []  # H-07: for the pose-diversity gate
        self._per_frame_object_points = []
        self._per_frame_image_points = []
        self._per_frame_board_poses = []
        self._per_frame_weights = []

        for pose_idx, (aruco_msg, board_msg) in enumerate(self.detection_buffer, 1):
            self.get_logger().info(
                f"\n{'-' * 80}\nProcessing Pose #{pose_idx}/{buffer_size}\n{'-' * 80}"
            )

            # Convert messages to internal format
            aruco_markers = self._detection2d_to_aruco_markers(aruco_msg)
            board_detection = self._detection3d_to_board_detection(
                board_msg.detections[0]
            )
            pose_board_detections.append(board_detection)

            # Create correspondences for this pose
            object_points, image_points = self._create_point_correspondences(
                aruco_markers, board_detection
            )

            self.get_logger().info(
                f"Pose #{pose_idx}: Added {len(object_points)} correspondences"
            )

            all_object_points.extend(object_points)
            all_image_points.extend(image_points)

            self._per_frame_object_points.append(
                np.asarray(object_points, dtype=np.float64)
            )
            self._per_frame_image_points.append(
                np.asarray(image_points, dtype=np.float64)
            )
            self._per_frame_board_poses.append(
                (board_detection.position, board_detection.orientation)
            )
            self._per_frame_weights.append(
                self._pose_weight(board_detection, object_points)
            )

        # Convert to numpy arrays
        all_object_points = np.array(all_object_points, dtype=np.float64)
        all_image_points = np.array(all_image_points, dtype=np.float64)

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

        # H-07: pose-diversity gate. Every correspondence lies on a 500 mm coplanar
        # patch, so a near-coplanar / single-depth buffer under-constrains the
        # extrinsic (rotation about the correspondence centroid is a near-null
        # direction: the board overlay stays glued while the background tilts).
        # Buffering more frames of the same placement does NOT help -- only new
        # board orientations and depths do.
        normal_spread_deg, depth_range_m = self._compute_pose_diversity(
            pose_board_detections
        )
        self.get_logger().info(
            f"Pose diversity: board-normal spread = {normal_spread_deg:.1f} deg "
            f"(min {self.min_normal_spread_deg:.0f}), depth range = {depth_range_m:.2f} m "
            f"(min {self.min_depth_range_m:.2f})"
        )
        if (
            normal_spread_deg < self.min_normal_spread_deg
            or depth_range_m < self.min_depth_range_m
        ):
            diversity_msg = (
                f"Low pose diversity (board-normal spread {normal_spread_deg:.1f} deg, "
                f"depth range {depth_range_m:.2f} m): the accumulated poses are nearly "
                f"coplanar / at one depth, so the extrinsic is under-constrained about "
                f"the board centroid and the background will tilt. Capture the board at "
                f"more orientations (>~{self.min_normal_spread_deg:.0f} deg apart) and "
                f"depths (>~{self.min_depth_range_m:.1f} m range); more frames of the same "
                f"placement will NOT help."
            )
            if self.enforce_pose_diversity:
                self.last_solve_status = (
                    f"Refused: insufficient pose diversity "
                    f"(normal spread {normal_spread_deg:.1f} deg, "
                    f"depth range {depth_range_m:.2f} m)"
                )
                self.get_logger().error(diversity_msg)
                return False
            self.get_logger().warn(diversity_msg)

        # Solve PnP
        self.get_logger().info(
            f"\n{'=' * 80}\n"
            f"Solving PnP with {num_correspondences} total correspondences...\n"
            f"{'=' * 80}"
        )
        weights = self._expand_weights_to_correspondences()
        success, rvec, tvec = self._solve_pnp(
            all_object_points, all_image_points, weights=weights
        )

        if not success:
            self.last_solve_status = "PnP solver failed"
            self.get_logger().error(self.last_solve_status)
            return False

        # M-12: one bad pose corrupts the whole solve, and every corner of that pose is an
        # outlier together. Now that there is an estimate to measure against, look at the
        # per-pose reprojection error and re-solve without any pose that stands out.
        if self.reject_outlier_poses:
            K = np.array(self.camera_info.k, dtype=np.float64).reshape(3, 3)
            rejected, per_pose_rms = self._reject_outlier_poses(
                self._per_frame_object_points,
                self._per_frame_image_points,
                K,
                rvec,
                tvec,
                k=self.outlier_pose_mad_k,
            )
            self.get_logger().info(
                f"Per-pose reprojection RMS (px): {[round(r, 2) for r in per_pose_rms]}"
            )

            if rejected:
                keep = [
                    i
                    for i in range(len(self._per_frame_object_points))
                    if i not in rejected
                ]
                kept_object = np.vstack(
                    [self._per_frame_object_points[i] for i in keep]
                )
                kept_image = np.vstack([self._per_frame_image_points[i] for i in keep])
                kept_weights = None
                if weights is not None:
                    kept_weights = np.concatenate(
                        [
                            np.full(
                                len(self._per_frame_object_points[i]),
                                self._per_frame_weights[i],
                                dtype=np.float64,
                            )
                            for i in keep
                        ]
                    )

                ok_retry, rvec_retry, tvec_retry = self._solve_pnp(
                    kept_object, kept_image, weights=kept_weights
                )
                if ok_retry:
                    self.get_logger().warning(
                        f"Rejected {len(rejected)} outlier pose(s) {rejected} with "
                        f"reprojection RMS "
                        f"{[round(per_pose_rms[i], 2) for i in rejected]} px against a "
                        f"median of {round(float(np.median(per_pose_rms)), 2)} px, and "
                        f"re-solved on the remaining {len(keep)}. A pose this far out is "
                        f"usually a bad board fit or a quarter-turned origin corner (M-14)."
                    )
                    rvec, tvec = rvec_retry, tvec_retry
                else:
                    self.get_logger().warning(
                        "Re-solve without the outlier poses failed; keeping the original "
                        "estimate."
                    )

        # H-09: assess the solve. This does not change the estimate and does not block publishing;
        # it says how much the estimate is worth, which the pipeline previously could not.
        self._assess_quality(rvec, tvec)

        # Store solved and current rvec/tvec
        with self.lock:
            self.solved_rvec = rvec.copy()
            self.solved_tvec = tvec.copy()
            self.current_rvec = rvec.copy()
            self.current_tvec = tvec.copy()

        # Convert rvec to rotation matrix for logging
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        euler_angles = R.from_matrix(rotation_matrix).as_euler("xyz", degrees=True)

        # Create transform message
        transform_msg = self._create_transform_message(rvec, tvec)

        # Store result and enable publishing
        with self.lock:
            self.last_transform = transform_msg
            self.publishing_enabled = True
            self.total_correspondences = num_correspondences
            self.last_solve_status = "Calibration successful"

        self.get_logger().info(
            f"\n{'#' * 80}\n"
            f"  CALIBRATION SOLVED SUCCESSFULLY!\n"
            f"{'#' * 80}\n"
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
            f"{'#' * 80}\n"
        )

        return True

    def _detection2d_to_aruco_markers(
        self, detection_msg: Detection2DArray
    ) -> List[ArUcoMarker]:
        """Convert ROS Detection2DArray to ArUcoMarker objects."""
        return detection2d_to_aruco_markers(detection_msg)

    def _compute_pose_diversity(
        self, board_detections: List[BoardDetection]
    ) -> Tuple[float, float]:
        """H-07: geometric diversity of the accumulated board poses.

        Returns ``(max_board_normal_spread_deg, depth_range_m)``:

        - ``max_board_normal_spread_deg`` -- the largest angle between any two board
          plane normals. A near-parallel set cannot constrain the extrinsic rotation.
          Normals are compared unsigned (a normal and its flip describe the same plane).
        - ``depth_range_m`` -- the span of board-centroid ranges from the sensor.

        Both are 0.0 for fewer than two poses.
        """
        if len(board_detections) < 2:
            return 0.0, 0.0

        normals = []
        depths = []
        for det in board_detections:
            rot = R.from_quat(det.orientation).as_matrix()
            # Board plane normal is the board frame's +Z axis in the LiDAR frame.
            normals.append(rot @ np.array([0.0, 0.0, 1.0]))
            depths.append(float(np.linalg.norm(det.position)))

        max_angle_deg = 0.0
        for i in range(len(normals)):
            for j in range(i + 1, len(normals)):
                cos_angle = abs(float(np.dot(normals[i], normals[j])))
                cos_angle = min(1.0, max(-1.0, cos_angle))
                angle_deg = float(np.degrees(np.arccos(cos_angle)))
                max_angle_deg = max(max_angle_deg, angle_deg)

        depth_range_m = max(depths) - min(depths)
        return max_angle_deg, depth_range_m

    def _detection3d_to_board_detection(self, detection) -> BoardDetection:
        """Convert ROS Detection3D to BoardDetection object."""
        if not detection.results:
            raise ValueError("No detection results available")

        result = detection.results[0]
        pose = result.pose.pose

        # M-13: read the board-pose covariance the detector now publishes. An all-zero covariance
        # means the detector did not compute one (too few correspondences, or an older detector) --
        # treat that as "unknown", NOT as "exact".
        covariance = np.array(result.pose.covariance, dtype=np.float64).reshape(6, 6)
        if not np.any(covariance):
            covariance = None

        return BoardDetection(
            position=(pose.position.x, pose.position.y, pose.position.z),
            orientation=(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ),
            covariance=covariance,
        )

    def _await_board_frame_convention(self, timeout_s: float) -> None:
        """H-11: block until `lidar_board_detector` announces a convention we agree with.

        The published board pose is ``T_sensor<-board`` and this node feeds board-local
        marker coordinates into it, so the convention sits on BOTH sides of one
        product. A mismatch is therefore undetectable from the reprojection error an
        operator would check: the symmetric 2x2 marker grid absorbs a 45-degree
        in-plane rotation without complaint.

        Failure raises out of ``__init__``, which is not wrapped in a ``try`` in
        ``main()`` — so the process exits non-zero and the launch system can see it.
        """
        received: List[str] = []

        # QoS must match the publisher EXACTLY: RELIABLE + TRANSIENT_LOCAL +
        # KEEP_LAST(1). The detector publishes the tag once and holds the publisher so
        # the latched sample survives for late joiners.
        #
        # L-07 is the repository's one recorded lesson here, and it points the wrong
        # way for this topic: tf_tree_broadcaster.py carries a comment explaining why
        # NOT to use TRANSIENT_LOCAL. Following it blindly would give this guard a
        # VOLATILE subscriber, which never receives the latched sample, concludes
        # "absent", and refuses to start — turning a guard into an outage.
        qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        def on_tag(msg: String) -> None:
            received.append(msg.data)
            # Every subsequent message is validated too: a detector restarting on an
            # older build must be caught, not trusted because the first check passed.
            error = frame_convention_error(msg.data)
            if error is not None and self._frame_convention_ok:
                self.get_logger().fatal(error)
                raise SystemExit(1)

        self._frame_convention_ok = False
        self._frame_convention_subscription = self.create_subscription(
            String, BOARD_FRAME_CONVENTION_TOPIC, on_tag, qos
        )

        self.get_logger().info(
            f"Waiting up to {timeout_s:.1f}s for the board-frame convention on "
            f"{BOARD_FRAME_CONVENTION_TOPIC}"
        )
        deadline = time.monotonic() + timeout_s
        while not received and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        error = frame_convention_error(received[-1] if received else None)
        if error is not None:
            self.get_logger().fatal(error)
            raise RuntimeError(error)

        self._frame_convention_ok = True
        self.get_logger().info(
            f"Board-frame convention confirmed: '{BOARD_FRAME_CONVENTION}'"
        )

    def _load_aruco_pattern_config(self, config_file_path: str) -> dict:
        """Load ArUco pattern configuration from JSON5 file."""
        self.get_logger().info(f"Loading ArUco pattern config from: {config_file_path}")

        config = load_aruco_pattern_config(config_file_path)

        self.get_logger().info(
            f"Loaded ArUco config: {config['num_squares_per_side']}x{config['num_squares_per_side']} grid, "
            f"board_size={config['board_size']}, "
            f"marker IDs={config['marker_ids']}"
        )

        return config

    def _parse_dimension(self, dim_str: str) -> float:
        """Parse dimension string like '500mm' or '10mm' to meters."""
        return parse_dimension(dim_str)

    def _compute_multi_marker_corners(
        self,
    ) -> Dict[int, List[Tuple[float, float, float]]]:
        """Compute 3D corner positions for all ArUco markers in board frame."""
        marker_corners = compute_multi_marker_corners(self.aruco_pattern_config)

        self.get_logger().debug(
            f"Board geometry: {marker_geometry_summary(self.aruco_pattern_config)}"
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
            R.from_quat(board_detection.orientation).as_matrix().astype(np.float64)
        )
        board_position = np.array(board_detection.position, dtype=np.float64)

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

            local_corners = np.array(board_frame_corners[marker_id], dtype=np.float64)
            world_corners = (board_rotation @ local_corners.T).T + board_position

            image_corners = np.array(marker.corners, dtype=np.float64)

            object_points.extend(world_corners)
            image_points.extend(image_corners)

        return np.array(object_points, dtype=np.float64), np.array(
            image_points, dtype=np.float64
        )

    @staticmethod
    def _pose_weight(
        board_detection: BoardDetection, object_points: np.ndarray
    ) -> float:
        """M-13: how much this board pose should count, from its ICP covariance.

        `cv2.solvePnP` assumes the 3D points are exact. They are not — they are ideal ArUco corners
        pushed through the board pose that ICP fitted, so the board-pose uncertainty lands squarely
        in the "known" 3D points. That is an errors-in-variables problem, and the estimate is
        *biased*, not merely noisy.

        Propagate the pose covariance onto the corners it generated. A corner at lever arm `c` from
        the pose origin moves under a pose perturbation as `dt + dtheta x c`, so

            J_corner = [ I | -[c]_x ]   (3x6)      Sigma_corner = J Sigma_pose J^T

        and the weight is the inverse of the resulting positional standard deviation. A pose whose
        board sat at a grazing angle with few edge returns is loose in-plane, produces sloppy 3D
        corners, and now contributes proportionally less.

        Returns 1.0 when no covariance was published, so an older detector behaves exactly as
        before rather than silently down-weighting everything.
        """
        if board_detection.covariance is None or len(object_points) == 0:
            return 1.0

        origin = np.asarray(board_detection.position, dtype=np.float64)
        cov = board_detection.covariance

        total_var = 0.0
        for corner in np.asarray(object_points, dtype=np.float64):
            lever = corner - origin
            skew = np.array(
                [
                    [0.0, -lever[2], lever[1]],
                    [lever[2], 0.0, -lever[0]],
                    [-lever[1], lever[0], 0.0],
                ]
            )
            jac = np.hstack([np.eye(3), -skew])  # 3x6
            total_var += float(np.trace(jac @ cov @ jac.T))

        mean_var = total_var / len(object_points)
        sigma = np.sqrt(max(mean_var, 1e-12))

        # 1 cm is the scale of a healthy board fit (VLP-32C range noise is ~3 cm, and the ICP loss
        # asymptotes near 2.6 cm), so a good pose lands near weight 1 and a badly-conditioned one
        # falls off smoothly rather than being cliff-edged out.
        weight = 1.0 / (1.0 + sigma / 0.01)
        return float(np.clip(weight, 1e-3, 1.0))

    @staticmethod
    def _pose_reprojection_rms(
        object_points: np.ndarray,
        image_points: np.ndarray,
        K: np.ndarray,
        rvec: np.ndarray,
        tvec: np.ndarray,
    ) -> float:
        """RMS reprojection error, in pixels, for one pose's corners."""
        projected, _ = cv2.projectPoints(
            object_points, rvec, tvec, K, np.zeros(5, dtype=np.float64)
        )
        err = projected.reshape(-1, 2) - image_points.reshape(-1, 2)
        return float(np.sqrt(np.mean(np.sum(err**2, axis=1))))

    @staticmethod
    def _reject_outlier_poses(
        per_frame_object_points: List[np.ndarray],
        per_frame_image_points: List[np.ndarray],
        K: np.ndarray,
        rvec: np.ndarray,
        tvec: np.ndarray,
        k: float = 4.0,
        min_keep: int = 3,
        floor_px: float = 2.0,
    ) -> Tuple[List[int], List[float]]:
        """Identify poses whose reprojection error marks them as outliers (M-12).

        Rejection is at **pose** granularity, never per corner. A pose's corners share one rigid
        `T_board`, so when the pose is wrong all of its corners are wrong *together* -- one rigid
        misplacement observed 16 times, not 16 independent draws. Rejecting individual corners
        would treat correlated error as if it were independent, which is what least squares
        already assumes and already gets wrong here.

        The threshold is robust by construction: `median + k * 1.4826 * MAD` over the per-pose RMS.
        A mean-and-sigma rule would be inflated by the very outlier it is meant to catch.

        Two guards keep this from eating good data:

        - `floor_px` -- with a tight, clean buffer the MAD is nearly zero, so any spread at all
          would look significant. Nothing is rejected unless it also exceeds this absolute floor.
        - `min_keep` -- refuse to reject if too few poses would survive. An under-constrained
          solve is worse than one contaminated pose, and pose diversity is already the scarce
          resource (H-07).

        Returns the indices to reject and the per-pose RMS for every pose, so callers can log the
        residuals whether or not anything was rejected.
        """
        rms = [
            AdvancedExtrinsicSolver._pose_reprojection_rms(obj, img, K, rvec, tvec)
            for obj, img in zip(per_frame_object_points, per_frame_image_points)
        ]
        if len(rms) < min_keep + 1:
            # Rejecting anything here would drop the buffer below what a solve needs.
            return [], rms

        arr = np.asarray(rms, dtype=np.float64)
        median = float(np.median(arr))
        mad = float(np.median(np.abs(arr - median)))
        threshold = max(median + k * 1.4826 * mad, median + floor_px)

        candidates = [i for i, r in enumerate(rms) if r > threshold]

        # Never fall below min_keep: drop the worst offenders first and stop.
        max_rejects = max(len(rms) - min_keep, 0)
        if len(candidates) > max_rejects:
            candidates = sorted(candidates, key=lambda i: rms[i], reverse=True)[
                :max_rejects
            ]

        return sorted(candidates), rms

    def _expand_weights_to_correspondences(self) -> Optional[np.ndarray]:
        """One weight per correspondence, from the per-frame board-pose weights.

        Returns None when every pose is equally trusted (no covariance anywhere), so the solve
        takes the plain unweighted path.
        """
        if not self._per_frame_weights:
            return None
        if all(w >= 1.0 for w in self._per_frame_weights):
            return None

        weights = []
        for w, obj in zip(self._per_frame_weights, self._per_frame_object_points):
            weights.extend([w] * len(obj))
        return np.asarray(weights, dtype=np.float64)

    def _refine_pnp_weighted(
        self,
        object_points: np.ndarray,
        image_points: np.ndarray,
        K: np.ndarray,
        rvec: np.ndarray,
        tvec: np.ndarray,
        weights: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Weighted Levenberg-Marquardt polish of the reprojection cost.

        M-12 added `cv2.solvePnPRefineLM`, but **no OpenCV PnP entry point accepts per-point
        weights** — not solvePnP, not RefineLM, not RefineVVS. So to consume M-13's covariance at
        all, the polish has to be ours. This is the same LM refinement, with each residual scaled by
        its pose's weight.
        """
        from scipy.optimize import least_squares

        dist = np.zeros(5, dtype=np.float64)
        w = np.repeat(weights, 2)  # one weight per residual component (u and v)

        def residuals(params: np.ndarray) -> np.ndarray:
            r = params[:3].reshape(3, 1)
            t = params[3:].reshape(3, 1)
            projected, _ = cv2.projectPoints(object_points, r, t, K, dist)
            err = (projected.reshape(-1, 2) - image_points).ravel()
            return w * err

        seed = np.concatenate([rvec.ravel(), tvec.ravel()])
        result = least_squares(residuals, seed, method="lm", max_nfev=200)

        return result.x[:3].reshape(3, 1), result.x[3:].reshape(3, 1)

    def _solve_pnp(
        self,
        object_points: np.ndarray,
        image_points: np.ndarray,
        weights: Optional[np.ndarray] = None,
    ) -> Tuple[bool, Optional[np.ndarray], Optional[np.ndarray]]:
        """Solve the Perspective-n-Point problem using OpenCV."""
        if len(object_points) < 4:
            self.get_logger().error("PnP requires at least 4 point correspondences")
            return False, None, None

        K = np.array(self.camera_info.k, dtype=np.float64).reshape(3, 3)
        dist_coeffs = np.zeros(5, dtype=np.float64)

        self.get_logger().info(f"Solving PnP with {len(object_points)} correspondences")

        try:
            success, rvec, tvec = cv2.solvePnP(
                object_points,
                image_points,
                K,
                dist_coeffs,
                flags=cv2.SOLVEPNP_SQPNP,
            )

            if success:
                # M-12: SQPnP is a direct global solver and performs no nonlinear polish of the
                # reprojection cost, so refine the initialisation with Levenberg-Marquardt.
                #
                # M-13: when the detector publishes a board-pose covariance, the poses are NOT
                # equally trustworthy, and no OpenCV PnP entry point accepts per-point weights.
                # So the polish becomes a weighted LM of our own; without covariance we fall back
                # to OpenCV's, which is exactly the previous behaviour.
                if weights is not None:
                    try:
                        rvec, tvec = self._refine_pnp_weighted(
                            object_points, image_points, K, rvec, tvec, weights
                        )
                        self.get_logger().info(
                            f"Weighted LM refine (M-13): pose weights "
                            f"{np.round(self._per_frame_weights, 2).tolist()}"
                        )
                    except (ValueError, np.linalg.LinAlgError) as refine_err:
                        self.get_logger().warning(
                            f"Weighted refine failed ({refine_err}); using SQPnP result"
                        )
                else:
                    try:
                        rvec, tvec = cv2.solvePnPRefineLM(
                            object_points, image_points, K, dist_coeffs, rvec, tvec
                        )
                    except cv2.error as refine_err:
                        self.get_logger().warning(
                            f"solvePnPRefineLM skipped ({refine_err}); using SQPnP result"
                        )
                self.get_logger().info(
                    f"PnP solved successfully!\nTranslation: {tvec.flatten()}"
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

        # M-01: publish with ROS TF semantics.
        #
        # solvePnP returns (R, t) with p_cam = R @ p_lidar + t -- that is T_camera<-lidar.
        # A transform labelled `frame_id=lidar, child_frame_id=camera` means the *opposite* in
        # TF: the camera's pose expressed in lidar coordinates. Publishing the raw solve under
        # those labels pointed every tf2 consumer the wrong way, which is the one thing standing
        # between this output and a correct Autoware sensor_kit_calibration.yaml.
        #
        # The dumped JSON keeps the raw rvec/tvec -- that is what the exporter consumes, and it
        # is deliberately not touched here.
        rotation_matrix = rotation_matrix.T
        translation = -rotation_matrix @ tvec.reshape(3, 1)

        quaternion = self._rotation_matrix_to_quaternion(rotation_matrix)

        transform_msg = TransformStamped()
        transform_msg.header = Header()
        transform_msg.header.stamp = self.get_clock().now().to_msg()
        transform_msg.header.frame_id = self.parent_frame
        transform_msg.child_frame_id = self.child_frame

        t = translation.flatten()
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
        return rotation_matrix_to_quaternion(rotation_matrix)


def main(args=None):
    """Main function to run the lidar_to_camera_solver node."""
    rclpy.init(args=args)

    node = LidarToCameraSolver()

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
        node.get_logger().info("Shutting down lidar_to_camera_solver")
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
        except Exception as e:
            node.get_logger().warning(f"Failed to log final sync statistics: {e}")

        try:
            node.destroy_node()
        except Exception as e:
            node.get_logger().warning(f"Error while destroying node: {e}")
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            # Context may already be invalid at this point; log to stderr.
            print(f"Error during rclpy shutdown: {e}", file=sys.stderr)


if __name__ == "__main__":
    main()
