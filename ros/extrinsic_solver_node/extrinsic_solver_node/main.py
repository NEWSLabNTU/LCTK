#!/usr/bin/env python3
"""
Educational Extrinsic Calibration Node

This simplified ROS2 node demonstrates LiDAR-camera extrinsic calibration
using the Perspective-n-Point (PnP) algorithm with OpenCV.

Learning Objectives:
1. Understand coordinate system transformations (camera, LiDAR, world)
2. Learn PnP problem formulation and solution
3. Practice with OpenCV computer vision functions
4. Work with ROS2 message types and transformations

Required packages: numpy (1.x), opencv-python, rclpy
Educational focus: cv2 for computer vision, numpy for array operations

Author: LCTK Educational Team
License: MIT
"""

import threading
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import cv2  # OpenCV for computer vision tasks
import numpy as np  # Ubuntu 22.04 default (1.x)
import rclpy
from conflux_py import DropPolicy, ROS2Synchronizer, SyncGroup
from geometry_msgs.msg import Quaternion, TransformStamped, Vector3
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation as R  # Quaternion operations
from sensor_msgs.msg import CameraInfo

# ROS2 message types
from std_msgs.msg import Header
from vision_msgs.msg import Detection2DArray, Detection3D, Detection3DArray


@dataclass
class ArUcoMarker:
    """
    Represents an ArUco marker detection in image coordinates.

    Educational note: ArUco markers provide known 3D-2D correspondences
    needed for PnP solving. Each marker has 4 corner points that can be
    precisely detected in images and matched to known 3D positions.
    """

    id: int
    corners: List[Tuple[float, float]]  # 4 corners in pixel coordinates
    center: Tuple[float, float]  # Center point in pixels


@dataclass
class BoardDetection:
    """
    Represents a calibration board detection in 3D LiDAR coordinates.

    Educational note: Board pose provides the 3D reference frame for
    transforming marker coordinates from local to world space. The board
    serves as a common reference object visible to both LiDAR and camera.
    """

    position: Tuple[float, float, float]  # x, y, z in meters (LiDAR frame)
    orientation: Tuple[float, float, float, float]  # quaternion w, x, y, z


class EducationalExtrinsicSolver(Node):
    """
    Educational ROS2 node for LiDAR-camera extrinsic calibration.

    This node demonstrates the complete calibration pipeline:
    1. Receive ArUco marker detections (2D image coordinates)
    2. Receive calibration board detections (3D LiDAR coordinates)
    3. Create 3D-2D point correspondences
    4. Solve PnP problem using OpenCV
    5. Publish camera-to-LiDAR transformation

    Key Educational Concepts:
    - Coordinate system transformations
    - Homogeneous coordinates and camera projection
    - PnP problem formulation and solution
    - Rotation representations (matrices, vectors, quaternions)

    Coordinate System Conventions:
    - Camera frame: X-right, Y-down, Z-forward (OpenCV convention)
    - LiDAR frame: X-forward, Y-left, Z-up (ROS REP-103)
    - Board frame: Z-normal to board surface
    - World frame: Same as LiDAR frame for this application
    """

    def __init__(self):
        super().__init__("educational_extrinsic_solver")

        # Essential parameter declarations
        self.declare_parameter("parent_frame", "lidar")
        self.declare_parameter("child_frame", "camera")
        self.declare_parameter("camera_topic", "")
        self.declare_parameter("aruco_config_file", "")
        self.declare_parameter("debug_mode", True)
        self.declare_parameter("use_best_effort_qos", True)
        self.declare_parameter("sync_tolerance_ms", 50.0)
        self.declare_parameter("sync_queue_size", 10)
        self.declare_parameter(
            "sync_drop_policy", "reject_new"
        )  # reject_new or drop_oldest

        # Get parameters with simple error handling
        self.parent_frame = (
            self.get_parameter("parent_frame").get_parameter_value().string_value
        )
        self.child_frame = (
            self.get_parameter("child_frame").get_parameter_value().string_value
        )
        aruco_config_file = (
            self.get_parameter("aruco_config_file").get_parameter_value().string_value
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

        # Load ArUco pattern configuration
        self.aruco_pattern_config = self._load_aruco_pattern_config(aruco_config_file)

        # Camera info is cached separately (doesn't need time synchronization)
        self.camera_info: Optional[CameraInfo] = None
        self.camera_info_lock = threading.Lock()

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

        # Publishers - only essential output
        self.transform_publisher = self.create_publisher(
            TransformStamped, "extrinsic_transform", qos_profile
        )

        # Create synchronizer for ArUco and board detections
        # Educational note: conflux_py synchronizes messages from multiple topics
        # within a configurable time window, ensuring matched detection pairs
        self.get_logger().info(
            f"Using Conflux synchronization (window={int(sync_tolerance_ms)}ms, buffer={sync_queue_size}, policy={sync_drop_policy.name})"
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

        # Derive camera_info topic from camera_topic parameter (image_pipeline convention)
        camera_topic = (
            self.get_parameter("camera_topic").get_parameter_value().string_value
        )
        if camera_topic:
            # Educational note: ROS image_pipeline convention
            # Replace last component with 'camera_info'
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

        # Educational note: Log configuration for learning purposes

        self.get_logger().info(
            f"Educational Extrinsic Solver initialized\n"
            f"Educational Mode: Simplified PnP calibration using OpenCV\n"
            f"Using conflux_py for time-synchronized detection pairs\n"
            f"Subscribing to: aruco_detections, calibration_board_detections, {camera_info_topic}\n"
            f"Publishing to: extrinsic_transform\n"
            f"Transform: {self.parent_frame} -> {self.child_frame}\n"
            f"Using cv2.solvePnP for educational demonstration"
        )

    def camera_info_callback(self, msg: CameraInfo):
        """
        Handle camera info messages.

        Educational note: Camera info provides the intrinsic parameters
        needed for PnP solving. This includes the camera matrix K and
        distortion coefficients.
        """
        with self.camera_info_lock:
            self.camera_info = msg
            self.get_logger().debug(f"Camera info received: {msg.width}x{msg.height}")

    def _handle_synchronized_detections(self, group: SyncGroup):
        """
        Handle synchronized ArUco and board detections.

        Educational note: This callback is invoked by conflux_py when it has
        received messages from all subscribed topics within the time window.
        This ensures that ArUco and board detections are from the same moment.
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

        # Skip if either detection is empty
        if not aruco_msg.detections:
            self.get_logger().debug("Ignoring empty ArUco detection in sync group")
            return
        if not board_msg.detections:
            self.get_logger().warn("Received empty board detection in sync group")
            return

        self.get_logger().info(
            f"Processing synchronized detection pair: "
            f"{len(aruco_msg.detections)} ArUco markers, "
            f"{len(board_msg.detections)} boards"
        )

        try:
            self._solve_extrinsic_calibration(aruco_msg, board_msg)
        except Exception as e:
            self.get_logger().error(f"Calibration failed: {e}")

    def _solve_extrinsic_calibration(
        self, aruco_msg: Detection2DArray, board_msg: Detection3DArray
    ) -> bool:
        """
        Solve extrinsic calibration using PnP.

        Educational Pipeline:
        1. Check prerequisites (camera info, detections)
        2. Convert ROS messages to internal format
        3. Create 3D-2D point correspondences
        4. Solve PnP problem using OpenCV
        5. Publish transformation result

        Returns:
            bool: True if calibration succeeded and transform was published
        """
        # Step 1: Check prerequisites
        if not self.camera_info:
            self.get_logger().error("No camera info available for PnP solving")
            return False

        if not aruco_msg.detections or not board_msg.detections:
            self.get_logger().error("Empty detections - cannot solve PnP")
            return False

        # Step 2: Convert ROS messages to internal format
        aruco_markers = self._detection2d_to_aruco_markers(aruco_msg)
        board_detection = self._detection3d_to_board_detection(board_msg.detections[0])

        # Step 3: Create point correspondences
        object_points, image_points = self._create_point_correspondences_educational(
            aruco_markers, board_detection
        )

        if len(object_points) < 4:
            self.get_logger().error(
                f"Insufficient correspondences: {len(object_points)} < 4 required for PnP"
            )
            return False

        self.get_logger().info(
            f"Created {len(object_points)} point correspondences for PnP solving"
        )

        # Debug: Print point correspondences for homework data collection
        self.get_logger().debug("=" * 80)
        self.get_logger().debug("HOMEWORK DATA COLLECTION - Point Correspondences")
        self.get_logger().debug("=" * 80)
        self.get_logger().debug(f"Number of correspondences: {len(object_points)}")
        self.get_logger().debug("\n3D Object Points (world frame, meters):")
        for i, pt in enumerate(object_points):
            self.get_logger().debug(f"  [{i}] ({pt[0]:.6f}, {pt[1]:.6f}, {pt[2]:.6f})")
        self.get_logger().debug(
            "\n2D Image Points (pixels, UNDISTORTED from pipeline):"
        )
        for i, pt in enumerate(image_points):
            self.get_logger().debug(f"  [{i}] ({pt[0]:.2f}, {pt[1]:.2f})")
        self.get_logger().debug(f"\nCamera Matrix K:\n{self.camera_info.k}")
        self.get_logger().debug(f"Distortion coefficients: {self.camera_info.d}")
        self.get_logger().debug("=" * 80)

        # Step 4: Solve PnP problem
        success, rvec, tvec = self._solve_pnp_educational(object_points, image_points)

        if not success:
            self.get_logger().error("PnP solver failed")
            return False

        # Step 5: Publish transformation
        transform_msg = self._create_transform_message_educational(
            rvec, tvec, aruco_msg.header
        )

        try:
            self.transform_publisher.publish(transform_msg)
            self.get_logger().info(
                f"Published extrinsic transform: "
                f"t=({tvec.flatten()[0]:.3f}, {tvec.flatten()[1]:.3f}, {tvec.flatten()[2]:.3f})"
            )
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to publish transform: {e}")
            return False

    def _detection2d_to_aruco_markers(
        self, detection_msg: Detection2DArray
    ) -> List[ArUcoMarker]:
        """
        Convert ROS Detection2DArray to ArUcoMarker objects.

        The real detected marker corners are carried in `detection.results`
        (one entry per corner, in detector order TL, TR, BR, BL) by the ArUco
        locator node. We read those directly; the axis-aligned bounding box is
        only a fallback for messages that do not carry corners.
        """
        markers = []

        for detection in detection_msg.detections:
            bbox = detection.bbox
            center = (bbox.center.position.x, bbox.center.position.y)

            # C-01: prefer the real per-corner pixel coordinates. Reconstructing
            # corners from `center +/- size/2` discards rotation and perspective,
            # biasing the PnP correspondences for any angled view of the board.
            if len(detection.results) >= 4:
                corners = [
                    (r.pose.pose.position.x, r.pose.pose.position.y)
                    for r in detection.results[:4]
                ]
            else:
                size_x = bbox.size_x
                size_y = bbox.size_y
                cx, cy = center
                corners = [
                    (cx - size_x / 2.0, cy - size_y / 2.0),  # Top-left
                    (cx + size_x / 2.0, cy - size_y / 2.0),  # Top-right
                    (cx + size_x / 2.0, cy + size_y / 2.0),  # Bottom-right
                    (cx - size_x / 2.0, cy + size_y / 2.0),  # Bottom-left
                ]

            # Extract marker ID
            marker_id = detection.id if hasattr(detection, "id") else 0

            markers.append(
                ArUcoMarker(id=marker_id, corners=corners, center=center)
            )

        return markers

    def _detection3d_to_board_detection(self, detection: Detection3D) -> BoardDetection:
        """
        Convert ROS Detection3D to BoardDetection object.

        Educational note: This extracts the 3D pose of the calibration board
        from the ROS message format. The pose includes both position and
        orientation in 3D space.
        """
        if not detection.results:
            raise ValueError("No detection results available")

        pose = detection.results[0].pose.pose
        return BoardDetection(
            position=(pose.position.x, pose.position.y, pose.position.z),
            orientation=(
                pose.orientation.x,  # ROS2 quaternion convention: (x, y, z, w)
                pose.orientation.y,  # This matches SciPy's default scalar-last format
                pose.orientation.z,
                pose.orientation.w,
            ),
        )

    def _load_aruco_pattern_config(self, config_file_path: str) -> dict:
        """
        Load ArUco pattern configuration from JSON5 file.

        The config file contains board geometry parameters needed for
        computing accurate 3D positions of ArUco markers on the calibration board.

        Educational note: JSON5 allows comments and trailing commas for better
        readability. We use standard json module since JSON5 is a superset of JSON.
        """
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
        """
        Parse dimension string like '500mm' or '10mm' to meters.

        Educational note: The config file uses human-readable units (mm)
        but ROS and OpenCV work in meters.
        """
        if dim_str.endswith("mm"):
            return float(dim_str[:-2]) / 1000.0
        elif dim_str.endswith("m"):
            return float(dim_str[:-1])
        else:
            return float(dim_str)

    def _compute_multi_marker_corners(
        self,
    ) -> Dict[int, List[Tuple[float, float, float]]]:
        """
        Compute 3D corner positions for all ArUco markers in board frame.

        This is the Python equivalent of Rust's HollowBoard::multi_marker_corners() method.

        The calibration board has a 2x2 grid layout with 4 ArUco markers positioned at:
        - Bottom (row 0, col 0)
        - Left (row 0, col 1)
        - Right (row 1, col 0)
        - Top (row 1, col 1)

        Each marker has 4 corners ordered as: TL, TR, BR, BL (counter-clockwise from top-left)

        Returns:
            Dict mapping marker IDs to their 4 corner positions in board frame coordinates
        """
        config = self.aruco_pattern_config

        # Parse board dimensions
        board_size = self._parse_dimension(config["board_size"])
        board_border_size = self._parse_dimension(config["board_border_size"])
        marker_square_size_ratio = config["marker_square_size_ratio"]
        num_squares = config["num_squares_per_side"]
        marker_ids = config["marker_ids"]

        # Calculate grid geometry (matching Rust implementation)
        square_size = (board_size - 2.0 * board_border_size) / num_squares
        marker_size = square_size * marker_square_size_ratio
        marker_border = (square_size - marker_size) / 2.0

        self.get_logger().debug(
            f"Board geometry: square_size={square_size * 1000:.1f}mm, "
            f"marker_size={marker_size * 1000:.1f}mm, "
            f"marker_border={marker_border * 1000:.1f}mm"
        )

        # Helper function to create corners for a marker at given base position
        def make_corners(
            base_x: float, base_y: float
        ) -> List[Tuple[float, float, float]]:
            """
            Create 4 corner points for a marker in board-local coordinates.
            Corner ordering matches Rust: [right, top, left, bottom]

            Rust implementation:
                let bottom = self.board_plane_point(base_x, base_y);
                let left = self.board_plane_point(base_x + marker_size, base_y);
                let right = self.board_plane_point(base_x, base_y + marker_size);
                let top = self.board_plane_point(base_x + marker_size, base_y + marker_size);
                vec![right, top, left, bottom]

            In board frame: X-axis points right, Y-axis points up, Z-axis is normal to board.
            Returns corners in board-local coordinates (z=0 plane).
            The board pose transformation is applied later when creating correspondences.
            """
            # Corners in board-local frame (matching Rust's board_plane_point with identity transform)
            bottom = (base_x, base_y, 0.0)
            left = (base_x + marker_size, base_y, 0.0)
            right = (base_x, base_y + marker_size, 0.0)
            top = (base_x + marker_size, base_y + marker_size, 0.0)

            # Return in Rust ordering: [right, top, left, bottom]
            return [right, top, left, bottom]

        # Calculate origin offset (top-left corner of first marker)
        origin_x = board_border_size + marker_border
        origin_y = board_border_size + marker_border

        # Create corners for each marker position (2x2 grid, x-major order)
        # marker_ids = [bottom, left, right, top] for 2x2 grid
        marker_corners = {}

        # Bottom marker (row 0, col 0)
        marker_corners[marker_ids[0]] = make_corners(origin_x, origin_y)

        # Left marker (row 0, col 1)
        marker_corners[marker_ids[1]] = make_corners(origin_x + square_size, origin_y)

        # Right marker (row 1, col 0)
        marker_corners[marker_ids[2]] = make_corners(origin_x, origin_y + square_size)

        # Top marker (row 1, col 1)
        marker_corners[marker_ids[3]] = make_corners(
            origin_x + square_size, origin_y + square_size
        )

        self.get_logger().debug(
            f"Computed corners for {len(marker_corners)} markers in board frame"
        )

        return marker_corners

    def _create_point_correspondences_educational(
        self, aruco_markers: List[ArUcoMarker], board_detection: BoardDetection
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Create 3D-2D point correspondences for PnP solving.

        Educational Pipeline:
        1. Define marker geometry in local coordinates (marker frame)
        2. Transform to world coordinates using board pose (LiDAR frame)
        3. Associate with detected 2D image coordinates (camera frame)

        The PnP algorithm needs these correspondences to estimate camera pose:
        - object_points: 3D coordinates in world space (LiDAR frame)
        - image_points: 2D coordinates in image space (pixel coordinates)

        Mathematical relationship: s * p = K * (R * P + t)
        Where:
        - P: 3D world point (object_points)
        - p: 2D image point (image_points)
        - K: camera intrinsic matrix
        - R, t: camera extrinsic parameters (what we solve for)
        - s: scale factor
        """
        object_points = []
        image_points = []

        self.get_logger().info(
            f"Creating correspondences for {len(aruco_markers)} markers "
            f"with board at position {board_detection.position}"
        )

        # Convert board orientation quaternion to rotation matrix using SciPy
        # Board pose represents the rigid transformation from board frame to LiDAR frame
        # board_detection.orientation is in (x, y, z, w) format matching SciPy's default
        board_rotation = (
            R.from_quat(board_detection.orientation).as_matrix().astype(np.float32)
        )
        board_position = np.array(board_detection.position, dtype=np.float32)

        self.get_logger().debug(
            f"Board pose: position={board_position}, "
            f"quaternion=(x,y,z,w)={board_detection.orientation}"
        )

        # Compute 3D corner positions for all markers in board frame
        # This matches the Rust implementation: HollowBoard::multi_marker_corners()
        board_frame_corners = self._compute_multi_marker_corners()

        for i, marker in enumerate(aruco_markers):
            # Parse marker ID from string format "aruco_696" to integer 696
            marker_id_str = marker.id
            if isinstance(marker_id_str, str) and marker_id_str.startswith("aruco_"):
                marker_id = int(marker_id_str.split("_")[1])
            else:
                marker_id = (
                    int(marker_id_str)
                    if isinstance(marker_id_str, str)
                    else marker_id_str
                )

            # Look up the pre-computed corner positions for this marker
            if marker_id not in board_frame_corners:
                self.get_logger().warn(
                    f"Marker ID {marker_id} not found in ArUco pattern config, skipping"
                )
                continue

            # Step 1: Get 4 corners in marker's local coordinate system (board frame)
            # Educational note: Each marker has its own position in the 2x2 grid
            local_corners = np.array(board_frame_corners[marker_id], dtype=np.float32)

            # Step 2: Transform to LiDAR frame using full rigid transformation
            # world_point = R * local_point + t
            world_corners = (board_rotation @ local_corners.T).T + board_position

            # Step 3: Add to correspondence lists
            object_points.extend(world_corners)

            # Add corresponding 2D image points
            image_corners = np.array(marker.corners, dtype=np.float32)
            image_points.extend(image_corners)

            self.get_logger().debug(
                f"Marker {marker_id} at grid position: "
                f"local corners in board frame, transformed to world -> "
                f"image center ({marker.center[0]:.1f}, {marker.center[1]:.1f})"
            )

        if len(object_points) == 0:
            self.get_logger().error("No valid marker correspondences found")

        return np.array(object_points, dtype=np.float32), np.array(
            image_points, dtype=np.float32
        )

    def _solve_pnp_educational(
        self, object_points: np.ndarray, image_points: np.ndarray
    ) -> Tuple[bool, Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Solve the Perspective-n-Point problem using OpenCV.

        Educational Context:
        The PnP problem estimates camera pose given:
        - N >= 4 known 3D points in world coordinates (object_points)
        - Corresponding 2D projections in image coordinates (image_points)
        - Camera intrinsic parameters (focal length, principal point, distortion)

        Mathematical formulation:
        For each point i: s_i * p_i = K * (R * P_i + t)
        Where:
        - P_i: 3D world point
        - p_i: 2D image point (homogeneous coordinates)
        - K: camera intrinsic matrix
        - R: rotation matrix (camera to world)
        - t: translation vector
        - s_i: scale factor

        OpenCV Methods Available:
        - SOLVEPNP_ITERATIVE: Iterative Levenberg-Marquardt optimization
        - SOLVEPNP_EPNP: Efficient PnP for N >= 4 points
        - SOLVEPNP_P3P: Perspective-3-Point for exactly 3 points
        """
        if len(object_points) < 4:
            self.get_logger().error("PnP requires at least 4 point correspondences")
            return False, None, None

        # Extract camera intrinsic matrix (3x3) from camera_info
        # Educational note: K matrix defines internal camera geometry
        K = np.array(self.camera_info.k, dtype=np.float32).reshape(3, 3)

        # Use zero distortion coefficients since ArUco detection now uses undistorted images
        # Educational note: Images are pre-undistorted in ArUco locator node
        dist_coeffs = np.zeros(5, dtype=np.float32)

        self.get_logger().info(
            f"Solving PnP with {len(object_points)} correspondences\n"
            f"Camera matrix K:\n{K}\n"
            f"Distortion coefficients: {dist_coeffs[:4] if len(dist_coeffs) >= 4 else dist_coeffs}"
        )

        try:
            # Solve PnP using OpenCV's iterative method
            # Educational note: ITERATIVE method is robust and educational
            success, rvec, tvec = cv2.solvePnP(
                object_points,  # 3D object points (Nx3)
                image_points,  # 2D image points (Nx2)
                K,  # Camera intrinsic matrix (3x3)
                dist_coeffs,  # Distortion coefficients
                flags=cv2.SOLVEPNP_ITERATIVE,
            )

            if success:
                self.get_logger().info(
                    f"PnP solved successfully!\n"
                    f"Rotation vector (axis-angle): {rvec.flatten()}\n"
                    f"Translation vector: {tvec.flatten()}"
                )
                return True, rvec, tvec
            else:
                self.get_logger().error("PnP solver failed to converge")
                return False, None, None

        except cv2.error as e:
            self.get_logger().error(f"OpenCV PnP error: {e}")
            return False, None, None

    def _create_transform_message_educational(
        self, rvec: np.ndarray, tvec: np.ndarray, header: Header
    ) -> TransformStamped:
        """
        Create ROS TransformStamped message from PnP solution.

        Educational note: This converts the PnP solution (rotation vector + translation)
        to a ROS transform message. The rotation vector is converted to a quaternion
        for ROS compatibility.

        Rotation representations:
        - Rotation vector (rvec): 3D vector encoding axis and angle (OpenCV output)
        - Rotation matrix: 3x3 matrix representation
        - Quaternion: 4D representation used by ROS (more compact, no singularities)
        """
        # Convert rotation vector to rotation matrix using OpenCV
        rotation_matrix, _ = cv2.Rodrigues(rvec)

        # Convert rotation matrix to quaternion
        quaternion = self._rotation_matrix_to_quaternion_educational(rotation_matrix)

        # Create ROS transform message
        transform_msg = TransformStamped()
        transform_msg.header = Header()
        transform_msg.header.stamp = header.stamp
        transform_msg.header.frame_id = self.parent_frame
        transform_msg.child_frame_id = self.child_frame

        # Set translation (direct copy from PnP solution)
        t = tvec.flatten()
        transform_msg.transform.translation = Vector3(
            x=float(t[0]), y=float(t[1]), z=float(t[2])
        )

        # Set rotation (converted to quaternion)
        transform_msg.transform.rotation = Quaternion(
            x=float(quaternion[0]),
            y=float(quaternion[1]),
            z=float(quaternion[2]),
            w=float(quaternion[3]),
        )

        return transform_msg

    def _rotation_matrix_to_quaternion_educational(
        self, rotation_matrix: np.ndarray
    ) -> np.ndarray:
        """
        Convert 3x3 rotation matrix to quaternion using OpenCV and numpy.

        Educational note: This demonstrates rotation representation conversion.
        We use OpenCV's Rodrigues function to convert back to rotation vector,
        then implement the axis-angle to quaternion conversion.

        Mathematical background:
        - Rotation vector: angle * unit_axis (3D)
        - Quaternion: [sin(angle/2) * axis, cos(angle/2)] (4D)

        This approach is educational because it shows the mathematical
        relationship between different rotation representations.
        """
        # Convert rotation matrix back to rotation vector using OpenCV
        rvec, _ = cv2.Rodrigues(rotation_matrix)

        # Convert rotation vector to quaternion (educational implementation)
        # This shows students the mathematical relationship
        angle = np.linalg.norm(rvec)

        if angle < 1e-6:
            # Handle small angle case (near identity rotation)
            return np.array([0.0, 0.0, 0.0, 1.0])  # Identity quaternion (x,y,z,w)

        # Extract rotation axis (unit vector)
        axis = rvec.flatten() / angle
        half_angle = angle / 2.0

        # Compute quaternion components
        # Educational note: Quaternion = [sin(θ/2) * axis, cos(θ/2)]
        qx = axis[0] * np.sin(half_angle)
        qy = axis[1] * np.sin(half_angle)
        qz = axis[2] * np.sin(half_angle)
        qw = np.cos(half_angle)

        return np.array([qx, qy, qz, qw])  # Return as (x, y, z, w) for ROS


def main(args=None):
    """
    Main function to run the educational extrinsic solver node.

    Educational note: This is the standard ROS2 node entry point.
    It initializes ROS2, creates the node, and handles shutdown.
    """
    import time

    rclpy.init(args=args)

    node = EducationalExtrinsicSolver()

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
        node.get_logger().info("Shutting down educational extrinsic solver")
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
