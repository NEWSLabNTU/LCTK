#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np
import cv2
import json
import os
import yaml
from typing import List, Tuple, Optional, Dict, Any
import threading
from dataclasses import dataclass

# ROS2 message types
from std_msgs.msg import String, Header
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import Transform, TransformStamped, Vector3, Quaternion
from vision_msgs.msg import Detection2DArray, Detection2D, Detection3DArray, Detection3D


@dataclass
class ArUcoMarker:
    """Represents an ArUco marker detection"""

    id: int
    corners: List[Tuple[float, float]]  # 4 corners in image coordinates
    center: Tuple[float, float]


@dataclass
class BoardDetection:
    """Represents a calibration board detection"""

    position: Tuple[float, float, float]  # x, y, z
    orientation: Tuple[float, float, float, float]  # quaternion w, x, y, z


class SimpleExtrinsicSolver(Node):
    """
    Simple ROS2 node for demonstrating solvePnP with ArUco and board detections.
    This is a simplified version of the Rust extrinsic_solver_node.
    """

    def __init__(self):
        super().__init__("extrinsic_solver_node")

        # Declare parameters
        self.declare_parameter("parent_frame", "lidar")
        self.declare_parameter("child_frame", "camera")
        self.declare_parameter("aruco_pattern_file", "")
        self.declare_parameter("enable_quality_assessment", True)
        # External config files
        self.declare_parameter("aruco_config_file", "")
        self.declare_parameter("board_detector_file", "")
        self.declare_parameter("intrinsics_file", "")

        # Get parameters
        self.parent_frame = (
            self.get_parameter("parent_frame").get_parameter_value().string_value
        )
        self.child_frame = (
            self.get_parameter("child_frame").get_parameter_value().string_value
        )
        self.aruco_pattern_file = (
            self.get_parameter("aruco_pattern_file").get_parameter_value().string_value
        )
        self.enable_quality_assessment = (
            self.get_parameter("enable_quality_assessment")
            .get_parameter_value()
            .bool_value
        )
        # Get config file paths, use defaults if empty
        aruco_config_file = (
            self.get_parameter("aruco_config_file").get_parameter_value().string_value
        )
        board_detector_file = (
            self.get_parameter("board_detector_file").get_parameter_value().string_value
        )
        intrinsics_file = (
            self.get_parameter("intrinsics_file").get_parameter_value().string_value
        )

        # Use environment variable to find workspace root for default configs (portable)
        workspace_root = ""
        # Prefer current working dir if it contains config/
        cwd = os.getcwd()
        if os.path.exists(os.path.join(cwd, "config")):
            workspace_root = cwd
        # Try to resolve from AMENT_PREFIX_PATH (dev/installed workspaces)
        if not workspace_root:
            ament_prefix = os.environ.get("AMENT_PREFIX_PATH", "")
            if ament_prefix:
                first_prefix = ament_prefix.split(":")[0]
                candidate = first_prefix
                if candidate.endswith("/install"):
                    candidate = candidate[: -len("/install")]
                if os.path.exists(os.path.join(candidate, "config")):
                    workspace_root = candidate
        # Final fallback: search upward for config/
        if not workspace_root:
            cur = cwd
            for _ in range(4):
                if os.path.exists(os.path.join(cur, "config")):
                    workspace_root = cur
                    break
                parent = os.path.dirname(cur)
                if parent == cur:
                    break
                cur = parent

        # Set default paths if empty
        self.aruco_config_file = (
            aruco_config_file
            if aruco_config_file
            else os.path.join(workspace_root, "config", "aruco_pattern.json5")
        )
        self.board_detector_file = (
            board_detector_file
            if board_detector_file
            else os.path.join(workspace_root, "config", "board_detector.json5")
        )
        self.intrinsics_file = (
            intrinsics_file
            if intrinsics_file
            else os.path.join(workspace_root, "config", "intrinsics.yaml")
        )

        # State variables
        self.latest_aruco_detection: Optional[Detection2DArray] = None
        self.latest_board_detection: Optional[Detection3DArray] = None
        self.camera_info: Optional[CameraInfo] = None
        self.latest_debug_image_header: Optional[Header] = None

        # Thread safety
        self.lock = threading.Lock()

        # ArUco pattern configuration (simplified)
        self.aruco_pattern = self._load_aruco_pattern()
        # Optionally preload intrinsics if provided (used when camera_info topic absent)
        if self.intrinsics_file:
            self._maybe_load_intrinsics_from_yaml(self.intrinsics_file)

        # QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Publishers
        self.transform_publisher = self.create_publisher(
            TransformStamped, "extrinsic_transform", qos_profile
        )
        self.quality_publisher = self.create_publisher(
            String, "calibration_quality", qos_profile
        )
        self.debug_aruco_publisher = self.create_publisher(
            Detection2DArray, "debug/recent_aruco_detections", qos_profile
        )
        self.debug_board_publisher = self.create_publisher(
            Detection3DArray, "debug/recent_board_detections", qos_profile
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

        self.camera_info_subscription = self.create_subscription(
            CameraInfo, "camera_info", self.camera_info_callback, qos_profile
        )

        # Optional debug overlay image (for timestamp alignment/monitoring)
        self.debug_image_subscription = self.create_subscription(
            Image, "image_with_detections", self.debug_image_callback, qos_profile
        )

        self.get_logger().info(
            f"Simple Extrinsic Solver initialized. "
            f"Subscribing to: aruco_detections, calibration_board_detections, camera_info. "
            f"Publishing to: extrinsic_transform, calibration_quality, debug topics. "
            f"Parent frame: {self.parent_frame}, Child frame: {self.child_frame}"
        )

    def _load_aruco_pattern(self) -> Dict[str, Any]:
        """Load ArUco pattern configuration"""
        # Legacy param name support
        if not self.aruco_config_file and self.aruco_pattern_file:
            self.aruco_config_file = self.aruco_pattern_file
        if self.aruco_config_file:
            try:
                with open(self.aruco_config_file, "r") as f:
                    text = f.read()
                    # Support JSON and JSON5 by stripping comments heuristically
                    try:
                        return json.loads(text)
                    except Exception:
                        # crude removal of // and /* */ comments
                        import re

                        text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
                        text = re.sub(r"//.*", "", text)
                        return json.loads(text)
            except Exception as e:
                self.get_logger().warn(
                    f"Failed to load ArUco config file '{self.aruco_config_file}': {e}"
                )

        # Default ArUco pattern (simplified)
        return {
            "markers": [
                {"id": 0, "size": 0.1},  # 10cm markers
                {"id": 1, "size": 0.1},
                {"id": 2, "size": 0.1},
                {"id": 3, "size": 0.1},
            ],
            "board_size": [1.0, 1.0],  # 1m x 1m board
            "marker_spacing": 0.2,  # 20cm spacing
        }

    def _maybe_load_intrinsics_from_yaml(self, yaml_path: str) -> None:
        """Load intrinsics YAML and populate self.camera_info if not already set."""
        try:
            if not os.path.isfile(yaml_path):
                self.get_logger().warn(f"Intrinsics file not found: {yaml_path}")
                return
            with open(yaml_path, "r") as f:
                data = yaml.safe_load(f)
            ci = CameraInfo()
            # image size
            ci.width = int(data.get("image_width", 0))
            ci.height = int(data.get("image_height", 0))
            # camera matrix
            km = data.get("camera_matrix", {}).get("data", [])
            if len(km) == 9:
                ci.k = [float(x) for x in km]
            # distortion
            ci.distortion_model = str(data.get("distortion_model", "plumb_bob"))
            d = data.get("distortion_coefficients", {}).get("data", [])
            ci.d = [float(x) for x in d]
            # rectification
            r = data.get("rectification_matrix", {}).get("data", [])
            if len(r) == 9:
                ci.r = [float(x) for x in r]
            # projection
            p = data.get("projection_matrix", {}).get("data", [])
            if len(p) == 12:
                ci.p = [float(x) for x in p]
            with self.lock:
                if self.camera_info is None:
                    self.camera_info = ci
            self.get_logger().info(
                f"Loaded intrinsics from {yaml_path} - {ci.width}x{ci.height}, model: {ci.distortion_model}"
            )
        except Exception as e:
            self.get_logger().warn(f"Failed to load intrinsics YAML: {e}")

    def camera_info_callback(self, msg: CameraInfo):
        """Handle camera info messages"""
        with self.lock:
            self.camera_info = msg
            self.get_logger().info(
                f"Camera info received - {msg.width}x{msg.height} resolution, "
                f"distortion model: {msg.distortion_model}"
            )

    def aruco_callback(self, msg: Detection2DArray):
        """Handle ArUco detection messages"""
        self.get_logger().info(
            f"ArUco callback - {len(msg.detections)} detections at timestamp "
            f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec}"
        )

        # Only cache non-empty detections
        if msg.detections:
            self.get_logger().info(
                f"Caching non-empty ArUco detection with {len(msg.detections)} markers"
            )

            # Log detailed info about detections
            for i, detection in enumerate(msg.detections):
                self.get_logger().debug(
                    f"  Marker {i}: bbox center=({detection.bbox.center.position.x:.2f}, "
                    f"{detection.bbox.center.position.y:.2f}), size=({detection.bbox.size_x:.2f}, "
                    f"{detection.bbox.size_y:.2f}), ID: {detection.id}"
                )

            with self.lock:
                self.latest_aruco_detection = msg

            # Publish to debug topic
            try:
                self.debug_aruco_publisher.publish(msg)
                self.get_logger().debug("Published ArUco detection to debug topic")
            except Exception as e:
                self.get_logger().warn(f"Failed to publish debug ArUco detection: {e}")
        else:
            self.get_logger().debug("Ignoring empty ArUco detection")

        # Try to process cached detections
        self._try_process_cached_detections()

    def debug_image_callback(self, msg: Image):
        """Cache the latest debug overlay image header for potential sync/monitoring"""
        with self.lock:
            self.latest_debug_image_header = Header()
            self.latest_debug_image_header.stamp = msg.header.stamp
        self.get_logger().debug(
            f"Received debug overlay image at {msg.header.stamp.sec}.{msg.header.stamp.nanosec}"
        )

    def board_callback(self, msg: Detection3DArray):
        """Handle board detection messages"""
        self.get_logger().info(
            f"Board callback - {len(msg.detections)} detections at timestamp "
            f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec}"
        )

        # Only cache non-empty detections
        if msg.detections:
            self.get_logger().info(
                f"Caching non-empty board detection with {len(msg.detections)} boards"
            )

            # Log detailed info about board detections
            for i, detection in enumerate(msg.detections):
                if detection.results:
                    pose = detection.results[0].pose.pose
                    self.get_logger().debug(
                        f"  Board {i}: position=({pose.position.x:.3f}, {pose.position.y:.3f}, "
                        f"{pose.position.z:.3f}), orientation=({pose.orientation.x:.3f}, "
                        f"{pose.orientation.y:.3f}, {pose.orientation.z:.3f}, {pose.orientation.w:.3f})"
                    )

            with self.lock:
                self.latest_board_detection = msg

            # Publish to debug topic
            try:
                self.debug_board_publisher.publish(msg)
                self.get_logger().debug("Published board detection to debug topic")
            except Exception as e:
                self.get_logger().warn(f"Failed to publish debug board detection: {e}")
        else:
            self.get_logger().error("Received EMPTY board detection - NOT caching")

        # Try to process cached detections
        self._try_process_cached_detections()

    def _try_process_cached_detections(self):
        """Try to process cached ArUco and board detections"""
        with self.lock:
            aruco_detection = self.latest_aruco_detection
            board_detection = self.latest_board_detection

        # Check if we have both non-empty cached detections
        if aruco_detection and board_detection:
            self.get_logger().info(
                f"BOTH cached detections available - ArUco: {len(aruco_detection.detections)} markers, "
                f"Board: {len(board_detection.detections)} boards"
            )

            try:
                solved = self._process_detection_pair(aruco_detection, board_detection)
                if solved:
                    self.get_logger().info(
                        "Successfully processed cached detection pair - SOLUTION COMPUTED!"
                    )
            except Exception as e:
                self.get_logger().error(f"Failed to process cached detection pair: {e}")
        else:
            self.get_logger().debug(
                f"Waiting for both detections - ArUco cached: {aruco_detection is not None}, "
                f"Board cached: {board_detection is not None}"
            )

    def _process_detection_pair(
        self, aruco_msg: Detection2DArray, board_msg: Detection3DArray
    ) -> bool:
        """Process a pair of ArUco and board detections. Returns True if a PnP solution was published."""
        self.get_logger().info(
            f"PROCESSING DETECTION PAIR - ArUco: {len(aruco_msg.detections)} detections, "
            f"Board: {len(board_msg.detections)} detections"
        )

        # Check if both detections are present
        if not aruco_msg.detections or not board_msg.detections:
            self.get_logger().warn(
                f"Skipping pair - ArUco empty: {not aruco_msg.detections}, "
                f"Board empty: {not board_msg.detections}"
            )
            return False

        self.get_logger().info("Both ArUco and Board detections present - proceeding")

        # Check if camera info is available
        if not self.camera_info:
            self.get_logger().error(
                "Camera info not available - cannot proceed with calibration"
            )
            return False

        # Convert detections to internal format
        aruco_markers = self._detection2d_to_aruco_markers(aruco_msg)
        board_detection = self._detection3d_to_board_detection(board_msg.detections[0])

        # Create point correspondences
        object_points, image_points = self._create_point_correspondences(
            aruco_markers, board_detection
        )

        if len(object_points) == 0:
            self.get_logger().error(
                "No point correspondences created - cannot solve PnP"
            )
            return

        self.get_logger().info(
            f"Created {len(object_points)} point correspondences for PnP solving"
        )

        # Solve PnP
        success, rvec, tvec = self._solve_pnp(object_points, image_points)

        if success:
            self.get_logger().info(
                f"PnP solver SUCCESS! - translation=({float(tvec.flatten()[0]):.3f}, {float(tvec.flatten()[1]):.3f}, {float(tvec.flatten()[2]):.3f})"
            )

            # Convert to transform message
            transform_msg = self._create_transform_message(rvec, tvec, aruco_msg.header)

            # Publish transform
            try:
                self.transform_publisher.publish(transform_msg)
                self.get_logger().info("Published extrinsic transform")

                # Log the 4x4 transformation matrix
                self._log_transformation_matrix(rvec, tvec)

            except Exception as e:
                self.get_logger().warn(f"Failed to publish transform: {e}")

            # Quality assessment if enabled
            if self.enable_quality_assessment:
                self._assess_quality(
                    object_points, image_points, rvec, tvec, aruco_msg, board_msg
                )
            return True
        else:
            self.get_logger().error("PnP solver FAILED to find solution")
            return False

    def _detection2d_to_aruco_markers(
        self, detection_msg: Detection2DArray
    ) -> List[ArUcoMarker]:
        """Convert Detection2DArray to ArUcoMarker objects"""
        markers = []

        for detection in detection_msg.detections:
            # Extract corner points from bounding box (simplified approach)
            bbox = detection.bbox
            center_x = bbox.center.position.x
            center_y = bbox.center.position.y
            size_x = bbox.size_x
            size_y = bbox.size_y

            # Create 4 corners from bounding box
            corners = [
                (center_x - size_x / 2.0, center_y - size_y / 2.0),
                (center_x + size_x / 2.0, center_y - size_y / 2.0),
                (center_x + size_x / 2.0, center_y + size_y / 2.0),
                (center_x - size_x / 2.0, center_y + size_y / 2.0),
            ]

            # Extract marker ID
            marker_id = detection.id if hasattr(detection, "id") else 0

            markers.append(
                ArUcoMarker(id=marker_id, corners=corners, center=(center_x, center_y))
            )

        return markers

    def _detection3d_to_board_detection(self, detection: Detection3D) -> BoardDetection:
        """Convert Detection3D to BoardDetection object"""
        if not detection.results:
            raise ValueError("No detection results available")

        pose = detection.results[0].pose.pose
        return BoardDetection(
            position=(pose.position.x, pose.position.y, pose.position.z),
            orientation=(
                pose.orientation.w,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
            ),
        )

    def _create_point_correspondences(
        self, aruco_markers: List[ArUcoMarker], board_detection: BoardDetection
    ) -> Tuple[List[np.ndarray], List[np.ndarray]]:
        """Create 3D-2D point correspondences for PnP solving"""
        object_points = []
        image_points = []

        # Get camera matrix from camera info
        camera_matrix = np.array(
            [
                [self.camera_info.k[0], self.camera_info.k[1], self.camera_info.k[2]],
                [self.camera_info.k[3], self.camera_info.k[4], self.camera_info.k[5]],
                [self.camera_info.k[6], self.camera_info.k[7], self.camera_info.k[8]],
            ]
        )

        # For each ArUco marker, create 3D object points and corresponding 2D image points
        for marker in aruco_markers:
            # Create 3D points for this marker (simplified - assuming markers are on a plane)
            marker_size = 0.1  # 10cm markers
            marker_3d_points = np.array(
                [
                    [-marker_size / 2, -marker_size / 2, 0],
                    [marker_size / 2, -marker_size / 2, 0],
                    [marker_size / 2, marker_size / 2, 0],
                    [-marker_size / 2, marker_size / 2, 0],
                ],
                dtype=np.float32,
            )

            # Transform 3D points to board coordinate system
            # This is simplified - in reality you'd need proper board pose transformation
            board_pose = np.array(board_detection.position)
            for point_3d in marker_3d_points:
                # Simple translation (in reality you'd apply full pose transformation)
                transformed_point = point_3d + board_pose
                object_points.append(transformed_point)

            # Add corresponding 2D image points
            for corner in marker.corners:
                image_points.append(np.array([corner[0], corner[1]], dtype=np.float32))

        return object_points, image_points

    def _solve_pnp(
        self, object_points: List[np.ndarray], image_points: List[np.ndarray]
    ) -> Tuple[bool, np.ndarray, np.ndarray]:
        """Solve PnP problem using OpenCV"""
        if len(object_points) < 4:
            return False, None, None

        # Convert to numpy arrays
        obj_pts = np.array(object_points, dtype=np.float32)
        img_pts = np.array(image_points, dtype=np.float32)

        # Get camera matrix and distortion coefficients
        camera_matrix = np.array(
            [
                [self.camera_info.k[0], self.camera_info.k[1], self.camera_info.k[2]],
                [self.camera_info.k[3], self.camera_info.k[4], self.camera_info.k[5]],
                [self.camera_info.k[6], self.camera_info.k[7], self.camera_info.k[8]],
            ]
        )

        dist_coeffs = np.array(self.camera_info.d, dtype=np.float32)

        # Solve PnP using SOLVEPNP_ITERATIVE method
        try:
            success, rvec, tvec = cv2.solvePnP(
                obj_pts,
                img_pts,
                camera_matrix,
                dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )
            return success, rvec, tvec
        except Exception as e:
            self.get_logger().error(f"PnP solving failed: {e}")
            return False, None, None

    def _create_transform_message(
        self, rvec: np.ndarray, tvec: np.ndarray, header: Header
    ) -> TransformStamped:
        """Create TransformStamped message from rotation vector and translation vector"""
        # Convert rotation vector to rotation matrix
        rotation_matrix, _ = cv2.Rodrigues(rvec)

        # Convert rotation matrix to quaternion
        quaternion = self._rotation_matrix_to_quaternion(rotation_matrix)

        # Create transform message
        transform_msg = TransformStamped()
        transform_msg.header = Header()
        transform_msg.header.stamp = header.stamp
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
        """Convert rotation matrix to quaternion (w, x, y, z)"""
        # Use OpenCV's Rodrigues to get rotation vector, then convert to quaternion
        rvec, _ = cv2.Rodrigues(rotation_matrix)
        r = rvec.flatten()

        # Convert rotation vector to quaternion
        angle = np.linalg.norm(r)
        if angle < 1e-6:
            return np.array([1.0, 0.0, 0.0, 0.0])  # Identity quaternion

        axis = r / angle
        half_angle = angle / 2.0

        w = np.cos(half_angle)
        s = np.sin(half_angle)
        x = axis[0] * s
        y = axis[1] * s
        z = axis[2] * s

        return np.array(
            [float(x), float(y), float(z), float(w)]
        )  # Return as (x, y, z, w)

    def _log_transformation_matrix(self, rvec: np.ndarray, tvec: np.ndarray):
        """Log the 4x4 transformation matrix"""
        # Convert rotation vector to rotation matrix
        rotation_matrix, _ = cv2.Rodrigues(rvec)

        # Create 4x4 transformation matrix
        transform_matrix = np.eye(4)
        transform_matrix[:3, :3] = rotation_matrix
        transform_matrix[:3, 3] = tvec.flatten()

        self.get_logger().info(
            f"Extrinsic T (4x4):\n"
            f"[ {transform_matrix[0,0]:.6f} {transform_matrix[0,1]:.6f} {transform_matrix[0,2]:.6f} {transform_matrix[0,3]:.6f} ]\n"
            f"[ {transform_matrix[1,0]:.6f} {transform_matrix[1,1]:.6f} {transform_matrix[1,2]:.6f} {transform_matrix[1,3]:.6f} ]\n"
            f"[ {transform_matrix[2,0]:.6f} {transform_matrix[2,1]:.6f} {transform_matrix[2,2]:.6f} {transform_matrix[2,3]:.6f} ]\n"
            f"[ {transform_matrix[3,0]:.6f} {transform_matrix[3,1]:.6f} {transform_matrix[3,2]:.6f} {transform_matrix[3,3]:.6f} ]"
        )

    def _assess_quality(
        self,
        object_points: List[np.ndarray],
        image_points: List[np.ndarray],
        rvec: np.ndarray,
        tvec: np.ndarray,
        aruco_msg: Detection2DArray,
        board_msg: Detection3DArray,
    ):
        """Assess calibration quality and publish metrics"""
        try:
            # Compute reprojection errors
            camera_matrix = np.array(
                [
                    [
                        self.camera_info.k[0],
                        self.camera_info.k[1],
                        self.camera_info.k[2],
                    ],
                    [
                        self.camera_info.k[3],
                        self.camera_info.k[4],
                        self.camera_info.k[5],
                    ],
                    [
                        self.camera_info.k[6],
                        self.camera_info.k[7],
                        self.camera_info.k[8],
                    ],
                ]
            )
            dist_coeffs = np.array(self.camera_info.d, dtype=np.float32)

            # Project 3D points to image plane
            projected_points, _ = cv2.projectPoints(
                np.array(object_points, dtype=np.float32),
                rvec,
                tvec,
                camera_matrix,
                dist_coeffs,
            )

            # Compute reprojection errors
            errors = []
            for i, (original, projected) in enumerate(
                zip(image_points, projected_points)
            ):
                error = np.linalg.norm(original - projected[0])
                errors.append(error)

            # Compute statistics
            mean_error = np.mean(errors) if errors else float("inf")
            max_error = np.max(errors) if errors else float("inf")
            num_inliers = sum(1 for e in errors if e < 2.0)  # 2 pixel threshold

            # Detection confidence
            expected_detections = 4  # Assuming 4 ArUco markers
            detection_confidence = min(
                len(aruco_msg.detections) / expected_detections, 1.0
            )

            # Create quality report
            quality_report = {
                "overall_quality": max(
                    0.0, 1.0 - mean_error / 10.0
                ),  # Simple quality metric
                "metrics": {
                    "reprojection_error": float(mean_error),
                    "max_reprojection_error": float(max_error),
                    "inlier_ratio": num_inliers / len(errors) if errors else 0.0,
                    "detection_confidence": float(detection_confidence),
                    "num_correspondences": len(object_points),
                },
                "validation": {
                    "is_valid": mean_error < 5.0 and detection_confidence > 0.5
                },
            }

            # Publish quality metrics
            quality_msg = String()
            quality_msg.data = json.dumps(quality_report, indent=2)
            self.quality_publisher.publish(quality_msg)

            self.get_logger().info(
                f"Calibration quality: {quality_report['overall_quality']*100:.1f}%, "
                f"Mean error: {mean_error:.2f}px, Inliers: {num_inliers}/{len(errors)}"
            )

        except Exception as e:
            self.get_logger().warn(f"Quality assessment failed: {e}")


def main(args=None):
    rclpy.init(args=args)

    node = SimpleExtrinsicSolver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
