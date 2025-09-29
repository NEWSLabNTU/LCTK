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

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np  # Ubuntu 22.04 default (1.x)
import cv2          # OpenCV for computer vision tasks
import yaml
import os
from typing import List, Tuple, Optional
from dataclasses import dataclass
import threading

# ROS2 message types
from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Transform, TransformStamped, Vector3, Quaternion
from vision_msgs.msg import Detection2DArray, Detection2D, Detection3DArray, Detection3D


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
    center: Tuple[float, float]         # Center point in pixels


@dataclass
class BoardDetection:
    """
    Represents a calibration board detection in 3D LiDAR coordinates.

    Educational note: Board pose provides the 3D reference frame for
    transforming marker coordinates from local to world space. The board
    serves as a common reference object visible to both LiDAR and camera.
    """
    position: Tuple[float, float, float]        # x, y, z in meters (LiDAR frame)
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

        # Parameter declaration (compatible with existing launch files)
        self.declare_parameter("parent_frame", "lidar")
        self.declare_parameter("child_frame", "camera")
        self.declare_parameter("camera_topic", "")
        self.declare_parameter("intrinsics_file", "")

        # Educational note: Accept additional parameters for launch file compatibility
        # These maintain compatibility with existing launch files but are ignored in educational version
        self.declare_parameter("solver_method", "SQPNP")  # Educational: We use OpenCV solvePnP
        self.declare_parameter("min_detections_required", 1)
        self.declare_parameter("max_solver_iterations", 1000)
        self.declare_parameter("convergence_threshold", 1e-6)
        self.declare_parameter("debug_mode", True)  # Educational: Always educational
        self.declare_parameter("enable_quality_assessment", False)  # Educational: Removed for simplicity

        # Get parameters with simple error handling
        self.parent_frame = (
            self.get_parameter("parent_frame").get_parameter_value().string_value
        )
        self.child_frame = (
            self.get_parameter("child_frame").get_parameter_value().string_value
        )

        # Educational note: Cache latest detections for processing
        # We use simple variables instead of complex synchronization
        self.latest_aruco_detection: Optional[Detection2DArray] = None
        self.latest_board_detection: Optional[Detection3DArray] = None
        self.camera_info: Optional[CameraInfo] = None

        # Thread safety for simple caching
        self.lock = threading.Lock()

        # Load camera intrinsics if provided
        intrinsics_file = (
            self.get_parameter("intrinsics_file").get_parameter_value().string_value
        )
        if intrinsics_file and os.path.exists(intrinsics_file):
            self._load_camera_intrinsics(intrinsics_file)

        # QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Publishers - only essential output
        self.transform_publisher = self.create_publisher(
            TransformStamped, "extrinsic_transform", qos_profile
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
        solver_method = self.get_parameter("solver_method").get_parameter_value().string_value
        min_detections = self.get_parameter("min_detections_required").get_parameter_value().integer_value

        self.get_logger().info(
            f"Educational Extrinsic Solver initialized\n"
            f"Educational Mode: Simplified PnP calibration using OpenCV\n"
            f"Subscribing to: aruco_detections, calibration_board_detections, {camera_info_topic}\n"
            f"Publishing to: extrinsic_transform\n"
            f"Transform: {self.parent_frame} -> {self.child_frame}\n"
            f"Launch file compatibility: solver_method={solver_method} (using cv2.solvePnP), "
            f"min_detections={min_detections}"
        )

    def _load_camera_intrinsics(self, yaml_path: str) -> None:
        """
        Load camera intrinsics from YAML file.

        Educational note: Camera intrinsics define the internal geometry
        of the camera including focal length, principal point, and distortion.
        These parameters are needed for the PnP problem.
        """
        try:
            with open(yaml_path, "r") as f:
                data = yaml.safe_load(f)

            # Create CameraInfo message from YAML data
            ci = CameraInfo()
            ci.width = int(data.get("image_width", 0))
            ci.height = int(data.get("image_height", 0))

            # Camera matrix K (3x3) - most important for PnP
            # K = [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
            km = data.get("camera_matrix", {}).get("data", [])
            if len(km) == 9:
                ci.k = [float(x) for x in km]

            # Distortion coefficients (typically 4, 5, or 8 parameters)
            ci.distortion_model = str(data.get("distortion_model", "plumb_bob"))
            d = data.get("distortion_coefficients", {}).get("data", [])
            ci.d = [float(x) for x in d]

            with self.lock:
                self.camera_info = ci

            self.get_logger().info(
                f"Loaded camera intrinsics: {ci.width}x{ci.height}, "
                f"fx={ci.k[0]:.1f}, fy={ci.k[4]:.1f}, "
                f"cx={ci.k[2]:.1f}, cy={ci.k[5]:.1f}"
            )
        except Exception as e:
            self.get_logger().warn(f"Failed to load intrinsics: {e}")

    def camera_info_callback(self, msg: CameraInfo):
        """
        Handle camera info messages.

        Educational note: Camera info provides the intrinsic parameters
        needed for PnP solving. This includes the camera matrix K and
        distortion coefficients.
        """
        with self.lock:
            self.camera_info = msg
            self.get_logger().debug(f"Camera info received: {msg.width}x{msg.height}")

    def aruco_callback(self, msg: Detection2DArray):
        """
        Handle ArUco detection messages.

        Educational note: ArUco markers provide precise 2D corner detections
        that correspond to known 3D marker geometry. These 2D-3D correspondences
        are essential for solving the PnP problem.
        """
        self.get_logger().debug(
            f"ArUco detection: {len(msg.detections)} markers at "
            f"t={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}"
        )

        # Only cache non-empty detections
        if msg.detections:
            with self.lock:
                self.latest_aruco_detection = msg

            # Try to process if we have both detection types
            self._try_solve_calibration()
        else:
            self.get_logger().debug("Ignoring empty ArUco detection")

    def board_callback(self, msg: Detection3DArray):
        """
        Handle board detection messages.

        Educational note: Board detections provide the 3D pose of the
        calibration board in LiDAR coordinates. This pose is used to
        transform marker coordinates from local to world space.
        """
        self.get_logger().debug(
            f"Board detection: {len(msg.detections)} boards at "
            f"t={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}"
        )

        # Only cache non-empty detections
        if msg.detections:
            with self.lock:
                self.latest_board_detection = msg

            # Try to process if we have both detection types
            self._try_solve_calibration()
        else:
            self.get_logger().warn("Received empty board detection")

    def _try_solve_calibration(self):
        """
        Attempt to solve calibration if both detection types are available.

        Educational note: We need both ArUco (2D) and board (3D) detections
        to create the point correspondences required for PnP solving.
        """
        with self.lock:
            aruco_msg = self.latest_aruco_detection
            board_msg = self.latest_board_detection

        # Check if we have both detection types
        if aruco_msg and board_msg:
            self.get_logger().info(
                f"Processing detection pair: {len(aruco_msg.detections)} ArUco markers, "
                f"{len(board_msg.detections)} boards"
            )

            try:
                self._solve_extrinsic_calibration(aruco_msg, board_msg)
            except Exception as e:
                self.get_logger().error(f"Calibration failed: {e}")
        else:
            missing = []
            if not aruco_msg:
                missing.append("ArUco")
            if not board_msg:
                missing.append("Board")
            self.get_logger().debug(f"Waiting for detections: missing {', '.join(missing)}")

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

        Educational note: This converts from ROS message format to our
        internal representation for easier processing. We extract the
        bounding box and convert to corner coordinates.
        """
        markers = []

        for detection in detection_msg.detections:
            # Extract bounding box information
            bbox = detection.bbox
            center_x = bbox.center.position.x
            center_y = bbox.center.position.y
            size_x = bbox.size_x
            size_y = bbox.size_y

            # Convert bounding box to 4 corner points
            # Educational note: ArUco detectors typically provide corner coordinates,
            # but this simplified version reconstructs from bounding box
            corners = [
                (center_x - size_x / 2.0, center_y - size_y / 2.0),  # Top-left
                (center_x + size_x / 2.0, center_y - size_y / 2.0),  # Top-right
                (center_x + size_x / 2.0, center_y + size_y / 2.0),  # Bottom-right
                (center_x - size_x / 2.0, center_y + size_y / 2.0),  # Bottom-left
            ]

            # Extract marker ID
            marker_id = detection.id if hasattr(detection, "id") else 0

            markers.append(
                ArUcoMarker(id=marker_id, corners=corners, center=(center_x, center_y))
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
                pose.orientation.w,  # Note: OpenCV uses w-first quaternions
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
            ),
        )

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

        # Standard ArUco marker size (educational assumption)
        marker_size = 0.1  # 10cm square markers

        self.get_logger().info(
            f"Creating correspondences for {len(aruco_markers)} markers "
            f"with board at position {board_detection.position}"
        )

        for i, marker in enumerate(aruco_markers):
            # Step 1: Define 4 corners in marker's local coordinate system
            # Educational note: Consistent corner ordering is critical for PnP
            # We use the standard ArUco corner ordering: TL, TR, BR, BL
            local_corners = np.array([
                [-marker_size/2, -marker_size/2, 0],  # Top-left
                [ marker_size/2, -marker_size/2, 0],  # Top-right
                [ marker_size/2,  marker_size/2, 0],  # Bottom-right
                [-marker_size/2,  marker_size/2, 0]   # Bottom-left
            ], dtype=np.float32)

            # Step 2: Transform to world coordinates using board pose
            # Educational simplification: assume markers are coplanar with board
            # In a full implementation, this would use the complete rigid transformation
            board_position = np.array(board_detection.position, dtype=np.float32)

            # Simple translation (could be extended to full rigid transformation)
            world_corners = local_corners + board_position

            # Step 3: Add to correspondence lists
            object_points.extend(world_corners)

            # Add corresponding 2D image points
            image_corners = np.array(marker.corners, dtype=np.float32)
            image_points.extend(image_corners)

            self.get_logger().debug(
                f"Marker {i}: 4 corners at board offset {board_position} "
                f"-> image center ({marker.center[0]:.1f}, {marker.center[1]:.1f})"
            )

        return np.array(object_points, dtype=np.float32), np.array(image_points, dtype=np.float32)

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

        # Extract distortion coefficients
        # Educational note: Distortion coefficients correct for lens imperfections
        dist_coeffs = np.array(self.camera_info.d, dtype=np.float32)

        self.get_logger().info(
            f"Solving PnP with {len(object_points)} correspondences\n"
            f"Camera matrix K:\n{K}\n"
            f"Distortion coefficients: {dist_coeffs[:4] if len(dist_coeffs) >= 4 else dist_coeffs}"
        )

        try:
            # Solve PnP using OpenCV's iterative method
            # Educational note: ITERATIVE method is robust and educational
            success, rvec, tvec = cv2.solvePnP(
                object_points,      # 3D object points (Nx3)
                image_points,       # 2D image points (Nx2)
                K,                  # Camera intrinsic matrix (3x3)
                dist_coeffs,        # Distortion coefficients
                flags=cv2.SOLVEPNP_ITERATIVE
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
            w=float(quaternion[3])
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
    rclpy.init(args=args)

    node = EducationalExtrinsicSolver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down educational extrinsic solver")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()