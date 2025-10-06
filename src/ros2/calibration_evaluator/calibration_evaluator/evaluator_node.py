"""
Calibration Quality Evaluator Node

Evaluates extrinsic calibration quality by computing IoU between detected
ArUco board regions and projected LiDAR points. Provides real-time feedback
on calibration accuracy.
"""

import json
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from lctk_interfaces.msg import CalibrationMetrics
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Float64, Header, String
from vision_msgs.msg import Detection2DArray


def read_extrinsic_json(path: str) -> Tuple[np.ndarray, np.ndarray]:
    """
    Read extrinsic calibration from JSON file.

    Args:
        path: Path to JSON file containing either 4x4 matrix or {R, t} format

    Returns:
        Tuple of (R, t) where R is 3x3 rotation matrix and t is 3D translation vector
    """
    with open(path, "r") as f:
        data = json.load(f)
    if "matrix" in data:
        T = np.asarray(data["matrix"], dtype=np.float64)
        R = T[:3, :3]
        t = T[:3, 3]
    else:
        R = np.asarray(data["R"], dtype=np.float64)
        t = np.asarray(data["t"], dtype=np.float64)
    return R, t


def transform_stamped_to_matrix(
    transform: TransformStamped,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Convert ROS TransformStamped to rotation matrix and translation vector.

    Args:
        transform: ROS TransformStamped message

    Returns:
        Tuple of (R, t) where R is 3x3 rotation matrix and t is 3D translation vector
    """
    q = transform.transform.rotation
    t_vec = transform.transform.translation

    # Convert quaternion to rotation matrix
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    R = np.array(
        [
            [1 - 2 * (qy**2 + qz**2), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
            [2 * (qx * qy + qw * qz), 1 - 2 * (qx**2 + qz**2), 2 * (qy * qz - qw * qx)],
            [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx**2 + qy**2)],
        ],
        dtype=np.float64,
    )

    t = np.array([t_vec.x, t_vec.y, t_vec.z], dtype=np.float64)

    return R, t


def project_points(points_cam: np.ndarray, K: np.ndarray) -> np.ndarray:
    """
    Project 3D points in camera frame to 2D image coordinates.

    Args:
        points_cam: Nx3 array of 3D points in camera frame
        K: 3x3 camera intrinsic matrix

    Returns:
        Mx2 array of 2D image coordinates (only points with z > 0)
    """
    zs = points_cam[:, 2:3]
    valid = zs.squeeze() > 1e-6
    pts = points_cam[valid]
    if pts.shape[0] == 0:
        return np.zeros((0, 2), dtype=np.float32)
    uv = (K @ pts.T).T
    uv[:, 0] /= uv[:, 2]
    uv[:, 1] /= uv[:, 2]
    return uv[:, :2]


def polygon_mask(h: int, w: int, polygon_xy: np.ndarray) -> np.ndarray:
    """
    Create a binary mask from a polygon.

    Args:
        h: Image height
        w: Image width
        polygon_xy: Nx2 array of polygon vertices

    Returns:
        Binary mask (h, w) with polygon filled
    """
    mask = np.zeros((h, w), dtype=np.uint8)
    if polygon_xy.shape[0] >= 3:
        cv2.fillPoly(mask, [polygon_xy.astype(np.int32)], 255)
    return mask


def pointcloud2_to_xyz(pc2: PointCloud2) -> np.ndarray:
    """
    Extract XYZ coordinates from PointCloud2 message.

    Args:
        pc2: ROS PointCloud2 message

    Returns:
        Nx3 array of XYZ coordinates
    """
    import struct

    xyz = []
    step = pc2.point_step
    offset_x = next(f.offset for f in pc2.fields if f.name == "x")
    offset_y = next(f.offset for f in pc2.fields if f.name == "y")
    offset_z = next(f.offset for f in pc2.fields if f.name == "z")
    for i in range(0, len(pc2.data), step):
        x = struct.unpack_from("f", pc2.data, i + offset_x)[0]
        y = struct.unpack_from("f", pc2.data, i + offset_y)[0]
        z = struct.unpack_from("f", pc2.data, i + offset_z)[0]
        if np.isfinite(x) and np.isfinite(y) and np.isfinite(z):
            xyz.append((x, y, z))
    if not xyz:
        return np.zeros((0, 3), dtype=np.float32)
    return np.asarray(xyz, dtype=np.float32)


def extract_aruco_board_polygon(detections: Detection2DArray) -> Optional[np.ndarray]:
    """
    Extract board polygon from ArUco detections.

    Uses the bounding box of all detected markers to define the board region.

    Args:
        detections: ArUco detection messages

    Returns:
        Nx2 polygon vertices or None if no detections
    """
    if not detections.detections:
        return None

    all_points = []
    for detection in detections.detections:
        bbox = detection.bbox
        cx = bbox.center.position.x
        cy = bbox.center.position.y
        w = bbox.size_x
        h = bbox.size_y

        # Extract 4 corners of bounding box
        corners = np.array(
            [
                [cx - w / 2, cy - h / 2],
                [cx + w / 2, cy - h / 2],
                [cx + w / 2, cy + h / 2],
                [cx - w / 2, cy + h / 2],
            ]
        )
        all_points.extend(corners)

    if not all_points:
        return None

    # Compute convex hull of all marker corners to get overall board region
    all_points = np.array(all_points, dtype=np.float32)
    hull = cv2.convexHull(all_points)
    return hull.reshape(-1, 2)


class CalibrationEvaluatorNode(Node):
    """
    ROS 2 node for evaluating calibration quality using IoU metrics.

    Subscribes to camera images, point clouds, ArUco detections, and extrinsic
    transforms to compute calibration quality metrics in real-time.
    """

    def __init__(self):
        super().__init__("calibration_evaluator")
        self.bridge = CvBridge()

        # Cached calibration data
        self.K: Optional[np.ndarray] = None
        self.dist: Optional[np.ndarray] = None
        self.R: Optional[np.ndarray] = None
        self.t: Optional[np.ndarray] = None

        # Parameters
        self.declare_parameter("extrinsic_json", "")
        self.declare_parameter("use_best_effort_qos", True)
        self.declare_parameter("sync_queue_size", 10)
        self.declare_parameter("sync_slop", 0.1)

        extrinsic_json = (
            self.get_parameter("extrinsic_json").get_parameter_value().string_value
        )
        use_best_effort = (
            self.get_parameter("use_best_effort_qos").get_parameter_value().bool_value
        )
        sync_queue_size = (
            self.get_parameter("sync_queue_size").get_parameter_value().integer_value
        )
        sync_slop = self.get_parameter("sync_slop").get_parameter_value().double_value

        # Load static extrinsic if provided
        if extrinsic_json:
            try:
                R, t = read_extrinsic_json(extrinsic_json)
                self.R, self.t = R, t.reshape(3)
                self.get_logger().info(f"Loaded static extrinsic from {extrinsic_json}")
            except Exception as e:
                self.get_logger().error(f"Failed to load extrinsic: {e}")

        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=(
                ReliabilityPolicy.BEST_EFFORT
                if use_best_effort
                else ReliabilityPolicy.RELIABLE
            ),
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        # Derive camera_info topic from image topic
        # Convention: replace last component with "camera_info"
        # This will be remapped at launch time
        self.sub_info = self.create_subscription(
            CameraInfo, "camera_info", self.on_caminfo, sensor_qos
        )

        # Extrinsic transform subscription (live calibration)
        self.sub_extrinsic = self.create_subscription(
            TransformStamped, "extrinsic_transform", self.on_extrinsic_transform, 10
        )

        # Synchronized subscriptions for image, pointcloud, and ArUco detections
        self.sub_img = Subscriber(self, Image, "image", qos_profile=sensor_qos)
        self.sub_pc = Subscriber(
            self, PointCloud2, "pointcloud", qos_profile=sensor_qos
        )
        self.sub_aruco = Subscriber(
            self, Detection2DArray, "aruco_detections", qos_profile=10
        )

        # Synchronize messages
        self.sync = ApproximateTimeSynchronizer(
            [self.sub_img, self.sub_pc, self.sub_aruco],
            queue_size=sync_queue_size,
            slop=sync_slop,
        )
        self.sync.registerCallback(self.on_synchronized_data)

        # Publishers
        self.pub_iou = self.create_publisher(Float64, "~/iou_score", 10)
        self.pub_metrics = self.create_publisher(CalibrationMetrics, "~/metrics", 10)
        self.pub_overlay = self.create_publisher(Image, "~/overlay_image", 10)
        self.pub_status = self.create_publisher(String, "~/status", 10)

        self.get_logger().info("Calibration Evaluator Node initialized")
        self.get_logger().info(
            f"Sync queue size: {sync_queue_size}, slop: {sync_slop}s"
        )

    def on_caminfo(self, msg: CameraInfo):
        """Handle camera info updates."""
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.K = K
        if msg.d:
            self.dist = np.array(msg.d, dtype=np.float64)
        self.get_logger().info("Camera info received", once=True)

    def on_extrinsic_transform(self, msg: TransformStamped):
        """Handle live extrinsic transform updates."""
        try:
            R, t = transform_stamped_to_matrix(msg)
            self.R = R
            self.t = t
            self.get_logger().info(
                "Extrinsic transform updated", throttle_duration_sec=5.0
            )
        except Exception as e:
            self.get_logger().error(f"Failed to parse extrinsic transform: {e}")

    def on_synchronized_data(
        self, img_msg: Image, pc_msg: PointCloud2, aruco_msg: Detection2DArray
    ):
        """
        Process synchronized image, pointcloud, and ArUco detections.

        Computes IoU and calibration metrics, publishes results.
        """
        # Check if calibration data is available
        if self.K is None:
            self.get_logger().warn(
                "Camera intrinsics not available", throttle_duration_sec=2.0
            )
            return

        if self.R is None or self.t is None:
            self.get_logger().warn(
                "Extrinsic calibration not available", throttle_duration_sec=2.0
            )
            return

        # Convert image
        cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        h, w = cv_img.shape[:2]

        # Extract ground truth board region from ArUco detections
        ground_truth_poly = extract_aruco_board_polygon(aruco_msg)

        if ground_truth_poly is None or len(ground_truth_poly) < 3:
            self.publish_no_board_status(img_msg.header, cv_img)
            return

        # Create ground truth mask
        truth_mask = polygon_mask(h, w, ground_truth_poly)
        truth_area = np.sum(truth_mask > 0)

        # Extract point cloud
        xyz_lidar = pointcloud2_to_xyz(pc_msg)

        if xyz_lidar.shape[0] == 0:
            self.publish_no_points_status(
                img_msg.header, cv_img, truth_mask, ground_truth_poly
            )
            return

        # Transform to camera frame: X_cam = R * X_lidar + t
        X_cam = (self.R @ xyz_lidar.T).T + self.t.reshape(1, 3)

        # Project to image
        uv = project_points(X_cam, self.K)

        # Filter points within image bounds
        in_img = (uv[:, 0] >= 0) & (uv[:, 0] < w) & (uv[:, 1] >= 0) & (uv[:, 1] < h)
        uv_in = uv[in_img]

        if uv_in.shape[0] < 3:
            self.publish_insufficient_points_status(
                img_msg.header, cv_img, truth_mask, ground_truth_poly, uv_in.shape[0]
            )
            return

        # Create projected mask from convex hull
        hull = cv2.convexHull(uv_in.astype(np.float32)).reshape(-1, 2)
        detected_mask = polygon_mask(h, w, hull)
        detected_area = np.sum(detected_mask > 0)

        # Compute metrics
        intersection = np.logical_and(truth_mask > 0, detected_mask > 0)
        union = np.logical_or(truth_mask > 0, detected_mask > 0)

        intersection_area = np.sum(intersection)
        union_area = np.sum(union)

        iou = float(intersection_area) / float(union_area) if union_area > 0 else 0.0
        coverage = (
            float(intersection_area) / float(truth_area) if truth_area > 0 else 0.0
        )
        precision = (
            float(intersection_area) / float(detected_area)
            if detected_area > 0
            else 0.0
        )

        # Count inlier points (points within ground truth region)
        inlier_count = 0
        for pt in uv_in:
            x, y = int(pt[0]), int(pt[1])
            if 0 <= y < h and 0 <= x < w and truth_mask[y, x] > 0:
                inlier_count += 1

        # Publish results
        self.publish_results(
            img_msg.header,
            cv_img,
            iou,
            coverage,
            precision,
            uv_in.shape[0],
            inlier_count,
            truth_area,
            detected_area,
            intersection_area,
            union_area,
            truth_mask,
            detected_mask,
            ground_truth_poly,
            hull,
        )

    def publish_results(
        self,
        header: Header,
        img: np.ndarray,
        iou: float,
        coverage: float,
        precision: float,
        projected_count: int,
        inlier_count: int,
        truth_area: float,
        detected_area: float,
        intersection_area: float,
        union_area: float,
        truth_mask: np.ndarray,
        detected_mask: np.ndarray,
        ground_truth_poly: np.ndarray,
        projected_hull: np.ndarray,
    ):
        """Publish all output messages with computed metrics."""
        # IoU score
        iou_msg = Float64()
        iou_msg.data = iou
        self.pub_iou.publish(iou_msg)

        # Detailed metrics
        metrics_msg = CalibrationMetrics()
        metrics_msg.header = header
        metrics_msg.iou = iou
        metrics_msg.coverage = coverage
        metrics_msg.precision = precision
        metrics_msg.projected_point_count = projected_count
        metrics_msg.inlier_point_count = inlier_count
        metrics_msg.ground_truth_area = float(truth_area)
        metrics_msg.projected_area = float(detected_area)
        metrics_msg.intersection_area = float(intersection_area)
        metrics_msg.union_area = float(union_area)
        metrics_msg.status = "OK"
        self.pub_metrics.publish(metrics_msg)

        # Status
        status_msg = String()
        status_msg.data = (
            f"OK: IoU={iou:.3f}, Coverage={coverage:.3f}, Precision={precision:.3f}"
        )
        self.pub_status.publish(status_msg)

        # Overlay visualization
        overlay = self.create_overlay(
            img,
            truth_mask,
            detected_mask,
            ground_truth_poly,
            projected_hull,
            iou,
            coverage,
            precision,
        )
        overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
        overlay_msg.header = header
        self.pub_overlay.publish(overlay_msg)

        self.get_logger().info(
            f"Metrics: IoU={iou:.3f}, Coverage={coverage:.3f}, Precision={precision:.3f}, "
            f"Points={projected_count}, Inliers={inlier_count}",
            throttle_duration_sec=1.0,
        )

    def publish_no_board_status(self, header: Header, img: np.ndarray):
        """Publish status when no board is detected."""
        status_msg = String()
        status_msg.data = "No ArUco board detected"
        self.pub_status.publish(status_msg)

        overlay = img.copy()
        cv2.putText(
            overlay,
            "No ArUco board detected",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )
        overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
        overlay_msg.header = header
        self.pub_overlay.publish(overlay_msg)

        self.get_logger().warn("No ArUco board detected", throttle_duration_sec=2.0)

    def publish_no_points_status(
        self,
        header: Header,
        img: np.ndarray,
        truth_mask: np.ndarray,
        ground_truth_poly: np.ndarray,
    ):
        """Publish status when no valid points are available."""
        metrics_msg = CalibrationMetrics()
        metrics_msg.header = header
        metrics_msg.iou = 0.0
        metrics_msg.coverage = 0.0
        metrics_msg.precision = 0.0
        metrics_msg.projected_point_count = 0
        metrics_msg.inlier_point_count = 0
        metrics_msg.ground_truth_area = float(np.sum(truth_mask > 0))
        metrics_msg.projected_area = 0.0
        metrics_msg.intersection_area = 0.0
        metrics_msg.union_area = float(np.sum(truth_mask > 0))
        metrics_msg.status = "No valid points"
        self.pub_metrics.publish(metrics_msg)

        status_msg = String()
        status_msg.data = "No valid points in point cloud"
        self.pub_status.publish(status_msg)

        overlay = self.create_overlay_with_ground_truth_only(
            img, truth_mask, ground_truth_poly, "No valid points"
        )
        overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
        overlay_msg.header = header
        self.pub_overlay.publish(overlay_msg)

    def publish_insufficient_points_status(
        self,
        header: Header,
        img: np.ndarray,
        truth_mask: np.ndarray,
        ground_truth_poly: np.ndarray,
        point_count: int,
    ):
        """Publish status when insufficient points are projected."""
        metrics_msg = CalibrationMetrics()
        metrics_msg.header = header
        metrics_msg.iou = 0.0
        metrics_msg.coverage = 0.0
        metrics_msg.precision = 0.0
        metrics_msg.projected_point_count = point_count
        metrics_msg.inlier_point_count = 0
        metrics_msg.ground_truth_area = float(np.sum(truth_mask > 0))
        metrics_msg.projected_area = 0.0
        metrics_msg.intersection_area = 0.0
        metrics_msg.union_area = float(np.sum(truth_mask > 0))
        metrics_msg.status = f"Insufficient points ({point_count})"
        self.pub_metrics.publish(metrics_msg)

        status_msg = String()
        status_msg.data = f"Insufficient projected points: {point_count}"
        self.pub_status.publish(status_msg)

        overlay = self.create_overlay_with_ground_truth_only(
            img, truth_mask, ground_truth_poly, f"Insufficient points ({point_count})"
        )
        overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
        overlay_msg.header = header
        self.pub_overlay.publish(overlay_msg)

    def create_overlay(
        self,
        img: np.ndarray,
        truth_mask: np.ndarray,
        detected_mask: np.ndarray,
        ground_truth_poly: np.ndarray,
        projected_hull: np.ndarray,
        iou: float,
        coverage: float,
        precision: float,
    ) -> np.ndarray:
        """Create visualization overlay with metrics."""
        overlay = img.copy()

        # Draw ground truth in green (semi-transparent)
        green_overlay = overlay.copy()
        green_overlay[truth_mask > 0] = (0, 255, 0)
        cv2.addWeighted(green_overlay, 0.3, overlay, 0.7, 0, overlay)

        # Draw projected region in red (semi-transparent)
        red_overlay = overlay.copy()
        red_overlay[detected_mask > 0] = (0, 0, 255)
        cv2.addWeighted(red_overlay, 0.3, overlay, 0.7, 0, overlay)

        # Draw polygon outlines
        cv2.polylines(
            overlay, [ground_truth_poly.astype(np.int32)], True, (0, 255, 0), 2
        )
        cv2.polylines(overlay, [projected_hull.astype(np.int32)], True, (0, 0, 255), 2)

        # Draw metrics text
        y_offset = 40
        cv2.putText(
            overlay,
            f"IoU: {iou:.3f}",
            (20, y_offset),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
        y_offset += 35
        cv2.putText(
            overlay,
            f"Coverage: {coverage:.3f}",
            (20, y_offset),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
        y_offset += 35
        cv2.putText(
            overlay,
            f"Precision: {precision:.3f}",
            (20, y_offset),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

        # Legend
        cv2.putText(
            overlay,
            "Green: Ground Truth",
            (20, img.shape[0] - 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            overlay,
            "Red: Projected",
            (20, img.shape[0] - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )

        return overlay

    def create_overlay_with_ground_truth_only(
        self,
        img: np.ndarray,
        truth_mask: np.ndarray,
        ground_truth_poly: np.ndarray,
        message: str,
    ) -> np.ndarray:
        """Create overlay showing only ground truth with error message."""
        overlay = img.copy()

        # Draw ground truth in green
        green_overlay = overlay.copy()
        green_overlay[truth_mask > 0] = (0, 255, 0)
        cv2.addWeighted(green_overlay, 0.3, overlay, 0.7, 0, overlay)

        cv2.polylines(
            overlay, [ground_truth_poly.astype(np.int32)], True, (0, 255, 0), 2
        )

        # Draw error message
        cv2.putText(
            overlay,
            message,
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )

        return overlay


def main():
    rclpy.init()
    node = CalibrationEvaluatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
