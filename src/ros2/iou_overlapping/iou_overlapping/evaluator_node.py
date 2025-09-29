import json
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2


def read_extrinsic_json(path: str) -> Tuple[np.ndarray, np.ndarray]:
    with open(path, "r") as f:
        data = json.load(f)
    # Expect either 4x4 matrix or {R: [[..]], t: [..]}
    if "matrix" in data:
        T = np.asarray(data["matrix"], dtype=np.float64)
        R = T[:3, :3]
        t = T[:3, 3]
    else:
        R = np.asarray(data["R"], dtype=np.float64)
        t = np.asarray(data["t"], dtype=np.float64)
    return R, t


def project_points(points_cam: np.ndarray, K: np.ndarray) -> np.ndarray:
    zs = points_cam[:, 2:3]
    valid = zs.squeeze() > 1e-6
    pts = points_cam[valid]
    uv = (K @ pts.T).T
    uv[:, 0] /= uv[:, 2]
    uv[:, 1] /= uv[:, 2]
    return uv[:, :2]


def polygon_mask(h: int, w: int, polygon_xy: np.ndarray) -> np.ndarray:
    mask = np.zeros((h, w), dtype=np.uint8)
    if polygon_xy.shape[0] >= 3:
        cv2.fillPoly(mask, [polygon_xy.astype(np.int32)], 255)
    return mask


def compute_iou(mask_a: np.ndarray, mask_b: np.ndarray) -> float:
    a = mask_a > 0
    b = mask_b > 0
    inter = np.logical_and(a, b).sum()
    union = np.logical_or(a, b).sum()
    if union == 0:
        return 0.0
    return float(inter) / float(union)


def pointcloud2_to_xyz(pc2: PointCloud2) -> np.ndarray:
    # Minimal, assumes fields x,y,z and float32 layout typical for PointCloud2
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


@dataclass
class CachedCalibration:
    K: Optional[np.ndarray] = None
    dist: Optional[np.ndarray] = None
    R: Optional[np.ndarray] = None
    t: Optional[np.ndarray] = None


class IoUEvaluatorNode(Node):
    def __init__(self):
        super().__init__("iou_evaluator")
        self.bridge = CvBridge()
        self.cache = CachedCalibration()

        # Parameters
        self.declare_parameter("extrinsic_json", "")
        self.declare_parameter("board_config", "config/board_detector.json5")

        extrinsic_json = (
            self.get_parameter("extrinsic_json").get_parameter_value().string_value
        )
        if extrinsic_json:
            try:
                R, t = read_extrinsic_json(extrinsic_json)
                self.cache.R, self.cache.t = R, t.reshape(3)
                self.get_logger().info(f"Loaded extrinsic from {extrinsic_json}")
            except Exception as e:
                self.get_logger().error(f"Failed to load extrinsic: {e}")

        # Subscriptions
        self.sub_img = self.create_subscription(
            Image, "/sensing/camera/front_center/synchronized_image", self.on_image, 10
        )
        self.sub_pc = self.create_subscription(
            PointCloud2,
            "/sensing/lidar/top/synchronized_pointcloud",
            self.on_pointcloud,
            10,
        )
        self.sub_info = self.create_subscription(
            CameraInfo, "/sensing/camera/front_center/camera_info", self.on_caminfo, 10
        )

        # Publisher
        self.pub_overlay = self.create_publisher(Image, "/calibration/iou_overlay", 10)

        # State
        self.last_image = None
        self.last_pc = None

    def on_caminfo(self, msg: CameraInfo):
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.cache.K = K
        if msg.d:
            self.cache.dist = np.array(msg.d, dtype=np.float64)
        self.get_logger().info(f"Received camera info: K matrix loaded")

    def on_image(self, msg: Image):
        self.last_image = msg
        self.get_logger().info(f"Received image: {msg.width}x{msg.height}")
        self.try_process()

    def on_pointcloud(self, msg: PointCloud2):
        self.last_pc = msg
        self.get_logger().info(f"Received pointcloud: {msg.width} points")
        self.try_process()

    def detect_board_truth_polygon(self, gray: np.ndarray) -> Optional[np.ndarray]:
        # Detect the large diamond board as the largest near-square rotated rectangle
        h, w = gray.shape

        # Preprocess to emphasize edges of the big board
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 60, 150)
        contours, _ = cv2.findContours(
            edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contours:
            self.get_logger().debug("No contours found for board")
            return None

        best_box = None
        best_score = -1.0
        img_area = float(h * w)

        for cnt in contours:
            if cv2.contourArea(cnt) < 500:  # skip tiny noise
                continue
            rect = cv2.minAreaRect(cnt)  # ((cx,cy),(w,h),angle)
            (cx, cy), (rw, rh), angle = rect
            if rw < 1 or rh < 1:
                continue
            box = cv2.boxPoints(rect)
            box = box.astype(np.float32)
            box_area = rw * rh
            # Prefer big, near-square boxes
            aspect = max(rw, rh) / min(rw, rh)
            area_ratio = box_area / img_area
            # penalize deviation from square and small areas
            score = area_ratio - 0.05 * abs(aspect - 1.0)
            if score > best_score:
                best_score = score
                best_box = box

        if best_box is None:
            self.get_logger().debug("No valid box for board")
            return None

        # Return polygon ordered as int32
        return best_box.reshape(-1, 2).astype(np.float32)

    def try_process(self):
        self.get_logger().info("=== TRY_PROCESS START ===")
        if self.last_image is None or self.last_pc is None:
            self.get_logger().info(
                f"Missing data: image={self.last_image is not None}, pc={self.last_pc is not None}"
            )
            return
        if self.cache.K is None or self.cache.R is None or self.cache.t is None:
            self.get_logger().info(
                f"Missing calibration: K={self.cache.K is not None}, R={self.cache.R is not None}, t={self.cache.t is not None}"
            )
            return

        self.get_logger().info("All data available, starting processing...")

        cv_img = self.bridge.imgmsg_to_cv2(self.last_image, desired_encoding="bgr8")
        h, w = cv_img.shape[:2]
        self.get_logger().info(f"Converted image to OpenCV: {h}x{w}")
        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)

        self.get_logger().info("Starting board detection...")
        truth_poly = self.detect_board_truth_polygon(gray)
        if truth_poly is None:
            # Publish raw image with note so viewer always sees something
            self.get_logger().info("No board detected in image — publishing raw frame")
            out_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
            out_msg.header = self.last_image.header
            self.pub_overlay.publish(out_msg)
            return
        self.get_logger().info(f"Board detected with {len(truth_poly)} vertices")
        truth_mask = polygon_mask(h, w, truth_poly)

        # Project point cloud
        self.get_logger().info("Starting point cloud processing...")
        xyz_lidar = pointcloud2_to_xyz(self.last_pc)
        self.get_logger().info(
            f"Extracted {xyz_lidar.shape[0]} points from point cloud"
        )
        if xyz_lidar.shape[0] == 0:
            self.get_logger().info(
                "No valid points in point cloud — publishing truth-only overlay"
            )
            overlay = cv_img.copy()
            green = np.zeros_like(overlay)
            green[:, :] = (0, 255, 0)
            alpha = 0.5
            overlay = np.where(
                truth_mask[..., None] > 0,
                (alpha * green + (1 - alpha) * overlay).astype(np.uint8),
                overlay,
            )
            cv2.putText(
                overlay,
                "IoU: 0.000 (no points)",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
            out_msg = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
            out_msg.header = self.last_image.header
            self.pub_overlay.publish(out_msg)
            return
        # Transform to camera: X_cam = R * X_lidar + t
        X_cam = (self.cache.R @ xyz_lidar.T).T + self.cache.t.reshape(1, 3)
        self.get_logger().info(f"Transformed {X_cam.shape[0]} points to camera frame")
        uv = project_points(X_cam, self.cache.K)
        self.get_logger().info(f"Projected {uv.shape[0]} points to image plane")
        # Create detected mask by convex hull of projected points within image
        in_img = (uv[:, 0] >= 0) & (uv[:, 0] < w) & (uv[:, 1] >= 0) & (uv[:, 1] < h)
        uv_in = uv[in_img]
        self.get_logger().info(f"Found {uv_in.shape[0]} points within image bounds")
        if uv_in.shape[0] < 3:
            self.get_logger().info(
                f"Not enough projected points in image: {uv_in.shape[0]} — publishing truth-only overlay"
            )
            overlay = cv_img.copy()
            green = np.zeros_like(overlay)
            green[:, :] = (0, 255, 0)
            alpha = 0.5
            overlay = np.where(
                truth_mask[..., None] > 0,
                (alpha * green + (1 - alpha) * overlay).astype(np.uint8),
                overlay,
            )
            cv2.putText(
                overlay,
                "IoU: 0.000 (no projected pts)",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
            out_msg = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
            out_msg.header = self.last_image.header
            self.pub_overlay.publish(out_msg)
            return
        hull = cv2.convexHull(uv_in.astype(np.float32)).reshape(-1, 2)
        self.get_logger().info(f"Created convex hull with {len(hull)} vertices")
        detected_mask = polygon_mask(h, w, hull)

        # IoU
        self.get_logger().info("Computing IoU...")
        iou = compute_iou(truth_mask, detected_mask)
        self.get_logger().info(f"IoU computed: {iou:.3f}")

        # Overlay
        self.get_logger().info("Creating overlay image...")
        overlay = cv_img.copy()
        green = np.zeros_like(overlay)
        green[:, :] = (0, 255, 0)
        red = np.zeros_like(overlay)
        red[:, :] = (0, 0, 255)
        alpha = 0.5
        overlay = np.where(
            truth_mask[..., None] > 0,
            (alpha * green + (1 - alpha) * overlay).astype(np.uint8),
            overlay,
        )
        overlay = np.where(
            detected_mask[..., None] > 0,
            (alpha * red + (1 - alpha) * overlay).astype(np.uint8),
            overlay,
        )
        cv2.putText(
            overlay,
            f"IoU: {iou:.3f}",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

        self.get_logger().info("Publishing overlay image...")
        out_msg = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
        out_msg.header = self.last_image.header
        self.pub_overlay.publish(out_msg)
        self.get_logger().info(f"SUCCESS: Published IoU overlay with IoU={iou:.3f}")
        self.get_logger().info("=== TRY_PROCESS END ===")


def main():
    rclpy.init()
    node = IoUEvaluatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
