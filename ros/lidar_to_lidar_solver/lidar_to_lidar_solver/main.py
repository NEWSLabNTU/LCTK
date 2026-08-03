#!/usr/bin/env python3
"""
LiDAR-to-LiDAR Extrinsic Calibration Solver

Subscribes to Detection3DArray from two lidar_board_detector nodes and computes
the transform between the two LiDAR frames.

Synchronization: manual nearest-timestamp matching. Keeps a rolling buffer of
lidar1 detections; on each lidar2 detection finds the closest lidar1 by timestamp.

Conflux was replaced here. Conflux's `try_match` pops oldest-of-each-buffer (FIFO),
which only works for similar-rate streams. With a 6:1 rate mismatch (VLP32 6 Hz vs
Seyond 1 Hz) the buffers drift apart in time (observed dt growing 0.8s -> 4.5s),
the fast buffer saturates under reject_new and freezes on stale data, and the
poll loop starves when the slow buffer drops below 2 messages. Manual nearest-
timestamp pairing avoids all three failure modes.

Transform computation:
    T_lidar2_to_lidar1 = pose1 * pose2.inverse()
"""

import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional, Tuple

import numpy as np
import rclpy
import rclpy.time
from geometry_msgs.msg import Transform, TransformStamped, Quaternion, Vector3
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from scipy.spatial.transform import Rotation
from tf2_ros import TransformBroadcaster
from vision_msgs.msg import Detection3DArray


@dataclass
class SyncStatistics:
    """Statistics for synchronization and calibration."""

    synced_pairs: int = 0
    dropped_no_match: int = 0
    last_timestamp_diff_ms: float = 0.0
    last_translation: Optional[Tuple[float, float, float]] = None
    last_rotation_rpy_deg: Optional[Tuple[float, float, float]] = None


class LidarToLidarSolver(Node):
    """
    ROS 2 node for computing LiDAR-to-LiDAR extrinsic calibration.

    Subscribes to board detection messages from two LiDARs, pairs them by nearest
    timestamp, and computes the transform between the two sensor frames.
    """

    def __init__(self):
        super().__init__("lidar_to_lidar_solver")

        # Declare parameters
        self.declare_parameter("lidar1_detections_topic", "lidar1/board_detections")
        self.declare_parameter("lidar2_detections_topic", "lidar2/board_detections")
        self.declare_parameter("lidar1_frame", "lidar1")
        self.declare_parameter("lidar2_frame", "lidar2")
        self.declare_parameter("sync_tolerance_ms", 0.0)  # 0 = infinite (accept any nearest)
        self.declare_parameter("sync_queue_size", 20)
        self.declare_parameter("sync_drop_policy", "drop_oldest")  # kept for launch compat
        self.declare_parameter("same_face_mode", True)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("max_message_age_ms", 0.0)
        self.declare_parameter("use_best_effort_qos", True)

        # Get parameters
        self.lidar1_topic = self.get_parameter("lidar1_detections_topic").value
        self.lidar2_topic = self.get_parameter("lidar2_detections_topic").value
        self.lidar1_frame = self.get_parameter("lidar1_frame").value
        self.lidar2_frame = self.get_parameter("lidar2_frame").value
        self.sync_tolerance_ms = self.get_parameter("sync_tolerance_ms").value
        sync_queue_size = self.get_parameter("sync_queue_size").value
        self.same_face_mode = self.get_parameter("same_face_mode").value
        self.publish_tf = self.get_parameter("publish_tf").value
        publish_rate_hz = self.get_parameter("publish_rate_hz").value
        self.max_message_age_ms = self.get_parameter("max_message_age_ms").value
        use_best_effort_qos = self.get_parameter("use_best_effort_qos").value

        # State
        self.current_transform: Optional[TransformStamped] = None
        self.stats = SyncStatistics()
        # Rolling buffer of recent lidar1 (fast stream) detections, ordered by arrival.
        self._lidar1_buf: Deque[Detection3DArray] = deque(maxlen=sync_queue_size)
        self._msg_received = {self.lidar1_topic: 0, self.lidar2_topic: 0}
        self._last_transform_time: Optional[float] = None

        # QoS profile configuration based on mode:
        # - BEST_EFFORT (realtime): Low latency, may drop messages
        # - RELIABLE (offline): No message drops, suitable for rosbag playback
        reliability = (
            ReliabilityPolicy.BEST_EFFORT
            if use_best_effort_qos
            else ReliabilityPolicy.RELIABLE
        )
        qos = QoSProfile(
            reliability=reliability,
            history=HistoryPolicy.KEEP_LAST,
            depth=sync_queue_size,
        )

        self.get_logger().info(
            f"Manual nearest-timestamp sync "
            f"(window={'infinite' if self.sync_tolerance_ms <= 0 else f'{self.sync_tolerance_ms:.0f}ms'}, "
            f"buf={sync_queue_size}, QoS={'BEST_EFFORT' if use_best_effort_qos else 'RELIABLE'})"
        )

        # Subscriptions. lidar1 fills the buffer; lidar2 triggers pairing.
        self.create_subscription(
            Detection3DArray, self.lidar1_topic, self._cb_lidar1, qos
        )
        self.create_subscription(
            Detection3DArray, self.lidar2_topic, self._cb_lidar2, qos
        )

        # TF broadcaster
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        # Publisher for transform (in addition to TF)
        self.transform_pub = self.create_publisher(
            TransformStamped, "lidar_to_lidar_transform", 10
        )

        # Timer for continuous TF publishing
        if self.publish_tf and publish_rate_hz > 0:
            self.create_timer(1.0 / publish_rate_hz, self._publish_timer_cb)

        # Periodic debug stats (every 10s)
        self.create_timer(10.0, self._log_stats)

        # Log configuration
        self.get_logger().info(f"LiDAR 1 topic: {self.lidar1_topic} ({self.lidar1_frame})")
        self.get_logger().info(f"LiDAR 2 topic: {self.lidar2_topic} ({self.lidar2_frame})")
        self.get_logger().info(f"Same face mode: {self.same_face_mode}")
        self.get_logger().info("LidarToLidarSolver initialized")

    # -------------------------------------------------------------------------
    # Subscriptions

    def _cb_lidar1(self, msg: Detection3DArray):
        """Fast stream — buffer detections for later pairing."""
        self._msg_received[self.lidar1_topic] += 1
        if msg.detections:
            self._lidar1_buf.append(msg)
        self.get_logger().debug(
            f"RX lidar1 #{self._msg_received[self.lidar1_topic]} "
            f"buf={len(self._lidar1_buf)} "
            f"stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec // 1_000_000:03d}s"
        )

    def _cb_lidar2(self, msg: Detection3DArray):
        """Slow stream — trigger nearest-timestamp pairing."""
        self._msg_received[self.lidar2_topic] += 1
        self.get_logger().debug(
            f"RX lidar2 #{self._msg_received[self.lidar2_topic]} "
            f"buf1={len(self._lidar1_buf)} "
            f"stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec // 1_000_000:03d}s"
        )
        if not msg.detections:
            return
        self._try_pair(msg)

    # -------------------------------------------------------------------------
    # Synchronization

    @staticmethod
    def _stamp_ns(msg: Detection3DArray) -> int:
        s = msg.header.stamp
        return s.sec * 1_000_000_000 + s.nanosec

    def _try_pair(self, lidar2_msg: Detection3DArray):
        """Find nearest lidar1 detection by timestamp and compute the transform."""
        if not self._lidar1_buf:
            self.get_logger().debug("No lidar1 buffered, skipping")
            self.stats.dropped_no_match += 1
            return

        t2_ns = self._stamp_ns(lidar2_msg)

        # Nearest lidar1 by absolute timestamp difference
        best = min(self._lidar1_buf, key=lambda m: abs(self._stamp_ns(m) - t2_ns))
        diff_ms = abs(self._stamp_ns(best) - t2_ns) / 1e6

        if self.sync_tolerance_ms > 0 and diff_ms > self.sync_tolerance_ms:
            self.stats.dropped_no_match += 1
            self.get_logger().debug(
                f"No lidar1 within {self.sync_tolerance_ms:.0f}ms (best={diff_ms:.0f}ms)"
            )
            return

        self._compute_and_publish(best, lidar2_msg, diff_ms)

    # -------------------------------------------------------------------------
    # Transform computation and publishing

    def _compute_and_publish(
        self,
        msg1: Detection3DArray,
        msg2: Detection3DArray,
        diff_ms: float,
    ):
        pose1 = msg1.detections[0].bbox.center
        pose2 = msg2.detections[0].bbox.center

        transform = self.compute_transform(pose1, pose2)
        if transform is None:
            self.get_logger().warn("Failed to compute transform")
            return

        ts = TransformStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = self.lidar1_frame
        ts.child_frame_id = self.lidar2_frame
        ts.transform = transform

        self.current_transform = ts
        self.stats.synced_pairs += 1
        self.stats.last_timestamp_diff_ms = diff_ms
        self._last_transform_time = time.monotonic()

        t = transform.translation
        self.stats.last_translation = (t.x, t.y, t.z)
        q = transform.rotation
        rpy = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler("xyz", degrees=True)
        self.stats.last_rotation_rpy_deg = tuple(rpy)

        self.transform_pub.publish(ts)
        if self.publish_tf:
            self.tf_broadcaster.sendTransform(ts)

        lidar1_short = self.lidar1_topic.split("/")[-2]
        lidar2_short = self.lidar2_topic.split("/")[-2]
        self.get_logger().info(
            f"Calibration #{self.stats.synced_pairs}: "
            f"t=[{t.x:.4f}, {t.y:.4f}, {t.z:.4f}] "
            f"rpy=[{rpy[0]:.2f}, {rpy[1]:.2f}, {rpy[2]:.2f}] deg "
            f"dt={diff_ms:.0f}ms "
            f"stamps: {lidar1_short}={msg1.header.stamp.sec}.{msg1.header.stamp.nanosec // 1_000_000:03d}s "
            f"{lidar2_short}={msg2.header.stamp.sec}.{msg2.header.stamp.nanosec // 1_000_000:03d}s"
        )

    # -------------------------------------------------------------------------
    # Timers

    def _publish_timer_cb(self):
        """Periodically re-broadcast the current transform to TF."""
        if self.current_transform is None:
            return
        self.current_transform.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(self.current_transform)

    def _log_stats(self):
        """Periodic diagnostics: receipt counts, buffer state, transform staleness."""
        lidar1_short = self.lidar1_topic.split("/")[-2]
        lidar2_short = self.lidar2_topic.split("/")[-2]
        rx1 = self._msg_received[self.lidar1_topic]
        rx2 = self._msg_received[self.lidar2_topic]

        stale_str = " | no transform yet"
        if self._last_transform_time is not None:
            gap = time.monotonic() - self._last_transform_time
            stale_str = f" | last_transform={gap:.1f}s ago"
            if gap > 5.0:
                stale_str += " *** STALE ***"

        self.get_logger().info(
            f"[STATS] node_rx: {lidar1_short}={rx1}(buf={len(self._lidar1_buf)}) "
            f"{lidar2_short}={rx2} "
            f"| pairs={self.stats.synced_pairs} no_match={self.stats.dropped_no_match} "
            f"last_dt={self.stats.last_timestamp_diff_ms:.0f}ms{stale_str}"
        )

    # -------------------------------------------------------------------------
    # Math

    def compute_transform(self, pose1, pose2) -> Optional[Transform]:
        """
        Compute transform from LiDAR 2 frame to LiDAR 1 frame.

            pose1 = T * pose2  ->  T = pose1 * pose2.inverse()
        """
        T1 = self.pose_to_matrix(pose1)
        T2 = self.pose_to_matrix(pose2)
        if T1 is None or T2 is None:
            return None

        if self.same_face_mode:
            # Both LiDARs see the same face of the board
            T_rel = T1 @ np.linalg.inv(T2)
        else:
            # LiDARs see opposite faces - need 180 deg rotation around Y
            R_flip = Rotation.from_euler("y", 180, degrees=True).as_matrix()
            T_flip = np.eye(4)
            T_flip[:3, :3] = R_flip
            T_rel = T1 @ T_flip @ np.linalg.inv(T2)

        return self.matrix_to_transform(T_rel)

    def pose_to_matrix(self, pose) -> Optional[np.ndarray]:
        """Convert geometry_msgs/Pose to 4x4 transformation matrix."""
        try:
            t = np.array([pose.position.x, pose.position.y, pose.position.z])
            q = pose.orientation
            R = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = t
            return T
        except Exception as e:
            self.get_logger().error(f"Failed to convert pose to matrix: {e}")
            return None

    def matrix_to_transform(self, T: np.ndarray) -> Transform:
        """Convert 4x4 transformation matrix to geometry_msgs/Transform."""
        transform = Transform()
        transform.translation = Vector3(x=T[0, 3], y=T[1, 3], z=T[2, 3])
        q = Rotation.from_matrix(T[:3, :3]).as_quat()  # [x, y, z, w]
        transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        return transform


def main(args=None):
    rclpy.init(args=args)
    node = LidarToLidarSolver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stats = node.stats
        node.get_logger().info(
            f"Final statistics: synced_pairs={stats.synced_pairs}, "
            f"dropped_no_match={stats.dropped_no_match}"
        )
        if stats.last_translation:
            node.get_logger().info(
                f"Last transform: t={stats.last_translation}, "
                f"rpy_deg={stats.last_rotation_rpy_deg}"
            )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
