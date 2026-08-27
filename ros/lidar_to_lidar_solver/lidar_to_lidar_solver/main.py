#!/usr/bin/env python3
"""
LiDAR-to-LiDAR Extrinsic Calibration Solver

A lightweight ROS 2 node that computes the transform between two LiDAR frames
by observing the same calibration board from both sensors.

The node subscribes to Detection3DArray messages from two lidar_board_detector
nodes, pairs them with `lctk_sync.DetectionPairSource`, and computes the relative
transform.

Transform computation:
    T_lidar2_to_lidar1 = pose1 * pose2.inverse()

Where pose1 and pose2 are the board poses as seen from LiDAR 1 and LiDAR 2
respectively.
"""

import threading
from dataclasses import dataclass
from typing import Any

import numpy as np
import rclpy
from geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from lctk_sync import DetectionPairSource, PairSourceConfig
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation
from tf2_ros import TransformBroadcaster
from vision_msgs.msg import Detection3DArray

from lidar_to_lidar_solver.identity_gate import (
    IdentityComparison,
    IdentityStatus,
    TargetIdentityGate,
    TargetIdentitySubscriptions,
)


@dataclass
class SyncStatistics:
    """Statistics for synchronization and calibration."""

    synced_pairs: int = 0
    dropped_stale: int = 0
    last_timestamp_diff_ms: float = 0.0
    last_translation: tuple[float, float, float] | None = None
    last_rotation_rpy_deg: tuple[float, float, float] | None = None
    identity_rejections: int = 0


class LidarToLidarSolver(Node):
    """
    ROS 2 node for computing LiDAR-to-LiDAR extrinsic calibration.

    Subscribes to board detection messages from two LiDARs, synchronizes them,
    and computes the transform between the two sensor frames.
    """

    def __init__(self):
        super().__init__("lidar_to_lidar_solver")

        # Declare parameters
        self.declare_parameter("lidar1_detections_topic", "lidar1/board_detections")
        self.declare_parameter("lidar2_detections_topic", "lidar2/board_detections")
        self.declare_parameter("lidar1_frame", "lidar1")
        self.declare_parameter("lidar2_frame", "lidar2")
        # The pairing window, in ms. It must be positive: 0 used to mean "infinite",
        # which makes conflux pair by arrival order rather than by time, and two streams
        # at different rates then drift apart without bound. `PairSourceConfig` rejects
        # it rather than let the drift go unnoticed.
        self.declare_parameter("sync_tolerance_ms", 100.0)
        self.declare_parameter("sync_queue_size", 10)
        self.declare_parameter(
            "sync_drop_policy", "reject_new"
        )  # reject_new or drop_oldest
        self.declare_parameter("same_face_mode", True)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("publish_rate_hz", 10.0)
        # M-04: staleness is measured against the node clock. Under rosbag
        # playback without use_sim_time the clock is wall-time while message
        # stamps are recorded time, so any positive threshold drops every pair.
        # Default to 0 (disabled); set > 0 only for live sensors (with a clock
        # that matches the stamps).
        self.declare_parameter("max_message_age_ms", 0.0)
        self.declare_parameter("use_best_effort_qos", True)

        # Get parameters
        self.lidar1_topic = self.get_parameter("lidar1_detections_topic").value
        self.lidar2_topic = self.get_parameter("lidar2_detections_topic").value
        self.lidar1_frame = self.get_parameter("lidar1_frame").value
        self.lidar2_frame = self.get_parameter("lidar2_frame").value
        sync_tolerance_ms = self.get_parameter("sync_tolerance_ms").value
        sync_queue_size = self.get_parameter("sync_queue_size").value
        sync_drop_policy_str = self.get_parameter("sync_drop_policy").value
        self.same_face_mode = self.get_parameter("same_face_mode").value
        self.publish_tf = self.get_parameter("publish_tf").value
        publish_rate_hz = self.get_parameter("publish_rate_hz").value
        self.max_message_age_ms = self.get_parameter("max_message_age_ms").value
        use_best_effort_qos = self.get_parameter("use_best_effort_qos").value

        # State
        self.current_transform: TransformStamped | None = None
        self.stats = SyncStatistics()
        self.target_identity_gate = TargetIdentityGate()
        self.state_lock = threading.RLock()
        self._identity_generation = 0

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
            f"Using {'BEST_EFFORT' if use_best_effort_qos else 'RELIABLE'} QoS"
        )

        # Synchronized detection pairs. `lctk_sync` owns the window (which it refuses to
        # make infinite -- an infinite window pairs by arrival order, not by time), the
        # epoch reset that keeps a replayed bag pairing, and the counters.
        self.pair_source = DetectionPairSource(
            self,
            topics=[self.lidar1_topic, self.lidar2_topic],
            msg_types=[Detection3DArray, Detection3DArray],
            config=PairSourceConfig(
                window_ms=sync_tolerance_ms,
                queue_size=sync_queue_size,
                drop_policy=sync_drop_policy_str,
                require_non_empty=True,
            ),
            qos=qos,
            on_pair=self._handle_sync_group,
            admit_pair=self._admit_sync_group,
            admission_lock=self.state_lock,
        )

        # Detector identities are latched and use dedicated relative endpoints.
        # Keeping these subscriptions alive is essential for late solver joins;
        # launch remaps each endpoint to the corresponding detector namespace.
        self.target_identity_subscriptions = TargetIdentitySubscriptions(
            self,
            self.target_identity_gate,
            on_update=self._handle_target_identity_update,
            update_lock=self.state_lock,
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
            self.publish_timer = self.create_timer(
                1.0 / publish_rate_hz, self.publish_timer_callback
            )

        # Log configuration
        self.get_logger().info(f"LiDAR 1 topic: {self.lidar1_topic}")
        self.get_logger().info(f"LiDAR 2 topic: {self.lidar2_topic}")
        self.get_logger().info(f"LiDAR 1 frame: {self.lidar1_frame}")
        self.get_logger().info(f"LiDAR 2 frame: {self.lidar2_frame}")
        self.get_logger().info(f"Same face mode: {self.same_face_mode}")
        self.get_logger().info("LidarToLidarSolver initialized")

    def _handle_sync_group(self, messages: tuple[Any, ...]):
        """Called for every usable pair, in `topics` order (lidar1, lidar2).

        Empty detection arrays never reach here: `require_non_empty` drops those groups
        in `DetectionPairSource`, which reports them.
        """
        with self.state_lock:
            identity = self.target_identity_gate.compare()
            identity_generation = self._identity_generation
            if not identity.accepted:
                self._reject_identity_pair(identity)
                return

        msg1: Detection3DArray = messages[0]
        msg2: Detection3DArray = messages[1]

        # Check message staleness (wall clock based)
        now = self.get_clock().now()
        msg1_time = rclpy.time.Time.from_msg(msg1.header.stamp)
        msg2_time = rclpy.time.Time.from_msg(msg2.header.stamp)

        age1_ms = (now - msg1_time).nanoseconds / 1e6
        age2_ms = (now - msg2_time).nanoseconds / 1e6

        if self.max_message_age_ms > 0 and (
            age1_ms > self.max_message_age_ms or age2_ms > self.max_message_age_ms
        ):
            self.stats.dropped_stale += 1
            self.get_logger().debug(
                f"Dropped stale messages: age1={age1_ms:.1f}ms, age2={age2_ms:.1f}ms"
            )
            return

        # Calculate timestamp difference for statistics after admission. Keep
        # this local until the generation is checked again below.
        time_diff_ns = abs(msg1_time.nanoseconds - msg2_time.nanoseconds)
        timestamp_diff_ms = time_diff_ns / 1e6

        # Extract poses from detections (take first detection from each)
        det1 = msg1.detections[0]
        det2 = msg2.detections[0]

        pose1 = det1.bbox.center
        pose2 = det2.bbox.center

        # Compute transform
        transform = self.compute_transform(pose1, pose2)

        if transform is None:
            self.get_logger().warn("Failed to compute transform")
            return

        with self.state_lock:
            # An identity update may have invalidated this pair while its
            # transform was being computed. Do not resurrect old output.
            identity = self.target_identity_gate.compare()
            if (
                identity_generation != self._identity_generation
                or not identity.accepted
            ):
                self._reject_identity_pair(identity)
                return

            self.stats.last_timestamp_diff_ms = timestamp_diff_ms

            # Create TransformStamped message
            transform_stamped = TransformStamped()
            transform_stamped.header.stamp = self.get_clock().now().to_msg()
            transform_stamped.header.frame_id = self.lidar1_frame
            transform_stamped.child_frame_id = self.lidar2_frame
            transform_stamped.transform = transform

            # Update state
            self.current_transform = transform_stamped
            self.stats.synced_pairs += 1

            # Store for logging
            t = transform.translation
            self.stats.last_translation = (t.x, t.y, t.z)

            # Convert quaternion to RPY for logging
            q = transform.rotation
            r = Rotation.from_quat([q.x, q.y, q.z, q.w])
            rpy = r.as_euler("xyz", degrees=True)
            self.stats.last_rotation_rpy_deg = tuple(rpy)

            # Publish transform message
            self.transform_pub.publish(transform_stamped)

            # Publish to TF if enabled
            if self.publish_tf:
                self.tf_broadcaster.sendTransform(transform_stamped)

            # Log
            self.get_logger().info(
                f"Calibration #{self.stats.synced_pairs}: "
                f"t=[{t.x:.4f}, {t.y:.4f}, {t.z:.4f}] "
                f"rpy=[{rpy[0]:.2f}, {rpy[1]:.2f}, {rpy[2]:.2f}] deg "
                f"(dt={self.stats.last_timestamp_diff_ms:.1f}ms)"
            )

    def _admit_sync_group(self, _messages: tuple[Any, ...]) -> str | None:
        """Gate a pair before ``DetectionPairSource`` caches it."""
        # DetectionPairSource invokes this callback while holding ``state_lock``
        # (its ``admission_lock``).  Do not acquire it again here: the public
        # source contract accepts any context-manager lock, not only RLock.
        identity = self.target_identity_gate.compare()
        if identity.accepted:
            return None
        self.stats.identity_rejections += 1
        return f"target identity {identity.status.value}: {identity.reason}"

    def _reject_identity_pair(self, identity: IdentityComparison) -> None:
        """Reject a raced pair without changing transform state."""
        self.stats.identity_rejections += 1
        self.pair_source.discard_cached_pair()
        self.get_logger().warn(
            f"Rejected synchronized pair before solver-state mutation: "
            f"target identity {identity.status.value}: {identity.reason}"
        )

    def _handle_target_identity_update(
        self, _lidar_index: int, comparison: IdentityComparison
    ) -> None:
        """Invalidate all solved/cached output after a protocol violation."""
        if comparison.status not in (IdentityStatus.MALFORMED, IdentityStatus.MISMATCH):
            return
        with self.state_lock:
            self._identity_generation += 1
            self.current_transform = None
            self.pair_source.discard_cached_pair()
            self.get_logger().warn(
                "Cleared cached LiDAR-to-LiDAR output after Target Identity "
                f"rejection: {comparison.reason}"
            )

    def compute_transform(self, pose1, pose2) -> Transform | None:
        """
        Compute transform from LiDAR 2 frame to LiDAR 1 frame.

        The calibration board is seen at pose1 in LiDAR 1's frame and at pose2
        in LiDAR 2's frame. The transform T satisfies:
            pose1 = T * pose2
            T = pose1 * pose2.inverse()

        Args:
            pose1: Board pose in LiDAR 1 frame (geometry_msgs/Pose)
            pose2: Board pose in LiDAR 2 frame (geometry_msgs/Pose)

        Returns:
            Transform from LiDAR 2 to LiDAR 1
        """
        # Convert poses to transformation matrices
        T1 = self.pose_to_matrix(pose1)
        T2 = self.pose_to_matrix(pose2)

        if T1 is None or T2 is None:
            return None

        # Compute relative transform
        if self.same_face_mode:
            # Both LiDARs see the same face of the board
            # T_lidar2_to_lidar1 = T1 * T2^(-1)
            T2_inv = np.linalg.inv(T2)
            T_rel = T1 @ T2_inv
        else:
            # LiDARs see opposite faces - need 180° rotation around Y
            R_flip = Rotation.from_euler("y", 180, degrees=True).as_matrix()
            T_flip = np.eye(4)
            T_flip[:3, :3] = R_flip

            T2_inv = np.linalg.inv(T2)
            T_rel = T1 @ T_flip @ T2_inv

        # Convert back to Transform message
        return self.matrix_to_transform(T_rel)

    def pose_to_matrix(self, pose) -> np.ndarray | None:
        """Convert geometry_msgs/Pose to 4x4 transformation matrix."""
        try:
            # Extract translation
            t = np.array([pose.position.x, pose.position.y, pose.position.z])

            # Extract rotation (quaternion to rotation matrix)
            q = pose.orientation
            r = Rotation.from_quat([q.x, q.y, q.z, q.w])
            R = r.as_matrix()

            # Build 4x4 matrix
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = t

            return T
        except Exception as e:  # noqa: BLE001 - a bad pose must return None, not kill the node
            self.get_logger().error(f"Failed to convert pose to matrix: {e}")
            return None

    def matrix_to_transform(self, T: np.ndarray) -> Transform:
        """Convert 4x4 transformation matrix to geometry_msgs/Transform."""
        transform = Transform()

        # Extract translation
        transform.translation = Vector3(x=T[0, 3], y=T[1, 3], z=T[2, 3])

        # Extract rotation (matrix to quaternion)
        R = T[:3, :3]
        r = Rotation.from_matrix(R)
        q = r.as_quat()  # [x, y, z, w]
        transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        return transform

    def publish_timer_callback(self):
        """Periodically publish the current transform to TF."""
        with self.state_lock:
            if self.current_transform is None:
                return

            # Update timestamp
            self.current_transform.header.stamp = self.get_clock().now().to_msg()

            # Publish to TF
            self.tf_broadcaster.sendTransform(self.current_transform)


def main(args=None):
    rclpy.init(args=args)
    node = LidarToLidarSolver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Log final node statistics
        stats = node.stats
        node.get_logger().info(
            f"Final statistics: "
            f"synced_pairs={stats.synced_pairs}, "
            f"dropped_stale={stats.dropped_stale}"
        )
        if stats.last_translation:
            node.get_logger().info(
                f"Last transform: t={stats.last_translation}, "
                f"rpy_deg={stats.last_rotation_rpy_deg}"
            )

        # Log synchronizer statistics
        node.get_logger().info(f"Sync statistics: {node.pair_source.status_line()}")
        if node.pair_source.epoch_resets:
            node.get_logger().info(
                f"Recording restarts handled: {node.pair_source.epoch_resets}"
            )

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
