#!/usr/bin/env python3
"""
LiDAR-to-LiDAR Extrinsic Calibration Solver

A lightweight ROS 2 node that computes the transform between two LiDAR frames
by observing the same calibration board from both sensors.

The node subscribes to Detection3DArray messages from two lidar_board_detector
nodes, synchronizes them by timestamp, and computes the relative transform.

Transform computation:
    T_lidar2_to_lidar1 = pose1 * pose2.inverse()

Where pose1 and pose2 are the board poses as seen from LiDAR 1 and LiDAR 2
respectively.
"""

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Transform, TransformStamped, Quaternion, Vector3
from message_filters import ApproximateTimeSynchronizer, Subscriber, TimeSynchronizer
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from scipy.spatial.transform import Rotation
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
from vision_msgs.msg import Detection3DArray


@dataclass
class SyncStatistics:
    """Statistics for synchronization and calibration."""

    synced_pairs: int = 0
    dropped_stale: int = 0
    last_timestamp_diff_ms: float = 0.0
    last_translation: Optional[Tuple[float, float, float]] = None
    last_rotation_rpy_deg: Optional[Tuple[float, float, float]] = None


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
        self.declare_parameter("sync_mode", "approximate")  # "exact" or "approximate"
        self.declare_parameter("sync_tolerance_ms", 100.0)
        self.declare_parameter("sync_queue_size", 10)
        self.declare_parameter("same_face_mode", True)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("max_message_age_ms", 500.0)

        # Get parameters
        lidar1_topic = self.get_parameter("lidar1_detections_topic").value
        lidar2_topic = self.get_parameter("lidar2_detections_topic").value
        self.lidar1_frame = self.get_parameter("lidar1_frame").value
        self.lidar2_frame = self.get_parameter("lidar2_frame").value
        sync_mode = self.get_parameter("sync_mode").value
        sync_tolerance_ms = self.get_parameter("sync_tolerance_ms").value
        sync_queue_size = self.get_parameter("sync_queue_size").value
        self.same_face_mode = self.get_parameter("same_face_mode").value
        self.publish_tf = self.get_parameter("publish_tf").value
        publish_rate_hz = self.get_parameter("publish_rate_hz").value
        self.max_message_age_ms = self.get_parameter("max_message_age_ms").value

        # State
        self.current_transform: Optional[TransformStamped] = None
        self.stats = SyncStatistics()

        # QoS profile for sensor data
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=sync_queue_size,
        )

        # Create message_filters subscribers
        self.lidar1_sub = Subscriber(self, Detection3DArray, lidar1_topic, qos_profile=qos)
        self.lidar2_sub = Subscriber(self, Detection3DArray, lidar2_topic, qos_profile=qos)

        # Create synchronizer based on mode
        if sync_mode == "exact":
            self.get_logger().info("Using ExactTime synchronization")
            self.sync = TimeSynchronizer(
                [self.lidar1_sub, self.lidar2_sub],
                queue_size=sync_queue_size,
            )
        else:
            self.get_logger().info(
                f"Using ApproximateTime synchronization (tolerance={sync_tolerance_ms}ms)"
            )
            self.sync = ApproximateTimeSynchronizer(
                [self.lidar1_sub, self.lidar2_sub],
                queue_size=sync_queue_size,
                slop=sync_tolerance_ms / 1000.0,  # Convert to seconds
            )

        self.sync.registerCallback(self.sync_callback)

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
        self.get_logger().info(f"LiDAR 1 topic: {lidar1_topic}")
        self.get_logger().info(f"LiDAR 2 topic: {lidar2_topic}")
        self.get_logger().info(f"LiDAR 1 frame: {self.lidar1_frame}")
        self.get_logger().info(f"LiDAR 2 frame: {self.lidar2_frame}")
        self.get_logger().info(f"Same face mode: {self.same_face_mode}")
        self.get_logger().info("LidarToLidarSolver initialized")

    def sync_callback(self, msg1: Detection3DArray, msg2: Detection3DArray):
        """Called when synchronized detection pairs are received."""
        # Check for valid detections
        if not msg1.detections or not msg2.detections:
            self.get_logger().debug("Received empty detections, skipping")
            return

        # Check message staleness (wall clock based)
        now = self.get_clock().now()
        msg1_time = rclpy.time.Time.from_msg(msg1.header.stamp)
        msg2_time = rclpy.time.Time.from_msg(msg2.header.stamp)

        age1_ms = (now - msg1_time).nanoseconds / 1e6
        age2_ms = (now - msg2_time).nanoseconds / 1e6

        if age1_ms > self.max_message_age_ms or age2_ms > self.max_message_age_ms:
            self.stats.dropped_stale += 1
            self.get_logger().debug(
                f"Dropped stale messages: age1={age1_ms:.1f}ms, age2={age2_ms:.1f}ms"
            )
            return

        # Calculate timestamp difference for statistics
        time_diff_ns = abs(msg1_time.nanoseconds - msg2_time.nanoseconds)
        self.stats.last_timestamp_diff_ms = time_diff_ns / 1e6

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

    def compute_transform(self, pose1, pose2) -> Optional[Transform]:
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

    def pose_to_matrix(self, pose) -> Optional[np.ndarray]:
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
        except Exception as e:
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
        # Log final statistics
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

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
