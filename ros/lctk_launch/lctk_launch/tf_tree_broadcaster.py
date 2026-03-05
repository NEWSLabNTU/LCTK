"""
TF tree broadcaster for calibration pipeline.

Subscribes to solver TransformStamped topics (tree edges only) and
broadcasts each to /tf_static. TF2 natively computes chain transforms,
so we only need to broadcast direct edges.

Usage:
    ros2 run lctk_launch tf_tree_broadcaster \
        --ros-args -p topics:='["/cal/L1_C1/extrinsic_transform", ...]'
"""

import json

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import StaticTransformBroadcaster


class TfTreeBroadcaster(Node):
    """Listens to solver transform topics and broadcasts them to /tf_static."""

    def __init__(self):
        super().__init__("tf_tree_broadcaster")

        # Declare parameter: JSON-encoded list of topics
        self.declare_parameter("topics", "[]")
        topics_json = self.get_parameter("topics").get_parameter_value().string_value
        topics = json.loads(topics_json)

        if not topics:
            self.get_logger().warn("No topics configured — nothing to broadcast")
            return

        self.broadcaster = StaticTransformBroadcaster(self)
        self.latest_transforms: dict[str, TransformStamped] = {}

        # Use transient local durability so we pick up latched transforms
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        for topic in topics:
            self.create_subscription(
                TransformStamped,
                topic,
                self._make_callback(topic),
                qos,
            )
            self.get_logger().info(f"Subscribing to: {topic}")

        self.get_logger().info(
            f"TF tree broadcaster initialized with {len(topics)} topic(s)"
        )

    def _make_callback(self, topic: str):
        """Create a subscription callback bound to a specific topic."""
        def callback(msg: TransformStamped) -> None:
            self._on_transform(topic, msg)
        return callback

    def _on_transform(self, topic: str, msg: TransformStamped) -> None:
        """Handle incoming transform and re-broadcast all known transforms."""
        self.latest_transforms[topic] = msg
        self.get_logger().info(
            f"Received transform on {topic}: "
            f"{msg.header.frame_id} → {msg.child_frame_id}"
        )
        # StaticTransformBroadcaster republishes all transforms each call
        self.broadcaster.sendTransform(list(self.latest_transforms.values()))


def main(args=None):
    rclpy.init(args=args)
    node = TfTreeBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
