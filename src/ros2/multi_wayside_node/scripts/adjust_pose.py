#!/usr/bin/env python3
"""
Simple script to send pose adjustments to the multi_wayside_node.
This demonstrates the manual adjustment functionality.

Usage:
    python3 adjust_pose.py --lidar 1 --x 0.1 --y 0.2 --z 0.0
    python3 adjust_pose.py --lidar 2 --yaw 0.1
"""

import argparse
import math

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from rclpy.node import Node
from tf_transformations import quaternion_from_euler


class PoseAdjuster(Node):
    def __init__(self):
        super().__init__("pose_adjuster")
        self.publishers = {
            1: self.create_publisher(PoseStamped, "/lidar1/board_pose_adjustment", 10),
            2: self.create_publisher(PoseStamped, "/lidar2/board_pose_adjustment", 10),
        }

    def send_adjustment(
        self, lidar_id, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0
    ):
        """Send a pose adjustment for the specified LiDAR."""
        if lidar_id not in self.publishers:
            self.get_logger().error(f"Invalid LiDAR ID: {lidar_id}")
            return

        # Create pose message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = f"lidar{lidar_id}"

        # Set position
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z

        # Convert Euler angles to quaternion
        q = quaternion_from_euler(roll, pitch, yaw)
        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]

        # Publish the adjustment
        self.publishers[lidar_id].publish(pose_msg)
        self.get_logger().info(
            f"Sent pose adjustment for LiDAR {lidar_id}: "
            f"pos=({x:.3f}, {y:.3f}, {z:.3f}), "
            f"rot=({roll:.3f}, {pitch:.3f}, {yaw:.3f})"
        )


def main():
    parser = argparse.ArgumentParser(
        description="Send pose adjustments to multi_wayside_node"
    )
    parser.add_argument(
        "--lidar", type=int, choices=[1, 2], required=True, help="LiDAR ID (1 or 2)"
    )
    parser.add_argument("--x", type=float, default=0.0, help="X translation (m)")
    parser.add_argument("--y", type=float, default=0.0, help="Y translation (m)")
    parser.add_argument("--z", type=float, default=0.0, help="Z translation (m)")
    parser.add_argument("--roll", type=float, default=0.0, help="Roll rotation (rad)")
    parser.add_argument("--pitch", type=float, default=0.0, help="Pitch rotation (rad)")
    parser.add_argument("--yaw", type=float, default=0.0, help="Yaw rotation (rad)")
    parser.add_argument("--yaw-deg", type=float, help="Yaw rotation (degrees)")

    args = parser.parse_args()

    # Convert yaw from degrees if specified
    yaw = args.yaw
    if args.yaw_deg is not None:
        yaw = math.radians(args.yaw_deg)

    rclpy.init()
    adjuster = PoseAdjuster()

    try:
        adjuster.send_adjustment(
            args.lidar, args.x, args.y, args.z, args.roll, args.pitch, yaw
        )
        # Spin briefly to ensure message is sent
        rclpy.spin_once(adjuster, timeout_sec=0.1)
    finally:
        adjuster.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
