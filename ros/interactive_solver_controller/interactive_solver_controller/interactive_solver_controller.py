#!/usr/bin/env python3
"""
Interactive Solver Controller

This script provides an interactive command-line interface for controlling
the advanced extrinsic solver node via ROS2 services.

Commands:
- add: Add current detection pair to buffer and re-solve
- status: Query buffer status and calibration state
- clear: Clear buffer and stop publishing
- help: Show available commands
- quit: Exit the program

Author: LCTK Team
License: MIT
"""

import sys
import time
from datetime import datetime
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from lctk_interfaces.srv import (
    AddDetectionToBuffer,
    ClearDetectionBuffer,
    GetBufferStatus,
    ListDetectionBuffer,
    RemoveDetectionFromBuffer,
)
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation as R


class InteractiveSolverController(Node):
    """Interactive controller for advanced extrinsic solver services."""

    def __init__(self):
        super().__init__("interactive_solver_controller")

        # Latest extrinsic transform
        self.latest_transform: Optional[TransformStamped] = None

        # Subscribe to extrinsic transform
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.transform_subscription = self.create_subscription(
            TransformStamped,
            "/calibration/extrinsic_solver/extrinsic_transform",
            self._transform_callback,
            qos,
        )

        # Service clients
        self.add_detection_client = self.create_client(
            AddDetectionToBuffer,
            "/calibration/advanced_extrinsic_solver/advanced_extrinsic_solver/add_detection",
        )
        self.clear_buffer_client = self.create_client(
            ClearDetectionBuffer,
            "/calibration/advanced_extrinsic_solver/advanced_extrinsic_solver/clear_buffer",
        )
        self.get_status_client = self.create_client(
            GetBufferStatus,
            "/calibration/advanced_extrinsic_solver/advanced_extrinsic_solver/get_status",
        )
        self.list_buffer_client = self.create_client(
            ListDetectionBuffer,
            "/calibration/advanced_extrinsic_solver/advanced_extrinsic_solver/list_buffer",
        )
        self.remove_detection_client = self.create_client(
            RemoveDetectionFromBuffer,
            "/calibration/advanced_extrinsic_solver/advanced_extrinsic_solver/remove_detection",
        )

        self.get_logger().info("Interactive Solver Controller initialized")
        self.get_logger().info("Waiting for services...")

        # Wait for services with timeout
        services_ready = True
        timeout_sec = 10.0

        if not self.add_detection_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().warn(
                f"add_detection service not available after {timeout_sec}s"
            )
            services_ready = False

        if not self.clear_buffer_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().warn(
                f"clear_buffer service not available after {timeout_sec}s"
            )
            services_ready = False

        if not self.get_status_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().warn(
                f"get_status service not available after {timeout_sec}s"
            )
            services_ready = False

        if not self.list_buffer_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().warn(
                f"list_buffer service not available after {timeout_sec}s"
            )
            services_ready = False

        if not self.remove_detection_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().warn(
                f"remove_detection service not available after {timeout_sec}s"
            )
            services_ready = False

        if services_ready:
            self.get_logger().info("All services available!")
        else:
            self.get_logger().warn(
                "Some services are not available. Commands may fail."
            )

    def _transform_callback(self, msg: TransformStamped):
        """Cache the latest extrinsic transform."""
        self.latest_transform = msg

    def _display_current_transform(self):
        """Display the current extrinsic transform in a user-friendly format."""
        if self.latest_transform is None:
            print("\n  [No extrinsic transform available yet]")
            return

        tf = self.latest_transform.transform
        
        # Extract translation
        tx = tf.translation.x
        ty = tf.translation.y
        tz = tf.translation.z
        
        # Extract rotation as quaternion
        qx = tf.rotation.x
        qy = tf.rotation.y
        qz = tf.rotation.z
        qw = tf.rotation.w
        
        # Convert to Euler angles
        rot = R.from_quat([qx, qy, qz, qw])
        euler = rot.as_euler('xyz', degrees=True)
        
        # Convert to rotation matrix
        rot_matrix = rot.as_matrix()
        
        print(
            f"\n"
            f"{'='*70}\n"
            f"  Current Extrinsic Transform (LiDAR → Camera)\n"
            f"{'='*70}\n"
            f"  Translation (m):\n"
            f"    x: {tx:+.6f}\n"
            f"    y: {ty:+.6f}\n"
            f"    z: {tz:+.6f}\n"
            f"\n"
            f"  Rotation (Euler angles XYZ, degrees):\n"
            f"    Roll:  {euler[0]:+.3f}°\n"
            f"    Pitch: {euler[1]:+.3f}°\n"
            f"    Yaw:   {euler[2]:+.3f}°\n"
            f"\n"
            f"  Quaternion (x, y, z, w):\n"
            f"    ({qx:+.6f}, {qy:+.6f}, {qz:+.6f}, {qw:+.6f})\n"
            f"\n"
            f"  Rotation Matrix:\n"
            f"    [{rot_matrix[0,0]:+.6f}, {rot_matrix[0,1]:+.6f}, {rot_matrix[0,2]:+.6f}]\n"
            f"    [{rot_matrix[1,0]:+.6f}, {rot_matrix[1,1]:+.6f}, {rot_matrix[1,2]:+.6f}]\n"
            f"    [{rot_matrix[2,0]:+.6f}, {rot_matrix[2,1]:+.6f}, {rot_matrix[2,2]:+.6f}]\n"
            f"{'='*70}\n"
        )

    def add_detection(self):
        """Call add_detection service."""
        self.get_logger().debug("Calling add_detection service...")

        request = AddDetectionToBuffer.Request()
        future = self.add_detection_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.done():
            response = future.result()
            if response.success:
                print(
                    f"[SUCCESS] {response.message} (buffer size: {response.buffer_size})"
                )
                
                # Give a brief moment for the transform to be published and received
                time.sleep(0.1)
                
                # Spin once to process any pending callbacks (like transform updates)
                rclpy.spin_once(self, timeout_sec=0.1)
                
                # Display the current extrinsic transform
                self._display_current_transform()
            else:
                print(
                    f"[FAILED] {response.message} (buffer size: {response.buffer_size})"
                )
            return response.success
        else:
            print("[ERROR] Service call timed out")
            return False

    def clear_buffer(self):
        """Call clear_buffer service."""
        self.get_logger().debug("Calling clear_buffer service...")

        request = ClearDetectionBuffer.Request()
        future = self.clear_buffer_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.done():
            response = future.result()
            if response.success:
                print(f"[SUCCESS] {response.message}")
            else:
                print(f"[FAILED] {response.message}")
            return response.success
        else:
            print("[ERROR] Service call timed out")
            return False

    def get_status(self):
        """Call get_status service."""
        self.get_logger().debug("Querying buffer status...")

        request = GetBufferStatus.Request()
        future = self.get_status_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.done():
            response = future.result()
            print(
                f"\n"
                f"  Buffer Status:\n"
                f"  ├─ Buffer size: {response.buffer_size} detection pairs\n"
                f"  ├─ Total correspondences: {response.total_correspondences}\n"
                f"  ├─ Publishing: {'Yes' if response.is_publishing else 'No'}\n"
                f"  └─ Last solve status: {response.last_solve_status}"
            )
            return True
        else:
            print("[ERROR] Service call timed out")
            return False

    def list_buffer(self):
        """Call list_buffer service."""
        self.get_logger().debug("Listing buffer contents...")

        request = ListDetectionBuffer.Request()
        future = self.list_buffer_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.done():
            response = future.result()
            if response.success:
                print(
                    f"\n"
                    f"  Buffer Contents:\n"
                    f"  ├─ Total pairs: {response.buffer_size}\n"
                    f"  └─ Details:"
                )
                for i, (aruco_count, board_count, ts_sec, ts_nsec) in enumerate(
                    zip(
                        response.aruco_counts,
                        response.board_counts,
                        response.timestamps_sec,
                        response.timestamps_nanosec,
                    )
                ):
                    # Convert ROS timestamp to datetime
                    timestamp = datetime.fromtimestamp(ts_sec + ts_nsec / 1e9)
                    timestamp_str = timestamp.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                    print(
                        f"      [{i}] {aruco_count} ArUco, {board_count} boards | {timestamp_str}"
                    )
            else:
                print(f"[FAILED] {response.message}")
            return response.success
        else:
            print("[ERROR] Service call timed out")
            return False

    def remove_detection(self, index: int):
        """Call remove_detection service."""
        self.get_logger().debug(f"Removing detection at index {index}...")

        request = RemoveDetectionFromBuffer.Request()
        request.index = index
        future = self.remove_detection_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.done():
            response = future.result()
            if response.success:
                print(
                    f"[SUCCESS] {response.message} (new buffer size: {response.buffer_size})"
                )
            else:
                print(
                    f"[FAILED] {response.message} (buffer size: {response.buffer_size})"
                )
            return response.success
        else:
            print("[ERROR] Service call timed out")
            return False


def print_help():
    """Print help message."""
    print("\n" + "=" * 70)
    print("Interactive Solver Controller - Available Commands")
    print("=" * 70)
    print("  add (a)          - Add current detection pair to buffer and re-solve")
    print("  status (s)       - Query buffer status and calibration state")
    print("  list (l)         - List all detection pairs in buffer with details")
    print("  transform (t)    - Show current extrinsic transform")
    print("  delete [index]   - Delete detection pair (default: last)")
    print("    (d, del)         [index] = specific index, 'last', or omit for last")
    print("  clear (c)        - Clear entire buffer and stop publishing")
    print("  help (h, ?)      - Show this help message")
    print("  quit (q, exit)   - Exit the program")
    print("=" * 70 + "\n")


def main(args=None):
    """Main interactive loop."""
    rclpy.init(args=args)

    try:
        node = InteractiveSolverController()

        print("\n" + "=" * 60)
        print("Interactive Solver Controller")
        print("=" * 60)
        print("Type 'help' for available commands, 'quit' to exit")
        print("=" * 60 + "\n")

        while rclpy.ok():
            try:
                command_input = input("solver> ").strip()
                command_parts = command_input.split()

                if not command_parts:
                    continue

                command = command_parts[0].lower()

                if command in ["quit", "q", "exit"]:
                    print("Exiting...")
                    break
                elif command in ["help", "h", "?"]:
                    print_help()
                elif command in ["add", "a"]:
                    node.add_detection()
                elif command in ["status", "s"]:
                    node.get_status()
                elif command in ["list", "l"]:
                    node.list_buffer()
                elif command in ["transform", "t"]:
                    # Spin once to ensure we have the latest transform
                    rclpy.spin_once(node, timeout_sec=0.1)
                    node._display_current_transform()
                elif command in ["delete", "del", "d"]:
                    # Default: delete last if no argument provided
                    if len(command_parts) < 2 or command_parts[1].lower() == "last":
                        # Get buffer size first
                        status_request = GetBufferStatus.Request()
                        status_future = node.get_status_client.call_async(
                            status_request
                        )
                        rclpy.spin_until_future_complete(
                            node, status_future, timeout_sec=5.0
                        )
                        if status_future.done():
                            buffer_size = status_future.result().buffer_size
                            if buffer_size > 0:
                                node.remove_detection(buffer_size - 1)
                            else:
                                print("Buffer is empty")
                        else:
                            print("Failed to get buffer status")
                    else:
                        try:
                            index = int(command_parts[1])
                            node.remove_detection(index)
                        except ValueError:
                            print(
                                f"Invalid index: '{command_parts[1]}'. Must be an integer or 'last'"
                            )
                elif command in ["clear", "c"]:
                    node.clear_buffer()
                else:
                    print(
                        f"Unknown command: '{command}'. Type 'help' for available commands."
                    )

            except EOFError:
                print("\nExiting...")
                break
            except KeyboardInterrupt:
                print("\nExiting...")
                break

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
