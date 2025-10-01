#!/usr/bin/env python3
"""
Generate synthetic test data for multi_wayside_node integration testing.

This script creates rosbag2 files with synthetic point cloud data containing
calibration boards at known positions for validation purposes.
"""

import argparse
import math
import os
import struct
from typing import List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


class SyntheticDataGenerator(Node):
    def __init__(self):
        super().__init__("synthetic_data_generator")

    def create_board_points(
        self,
        center: Tuple[float, float, float],
        size: float = 1.0,
        normal: Tuple[float, float, float] = (0, 0, 1),
        num_points: int = 1000,
    ) -> np.ndarray:
        """Generate synthetic point cloud for a calibration board"""

        # Create points in local board coordinate system
        # Board is a square with holes
        points = []

        # Main board points (dense square)
        for _ in range(num_points):
            # Random points within the board square
            x = (np.random.random() - 0.5) * size
            y = (np.random.random() - 0.5) * size
            z = 0.0  # Board is flat

            # Check if point is not in a hole (simplified - just 3 circular holes)
            hole_positions = [
                (-0.3 * size, -0.3 * size),
                (0.3 * size, -0.3 * size),
                (0.0, 0.3 * size),
            ]
            hole_radius = 0.1 * size

            in_hole = False
            for hx, hy in hole_positions:
                if (x - hx) ** 2 + (y - hy) ** 2 < hole_radius ** 2:
                    in_hole = True
                    break

            if not in_hole:
                # Add some noise to simulate real point cloud
                x += np.random.normal(0, 0.001)
                y += np.random.normal(0, 0.001)
                z += np.random.normal(0, 0.001)

                points.append([x, y, z])

        points = np.array(points)

        # Transform to world coordinates
        # Apply rotation to align with normal vector
        # (Simplified - assume normal is close to Z-axis)
        points[:, 0] += center[0]
        points[:, 1] += center[1]
        points[:, 2] += center[2]

        # Add intensity values
        intensities = np.random.randint(50, 200, len(points))

        # Combine xyz and intensity
        return np.column_stack([points, intensities])

    def create_background_points(
        self,
        num_points: int = 5000,
        bounds: Tuple[float, float, float, float, float, float] = (
            -10,
            10,
            -10,
            10,
            -2,
            5,
        ),
    ) -> np.ndarray:
        """Generate background/environment points"""
        x_min, x_max, y_min, y_max, z_min, z_max = bounds

        points = []
        for _ in range(num_points):
            x = np.random.uniform(x_min, x_max)
            y = np.random.uniform(y_min, y_max)
            z = np.random.uniform(z_min, z_max)
            intensity = np.random.randint(10, 100)
            points.append([x, y, z, intensity])

        return np.array(points)

    def points_to_pointcloud2(
        self, points: np.ndarray, frame_id: str, timestamp
    ) -> PointCloud2:
        """Convert numpy array to PointCloud2 message"""

        # Define fields
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(
                name="intensity", offset=12, datatype=PointField.FLOAT32, count=1
            ),
        ]

        # Pack data
        data = []
        for point in points:
            data.append(
                struct.pack(
                    "ffff",
                    float(point[0]),
                    float(point[1]),
                    float(point[2]),
                    float(point[3]),
                )
            )

        # Create message
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = timestamp
        msg.header.frame_id = frame_id
        msg.height = 1
        msg.width = len(points)
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 16  # 4 fields * 4 bytes each
        msg.row_step = msg.point_step * msg.width
        msg.data = b"".join(data)
        msg.is_dense = True

        return msg

    def generate_scenario_perfect_boards(self, output_dir: str):
        """Generate scenario 1: Perfect boards for ideal calibration"""
        self.get_logger().info("Generating scenario 1: Perfect boards")

        # Define board positions for two LiDARs
        # LiDAR 1 at origin (0, 0, 0)
        # LiDAR 2 at (1.5, 0.3, 0.1) with slight rotation
        # Board visible to both at (3, 0, 0.5)

        board_position = (3.0, 0.0, 0.5)  # Board position in world coordinates
        lidar2_transform = (1.5, 0.3, 0.1, 0.1)  # x, y, z, rotation_z

        # Generate frames over time
        import rosbag2_py
        from rclpy.serialization import serialize_message
        from rclpy.time import Time

        bag_path = os.path.join(output_dir, "scenario_1_perfect_boards")

        writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
        converter_options = rosbag2_py.ConverterOptions("", "")
        writer.open(storage_options, converter_options)

        # Create topics
        lidar1_topic = rosbag2_py.TopicMetadata(
            name="/lidar1/points",
            type="sensor_msgs/msg/PointCloud2",
            serialization_format="cdr",
        )
        lidar2_topic = rosbag2_py.TopicMetadata(
            name="/lidar2/points",
            type="sensor_msgs/msg/PointCloud2",
            serialization_format="cdr",
        )

        writer.create_topic(lidar1_topic)
        writer.create_topic(lidar2_topic)

        # Generate 50 frames over 25 seconds (2 Hz)
        for i in range(50):
            timestamp = Time(seconds=i * 0.5).to_msg()

            # LiDAR 1 point cloud (sees board from origin)
            board_points_1 = self.create_board_points(
                board_position, size=1.0, num_points=2000
            )
            background_1 = self.create_background_points(1000)
            all_points_1 = np.vstack([board_points_1, background_1])

            pc_msg_1 = self.points_to_pointcloud2(
                all_points_1, "test_lidar1", timestamp
            )

            # LiDAR 2 point cloud (transform board to LiDAR 2 frame)
            board_points_2 = self.create_board_points(
                board_position, size=1.0, num_points=2000
            )
            # Apply inverse transform to simulate LiDAR 2's viewpoint
            tx, ty, tz, rz = lidar2_transform
            cos_r = np.cos(rz)
            sin_r = np.sin(rz)

            # Transform board points to LiDAR 2 frame
            for idx in range(len(board_points_2)):
                x, y, z = board_points_2[idx, :3]
                # Translate to world then to LiDAR 2 frame
                x_world = x
                y_world = y
                # Apply inverse transform
                x_new = cos_r * (x_world - tx) + sin_r * (y_world - ty)
                y_new = -sin_r * (x_world - tx) + cos_r * (y_world - ty)
                z_new = z - tz
                board_points_2[idx, :3] = [x_new, y_new, z_new]

            background_2 = self.create_background_points(1000)
            # Also transform background
            for idx in range(len(background_2)):
                x, y, z = background_2[idx, :3]
                x_new = cos_r * (x - tx) + sin_r * (y - ty)
                y_new = -sin_r * (x - tx) + cos_r * (y - ty)
                z_new = z - tz
                background_2[idx, :3] = [x_new, y_new, z_new]

            all_points_2 = np.vstack([board_points_2, background_2])

            pc_msg_2 = self.points_to_pointcloud2(
                all_points_2, "test_lidar2", timestamp
            )

            # Write to bag
            writer.write(
                "/lidar1/points",
                serialize_message(pc_msg_1),
                int(timestamp.sec * 1e9 + timestamp.nanosec),
            )
            writer.write(
                "/lidar2/points",
                serialize_message(pc_msg_2),
                int(timestamp.sec * 1e9 + timestamp.nanosec),
            )

        writer.close()
        self.get_logger().info(f"Generated perfect boards scenario: {bag_path}")

    def generate_scenario_noisy_data(self, output_dir: str):
        """Generate scenario 2: Noisy data with sensor artifacts"""
        self.get_logger().info("Generating scenario 2: Noisy data")

        board_position = (3.0, 0.0, 0.5)
        lidar2_transform = (1.5, 0.3, 0.1, 0.1)

        import rosbag2_py
        from rclpy.serialization import serialize_message
        from rclpy.time import Time

        bag_path = os.path.join(output_dir, "scenario_2_noisy_data")

        writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
        converter_options = rosbag2_py.ConverterOptions("", "")
        writer.open(storage_options, converter_options)

        # Create topics
        lidar1_topic = rosbag2_py.TopicMetadata(
            name="/lidar1/points",
            type="sensor_msgs/msg/PointCloud2",
            serialization_format="cdr",
        )
        lidar2_topic = rosbag2_py.TopicMetadata(
            name="/lidar2/points",
            type="sensor_msgs/msg/PointCloud2",
            serialization_format="cdr",
        )

        writer.create_topic(lidar1_topic)
        writer.create_topic(lidar2_topic)

        # Generate frames with noise
        for i in range(50):
            timestamp = Time(seconds=i * 0.5).to_msg()

            # Add noise to board position (simulating vibration)
            noisy_position = (
                board_position[0] + np.random.normal(0, 0.02),
                board_position[1] + np.random.normal(0, 0.02),
                board_position[2] + np.random.normal(0, 0.01),
            )

            # LiDAR 1 with noise
            board_points_1 = self.create_board_points(
                noisy_position, size=1.0, num_points=1500
            )
            # Add measurement noise
            board_points_1[:, :3] += np.random.normal(
                0, 0.005, (len(board_points_1), 3)
            )
            # Add some outliers
            outliers_1 = np.random.uniform(-5, 5, (100, 4))
            outliers_1[:, 3] = np.random.randint(10, 50, 100)  # Low intensity outliers

            background_1 = self.create_background_points(1500)
            all_points_1 = np.vstack([board_points_1, background_1, outliers_1])

            pc_msg_1 = self.points_to_pointcloud2(
                all_points_1, "test_lidar1", timestamp
            )

            # LiDAR 2 with different noise characteristics
            board_points_2 = self.create_board_points(
                noisy_position, size=1.0, num_points=1200
            )
            # Different noise level
            board_points_2[:, :3] += np.random.normal(
                0, 0.008, (len(board_points_2), 3)
            )

            # Transform to LiDAR 2 frame
            tx, ty, tz, rz = lidar2_transform
            # Add noise to transform (calibration uncertainty)
            tx += np.random.normal(0, 0.005)
            ty += np.random.normal(0, 0.005)

            cos_r = np.cos(rz)
            sin_r = np.sin(rz)

            for idx in range(len(board_points_2)):
                x, y, z = board_points_2[idx, :3]
                x_new = cos_r * (x - tx) + sin_r * (y - ty)
                y_new = -sin_r * (x - tx) + cos_r * (y - ty)
                z_new = z - tz
                board_points_2[idx, :3] = [x_new, y_new, z_new]

            # More outliers for LiDAR 2
            outliers_2 = np.random.uniform(-5, 5, (150, 4))
            outliers_2[:, 3] = np.random.randint(5, 40, 150)

            background_2 = self.create_background_points(1800)
            all_points_2 = np.vstack([board_points_2, background_2, outliers_2])

            pc_msg_2 = self.points_to_pointcloud2(
                all_points_2, "test_lidar2", timestamp
            )

            writer.write(
                "/lidar1/points",
                serialize_message(pc_msg_1),
                int(timestamp.sec * 1e9 + timestamp.nanosec),
            )
            writer.write(
                "/lidar2/points",
                serialize_message(pc_msg_2),
                int(timestamp.sec * 1e9 + timestamp.nanosec),
            )

        writer.close()
        self.get_logger().info(f"Generated noisy data scenario: {bag_path}")

    def generate_scenario_partial_occlusion(self, output_dir: str):
        """Generate scenario 3: Partial occlusion of calibration board"""
        self.get_logger().info("Generating scenario 3: Partial occlusion")

        board_position = (3.0, 0.0, 0.5)
        lidar2_transform = (1.5, 0.3, 0.1, 0.1)

        import rosbag2_py
        from rclpy.serialization import serialize_message
        from rclpy.time import Time

        bag_path = os.path.join(output_dir, "scenario_3_partial_occlusion")

        writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
        converter_options = rosbag2_py.ConverterOptions("", "")
        writer.open(storage_options, converter_options)

        lidar1_topic = rosbag2_py.TopicMetadata(
            name="/lidar1/points",
            type="sensor_msgs/msg/PointCloud2",
            serialization_format="cdr",
        )
        lidar2_topic = rosbag2_py.TopicMetadata(
            name="/lidar2/points",
            type="sensor_msgs/msg/PointCloud2",
            serialization_format="cdr",
        )

        writer.create_topic(lidar1_topic)
        writer.create_topic(lidar2_topic)

        for i in range(50):
            timestamp = Time(seconds=i * 0.5).to_msg()

            # Create board with occlusion
            board_points_1 = self.create_board_points(
                board_position, size=1.0, num_points=2000
            )

            # Remove points in occluded region (simulate object blocking part of board)
            occlusion_center = (-0.2, 0.1)  # Occlusion in board local coordinates
            occlusion_radius = 0.3

            mask = np.ones(len(board_points_1), dtype=bool)
            for idx in range(len(board_points_1)):
                x, y = (
                    board_points_1[idx, 0] - board_position[0],
                    board_points_1[idx, 1] - board_position[1],
                )
                if (x - occlusion_center[0]) ** 2 + (
                    y - occlusion_center[1]
                ) ** 2 < occlusion_radius ** 2:
                    mask[idx] = False

            board_points_1 = board_points_1[mask]  # Remove occluded points

            background_1 = self.create_background_points(1000)
            # Add occluding object points
            occluder_points = []
            for _ in range(200):
                r = np.random.uniform(0, occlusion_radius)
                theta = np.random.uniform(0, 2 * np.pi)
                x = board_position[0] + occlusion_center[0] + r * np.cos(theta)
                y = board_position[1] + occlusion_center[1] + r * np.sin(theta)
                z = board_position[2] + np.random.uniform(-0.5, 0.5)
                intensity = np.random.randint(20, 80)
                occluder_points.append([x, y, z, intensity])

            occluder_points = np.array(occluder_points)
            all_points_1 = np.vstack([board_points_1, background_1, occluder_points])

            pc_msg_1 = self.points_to_pointcloud2(
                all_points_1, "test_lidar1", timestamp
            )

            # LiDAR 2 sees different occlusion pattern
            board_points_2 = self.create_board_points(
                board_position, size=1.0, num_points=2000
            )

            # Different occlusion from LiDAR 2's perspective
            occlusion_center_2 = (0.3, -0.1)
            occlusion_radius_2 = 0.25

            mask2 = np.ones(len(board_points_2), dtype=bool)
            for idx in range(len(board_points_2)):
                x, y = (
                    board_points_2[idx, 0] - board_position[0],
                    board_points_2[idx, 1] - board_position[1],
                )
                if (x - occlusion_center_2[0]) ** 2 + (
                    y - occlusion_center_2[1]
                ) ** 2 < occlusion_radius_2 ** 2:
                    mask2[idx] = False

            board_points_2 = board_points_2[mask2]

            # Transform to LiDAR 2 frame
            tx, ty, tz, rz = lidar2_transform
            cos_r = np.cos(rz)
            sin_r = np.sin(rz)

            for idx in range(len(board_points_2)):
                x, y, z = board_points_2[idx, :3]
                x_new = cos_r * (x - tx) + sin_r * (y - ty)
                y_new = -sin_r * (x - tx) + cos_r * (y - ty)
                z_new = z - tz
                board_points_2[idx, :3] = [x_new, y_new, z_new]

            background_2 = self.create_background_points(1000)
            all_points_2 = np.vstack([board_points_2, background_2])

            pc_msg_2 = self.points_to_pointcloud2(
                all_points_2, "test_lidar2", timestamp
            )

            writer.write(
                "/lidar1/points",
                serialize_message(pc_msg_1),
                int(timestamp.sec * 1e9 + timestamp.nanosec),
            )
            writer.write(
                "/lidar2/points",
                serialize_message(pc_msg_2),
                int(timestamp.sec * 1e9 + timestamp.nanosec),
            )

        writer.close()
        self.get_logger().info(f"Generated partial occlusion scenario: {bag_path}")

    def generate_scenario_multi_board(self, output_dir: str):
        """Generate scenario 4: Multiple boards in scene"""
        self.get_logger().info("Generating scenario 4: Multi-board scene")

        # Multiple board positions
        board_positions = [
            (3.0, 0.0, 0.5),  # Main calibration board
            (2.5, 2.0, 0.3),  # Distractor board 1
            (3.5, -1.8, 0.6),  # Distractor board 2
        ]
        lidar2_transform = (1.5, 0.3, 0.1, 0.1)

        import rosbag2_py
        from rclpy.serialization import serialize_message
        from rclpy.time import Time

        bag_path = os.path.join(output_dir, "scenario_4_multi_boards")

        writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
        converter_options = rosbag2_py.ConverterOptions("", "")
        writer.open(storage_options, converter_options)

        lidar1_topic = rosbag2_py.TopicMetadata(
            name="/lidar1/points",
            type="sensor_msgs/msg/PointCloud2",
            serialization_format="cdr",
        )
        lidar2_topic = rosbag2_py.TopicMetadata(
            name="/lidar2/points",
            type="sensor_msgs/msg/PointCloud2",
            serialization_format="cdr",
        )

        writer.create_topic(lidar1_topic)
        writer.create_topic(lidar2_topic)

        for i in range(50):
            timestamp = Time(seconds=i * 0.5).to_msg()

            # Generate all boards for LiDAR 1
            all_boards_1 = []
            for idx, pos in enumerate(board_positions):
                # Main board has more points and better quality
                num_points = 2000 if idx == 0 else np.random.randint(800, 1200)
                board_points = self.create_board_points(
                    pos, size=1.0, num_points=num_points
                )

                # Add varying noise levels
                noise_level = 0.003 if idx == 0 else 0.01
                board_points[:, :3] += np.random.normal(
                    0, noise_level, (len(board_points), 3)
                )

                all_boards_1.append(board_points)

            all_boards_1 = np.vstack(all_boards_1)
            background_1 = self.create_background_points(1000)
            all_points_1 = np.vstack([all_boards_1, background_1])

            pc_msg_1 = self.points_to_pointcloud2(
                all_points_1, "test_lidar1", timestamp
            )

            # Generate all boards for LiDAR 2
            all_boards_2 = []
            for idx, pos in enumerate(board_positions):
                # Some boards might not be visible from LiDAR 2
                if idx == 2 and i % 10 < 3:  # Board 2 occasionally not visible
                    continue

                num_points = 1800 if idx == 0 else np.random.randint(600, 1000)
                board_points = self.create_board_points(
                    pos, size=1.0, num_points=num_points
                )

                # Transform to LiDAR 2 frame
                tx, ty, tz, rz = lidar2_transform
                cos_r = np.cos(rz)
                sin_r = np.sin(rz)

                for pt_idx in range(len(board_points)):
                    x, y, z = board_points[pt_idx, :3]
                    x_new = cos_r * (x - tx) + sin_r * (y - ty)
                    y_new = -sin_r * (x - tx) + cos_r * (y - ty)
                    z_new = z - tz
                    board_points[pt_idx, :3] = [x_new, y_new, z_new]

                all_boards_2.append(board_points)

            all_boards_2 = np.vstack(all_boards_2) if all_boards_2 else np.empty((0, 4))
            background_2 = self.create_background_points(1000)

            if len(all_boards_2) > 0:
                all_points_2 = np.vstack([all_boards_2, background_2])
            else:
                all_points_2 = background_2

            pc_msg_2 = self.points_to_pointcloud2(
                all_points_2, "test_lidar2", timestamp
            )

            writer.write(
                "/lidar1/points",
                serialize_message(pc_msg_1),
                int(timestamp.sec * 1e9 + timestamp.nanosec),
            )
            writer.write(
                "/lidar2/points",
                serialize_message(pc_msg_2),
                int(timestamp.sec * 1e9 + timestamp.nanosec),
            )

        writer.close()
        self.get_logger().info(f"Generated multi-board scenario: {bag_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Generate synthetic test data for multi_wayside_node"
    )
    parser.add_argument(
        "--output_dir",
        type=str,
        default="test_data",
        help="Output directory for generated bags",
    )
    parser.add_argument(
        "--scenarios",
        nargs="+",
        default=["perfect"],
        choices=["perfect", "noisy", "occlusion", "multi"],
        help="Scenarios to generate",
    )

    args = parser.parse_args()

    # Create output directory
    os.makedirs(args.output_dir, exist_ok=True)

    rclpy.init()

    try:
        generator = SyntheticDataGenerator()

        if "perfect" in args.scenarios:
            generator.generate_scenario_perfect_boards(args.output_dir)

        if "noisy" in args.scenarios:
            generator.generate_scenario_noisy_data(args.output_dir)

        if "occlusion" in args.scenarios:
            generator.generate_scenario_partial_occlusion(args.output_dir)

        if "multi" in args.scenarios:
            generator.generate_scenario_multi_board(args.output_dir)

    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
