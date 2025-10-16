#!/usr/bin/env python3
"""
Calibration Judge Node

This node subscribes to the extrinsic transform topic and compares it against
a ground truth transform matrix to evaluate calibration quality.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import numpy as np
from typing import Optional


class CalibrationJudgeNode(Node):
    """
    ROS2 node that evaluates calibration quality by comparing estimated
    transforms against ground truth.
    """

    def __init__(self):
        super().__init__('calibration_judge')

        # Declare parameters
        self.declare_parameter('ground_truth_file', '')
        self.declare_parameter('transform_topic', '/calibration/extrinsic_solver/extrinsic_transform')

        # Get parameters
        ground_truth_file = self.get_parameter('ground_truth_file').value
        transform_topic = self.get_parameter('transform_topic').value

        # Load ground truth transform
        self.ground_truth_matrix: Optional[np.ndarray] = None
        if ground_truth_file:
            self.ground_truth_matrix = self._load_ground_truth(ground_truth_file)
            if self.ground_truth_matrix is not None:
                self.get_logger().info(f'Loaded ground truth from: {ground_truth_file}')
            else:
                self.get_logger().error(f'Failed to load ground truth from: {ground_truth_file}')
        else:
            self.get_logger().warn('No ground truth file specified. Use "ground_truth_file" parameter.')

        # Create subscription to extrinsic transform topic
        self.subscription = self.create_subscription(
            TransformStamped,
            transform_topic,
            self._transform_callback,
            10
        )

        self.get_logger().info(f'Calibration judge node started')
        self.get_logger().info(f'Subscribing to: {transform_topic}')

    def _load_ground_truth(self, filepath: str) -> Optional[np.ndarray]:
        """
        Load ground truth transformation matrix from file.

        Expected file format: 4x4 transformation matrix (space or comma separated)

        Args:
            filepath: Path to the ground truth file

        Returns:
            4x4 numpy array or None if loading fails
        """
        try:
            matrix = np.loadtxt(filepath)

            # Validate shape
            if matrix.shape != (4, 4):
                self.get_logger().error(
                    f'Ground truth matrix must be 4x4, got shape: {matrix.shape}'
                )
                return None

            # Validate that it's a valid transformation matrix
            if not np.allclose(matrix[3, :], [0, 0, 0, 1]):
                self.get_logger().warn(
                    f'Last row of transformation matrix should be [0, 0, 0, 1], '
                    f'got: {matrix[3, :]}'
                )

            return matrix

        except Exception as e:
            self.get_logger().error(f'Error loading ground truth file: {e}')
            return None

    def _transform_to_matrix(self, transform: TransformStamped) -> np.ndarray:
        """
        Convert ROS TransformStamped message to 4x4 transformation matrix.

        Args:
            transform: ROS TransformStamped message

        Returns:
            4x4 numpy transformation matrix
        """
        # Extract translation
        t = transform.transform.translation
        translation = np.array([t.x, t.y, t.z])

        # Extract rotation quaternion
        q = transform.transform.rotation
        qx, qy, qz, qw = q.x, q.y, q.z, q.w

        # Convert quaternion to rotation matrix
        # Formula from: https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/
        rotation = np.array([
            [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
            [2*(qx*qy + qw*qz), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qw*qx)],
            [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx**2 + qy**2)]
        ])

        # Build 4x4 transformation matrix
        matrix = np.eye(4)
        matrix[:3, :3] = rotation
        matrix[:3, 3] = translation

        return matrix

    def _compute_score(self, estimated_matrix: np.ndarray, ground_truth_matrix: np.ndarray) -> dict:
        """
        Compute calibration quality score by comparing estimated and ground truth matrices.

        Args:
            estimated_matrix: 4x4 estimated transformation matrix
            ground_truth_matrix: 4x4 ground truth transformation matrix

        Returns:
            Dictionary containing various error metrics (to be defined)
        """
        # TODO: Implement scoring function
        # Placeholder for now - will discuss scoring metrics

        # Extract rotation and translation components
        R_est = estimated_matrix[:3, :3]
        t_est = estimated_matrix[:3, 3]
        R_gt = ground_truth_matrix[:3, :3]
        t_gt = ground_truth_matrix[:3, 3]

        # Compute translation error (Euclidean distance)
        translation_error = np.linalg.norm(t_est - t_gt)

        # Compute rotation error (Frobenius norm of difference)
        rotation_error = np.linalg.norm(R_est - R_gt, 'fro')

        # Compute relative rotation error (angle of rotation difference)
        R_diff = R_gt.T @ R_est
        trace = np.trace(R_diff)
        rotation_angle_error = np.arccos(np.clip((trace - 1) / 2, -1.0, 1.0))
        rotation_angle_error_deg = np.degrees(rotation_angle_error)

        score = {
            'translation_error_m': float(translation_error),
            'rotation_frobenius_error': float(rotation_error),
            'rotation_angle_error_deg': float(rotation_angle_error_deg),
            'overall_score': 0.0  # TODO: Define overall scoring function
        }

        return score

    def _transform_callback(self, msg: TransformStamped):
        """
        Callback for extrinsic transform messages.

        Args:
            msg: TransformStamped message from extrinsic solver
        """
        # Check if ground truth is loaded
        if self.ground_truth_matrix is None:
            self.get_logger().warn('No ground truth loaded. Cannot compute score.', throttle_duration_sec=5.0)
            return

        # Convert transform message to matrix
        estimated_matrix = self._transform_to_matrix(msg)

        # Compute score
        score = self._compute_score(estimated_matrix, self.ground_truth_matrix)

        # Log results
        self.get_logger().info('='*60)
        self.get_logger().info(f'Calibration Quality Score:')
        self.get_logger().info(f'  Translation Error: {score["translation_error_m"]:.6f} m')
        self.get_logger().info(f'  Rotation Error (Frobenius): {score["rotation_frobenius_error"]:.6f}')
        self.get_logger().info(f'  Rotation Angle Error: {score["rotation_angle_error_deg"]:.4f} degrees')
        self.get_logger().info(f'  Overall Score: {score["overall_score"]:.4f}')
        self.get_logger().info('='*60)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = CalibrationJudgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in calibration_judge node: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
