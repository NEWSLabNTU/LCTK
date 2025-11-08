#!/usr/bin/env python3
"""
Calibration Judge Node

This node subscribes to the extrinsic transform topic and compares it against
a ground truth transform matrix to evaluate calibration quality.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import TransformStamped
import numpy as np
import yaml
from typing import Optional, Dict, Any
from scipy.spatial.transform import Rotation


class CalibrationJudgeNode(Node):
    """
    ROS2 node that evaluates calibration quality by comparing estimated
    transforms against ground truth.
    """

    def __init__(self):
        super().__init__('calibration_judge')

        # Declare parameters (ground_truth_file is mandatory, no default)
        self.declare_parameter('ground_truth_file')
        self.declare_parameter('transform_topic', '/calibration/extrinsic_solver/extrinsic_transform')

        # Get parameters
        ground_truth_file = self.get_parameter('ground_truth_file').value
        transform_topic = self.get_parameter('transform_topic').value

        # Track best score so far
        self.best_score = None
        self.best_matrix = None

        # Validate that ground truth file is provided
        if not ground_truth_file or ground_truth_file == '':
            self.get_logger().error('FATAL: ground_truth_file parameter is mandatory but not provided!')
            self.get_logger().error('Usage: ros2 run calibration_judge judge_node --ros-args -p ground_truth_file:=/path/to/config.yaml')
            raise ValueError('ground_truth_file parameter is required')

        # Load ground truth configuration (matrix + scoring parameters)
        config = self._load_config(ground_truth_file)
        if config is None:
            self.get_logger().error(f'FATAL: Failed to load configuration from: {ground_truth_file}')
            raise RuntimeError(f'Could not load configuration from {ground_truth_file}')

        self.ground_truth_matrix = config['matrix']
        self.scoring_params = config['scoring']

        self.get_logger().info(f'Successfully loaded ground truth from: {ground_truth_file}')
        self.get_logger().info(f'Scoring: Total={self.scoring_params["total_score"]}, '
                              f'Trans[{self.scoring_params["translation"]["min_error_m"]}-'
                              f'{self.scoring_params["translation"]["max_error_m"]}m], '
                              f'Rot[{self.scoring_params["rotation"]["min_error_deg"]}-'
                              f'{self.scoring_params["rotation"]["max_error_deg"]}°]')

        # Create QoS profile with Best Effort reliability and depth 1
        # This matches the QoS used by the extrinsic_solver publisher
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create subscription to extrinsic transform topic
        self.subscription = self.create_subscription(
            TransformStamped,
            transform_topic,
            self._transform_callback,
            qos_profile
        )

        # Create timer to print best score every second
        self.status_timer = self.create_timer(1.0, self._print_status)

        self.get_logger().info(f'Calibration judge node started')
        self.get_logger().info(f'Subscribing to: {transform_topic}')

    def _load_config(self, filepath: str) -> Optional[Dict[str, Any]]:
        """
        Load ground truth configuration from YAML file.

        Expected YAML format:
            ground_truth:
                matrix: [[...], [...], [...], [...]]
            scoring:
                total_score: 100.0
                translation:
                    weight: 0.5
                    min_error_m: 0.01
                    max_error_m: 0.10
                rotation:
                    weight: 0.5
                    min_error_deg: 0.5
                    max_error_deg: 5.0

        Args:
            filepath: Path to the YAML configuration file

        Returns:
            Dictionary with 'matrix' (numpy array) and 'scoring' (dict), or None if loading fails
        """
        try:
            # Load YAML file
            with open(filepath, 'r') as f:
                yaml_data = yaml.safe_load(f)

            # Extract ground truth matrix
            matrix_list = yaml_data['ground_truth']['matrix']
            matrix = np.array(matrix_list, dtype=float)

            # Validate matrix shape
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

            # Extract scoring parameters
            scoring = yaml_data['scoring']

            # Validate scoring parameters
            required_keys = ['total_score', 'translation', 'rotation']
            if not all(key in scoring for key in required_keys):
                self.get_logger().error(f'Missing required scoring keys: {required_keys}')
                return None

            trans_keys = ['weight', 'min_error_m', 'max_error_m']
            if not all(key in scoring['translation'] for key in trans_keys):
                self.get_logger().error(f'Missing translation keys: {trans_keys}')
                return None

            rot_keys = ['weight', 'min_error_deg', 'max_error_deg']
            if not all(key in scoring['rotation'] for key in rot_keys):
                self.get_logger().error(f'Missing rotation keys: {rot_keys}')
                return None

            # Validate weights sum to 1.0
            total_weight = scoring['translation']['weight'] + scoring['rotation']['weight']
            if not np.isclose(total_weight, 1.0):
                self.get_logger().warn(
                    f'Translation and rotation weights should sum to 1.0, got: {total_weight}'
                )

            return {
                'matrix': matrix,
                'scoring': scoring
            }

        except FileNotFoundError:
            self.get_logger().error(f'Configuration file not found: {filepath}')
            return None
        except KeyError as e:
            self.get_logger().error(f'Missing required key in configuration: {e}')
            return None
        except Exception as e:
            self.get_logger().error(f'Error loading configuration file: {e}')
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

        # Extract rotation quaternion and convert to rotation matrix using scipy
        q = transform.transform.rotation
        quat = np.array([q.x, q.y, q.z, q.w])
        r = Rotation.from_quat(quat)
        rotation = r.as_matrix()

        # Build 4x4 transformation matrix
        matrix = np.eye(4)
        matrix[:3, :3] = rotation
        matrix[:3, 3] = translation

        return matrix

    def _compute_score(self, estimated_matrix: np.ndarray, ground_truth_matrix: np.ndarray) -> dict:
        """
        Compute calibration quality score by comparing estimated and ground truth matrices.

        Uses linear interpolation between min and max error thresholds to compute scores.

        Args:
            estimated_matrix: 4x4 estimated transformation matrix
            ground_truth_matrix: 4x4 ground truth transformation matrix

        Returns:
            Dictionary containing error metrics and scores
        """
        # Extract rotation and translation components
        R_est = estimated_matrix[:3, :3]
        t_est = estimated_matrix[:3, 3]
        R_gt = ground_truth_matrix[:3, :3]
        t_gt = ground_truth_matrix[:3, 3]

        # Compute translation error (Euclidean distance in meters)
        translation_error = np.linalg.norm(t_est - t_gt)

        # Compute rotation error (angle of rotation difference in degrees)
        R_diff = R_gt.T @ R_est
        trace = np.trace(R_diff)
        rotation_angle_error = np.arccos(np.clip((trace - 1) / 2, -1.0, 1.0))
        rotation_angle_error_deg = np.degrees(rotation_angle_error)

        # Get scoring parameters
        total_score = self.scoring_params['total_score']
        trans_params = self.scoring_params['translation']
        rot_params = self.scoring_params['rotation']

        # Compute translation score with linear interpolation
        trans_weight = trans_params['weight']
        trans_min = trans_params['min_error_m']
        trans_max = trans_params['max_error_m']

        if translation_error <= trans_min:
            # Perfect score
            trans_score = trans_weight * total_score
            trans_percentage = 100.0
        elif translation_error >= trans_max:
            # Zero score
            trans_score = 0.0
            trans_percentage = 0.0
        else:
            # Linear interpolation
            ratio = (trans_max - translation_error) / (trans_max - trans_min)
            trans_score = trans_weight * total_score * ratio
            trans_percentage = ratio * 100.0

        # Compute rotation score with linear interpolation
        rot_weight = rot_params['weight']
        rot_min = rot_params['min_error_deg']
        rot_max = rot_params['max_error_deg']

        if rotation_angle_error_deg <= rot_min:
            # Perfect score
            rot_score = rot_weight * total_score
            rot_percentage = 100.0
        elif rotation_angle_error_deg >= rot_max:
            # Zero score
            rot_score = 0.0
            rot_percentage = 0.0
        else:
            # Linear interpolation
            ratio = (rot_max - rotation_angle_error_deg) / (rot_max - rot_min)
            rot_score = rot_weight * total_score * ratio
            rot_percentage = ratio * 100.0

        # Compute final score
        final_score = trans_score + rot_score
        final_percentage = (final_score / total_score) * 100.0

        return {
            'translation_error_m': float(translation_error),
            'translation_score': float(trans_score),
            'translation_max_score': float(trans_weight * total_score),
            'translation_percentage': float(trans_percentage),
            'rotation_error_deg': float(rotation_angle_error_deg),
            'rotation_score': float(rot_score),
            'rotation_max_score': float(rot_weight * total_score),
            'rotation_percentage': float(rot_percentage),
            'final_score': float(final_score),
            'final_max_score': float(total_score),
            'final_percentage': float(final_percentage)
        }

    def _linear_interpolate_score(self, error: float, min_threshold: float,
                                  max_threshold: float, weight: float,
                                  total_score: float) -> tuple:
        """
        Compute score using linear interpolation.

        Args:
            error: The measured error
            min_threshold: Error below this gets full points
            max_threshold: Error above this gets zero points
            weight: Weight of this component (0-1)
            total_score: Total possible score

        Returns:
            Tuple of (score, percentage)
        """
        if error <= min_threshold:
            score = weight * total_score
            percentage = 100.0
        elif error >= max_threshold:
            score = 0.0
            percentage = 0.0
        else:
            ratio = (max_threshold - error) / (max_threshold - min_threshold)
            score = weight * total_score * ratio
            percentage = ratio * 100.0

        return score, percentage

    def _print_status(self):
        """
        Timer callback to print best score status every second.
        """
        if self.best_score is None:
            self.get_logger().info('Best Calib Score: N/A (waiting for first calibration...)')
        else:
            score = self.best_score
            self.get_logger().info(
                f'Best Calib Score: Trans err={score["translation_error_m"]:.4f}m ({score["translation_score"]:.1f}/{score["translation_max_score"]:.1f}pts), '
                f'Rot err={score["rotation_error_deg"]:.3f}° ({score["rotation_score"]:.1f}/{score["rotation_max_score"]:.1f}pts), '
                f'FINAL={score["final_score"]:.1f}/{score["final_max_score"]:.1f} ({score["final_percentage"]:.1f}%)'
            )

    def _transform_callback(self, msg: TransformStamped):
        """
        Callback for extrinsic transform messages.

        Args:
            msg: TransformStamped message from extrinsic solver
        """
        # Convert transform message to matrix
        estimated_matrix = self._transform_to_matrix(msg)

        # Compute score (ground_truth_matrix is guaranteed to be loaded)
        score = self._compute_score(estimated_matrix, self.ground_truth_matrix)

        # Update best score if this is better
        if self.best_score is None or score['final_score'] > self.best_score['final_score']:
            self.best_score = score
            self.best_matrix = estimated_matrix
            self.get_logger().info(f'New best score: {score["final_score"]:.1f} pts ({score["final_percentage"]:.1f}%)')


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
