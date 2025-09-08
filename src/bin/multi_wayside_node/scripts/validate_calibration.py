#!/usr/bin/env python3
"""
Calibration validation script for multi_wayside_node integration testing.

This script:
1. Subscribes to calibration transform topic
2. Validates transform parameters against expected ranges
3. Reports calibration quality and success/failure
4. Provides detailed diagnostics for debugging
"""

import argparse
import math
import sys
import time
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node


class CalibrationValidator(Node):
    def __init__(self, args):
        super().__init__("calibration_validator")

        self.args = args
        self.calibration_received = False
        self.latest_transform: Optional[TransformStamped] = None
        self.start_time = time.time()

        # Parse expected ranges
        self.translation_range = self._parse_range(args.expected_translation_range)
        self.rotation_range = self._parse_range(args.expected_rotation_range)

        # Subscribe to calibration transform
        self.calibration_sub = self.create_subscription(
            TransformStamped, "/calibration_transform", self.calibration_callback, 10
        )

        # Timer for timeout checking
        self.timer = self.create_timer(1.0, self.check_timeout)

        self.get_logger().info(
            f"Validation started - waiting up to {args.timeout}s for calibration"
        )
        self.get_logger().info(f"Expected translation range: {self.translation_range}")
        self.get_logger().info(f"Expected rotation range: {self.rotation_range}")

    def _parse_range(self, range_str: str) -> Tuple[float, float]:
        """Parse range string like '1.0,5.0' into tuple (min, max)"""
        parts = range_str.split(",")
        if len(parts) != 2:
            raise ValueError(f"Invalid range format: {range_str}")
        return (float(parts[0]), float(parts[1]))

    def calibration_callback(self, msg: TransformStamped):
        """Handle incoming calibration transform"""
        self.latest_transform = msg
        self.calibration_received = True

        self.get_logger().info("Calibration transform received!")
        self.validate_transform(msg)

    def validate_transform(self, transform: TransformStamped):
        """Validate the calibration transform against expected ranges"""
        # Extract translation and rotation
        translation = transform.transform.translation
        rotation = transform.transform.rotation

        # Compute magnitudes
        translation_magnitude = math.sqrt(
            translation.x ** 2 + translation.y ** 2 + translation.z ** 2
        )

        # Convert quaternion to rotation angle (simplified)
        rotation_angle = 2 * math.acos(abs(rotation.w))
        if rotation_angle > math.pi:
            rotation_angle = 2 * math.pi - rotation_angle

        # Log detailed information
        self.get_logger().info("=== Calibration Transform Validation ===")
        self.get_logger().info(
            f"Frame: {transform.header.frame_id} -> {transform.child_frame_id}"
        )
        self.get_logger().info(
            f"Translation: ({translation.x:.3f}, {translation.y:.3f}, {translation.z:.3f})"
        )
        self.get_logger().info(f"Translation magnitude: {translation_magnitude:.3f}m")
        self.get_logger().info(
            f"Rotation: ({rotation.x:.3f}, {rotation.y:.3f}, {rotation.z:.3f}, {rotation.w:.3f})"
        )
        self.get_logger().info(
            f"Rotation angle: {rotation_angle:.3f} rad ({math.degrees(rotation_angle):.1f}°)"
        )

        # Validate ranges
        translation_valid = (
            self.translation_range[0]
            <= translation_magnitude
            <= self.translation_range[1]
        )
        rotation_valid = (
            self.rotation_range[0] <= rotation_angle <= self.rotation_range[1]
        )

        # Report validation results
        if translation_valid:
            self.get_logger().info(
                f"✅ Translation magnitude VALID ({translation_magnitude:.3f}m within {self.translation_range})"
            )
        else:
            self.get_logger().error(
                f"❌ Translation magnitude INVALID ({translation_magnitude:.3f}m outside {self.translation_range})"
            )

        if rotation_valid:
            self.get_logger().info(
                f"✅ Rotation angle VALID ({rotation_angle:.3f} rad within {self.rotation_range})"
            )
        else:
            self.get_logger().error(
                f"❌ Rotation angle INVALID ({rotation_angle:.3f} rad outside {self.rotation_range})"
            )

        # Overall validation result
        overall_valid = translation_valid and rotation_valid

        if overall_valid:
            self.get_logger().info("🎉 CALIBRATION VALIDATION PASSED!")
            sys.exit(0)  # Success
        else:
            self.get_logger().error("💥 CALIBRATION VALIDATION FAILED!")
            sys.exit(1)  # Failure

    def check_timeout(self):
        """Check if we've exceeded the timeout"""
        elapsed = time.time() - self.start_time

        if elapsed > self.args.timeout:
            if self.calibration_received:
                self.get_logger().warn(
                    "Timeout reached, but calibration was received and validated"
                )
            else:
                self.get_logger().error(
                    f"⏰ TIMEOUT: No calibration received within {self.args.timeout}s"
                )
                sys.exit(2)  # Timeout failure


def main():
    parser = argparse.ArgumentParser(
        description="Validate multi_wayside_node calibration"
    )
    parser.add_argument(
        "--timeout",
        type=int,
        default=60,
        help="Timeout in seconds to wait for calibration",
    )
    parser.add_argument(
        "--expected_translation_range",
        type=str,
        default="0.5,10.0",
        help='Expected translation magnitude range as "min,max"',
    )
    parser.add_argument(
        "--expected_rotation_range",
        type=str,
        default="0.0,1.57",
        help='Expected rotation angle range as "min,max" (radians)',
    )

    args = parser.parse_args()

    rclpy.init()

    try:
        validator = CalibrationValidator(args)
        rclpy.spin(validator)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Validation failed with exception: {e}")
        sys.exit(3)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
