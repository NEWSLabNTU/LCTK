#!/usr/bin/env python3
"""Test script for calibration config parser.

Usage:
    # Requires ROS environment to be sourced
    source install/setup.bash
    python3 ros/lctk_launch/test/test_config_parser.py
"""

import sys
from pathlib import Path

# Add the package to path for standalone testing
sys.path.insert(0, str(Path(__file__).parent.parent))

try:
    from lctk_launch.config_parser import CalibrationConfigParser
except ImportError as e:
    print(f"Error: {e}")
    print()
    print("This test requires the ROS environment to be sourced:")
    print("  source install/setup.bash")
    print("  python3 ros/lctk_launch/test/test_config_parser.py")
    sys.exit(1)


def test_sample_data_config():
    """Test parsing the sample_data.yaml config."""
    config_path = Path(__file__).parent.parent / "config" / "examples" / "sample_data.yaml"
    if not config_path.exists():
        print(f"Config file not found: {config_path}")
        return False

    parser = CalibrationConfigParser(str(config_path))
    pipeline = parser.parse()

    print("=== Pipeline Configuration (sample_data.yaml) ===")
    print()
    print("LiDAR Board Detectors:")
    for d in pipeline.lidar_board_detectors:
        print(f"  - {d.node_name}: {d.lidar_name} -> {d.marker_name}")
        print(f"    Input: {d.pointcloud_topic}")
        print(f"    Output: {d.output_topic}")
    print()
    print("ArUco Locators:")
    for l in pipeline.aruco_locators:
        print(f"  - {l.node_name}: {l.camera_name}")
        print(f"    Input: {l.image_topic}")
        print(f"    Output: {l.output_topic}")
    print()
    print("LiDAR-Camera Solvers:")
    for s in pipeline.lidar_camera_solvers:
        print(f"  - {s.node_name}: {s.lidar_name} <-> {s.camera_name}")
        print(f"    Board: {s.board_detections_topic}")
        print(f"    ArUco: {s.aruco_detections_topic}")
        print(f"    Output: {s.output_topic}")
    print()
    print("LiDAR-LiDAR Solvers:")
    for s in pipeline.lidar_lidar_solvers:
        print(f"  - {s.node_name}: {s.lidar1_name} <-> {s.lidar2_name}")
        print(f"    L1 detections: {s.lidar1_detections_topic}")
        print(f"    L2 detections: {s.lidar2_detections_topic}")
        print(f"    Output: {s.output_topic}")

    # Basic assertions
    assert len(pipeline.lidar_board_detectors) == 1, "Expected 1 board detector"
    assert len(pipeline.aruco_locators) == 1, "Expected 1 aruco locator"
    assert len(pipeline.lidar_camera_solvers) == 1, "Expected 1 lidar-camera solver"
    assert len(pipeline.lidar_lidar_solvers) == 0, "Expected 0 lidar-lidar solvers"

    print()
    print("All assertions passed!")
    return True


def test_vehicle_config():
    """Test parsing the vehicle.yaml config (multi-sensor)."""
    config_path = Path(__file__).parent.parent / "config" / "examples" / "vehicle.yaml"
    if not config_path.exists():
        print(f"Config file not found: {config_path}")
        return False

    parser = CalibrationConfigParser(str(config_path))
    pipeline = parser.parse()

    print("=== Pipeline Configuration (vehicle.yaml) ===")
    print()
    print(f"LiDAR Board Detectors: {len(pipeline.lidar_board_detectors)}")
    print(f"ArUco Locators: {len(pipeline.aruco_locators)}")
    print(f"LiDAR-Camera Solvers: {len(pipeline.lidar_camera_solvers)}")
    print(f"LiDAR-LiDAR Solvers: {len(pipeline.lidar_lidar_solvers)}")

    print()
    print("Vehicle config parsed successfully!")
    return True


if __name__ == "__main__":
    success = True
    success = test_sample_data_config() and success
    print()
    success = test_vehicle_config() and success

    sys.exit(0 if success else 1)
