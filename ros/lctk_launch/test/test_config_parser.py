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
    config_path = (
        Path(__file__).parent.parent / "config" / "examples" / "sample_data.yaml"
    )
    if not config_path.exists():
        import pytest

        pytest.skip(f"Config file not found: {config_path}")

    parser = CalibrationConfigParser(str(config_path))
    pipeline = parser.parse()

    # Basic assertions
    assert len(pipeline.lidar_board_detectors) == 1, "Expected 1 board detector"
    assert len(pipeline.aruco_locators) == 1, "Expected 1 aruco locator"
    assert len(pipeline.lidar_camera_solvers) == 1, "Expected 1 lidar-camera solver"
    assert len(pipeline.lidar_lidar_solvers) == 0, "Expected 0 lidar-lidar solvers"

    # Calibration plan assertions
    assert pipeline.calibration_plan is not None, "Expected calibration plan"
    assert pipeline.calibration_plan.reference_frame == "top_lidar"
    assert len(pipeline.calibration_plan.tree_edges) == 1
    assert len(pipeline.calibration_plan.validation_edges) == 0


def test_vehicle_config():
    """Test parsing the vehicle.yaml config (multi-sensor)."""
    config_path = Path(__file__).parent.parent / "config" / "examples" / "vehicle.yaml"
    if not config_path.exists():
        import pytest

        pytest.skip(f"Config file not found: {config_path}")

    parser = CalibrationConfigParser(str(config_path))
    pipeline = parser.parse()

    # Calibration plan assertions
    assert pipeline.calibration_plan is not None, "Expected calibration plan"
    assert pipeline.calibration_plan.reference_frame == "L1"
    assert len(pipeline.calibration_plan.tree_edges) == 5
    assert len(pipeline.calibration_plan.validation_edges) == 0


def test_two_lidar_config():
    """Test parsing the two_lidar.yaml config."""
    config_path = (
        Path(__file__).parent.parent / "config" / "examples" / "two_lidar.yaml"
    )
    if not config_path.exists():
        import pytest

        pytest.skip(f"Config file not found: {config_path}")

    parser = CalibrationConfigParser(str(config_path))
    pipeline = parser.parse()

    # Basic assertions
    assert len(pipeline.lidar_board_detectors) == 2, "Expected 2 board detectors"
    assert len(pipeline.aruco_locators) == 0, "Expected 0 aruco locators"
    assert len(pipeline.lidar_camera_solvers) == 0, "Expected 0 lidar-camera solvers"
    assert len(pipeline.lidar_lidar_solvers) == 1, "Expected 1 lidar-lidar solver"

    # Device dicts populated
    assert len(pipeline.lidars) == 2
    assert "top_lidar" in pipeline.lidars
    assert "front_lidar" in pipeline.lidars
    assert len(pipeline.cameras) == 0

    # Calibration plan assertions
    assert pipeline.calibration_plan is not None
    assert pipeline.calibration_plan.reference_frame == "top_lidar"
    assert len(pipeline.calibration_plan.tree_edges) == 1
    assert len(pipeline.calibration_plan.validation_edges) == 0


def test_sample_data_node_parity():
    """Verify sample_data.yaml produces nodes matching what the XML would generate.

    The XML path (lidar_camera_calibration.launch.xml) produced:
    - 1 aruco_locator in calibration/aruco_locator namespace
    - 1 lidar_board_detector in calibration/lidar_board_detector namespace
    - 1 extrinsic_solver in calibration/extrinsic_solver namespace

    The config-driven path should produce equivalent nodes (possibly different
    namespaces, but same count and correct topic wiring).
    """
    config_path = (
        Path(__file__).parent.parent / "config" / "examples" / "sample_data.yaml"
    )
    if not config_path.exists():
        import pytest

        pytest.skip(f"Config file not found: {config_path}")

    parser = CalibrationConfigParser(str(config_path))
    pipeline = parser.parse()

    # 1 aruco locator
    assert len(pipeline.aruco_locators) == 1
    locator = pipeline.aruco_locators[0]
    assert locator.camera_name == "front_center"
    assert locator.image_topic == "/sensing/camera/front_center/image_raw"

    # 1 board detector
    assert len(pipeline.lidar_board_detectors) == 1
    detector = pipeline.lidar_board_detectors[0]
    assert detector.lidar_name == "top_lidar"
    assert detector.pointcloud_topic == "/sensing/lidar/top/pointcloud_raw"

    # 1 lidar-camera solver
    assert len(pipeline.lidar_camera_solvers) == 1
    solver = pipeline.lidar_camera_solvers[0]
    assert solver.lidar_name == "top_lidar"
    assert solver.camera_name == "front_center"
    assert solver.parent_frame == "velodyne_top"
    assert solver.child_frame == "camera_front_center"

    # Correct topic wiring: solver subscribes to detector/locator output topics
    assert solver.board_detections_topic == detector.output_topic
    assert solver.aruco_detections_topic == locator.output_topic
    assert solver.camera_topic == locator.image_topic

    # Device dicts populated
    assert len(pipeline.lidars) == 1
    assert len(pipeline.cameras) == 1
    assert (
        pipeline.lidars["top_lidar"].pointcloud_topic
        == "/sensing/lidar/top/pointcloud_raw"
    )
    assert (
        pipeline.cameras["front_center"].image_topic
        == "/sensing/camera/front_center/image_raw"
    )


def test_two_lidar_node_parity():
    """Verify two_lidar.yaml produces correct nodes for two-lidar calibration.

    Expected:
    - 2 lidar_board_detectors (one per lidar)
    - 1 lidar-lidar solver
    - 0 aruco locators (no cameras)
    """
    config_path = (
        Path(__file__).parent.parent / "config" / "examples" / "two_lidar.yaml"
    )
    if not config_path.exists():
        import pytest

        pytest.skip(f"Config file not found: {config_path}")

    parser = CalibrationConfigParser(str(config_path))
    pipeline = parser.parse()

    # 2 board detectors
    assert len(pipeline.lidar_board_detectors) == 2
    detector_names = {d.lidar_name for d in pipeline.lidar_board_detectors}
    assert detector_names == {"top_lidar", "front_lidar"}

    # 0 aruco locators
    assert len(pipeline.aruco_locators) == 0

    # 1 lidar-lidar solver
    assert len(pipeline.lidar_lidar_solvers) == 1
    solver = pipeline.lidar_lidar_solvers[0]
    assert {solver.lidar1_name, solver.lidar2_name} == {"top_lidar", "front_lidar"}
    # Frame ids come from two_lidar.yaml, which describes the real two-LiDAR rig
    # (VLP-32C + Seyond Falcon). The recorded bags publish exactly these frames.
    frames = {
        solver.lidar1_name: solver.lidar1_frame,
        solver.lidar2_name: solver.lidar2_frame,
    }
    assert frames["top_lidar"] == "velodyne"
    assert frames["front_lidar"] == "seyond"

    # Correct topic wiring: solver subscribes to detector output topics
    detector_by_lidar = {d.lidar_name: d for d in pipeline.lidar_board_detectors}
    assert (
        solver.lidar1_detections_topic
        == detector_by_lidar[solver.lidar1_name].output_topic
    )
    assert (
        solver.lidar2_detections_topic
        == detector_by_lidar[solver.lidar2_name].output_topic
    )

    # 0 lidar-camera solvers
    assert len(pipeline.lidar_camera_solvers) == 0


def test_hollow_board_missing_bbox_config_raises(tmp_path):
    """H-04: a hollow_board marker used by a lidar must have bbox_config.

    The lidar_board_detector declares bbox_file/aruco_pattern_file as mandatory
    ROS parameters, so the config parser must reject a marker that omits them
    (with a clear message) rather than letting the node crash at startup.
    """
    import pytest

    config_text = """
devices:
  lidars:
    top_lidar:
      pointcloud_topic: /lidar/points
      frame_id: velodyne_top
  cameras:
    front_center:
      image_topic: /camera/image_raw
      frame_id: camera_front_center

markers:
  calibration_board:
    type: hollow_board
    board_config: /tmp/board.json5
    aruco_config: /tmp/aruco.json5
    # bbox_config intentionally omitted
    pairs:
      - [top_lidar, front_center]
"""
    config_path = tmp_path / "missing_bbox.yaml"
    config_path.write_text(config_text)

    parser = CalibrationConfigParser(str(config_path))
    with pytest.raises(ValueError, match="bbox_config"):
        parser.parse()


if __name__ == "__main__":
    import inspect

    tests = [
        obj
        for name, obj in sorted(globals().items())
        if name.startswith("test_") and inspect.isfunction(obj)
    ]

    passed = 0
    failed = 0
    for test in tests:
        try:
            test()
            print(f"  PASS: {test.__name__}")
            passed += 1
        except Exception as e:  # noqa: BLE001 - hand-rolled runner: reporting per-test failure is the point
            print(f"  FAIL: {test.__name__}: {e}")
            failed += 1

    print()
    print(f"{passed} passed, {failed} failed")
    sys.exit(0 if failed == 0 else 1)
