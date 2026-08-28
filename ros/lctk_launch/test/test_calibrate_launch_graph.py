"""Graph contracts for the config-driven calibration launch."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import ModuleType

import pytest
from launch_ros.actions import Node

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_FILE = PACKAGE_ROOT / "launch" / "calibrate.launch.py"
CONFIG_ROOT = PACKAGE_ROOT / "config" / "examples"
TARGETS_ROOT = PACKAGE_ROOT / "config" / "targets"

# Keep source-tree execution consistent with the existing lctk_launch tests.
sys.path.insert(0, str(PACKAGE_ROOT))


def _write_new_schema_detector_config(tmp_path: Path) -> Path:
    """Write a new-schema (target_config/detector_config) marker config.

    Mirrors ``test_config_parser._write_new_schema_config``'s shape: only the
    Target Definition manifest (``target_config``) needs to exist on disk to
    parse -- ``detector_config``/``bbox_config`` are opaque paths as far as
    the parser and this launch file are concerned. Maintained examples under
    ``config/examples/`` stay legacy-only (W5-D); this writes its own file
    into pytest's tmp_path instead of touching them.

    Deliberately a two-LiDAR pairing (no camera), like
    ``config/examples/two_lidar.yaml``: this piece (W5-C/C1) only routes the
    new schema to lidar_board_detector. aruco_locator_node still ships the
    legacy-only ``aruco_config_file`` param unconditionally (W5-C/C2's job),
    and launch_ros's ``Node()`` normalizes -- and rejects ``None`` -- params
    at construction time. A camera pair here would build that locator node
    and fail before this test ever reached the detector params it means to
    check.
    """

    target_config = TARGETS_ROOT / "solid_600_aruco_1_v1.json5"
    # Never opened by the parser or this launch file -- just opaque path
    # strings that must round-trip unchanged. Named "not-a-real-file" and
    # placed under tmp_path (never /tmp/, per CLAUDE.md) so no reader
    # mistakes them for files this test depends on existing.
    detector_config = tmp_path / "not-a-real-file-detector-tuning.json5"
    bbox_config = tmp_path / "not-a-real-file-bbox.json5"
    config_path = tmp_path / "new_schema.yaml"
    config_path.write_text(
        f"""
devices:
  lidars:
    top_lidar:
      pointcloud_topic: /velodyne_points
      frame_id: velodyne
    front_lidar:
      pointcloud_topic: /iv_points
      frame_id: seyond

markers:
  calibration_target:
    target_config: {target_config}
    detector_config: {detector_config}
    bbox_config: {bbox_config}
    pairs:
      - [top_lidar, front_lidar]
"""
    )
    return config_path


class _LaunchContext:
    """Minimal launch context for evaluating LaunchConfiguration values."""

    def __init__(self, config_file: Path):
        self.launch_configurations = {
            "config_file": str(config_file),
            "debug_mode": "false",
            "log_level": "info",
            "mode": "offline",
            "solver_mode": "continuous",
            "enable_overlay": "false",
            "enable_judge": "false",
        }

    def perform_substitution(self, substitution):
        return substitution.perform(self)


@pytest.fixture(scope="module")
def calibrate_launch() -> ModuleType:
    """Load the launch file directly; it is not a Python package module."""

    spec = importlib.util.spec_from_file_location("calibrate_launch", LAUNCH_FILE)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _nodes_for_package(nodes, package: str) -> list[Node]:
    return [
        node
        for node in nodes
        if isinstance(node, Node) and vars(node)["_Node__package"] == package
    ]


def _resolve(value):
    """Resolve a launch substitution stored in a Node's private graph data."""

    if isinstance(value, tuple):
        assert len(value) == 1
        value = value[0]
    if hasattr(value, "perform"):
        value = value.perform(None)
    if isinstance(value, str):
        # launch_ros serializes string parameter values as YAML documents.
        value = value.removesuffix("\n...\n")
    return value


def _parameters(node: Node) -> dict:
    parameters = {}
    for parameter_set in vars(node)["_Node__parameters"]:
        for key, value in parameter_set.items():
            parameters[_resolve(key)] = _resolve(value)
    return parameters


def _remappings(node: Node) -> dict[str, str]:
    return {
        _resolve(source): _resolve(destination)
        for source, destination in vars(node)["_Node__remappings"]
    }


def test_legacy_lidar_camera_graph_routes_each_identity(
    calibrate_launch: ModuleType,
):
    """Legacy hollow graph keeps detection wiring and adds exact identity routes."""

    nodes = calibrate_launch.generate_nodes(
        _LaunchContext(CONFIG_ROOT / "sample_data.yaml")
    )

    detectors = _nodes_for_package(nodes, "lidar_board_detector")
    locators = _nodes_for_package(nodes, "aruco_locator_node")
    solvers = _nodes_for_package(nodes, "lidar_to_camera_solver")
    assert len(detectors) == 1
    assert len(locators) == 1
    assert len(solvers) == 1

    solver_params = _parameters(solvers[0])
    assert solver_params["lidar_target_identity_topic"] == "lidar_target_identity"
    assert solver_params["camera_target_identity_topic"] == "camera_target_identity"

    remappings = _remappings(solvers[0])
    assert (
        remappings["aruco_detections"] == "/calibration/front_center/aruco_detections"
    )
    assert (
        remappings["calibration_board_detections"]
        == "/calibration/top_lidar_calibration_board/calibration_board_detections"
    )
    assert (
        remappings["lidar_target_identity"]
        == "/calibration/top_lidar_calibration_board/target_identity"
    )
    assert (
        remappings["camera_target_identity"]
        == "/calibration/front_center/target_identity"
    )


def test_legacy_lidar_lidar_graph_routes_each_identity(
    calibrate_launch: ModuleType,
):
    """Legacy hollow two-LiDAR graph routes both detector identities exactly."""

    nodes = calibrate_launch.generate_nodes(
        _LaunchContext(CONFIG_ROOT / "two_lidar.yaml")
    )

    detectors = _nodes_for_package(nodes, "lidar_board_detector")
    solvers = _nodes_for_package(nodes, "lidar_to_lidar_solver")
    assert len(detectors) == 2
    assert len(solvers) == 1

    remappings = _remappings(solvers[0])
    assert remappings == {
        "lidar1_target_identity": "/calibration/top_lidar_calibration_board/target_identity",
        "lidar2_target_identity": "/calibration/front_lidar_calibration_board/target_identity",
    }


def test_new_schema_detector_gets_target_config_and_omits_legacy_keys(
    calibrate_launch: ModuleType, tmp_path: Path
):
    """A new-schema marker routes target_config/detector_config to the node
    and must not carry the legacy board_detector_file/aruco_pattern_file keys
    at all -- select_config_source in lidar_board_detector's main.rs refuses
    to start if both sources are present, and a present-but-None value is
    still "present" to it.
    """

    config_path = _write_new_schema_detector_config(tmp_path)
    nodes = calibrate_launch.generate_nodes(_LaunchContext(config_path))

    detectors = _nodes_for_package(nodes, "lidar_board_detector")
    # One detector per lidar in the pair (top_lidar, front_lidar).
    assert len(detectors) == 2

    for detector in detectors:
        params = _parameters(detector)

        assert params["target_config"].endswith("solid_600_aruco_1_v1.json5")
        assert params["detector_config"].endswith(
            "not-a-real-file-detector-tuning.json5"
        )
        assert "board_detector_file" not in params
        assert "aruco_pattern_file" not in params
        assert params["bbox_file"].endswith("not-a-real-file-bbox.json5")
        assert None not in params.values()


def test_legacy_detector_gets_legacy_keys_and_omits_new_schema_keys(
    calibrate_launch: ModuleType,
):
    """A legacy marker keeps today's board_detector_file/aruco_pattern_file
    params and must not carry target_config/detector_config -- those would
    make the node see both sources and refuse to start.
    """

    nodes = calibrate_launch.generate_nodes(
        _LaunchContext(CONFIG_ROOT / "sample_data.yaml")
    )

    detectors = _nodes_for_package(nodes, "lidar_board_detector")
    assert len(detectors) == 1

    params = _parameters(detectors[0])

    assert params["board_detector_file"].endswith("board_detector.json5")
    assert params["aruco_pattern_file"].endswith("aruco_pattern.json5")
    assert "target_config" not in params
    assert "detector_config" not in params
    assert params["bbox_file"].endswith("bbox.json5")
    assert None not in params.values()
