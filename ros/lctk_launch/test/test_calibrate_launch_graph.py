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

# Keep source-tree execution consistent with the existing lctk_launch tests.
sys.path.insert(0, str(PACKAGE_ROOT))


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
