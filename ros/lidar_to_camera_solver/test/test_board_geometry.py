"""Camera-side tests for the shared Target Definition and identity gate."""

import subprocess
import sys
from pathlib import Path

import pytest
from lctk_target import load_target
from lidar_to_camera_solver.board_geometry import (
    CAMERA_TARGET_IDENTITY_TOPIC,
    LIDAR_TARGET_IDENTITY_TOPIC,
    TargetIdentityGate,
    identity_fields,
    identity_gate_error,
    marker_geometry_summary,
    parse_target_identity,
)

TARGETS = Path(__file__).resolve().parents[2] / "lctk_launch" / "config" / "targets"
SOLID = TARGETS / "solid_600_aruco_1_v1.json5"
HOLLOW = TARGETS / "hollow_1000_aruco_4_v1.json5"


def wire_identity(target):
    return identity_fields(target.identity)


def test_module_imports_without_rclpy():
    """The shared geometry/gate stays testable without a ROS graph."""

    probe = (
        "import sys;"
        "import lidar_to_camera_solver.board_geometry;"
        "print(f'LCTK_BOARD_GEOMETRY_RCLPY={\"rclpy\" in sys.modules}')"
    )
    result = subprocess.run(
        [sys.executable, "-c", probe], capture_output=True, text=True, check=False
    )
    assert result.returncode == 0, result.stderr
    assert "LCTK_BOARD_GEOMETRY_RCLPY=False" in result.stdout


def test_target_geometry_comes_from_shared_reader():
    solid = load_target(SOLID)
    hollow = load_target(HOLLOW)

    assert solid.plate.side_um == 600_000
    assert solid.fiducial.marker_ids == (1,)
    assert len(solid.marker_corners_by_id[1]) == 4
    assert hollow.plate.side_um == 1_000_000
    assert len(hollow.marker_corners_by_id) == 4
    assert "marker_size=480.0mm" in marker_geometry_summary(solid)


def test_identity_topics_are_relative_for_launch_routing():
    assert not LIDAR_TARGET_IDENTITY_TOPIC.startswith("/")
    assert not CAMERA_TARGET_IDENTITY_TOPIC.startswith("/")


@pytest.fixture
def solid_identity():
    return wire_identity(load_target(SOLID))


def test_identity_gate_decision_table(solid_identity):
    different = dict(solid_identity, target_id="hollow_1000_aruco_4")

    assert "missing" in identity_gate_error(
        parse_target_identity(solid_identity), None, solid_identity
    )
    assert "malformed" in identity_gate_error(
        parse_target_identity(solid_identity), solid_identity, {}
    )
    assert "does not exactly match" in identity_gate_error(
        parse_target_identity(solid_identity), different, solid_identity
    )
    assert (
        identity_gate_error(
            parse_target_identity(solid_identity), solid_identity, solid_identity
        )
        is None
    )


def test_identity_gate_allows_late_join_and_rejects_restart(solid_identity):
    local = parse_target_identity(solid_identity)
    gate = TargetIdentityGate(local)

    assert not gate.ready
    assert gate.update("lidar", solid_identity) is not None
    assert not gate.ready
    assert gate.update("camera", solid_identity) is None
    assert gate.ready

    changed = dict(solid_identity, revision=2)
    reason = gate.update("camera", changed)
    assert reason is not None
    assert "changed during this solver session" in reason
    assert not gate.ready
