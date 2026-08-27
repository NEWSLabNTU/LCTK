"""Structural Target Identity validation for the LiDAR-camera gate."""

from pathlib import Path

import pytest
from lctk_target import load_target
from lidar_to_camera_solver.board_geometry import (
    BOARD_FRAME_CONVENTION,
    identity_fields,
    parse_target_identity,
    target_identity_error,
)

SOLID = (
    Path(__file__).resolve().parents[2]
    / "lctk_launch"
    / "config"
    / "targets"
    / "solid_600_aruco_1_v1.json5"
)


@pytest.fixture
def identity():
    return identity_fields(load_target(SOLID).identity)


def test_expected_frame_convention_is_shared(identity):
    assert identity["board_frame_convention"] == BOARD_FRAME_CONVENTION
    assert (
        parse_target_identity(identity).board_frame_convention == BOARD_FRAME_CONVENTION
    )


@pytest.mark.parametrize(
    "field,value",
    [
        ("schema_version", 0),
        ("target_id", ""),
        ("revision", 0),
        ("semantic_sha256", "not-a-hash"),
        ("board_frame_convention", ""),
    ],
)
def test_malformed_identity_fails_closed(identity, field, value):
    malformed = dict(identity, **{field: value})
    error = target_identity_error(malformed)
    assert error is not None
    assert field in error


def test_exact_wire_fields_are_required(identity):
    malformed = dict(identity)
    del malformed["revision"]
    error = target_identity_error(malformed)
    assert error is not None
    assert "revision" in error


def test_surrounding_identity_values_are_not_normalized(identity):
    malformed = dict(identity, target_id=f" {identity['target_id']} ")
    parsed = parse_target_identity(malformed)
    assert parsed.target_id != identity["target_id"]
