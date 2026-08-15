"""Frame-algebra tests for the Autoware exporter (Phase 6, deliverable 1)."""

import math

import numpy as np
import pytest
from lctk_autoware_export.frames import (
    OPTICAL_IN_CAMERA_LINK,
    entry_to_transform,
    inv_transform,
    kit_to_camera_link,
    matrix_to_rpy,
    rodrigues,
    rpy_to_matrix,
    transform_to_entry,
)


def random_rpy(rng):
    # pitch away from the +-pi/2 gimbal-lock poles
    return (
        rng.uniform(-math.pi, math.pi),
        rng.uniform(-1.4, 1.4),
        rng.uniform(-math.pi, math.pi),
    )


def test_rpy_matrix_roundtrip():
    rng = np.random.default_rng(42)
    for _ in range(200):
        rpy = random_rpy(rng)
        rec = matrix_to_rpy(rpy_to_matrix(*rpy))
        np.testing.assert_allclose(rpy_to_matrix(*rec), rpy_to_matrix(*rpy), atol=1e-12)


def test_rpy_is_fixed_axis_xyz():
    # URDF convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    r, p, y = 0.3, -0.2, 1.1
    cz, sz = math.cos(y), math.sin(y)
    cy, sy = math.cos(p), math.sin(p)
    cx, sx = math.cos(r), math.sin(r)
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    np.testing.assert_allclose(rpy_to_matrix(r, p, y), Rz @ Ry @ Rx, atol=1e-12)


def test_rodrigues_known_rotation():
    R = rodrigues(np.array([0.0, 0.0, math.pi / 2]))
    np.testing.assert_allclose(R @ [1, 0, 0], [0, 1, 0], atol=1e-12)


def test_rodrigues_zero_vector_is_identity():
    np.testing.assert_allclose(rodrigues(np.zeros(3)), np.eye(3), atol=1e-15)


def test_optical_in_camera_link_convention():
    # optical z (forward) -> link x; optical x (right) -> link -y; optical y (down) -> link -z
    R = OPTICAL_IN_CAMERA_LINK[:3, :3]
    np.testing.assert_allclose(R @ [0, 0, 1], [1, 0, 0], atol=1e-12)
    np.testing.assert_allclose(R @ [1, 0, 0], [0, -1, 0], atol=1e-12)
    np.testing.assert_allclose(R @ [0, 1, 0], [0, 0, -1], atol=1e-12)
    np.testing.assert_allclose(OPTICAL_IN_CAMERA_LINK[:3, 3], 0, atol=1e-15)


def test_entry_transform_roundtrip():
    entry = {
        "x": 0.1,
        "y": -0.5,
        "z": 2.0,
        "roll": -0.025,
        "pitch": 0.315,
        "yaw": 1.035,
    }
    rec = transform_to_entry(entry_to_transform(entry))
    for k, v in entry.items():
        assert rec[k] == pytest.approx(v, abs=1e-12)


def test_kit_to_camera_link_forward_camera():
    """Camera 0.5 m ahead of the lidar (kit==lidar), both facing +x: entry
    must be pure translation."""
    # Pose of the optical frame in the lidar frame: same attitude as a
    # forward-facing camera_link, i.e. the fixed optical<-link rotation.
    T_lidar_optical = OPTICAL_IN_CAMERA_LINK.copy()
    T_lidar_optical[:3, 3] = [0.5, 0.0, 0.0]
    # Solver output is the inverse mapping (lidar points -> optical frame).
    T_solve = inv_transform(T_lidar_optical)
    rvec = matrix_to_rvec_for_test(T_solve[:3, :3])
    tvec = T_solve[:3, 3]

    T_kit_lidar = np.eye(4)
    T = kit_to_camera_link(T_kit_lidar, rvec, tvec)
    entry = transform_to_entry(T)
    assert entry["x"] == pytest.approx(0.5, abs=1e-9)
    for k in ("y", "z", "roll", "pitch", "yaw"):
        assert entry[k] == pytest.approx(0.0, abs=1e-9)


def test_kit_to_camera_link_composes_lidar_entry():
    """A yawed lidar mount must rotate the exported camera pose with it."""
    yaw = 1.575
    T_kit_lidar = entry_to_transform(
        {"x": 0.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": yaw}
    )
    T_lidar_optical = OPTICAL_IN_CAMERA_LINK.copy()
    T_lidar_optical[:3, 3] = [0.5, 0.0, 0.0]
    T_solve = inv_transform(T_lidar_optical)
    rvec = matrix_to_rvec_for_test(T_solve[:3, :3])
    tvec = T_solve[:3, 3]

    T = kit_to_camera_link(T_kit_lidar, rvec, tvec)
    expected = T_kit_lidar @ T_lidar_optical @ inv_transform(OPTICAL_IN_CAMERA_LINK)
    np.testing.assert_allclose(T, expected, atol=1e-12)
    entry = transform_to_entry(T)
    assert entry["yaw"] == pytest.approx(yaw, abs=1e-9)
    assert entry["x"] == pytest.approx(0.5 * math.cos(yaw), abs=1e-9)
    assert entry["y"] == pytest.approx(0.5 * math.sin(yaw), abs=1e-9)


def matrix_to_rvec_for_test(R):
    """Log map, test-side only (implementation must not depend on it)."""
    angle = math.acos(max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0)))
    if angle < 1e-12:
        return np.zeros(3)
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (
        2.0 * math.sin(angle)
    )
    return axis * angle
