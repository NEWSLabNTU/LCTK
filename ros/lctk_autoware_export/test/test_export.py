"""Exporter behavior tests (Phase 6, deliverable 2): YAML patching, errors, dry-run."""

import json
import math
import shutil
from pathlib import Path

import numpy as np
import pytest
from lctk_autoware_export.export import (
    ExportError,
    load_solver_transform,
    patch_calibration,
)
from lctk_autoware_export.frames import OPTICAL_IN_CAMERA_LINK, inv_transform

FIXTURE = Path(__file__).parent / "fixtures" / "sensor_kit_calibration.yaml"


def solve_for_forward_camera():
    """rvec/tvec for a camera 0.5 m ahead of the lidar, both facing +x."""
    T_lidar_optical = OPTICAL_IN_CAMERA_LINK.copy()
    T_lidar_optical[:3, 3] = [0.5, 0.0, 0.0]
    T_solve = inv_transform(T_lidar_optical)
    R = T_solve[:3, :3]
    angle = math.acos(max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0)))
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (
        2.0 * math.sin(angle)
    )
    return (axis * angle).tolist(), T_solve[:3, 3].tolist()


@pytest.fixture
def target(tmp_path):
    dst = tmp_path / "sensor_kit_calibration.yaml"
    shutil.copy(FIXTURE, dst)
    return dst


@pytest.fixture
def detections(tmp_path):
    rvec, tvec = solve_for_forward_camera()
    p = tmp_path / "detections.json"
    p.write_text(
        json.dumps(
            {
                "version": 4,
                "board_frame_convention": "corner_aligned_plate_center_v1",
                "transform": {"rvec": rvec, "tvec": tvec},
            }
        )
    )
    return p


def test_load_solver_transform(detections):
    rvec, tvec = load_solver_transform(detections)
    assert rvec.shape == (3,) and tvec.shape == (3,)


def test_load_solver_transform_missing_transform(tmp_path):
    p = tmp_path / "detections.json"
    p.write_text(
        json.dumps(
            {
                "version": 4,
                "board_frame_convention": "corner_aligned_plate_center_v1",
                "detections": [],
            }
        )
    )
    with pytest.raises(ExportError, match="dump_detections"):
        load_solver_transform(p)


def test_load_solver_transform_refuses_an_older_format(tmp_path):
    """H-11: this exporter writes into a file that reaches a vehicle, so it is the
    single most important place for a version check. It had none: its own fixtures
    declared "version": 2 and passed. A version-3 file predates the corner-aligned
    board frame, so its transform may be wrong by a silent 45 degrees."""
    rvec, tvec = solve_for_forward_camera()
    p = tmp_path / "detections.json"
    p.write_text(json.dumps({"version": 3, "transform": {"rvec": rvec, "tvec": tvec}}))
    with pytest.raises(ExportError, match="version"):
        load_solver_transform(p)


def test_load_solver_transform_refuses_a_stale_frame_convention(tmp_path):
    rvec, tvec = solve_for_forward_camera()
    p = tmp_path / "detections.json"
    p.write_text(
        json.dumps(
            {
                "version": 4,
                "board_frame_convention": "edge_aligned_corner_origin_v0",
                "transform": {"rvec": rvec, "tvec": tvec},
            }
        )
    )
    with pytest.raises(ExportError, match="edge_aligned_corner_origin_v0"):
        load_solver_transform(p)


def test_patch_updates_only_target_entry(target, detections):
    rvec, tvec = load_solver_transform(detections)
    before = target.read_text()
    entry = patch_calibration(
        target,
        rvec=rvec,
        tvec=tvec,
        camera_frame="camera0/camera_link",
        lidar_frame="velodyne_top_base_link",
    )
    after = target.read_text()
    assert after != before
    # comments preserved
    assert "# top lidar is the kit reference" in after
    assert "# Angles are radians" in after
    # untouched entries byte-identical
    for line in ("    yaw: 1.575", "  gnss_link:", "    z: -0.2"):
        assert line in after
    # patched entry reflects composed transform: lidar yawed 1.575, camera 0.5 m ahead of it
    assert entry["x"] == pytest.approx(0.5 * math.cos(1.575), abs=1e-9)
    assert entry["y"] == pytest.approx(0.5 * math.sin(1.575), abs=1e-9)
    assert entry["yaw"] == pytest.approx(1.575, abs=1e-9)
    assert f"yaw: {entry['yaw']}" in after or "yaw: 1.575" in after


def test_patch_missing_lidar_entry_lists_children(target, detections):
    rvec, tvec = load_solver_transform(detections)
    with pytest.raises(ExportError, match="velodyne_top_base_link"):
        patch_calibration(
            target,
            rvec=rvec,
            tvec=tvec,
            camera_frame="camera0/camera_link",
            lidar_frame="no_such_lidar",
        )


def test_patch_missing_kit_frame_errors(target, detections):
    rvec, tvec = load_solver_transform(detections)
    with pytest.raises(ExportError, match="no_such_kit"):
        patch_calibration(
            target,
            rvec=rvec,
            tvec=tvec,
            camera_frame="camera0/camera_link",
            lidar_frame="velodyne_top_base_link",
            kit_frame="no_such_kit",
        )


def test_dry_run_writes_nothing(target, detections):
    rvec, tvec = load_solver_transform(detections)
    before = target.read_text()
    entry = patch_calibration(
        target,
        rvec=rvec,
        tvec=tvec,
        camera_frame="camera0/camera_link",
        lidar_frame="velodyne_top_base_link",
        dry_run=True,
    )
    assert target.read_text() == before
    assert not target.with_suffix(".yaml.bak").exists()
    assert entry["x"] == pytest.approx(0.5 * math.cos(1.575), abs=1e-9)


def test_backup_created(target, detections):
    rvec, tvec = load_solver_transform(detections)
    original = target.read_text()
    patch_calibration(
        target,
        rvec=rvec,
        tvec=tvec,
        camera_frame="camera0/camera_link",
        lidar_frame="velodyne_top_base_link",
    )
    bak = target.with_suffix(".yaml.bak")
    assert bak.exists()
    assert bak.read_text() == original


def test_new_camera_entry_appended(target, detections):
    rvec, tvec = load_solver_transform(detections)
    patch_calibration(
        target,
        rvec=rvec,
        tvec=tvec,
        camera_frame="camera9/camera_link",
        lidar_frame="velodyne_top_base_link",
    )
    assert "camera9/camera_link:" in target.read_text()


def test_cli_dry_run(target, detections, capsys):
    from lctk_autoware_export.export import main

    rc = main(
        [
            "--detections",
            str(detections),
            "--target",
            str(target),
            "--camera-frame",
            "camera0/camera_link",
            "--lidar-frame",
            "velodyne_top_base_link",
            "--dry-run",
        ]
    )
    assert rc == 0
    out = capsys.readouterr().out
    assert "would write" in out and "yaw:" in out


def test_cli_error_exit_code(target, detections, capsys):
    from lctk_autoware_export.export import main

    rc = main(
        [
            "--detections",
            str(detections),
            "--target",
            str(target),
            "--camera-frame",
            "camera0/camera_link",
            "--lidar-frame",
            "bogus",
        ]
    )
    assert rc == 1
    assert "velodyne_top_base_link" in capsys.readouterr().err
