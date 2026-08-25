"""Exporter archive compatibility stays independent of target-manifest loading."""

import copy
import json
from pathlib import Path

import pytest
from lctk_autoware_export.export import (
    ExportError,
    load_solver_transform,
    patch_calibration,
)

FIXTURES = Path(__file__).resolve().parents[3] / "fixtures" / "detection_archives"
CALIBRATION_FIXTURE = Path(__file__).parent / "fixtures" / "sensor_kit_calibration.yaml"


def fixture(name):
    return json.loads((FIXTURES / name).read_text())


@pytest.mark.parametrize("name", ["solved_v4.json", "solved_v5.json"])
def test_solved_v4_and_v5_archives_are_exportable(name, tmp_path):
    path = tmp_path / name
    path.write_text(json.dumps(fixture(name)))
    rvec, tvec = load_solver_transform(path)
    assert rvec.tolist() == [0.1, -0.2, 0.3]
    assert tvec.tolist() == [1.25, -0.5, 2.75]


def test_paired_v4_v5_archives_export_identical_six_values(tmp_path):
    entries = []
    for name in ("solved_v4.json", "solved_v5.json"):
        detections = tmp_path / name
        detections.write_text(json.dumps(fixture(name)))
        target = tmp_path / f"{name}.yaml"
        target.write_text(CALIBRATION_FIXTURE.read_text())
        rvec, tvec = load_solver_transform(detections)
        entries.append(
            patch_calibration(
                target,
                rvec=rvec,
                tvec=tvec,
                camera_frame="camera0/camera_link",
                lidar_frame="velodyne_top_base_link",
            )
        )

    assert tuple(entries[0]) == ("x", "y", "z", "roll", "pitch", "yaw")
    assert entries[0] == entries[1]


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("schema_version", 0),
        ("schema_version", True),
        ("target_id", ""),
        ("target_id", 1),
        ("revision", 0),
        ("revision", False),
        ("semantic_sha256", "f" * 63),
        ("semantic_sha256", "F" * 64),
        ("semantic_sha256", 1),
        ("board_frame_convention", ""),
        ("board_frame_convention", []),
    ],
)
def test_v5_export_rejects_malformed_identity_without_loading_a_target(
    field, value, tmp_path
):
    archive = fixture("solved_v5.json")
    archive["target_identity"][field] = value
    path = tmp_path / "bad.json"
    path.write_text(json.dumps(archive))
    with pytest.raises(ExportError, match="target_identity"):
        load_solver_transform(path)


def test_v5_export_rejects_identity_that_has_extra_fields(tmp_path):
    archive = copy.deepcopy(fixture("solved_v5.json"))
    archive["target_identity"]["extra"] = "not allowed"
    path = tmp_path / "bad.json"
    path.write_text(json.dumps(archive))
    with pytest.raises(ExportError, match="target_identity"):
        load_solver_transform(path)


def test_v5_export_rejects_identity_that_is_missing_a_field(tmp_path):
    archive = fixture("solved_v5.json")
    del archive["target_identity"]["revision"]
    path = tmp_path / "bad.json"
    path.write_text(json.dumps(archive))
    with pytest.raises(ExportError, match="target_identity"):
        load_solver_transform(path)


@pytest.mark.parametrize("identity", [None, [], "not an object"])
def test_v5_export_rejects_identity_that_is_not_an_object(identity, tmp_path):
    archive = fixture("solved_v5.json")
    archive["target_identity"] = identity
    path = tmp_path / "bad.json"
    path.write_text(json.dumps(archive))
    with pytest.raises(ExportError, match="target_identity"):
        load_solver_transform(path)


@pytest.mark.parametrize("version", [1, 2, 3, 6, 99])
def test_export_rejects_unsupported_past_and_future_versions(version, tmp_path):
    archive = fixture("solved_v5.json")
    archive["version"] = version
    path = tmp_path / "bad-version.json"
    path.write_text(json.dumps(archive))
    with pytest.raises(ExportError, match="expected 4 or 5"):
        load_solver_transform(path)


@pytest.mark.parametrize("version", [True, False, 4.0, 5.0, "5"])
def test_export_rejects_versions_that_are_not_literal_integers(version, tmp_path):
    archive = fixture("solved_v5.json")
    archive["version"] = version
    path = tmp_path / "bad-version.json"
    path.write_text(json.dumps(archive))
    with pytest.raises(ExportError, match="version"):
        load_solver_transform(path)


def test_v5_export_rejects_archive_identity_frame_conflict(tmp_path):
    archive = fixture("solved_v5.json")
    archive["target_identity"]["board_frame_convention"] = "stale_frame_v0"
    path = tmp_path / "conflicting-frame.json"
    path.write_text(json.dumps(archive))
    with pytest.raises(ExportError, match="conflicts"):
        load_solver_transform(path)


def test_export_requires_the_exact_board_frame_convention(tmp_path):
    archive = fixture("solved_v5.json")
    archive["board_frame_convention"] = " corner_aligned_plate_center_v1 "
    archive["target_identity"]["board_frame_convention"] = archive[
        "board_frame_convention"
    ]
    path = tmp_path / "whitespace-frame.json"
    path.write_text(json.dumps(archive))
    with pytest.raises(ExportError, match="board-frame convention"):
        load_solver_transform(path)
