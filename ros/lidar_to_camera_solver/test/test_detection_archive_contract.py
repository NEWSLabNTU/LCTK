"""The v6 Detection Archive restore contract."""

import copy
import json
from pathlib import Path

import pytest
from lidar_to_camera_solver.archive_contract import archive_restore_error

FIXTURES = Path(__file__).resolve().parents[3] / "fixtures" / "detection_archives"


def fixture(name):
    return json.loads((FIXTURES / name).read_text())


def v6_archive():
    archive = fixture("solved_v5.json")
    archive.update(
        {
            "version": 6,
            "camera_projection": {
                "model": "undistorted_pixels_using_k",
                "frame_id": "camera_optical",
                "width": 640,
                "height": 480,
                "k": [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0],
            },
            "num_detections": 0,
            "detections": [],
        }
    )
    return archive


def test_v6_restores_only_against_the_exact_local_identity():
    archive = v6_archive()
    assert archive_restore_error(archive, archive["target_identity"]) is None

    different = copy.deepcopy(archive["target_identity"])
    different["revision"] = 2
    assert "does not exactly match" in archive_restore_error(archive, different)


@pytest.mark.parametrize("version", [4, 5])
def test_legacy_archive_versions_are_unsupported_without_migration(version):
    archive = v6_archive()
    archive["version"] = version

    error = archive_restore_error(archive, archive["target_identity"])

    assert "unsupported past version" in error
    assert "integer 6" in error
    assert "migrate_detections" not in error


def test_restore_rejects_a_future_version():
    archive = v6_archive()
    archive["version"] = 7

    error = archive_restore_error(archive, archive["target_identity"])

    assert "unsupported future version" in error
    assert "integer 6" in error


@pytest.mark.parametrize("version", [True, False, 6.0, "6"])
def test_restore_rejects_versions_that_are_not_literal_integers(version):
    archive = v6_archive()
    archive["version"] = version

    error = archive_restore_error(archive, archive["target_identity"])

    assert "expected integer 6" in error


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("schema_version", 0),
        ("schema_version", True),
        ("target_id", ""),
        ("revision", 0),
        ("semantic_sha256", "A" * 64),
        ("semantic_sha256", "a" * 63),
        ("board_frame_convention", ""),
    ],
)
def test_v6_identity_requires_all_structural_fields(field, value):
    archive = v6_archive()
    archive["target_identity"][field] = value

    assert archive_restore_error(archive, fixture("solved_v5.json")["target_identity"])


def test_v6_identity_rejects_missing_or_unknown_fields():
    archive = v6_archive()
    del archive["target_identity"]["revision"]
    assert archive_restore_error(archive, fixture("solved_v5.json")["target_identity"])

    archive = v6_archive()
    archive["target_identity"]["unexpected"] = "value"
    assert archive_restore_error(archive, fixture("solved_v5.json")["target_identity"])
