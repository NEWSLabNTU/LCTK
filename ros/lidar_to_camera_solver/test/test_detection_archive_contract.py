"""Pure v4/v5 archive compatibility contract."""

import copy
import json
from pathlib import Path

import pytest
from lidar_to_camera_solver.archive_contract import archive_restore_error
from lidar_to_camera_solver.detection_format import migrate_v3_to_v4

FIXTURES = Path(__file__).resolve().parents[3] / "fixtures" / "detection_archives"


def fixture(name):
    return json.loads((FIXTURES / name).read_text())


def test_paired_archives_keep_the_same_solved_transform():
    assert (
        fixture("solved_v4.json")["transform"] == fixture("solved_v5.json")["transform"]
    )


def test_paired_archives_have_equal_content_except_identity_and_version():
    v4 = fixture("solved_v4.json")
    v5 = fixture("solved_v5.json")
    v4.pop("version")
    v5.pop("version")
    v5.pop("target_identity")
    assert v4 == v5


def test_v5_restores_only_against_the_exact_local_identity():
    archive = fixture("solved_v5.json")
    assert archive_restore_error(archive, archive["target_identity"]) is None

    different = copy.deepcopy(archive["target_identity"])
    different["revision"] = 2
    assert "does not exactly match" in archive_restore_error(archive, different)


@pytest.mark.parametrize("convention", [None, "", 17])
def test_v5_restore_requires_a_nonempty_archive_frame_convention(convention):
    archive = fixture("solved_v5.json")
    archive["board_frame_convention"] = convention
    assert "board_frame_convention" in archive_restore_error(
        archive, fixture("solved_v5.json")["target_identity"]
    )


def test_v5_restore_rejects_convention_conflicts_with_archive_identity():
    archive = fixture("solved_v5.json")
    archive["board_frame_convention"] = "stale_frame_v0"
    error = archive_restore_error(archive, fixture("solved_v5.json")["target_identity"])
    assert "conflicts with its Target Identity" in error


def test_v5_restore_rejects_convention_conflicts_with_local_identity():
    archive = fixture("solved_v5.json")
    local = copy.deepcopy(archive["target_identity"])
    local["board_frame_convention"] = "other_valid_local_frame"
    error = archive_restore_error(archive, local)
    assert "does not match the local target" in error


def test_v4_is_never_restorable_after_target_selection_is_required():
    v5 = fixture("solved_v5.json")
    error = archive_restore_error(fixture("solved_v4.json"), v5["target_identity"])
    assert "cannot be restored" in error
    assert "migrate_detections" in error
    assert "--target-config" in error


@pytest.mark.parametrize("version", [True, False, 4.0, 5.0, "5"])
def test_restore_rejects_versions_that_are_not_literal_integers(version):
    archive = fixture("solved_v5.json")
    archive["version"] = version
    error = archive_restore_error(archive, fixture("solved_v5.json")["target_identity"])
    assert "expected integer 5" in error


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
def test_v5_identity_requires_all_structural_fields(field, value):
    archive = fixture("solved_v5.json")
    archive["target_identity"][field] = value
    assert (
        archive_restore_error(archive, fixture("solved_v5.json")["target_identity"])
        is not None
    )


def test_v5_identity_rejects_missing_or_unknown_fields():
    archive = fixture("solved_v5.json")
    del archive["target_identity"]["revision"]
    assert (
        archive_restore_error(archive, fixture("solved_v5.json")["target_identity"])
        is not None
    )

    archive = fixture("solved_v5.json")
    archive["target_identity"]["unexpected"] = "value"
    assert (
        archive_restore_error(archive, fixture("solved_v5.json")["target_identity"])
        is not None
    )


def test_v3_migration_stays_version_four_if_current_format_later_changes():
    migrated = migrate_v3_to_v4(
        {"version": 3, "detections": []},
        convention="corner_aligned_plate_center_v1",
    )
    assert migrated["version"] == 4
