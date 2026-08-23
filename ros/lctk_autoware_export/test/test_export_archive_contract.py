"""Exporter archive compatibility stays independent of target-manifest loading."""

import copy
import json
from pathlib import Path

import pytest
from lctk_autoware_export.export import ExportError, load_solver_transform

FIXTURES = Path(__file__).resolve().parents[3] / "fixtures" / "detection_archives"


def fixture(name):
    return json.loads((FIXTURES / name).read_text())


@pytest.mark.parametrize("name", ["solved_v4.json", "solved_v5.json"])
def test_solved_v4_and_v5_archives_are_exportable(name, tmp_path):
    path = tmp_path / name
    path.write_text(json.dumps(fixture(name)))
    rvec, tvec = load_solver_transform(path)
    assert rvec.tolist() == [0.1, -0.2, 0.3]
    assert tvec.tolist() == [1.25, -0.5, 2.75]


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("schema_version", 0),
        ("target_id", ""),
        ("revision", 0),
        ("semantic_sha256", "f" * 63),
        ("semantic_sha256", "F" * 64),
        ("board_frame_convention", ""),
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
