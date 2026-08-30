"""End-to-end CLI contract for the version 4 to 5 detection archive migration.

W4-Ec adds an explicit ``--target-config`` hop beside the existing version 3 to 4
``--assume-convention`` hop. Each hop asserts a different operator fact -- frame
convention, then Target Identity -- so the two never collapse into one command,
and every rejection must leave the filesystem exactly as it found it (no partial
or stale output file).
"""

import json
from pathlib import Path

import pytest
from lidar_to_camera_solver.archive_contract import archive_restore_error
from lidar_to_camera_solver.board_geometry import BOARD_FRAME_CONVENTION
from lidar_to_camera_solver.migrate_detections import main

FIXTURES = Path(__file__).resolve().parents[3] / "fixtures" / "detection_archives"
TARGETS = Path(__file__).resolve().parents[2] / "lctk_launch" / "config" / "targets"
SOLID_TARGET_CONFIG = TARGETS / "solid_600_aruco_1_v1.json5"
HOLLOW_TARGET_CONFIG = TARGETS / "hollow_1000_aruco_4_v1.json5"


def fixture(name):
    return json.loads((FIXTURES / name).read_text())


def _v4_archive_observing(marker_id: int) -> dict:
    """Build a minimal, self-contained version-4 archive observing one marker ID.

    Not a fixture under ``fixtures/detection_archives/`` -- the shared paired
    fixtures there carry no ArUco detections at all, so the marker-ID mismatch
    case needs its own hand-built input.
    """
    return {
        "version": 4,
        "board_frame_convention": BOARD_FRAME_CONVENTION,
        "num_detections": 1,
        "detections": [
            {
                "aruco": {
                    "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": "cam"},
                    "detections": [
                        {
                            "id": f"aruco_{marker_id}",
                            "bbox": {
                                "center": {"x": 0.0, "y": 0.0},
                                "size_x": 1.0,
                                "size_y": 1.0,
                            },
                            "results": [],
                        }
                    ],
                },
                "board": {
                    "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": "lidar"},
                    "detections": [],
                },
            }
        ],
    }


def _write(tmp_path: Path, name: str, data: dict) -> Path:
    path = tmp_path / name
    path.write_text(json.dumps(data))
    return path


def test_v4_to_v5_on_the_shared_fixture_restores_against_the_selected_target(
    tmp_path,
):
    output = tmp_path / "out.json"
    exit_code = main(
        [
            "--input",
            str(FIXTURES / "solved_v4.json"),
            "--output",
            str(output),
            "--target-config",
            str(SOLID_TARGET_CONFIG),
        ]
    )

    assert exit_code == 0
    migrated = json.loads(output.read_text())
    assert migrated["version"] == 5
    assert archive_restore_error(migrated, migrated["target_identity"]) is None, (
        "the migrated archive must restore against the identity it was just bound to"
    )


def test_migrated_output_equals_source_once_identity_and_version_are_removed(
    tmp_path,
):
    """Wave acceptance: source archive contents deep-equal after removing the
    added identity/version fields. Checked structurally, as a whole-dict
    comparison, not field by field."""
    output = tmp_path / "out.json"
    main(
        [
            "--input",
            str(FIXTURES / "solved_v4.json"),
            "--output",
            str(output),
            "--target-config",
            str(SOLID_TARGET_CONFIG),
        ]
    )

    migrated = json.loads(output.read_text())
    del migrated["target_identity"]
    migrated["version"] = 4

    assert migrated == fixture("solved_v4.json")


def test_marker_id_mismatch_rejects_names_the_id_and_writes_no_output(tmp_path):
    archive = _v4_archive_observing(marker_id=999)
    input_path = _write(tmp_path, "in.json", archive)
    output = tmp_path / "out.json"

    exit_code = main(
        [
            "--input",
            str(input_path),
            "--output",
            str(output),
            "--target-config",
            str(SOLID_TARGET_CONFIG),
        ]
    )

    assert exit_code != 0
    assert not output.exists()


def test_marker_id_present_on_target_migrates_successfully(tmp_path):
    archive = _v4_archive_observing(marker_id=24)  # solid_600_aruco_1 defines ArUco id 24
    input_path = _write(tmp_path, "in.json", archive)
    output = tmp_path / "out.json"

    exit_code = main(
        [
            "--input",
            str(input_path),
            "--output",
            str(output),
            "--target-config",
            str(SOLID_TARGET_CONFIG),
        ]
    )

    assert exit_code == 0
    assert output.exists()


def test_v3_input_with_target_config_rejects_with_guidance_and_writes_no_output(
    tmp_path,
):
    input_path = _write(
        tmp_path, "in.json", {"version": 3, "num_detections": 0, "detections": []}
    )
    output = tmp_path / "out.json"

    exit_code = main(
        [
            "--input",
            str(input_path),
            "--output",
            str(output),
            "--target-config",
            str(SOLID_TARGET_CONFIG),
        ]
    )

    assert exit_code != 0
    assert not output.exists()


def test_v4_input_with_assume_convention_rejects_with_guidance_and_writes_no_output(
    tmp_path,
):
    input_path = _write(tmp_path, "in.json", fixture("solved_v4.json"))
    output = tmp_path / "out.json"

    exit_code = main(
        [
            "--input",
            str(input_path),
            "--output",
            str(output),
            "--assume-convention",
            BOARD_FRAME_CONVENTION,
        ]
    )

    assert exit_code != 0
    assert not output.exists()


def test_both_migration_flags_together_refuse_to_chain_v3_to_v5_in_one_run(
    tmp_path,
):
    """Each hop asserts a different operator fact; version 4 is never reinterpreted
    implicitly, so one command may never claim both at once."""
    input_path = _write(
        tmp_path, "in.json", {"version": 3, "num_detections": 0, "detections": []}
    )
    output = tmp_path / "out.json"

    exit_code = main(
        [
            "--input",
            str(input_path),
            "--output",
            str(output),
            "--assume-convention",
            BOARD_FRAME_CONVENTION,
            "--target-config",
            str(SOLID_TARGET_CONFIG),
        ]
    )

    assert exit_code != 0
    assert not output.exists()


def test_a_version_5_input_rejects_as_already_migrated_and_writes_no_output(
    tmp_path,
):
    input_path = _write(tmp_path, "in.json", fixture("solved_v5.json"))
    output = tmp_path / "out.json"

    exit_code = main(
        [
            "--input",
            str(input_path),
            "--output",
            str(output),
            "--target-config",
            str(SOLID_TARGET_CONFIG),
        ]
    )

    assert exit_code != 0
    assert not output.exists()


@pytest.mark.parametrize("version", [0, 1, 2, 6, "5", None])
def test_unsupported_versions_reject_and_write_no_output(tmp_path, version):
    input_path = _write(
        tmp_path, "in.json", {"version": version, "num_detections": 0, "detections": []}
    )
    output = tmp_path / "out.json"

    exit_code = main(
        [
            "--input",
            str(input_path),
            "--output",
            str(output),
            "--target-config",
            str(SOLID_TARGET_CONFIG),
        ]
    )

    assert exit_code != 0
    assert not output.exists()


def test_missing_target_config_for_a_v4_input_rejects_and_writes_no_output(tmp_path):
    input_path = _write(tmp_path, "in.json", fixture("solved_v4.json"))
    output = tmp_path / "out.json"

    exit_code = main(["--input", str(input_path), "--output", str(output)])

    assert exit_code != 0
    assert not output.exists()


def test_missing_assume_convention_for_a_v3_input_rejects_and_writes_no_output(
    tmp_path,
):
    input_path = _write(
        tmp_path, "in.json", {"version": 3, "num_detections": 0, "detections": []}
    )
    output = tmp_path / "out.json"

    exit_code = main(["--input", str(input_path), "--output", str(output)])

    assert exit_code != 0
    assert not output.exists()


def test_hollow_target_config_also_migrates_the_solid_fixture_structurally(
    tmp_path,
):
    """The fixture itself carries the solid target's identity; binding a
    different, structurally valid target is still a legal (if operator-wrong)
    invocation as far as this command's ID check can tell -- it only proves ID
    compatibility, never physical provenance. Both markers happen to be absent
    from the fixture's empty detections, so this exercises the vacuous-pass
    path against the *other* target manifest."""
    output = tmp_path / "out.json"
    exit_code = main(
        [
            "--input",
            str(FIXTURES / "solved_v4.json"),
            "--output",
            str(output),
            "--target-config",
            str(HOLLOW_TARGET_CONFIG),
        ]
    )

    assert exit_code == 0
    migrated = json.loads(output.read_text())
    assert migrated["target_identity"]["target_id"] == "hollow_1000_aruco_4"


@pytest.mark.parametrize("malformed_id", ["not-a-marker", "aruco_x", None, 3.5, [1]])
def test_a_malformed_detection_id_rejects_and_writes_no_output(tmp_path, malformed_id):
    """A detection id this command cannot parse must reject rather than vanish
    from the marker-ID check -- silently dropping it would let a wrong target
    selection pass vacuously, defeating the one check this migration performs."""
    archive = _v4_archive_observing(marker_id=24)
    archive["detections"][0]["aruco"]["detections"][0]["id"] = malformed_id
    input_path = _write(tmp_path, "in.json", archive)
    output = tmp_path / "out.json"

    exit_code = main(
        [
            "--input",
            str(input_path),
            "--output",
            str(output),
            "--target-config",
            str(SOLID_TARGET_CONFIG),
        ]
    )

    assert exit_code != 0
    assert not output.exists()


def test_an_unwritable_output_path_exits_nonzero_with_a_message_for_v3_to_v4(
    tmp_path,
):
    """An OSError from the atomic write (here: a nonexistent output directory)
    must surface as the same "message on stderr, exit 1" contract as every
    other failure path, not as a raw traceback, and must not leave partial
    output behind."""
    input_path = _write(
        tmp_path, "in.json", {"version": 3, "num_detections": 0, "detections": []}
    )
    output = tmp_path / "missing_dir" / "out.json"

    exit_code = main(
        [
            "--input",
            str(input_path),
            "--output",
            str(output),
            "--assume-convention",
            BOARD_FRAME_CONVENTION,
        ]
    )

    assert exit_code != 0
    assert not output.exists()


def test_an_unwritable_output_path_exits_nonzero_with_a_message_for_v4_to_v5(
    tmp_path,
):
    input_path = _write(tmp_path, "in.json", fixture("solved_v4.json"))
    output = tmp_path / "missing_dir" / "out.json"

    exit_code = main(
        [
            "--input",
            str(input_path),
            "--output",
            str(output),
            "--target-config",
            str(SOLID_TARGET_CONFIG),
        ]
    )

    assert exit_code != 0
    assert not output.exists()
