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

CALIBRATION_FIXTURE = Path(__file__).parent / "fixtures" / "sensor_kit_calibration.yaml"


def solved_v6_archive():
    return {
        "version": 6,
        "board_frame_convention": "corner_aligned_plate_center_v1",
        "target_identity": {
            "schema_version": 1,
            "target_id": "solid_600_aruco_1",
            "revision": 1,
            "semantic_sha256": (
                "9cf600d684a91ab3df6fdf549d71460fc6fc5321a037d5f61393bbd9db5b04fb"
            ),
            "board_frame_convention": "corner_aligned_plate_center_v1",
        },
        "transform": {"rvec": [0.1, -0.2, 0.3], "tvec": [1.25, -0.5, 2.75]},
        "camera_projection": {
            "model": "undistorted_pixels_using_k",
            "frame_id": "camera_optical",
            "width": 1920,
            "height": 1080,
            "k": [1000.0, 0.0, 960.0, 0.0, 1000.0, 540.0, 0.0, 0.0, 1.0],
        },
        "num_detections": 1,
        "detections": [
            {
                "aruco": {
                    "header": {
                        "stamp": {"sec": 1, "nanosec": 2},
                        "frame_id": "camera_optical",
                    },
                    "detections": [],
                },
                "board": {
                    "header": {
                        "stamp": {"sec": 1, "nanosec": 2},
                        "frame_id": "velodyne",
                    },
                    "detections": [],
                },
            }
        ],
    }


def test_solved_v6_archive_exports_the_saved_transform_without_relabeling(tmp_path):
    path = tmp_path / "solved_v6.json"
    path.write_text(json.dumps(solved_v6_archive()))

    rvec, tvec = load_solver_transform(path)

    assert rvec.tolist() == [0.1, -0.2, 0.3]
    assert tvec.tolist() == [1.25, -0.5, 2.75]


@pytest.mark.parametrize(
    ("field", "value"),
    [("rvec", [0.1, 0.2]), ("tvec", [1.0, 2.0, 3.0, 4.0])],
)
def test_load_solver_transform_rejects_a_vector_with_the_wrong_length(
    field, value, tmp_path
):
    archive = solved_v6_archive()
    archive["transform"][field] = value
    path = tmp_path / f"malformed-{field}.json"
    path.write_text(json.dumps(archive))

    with pytest.raises(ExportError, match=f"transform.{field}"):
        load_solver_transform(path)


def test_load_solver_transform_rejects_a_nonnumeric_vector_component(tmp_path):
    archive = solved_v6_archive()
    archive["transform"]["tvec"] = [1.0, "2.0", 3.0]
    path = tmp_path / "nonnumeric-translation-vector.json"
    path.write_text(json.dumps(archive))

    with pytest.raises(ExportError, match="transform.tvec"):
        load_solver_transform(path)


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("rvec", [float("nan"), 0.0, 0.0]),
        ("tvec", [0.0, float("inf"), 0.0]),
    ],
)
def test_load_solver_transform_rejects_nonfinite_vectors(field, value, tmp_path):
    archive = solved_v6_archive()
    archive["transform"][field] = value
    path = tmp_path / f"nonfinite-{field}.json"
    path.write_text(json.dumps(archive))

    with pytest.raises(ExportError, match=f"transform.{field}"):
        load_solver_transform(path)


def test_v6_export_rejects_an_unsupported_camera_projection_model(tmp_path):
    archive = solved_v6_archive()
    archive["camera_projection"]["model"] = "raw_distorted_pixels"
    path = tmp_path / "unsupported-projection.json"
    path.write_text(json.dumps(archive))

    with pytest.raises(ExportError, match="camera_projection.model"):
        load_solver_transform(path)


def test_v6_export_rejects_projection_numbers_outside_finite_float_range(tmp_path):
    archive = solved_v6_archive()
    archive["camera_projection"]["k"][0] = 10**400
    path = tmp_path / "unrepresentable-projection.json"
    path.write_text(json.dumps(archive))

    with pytest.raises(ExportError, match="camera_projection.k"):
        load_solver_transform(path)


def test_v6_export_rejects_camera_header_that_disagrees_with_projection(tmp_path):
    archive = solved_v6_archive()
    archive["detections"][0]["aruco"]["header"]["frame_id"] = "stale_camera_frame"
    path = tmp_path / "stale-camera-frame.json"
    path.write_text(json.dumps(archive))

    with pytest.raises(ExportError, match="Camera detection frame"):
        load_solver_transform(path)


def test_v6_export_rejects_mixed_lidar_header_frames(tmp_path):
    archive = solved_v6_archive()
    second = copy.deepcopy(archive["detections"][0])
    second["board"]["header"]["frame_id"] = "another_lidar"
    archive["detections"].append(second)
    archive["num_detections"] = 2
    path = tmp_path / "mixed-lidar-frames.json"
    path.write_text(json.dumps(archive))

    with pytest.raises(ExportError, match="mixes LiDAR frames"):
        load_solver_transform(path)


def test_v6_export_rejects_count_that_disagrees_with_saved_pairs(tmp_path):
    archive = solved_v6_archive()
    archive["num_detections"] = 2
    path = tmp_path / "count-mismatch.json"
    path.write_text(json.dumps(archive))

    with pytest.raises(ExportError, match="count mismatch"):
        load_solver_transform(path)


def test_paired_v6_archives_export_identical_six_values(tmp_path):
    entries = []
    for name in ("first.json", "second.json"):
        detections = tmp_path / name
        detections.write_text(json.dumps(solved_v6_archive()))
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
def test_v6_export_rejects_malformed_identity_without_loading_a_target(
    field, value, tmp_path
):
    archive = solved_v6_archive()
    archive["target_identity"][field] = value
    path = tmp_path / "bad.json"
    path.write_text(json.dumps(archive))
    with pytest.raises(ExportError, match="target_identity"):
        load_solver_transform(path)


def test_v6_export_rejects_identity_that_has_extra_fields(tmp_path):
    archive = copy.deepcopy(solved_v6_archive())
    archive["target_identity"]["extra"] = "not allowed"
    path = tmp_path / "bad.json"
    path.write_text(json.dumps(archive))
    with pytest.raises(ExportError, match="target_identity"):
        load_solver_transform(path)


def test_v6_export_rejects_identity_that_is_missing_a_field(tmp_path):
    archive = solved_v6_archive()
    del archive["target_identity"]["revision"]
    path = tmp_path / "bad.json"
    path.write_text(json.dumps(archive))
    with pytest.raises(ExportError, match="target_identity"):
        load_solver_transform(path)


@pytest.mark.parametrize("identity", [None, [], "not an object"])
def test_v6_export_rejects_identity_that_is_not_an_object(identity, tmp_path):
    archive = solved_v6_archive()
    archive["target_identity"] = identity
    path = tmp_path / "bad.json"
    path.write_text(json.dumps(archive))
    with pytest.raises(ExportError, match="target_identity"):
        load_solver_transform(path)


@pytest.mark.parametrize("version", [1, 2, 3, 4, 5, 7, 99])
def test_export_rejects_unsupported_past_and_future_versions(version, tmp_path):
    archive = solved_v6_archive()
    archive["version"] = version
    path = tmp_path / "bad-version.json"
    path.write_text(json.dumps(archive))
    with pytest.raises(ExportError, match="expected 6"):
        load_solver_transform(path)


@pytest.mark.parametrize("version", [True, False, 4.0, 5.0, "5"])
def test_export_rejects_versions_that_are_not_literal_integers(version, tmp_path):
    archive = solved_v6_archive()
    archive["version"] = version
    path = tmp_path / "bad-version.json"
    path.write_text(json.dumps(archive))
    with pytest.raises(ExportError, match="version"):
        load_solver_transform(path)


def test_v6_export_rejects_archive_identity_frame_conflict(tmp_path):
    archive = solved_v6_archive()
    archive["target_identity"]["board_frame_convention"] = "stale_frame_v0"
    path = tmp_path / "conflicting-frame.json"
    path.write_text(json.dumps(archive))
    with pytest.raises(ExportError, match="conflicts"):
        load_solver_transform(path)


def test_export_requires_the_exact_board_frame_convention(tmp_path):
    archive = solved_v6_archive()
    archive["board_frame_convention"] = " corner_aligned_plate_center_v1 "
    archive["target_identity"]["board_frame_convention"] = archive[
        "board_frame_convention"
    ]
    path = tmp_path / "whitespace-frame.json"
    path.write_text(json.dumps(archive))
    with pytest.raises(ExportError, match="board-frame convention"):
        load_solver_transform(path)
