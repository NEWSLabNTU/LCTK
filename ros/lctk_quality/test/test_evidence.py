"""Contract tests for deterministic W7-A evidence sidecars and reports."""

from __future__ import annotations

import hashlib
import json

import pytest
from lctk_quality.evidence import (
    ArtifactRef,
    ArucoObservation,
    BagFingerprint,
    EvidenceCollector,
    EvidenceInterval,
    EvidenceManifest,
    EvidenceSample,
    EvidenceSchemaError,
    PoseRecord,
    RejectionReason,
    TargetIdentityRecord,
    labels_at,
    sha256_file,
)

IDENTITY = TargetIdentityRecord(
    schema_version=1,
    target_id="solid_600_aruco_1",
    revision=1,
    semantic_sha256="a" * 64,
    board_frame_convention="corner_aligned_plate_center_v1",
)


def manifest(*, provenance="test_only"):
    return EvidenceManifest(
        bag=BagFingerprint(
            sha256="b" * 64,
            size_bytes=1234,
            storage_id="sqlite3",
            relative_path="bags/solid.db3",
        ),
        target_identity=IDENTITY,
        sensor="velodyne_top",
        preset="solid_600/velodyne",
        topics={
            "pointcloud": "/sensing/lidar/top/points_raw",
            "board_detection": "/sensing/lidar/top/calibration_board_detections",
            "target_identity": "/sensing/lidar/top/target_identity",
            "aruco_detection": "/camera/aruco_detections",
            "solver_status": "/calibration/get_buffer_status",
            "overlay": "/camera/image_with_detections",
        },
        intervals=(
            EvidenceInterval("visible", 100, 500, "moving-visible"),
            EvidenceInterval("stationary", 200, 300, "static-hold"),
            EvidenceInterval("absent", 600, 800, "clutter"),
        ),
        provenance=provenance,
    )


def pose(value=1.0, covariance=True):
    return PoseRecord(
        position=(value, 2.0, 3.0),
        orientation=(0.0, 0.0, 0.0, 1.0),
        covariance=(0.01,) * 36 if covariance else None,
    )


def accepted(timestamp, *, value=1.0, artifacts=()):
    return EvidenceSample(
        timestamp_ns=timestamp,
        accepted=True,
        target_identity=IDENTITY,
        pose=pose(value),
        alignment_dot=0.95,
        quadrant=0,
        aruco_observations=(
            ArucoObservation(
                marker_id=1,
                corners=((10.0, 20.0), (30.0, 20.0), (30.0, 40.0), (10.0, 40.0)),
                score=0.99,
            ),
        ),
        solver_outputs={"mode": "continuous", "has_pose": True},
        artifact_ids=tuple(artifacts),
    )


def rejected(timestamp, code="insufficient_outer_edge_evidence"):
    return EvidenceSample(
        timestamp_ns=timestamp,
        accepted=False,
        rejection=RejectionReason(
            code=code,
            detail="three edge bins observed",
            evidence={"covered_edge_count": 2, "required_edges": 3},
        ),
    )


def test_interval_selection_is_half_open_and_supports_stationary_subset():
    intervals = (
        EvidenceInterval("stationary", 20, 30),
        EvidenceInterval("visible", 10, 40),
    )

    assert labels_at(9, intervals) == ()
    assert labels_at(10, intervals) == ("visible",)
    assert labels_at(20, intervals) == ("visible", "stationary")
    assert labels_at(30, intervals) == ("visible",)
    assert labels_at(40, intervals) == ()


def test_report_is_sorted_and_counts_each_label_denominator():
    artifacts = (
        ArtifactRef("overlay-20", "overlay", "overlays/frame-20.png", "c" * 64, 20),
    )
    report = EvidenceCollector(manifest()).collect(
        [
            rejected(650),
            accepted(250, artifacts=("overlay-20",)),
            accepted(150),
            accepted(5),
        ],
        artifacts,
    )

    assert report.selected_timestamps_ns == (150, 250, 650)
    assert report.summary["input_frame_count"] == 4
    assert report.summary["selected_frame_count"] == 3
    assert report.summary["unlabelled_frame_count"] == 1
    assert report.summary["accepted_frame_count"] == 2
    assert report.summary["rejected_frame_count"] == 1
    assert report.summary["accepted_pose_count"] == 2
    assert report.summary["accepted_covariance_count"] == 2
    assert report.denominators == {
        "visible": {"frames": 2, "accepted": 2, "rejected": 0},
        "absent": {"frames": 1, "accepted": 0, "rejected": 1},
        "stationary": {"frames": 1, "accepted": 1, "rejected": 0},
    }
    assert report.frames[1]["labels"] == ["visible", "stationary"]
    assert report.frames[2]["rejection"]["code"] == "insufficient_outer_edge_evidence"
    assert report.artifacts[0].artifact_id == "overlay-20"


def test_repeated_collection_has_identical_json_and_timestamp_set(tmp_path):
    samples_path = tmp_path / "samples.jsonl"
    lines = [
        json.dumps(rejected(650).to_dict(), sort_keys=True),
        json.dumps(accepted(250).to_dict(), sort_keys=True),
        json.dumps(accepted(150).to_dict(), sort_keys=True),
    ]
    samples_path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    collector = EvidenceCollector(manifest(provenance="test_only"))
    first = collector.collect_jsonl(samples_path)
    second = collector.collect_jsonl(samples_path)

    assert (
        first.selected_timestamps_ns == second.selected_timestamps_ns == (150, 250, 650)
    )
    assert first.to_json() == second.to_json()
    assert first.summary["synthetic_test_only"] is True
    assert first.provenance == "test_only"
    assert first.__class__.from_json(first.to_json()).to_json() == first.to_json()

    report_path = tmp_path / "report.json"
    first.write(report_path)
    assert report_path.read_text(encoding="utf-8") == first.to_json()


def test_accepted_identity_is_exactly_bound_to_manifest():
    wrong = TargetIdentityRecord(
        schema_version=1,
        target_id="hollow_1000_aruco_4",
        revision=1,
        semantic_sha256="d" * 64,
        board_frame_convention=IDENTITY.board_frame_convention,
    )
    sample = EvidenceSample(
        timestamp_ns=150,
        accepted=True,
        target_identity=wrong,
        pose=pose(),
    )

    with pytest.raises(EvidenceSchemaError, match="does not match manifest"):
        EvidenceCollector(manifest()).collect([sample])


@pytest.mark.parametrize(
    ("kwargs", "message"),
    [
        ({"accepted": True}, "accepted sample needs target identity"),
        (
            {"accepted": True, "target_identity": IDENTITY},
            "accepted sample needs pose",
        ),
        ({"accepted": False}, "rejected sample needs structured reason"),
        (
            {
                "accepted": False,
                "rejection": RejectionReason("x"),
                "alignment_dot": 2.0,
            },
            r"must be in \[-1, 1\]",
        ),
    ],
)
def test_invalid_sample_contracts_are_rejected(kwargs, message):
    with pytest.raises(EvidenceSchemaError, match=message):
        EvidenceSample(timestamp_ns=1, **kwargs)


def test_rejected_sample_requires_structured_reason():
    with pytest.raises(
        EvidenceSchemaError, match="rejected sample needs structured reason"
    ):
        EvidenceSample(timestamp_ns=1, accepted=False)


def test_artifact_paths_are_relative_and_artifact_ids_must_be_declared():
    with pytest.raises(EvidenceSchemaError, match="relative path"):
        ArtifactRef("overlay", "overlay", "/tmp/overlay.png")

    with pytest.raises(EvidenceSchemaError, match="unknown artifact_id"):
        EvidenceCollector(manifest()).collect([accepted(150, artifacts=("missing",))])


def test_manifest_json_round_trip_and_hash_ignore_mapping_order(tmp_path):
    original = manifest()
    loaded = EvidenceManifest.from_json(original.to_json())
    assert loaded == original
    assert loaded.sha256() == original.sha256()

    path = tmp_path / "manifest.json"
    original.write(path)
    assert EvidenceManifest.load(path) == original


def test_report_reader_rejects_timestamp_or_denominator_drift():
    report = EvidenceCollector(manifest()).collect([accepted(150), rejected(650)])
    encoded = report.to_dict()

    encoded["frames"][0]["timestamp_ns"] = 151
    with pytest.raises(EvidenceSchemaError, match="selected_timestamps_ns"):
        report.__class__.from_mapping(encoded)

    encoded = report.to_dict()
    encoded["denominators"]["visible"]["accepted"] = 0
    with pytest.raises(EvidenceSchemaError, match=r"accepted \+ rejected"):
        report.__class__.from_mapping(encoded)


def test_bag_fingerprint_from_file_records_checksum_and_size(tmp_path):
    bag = tmp_path / "bag.db3"
    bag.write_bytes(b"field-bag")
    fingerprint = BagFingerprint.from_file(
        bag,
        storage_id="sqlite3",
        relative_path="bags/bag.db3",
    )
    assert fingerprint.sha256 == hashlib.sha256(b"field-bag").hexdigest()
    assert fingerprint.sha256 == sha256_file(bag)
    assert fingerprint.size_bytes == len(b"field-bag")


def test_duplicate_timestamps_are_rejected_after_merge():
    with pytest.raises(EvidenceSchemaError, match="must be unique"):
        EvidenceCollector(manifest()).collect([accepted(150), rejected(150)])


def test_bag_relative_path_must_be_relative():
    with pytest.raises(EvidenceSchemaError, match="relative path"):
        BagFingerprint(sha256="b" * 64, relative_path="/abs/bag.db3")


def test_interval_rejects_unknown_label():
    with pytest.raises(EvidenceSchemaError, match="expected one of"):
        EvidenceInterval("bogus", 0, 100)


@pytest.mark.parametrize("value", [float("nan"), float("inf"), float("-inf")])
def test_pose_rejects_nan_and_infinite_values(value):
    with pytest.raises(EvidenceSchemaError, match="finite number"):
        PoseRecord(position=(value, 0.0, 0.0), orientation=(0.0, 0.0, 0.0, 1.0))


@pytest.mark.parametrize("value", [float("nan"), float("inf")])
def test_rejection_evidence_rejects_nan_and_infinite_values(value):
    with pytest.raises(EvidenceSchemaError, match="NaN or infinity"):
        RejectionReason("code", evidence={"score": value})


def test_field_provenance_is_not_reported_as_synthetic():
    # Exercises the "field" branch of the schema explicitly; every other
    # fixture in this file is synthetic and must stay marked "test_only".
    report = EvidenceCollector(manifest(provenance="field")).collect([accepted(150)])
    assert report.provenance == "field"
    assert report.summary["synthetic_test_only"] is False
