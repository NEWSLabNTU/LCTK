"""Observable behavior of the assisted camera/cloud evidence seam."""

import cv2
import numpy as np
from lidar_to_camera_solver.evidence_store import CaptureEvidence, EvidenceStore


def make_store(**overrides):
    values = {
        "max_previews": 4,
        "jpeg_quality": 100,
        "review_evidence_seconds": 1.0,
        "stamp_tolerance_s": 0.001,
    }
    values.update(overrides)
    return EvidenceStore(**values)


def camera_matrix():
    return np.array(
        [[100.0, 0.0, 4.0], [0.0, 100.0, 3.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )


def frame(height=6, width=8):
    result = np.full((height, width, 3), 255, dtype=np.uint8)
    result[:, :, 2] = 128
    return result


def observe_frame(store, stamp=1.0, value=None):
    value = frame() if value is None else value
    height, width = value.shape[:2]
    store.observe_frame(
        stamp,
        height,
        width,
        "bgr8",
        width * 3,
        value.tobytes(),
    )


def test_capture_without_a_matching_frame_reports_missing_evidence():
    store = make_store()
    store.observe_intrinsics(camera_matrix(), np.zeros(5, dtype=np.float64))
    observe_frame(store, stamp=10.0)

    evidence = store.capture(1, 10.1, corners=[])

    assert isinstance(evidence, CaptureEvidence)
    assert evidence.preview_jpeg is None
    assert "camera frame" in evidence.missing
    assert "plane inliers" in evidence.missing


def test_matching_tolerance_is_one_millisecond_by_default():
    store = make_store()
    store.observe_intrinsics(camera_matrix(), np.zeros(5, dtype=np.float64))
    observe_frame(store, stamp=10.0)

    close = store.capture(1, 10.0009, corners=[])
    far = store.capture(2, 10.002, corners=[])

    assert close.preview_jpeg is not None
    assert far.preview_jpeg is None
    assert "camera frame" in far.missing


def test_a_frame_is_stored_at_half_resolution():
    store = make_store()
    store.observe_intrinsics(camera_matrix(), np.zeros(5, dtype=np.float64))
    observe_frame(store)

    evidence = store.capture(1, 1.0, corners=[])
    image = cv2.imdecode(
        np.frombuffer(evidence.preview_jpeg, dtype=np.uint8), cv2.IMREAD_COLOR
    )

    assert image is not None
    assert image.shape[:2] == (3, 4)


def test_cloud_is_packed_as_little_endian_float32_xyz():
    store = make_store()
    points = np.array([[1.25, -2.5, 3.75], [4.0, 5.5, -6.25]], dtype=np.float64)
    store.observe_cloud(2.0, points)

    evidence = store.capture(1, 2.0, corners=[], cloud_stamp=2.0)
    unpacked = np.frombuffer(evidence.cloud_xyz, dtype="<f4").reshape(-1, 3)

    assert np.array_equal(unpacked, points.astype("<f4"))
    assert "plane inliers" not in evidence.missing


def test_drop_removes_both_preview_and_cloud_payloads():
    store = make_store()
    store.observe_intrinsics(camera_matrix(), np.zeros(5, dtype=np.float64))
    observe_frame(store)
    store.observe_cloud(1.0, [[1.0, 2.0, 3.0]])
    evidence = store.capture(7, 1.0, corners=[], cloud_stamp=1.0)
    assert evidence.preview_jpeg is not None
    assert evidence.cloud_xyz is not None

    store.drop(7)

    assert store.get(7) is None


def test_late_cloud_for_the_exact_board_stamp_completes_a_capture():
    store = make_store()
    store.observe_intrinsics(camera_matrix(), np.zeros(5, dtype=np.float64))
    observe_frame(store, stamp=10.0)
    evidence = store.capture(7, 10.0, corners=[], cloud_stamp=10.5)
    assert evidence.cloud_xyz is None
    assert "plane inliers" in evidence.missing

    store.observe_cloud(10.5, [[7.0, 8.0, 9.0]])
    completed = store.get(7)

    assert completed is not None
    assert completed.cloud_xyz is not None
    assert np.array_equal(
        np.frombuffer(completed.cloud_xyz, dtype="<f4"),
        np.array([7.0, 8.0, 9.0], dtype="<f4"),
    )


def test_evidence_revision_advances_when_delayed_cloud_completes():
    store = make_store()
    store.observe_intrinsics(camera_matrix(), np.zeros(5, dtype=np.float64))
    observe_frame(store, stamp=10.0)
    store.capture(7, 10.0, corners=[], cloud_stamp=10.5)
    before = store.evidence_revision(7)

    store.observe_cloud(10.5, [[7.0, 8.0, 9.0]])

    assert before == 1
    assert store.evidence_revision(7) == 2


def test_evicted_and_recaptured_evidence_has_a_fresh_revision():
    store = make_store(max_previews=1)
    store.capture(1, 1.0, corners=[])
    original = store.evidence_revision(1)
    store.capture(2, 2.0, corners=[])
    assert store.snapshot([1])[1] == (None, 0)
    store.capture(1, 1.0, corners=[])
    assert store.evidence_revision(1) > original


def test_late_cloud_does_not_fill_from_a_neighboring_sweep():
    store = make_store()
    store.observe_intrinsics(camera_matrix(), np.zeros(5, dtype=np.float64))
    observe_frame(store, stamp=10.0)
    store.capture(7, 10.0, corners=[], cloud_stamp=10.5)

    store.observe_cloud(10.51, [[1.0, 2.0, 3.0]])
    pending = store.get(7)
    assert pending is not None
    assert pending.cloud_xyz is None
    assert "plane inliers" in pending.missing


def test_a_camera_stamp_rollback_invalidates_pending_evidence_and_cloud_ring():
    store = make_store()
    store.observe_intrinsics(camera_matrix(), np.zeros(5, dtype=np.float64))
    store.capture(7, 10.0, corners=[])

    observe_frame(store, stamp=11.0)
    store.observe_cloud(11.0, [[1.0, 2.0, 3.0]])
    observe_frame(store, stamp=9.0)  # Starts a fresh source timeline.
    observe_frame(store, stamp=10.0)
    store.observe_cloud(10.0, [[4.0, 5.0, 6.0]])

    evidence = store.get(7)
    assert evidence is not None
    assert evidence.preview_jpeg is None
    assert evidence.cloud_xyz is None


def test_a_cloud_stamp_rollback_invalidates_pending_evidence_and_frame_ring():
    store = make_store()
    store.observe_intrinsics(camera_matrix(), np.zeros(5, dtype=np.float64))
    observe_frame(store, stamp=10.0)
    store.capture(7, 10.0, corners=[], cloud_stamp=20.0)

    store.observe_cloud(21.0, [[1.0, 2.0, 3.0]])
    store.observe_cloud(19.0, [[4.0, 5.0, 6.0]])  # Fresh cloud timeline.
    observe_frame(store, stamp=10.0)
    store.observe_cloud(20.0, [[7.0, 8.0, 9.0]])

    evidence = store.get(7)
    assert evidence is not None
    assert evidence.preview_jpeg is not None
    assert evidence.cloud_xyz is None


def test_malformed_frame_does_not_reuse_a_previous_valid_frame():
    store = make_store()
    store.observe_intrinsics(camera_matrix(), np.zeros(5, dtype=np.float64))
    observe_frame(store, stamp=1.0)
    store.observe_frame(1.01, 6, 8, "bgr8", 24, b"too short")

    evidence = store.capture(1, 1.01, corners=[])

    assert evidence.preview_jpeg is None
    assert "camera frame" in evidence.missing


def test_malformed_intrinsics_does_not_reuse_previous_valid_intrinsics():
    store = make_store()
    store.observe_intrinsics(camera_matrix(), np.zeros(5, dtype=np.float64))
    store.observe_intrinsics(np.ones((2, 2)), np.zeros(5, dtype=np.float64))
    observe_frame(store)

    evidence = store.capture(1, 1.0, corners=[])

    assert evidence.preview_jpeg is None
    assert "camera intrinsics" in evidence.missing


def test_first_intrinsics_model_does_not_reuse_frames_captured_before_it():
    store = make_store()
    observe_frame(store, stamp=1.0)
    store.observe_intrinsics(camera_matrix(), np.zeros(5, dtype=np.float64))

    evidence = store.capture(1, 1.0, corners=[])

    assert evidence.preview_jpeg is None
    assert "camera frame" in evidence.missing


def test_malformed_corners_refuse_the_preview_instead_of_drawing_an_empty_one():
    store = make_store()
    store.observe_intrinsics(camera_matrix(), np.zeros(5, dtype=np.float64))
    observe_frame(store)

    evidence = store.capture(1, 1.0, corners=[np.zeros((3, 2))])

    assert evidence.preview_jpeg is None
    assert "camera annotation" in evidence.missing


def test_intrinsics_change_does_not_fill_an_old_pending_capture():
    store = make_store()
    first_matrix = camera_matrix()
    store.observe_intrinsics(first_matrix, np.zeros(5, dtype=np.float64))
    store.capture(1, 5.0, corners=[])

    changed_matrix = first_matrix.copy()
    changed_matrix[0, 0] += 1.0
    store.observe_intrinsics(changed_matrix, np.zeros(5, dtype=np.float64))
    observe_frame(store, stamp=5.0)

    evidence = store.get(1)
    assert evidence is not None
    assert evidence.preview_jpeg is None
    assert "camera frame" in evidence.missing


def test_zero_distortion_does_not_call_the_undistort_branch(monkeypatch):
    store = make_store()
    store.observe_intrinsics(camera_matrix(), np.zeros(5, dtype=np.float64))
    observe_frame(store)

    def fail_if_called(*_args, **_kwargs):
        raise AssertionError("zero-D capture must not resample the image")

    monkeypatch.setattr(cv2, "undistort", fail_if_called)
    evidence = store.capture(1, 1.0, corners=[])

    assert evidence.preview_jpeg is not None
