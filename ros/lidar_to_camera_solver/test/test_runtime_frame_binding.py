"""Runtime frame and camera-projection binding for the LiDAR-camera solver."""

import copy
import json
import threading
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from builtin_interfaces.msg import Time
from lctk_autoware_export.export import ExportError, check_format_version
from lctk_interfaces.msg import CalibrationTargetIdentity
from lctk_quality.projection_metadata import PROJECTION_MODEL
from lctk_target import load_target
from lidar_to_camera_solver.board_geometry import TargetIdentityGate, identity_fields
from lidar_to_camera_solver.detection_buffer import (
    BufferSnapshot,
    BufferUpdate,
    DetectionPair,
    Empty,
)
from lidar_to_camera_solver.detection_format import (
    decode_detection_archive,
    encode_detection_archive,
)
from lidar_to_camera_solver.main import LidarToCameraSolver
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)

ROOT = Path(__file__).resolve().parents[3]
TARGET = load_target(ROOT / "ros/lctk_launch/config/targets/solid_600_aruco_1_v1.json5")
K = [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]


class _Logger:
    def __init__(self):
        self.messages = []

    def info(self, message, **_kwargs):
        self.messages.append(message)

    def debug(self, message, **_kwargs):
        self.messages.append(message)

    def warn(self, message, **_kwargs):
        self.messages.append(message)


class _PairSource:
    def __init__(self):
        self.discarded = 0

    def discard_cached_pair(self):
        self.discarded += 1


class _ArchiveBuffer:
    def __init__(self, snapshot):
        self._snapshot = snapshot
        self.restores = 0

    def snapshot(self):
        return self._snapshot

    def clear(self):
        self._snapshot = _snapshot(revision=self._snapshot.revision + 1)
        return BufferUpdate(accepted=True, changed=True, snapshot=self._snapshot)

    def restore(self, pairs, *, append):
        self.restores += 1
        next_pairs = (*self._snapshot.pairs, *pairs) if append else tuple(pairs)
        self._snapshot = BufferSnapshot(
            revision=self._snapshot.revision + 1,
            pairs=next_pairs,
            placements=(),
            correspondence_count=0,
            outcome=Empty(),
        )
        return BufferUpdate(accepted=True, changed=True, snapshot=self._snapshot)


def _solver():
    solver = object.__new__(LidarToCameraSolver)
    solver.target = TARGET
    solver.marker_corners_by_id = TARGET.marker_corners_by_id
    solver.solve_min_frames = 1
    solver.min_normal_spread_deg = 0.0
    solver.min_depth_range_m = 0.0
    solver.enforce_pose_diversity = False
    solver.state_lock = threading.RLock()
    solver.identity_gate = TargetIdentityGate(TARGET.identity)
    message = CalibrationTargetIdentity(**identity_fields(TARGET.identity))
    solver.identity_gate.update("lidar", message)
    solver.identity_gate.update("camera", message)
    solver._identity_generation = 0
    solver._scene_revision = 0
    solver._autoware_pending_export = None
    solver._camera_projection = None
    solver._bound_lidar_frame = None
    solver._bound_camera_frame = None
    solver.camera_info = None
    solver._camera_matrix = None
    solver.detection_buffer = None
    solver.pair_source = _PairSource()
    solver._evidence_store = None
    solver._review_capture_ids = set()
    solver._review_read_model = None
    solver._stillness = None
    solver.current_rvec = None
    solver.current_tvec = None
    solver.last_transform = None
    solver.publishing_enabled = False
    solver._logger = _Logger()
    solver.get_logger = lambda: solver._logger
    return solver


def _camera_info(*, frame="camera_optical", width=640, height=480, k=K):
    message = CameraInfo()
    message.header.frame_id = frame
    message.width = width
    message.height = height
    message.k = list(k)
    return message


def _pair(*, camera="camera_optical", lidar="lidar_top"):
    aruco = Detection2DArray()
    aruco.header.frame_id = camera
    board = Detection3DArray()
    board.header.frame_id = lidar
    return aruco, board


def _valid_pair(*, camera="camera_optical", lidar="lidar_top"):
    """Build one restorable Capture with marker 24 and a finite board pose."""
    aruco, board = _pair(camera=camera, lidar=lidar)
    aruco.header.stamp.sec = board.header.stamp.sec = 1

    aruco_detection = Detection2D()
    aruco_detection.id = "aruco_24"
    for x, y, _z in TARGET.marker_corners_by_id[24]:
        corner = ObjectHypothesisWithPose()
        corner.pose.pose.position.x = 500.0 * x / 3.0 + 320.0
        corner.pose.pose.position.y = 500.0 * y / 3.0 + 240.0
        aruco_detection.results.append(corner)
    aruco.detections.append(aruco_detection)

    board_detection = Detection3D()
    board_pose = ObjectHypothesisWithPose()
    board_pose.pose.pose.position.z = 3.0
    board_pose.pose.pose.orientation.w = 1.0
    board_pose.pose.covariance = [0.0] * 36
    board_detection.results.append(board_pose)
    board.detections.append(board_detection)
    return aruco, board


def _snapshot(pairs=(), *, revision=0):
    return BufferSnapshot(
        revision=revision,
        pairs=tuple(DetectionPair(aruco=aruco, board=board) for aruco, board in pairs),
        placements=(),
        correspondence_count=0,
        outcome=Empty(),
    )


def _write_archive(path, *, projection, pairs=()):
    archive = encode_detection_archive(
        _snapshot(pairs),
        local_identity=TARGET.identity,
        camera_projection=projection,
        adjusted_rvec=None,
        adjusted_tvec=None,
    )
    path.write_text(json.dumps(archive))


def _load(solver, path, *, append):
    request = SimpleNamespace(file_path=str(path), append=append)
    response = SimpleNamespace(
        success=None, message=None, num_detections=None, buffer_size=None
    )
    return LidarToCameraSolver.load_detections_callback(solver, request, response)


def test_admission_requires_camera_info_and_binds_actual_detection_headers():
    solver = _solver()
    aruco, board = _pair()

    assert "CameraInfo" in solver._admit_detection_pair((aruco, board))

    solver.camera_info_callback(_camera_info())
    assert solver._admit_detection_pair((aruco, board)) is None
    assert solver._bound_lidar_frame == "lidar_top"
    assert solver._bound_camera_frame == "camera_optical"
    assert solver._camera_projection == {
        "model": PROJECTION_MODEL,
        "frame_id": "camera_optical",
        "width": 640,
        "height": 480,
        "k": K,
    }


def test_empty_or_camera_info_mismatched_detection_frames_are_rejected():
    solver = _solver()
    solver.camera_info_callback(_camera_info())

    aruco, board = _pair(camera="", lidar="lidar_top")
    assert "nonempty" in solver._admit_detection_pair((aruco, board))

    aruco, board = _pair(camera="other_camera", lidar="lidar_top")
    assert "CameraInfo" in solver._admit_detection_pair((aruco, board))

    assert solver._admit_detection_pair(_pair()) is None
    old_buffer = _ArchiveBuffer(_snapshot([_pair()]))
    solver.detection_buffer = old_buffer
    solver.current_rvec = np.ones((3, 1))
    solver.current_tvec = np.ones((3, 1))
    solver.last_transform = object()
    solver.publishing_enabled = True
    generation = solver._identity_generation
    aruco, board = _pair(camera="other_camera", lidar="lidar_top")

    assert "CameraInfo" in solver._admit_detection_pair((aruco, board))
    assert solver._identity_generation == generation + 1
    assert old_buffer.snapshot().frame_count == 0
    assert solver.last_transform is None
    assert solver.current_rvec is None
    assert solver.current_tvec is None
    assert not solver.publishing_enabled
    assert solver._bound_lidar_frame is None
    assert solver._bound_camera_frame is None

    assert solver._admit_detection_pair(_pair()) is None
    old_buffer._snapshot = _snapshot([_pair()], revision=old_buffer.snapshot().revision)
    solver.current_rvec = np.ones((3, 1))
    solver.last_transform = object()
    solver.publishing_enabled = True
    generation = solver._identity_generation
    aruco, board = _pair(camera="", lidar="lidar_top")

    assert "nonempty" in solver._admit_detection_pair((aruco, board))
    assert solver._identity_generation == generation + 1
    assert old_buffer.snapshot().frame_count == 0
    assert solver.last_transform is None
    assert solver.current_rvec is None
    assert solver.current_tvec is None
    assert not solver.publishing_enabled


def test_lidar_frame_change_clears_old_estimate_before_binding_new_pair():
    solver = _solver()
    solver.camera_info_callback(_camera_info())
    assert solver._admit_detection_pair(_pair()) is None
    old_buffer = _ArchiveBuffer(_snapshot([_pair()]))
    solver.detection_buffer = old_buffer
    solver.current_rvec = np.ones((3, 1))
    solver.current_tvec = np.ones((3, 1))
    solver.last_transform = object()
    solver.publishing_enabled = True
    old_generation = solver._identity_generation

    assert solver._admit_detection_pair(_pair(lidar="lidar_reconfigured")) is None

    assert solver._identity_generation == old_generation + 1
    assert solver.current_rvec is None
    assert solver.current_tvec is None
    assert solver.last_transform is None
    assert not solver.publishing_enabled
    assert old_buffer.snapshot().frame_count == 0
    assert solver._bound_lidar_frame == "lidar_reconfigured"
    assert solver._bound_camera_frame == "camera_optical"
    assert solver.pair_source.discarded == 1


def test_camera_projection_frame_or_dimensions_change_starts_new_epoch():
    solver = _solver()
    solver.camera_info_callback(_camera_info())
    assert solver._admit_detection_pair(_pair()) is None
    solver.current_rvec = np.ones((3, 1))
    solver.last_transform = object()
    solver.publishing_enabled = True
    old_generation = solver._identity_generation
    old_buffer = _ArchiveBuffer(_snapshot([_pair()]))
    solver.detection_buffer = old_buffer

    solver.camera_info_callback(_camera_info(frame="camera_replaced"))

    assert solver._identity_generation == old_generation + 1
    assert solver.detection_buffer is not old_buffer
    assert old_buffer.snapshot().frame_count == 0
    assert solver.current_rvec is None
    assert solver.last_transform is None
    assert not solver.publishing_enabled
    assert solver._camera_projection["frame_id"] == "camera_replaced"
    assert solver._camera_projection["width"] == 640
    assert solver._bound_lidar_frame is None
    assert solver._bound_camera_frame is None
    assert solver.pair_source.discarded == 1


def test_camera_info_dimension_change_starts_new_epoch():
    solver = _solver()
    solver.camera_info_callback(_camera_info())
    old_generation = solver._identity_generation
    old_buffer = solver.detection_buffer

    solver.camera_info_callback(_camera_info(width=800))

    assert solver._identity_generation == old_generation + 1
    assert solver.detection_buffer is not old_buffer
    assert solver._camera_projection["width"] == 800


def test_camera_info_intrinsic_updates_preserve_captures_and_estimate():
    from unittest.mock import Mock

    solver = _solver()
    solver._evidence_store = Mock()
    solver._review_read_model = Mock()
    first = _camera_info()
    first.d = [0.1, 0.0, 0.0, 0.0, 0.0]
    solver.camera_info_callback(first)
    buffer = _ArchiveBuffer(_snapshot([_pair()]))
    solver.detection_buffer = buffer
    transform = object()
    solver.last_transform = transform
    solver.current_rvec = np.ones((3, 1))
    solver.publishing_enabled = True
    generation = solver._identity_generation
    scene_revision = solver._scene_revision
    changed_k = list(K)
    changed_k[0] += 0.01
    for k in (K, changed_k, changed_k):
        update = _camera_info(k=k)
        update.d = [0.2, 0.0, 0.0, 0.0, 0.0]
        solver.camera_info_callback(update)

    assert solver._identity_generation == generation
    assert solver._scene_revision == scene_revision
    assert solver.detection_buffer is buffer
    assert buffer.snapshot().frame_count == 1
    assert solver.last_transform is transform
    assert solver.current_rvec is not None
    assert solver.publishing_enabled
    assert solver._camera_projection["k"] == K
    np.testing.assert_array_equal(solver._camera_matrix, np.asarray(K).reshape(3, 3))
    assert list(solver.camera_info.d) == list(first.d)
    for call in solver._evidence_store.observe_intrinsics.call_args_list:
        np.testing.assert_array_equal(call.args[0], np.asarray(K).reshape(3, 3))
        np.testing.assert_array_equal(call.args[1], first.d)
    solver._review_read_model.reset.assert_not_called()


def test_intrinsic_pin_survives_frame_reset_and_is_local_to_solver():
    solver = _solver()
    solver.camera_info_callback(_camera_info())
    changed_k = list(K)
    changed_k[0] += 1.0
    solver.camera_info_callback(_camera_info(frame="camera_replaced", k=changed_k))
    assert solver._camera_projection["frame_id"] == "camera_replaced"
    assert solver._camera_projection["k"] == K
    other = _solver()
    other.camera_info_callback(_camera_info(k=changed_k))
    assert other._camera_projection["k"] == changed_k


def test_invalid_first_camera_info_does_not_pin_intrinsics():
    solver = _solver()
    solver.camera_info_callback(_camera_info(k=[0.0] * 9))
    assert solver._camera_projection is None
    solver.camera_info_callback(_camera_info())
    assert solver._camera_projection["k"] == K


def test_transform_labels_use_bound_sensor_headers():
    solver = _solver()
    solver._bound_lidar_frame = "lidar_from_detection"
    solver._bound_camera_frame = "camera_from_detection"
    solver.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: Time())
    )

    message = solver._create_transform_message(np.zeros(3), np.zeros(3))

    assert message.header.frame_id == "lidar_from_detection"
    assert message.child_frame_id == "camera_from_detection"


def test_transform_cannot_be_created_without_two_distinct_bound_frames():
    solver = _solver()
    solver._bound_lidar_frame = "lidar"
    solver._bound_camera_frame = "lidar"

    try:
        solver._create_transform_message(np.zeros(3), np.zeros(3))
    except ValueError as error:
        assert "distinct" in str(error)
    else:
        raise AssertionError("invalid runtime sensor frame binding was published")


def test_load_rejects_archive_with_different_camera_projection_without_restore(
    tmp_path,
):
    solver = _solver()
    solver.camera_info_callback(_camera_info())
    buffer = _ArchiveBuffer(_snapshot())
    solver.detection_buffer = buffer
    archive_projection = dict(solver._camera_projection)
    archive_projection["frame_id"] = "other_camera"
    path = tmp_path / "projection.json"
    _write_archive(path, projection=archive_projection)

    response = _load(solver, path, append=False)

    assert response.success is False
    assert "projection does not match" in response.message
    assert buffer.restores == 0
    assert buffer.snapshot().frame_count == 0


def test_load_archive_without_camera_info_uses_saved_projection(tmp_path):
    solver = _solver()
    archived_pair = _valid_pair(camera="archived_camera", lidar="archived_lidar")
    archive_projection = {
        "model": PROJECTION_MODEL,
        "frame_id": "archived_camera",
        "width": 640,
        "height": 480,
        "k": K,
    }
    path = tmp_path / "offline.json"
    _write_archive(path, projection=archive_projection, pairs=[archived_pair])
    solver.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: Time())
    )
    response = _load(solver, path, append=False)

    assert response.success is True
    assert response.num_detections == 1
    assert response.buffer_size == 1
    assert solver.camera_info is None
    assert solver._camera_projection == archive_projection
    assert np.array_equal(solver._camera_matrix, np.asarray(K).reshape(3, 3))
    assert solver._bound_camera_frame == "archived_camera"
    assert solver._bound_lidar_frame == "archived_lidar"
    assert solver.detection_buffer.snapshot().frame_count == 1
    assert solver.detection_buffer.snapshot().estimate is not None


def test_archive_load_cannot_replace_pinned_intrinsics(tmp_path):
    solver = _solver()
    solver.camera_info_callback(_camera_info())
    projection = copy.deepcopy(solver._camera_projection)
    projection["k"][0] += 1.0
    path = tmp_path / "different-intrinsics.json"
    _write_archive(path, projection=projection)

    response = _load(solver, path, append=False)

    assert not response.success
    assert "projection does not match" in response.message
    assert solver._camera_projection["k"] == K
    assert list(solver._pinned_camera_info.k) == K


def test_first_camera_info_after_offline_archive_establishes_pin(tmp_path):
    solver = _solver()
    projection = {
        "model": PROJECTION_MODEL,
        "frame_id": "camera_optical",
        "width": 640,
        "height": 480,
        "k": K,
    }
    path = tmp_path / "offline.json"
    _write_archive(path, projection=projection, pairs=[_valid_pair()])
    solver.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: Time())
    )
    assert _load(solver, path, append=False).success
    changed_k = list(K)
    changed_k[0] += 1.0
    solver.camera_info_callback(_camera_info(k=changed_k))
    assert solver._camera_projection["k"] == changed_k
    assert list(solver._pinned_camera_info.k) == changed_k
    solver.camera_info_callback(_camera_info())
    assert solver._camera_projection["k"] == changed_k


def test_rejected_cold_start_archive_does_not_bind_projection(tmp_path):
    solver = _solver()
    first_projection = {
        "model": PROJECTION_MODEL,
        "frame_id": "empty_archive_camera",
        "width": 640,
        "height": 480,
        "k": K,
    }
    first_path = tmp_path / "empty.json"
    _write_archive(
        first_path,
        projection=first_projection,
        pairs=[_pair(camera="empty_archive_camera", lidar="empty_archive_lidar")],
    )

    rejected = _load(solver, first_path, append=False)

    assert rejected.success is False
    assert "No board detection available" in rejected.message
    assert solver.detection_buffer is None
    assert solver._camera_projection is None
    assert solver._camera_matrix is None

    second_projection = {
        **first_projection,
        "frame_id": "valid_archive_camera",
    }
    second_path = tmp_path / "valid.json"
    _write_archive(
        second_path,
        projection=second_projection,
        pairs=[_valid_pair(camera="valid_archive_camera", lidar="valid_archive_lidar")],
    )
    solver.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: Time())
    )

    accepted = _load(solver, second_path, append=False)

    assert accepted.success is True
    assert solver._camera_projection == second_projection
    assert solver._bound_camera_frame == "valid_archive_camera"
    assert solver._bound_lidar_frame == "valid_archive_lidar"


def test_append_rejects_archive_from_another_lidar_frame_without_restore(tmp_path):
    solver = _solver()
    solver.camera_info_callback(_camera_info())
    active_pair = _pair(lidar="lidar_current")
    buffer = _ArchiveBuffer(_snapshot([active_pair]))
    solver.detection_buffer = buffer
    solver._bound_lidar_frame = "lidar_current"
    solver._bound_camera_frame = "camera_optical"
    archived_pair = _pair(lidar="lidar_archive")
    path = tmp_path / "other_lidar.json"
    _write_archive(path, projection=solver._camera_projection, pairs=[archived_pair])

    response = _load(solver, path, append=True)

    assert response.success is False
    assert "sensor frames do not match" in response.message
    assert buffer.restores == 0
    assert buffer.snapshot().frame_count == 1


def test_replacement_load_binds_frames_from_archive_headers(tmp_path):
    solver = _solver()
    solver.camera_info_callback(_camera_info())
    buffer = _ArchiveBuffer(_snapshot())
    solver.detection_buffer = buffer
    pair = _pair(lidar="lidar_archived")
    path = tmp_path / "replacement.json"
    _write_archive(path, projection=solver._camera_projection, pairs=[pair])

    response = _load(solver, path, append=False)

    assert response.success is True
    assert solver._bound_lidar_frame == "lidar_archived"
    assert solver._bound_camera_frame == "camera_optical"
    assert buffer.restores == 1
    assert solver.pair_source.discarded == 1


def test_solver_and_exporter_accept_the_same_v6_archive():
    archive = encode_detection_archive(
        _snapshot([_pair()]),
        local_identity=TARGET.identity,
        camera_projection={
            "model": PROJECTION_MODEL,
            "frame_id": "camera_optical",
            "width": 640,
            "height": 480,
            "k": K,
        },
        adjusted_rvec=None,
        adjusted_tvec=None,
    )

    decoded = decode_detection_archive(archive, local_identity=TARGET.identity)

    assert decoded.camera_frame_id == "camera_optical"
    assert decoded.lidar_frame_id == "lidar_top"
    assert check_format_version("valid-v6.json", archive) is None


@pytest.mark.parametrize(
    ("mutation", "diagnostic"),
    [
        ("version", "version"),
        ("projection_model", "camera_projection.model"),
        ("projection_frame", "Camera detection frame"),
        ("empty_header", "nonempty"),
        ("count", "count mismatch"),
    ],
)
def test_solver_and_exporter_reject_the_same_malformed_v6_archive(mutation, diagnostic):
    archive = encode_detection_archive(
        _snapshot([_pair()]),
        local_identity=TARGET.identity,
        camera_projection={
            "model": PROJECTION_MODEL,
            "frame_id": "camera_optical",
            "width": 640,
            "height": 480,
            "k": K,
        },
        adjusted_rvec=None,
        adjusted_tvec=None,
    )
    archive = copy.deepcopy(archive)
    if mutation == "version":
        archive["version"] = 5
    elif mutation == "projection_model":
        archive["camera_projection"]["model"] = "raw_distorted_pixels"
    elif mutation == "projection_frame":
        archive["camera_projection"]["frame_id"] = "stale_camera"
    elif mutation == "empty_header":
        archive["detections"][0]["board"]["header"]["frame_id"] = ""
    else:
        archive["num_detections"] += 1

    with pytest.raises(ValueError, match=diagnostic):
        decode_detection_archive(archive, local_identity=TARGET.identity)
    with pytest.raises(ExportError, match=diagnostic):
        check_format_version("invalid-v6.json", archive)
