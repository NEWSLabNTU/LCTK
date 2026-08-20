"""Public-interface contract for the detection buffer ADR."""

import cv2
import numpy as np
import pytest
from geometry_msgs.msg import Pose, PoseWithCovariance
from lidar_to_camera_solver.detection_buffer import (
    DetectionBuffer,
    DetectionPair,
    Empty,
    NotReady,
    Refused,
    RejectionCode,
    Solved,
)
from lidar_to_camera_solver.detection_format import (
    decode_detection_archive,
    encode_detection_archive,
    select_loaded_adjustment,
)
from lidar_to_camera_solver.main import parse_solver_mode, rotation_vector_to_euler
from scipy.spatial.transform import Rotation
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)

K = np.array([[800.0, 0.0, 640.0], [0.0, 805.0, 360.0], [0.0, 0.0, 1.0]])
MARKERS = {
    1: [
        (-0.22, -0.22, 0.0),
        (-0.12, -0.22, 0.0),
        (-0.12, -0.12, 0.0),
        (-0.22, -0.12, 0.0),
    ],
    2: [(0.12, -0.22, 0.0), (0.22, -0.22, 0.0), (0.22, -0.12, 0.0), (0.12, -0.12, 0.0)],
    3: [(-0.22, 0.12, 0.0), (-0.12, 0.12, 0.0), (-0.12, 0.22, 0.0), (-0.22, 0.22, 0.0)],
    4: [(0.12, 0.12, 0.0), (0.22, 0.12, 0.0), (0.22, 0.22, 0.0), (0.12, 0.22, 0.0)],
}


def make_buffer(*, minimum=2, enforce=False, camera_matrix=K):
    return DetectionBuffer(
        camera_matrix=camera_matrix,
        marker_corners_by_id=MARKERS,
        min_frames_required=minimum,
        min_normal_spread_deg=20.0,
        min_depth_range_m=1.0,
        enforce_pose_diversity=enforce,
    )


def make_pair(
    position=(0.0, 0.0, 3.0),
    rotation=(0.0, 0.0, 0.0),
    *,
    image_position=None,
    covariance=None,
    noise_px=0.0,
    rng=None,
):
    """Build one self-consistent wire pair; image_position can simulate bad ICP."""
    quaternion = Rotation.from_euler("xyz", rotation).as_quat()
    board_rotation = Rotation.from_quat(quaternion).as_matrix()
    projected_position = np.asarray(
        position if image_position is None else image_position, dtype=np.float64
    )

    aruco = Detection2DArray()
    for marker_id, local in MARKERS.items():
        world = (board_rotation @ np.asarray(local).T).T + projected_position
        pixels, _ = cv2.projectPoints(
            world, np.zeros((3, 1)), np.zeros((3, 1)), K, np.zeros(5)
        )
        pixels = pixels.reshape(-1, 2)
        if noise_px:
            pixels += rng.normal(0.0, noise_px, pixels.shape)

        detection = Detection2D()
        detection.id = str(marker_id)
        for pixel in pixels:
            result = ObjectHypothesisWithPose()
            result.pose = PoseWithCovariance()
            result.pose.pose = Pose()
            result.pose.pose.position.x = float(pixel[0])
            result.pose.pose.position.y = float(pixel[1])
            result.pose.pose.orientation.w = 1.0
            detection.results.append(result)
        aruco.detections.append(detection)

    board = Detection3DArray()
    detection = Detection3D()
    result = ObjectHypothesisWithPose()
    result.pose = PoseWithCovariance()
    result.pose.pose = Pose()
    result.pose.pose.position.x = float(position[0])
    result.pose.pose.position.y = float(position[1])
    result.pose.pose.position.z = float(position[2])
    result.pose.pose.orientation.x = float(quaternion[0])
    result.pose.pose.orientation.y = float(quaternion[1])
    result.pose.pose.orientation.z = float(quaternion[2])
    result.pose.pose.orientation.w = float(quaternion[3])
    if covariance is not None:
        result.pose.covariance = [
            float(value) for value in np.asarray(covariance).ravel()
        ]
    detection.results.append(result)
    board.detections.append(detection)
    return DetectionPair(aruco=aruco, board=board)


@pytest.mark.parametrize("mode", ["continuous", "manual"])
def test_solver_mode_accepts_only_named_behaviours(mode):
    assert parse_solver_mode(mode) == mode


@pytest.mark.parametrize("mode", ["", "standard", "advanced", "true"])
def test_solver_mode_rejects_removed_or_unknown_values(mode):
    with pytest.raises(ValueError, match="expected 'continuous', 'manual'"):
        parse_solver_mode(mode)


def test_continuous_policy_replaces_latest_pair_and_uses_sqpnp_plus_lm(monkeypatch):
    buffer = make_buffer(minimum=1)
    solve_flags = []
    refinement_calls = 0
    real_solve_pnp = cv2.solvePnP
    real_refine_lm = cv2.solvePnPRefineLM

    def record_solve(*args, **kwargs):
        solve_flags.append(kwargs["flags"])
        return real_solve_pnp(*args, **kwargs)

    def record_refinement(*args, **kwargs):
        nonlocal refinement_calls
        refinement_calls += 1
        return real_refine_lm(*args, **kwargs)

    monkeypatch.setattr(cv2, "solvePnP", record_solve)
    monkeypatch.setattr(cv2, "solvePnPRefineLM", record_refinement)

    first = buffer.restore([make_pair()], append=False)
    second_pair = make_pair(position=(0.7, -0.4, 4.2), rotation=(0.4, -0.25, 0.0))
    second = buffer.restore([second_pair], append=False)

    assert isinstance(first.snapshot.outcome, Solved)
    assert isinstance(second.snapshot.outcome, Solved)
    assert second.snapshot.frame_count == 1
    assert second.snapshot.estimate.quality.n_frames == 1
    assert second.snapshot.pairs[0].board.detections[0].results[
        0
    ].pose.pose.position.x == pytest.approx(0.7)
    assert solve_flags == [cv2.SOLVEPNP_SQPNP, cv2.SOLVEPNP_SQPNP]
    assert refinement_calls == 2


def test_twenty_noisy_static_captures_are_one_placement():
    rng = np.random.default_rng(0)
    buffer = make_buffer(minimum=21)
    for _ in range(20):
        jitter = rng.normal(0.0, 0.005, 3)
        update = buffer.capture(make_pair(tuple(np.array((0.0, 0.0, 3.0)) + jitter)))
        assert update.accepted

    snapshot = buffer.snapshot()
    assert snapshot.frame_count == 20
    assert snapshot.placement_count == 1


def test_moving_or_tilting_creates_placement_but_in_plane_spin_does_not():
    buffer = make_buffer(minimum=10)
    first = buffer.capture(make_pair())
    spin = buffer.capture(make_pair(rotation=(0.0, 0.0, 0.7)))
    moved = buffer.capture(make_pair(position=(0.7, 0.0, 3.5)))
    tilted = buffer.capture(
        make_pair(position=(0.7, 0.0, 3.5), rotation=(0.45, 0.0, 0.0))
    )

    assert first.added_new_placement is True
    assert spin.added_new_placement is False
    assert moved.added_new_placement is True
    assert tilted.added_new_placement is True
    assert buffer.snapshot().placement_count == 3


def test_duplicate_is_retained_and_first_capture_is_success_plus_not_ready():
    buffer = make_buffer(minimum=3)
    first = buffer.capture(make_pair())
    duplicate = buffer.capture(make_pair())

    assert first.accepted and isinstance(first.snapshot.outcome, NotReady)
    assert duplicate.accepted and duplicate.added_new_placement is False
    assert duplicate.snapshot.frame_count == 2
    assert duplicate.snapshot.placement_count == 1


def test_structural_rejection_is_atomic():
    buffer = make_buffer(minimum=2)
    accepted = buffer.capture(make_pair())
    invalid = make_pair()
    invalid.aruco.detections[0].results.clear()
    invalid.aruco.detections[1].results.clear()
    invalid.aruco.detections[2].results.clear()
    invalid.aruco.detections[3].results.clear()

    rejected = buffer.capture(invalid)

    assert not rejected.accepted
    assert rejected.rejection.code is RejectionCode.NO_REAL_ARUCO_CORNERS
    assert rejected.snapshot.revision == accepted.snapshot.revision
    assert rejected.snapshot.frame_count == 1


def test_enough_valid_captures_solve_and_keep_quality_verdict():
    buffer = make_buffer(minimum=2)
    buffer.capture(make_pair())
    update = buffer.capture(
        make_pair(position=(0.7, -0.4, 4.2), rotation=(0.4, -0.25, 0.0))
    )

    assert isinstance(update.snapshot.outcome, Solved)
    assert update.snapshot.estimate.quality.n_frames == 2
    assert update.snapshot.estimate.quality.status_line()


def test_solved_estimate_can_be_rendered_as_pose_info():
    """Node must be able to pass snapshot rotation into system SciPy."""
    buffer = make_buffer(minimum=2)
    buffer.capture(make_pair())
    snapshot = buffer.capture(
        make_pair(position=(0.7, -0.4, 4.2), rotation=(0.4, -0.25, 0.0))
    ).snapshot

    rotation_vector_to_euler(snapshot.estimate.rvec)


def test_degenerate_solve_is_solved_unless_policy_refuses_it():
    warning_buffer = make_buffer(minimum=2, enforce=False)
    warning_buffer.capture(make_pair())
    warning = warning_buffer.capture(make_pair())
    assert isinstance(warning.snapshot.outcome, Solved)
    assert warning.snapshot.estimate.quality.is_degenerate

    enforcing_buffer = make_buffer(minimum=2, enforce=True)
    enforcing_buffer.capture(make_pair())
    refused = enforcing_buffer.capture(make_pair())
    assert isinstance(refused.snapshot.outcome, Refused)
    assert refused.snapshot.estimate is None


def test_invalid_removal_is_atomic_and_valid_removal_invalidates_estimate():
    buffer = make_buffer(minimum=2)
    buffer.capture(make_pair())
    solved = buffer.capture(
        make_pair(position=(0.7, -0.4, 4.2), rotation=(0.4, -0.25, 0.0))
    )

    rejected = buffer.remove(9)
    assert not rejected.accepted
    assert rejected.snapshot.revision == solved.snapshot.revision
    assert isinstance(rejected.snapshot.outcome, Solved)

    removed = buffer.remove(0)
    assert removed.accepted
    assert removed.snapshot.revision == solved.snapshot.revision + 1
    assert isinstance(removed.snapshot.outcome, NotReady)
    assert removed.snapshot.estimate is None


def test_clear_removes_every_derived_value_and_empty_clear_is_noop():
    buffer = make_buffer(minimum=2)
    buffer.capture(make_pair())
    solved = buffer.capture(
        make_pair(position=(0.7, -0.4, 4.2), rotation=(0.4, -0.25, 0.0))
    )

    cleared = buffer.clear()
    assert cleared.accepted and cleared.changed
    assert cleared.snapshot.revision == solved.snapshot.revision + 1
    assert cleared.snapshot.frame_count == 0
    assert cleared.snapshot.placement_count == 0
    assert cleared.snapshot.correspondence_count == 0
    assert cleared.snapshot.estimate is None
    assert isinstance(cleared.snapshot.outcome, Empty)

    again = buffer.clear()
    assert again.accepted and not again.changed
    assert again.snapshot.revision == cleared.snapshot.revision


def test_restore_append_and_replace_are_atomic_and_solve_once(monkeypatch):
    buffer = make_buffer(minimum=2)
    calls = 0
    real_solve_pnp = cv2.solvePnP

    def counted(*args, **kwargs):
        nonlocal calls
        calls += 1
        return real_solve_pnp(*args, **kwargs)

    monkeypatch.setattr(cv2, "solvePnP", counted)
    restored = buffer.restore(
        [
            make_pair(),
            make_pair(position=(0.7, -0.4, 4.2), rotation=(0.4, -0.25, 0.0)),
        ],
        append=False,
    )
    assert restored.accepted and isinstance(restored.snapshot.outcome, Solved)
    assert calls == 1

    appended = buffer.restore([make_pair(position=(-0.8, 0.5, 3.7))], append=True)
    assert appended.accepted
    assert appended.snapshot.frame_count == 3
    assert calls == 2

    replacement = buffer.restore([make_pair()], append=False)
    assert replacement.accepted
    assert replacement.snapshot.frame_count == 1
    assert isinstance(replacement.snapshot.outcome, NotReady)
    assert calls == 2


def test_rejected_restore_leaves_previous_snapshot_unchanged():
    buffer = make_buffer(minimum=2)
    before = buffer.restore(
        [
            make_pair(),
            make_pair(position=(0.7, -0.4, 4.2), rotation=(0.4, -0.25, 0.0)),
        ],
        append=False,
    ).snapshot
    invalid = make_pair()
    invalid.board.detections.clear()

    rejected = buffer.restore([make_pair(), invalid], append=False)

    assert not rejected.accepted
    assert rejected.snapshot.revision == before.revision
    assert rejected.snapshot.frame_count == before.frame_count
    assert np.array_equal(rejected.snapshot.estimate.rvec, before.estimate.rvec)


def test_snapshot_is_detached_from_live_messages_and_estimate_arrays():
    buffer = make_buffer(minimum=2)
    buffer.capture(make_pair())
    snapshot = buffer.capture(
        make_pair(position=(0.7, -0.4, 4.2), rotation=(0.4, -0.25, 0.0))
    ).snapshot

    snapshot.pairs[0].aruco.detections.clear()
    with pytest.raises(ValueError):
        snapshot.estimate.rvec[0, 0] = 99.0

    fresh = buffer.snapshot()
    assert len(fresh.pairs[0].aruco.detections) == 4
    assert fresh.estimate.rvec[0, 0] != 99.0


def test_covariance_weighting_improves_public_solved_result():
    tight = np.diag([1e-6] * 6)
    loose = np.diag([1e-2, 1e-2, 1e-6, 1e-6, 1e-6, 1e-2])
    specs = [
        ((-0.5, -0.4, 3.0), (0.1, -0.1, 0.0)),
        ((0.4, -0.3, 3.6), (-0.2, 0.25, 0.0)),
        ((-0.3, 0.5, 4.2), (0.3, 0.1, 0.0)),
        ((0.6, 0.4, 4.8), (-0.3, -0.2, 0.0)),
    ]

    weighted_pairs = []
    unweighted_pairs = []
    for index, (true_position, rotation) in enumerate(specs):
        observed_position = true_position
        covariance = tight
        if index == 1:
            observed_position = tuple(np.asarray(true_position) + (0.0, 0.08, 0.08))
            covariance = loose
        weighted_pairs.append(
            make_pair(
                observed_position,
                rotation,
                image_position=true_position,
                covariance=covariance,
            )
        )
        unweighted_pairs.append(
            make_pair(
                observed_position,
                rotation,
                image_position=true_position,
                covariance=np.zeros((6, 6)),
            )
        )

    weighted = make_buffer(minimum=4).restore(weighted_pairs, append=False).snapshot
    unweighted = make_buffer(minimum=4).restore(unweighted_pairs, append=False).snapshot
    assert isinstance(weighted.outcome, Solved)
    assert isinstance(unweighted.outcome, Solved)

    weighted_error = np.linalg.norm(weighted.estimate.tvec)
    unweighted_error = np.linalg.norm(unweighted.estimate.tvec)
    assert weighted_error < unweighted_error


def test_complete_archive_round_trip_keeps_pairs_quality_and_adjustment():
    buffer = make_buffer(minimum=2)
    snapshot = buffer.restore(
        [
            make_pair(),
            make_pair(position=(0.7, -0.4, 4.2), rotation=(0.4, -0.25, 0.0)),
        ],
        append=False,
    ).snapshot
    adjusted_rvec = snapshot.estimate.rvec + np.array([[0.01], [0.0], [0.0]])
    adjusted_tvec = snapshot.estimate.tvec + np.array([[0.0], [0.02], [0.0]])

    archive = decode_detection_archive(
        encode_detection_archive(
            snapshot,
            adjusted_rvec=adjusted_rvec,
            adjusted_tvec=adjusted_tvec,
        )
    )

    assert len(archive.pairs) == 2
    assert archive.quality.status == snapshot.estimate.quality.status_line()
    assert archive.quality.is_degenerate == snapshot.estimate.quality.is_degenerate
    assert np.array_equal(archive.adjusted_transform.rvec, adjusted_rvec)
    assert np.array_equal(archive.adjusted_transform.tvec, adjusted_tvec)

    replacement = select_loaded_adjustment(archive, snapshot, append=False)
    appended = select_loaded_adjustment(archive, snapshot, append=True)
    assert np.array_equal(replacement.rvec, adjusted_rvec)
    assert np.array_equal(replacement.tvec, adjusted_tvec)
    assert np.array_equal(appended.rvec, snapshot.estimate.rvec)
    assert np.array_equal(appended.tvec, snapshot.estimate.tvec)


def test_archive_adjustment_is_never_restored_without_current_solve():
    buffer = make_buffer(minimum=2)
    not_ready = buffer.capture(make_pair()).snapshot
    archive = decode_detection_archive(
        {
            "version": 4,
            "board_frame_convention": "corner_aligned_plate_center_v1",
            "num_detections": 0,
            "detections": [],
            "transform": {"rvec": [0.1, 0.2, 0.3], "tvec": [1.0, 2.0, 3.0]},
        }
    )

    assert select_loaded_adjustment(archive, not_ready, append=False) is None


def test_malformed_archive_is_rejected_before_any_restore():
    with pytest.raises(ValueError, match="count mismatch"):
        decode_detection_archive(
            {
                "version": 4,
                "board_frame_convention": "corner_aligned_plate_center_v1",
                "num_detections": 1,
                "detections": [],
            }
        )
