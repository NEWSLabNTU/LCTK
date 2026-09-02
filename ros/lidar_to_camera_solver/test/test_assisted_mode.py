"""assisted mode: the third solver_mode.

These tests build the node with ``object.__new__`` and set only the attributes
under test, the same way test_identity_node_contract.py does -- no ROS graph, no
camera, no synchronizer.

What is worth pinning here is the *gating*, not the plumbing. Assisted mode
auto-captures, so the two gates are the only thing standing between an operator
walking a board around and a buffer full of forty views of one placement --
exactly the degenerate capture ``lctk_quality`` exists to detect and that every
residual-based number rates as excellent.
"""

from __future__ import annotations

import threading
from types import SimpleNamespace

import numpy as np
import pytest
from lctk_quality.placements import Placement
from lidar_to_camera_solver.detection_buffer import (
    BufferSnapshot,
    BufferUpdate,
    Empty,
)
from lidar_to_camera_solver.main import (
    SOLVER_MODES,
    LidarToCameraSolver,
    aruco_corner_quads,
    board_pose_from_detections,
    parse_solver_mode,
    placement_is_new,
)
from lidar_to_camera_solver.stability import StillnessTracker


def test_assisted_is_a_valid_mode():
    assert parse_solver_mode("assisted") == "assisted"


def test_the_three_modes_are_exactly_these():
    assert SOLVER_MODES == ("continuous", "manual", "assisted")


def test_an_unknown_mode_names_all_three():
    with pytest.raises(ValueError, match="continuous', 'manual', 'assisted"):
        parse_solver_mode("automatic")


# --- message shims ------------------------------------------------------------
# Duck-typed stand-ins for Detection3DArray/Detection2DArray. The extraction
# helpers read them with getattr, exactly as DetectionBuffer._prepare_pair does,
# so a namespace is a faithful stand-in and needs no ROS graph.


def board_message(position=(1.0, 2.0, 3.0), quaternion=(0.0, 0.0, 0.0, 1.0)):
    pose = SimpleNamespace(
        position=SimpleNamespace(
            x=float(position[0]), y=float(position[1]), z=float(position[2])
        ),
        orientation=SimpleNamespace(
            x=float(quaternion[0]),
            y=float(quaternion[1]),
            z=float(quaternion[2]),
            w=float(quaternion[3]),
        ),
    )
    result = SimpleNamespace(pose=SimpleNamespace(pose=pose))
    return SimpleNamespace(detections=[SimpleNamespace(results=[result])])


def aruco_message(quads=((0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0))):
    results = [
        SimpleNamespace(
            pose=SimpleNamespace(
                pose=SimpleNamespace(position=SimpleNamespace(x=x, y=y))
            )
        )
        for x, y in quads
    ]
    return SimpleNamespace(detections=[SimpleNamespace(id="aruco_0", results=results)])


def test_board_pose_reads_position_and_normalised_orientation():
    pose = board_pose_from_detections(
        board_message(position=(1.0, 2.0, 3.0), quaternion=(0.0, 0.0, 0.0, 2.0))
    )
    assert pose is not None
    position, orientation = pose
    assert position == (1.0, 2.0, 3.0)
    assert orientation == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_board_pose_is_none_when_there_is_nothing_to_read():
    assert board_pose_from_detections(SimpleNamespace(detections=[])) is None
    assert (
        board_pose_from_detections(SimpleNamespace(detections=[SimpleNamespace()]))
        is None
    )


def test_board_pose_is_none_for_a_degenerate_quaternion():
    assert (
        board_pose_from_detections(board_message(quaternion=(0.0, 0.0, 0.0, 0.0)))
        is None
    )


def test_aruco_corner_quads_returns_one_four_by_two_array_per_marker():
    quads = aruco_corner_quads(aruco_message())
    assert len(quads) == 1
    assert quads[0].shape == (4, 2)
    assert quads[0][1].tolist() == [4.0, 0.0]


def test_aruco_corner_quads_skips_a_detection_without_four_corners():
    message = aruco_message()
    message.detections[0].results = message.detections[0].results[:3]
    assert aruco_corner_quads(message) == []


# --- capture-callback harness -------------------------------------------------


class _Logger:
    def __init__(self):
        self.messages = []

    def info(self, message, **_kwargs):
        self.messages.append(message)

    def debug(self, message, **_kwargs):
        self.messages.append(message)

    def warn(self, message, **_kwargs):
        self.messages.append(message)

    def warning(self, message, **_kwargs):
        self.messages.append(message)

    def error(self, message, **_kwargs):
        self.messages.append(message)


def _snapshot(frame_count: int, placements=()) -> BufferSnapshot:
    return BufferSnapshot(
        revision=frame_count,
        pairs=tuple(object() for _ in range(frame_count)),
        placements=tuple(placements),
        correspondence_count=4 * frame_count,
        outcome=Empty(),
    )


class _Buffer:
    """Accepts every capture; reports novelty from a scripted sequence."""

    def __init__(self, novelty=(True,) * 32, accepted=True, placements=()):
        self.captures = []
        self.removed = []
        self.count = 0
        self.placements = tuple(placements)
        self._novelty = list(novelty)
        self._accepted = accepted

    def capture(self, pair):
        self.captures.append(pair)
        if not self._accepted:
            return BufferUpdate(
                accepted=False,
                changed=False,
                snapshot=_snapshot(self.count, self.placements),
            )
        self.count += 1
        return BufferUpdate(
            accepted=True,
            changed=True,
            snapshot=_snapshot(self.count, self.placements),
            added_new_placement=self._novelty.pop(0),
        )

    def remove(self, index):
        self.removed.append(index)
        self.count -= 1
        return BufferUpdate(
            accepted=True, changed=True, snapshot=_snapshot(self.count, self.placements)
        )

    def snapshot(self):
        return _snapshot(self.count, self.placements)


class _PreviewStore:
    def __init__(self):
        self.captured = []
        self.dropped = []

    def capture(self, pair_id, corners, reprojected):
        self.captured.append((pair_id, corners, reprojected))
        return True

    def get(self, pair_id):
        return b"\xff\xd8" if pair_id in [p for p, _, _ in self.captured] else None

    def drop(self, pair_id):
        self.dropped.append(pair_id)


class _Gate:
    def __init__(self, error=None):
        self.error = error


class _PairSource:
    def __init__(self):
        self.epoch_resets = 0

    def status_line(self):
        return "sync: groups=12"


class _Clock:
    def __init__(self):
        self.seconds = 0.0

    def now(self):
        return SimpleNamespace(nanoseconds=int(self.seconds * 1e9))


def assisted_harness(
    *, novelty=(True,) * 32, accepted=True, placements=()
) -> LidarToCameraSolver:
    solver = object.__new__(LidarToCameraSolver)
    solver.solver_mode = "assisted"
    solver.state_lock = threading.RLock()
    solver._identity_generation = 0
    solver.identity_gate = _Gate()
    solver.detection_buffer = _Buffer(
        novelty=novelty, accepted=accepted, placements=placements
    )
    solver.pair_source = _PairSource()
    solver._preview_store = _PreviewStore()
    solver._stillness = StillnessTracker(
        # `hold()` below steps the clock 0.1 s per pair, so a 0.25 s window is
        # satisfied by the fourth pair of a hold: three inside the window plus
        # one bracketing it.  Short on purpose -- these tests are about the
        # capture policy around the tracker, not about the window itself.
        window_s=0.25,
        max_translation_m=0.005,
        max_rotation_deg=0.5,
        cooldown_s=0.0,
    )
    solver._last_stillness = None
    solver._last_epoch_resets = 0
    solver._novelty_position_tol_m = 0.05
    solver._novelty_orientation_tol_deg = 5.0
    solver.applied = []
    solver._apply_update = lambda update, **kwargs: (
        solver.applied.append((update, kwargs)) or True
    )
    solver._clock = _Clock()
    solver.get_clock = lambda: solver._clock
    solver._logger = _Logger()
    solver.get_logger = lambda: solver._logger
    return solver


def hold(solver, times, *, position=(1.0, 2.0, 3.0)):
    for _ in range(times):
        solver._clock.seconds += 0.1
        solver._assisted_pair_callback((aruco_message(), board_message(position)))


def test_a_moving_board_is_never_captured():
    solver = assisted_harness()
    for index in range(10):
        solver._clock.seconds += 0.1
        solver._assisted_pair_callback(
            (aruco_message(), board_message(position=(0.05 * index, 0.0, 3.0)))
        )
    assert solver.detection_buffer.captures == []
    assert solver.applied == []
    assert solver._last_stillness is not None
    assert not solver._last_stillness.is_still


def test_a_still_board_is_captured_exactly_once_per_hold():
    solver = assisted_harness()
    hold(solver, 10)
    assert len(solver.detection_buffer.captures) == 1
    assert len(solver.applied) == 1
    assert solver.detection_buffer.removed == []


def test_a_second_placement_captures_again():
    solver = assisted_harness()
    hold(solver, 6, position=(1.0, 2.0, 3.0))
    hold(solver, 6, position=(2.0, 0.0, 4.0))
    assert len(solver.detection_buffer.captures) == 2


def test_a_repeated_placement_is_undone_rather_than_padding_the_buffer():
    solver = assisted_harness(novelty=(True, False))
    hold(solver, 6, position=(1.0, 2.0, 3.0))
    hold(solver, 6, position=(2.0, 0.0, 4.0))
    assert len(solver.detection_buffer.captures) == 2
    assert solver.detection_buffer.removed == [1], (
        "the second capture was not a new placement, so it must be undone"
    )
    assert len(solver.applied) == 1, "an undone capture must not be applied"
    assert [pair_id for pair_id, _, _ in solver._preview_store.captured] == [0]


def test_a_captured_pair_gets_a_preview_against_its_own_index():
    solver = assisted_harness()
    hold(solver, 6)
    assert len(solver._preview_store.captured) == 1
    pair_id, corners, reprojected = solver._preview_store.captured[0]
    assert pair_id == 0
    assert reprojected is None
    assert len(corners) == 1
    assert np.asarray(corners[0]).shape == (4, 2)


def test_a_closed_identity_gate_refuses_the_capture():
    solver = assisted_harness()
    solver.identity_gate = _Gate(error="LiDAR identity disagrees")
    hold(solver, 10)
    assert solver.detection_buffer.captures == []
    assert solver.applied == []


def test_a_missing_buffer_refuses_the_capture():
    solver = assisted_harness()
    solver.detection_buffer = None
    hold(solver, 10)
    assert solver.applied == []


def test_a_rejected_capture_is_reported_and_not_applied():
    solver = assisted_harness(accepted=False)
    hold(solver, 10)
    assert len(solver.detection_buffer.captures) == 1
    assert solver.applied == []
    assert solver._preview_store.captured == []


def test_an_unreadable_board_pose_is_ignored():
    solver = assisted_harness()
    for _ in range(10):
        solver._clock.seconds += 0.1
        solver._assisted_pair_callback(
            (aruco_message(), SimpleNamespace(detections=[]))
        )
    assert solver.detection_buffer.captures == []
    assert solver._last_stillness is None


def test_an_epoch_reset_rearms_the_tracker():
    solver = assisted_harness()
    hold(solver, 6)
    assert len(solver.detection_buffer.captures) == 1
    # The recording restarted underneath the synchronizer; the window it filled
    # belongs to the previous epoch and must not carry a "still" verdict across.
    solver.pair_source.epoch_resets = 1
    solver._clock.seconds += 0.1
    solver._assisted_pair_callback((aruco_message(), board_message()))
    assert solver._last_stillness.frames == 1
    assert not solver._last_stillness.is_still
    hold(solver, 6)
    assert len(solver.detection_buffer.captures) == 2, (
        "after a reset the same placement is a fresh hold, not a latched one"
    )


# --- NodeFacade ---------------------------------------------------------------
# The review server calls these from its own thread. What is worth pinning is
# that none of them raise: a failure has to arrive on the page as a reason an
# operator can read, not as a stack trace in a log nobody is watching.


def facade_harness(**kwargs) -> LidarToCameraSolver:
    solver = assisted_harness(**kwargs)
    parameters = {
        "review_archive_path": "/tmp/detections.json",
        "export_autoware_target": "",
        "export_camera_frame": "",
        "export_lidar_frame": "",
    }
    solver.parameters = parameters
    solver._string_parameter = lambda name: parameters[name]
    return solver


def test_state_is_json_shaped_before_anything_has_been_captured():
    solver = facade_harness()
    state = solver.state()
    assert state["mode"] == "assisted"
    assert state["sync"] == "sync: groups=12"
    assert state["identity_error"] is None
    assert state["stillness"] == {
        "is_still": False,
        "reason": "waiting for detections",
        "frames": 0,
    }
    assert state["diversity"]["n_placements"] == 0
    assert state["diversity"]["shortfalls"] == ["no placements yet"]
    assert state["solve"]["rms_px"] is None
    assert state["pairs"] == []
    assert state["export"] == {
        "archive_path": "/tmp/detections.json",
        "autoware_ready": False,
    }


def test_state_lists_one_entry_per_buffered_pair():
    solver = facade_harness()
    hold(solver, 6, position=(1.0, 2.0, 3.0))
    hold(solver, 6, position=(2.0, 0.0, 4.0))
    state = solver.state()
    assert [pair["id"] for pair in state["pairs"]] == [0, 1]
    assert all(pair["rms_px"] is None for pair in state["pairs"]), (
        "an unsolved buffer has no per-pose residuals to report"
    )
    assert all(pair["has_preview"] for pair in state["pairs"])
    assert state["stillness"]["is_still"]


def test_state_survives_having_no_camera_info():
    solver = facade_harness()
    solver.detection_buffer = None
    state = solver.state()
    assert state["solve"]["status"] == "No camera info available"
    assert state["pairs"] == []


def test_state_reports_a_closed_identity_gate():
    solver = facade_harness()
    solver.identity_gate = _Gate(error="camera identity disagrees")
    assert solver.state()["identity_error"] == "camera identity disagrees"


def test_drop_removes_the_pair_and_its_preview():
    solver = facade_harness()
    hold(solver, 6)
    ok, detail = solver.drop(0)
    assert ok is True
    assert "0" in detail
    assert solver.detection_buffer.removed == [0]
    assert solver._preview_store.dropped == [0]


def test_drop_without_a_buffer_reports_the_reason_instead_of_raising():
    solver = facade_harness()
    solver.detection_buffer = None
    assert solver.drop(0) == (False, "No camera info available")


def test_export_autoware_names_every_unset_parameter():
    solver = facade_harness()
    ok, detail, entry = solver.export_autoware(dry_run=True)
    assert ok is False
    assert entry is None
    for name in (
        "export_autoware_target",
        "export_camera_frame",
        "export_lidar_frame",
    ):
        assert name in detail


def test_export_autoware_refuses_without_a_solved_estimate():
    solver = facade_harness()
    solver.parameters.update(
        {
            "export_autoware_target": "/tmp/sensor_kit_calibration.yaml",
            "export_camera_frame": "camera0/camera_link",
            "export_lidar_frame": "velodyne_top_base_link",
        }
    )
    ok, detail, entry = solver.export_autoware(dry_run=True)
    assert (ok, entry) == (False, None)
    assert "no solved estimate" in detail


# --- the configurable novelty gate --------------------------------------------


def placement_at(position, normal=(0.0, 0.0, 1.0)) -> Placement:
    return Placement(position=position, normal=normal, frame_indices=(0,))


def test_an_empty_buffer_makes_every_pose_new():
    assert placement_is_new((1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0), ())


def test_a_pose_inside_both_tolerances_is_not_new():
    assert not placement_is_new(
        (1.0, 2.0, 3.0),
        (0.0, 0.0, 0.0, 1.0),
        (placement_at((1.02, 2.0, 3.0)),),
    )


def test_moving_far_enough_makes_a_pose_new_again():
    assert placement_is_new(
        (1.0, 2.0, 3.0),
        (0.0, 0.0, 0.0, 1.0),
        (placement_at((1.5, 2.0, 3.0)),),
    )


def test_tilting_far_enough_makes_a_pose_new_without_moving():
    # 30 deg about x: same position, a plainly different board orientation.
    half = np.radians(30.0) / 2.0
    tilted = (float(np.sin(half)), 0.0, 0.0, float(np.cos(half)))
    assert placement_is_new((1.0, 2.0, 3.0), tilted, (placement_at((1.0, 2.0, 3.0)),))


def test_a_looser_configured_tolerance_widens_what_counts_as_the_same_placement():
    same = (placement_at((1.3, 2.0, 3.0)),)
    assert placement_is_new((1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0), same)
    assert not placement_is_new(
        (1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0), same, position_tol_m=0.5
    )


def test_a_pose_matching_an_existing_placement_is_never_captured():
    solver = assisted_harness(placements=(placement_at((1.0, 2.0, 3.0)),))
    hold(solver, 10, position=(1.0, 2.0, 3.0))
    assert solver.detection_buffer.captures == [], (
        "the buffer already holds this placement; capturing it again adds no geometry"
    )
    assert solver._last_stillness.is_still


def test_the_configured_tolerance_is_what_the_capture_gate_uses():
    solver = assisted_harness(placements=(placement_at((1.3, 2.0, 3.0)),))
    solver._novelty_position_tol_m = 0.5
    hold(solver, 10, position=(1.0, 2.0, 3.0))
    assert solver.detection_buffer.captures == []
