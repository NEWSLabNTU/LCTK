"""H-07: the buffer must count DISTINCT board placements, not frames.

The old buffer accepted anything and reported success in the operator's own language:

    "Added detection pair and solved calibration successfully
     (320 correspondences from 20 poses)"

Both numbers are the lies H-09 disproved. Twenty frames of a board that never moved are ONE
placement — they cannot determine the extrinsic, however many correspondences they contribute. An
operator could hit Add twenty times, be congratulated twenty times, and end up with a degenerate
calibration.

These tests pin the counting. `_count_placements` is what the Add handler uses to tell the operator,
in the moment, whether they just contributed geometry or a duplicate.
"""

import numpy as np
import pytest
from lidar_to_camera_solver.main import LidarToCameraSolver as S
from scipy.spatial.transform import Rotation
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose


def board_msg(position, yaw=0.0, pitch=0.0):
    """A Detection3DArray carrying one board pose, as lidar_board_detector publishes."""
    msg = Detection3DArray()
    det = Detection3D()
    result = ObjectHypothesisWithPose()

    result.pose.pose.position.x = float(position[0])
    result.pose.pose.position.y = float(position[1])
    result.pose.pose.position.z = float(position[2])

    q = Rotation.from_euler("xyz", [0.0, pitch, yaw]).as_quat()
    result.pose.pose.orientation.x = float(q[0])
    result.pose.pose.orientation.y = float(q[1])
    result.pose.pose.orientation.z = float(q[2])
    result.pose.pose.orientation.w = float(q[3])

    det.results.append(result)
    msg.detections.append(det)
    return msg


class FakeSolver:
    """Just the buffer and the counting method — no node, no ROS spin."""

    _count_placements = S._count_placements

    def __init__(self, board_msgs):
        self.detection_buffer = [(None, b) for b in board_msgs]


def test_static_board_is_one_placement_however_many_frames():
    """The headline. Twenty frames of an unmoved board are ONE placement."""
    rng = np.random.default_rng(0)
    # A real static board still jitters by a centimetre or two of ICP noise.
    frames = [
        board_msg(
            (2.6 + rng.normal(0, 0.005), rng.normal(0, 0.005), rng.normal(0, 0.005))
        )
        for _ in range(20)
    ]

    solver = FakeSolver(frames)

    assert len(solver.detection_buffer) == 20
    assert solver._count_placements() == 1, (
        "twenty frames of a board that never moved were counted as more than one placement; "
        "the operator would be told they had collected geometry they do not have"
    )


def test_moving_the_board_creates_a_new_placement():
    frames = [
        board_msg((2.6, 0.0, 0.0)),
        board_msg((2.6, 0.0, 0.0)),  # duplicate
        board_msg((1.8, 0.9, 0.3), yaw=0.5),  # moved
        board_msg((3.4, -0.8, -0.2), yaw=-0.6, pitch=0.4),  # moved again
    ]

    solver = FakeSolver(frames)

    assert solver._count_placements() == 3


def test_tilting_the_board_in_place_is_a_new_placement():
    """Changing the board's TILT changes its plane normal, so it is new geometry."""
    frames = [
        board_msg((2.6, 0.0, 0.0), pitch=0.0),
        board_msg((2.6, 0.0, 0.0), pitch=0.6),
    ]

    assert FakeSolver(frames)._count_placements() == 2


def test_spinning_the_board_in_its_own_plane_is_NOT_a_new_placement():
    """Deliberate: a board spun about its own normal is the same placement.

    Yaw here is rotation about the board's normal axis, so the plane, the depth and the centroid
    are all unchanged. The ArUco corners move *within* the plane, but the geometry that constrains
    the extrinsic — where the board plane is and which way it faces — does not. It contributes
    nothing against the near-null direction of H-07, so counting it as new would overstate how
    well-constrained the capture is, which is the whole failure mode this counting exists to
    prevent.
    """
    frames = [
        board_msg((2.6, 0.0, 0.0), yaw=0.0),
        board_msg((2.6, 0.0, 0.0), yaw=0.6),
    ]

    assert FakeSolver(frames)._count_placements() == 1


def test_exclude_last_lets_add_detection_detect_a_duplicate():
    """The Add handler compares the count before and after to say 'new' or 'duplicate'."""
    frames = [board_msg((2.6, 0.0, 0.0)), board_msg((2.6, 0.0, 0.0))]
    solver = FakeSolver(frames)

    before = solver._count_placements(exclude_last=True)
    after = solver._count_placements()

    assert before == 1 and after == 1, (
        "a duplicate must not increase the placement count"
    )

    solver = FakeSolver(
        [board_msg((2.6, 0.0, 0.0)), board_msg((1.7, 1.0, 0.4), yaw=0.7)]
    )
    assert solver._count_placements(exclude_last=True) == 1
    assert solver._count_placements() == 2


def test_empty_buffer_counts_zero():
    assert FakeSolver([])._count_placements() == 0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
