"""The board-frame convention guard.

`lidar_board_detector` publishes the convention its poses are expressed in on the
latched topic `/lctk/board_frame_convention`. The solver supplies board-local marker
coordinates to those poses, so a disagreement is not merely undetected — it is
*undetectable* by the quality metric an operator would consult: half the failure is a
45-degree in-plane rotation, which the symmetric 2x2 marker grid solves cleanly.

The decision is a pure function over the received string so that the whole table —
match, mismatch, absent — is testable without constructing a node. Absence is the case
that matters most: a solver started before any detector, or after the bag ended and the
detector exited, receives nothing, and treating that as consent would make the guard
useless exactly when it is needed.
"""

import pytest
from lidar_to_camera_solver.board_geometry import (
    BOARD_FRAME_CONVENTION,
    BOARD_FRAME_CONVENTION_TOPIC,
    frame_convention_error,
)


def test_the_expected_convention_is_the_phase_1_identifier():
    """Pinned to the string `lidar_board_detector` publishes (main.rs). If the geometry
    below it changes meaning, this string changes with it — that is the point of it."""
    assert BOARD_FRAME_CONVENTION == "corner_aligned_plate_center_v1"
    assert BOARD_FRAME_CONVENTION_TOPIC == "/lctk/board_frame_convention"


def test_matching_convention_passes():
    assert frame_convention_error(BOARD_FRAME_CONVENTION) is None


def test_mismatching_convention_names_both_sides():
    """An operator must learn what to change, not merely that something failed."""
    message = frame_convention_error("edge_aligned_corner_origin_v0")

    assert message is not None
    assert "edge_aligned_corner_origin_v0" in message
    assert BOARD_FRAME_CONVENTION in message


def test_absent_convention_fails():
    """Absence is failure, not consent."""
    message = frame_convention_error(None)

    assert message is not None
    assert BOARD_FRAME_CONVENTION_TOPIC in message


@pytest.mark.parametrize("received", ["", "   "])
def test_blank_convention_fails(received):
    """An empty tag is not a match, and must not be mistaken for one."""
    assert frame_convention_error(received) is not None


def test_surrounding_whitespace_is_tolerated():
    """The tag travels as a std_msgs/String; trailing whitespace is not a disagreement
    about geometry."""
    assert frame_convention_error(f"  {BOARD_FRAME_CONVENTION}\n") is None
