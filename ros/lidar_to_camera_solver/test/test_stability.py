"""StillnessTracker: is the board being held still enough to capture?

Two properties are pinned here, and both have been wrong in this file's history.

The gate is a span over a sliding window, not a frame-to-frame delta. A board
drifting steadily at 1 mm per frame has a tiny per-frame delta and is not still;
only the span across the whole window sees it.

The window is a **duration**, not a count of detection pairs. Pairs arrive
irregularly -- the board leaves the field of view, an ICP fit is rejected, a
sweep returns too few points -- so ten consecutive pairs can span half a second
or nineteen. Measured on `sessions/solid600-handheld-vlp`'s 58 s recording, a
ten-pair window ran from 0.48 s to 19.42 s, median 1.30 s, with 71 of 195
windows shorter than one second. A frame count therefore says nothing about how
long the board actually held.
"""

import math
from dataclasses import FrozenInstanceError

import pytest
from lidar_to_camera_solver.stability import StillnessState, StillnessTracker

IDENTITY = (0.0, 0.0, 0.0, 1.0)


def make_tracker(**overrides):
    kwargs = {
        "window_s": 1.0,
        "max_translation_m": 0.005,
        "max_rotation_deg": 0.5,
        "cooldown_s": 1.0,
    }
    kwargs.update(overrides)
    return StillnessTracker(**kwargs)


def quaternion_about_z(degrees):
    half = math.radians(degrees) / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


def push_stream(tracker, count, *, start=0.0, step=0.1, position=(0.0, 0.0, 0.0)):
    """Push `count` samples of one fixed pose, `step` seconds apart."""
    return [
        tracker.push(position, IDENTITY, start + step * index) for index in range(count)
    ]


# --- the window is a duration -------------------------------------------------


def test_window_must_cover_the_duration_before_any_verdict():
    tracker = make_tracker()
    states = push_stream(tracker, 10, step=0.1)  # 0.0 s .. 0.9 s
    assert not any(state.is_still for state in states)
    assert all("filling" in state.reason for state in states)
    # The reason names the measured coverage, not a frame count.
    assert "0.90/1.00 s" in states[-1].reason
    # One more sample crosses the second and a verdict becomes possible.
    final = tracker.push((0.0, 0.0, 0.0), IDENTITY, 1.0)
    assert final.is_still


def test_ten_pairs_spanning_eight_seconds_are_not_still():
    """The bug this window change exists to fix.

    Ten pairs 0.8 s apart fill any ten-frame window completely, and the board is
    perfectly stationary in all of them, so a frame-count window calls this still
    and captures. It is eight seconds of a board that was seen ten times, which
    is not evidence of a one-second hold.
    """
    tracker = make_tracker()
    states = push_stream(tracker, 10, step=0.8)
    assert not any(state.is_still for state in states)
    assert not any(state.should_capture for state in states)


def test_a_sparse_stream_cannot_satisfy_the_window_with_two_samples():
    """Two samples one second apart span the window but do not evidence it."""
    tracker = make_tracker()
    states = push_stream(tracker, 4, step=1.0)
    assert not any(state.is_still for state in states)
    assert any("too few" in state.reason for state in states)
    late = states[-1]
    assert str(late.frames) in late.reason, "the reason names the measured count"
    assert "too few" in late.reason


def test_min_samples_is_reached_by_a_denser_stream():
    tracker = make_tracker()
    # 0.00, 0.34, 0.68, 1.02: the 0.00 sample brackets the window and the other
    # three sit inside it, which is the floor exactly.
    states = push_stream(tracker, 4, step=0.34)
    assert states[-1].is_still
    assert states[-1].frames == 3


def test_min_samples_floor_is_configurable():
    tracker = make_tracker(min_samples=6)
    states = push_stream(tracker, 3, step=0.5)
    assert not states[-1].is_still
    assert "too few" in states[-1].reason
    assert "6" in states[-1].reason, "the reason names the floor the operator set"


def test_samples_older_than_the_window_leave_it():
    tracker = make_tracker(max_translation_m=1.0)
    # A far-away sample well outside the window must not enter the span.
    tracker.push((10.0, 0.0, 0.0), IDENTITY, 0.0)
    states = push_stream(tracker, 30, start=5.0, step=0.1)
    assert states[-1].translation_span_m == pytest.approx(0.0), (
        "the 10 m sample aged out of the window and must not count"
    )


# --- spans, not deltas (unchanged semantics) ----------------------------------


def test_a_perfectly_still_board_captures_once():
    tracker = make_tracker()
    states = push_stream(tracker, 40, step=0.1, position=(1.0, 2.0, 3.0))
    captured = [s for s in states if s.should_capture]
    assert len(captured) == 1, "a single uninterrupted hold must capture exactly once"
    assert captured[0].is_still
    assert captured[0].translation_span_m == pytest.approx(0.0)


def test_a_steadily_drifting_board_never_captures():
    tracker = make_tracker()
    # 2 mm per sample: each delta is under the 5 mm gate, the window span is not.
    states = [tracker.push((0.002 * i, 0.0, 0.0), IDENTITY, 0.1 * i) for i in range(40)]
    assert not any(s.should_capture for s in states)
    assert not any(s.is_still for s in states)


def test_translation_span_is_measured_across_the_window():
    tracker = make_tracker(max_translation_m=1.0)
    push_stream(tracker, 10, step=0.1)
    state = tracker.push((0.3, 0.4, 0.0), IDENTITY, 1.0)
    assert state.translation_span_m == pytest.approx(0.5)


def test_rotation_span_is_measured_across_the_window():
    tracker = make_tracker(max_rotation_deg=90.0)
    push_stream(tracker, 10, step=0.1)
    state = tracker.push((0.0, 0.0, 0.0), quaternion_about_z(30.0), 1.0)
    assert state.rotation_span_deg == pytest.approx(30.0, abs=1e-6)


def test_rotation_alone_breaks_stillness():
    tracker = make_tracker()
    states = [
        tracker.push((0.0, 0.0, 0.0), quaternion_about_z(0.1 * i), 0.1 * i)
        for i in range(40)
    ]
    assert not any(s.should_capture for s in states)


def test_exactly_at_the_tolerance_is_still():
    tracker = make_tracker(max_translation_m=0.005)
    push_stream(tracker, 10, step=0.1)
    state = tracker.push((0.005, 0.0, 0.0), IDENTITY, 1.0)
    assert state.is_still, "the tolerance is inclusive; 5 mm with a 5 mm gate is still"


# --- the capture latch and the cooldown ---------------------------------------


def test_capture_re_arms_only_after_the_board_moves():
    tracker = make_tracker(cooldown_s=0.0)
    push_stream(tracker, 30, start=0.0, step=0.1)
    # Move far enough to break the window, then settle again.
    push_stream(tracker, 30, start=3.0, step=0.1, position=(5.0, 0.0, 0.0))
    states = push_stream(tracker, 30, start=6.0, step=0.1, position=(5.0, 0.0, 0.0))
    assert not any(s.should_capture for s in states), (
        "the second hold already captured while settling; it must not capture again"
    )


def test_cooldown_suppresses_a_second_capture():
    tracker = make_tracker(cooldown_s=100.0)
    push_stream(tracker, 30, start=0.0, step=0.1)
    push_stream(tracker, 30, start=3.0, step=0.1, position=(5.0, 0.0, 0.0))
    states = push_stream(tracker, 30, start=6.0, step=0.1, position=(5.0, 0.0, 0.0))
    assert not any(s.should_capture for s in states)


# --- reasons ------------------------------------------------------------------


def test_reason_names_the_measurement_when_not_still():
    tracker = make_tracker()
    for index in range(11):
        tracker.push((0.02 * index, 0.0, 0.0), IDENTITY, 0.1 * index)
    state = tracker.push((0.2, 0.0, 0.0), IDENTITY, 1.1)
    assert "mm" in state.reason
    assert "deg" in state.reason
    assert str(state.frames) in state.reason


# --- clocks that misbehave ----------------------------------------------------


def test_time_going_backwards_resets_the_window():
    """A `--clock` restart or a replay loop must not carry a verdict across."""
    tracker = make_tracker()
    states = push_stream(tracker, 30, step=0.1)
    assert states[-1].is_still
    rewound = tracker.push((0.0, 0.0, 0.0), IDENTITY, 0.0)
    assert not rewound.is_still
    assert rewound.frames == 1, "the pre-rewind samples belong to an abandoned timeline"
    assert "filling" in rewound.reason


def test_time_going_backwards_does_not_strand_the_cooldown():
    """The cooldown anchor lives in the abandoned timeline too.

    Keeping it would compare a new, smaller stamp against an old, larger one and
    suppress every capture until the new clock caught up.
    """
    tracker = make_tracker(cooldown_s=5.0)
    captured = [s for s in push_stream(tracker, 30, step=0.1) if s.should_capture]
    assert len(captured) == 1
    # Rewind far behind the capture stamp, then hold still again.
    states = push_stream(tracker, 30, start=-100.0, step=0.1)
    assert any(s.should_capture for s in states)


def test_repeated_stamps_never_manufacture_a_verdict():
    """Twenty samples at one instant are zero seconds of evidence."""
    tracker = make_tracker()
    states = [tracker.push((0.0, 0.0, 0.0), IDENTITY, 5.0) for _ in range(20)]
    assert not any(s.is_still for s in states)
    assert all("filling" in s.reason for s in states)


# --- construction -------------------------------------------------------------


@pytest.mark.parametrize("window_s", [0.0, -1.0, float("inf"), float("nan")])
def test_window_s_must_be_finite_and_positive(window_s):
    with pytest.raises(ValueError, match="window_s"):
        make_tracker(window_s=window_s)


def test_min_samples_must_allow_a_span():
    with pytest.raises(ValueError, match="min_samples"):
        make_tracker(min_samples=1)


# --- housekeeping -------------------------------------------------------------


def test_reset_clears_the_window():
    tracker = make_tracker()
    push_stream(tracker, 30, step=0.1)
    tracker.reset()
    state = tracker.push((0.0, 0.0, 0.0), IDENTITY, 20.0)
    assert state.frames == 1
    assert not state.is_still


def test_state_is_frozen():
    state = StillnessState(
        is_still=True,
        should_capture=False,
        translation_span_m=0.0,
        rotation_span_deg=0.0,
        frames=5,
        reason="",
    )
    with pytest.raises(FrozenInstanceError):
        state.is_still = False
