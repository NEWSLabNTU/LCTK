"""StillnessTracker: is the board being held still enough to capture?

The gate is a span over a sliding window, not a frame-to-frame delta. A board
drifting steadily at 1 mm per frame has a tiny per-frame delta and is not still;
only the span across the whole window sees it.
"""

import math
from dataclasses import FrozenInstanceError

import pytest
from lidar_to_camera_solver.stability import StillnessState, StillnessTracker

IDENTITY = (0.0, 0.0, 0.0, 1.0)


def make_tracker(**overrides):
    kwargs = {
        "window_frames": 5,
        "max_translation_m": 0.005,
        "max_rotation_deg": 0.5,
        "cooldown_s": 1.0,
    }
    kwargs.update(overrides)
    return StillnessTracker(**kwargs)


def quaternion_about_z(degrees):
    half = math.radians(degrees) / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


def test_window_must_fill_before_any_verdict():
    tracker = make_tracker()
    for index in range(4):
        state = tracker.push((0.0, 0.0, 0.0), IDENTITY, float(index))
        assert not state.is_still
        assert state.frames == index + 1
        assert "filling" in state.reason


def test_a_perfectly_still_board_captures_once():
    tracker = make_tracker()
    states = [tracker.push((1.0, 2.0, 3.0), IDENTITY, float(i)) for i in range(10)]
    captured = [s for s in states if s.should_capture]
    assert len(captured) == 1, "a single uninterrupted hold must capture exactly once"
    assert captured[0].is_still
    assert captured[0].translation_span_m == pytest.approx(0.0)


def test_a_steadily_drifting_board_never_captures():
    tracker = make_tracker()
    # 2 mm per frame: each frame delta is under the 5 mm gate, the window span is not.
    states = [
        tracker.push((0.002 * i, 0.0, 0.0), IDENTITY, float(i)) for i in range(20)
    ]
    assert not any(s.should_capture for s in states)
    assert not any(s.is_still for s in states)


def test_translation_span_is_measured_across_the_window():
    tracker = make_tracker(max_translation_m=1.0)
    for index in range(4):
        tracker.push((0.0, 0.0, 0.0), IDENTITY, float(index))
    state = tracker.push((0.3, 0.4, 0.0), IDENTITY, 4.0)
    assert state.translation_span_m == pytest.approx(0.5)


def test_rotation_span_is_measured_across_the_window():
    tracker = make_tracker(max_rotation_deg=90.0)
    for index in range(4):
        tracker.push((0.0, 0.0, 0.0), IDENTITY, float(index))
    state = tracker.push((0.0, 0.0, 0.0), quaternion_about_z(30.0), 4.0)
    assert state.rotation_span_deg == pytest.approx(30.0, abs=1e-6)


def test_rotation_alone_breaks_stillness():
    tracker = make_tracker()
    states = [
        tracker.push((0.0, 0.0, 0.0), quaternion_about_z(1.0 * i), float(i))
        for i in range(20)
    ]
    assert not any(s.should_capture for s in states)


def test_exactly_at_the_tolerance_is_still():
    tracker = make_tracker(max_translation_m=0.005)
    for index in range(4):
        tracker.push((0.0, 0.0, 0.0), IDENTITY, float(index))
    state = tracker.push((0.005, 0.0, 0.0), IDENTITY, 4.0)
    assert state.is_still, "the tolerance is inclusive; 5 mm with a 5 mm gate is still"


def test_capture_re_arms_only_after_the_board_moves():
    tracker = make_tracker(cooldown_s=0.0)
    for index in range(10):
        tracker.push((0.0, 0.0, 0.0), IDENTITY, float(index))
    # Move far enough to break the window, then settle again.
    for index in range(10, 20):
        tracker.push((5.0, 0.0, 0.0), IDENTITY, float(index))
    states = [tracker.push((5.0, 0.0, 0.0), IDENTITY, float(i)) for i in range(20, 30)]
    assert not any(s.should_capture for s in states), (
        "the second hold already captured while settling; it must not capture again"
    )


def test_cooldown_suppresses_a_second_capture():
    tracker = make_tracker(cooldown_s=10.0)
    for index in range(10):
        tracker.push((0.0, 0.0, 0.0), IDENTITY, float(index))
    for index in range(10, 20):
        tracker.push((5.0, 0.0, 0.0), IDENTITY, float(index))
    states = [tracker.push((5.0, 0.0, 0.0), IDENTITY, float(i)) for i in range(20, 30)]
    assert not any(s.should_capture for s in states)


def test_reason_names_the_measurement_when_not_still():
    tracker = make_tracker()
    for index in range(5):
        tracker.push((0.02 * index, 0.0, 0.0), IDENTITY, float(index))
    state = tracker.push((0.2, 0.0, 0.0), IDENTITY, 5.0)
    assert "mm" in state.reason
    assert str(state.frames) in state.reason


def test_reset_clears_the_window():
    tracker = make_tracker()
    for index in range(10):
        tracker.push((0.0, 0.0, 0.0), IDENTITY, float(index))
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
