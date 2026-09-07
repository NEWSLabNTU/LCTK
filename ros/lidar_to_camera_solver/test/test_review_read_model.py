"""Revision and projection-cache contracts for assisted review."""

from lidar_to_camera_solver.review_read_model import ReviewReadModel


def test_capture_projection_is_cached_until_buffer_revision_changes():
    model = ReviewReadModel(session_epoch="epoch")
    calls = []

    model.observe_capture(7)
    assert model.cached((7,), lambda: calls.append(1) or {"value": 1}) == {"value": 1}
    assert model.cached((7,), lambda: calls.append(1) or {"value": 2}) == {"value": 1}
    model.observe_capture(8)
    assert model.cached((8,), lambda: calls.append(1) or {"value": 3}) == {"value": 3}
    assert len(calls) == 2


def test_evidence_revision_advances_only_the_changed_capture():
    model = ReviewReadModel(session_epoch="epoch")
    first = model.observe_evidence({1: (True, False), 2: (False, True)})
    second = model.observe_evidence({1: (True, True), 2: (False, True)})

    assert first == {1: 1, 2: 1}
    assert second == {1: 2, 2: 1}


def test_state_revision_is_idempotent_for_unchanged_heartbeat():
    model = ReviewReadModel(session_epoch="epoch")
    assert model.observe_state(("still", "sync")) == 1
    assert model.observe_state(("still", "sync")) == 1
    assert model.observe_state(("moving", "sync")) == 2


def test_replacement_buffer_at_same_revision_invalidates_quality():
    model = ReviewReadModel(session_epoch="epoch")
    original_buffer = object()
    replacement = object()
    before = model.observe_capture((original_buffer, 0))
    after = model.observe_capture((replacement, 0))
    assert after > before


def test_evidence_only_changes_replace_the_cached_projection_key():
    model = ReviewReadModel(session_epoch="epoch")
    assert model.cached((1, 1), lambda: "missing") == "missing"
    assert model.cached((1, 2), lambda: "complete") == "complete"
    assert model.cached((1, 2), lambda: "wrong") == "complete"
