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


def test_live_and_capture_revisions_are_independent():
    model = ReviewReadModel(session_epoch="epoch")

    assert model.observe_live(("moving", "sync: 0")) == 1
    assert model.revisions().captures_revision == 0
    assert model.observe_capture(7) == 1
    assert model.revisions().captures_revision == 1

    # A stillness heartbeat never invalidates the detection list.
    assert model.observe_live(("still", "sync: 1")) == 2
    assert model.revisions().captures_revision == 1


def test_evidence_advances_the_aggregate_capture_revision_only():
    model = ReviewReadModel(session_epoch="epoch")
    model.observe_capture(7)
    before = model.revisions()

    model.observe_evidence({1: (True, False)})
    after = model.revisions()

    assert after.capture_revision == before.capture_revision
    assert after.captures_revision == before.captures_revision + 1
    assert after.live_revision == before.live_revision


def test_revision_listener_receives_compact_vectors_and_can_unsubscribe():
    model = ReviewReadModel(session_epoch="epoch")
    events = []
    remove = model.add_listener(events.append)

    model.mark_live_changed()
    assert events[-1].event_vector() == {
        "session_epoch": "epoch",
        "live_revision": 1,
        "captures_revision": 0,
        "scene_revision": 0,
    }

    remove()
    model.mark_captures_changed()
    assert len(events) == 1


def test_explicit_marks_are_consumed_by_the_next_observation():
    model = ReviewReadModel(session_epoch="epoch")
    events = []
    model.add_listener(events.append)
    model.mark_capture_changed()

    # The mutation callback and the later read observe one buffer change, not
    # two changes caused by the same mutation.
    assert model.observe_capture(7) == 1
    assert model.revisions().captures_revision == 1
    assert len(events) == 1


def test_combined_capture_observation_does_not_double_publish_one_request():
    model = ReviewReadModel(session_epoch="epoch")
    events = []
    model.add_listener(events.append)
    model.mark_capture_changed()
    model.mark_captures_changed()

    capture_revision, evidence_revisions = model.observe_captures(7, {1: (True, False)})

    assert capture_revision == 1
    assert evidence_revisions == {1: 1}
    assert model.revisions().captures_revision == 2
    # Both explicit mutation callbacks already published the two aggregate
    # changes; the coherent read only consumes their dirty markers.
    assert len(events) == 2


def test_repeated_marks_are_not_followed_by_a_spurious_lazy_revision():
    model = ReviewReadModel(session_epoch="epoch")
    model.observe_capture(1)
    before = model.revisions().captures_revision
    model.mark_capture_changed()
    model.mark_capture_changed()

    model.observe_capture(2)

    assert model.revisions().captures_revision == before + 2


def test_failed_revision_listener_cannot_break_a_mutation():
    model = ReviewReadModel(session_epoch="epoch")

    def fail(_snapshot):
        raise OSError("disconnected browser")

    model.add_listener(fail)
    assert model.mark_live_changed() == 1
