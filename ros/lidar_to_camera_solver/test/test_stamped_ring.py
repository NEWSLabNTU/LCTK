"""Public timestamp matching behavior for the assisted evidence rings."""

import pytest
from lidar_to_camera_solver.stamped_ring import StampedRing


def make_ring(**overrides):
    values = {
        "max_age_s": 1.0,
        "tolerance_s": 0.05,
        "max_items": 8,
    }
    values.update(overrides)
    return StampedRing(**values)


def test_exact_stamp_wins_over_a_nearer_candidate():
    ring = make_ring()
    ring.put(10.00, "exact")
    ring.put(10.03, "near")

    assert ring.match(10.00) == "exact"


def test_nearest_stamp_is_used_only_inside_the_tolerance():
    ring = make_ring(tolerance_s=0.02)
    ring.put(10.00, "sample")

    assert ring.match(10.015) == "sample"
    assert ring.match(10.021) is None


def test_time_bound_evicts_old_entries():
    ring = make_ring(max_age_s=1.0, tolerance_s=0.0)
    ring.put(1.0, "old")
    ring.put(1.5, "kept")
    ring.put(2.1, "new")

    assert ring.match(1.0) is None
    assert ring.match(1.5) == "kept"
    assert ring.match(2.1) == "new"


def test_count_cap_evicts_the_oldest_even_with_a_long_time_bound():
    ring = make_ring(max_age_s=100.0, tolerance_s=0.0, max_items=2)
    for stamp in (1.0, 2.0, 3.0):
        ring.put(stamp, stamp)

    assert len(ring) == 2
    assert ring.match(1.0) is None
    assert ring.match(2.0) == 2.0
    assert ring.match(3.0) == 3.0


def test_non_monotonic_stamp_starts_a_new_epoch():
    ring = make_ring(tolerance_s=0.0)
    ring.put(5.0, "previous recording")
    previous_epoch = ring.epoch
    ring.put(1.0, "new recording")

    assert ring.epoch == previous_epoch + 1
    assert ring.match(5.0) is None
    assert ring.match(1.0) == "new recording"


def test_invalid_stamp_clears_the_ring_without_raising():
    ring = make_ring()
    ring.put(1.0, "sample")

    ring.put(10**1000, "invalid")

    assert len(ring) == 0
    assert ring.match(1.0) is None


def test_duplicate_stamp_replaces_the_previous_value_without_bypassing_cap():
    ring = make_ring(max_items=2, tolerance_s=0.0)
    ring.put(1.0, "first")
    ring.put(1.0, "replacement")
    ring.put(2.0, "second")

    assert len(ring) == 2
    assert ring.match(1.0) == "replacement"


def test_exact_match_helper_does_not_borrow_a_neighbor():
    ring = make_ring(tolerance_s=0.1)
    ring.put(2.0, "nearby")

    assert ring.match_exact(2.05) is None
    assert ring.match_exact(2.0) == "nearby"


@pytest.mark.parametrize(
    "kwargs",
    [
        {"max_age_s": 0.0},
        {"max_age_s": float("inf")},
        {"tolerance_s": -0.1},
        {"tolerance_s": float("nan")},
        {"max_items": 0},
    ],
)
def test_ring_bounds_are_validated(kwargs):
    with pytest.raises(ValueError):
        make_ring(**kwargs)
