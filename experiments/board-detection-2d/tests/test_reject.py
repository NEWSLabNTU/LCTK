import pytest

from boarddet.reject import RejectReason, Stage, band, furthest, lower, upper


def test_stage_bands_order():
    # generation < scorer < detector, monotonic within a path
    assert Stage.PATCH_EXTENT < Stage.MIN_POINTS
    assert Stage.SIDE_ERR < Stage.SQUARE_FIT
    assert Stage.MIN_POINTS < Stage.MIN_SCORE < Stage.ISOLATION


def test_upper_margin():
    r = upper(Stage.PATCH_FLATNESS, "flatness", "flatness_rms_max", 0.07, 0.035)
    assert r.stage is Stage.PATCH_FLATNESS
    assert r.param == "flatness_rms_max"
    assert r.value == 0.07
    assert r.threshold == 0.035
    assert r.margin == pytest.approx(1.0)  # (0.07-0.035)/0.035


def test_lower_margin():
    r = lower(Stage.MIN_SCORE, "min_score", "min_score", 0.4, 0.5)
    assert r.margin == pytest.approx(0.2)  # (0.5-0.4)/0.5


def test_band_margin():
    r = band(Stage.PATCH_EXTENT, "extent", None, 0.2, 0.5, 2.5)
    # dist_outside = 0.5-0.2 = 0.3 ; half-width = (2.5-0.5)/2 = 1.0
    assert r.margin == pytest.approx(0.3)
    assert r.param is None


def test_margin_zero_threshold_guard():
    r = lower(Stage.STANCE_2D, "stance", "stance_floor", 0.0, 0.0)
    assert r.margin == 0.0


def test_furthest_picks_max_stage_first_on_tie():
    a = upper(Stage.PATCH_FLATNESS, "flatness", "flatness_rms_max", 0.07, 0.035)
    b = lower(Stage.MIN_SCORE, "min_score", "min_score", 0.4, 0.5)
    c = lower(Stage.MIN_SCORE, "min_score", "min_score", 0.1, 0.5)
    assert furthest([a, b, c]) is b   # max stage 22, first of the two
    assert furthest([]) is None


def test_frozen():
    r = upper(Stage.SIZE_GATE, "size", "side_tol", 2.0, 1.0)
    with pytest.raises(Exception):
        r.stage = Stage.MIN_POINTS  # frozen
