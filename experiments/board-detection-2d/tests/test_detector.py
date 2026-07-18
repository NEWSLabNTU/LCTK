import numpy as np
import pytest
from boarddet.board_config import BoardConfig
from boarddet.detector import GENERATORS, _stance, _up_2d, detect
from boarddet.geometry import PlaneModel
from boarddet.synth import make_scene


@pytest.mark.parametrize("gen", list(GENERATORS))
def test_detects_board_in_synthetic_scene(gen):
    pts, truth = make_scene(rng=np.random.default_rng(13))
    out = detect(pts, BoardConfig(side_m=1.0), generator=gen)
    assert out.detection is not None, f"generator {gen} found nothing"
    assert np.linalg.norm(out.detection.center - truth.center) < 0.05
    assert abs(out.detection.rotation[:, 2] @ truth.normal) > 0.99
    assert out.timings_ms["total"] > 0


@pytest.mark.parametrize("gen", list(GENERATORS))
def test_no_detection_in_boardless_scene(gen):
    rng = np.random.default_rng(14)
    pts, _ = make_scene(rng=rng)
    # strip points near the board plane region entirely: keep clutter only
    keep = pts[:, 0] < 2.0
    out = detect(pts[keep], BoardConfig(side_m=1.0), generator=gen)
    assert out.detection is None


def test_stance_diamond_beats_axis_aligned():
    # Diamond standing on a corner: one diagonal gravity (z) aligned.
    diamond = np.array([
        [0.0, 0.0, 1.0],   # top (gravity-aligned corner)
        [1.0, 0.0, 0.0],   # right
        [0.0, 0.0, -1.0],  # bottom (gravity-aligned corner)
        [-1.0, 0.0, 0.0],  # left
    ])
    # Axis-aligned square panel (upright, sides horizontal/vertical): both
    # diagonals sit at ~45 deg off gravity.
    flat = np.array([
        [0.5, 0.0, 0.5],    # top-right
        [-0.5, 0.0, 0.5],   # top-left
        [-0.5, 0.0, -0.5],  # bottom-left
        [0.5, 0.0, -0.5],   # bottom-right
    ])
    assert _stance(flat) < _stance(diamond)
    assert _stance(diamond) > 0.99
    assert 0.6 < _stance(flat) < 0.8


def test_detect_with_stance_weight_still_finds_board():
    pts, truth = make_scene(rng=np.random.default_rng(13))
    out = detect(pts, BoardConfig(side_m=1.0, stance_weight=0.5),
                 generator="a")
    assert out.detection is not None
    assert np.isfinite(out.detection.score)
    assert np.linalg.norm(out.detection.center - truth.center) < 0.05


def test_best_rejected_populated_when_min_score_too_high():
    pts, _ = make_scene(rng=np.random.default_rng(13))
    out = detect(pts, BoardConfig(side_m=1.0, min_score=0.99), generator="a")
    assert out.detection is None
    assert out.best_rejected is not None
    assert out.best_rejected.score < 0.99


def test_best_rejected_none_when_detection_accepted():
    pts, _ = make_scene(rng=np.random.default_rng(13))
    out = detect(pts, BoardConfig(side_m=1.0), generator="a")
    assert out.detection is not None
    assert out.best_rejected is None


def test_up_2d_none_for_near_horizontal_plane():
    """A near-horizontal plane (normal ~= world z) has u, v spanning a
    near-horizontal patch, so world +z projects to ~zero in-plane -- no
    privileged "up" stripe direction to rotate onto. _up_2d must signal
    this (None) so the detector falls back to the isotropic kernel rather
    than rotating by a near-arbitrary, noise-dominated direction."""
    horizontal = PlaneModel(center=np.zeros(3), normal=np.array([0.0, 0.0, 1.0]),
                            u=np.array([1.0, 0.0, 0.0]),
                            v=np.array([0.0, 1.0, 0.0]))
    assert _up_2d(horizontal) is None


def test_up_2d_present_for_vertical_plane():
    """Sanity check on the other side of the gate: a vertical plane (world
    z lies entirely in-plane) must return a unit vector, not None."""
    vertical = PlaneModel(center=np.zeros(3), normal=np.array([1.0, 0.0, 0.0]),
                          u=np.array([0.0, 1.0, 0.0]),
                          v=np.array([0.0, 0.0, 1.0]))
    up = _up_2d(vertical)
    assert up is not None
    np.testing.assert_allclose(np.linalg.norm(up), 1.0)
    np.testing.assert_allclose(up, [0.0, 1.0], atol=1e-12)
