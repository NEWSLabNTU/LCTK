import numpy as np
import pytest
from boarddet.board_config import BoardConfig
from boarddet.detector import GENERATORS, _stance, detect
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
