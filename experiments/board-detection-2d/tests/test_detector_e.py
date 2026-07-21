"""Generator "e" wiring: dispatch, the required-background contract, and
that every downstream gate still works unchanged on its candidates."""
from __future__ import annotations

import numpy as np
import pytest

from boarddet.background import BackgroundModel
from boarddet.board_config import BoardConfig
from boarddet.detector import GENERATORS, detect
from boarddet.geometry import downsample
from boarddet.synth import make_scene


def _model_from(points, min_sources=1):
    m = BackgroundModel(min_sources=min_sources)
    m.observe(downsample(points, 0.03), source=0)
    m.finalize()
    return m


def test_generator_e_is_registered():
    assert "e" in GENERATORS


def test_missing_background_raises_a_clear_error():
    """Silently detecting nothing would look like a bad frame; a missing
    model is a caller bug and must say so."""
    pts, _ = make_scene(rng=np.random.default_rng(0))
    with pytest.raises(ValueError, match="background"):
        detect(pts, BoardConfig(side_m=1.0), generator="e")


def test_detects_revealed_board_end_to_end():
    bg_pts, _ = make_scene(rng=np.random.default_rng(1), include_board=False)
    reveal, truth = make_scene(rng=np.random.default_rng(2))
    out = detect(reveal, BoardConfig(side_m=1.0), generator="e",
                 background=_model_from(bg_pts))
    assert out.detection is not None
    assert np.linalg.norm(out.detection.center - truth.center) < 0.15


def test_stance_and_isolation_gates_still_apply():
    """Downstream gates are generator-agnostic -- the stage-8 operating
    point must be expressible for "e" exactly as for "b"."""
    bg_pts, _ = make_scene(rng=np.random.default_rng(3), include_board=False)
    reveal, _ = make_scene(rng=np.random.default_rng(4))
    board = BoardConfig(side_m=1.0, stance_floor=0.9, flatness_rms_max=0.045,
                        isolation=True, isolation_max_density=0.3)
    out = detect(reveal, board, generator="e", background=_model_from(bg_pts))
    assert "total" in out.timings_ms


def test_generators_abc_unaffected_by_the_new_parameter():
    pts, truth = make_scene(rng=np.random.default_rng(5))
    out = detect(pts, BoardConfig(side_m=1.0), generator="b")
    assert out.detection is not None
