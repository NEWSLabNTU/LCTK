"""Generator "e" wiring: dispatch, the required-background contract, and
that every downstream gate still works unchanged on its candidates."""
from __future__ import annotations

from unittest.mock import patch

import numpy as np
import pytest

from boarddet.background import BackgroundModel
from boarddet.board_config import BoardConfig
from boarddet.detector import GENERATORS, _stance, detect
from boarddet.geometry import downsample
from boarddet.synth import make_board, make_scene


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


# --- Q2: cluster_min_points is a per-run knob, not a hardcoded 30 ----------
# The far (9 m) VLP board's sparse corner points fall below a 30-neighbour
# DBSCAN density and get dropped as noise, truncating the cluster so the
# fitted quad no longer spans the board (measured: recall 79.9% -> 91.2% at
# min_points=20, precision held at 100%). detect() must thread the config
# value into generator "e" instead of using the generator's default.

def test_cluster_min_points_is_threaded_into_generator_e():
    pts, _ = make_scene(rng=np.random.default_rng(6))
    board = BoardConfig(side_m=1.0, cluster_min_points=7)
    captured: dict = {}
    real = GENERATORS["e"]

    def spy(*args, **kwargs):
        captured.update(kwargs)
        return real(*args, **kwargs)

    with patch.dict(GENERATORS, {"e": spy}):
        detect(pts, board, generator="e",
               background=_model_from(
                   make_scene(rng=np.random.default_rng(7),
                              include_board=False)[0]))
    assert captured.get("cluster_min_points") == 7


# --- Q3: gravity axis is per-rig, not a hardcoded world +z -----------------
# A z-forward sensor frame (e.g. the Seyond Falcon, board at z~7.4 m) has
# gravity along a different axis. With the stance gate on and world-up wired
# to +z, an upright board reads stance ~0 and is rejected (measured: Falcon
# recall collapses to 0/397). The up axis must come from the board config.

def test_stance_respects_the_supplied_up_axis():
    # diamond standing on a corner, vertical diagonal along +y (z-forward rig)
    corners = np.array([[0.0, 0.7, 7.0], [0.7, 0.0, 7.0],
                        [0.0, -0.7, 7.0], [-0.7, 0.0, 7.0]])
    assert _stance(corners, np.array([0.0, 1.0, 0.0])) > 0.99
    assert _stance(corners, np.array([0.0, 0.0, 1.0])) < 0.1


def _zforward_reveal_and_background():
    """A board facing a z-forward sensor at ~7 m, standing on a corner with
    its vertical diagonal along +y, plus a static wall the background has
    already seen so the board is the only foreground."""
    rng = np.random.default_rng(8)
    board_pts, truth = make_board(
        side=1.0, center=np.array([0.0, 0.0, 7.0]),
        normal=np.array([0.05, 0.1, -1.0]), up_hint=np.array([0.0, 1.0, 0.0]),
        spacing=0.02, noise=0.004, rng=rng)
    w = rng.uniform([-3.0, -3.0, 10.0], [3.0, 3.0, 10.0], size=(4000, 3))
    wall = w.astype(np.float32)
    reveal = np.concatenate([board_pts, wall]).astype(np.float32)
    return reveal, wall, truth


def test_zforward_board_needs_the_correct_up_axis_under_stance_gate():
    reveal, wall, truth = _zforward_reveal_and_background()
    model = _model_from(wall)
    common = dict(side_m=1.0, square_icp=True, stance_floor=0.9,
                  vertical_gap_deg=0.0, flatness_rms_max=0.045)
    # world-up = +z is wrong for this frame: the upright board fails stance
    out_z = detect(reveal, BoardConfig(up_axis=(0.0, 0.0, 1.0), **common),
                   generator="e", background=model)
    assert out_z.detection is None
    # gravity along +y recovers it
    out_y = detect(reveal, BoardConfig(up_axis=(0.0, 1.0, 0.0), **common),
                   generator="e", background=model)
    assert out_y.detection is not None
    assert np.linalg.norm(out_y.detection.center - truth.center) < 0.2
