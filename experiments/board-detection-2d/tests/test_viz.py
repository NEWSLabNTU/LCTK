"""Smoke + edge-case tests for the Method E 6-panel renderer. An Agg render
can't be pixel-asserted cheaply, so these pin 'writes a valid non-empty PNG'
and 'never crashes on the None-detection / empty-foreground paths' -- where
this code actually breaks."""
from __future__ import annotations

import numpy as np

from boarddet.background import BackgroundModel
from boarddet.bbox_ref import load_bbox
from boarddet.benchmark_e_loo import DEFAULT_BBOX_PATH
from boarddet.board_config import BoardConfig
from boarddet.detector import detect
from boarddet.geometry import downsample
from boarddet.viz import render_methode


def _png_header(p) -> bytes:
    return p.read_bytes()[:8]


_PNG_MAGIC = b"\x89PNG\r\n\x1a\n"


def _model(points) -> BackgroundModel:
    m = BackgroundModel(min_sources=1)
    m.observe(downsample(points, 0.03), source=0)
    m.finalize()
    return m


def test_renders_a_detection(tmp_path):
    from boarddet.synth import make_scene
    bg, _ = make_scene(rng=np.random.default_rng(0), include_board=False)
    reveal, _ = make_scene(rng=np.random.default_rng(1))
    board = BoardConfig(side_m=1.0)
    out = detect(reveal, board, generator="e", background=_model(bg))
    assert out.detection is not None
    box = load_bbox(DEFAULT_BBOX_PATH)
    p = tmp_path / "det.png"
    render_methode(reveal, board, _model(bg), out, box, p)
    assert p.exists()
    assert _png_header(p) == _PNG_MAGIC
    assert p.stat().st_size > 3000


def test_renders_without_detection(tmp_path):
    """Background memorizes the board, so the reveal has no foreground and no
    detection -- the None path must render, not crash."""
    from boarddet.synth import make_scene
    scene, _ = make_scene(rng=np.random.default_rng(2))
    board = BoardConfig(side_m=1.0)
    model = _model(scene)  # same scene as background -> nothing new
    out = detect(scene, board, generator="e", background=model)
    assert out.detection is None
    box = load_bbox(DEFAULT_BBOX_PATH)
    p = tmp_path / "nodet.png"
    render_methode(scene, board, model, out, box, p)
    assert p.exists()
    assert _png_header(p) == _PNG_MAGIC


def test_renders_with_empty_foreground(tmp_path):
    """A finalized-but-identical background yields an empty foreground array;
    the renderer must handle a 0-row layer without raising."""
    from boarddet.synth import make_scene
    scene, _ = make_scene(rng=np.random.default_rng(3))
    board = BoardConfig(side_m=1.0)
    model = _model(scene)
    out = detect(scene, board, generator="e", background=model)
    box = load_bbox(DEFAULT_BBOX_PATH)
    p = tmp_path / "empty.png"
    # sanity: foreground really is empty on this identical replay
    assert len(model.foreground_points(downsample(scene, 0.03))) == 0
    render_methode(scene, board, model, out, box, p)
    assert p.exists()
    assert _png_header(p) == _PNG_MAGIC


def test_front_side_panel_convention():
    """x=front, y=left, z=up: front view is y-z with the horizontal axis
    inverted (left renders left); side view is x-z, +x (front) to the right."""
    from boarddet.viz import _FRONT, _SIDE
    ai, bi, invert_h, _, _, title = _FRONT
    assert (ai, bi) == (1, 2)      # y-z projection (look along x)
    assert invert_h is True        # +y (left) rendered on the left
    assert "front" in title
    ai, bi, invert_h, _, _, title = _SIDE
    assert (ai, bi) == (0, 2)      # x-z projection (look along y)
    assert invert_h is False       # +x (front) to the right
    assert "side" in title
