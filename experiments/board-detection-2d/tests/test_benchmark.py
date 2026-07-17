import numpy as np
from boarddet.benchmark import summarize
from boarddet.board_config import BoardConfig
from boarddet.detector import detect
from boarddet.synth import make_scene
from boarddet.viz import save_overlay


def test_save_overlay_writes_png(tmp_path):
    pts, _ = make_scene(rng=np.random.default_rng(15))
    out = detect(pts, BoardConfig(), generator="b")
    p = tmp_path / "overlay.png"
    save_overlay(pts, out, p)
    assert p.exists() and p.stat().st_size > 1000


def test_summarize_computes_rates_and_jitter():
    pts, _ = make_scene(rng=np.random.default_rng(16))
    outcomes = [detect(pts, BoardConfig(), generator="b") for _ in range(3)]
    s = summarize(outcomes)
    assert s["detection_rate"] == 1.0
    assert s["jitter_center_mm"] < 1e-6  # identical frames -> zero jitter
    assert s["median_total_ms"] > 0
