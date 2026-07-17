import numpy as np
from boarddet.benchmark import accumulate_frames, summarize
from boarddet.board_config import BoardConfig
from boarddet.detector import detect
from boarddet.ingest import Frame
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


def _dummy_frame(n_pts: int, tag: float) -> Frame:
    return Frame(
        stamp=tag,
        xyz=np.full((n_pts, 3), tag, dtype=np.float32),
        intensity=np.zeros(n_pts, dtype=np.float32),
        ring=np.zeros(n_pts, dtype=np.uint8),
    )


def test_accumulate_frames_chunks_and_drops_short_partial():
    frames = [_dummy_frame(10, float(i)) for i in range(7)]
    windows = accumulate_frames(frames, 3)
    assert len(windows) == 2  # [3, 3]; trailing 1-frame window (<1.5) dropped
    assert len(windows[0]) == 30
    assert len(windows[1]) == 30


def test_accumulate_frames_keeps_partial_at_least_half():
    # 5 frames, n=3 -> windows [3, 2]; trailing 2 frames >= 3/2 so kept.
    frames = [_dummy_frame(10, float(i)) for i in range(5)]
    windows = accumulate_frames(frames, 3)
    assert len(windows) == 2
    assert len(windows[0]) == 30
    assert len(windows[1]) == 20


def test_accumulate_frames_n1_reproduces_stage1():
    frames = [_dummy_frame(10, float(i)) for i in range(4)]
    windows = accumulate_frames(frames, 1)
    assert len(windows) == 4
    for w, f in zip(windows, frames, strict=True):
        assert np.array_equal(w, f.xyz)
