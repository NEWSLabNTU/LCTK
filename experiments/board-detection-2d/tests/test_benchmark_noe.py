import json

import numpy as np

from boarddet.benchmark_noe import run_noe
from boarddet.bbox_ref import BoxRef
from boarddet.board_config import BoardConfig
from boarddet.detector import detect
from boarddet.ingest import Frame
from boarddet.synth import make_scene


def _frames(n: int) -> list[Frame]:
    pts, _ = make_scene(rng=np.random.default_rng(16))
    return [Frame(stamp=float(i), xyz=pts,
                  intensity=np.zeros(len(pts), dtype=np.float32),
                  ring=np.zeros(len(pts), dtype=np.uint8))
            for i in range(n)]


def test_run_noe_writes_recall_precision_and_overlays(tmp_path):
    frames = _frames(3)
    center = detect(frames[0].xyz, BoardConfig(), generator="b").detection.center
    box = BoxRef(center=np.asarray(center, dtype=float),
                 half=np.array([0.5, 0.5, 0.5]), rot=np.eye(3))
    summary = run_noe({"synthA": frames}, BoardConfig(), tmp_path,
                      box=box, save_overlays=2)
    cap = summary["captures"]["synthA"]
    assert cap["n_frames"] == 3
    assert cap["n_true_board"] == 3
    assert cap["recall"] == 1.0
    assert cap["precision"] == 1.0
    assert cap["median_total_ms"] > 0
    assert (tmp_path / "noe_summary.json").exists()
    on_disk = json.loads((tmp_path / "noe_summary.json").read_text())
    assert on_disk["captures"]["synthA"]["recall"] == 1.0
    assert len(list(tmp_path.glob("overlay_synthA_*.png"))) >= 1


def test_run_noe_far_box_is_all_clutter(tmp_path):
    frames = _frames(2)
    center = detect(frames[0].xyz, BoardConfig(), generator="b").detection.center
    far = BoxRef(center=np.asarray(center, dtype=float) + 100.0,
                 half=np.array([0.5, 0.5, 0.5]), rot=np.eye(3))
    summary = run_noe({"synthB": frames}, BoardConfig(), tmp_path, box=far)
    cap = summary["captures"]["synthB"]
    assert cap["n_true_board"] == 0
    assert cap["recall"] == 0.0
    assert cap["precision"] == 0.0  # detections exist, none in box
