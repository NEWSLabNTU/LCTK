import numpy as np
import pytest
from boarddet.board_config import BoardConfig
from boarddet.detector import GENERATORS, detect
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
