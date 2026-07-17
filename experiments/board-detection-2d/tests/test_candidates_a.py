import numpy as np
from boarddet.board_config import BoardConfig
from boarddet.candidates import plausible_board_patch
from boarddet.candidates.ransac_iterative import generate_ransac_iterative
from boarddet.geometry import downsample
from boarddet.synth import make_scene


def test_finds_board_plane_among_candidates():
    pts, truth = make_scene(rng=np.random.default_rng(7))
    board = BoardConfig(side_m=1.0)
    cands = generate_ransac_iterative(downsample(pts, 0.03), board)
    assert len(cands) >= 1
    # at least one candidate's plane matches the true board plane
    matches = [
        c for c in cands
        if abs(c.plane.normal @ truth.normal) > 0.99
        and abs((c.plane.center - truth.center) @ truth.normal) < 0.05
    ]
    assert matches


def test_gate_rejects_huge_patch():
    rng = np.random.default_rng(8)
    # 6x6 m planar patch: flat but far too large
    g = rng.uniform(-3, 3, size=(4000, 2))
    patch = np.c_[g[:, 0], g[:, 1], rng.normal(0, 0.005, 4000)]
    assert plausible_board_patch(patch.astype(np.float32),
                                 BoardConfig(side_m=1.0)) is None
