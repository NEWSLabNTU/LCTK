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


def test_flatness_gate_clears_real_vlp32c_noise_floor():
    """CLAUDE.md documents the VLP-32C's own range noise pushing ICP loss on
    the recorded board to 0.026-0.029 m (icp_good_fit_threshold=0.035 to
    clear it with margin). This gate's plane-fit RMS on real board clusters
    measured in the same 0.029-0.031 m band, so it must accept patches noisy
    up to that floor, not just noiseless synthetic ones."""
    rng = np.random.default_rng(3)
    g = rng.uniform(-0.5, 0.5, size=(200, 2))
    patch = np.c_[g[:, 0], g[:, 1], rng.normal(0, 0.029, 200)]
    assert plausible_board_patch(patch.astype(np.float32),
                                 BoardConfig(side_m=1.0)) is not None


def test_flatness_gate_still_rejects_non_planar_patch():
    rng = np.random.default_rng(3)
    g = rng.uniform(-0.5, 0.5, size=(200, 2))
    patch = np.c_[g[:, 0], g[:, 1], rng.normal(0, 0.08, 200)]
    assert plausible_board_patch(patch.astype(np.float32),
                                 BoardConfig(side_m=1.0)) is None
