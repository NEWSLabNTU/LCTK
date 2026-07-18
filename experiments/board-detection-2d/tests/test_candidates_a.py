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


def _near_miss_patch(seed: int = 21) -> np.ndarray:
    """A synthetic planar patch with plane-fit RMS ~0.040 m -- inside the
    27%-of-frames "near-miss" band (0.035-0.048) the stage6 diagnosis found
    between the old 0.035 gate and real VLP-32C board noise."""
    rng = np.random.default_rng(seed)
    g = rng.uniform(-0.5, 0.5, size=(200, 2))
    return np.c_[g[:, 0], g[:, 1], rng.normal(0, 0.040, 200)].astype(np.float32)


def test_flatness_rms_max_param_configurable_and_discriminates():
    """Task 20: flatness_rms_max is a real, discriminating parameter -- the
    same ~0.040 RMS patch is rejected at 0.035 (current default) and
    accepted at 0.045 (the sweep value the diagnosis recommends)."""
    patch = _near_miss_patch()
    board = BoardConfig(side_m=1.0)
    assert plausible_board_patch(patch, board, flatness_rms_max=0.035) is None
    assert plausible_board_patch(
        patch, board, flatness_rms_max=0.045) is not None


def test_flatness_rms_max_default_none_reads_board_config():
    """When flatness_rms_max=None (the default), the gate must read
    board.flatness_rms_max rather than silently using the module constant."""
    patch = _near_miss_patch()
    strict_board = BoardConfig(side_m=1.0, flatness_rms_max=0.035)
    loose_board = BoardConfig(side_m=1.0, flatness_rms_max=0.045)
    assert plausible_board_patch(patch, strict_board) is None
    assert plausible_board_patch(patch, loose_board) is not None
