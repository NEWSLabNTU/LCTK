import numpy as np
from boarddet.board_config import BoardConfig
from boarddet.geometry import fit_plane, project_to_plane
from boarddet.scorer import score_candidate
from boarddet.synth import make_board


def _board_2d(side=1.0, noise=0.005, spacing=0.02, seed=4, holes=None,
              pattern="grid"):
    rng = np.random.default_rng(seed)
    pts, truth = make_board(
        side=side, center=np.array([3.0, 0.0, 0.5]),
        normal=np.array([-1.0, 0.1, 0.05]),
        up_hint=np.array([0.0, 0.0, 1.0]),
        spacing=spacing, noise=noise, rng=rng, holes=holes, pattern=pattern,
    )
    return project_to_plane(pts, fit_plane(pts))


# Matches ros/lctk_launch/config/board/board_detector.json5: hole_radius
# 150mm, hole_center_shift 200mm, 3 of the 4 possible corner holes punched
# (the recorded board is a hollow diamond, not a solid one).
_REAL_HOLES = [((0.2, 0.2), 0.15), ((0.2, -0.2), 0.15), ((-0.2, 0.2), 0.15)]


def test_scores_true_board_high():
    res = score_candidate(_board_2d(), BoardConfig(side_m=1.0))
    assert res is not None
    assert res.score > 0.6
    np.testing.assert_allclose(res.side_lengths.mean(), 1.0, atol=0.08)
    assert res.angle_err_deg < 6.0


def test_scores_hollow_board_high():
    """The recorded board has 3 punched holes (fill_ratio << 1 even for a
    perfect, fully-observed board), so the fill term must not tank the score
    below min_score for an otherwise-perfect fit."""
    res = score_candidate(_board_2d(holes=_REAL_HOLES), BoardConfig(side_m=1.0))
    assert res is not None
    assert res.fill_ratio < 0.85  # holes measurably reduce fill...
    assert res.score > 0.5        # ...but the fit must still clear min_score


def test_scores_sparse_hollow_board_above_min_score():
    """Reproduces what dataset 3 frame 5 actually looked like: a hollow
    board seen through VLP-32C ring gaps has fill_ratio ~0.44-0.45 (holes
    plus real sparsity, not just holes) with an otherwise good outer-border
    fit. Before the sqrt(fill)/loosened side_err weighting (this task), this
    scenario scored ~0.39 and was silently rejected by min_score=0.5."""
    coords = _board_2d(spacing=0.05, noise=0.02, holes=_REAL_HOLES,
                       pattern="uniform")
    res = score_candidate(coords, BoardConfig(side_m=1.0))
    assert res is not None
    assert 0.3 < res.fill_ratio < 0.6  # matches the real observed range
    assert res.score > 0.5


def test_corner_accuracy_beats_cell_size():
    board = BoardConfig(side_m=1.0)
    res = score_candidate(_board_2d(noise=0.003), board)
    assert res is not None
    d = 1.0 / np.sqrt(2.0)  # half-diagonal
    c = res.corners_2d.mean(axis=0)
    # board is centred at the projection origin
    assert np.linalg.norm(c) < board.cell_m
    # each corner sits at radius d from the centroid (rotation-invariant)
    radii = np.linalg.norm(res.corners_2d - c, axis=1)
    assert np.abs(radii - d).max() < board.cell_m
    # diagonals are orthogonal
    d1 = res.corners_2d[2] - res.corners_2d[0]
    d2 = res.corners_2d[3] - res.corners_2d[1]
    cosang = abs(d1 @ d2) / (np.linalg.norm(d1) * np.linalg.norm(d2))
    assert cosang < 0.02


def test_rejects_wrong_size():
    assert score_candidate(_board_2d(side=2.5), BoardConfig(side_m=1.0)) is None


def test_rejects_sparse_garbage():
    rng = np.random.default_rng(5)
    junk = rng.uniform(-1, 1, size=(150, 2)).astype(np.float32)
    res = score_candidate(junk, BoardConfig(side_m=1.0))
    assert res is None or res.score < 0.5
