import numpy as np
from boarddet.board_config import BoardConfig
from boarddet.geometry import fit_plane, project_to_plane
from boarddet.scorer import score_candidate
from boarddet.synth import make_board


def _board_2d(side=1.0, noise=0.005, spacing=0.02, seed=4):
    rng = np.random.default_rng(seed)
    pts, truth = make_board(
        side=side, center=np.array([3.0, 0.0, 0.5]),
        normal=np.array([-1.0, 0.1, 0.05]),
        up_hint=np.array([0.0, 0.0, 1.0]),
        spacing=spacing, noise=noise, rng=rng,
    )
    return project_to_plane(pts, fit_plane(pts))


def test_scores_true_board_high():
    res = score_candidate(_board_2d(), BoardConfig(side_m=1.0))
    assert res is not None
    assert res.score > 0.6
    np.testing.assert_allclose(res.side_lengths.mean(), 1.0, atol=0.08)
    assert res.angle_err_deg < 6.0


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
    junk = rng.uniform(-1, 1, size=(40, 2)).astype(np.float32)
    res = score_candidate(junk, BoardConfig(side_m=1.0))
    assert res is None or res.score < 0.5
