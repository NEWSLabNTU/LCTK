import numpy as np
from boarddet.synth import make_scene, make_board


def test_make_board_points_lie_on_plane_within_noise():
    rng = np.random.default_rng(0)
    pts, truth = make_board(
        side=1.0,
        center=np.array([3.0, 0.0, 0.5]),
        normal=np.array([-1.0, 0.2, 0.0]),
        up_hint=np.array([0.0, 0.0, 1.0]),
        spacing=0.03,
        noise=0.005,
        rng=rng,
    )
    n = truth.normal / np.linalg.norm(truth.normal)
    d = (pts - truth.center) @ n
    assert np.abs(d).max() < 0.03  # within a few noise sigma
    # diamond diagonal = side * sqrt(2)
    assert np.isclose(
        np.linalg.norm(truth.corners[0] - truth.corners[2]),
        np.sqrt(2.0),
        atol=1e-6,
    )


def test_make_scene_contains_board_and_clutter():
    pts, truth = make_scene(rng=np.random.default_rng(1))
    assert pts.dtype == np.float32
    assert len(pts) > 5_000
    # some points near the board plane, many not
    n = truth.normal
    d = np.abs((pts - truth.center) @ n)
    near = d < 0.02
    assert 200 < near.sum() < len(pts) // 2


def test_uniform_pattern_differs_from_grid():
    g, _ = make_scene(pattern="grid", rng=np.random.default_rng(2))
    u, _ = make_scene(pattern="uniform", rng=np.random.default_rng(2))
    assert g.shape != u.shape or not np.allclose(g[: len(u)], u[: len(g)])


def test_noise_displacement_is_normal_only():
    """Verify that noise is confined to the plane normal direction."""
    kwargs = dict(
        side=1.0, center=np.array([3.0, 0.0, 0.5]),
        normal=np.array([-1.0, 0.2, 0.0]),
        up_hint=np.array([0.0, 0.0, 1.0]), spacing=0.03,
    )
    clean, truth = make_board(noise=0.0, rng=np.random.default_rng(42), **kwargs)
    noisy, _ = make_board(noise=0.01, rng=np.random.default_rng(42), **kwargs)
    d = noisy - clean
    tangential = d - (d @ truth.normal)[:, None] * truth.normal
    assert np.abs(tangential).max() < 1e-6
