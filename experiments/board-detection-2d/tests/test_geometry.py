import numpy as np
from boarddet.geometry import (
    fit_plane, plane_rms, project_to_plane, unproject, downsample, extent_2d,
)
from boarddet.synth import make_board


def _board(noise=0.005):
    rng = np.random.default_rng(3)
    return make_board(
        side=1.0, center=np.array([3.0, 0.0, 0.5]),
        normal=np.array([-1.0, 0.2, 0.1]),
        up_hint=np.array([0.0, 0.0, 1.0]),
        spacing=0.02, noise=noise, rng=rng,
    )


def test_fit_plane_recovers_normal():
    pts, truth = _board()
    plane = fit_plane(pts)
    assert abs(plane.normal @ truth.normal) > 0.999
    assert plane_rms(pts, plane) < 0.01
    # basis orthonormal
    for a, b in [(plane.u, plane.v), (plane.u, plane.normal),
                 (plane.v, plane.normal)]:
        assert abs(a @ b) < 1e-9
    assert np.isclose(np.linalg.norm(plane.u), 1.0)


def test_project_unproject_roundtrip():
    pts, _ = _board(noise=0.0)
    plane = fit_plane(pts)
    c2 = project_to_plane(pts, plane)
    back = unproject(c2, plane)
    assert np.abs(back - pts).max() < 1e-5


def test_extent_and_downsample():
    pts, _ = _board(noise=0.0)
    c2 = project_to_plane(pts, fit_plane(pts))
    # diamond of side 1.0: bbox is diagonal x diagonal = sqrt(2) x sqrt(2)
    assert np.isclose(extent_2d(c2), np.sqrt(2.0), atol=0.05)
    dn = downsample(pts, voxel=0.1)
    assert 10 < len(dn) < len(pts)


def test_finite_only_drops_non_finite_rows():
    from boarddet.geometry import finite_only
    pts = np.array([
        [1.0, 2.0, 3.0],
        [np.nan, 0.0, 0.0],
        [0.0, np.inf, 0.0],
        [4.0, 5.0, 6.0],
        [0.0, 0.0, -np.inf],
    ], dtype=np.float32)
    out = finite_only(pts)
    assert out.shape == (2, 3)
    np.testing.assert_allclose(out, [[1, 2, 3], [4, 5, 6]])
