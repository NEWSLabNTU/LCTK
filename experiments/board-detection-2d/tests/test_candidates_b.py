import numpy as np
from boarddet.board_config import BoardConfig
from boarddet.candidates.cluster_after_ground import \
    _anisotropic_scaled, _merge_coplanar_clusters, _remove_big_planes, \
    generate_cluster_after_ground, big_plane_residual, _BIG_PLANE_DIST, \
    _BIG_PLANE_MIN_FRAC
from boarddet.geometry import downsample
from boarddet.synth import make_board, make_scene


def test_finds_board_cluster():
    pts, truth = make_scene(rng=np.random.default_rng(9))
    cands = generate_cluster_after_ground(
        downsample(pts, 0.03), BoardConfig(side_m=1.0))
    matches = [
        c for c in cands
        if abs(c.plane.normal @ truth.normal) > 0.99
        and abs((c.plane.center - truth.center) @ truth.normal) < 0.05
    ]
    assert matches


def test_ground_and_wall_removed_before_clustering():
    pts, _ = make_scene(rng=np.random.default_rng(10))
    cands = generate_cluster_after_ground(
        downsample(pts, 0.03), BoardConfig(side_m=1.0))
    # no candidate should be near-horizontal (the ground)
    for c in cands:
        assert abs(c.plane.normal[2]) < 0.9


def test_merge_coplanar_clusters_rejoins_ring_gap_stripes():
    """Two coplanar horizontal stripes of one board face, split by a ring-gap
    wider than the upstream DBSCAN eps (0.15 m), must be regrown into a
    single group by _merge_coplanar_clusters. A non-coplanar decoy cluster
    that sits within merge range but well off the stripes' plane must stay
    separate -- this exercises the plane-offset test, not just the distance
    gate.

    Mirrors the real VLP-32C failure mode described in the function's
    docstring: a physical board returns as several thin horizontal-stripe
    DBSCAN clusters because ring gaps cross the board face itself.
    """
    xs = np.arange(-0.5, 0.51, 0.05)
    stripe1 = np.array([[x, y, 0.0] for x in xs for y in (0.0, 0.04, 0.08)],
                        dtype=np.float32)
    # gap of 0.20 m to stripe1 (> cluster_eps=0.15 used upstream), same plane
    stripe2 = np.array([[x, y, 0.0] for x in xs for y in (0.28, 0.32, 0.36)],
                        dtype=np.float32)
    # sits between the stripes in xy (within merge_dist_factor range) but
    # 0.5 m off the z=0 plane -- must be rejected on coplanarity, not distance
    decoy = np.array([[x, y, 0.5] for x in xs for y in (0.15, 0.19)],
                      dtype=np.float32)

    points = np.concatenate([stripe1, stripe2, decoy], axis=0)
    labels = np.concatenate([
        np.zeros(len(stripe1), dtype=int),
        np.ones(len(stripe2), dtype=int),
        np.full(len(decoy), 2, dtype=int),
    ])

    groups = _merge_coplanar_clusters(points, labels, BoardConfig(side_m=1.0))
    sizes = sorted(len(g) for g in groups)

    # the two stripes merged into one group of (approximately) their combined
    # point count, and nothing else
    assert sizes == [len(decoy), len(stripe1) + len(stripe2)]

    merged = next(g for g in groups if len(g) == len(stripe1) + len(stripe2))
    decoy_group = next(g for g in groups if len(g) == len(decoy))

    # the merged group is exactly the two stripes (all z == 0), the decoy
    # (z == 0.5) never entered it
    assert np.allclose(merged[:, 2], 0.0)
    assert np.allclose(decoy_group[:, 2], 0.5)


def test_remove_big_planes_continues_after_noise_dbscan():
    """When a big plane's inliers DBSCAN entirely to noise (sparse, below
    the eps=0.20/min_points=10 density needed for any cluster), the strip
    loop must keep going and still strip the *next* big plane -- not
    `break` out of the whole loop (which used to leave every later big
    plane unstripped)."""
    # plane1: sparse grid, spacing 0.22 m > cluster eps (0.20 m), so every
    # point is DBSCAN noise (no point has >=10 neighbours within eps).
    xs1 = np.arange(-5.0, 5.01, 0.22)
    yy1, xx1 = np.meshgrid(xs1, xs1)
    plane1 = np.stack([xx1.ravel(), yy1.ravel(), np.zeros(xx1.size)],
                      axis=1).astype(np.float32)

    # plane2: dense grid (genuinely big, extent >> board diag), offset in z
    # so it never mixes with plane1 within dist_thresh=0.05.
    xs2 = np.arange(-2.0, 2.01, 0.1)
    yy2, xx2 = np.meshgrid(xs2, xs2)
    plane2 = np.stack([xx2.ravel(), yy2.ravel(), np.full(xx2.size, 1.0)],
                      axis=1).astype(np.float32)

    # plane1 must have more points than plane2 so open3d's RANSAC (which
    # picks the largest-consensus model) extracts it *first*, reproducing
    # the "noise-DBSCAN plane processed before the genuine big plane" order
    # the bug depended on.
    assert len(plane1) > len(plane2)

    points = np.concatenate([plane1, plane2], axis=0)
    board = BoardConfig(side_m=1.0)

    remaining = _remove_big_planes(points, board, dist=0.05, min_frac=0.08)

    # both big planes must be stripped: plane1 via the noise-DBSCAN branch,
    # plane2 via the normal extent check on the next iteration. Under the
    # old `break`-on-noise bug, plane2 (dense, clearly board-scale-plus)
    # would have survived untouched in `remaining`.
    assert len(remaining) < 50


def test_anisotropic_scaled_clamps_to_isotropic_near_origin():
    """At small range, eps_v(r) = max(eps_h, 2r*tan(gap)) clamps to eps_h, so
    z is left unscaled -- dense/near/solid-state clouds are unaffected;
    anisotropy is range-driven, not density-driven."""
    pts = np.array([[0.1, 0.0, 1.0], [0.0, 0.05, -2.0]], dtype=np.float64)
    scaled = _anisotropic_scaled(pts, eps_h=0.15, vertical_gap_deg=3.0)
    assert np.allclose(scaled[:, 2], pts[:, 2])
    assert np.allclose(scaled[:, :2], pts[:, :2])  # x, y never touched


def test_anisotropic_scaled_compresses_z_at_range():
    """At range where eps_v(r) > eps_h, z is compressed by eps_h/eps_v so the
    scaled cloud's vertical DBSCAN tolerance matches eps_h again."""
    r = 3.0
    pts = np.array([[r, 0.0, 1.0]], dtype=np.float64)
    eps_h = 0.15
    gap_deg = 3.5
    eps_v = 2.0 * r * np.tan(np.radians(gap_deg))
    assert eps_v > eps_h  # sanity: this range actually triggers widening
    scaled = _anisotropic_scaled(pts, eps_h=eps_h, vertical_gap_deg=gap_deg)
    assert np.isclose(scaled[0, 2], 1.0 * (eps_h / eps_v))


def test_anisotropic_scaled_zero_gap_disables():
    pts = np.array([[3.0, 0.0, 1.0]], dtype=np.float64)
    scaled = _anisotropic_scaled(pts, eps_h=0.15, vertical_gap_deg=0.0)
    assert np.array_equal(scaled, pts)
    assert scaled is not pts  # still a copy, not the same array


def _striped_ring_board(rng, center=(3.0, 0.0, 0.5), side=1.0, gap_deg=4.0,
                        spacing=0.02, noise=0.005, band_deg=0.2):
    """A board sampled only in thin horizontal stripes at ~gap_deg vertical
    angular spacing as seen from the origin, reproducing a VLP-32C ring
    pattern (elevation-angle geometry from xyz only -- no ring/intensity
    field). At ~3 m range and gap_deg=4, adjacent stripes are ~0.21 m apart
    in z: wider than the plain isotropic cluster_eps (0.15 m), so plain
    DBSCAN fragments the board into per-stripe islands too small/thin to
    pass plausible_board_patch (and too small individually, < the 40-point
    seed_min_points floor in `_merge_coplanar_clusters`, to regrow via the
    coplanar-merge fallback either)."""
    pts, truth = make_board(
        side=side, center=np.asarray(center, float),
        normal=np.array([-1.0, 0.0, 0.0]), up_hint=np.array([0.0, 0.0, 1.0]),
        spacing=spacing, noise=noise, rng=rng, pattern="grid")
    r = np.hypot(pts[:, 0], pts[:, 1])
    elev = np.arctan2(pts[:, 2], r)
    gap = np.radians(gap_deg)
    ring_center = np.round(elev / gap) * gap
    keep = np.abs(elev - ring_center) < np.radians(band_deg)
    return pts[keep].astype(np.float32), truth


def _matches_truth(cands, truth):
    return [
        c for c in cands
        if abs(c.plane.normal @ truth.normal) > 0.99
        and abs((c.plane.center - truth.center) @ truth.normal) < 0.05
    ]


def test_ring_striped_board_fragments_with_gap_disabled():
    """Discrimination: with vertical_gap_deg=0 (plain isotropic DBSCAN),
    generator B must find NO candidate matching the true board plane -- the
    ring-gap fragmentation this task fixes must be reproducible first."""
    pts, truth = _striped_ring_board(np.random.default_rng(1))
    cands = generate_cluster_after_ground(pts, BoardConfig(side_m=1.0),
                                          vertical_gap_deg=0.0)
    assert not _matches_truth(cands, truth)


def test_ring_striped_board_recovered_with_anisotropic_scaling():
    """Fix: the same ring-striped scan, with vertical_gap_deg>=3.5, DOES
    yield a candidate matching the true board plane."""
    pts, truth = _striped_ring_board(np.random.default_rng(1))
    cands = generate_cluster_after_ground(pts, BoardConfig(side_m=1.0),
                                          vertical_gap_deg=3.5)
    assert _matches_truth(cands, truth)


def test_horizontal_separation_not_over_merged_by_anisotropic_scaling():
    """Two boards side by side (same range/elevation, 0.5 m edge gap in y)
    must stay two separate clusters/candidates under anisotropic scaling --
    it only compresses z, so it must never bridge a purely horizontal gap
    the plain isotropic eps already respected."""
    board_side = 0.5
    half_diag = board_side / np.sqrt(2.0)
    edge_gap = 0.5
    center_sep = edge_gap + 2.0 * half_diag
    rng = np.random.default_rng(3)
    b1, t1 = make_board(
        side=board_side, center=(3.0, -center_sep / 2.0, 0.3),
        normal=(-1.0, 0.0, 0.0), up_hint=(0.0, 0.0, 1.0),
        spacing=0.03, noise=0.005, rng=rng, pattern="grid")
    b2, t2 = make_board(
        side=board_side, center=(3.0, center_sep / 2.0, 0.3),
        normal=(-1.0, 0.0, 0.0), up_hint=(0.0, 0.0, 1.0),
        spacing=0.03, noise=0.005, rng=rng, pattern="grid")
    pts = np.concatenate([b1, b2], axis=0)
    board = BoardConfig(side_m=board_side)

    cands = generate_cluster_after_ground(pts, board, vertical_gap_deg=3.5)

    assert _matches_truth(cands, t1)
    assert _matches_truth(cands, t2)
    # each match is its own candidate, not one merged blob spanning both
    for c in cands:
        ext_y = c.points[:, 1].max() - c.points[:, 1].min()
        assert ext_y < center_sep


def test_big_plane_residual_is_subset_and_matches_strip():
    pts, _ = make_scene(rng=np.random.default_rng(5))
    board = BoardConfig(side_m=1.0)
    res = big_plane_residual(pts, board, board.vertical_gap_deg)
    # Strip removes the big ground/wall planes -> strictly fewer points.
    assert 0 < len(res) < len(pts)
    # Reproduces the generator's own strip exactly (shared params + seeded
    # RANSAC make this deterministic), so a viz of `res` shows what the
    # detector actually clustered.
    direct = _remove_big_planes(pts, board, _BIG_PLANE_DIST,
                                _BIG_PLANE_MIN_FRAC, board.vertical_gap_deg)
    assert np.array_equal(res, direct)


def test_generator_forwards_rejects_kwarg():
    from boarddet.reject import RejectReason

    board = BoardConfig()
    # a scene with clusters that fail the patch gate produces patch rejects;
    # here we only assert the kwarg is accepted and the list type is honored.
    rng = np.random.default_rng(0)
    scene = rng.uniform(-1, 1, size=(500, 3))
    rejects: list[RejectReason] = []
    out = generate_cluster_after_ground(
        scene, board, vertical_gap_deg=board.vertical_gap_deg,
        cluster_min_points=board.cluster_min_points, rejects=rejects)
    assert isinstance(out, list)
    # kwarg omitted still works and is byte-identical in shape
    out2 = generate_cluster_after_ground(
        scene, board, vertical_gap_deg=board.vertical_gap_deg,
        cluster_min_points=board.cluster_min_points)
    assert isinstance(out2, list)
    assert len(out) == len(out2)
