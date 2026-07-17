import numpy as np
from boarddet.board_config import BoardConfig
from boarddet.candidates.cluster_after_ground import \
    _merge_coplanar_clusters, _remove_big_planes, generate_cluster_after_ground
from boarddet.geometry import downsample
from boarddet.synth import make_scene


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
