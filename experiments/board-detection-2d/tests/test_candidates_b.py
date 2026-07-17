import numpy as np
from boarddet.board_config import BoardConfig
from boarddet.candidates.cluster_after_ground import \
    _merge_coplanar_clusters, generate_cluster_after_ground
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
