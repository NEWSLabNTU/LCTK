import numpy as np
from boarddet.board_config import BoardConfig
from boarddet.candidates.cluster_after_ground import \
    generate_cluster_after_ground
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
