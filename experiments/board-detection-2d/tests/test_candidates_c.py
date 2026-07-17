import numpy as np
from boarddet.board_config import BoardConfig
from boarddet.candidates.region_growing import generate_region_growing
from boarddet.geometry import downsample
from boarddet.synth import make_scene


def test_finds_board_region():
    pts, truth = make_scene(rng=np.random.default_rng(11))
    cands = generate_region_growing(
        downsample(pts, 0.03), BoardConfig(side_m=1.0))
    matches = [
        c for c in cands
        if abs(c.plane.normal @ truth.normal) > 0.99
        and abs((c.plane.center - truth.center) @ truth.normal) < 0.05
    ]
    assert matches


def test_separates_board_leaning_near_wall():
    # board 0.3 m in front of the wall, tilted 30 deg from wall normal:
    # clustering by distance would merge, normals must separate
    rng = np.random.default_rng(12)
    pts, truth = make_scene(board_center=(7.6, 0.5, 0.3),
                            rng=rng)
    cands = generate_region_growing(
        downsample(pts, 0.03), BoardConfig(side_m=1.0))
    matches = [c for c in cands
               if abs(c.plane.normal @ truth.normal) > 0.99]
    assert matches
