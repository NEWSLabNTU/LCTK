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


def test_normal_gate_separates_touching_perpendicular_patches():
    # two 0.7 m square patches meeting at a 90 deg crease, one point-spacing
    # apart: the kNN graph bridges the crease, so only the normal-coherence
    # gate can keep the regions apart
    s = np.arange(-0.35, 0.35, 0.03)
    t = np.arange(0.0, 0.7, 0.03)
    yy, zz = np.meshgrid(s, t)
    vertical = np.stack(
        [np.full(yy.size, 4.0), yy.ravel(), zz.ravel()], axis=1)
    yy2, xx2 = np.meshgrid(s, np.arange(0.03, 0.73, 0.03))
    horizontal = np.stack(
        [4.0 + xx2.ravel(), yy2.ravel(), np.zeros(yy2.size)], axis=1)
    pts = np.concatenate([vertical, horizontal]).astype(np.float32)
    rng = np.random.default_rng(22)
    pts = pts + rng.normal(0.0, 0.002, pts.shape).astype(np.float32)

    cands = generate_region_growing(pts, BoardConfig(side_m=1.0))

    # a merged crease-spanning region would fail the flatness gate and yield
    # nothing; correct behaviour is exactly two pure planar candidates
    assert len(cands) == 2
    xs = sorted(abs(c.plane.normal[0]) for c in cands)
    assert xs[0] < 0.1   # horizontal patch: normal ~ ±z
    assert xs[1] > 0.99  # vertical patch: normal ~ ±x
