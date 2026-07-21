"""LOO harness: bbox classification grounded in the phase-7 doc's own
confirmed coordinates, and background construction that never sees the
held-out dataset."""
from __future__ import annotations

import numpy as np
import pytest

from boarddet import benchmark_e_loo as loo
from boarddet.ingest import Frame

# Confirmed true-board centers, phase-7 doc "Pose sanity" table (:782-793).
_CONFIRMED_BOARDS = [
    (2.256, -0.059, 0.074), (2.147, 0.420, 0.076), (2.101, -0.314, 0.074),
    (2.077, -0.605, 0.066), (2.090, -0.829, 0.039),
]
# Documented static clutter attractors (phase-7 doc :572, :633, :~1697).
_CONFIRMED_CLUTTER = [(-1.83, -2.89, -0.1), (4.7, 2.6, -0.1), (-3.3, 3.4, 0.5)]


@pytest.mark.parametrize("center", _CONFIRMED_BOARDS)
def test_confirmed_board_centers_are_in_bbox(center):
    assert loo.in_bbox(np.array(center))


@pytest.mark.parametrize("center", _CONFIRMED_CLUTTER)
def test_confirmed_clutter_is_outside_bbox(center):
    assert not loo.in_bbox(np.array(center))


@pytest.mark.parametrize("center", _CONFIRMED_CLUTTER)
def test_confirmed_clutter_is_recognized_as_known(center):
    assert loo.near_known_clutter(np.array(center))


def test_true_board_is_not_flagged_as_known_clutter():
    for center in _CONFIRMED_BOARDS:
        assert not loo.near_known_clutter(np.array(center))


def _frames(x: float, n: int = 2) -> list[Frame]:
    """n frames of a small planar patch centred at (x, 0, 0)."""
    ys = np.arange(-0.2, 0.2, 0.02)
    yy, zz = np.meshgrid(ys, ys)
    xyz = np.stack([np.full(yy.size, x), yy.ravel(), zz.ravel()],
                   axis=1).astype(np.float32)
    return [Frame(stamp=float(i), xyz=xyz,
                  intensity=np.zeros(len(xyz), dtype=np.float32),
                  ring=np.zeros(len(xyz), dtype=np.uint8)) for i in range(n)]


def test_build_background_excludes_the_held_out_dataset():
    """The core LOO invariant. Dataset 3's own geometry must never enter
    its own background, or the fold is self-referential."""
    all_frames = {1: _frames(1.0), 2: _frames(1.0), 3: _frames(9.0)}
    model = loo.build_background(all_frames, held_out=3, voxel=0.06,
                                 dilation_radius=0, min_sources=1)
    assert model.n_sources == 2
    held = all_frames[3][0].xyz
    assert len(model.foreground_points(held)) == len(held)


def test_consensus_drops_a_single_contributors_unique_geometry():
    """With min_sources=2, geometry only one contributor saw (its own
    board) must not become background."""
    all_frames = {1: _frames(1.0), 2: _frames(1.0), 3: _frames(5.0),
                  4: _frames(9.0)}
    model = loo.build_background(all_frames, held_out=4, voxel=0.06,
                                 dilation_radius=0, min_sources=2)
    unique = all_frames[3][0].xyz          # seen by contributor 3 alone
    shared = all_frames[1][0].xyz          # seen by contributors 1 and 2
    assert len(model.foreground_points(unique)) == len(unique)
    assert len(model.foreground_points(shared)) == 0
