"""TDD for Method E's BackgroundModel: per-source voxel accumulation,
voxel-boundary aliasing via query-time dilation, and >=min_sources consensus."""
from __future__ import annotations

import numpy as np
import pytest

from boarddet.background import BackgroundModel


def _patch(lo: float, hi: float, spacing: float, x: float,
           noise: float, rng: np.random.Generator) -> np.ndarray:
    """A planar patch at constant x, spanning [lo, hi] in y and z, with
    per-point range noise along x (the VLP-32C's own noise axis)."""
    ys = np.arange(lo, hi, spacing)
    yy, zz = np.meshgrid(ys, ys)
    pts = np.stack([np.full(yy.size, x), yy.ravel(), zz.ravel()], axis=1)
    pts[:, 0] += rng.normal(0.0, noise, size=len(pts))
    return pts.astype(np.float32)


def test_foreground_before_finalize_raises():
    """Querying an un-finalized model is a caller bug, not a silent
    pass-through -- observe() invalidates any earlier finalize()."""
    m = BackgroundModel()
    m.observe(np.zeros((10, 3), dtype=np.float32), source="a")
    with pytest.raises(RuntimeError, match="finalize"):
        m.foreground_points(np.zeros((3, 3), dtype=np.float32))


def test_empty_model_treats_everything_as_foreground():
    """A finalized model with no observations has an empty background, so
    every query point is new."""
    m = BackgroundModel()
    m.finalize()
    assert m.n_voxels == 0
    dn = np.array([[1.0, 2.0, 3.0]], dtype=np.float32)
    assert len(m.foreground_points(dn)) == 1


def test_observe_empty_input_is_a_noop():
    m = BackgroundModel(min_sources=1)
    m.observe(np.zeros((0, 3), dtype=np.float32), source="a")
    m.finalize()
    assert m.n_voxels == 0


def test_repeated_observation_of_same_geometry_dedupes():
    """Two independently-noised looks at one static patch must not double
    the voxel count -- occupancy is a set, not a tally."""
    rng = np.random.default_rng(0)
    m = BackgroundModel(min_sources=1)
    m.observe(_patch(-0.5, 0.5, 0.02, 4.0, 0.01, rng), source="a")
    m.finalize()
    once = m.n_voxels
    m.observe(_patch(-0.5, 0.5, 0.02, 4.0, 0.01, rng), source="a")
    m.finalize()
    assert m.n_voxels < 1.3 * once


def test_adjacent_cell_query_still_reads_as_background():
    """THE aliasing fix: a background point at x=0.05 lands in voxel index
    0 (0.05/0.06 -> floor 0); a query point 0.02 m away at x=0.07 lands in
    index 1. 0.02 m is well below the 0.03 m sensor noise floor, so this is
    the same physical surface and must NOT be reported as new."""
    bg = np.array([[0.05, 0.0, 0.0]], dtype=np.float32)
    query = np.array([[0.07, 0.0, 0.0]], dtype=np.float32)
    m = BackgroundModel(voxel=0.06, dilation_radius=1, min_sources=1)
    m.observe(bg, source="a")
    m.finalize()
    assert len(m.foreground_points(query)) == 0


def test_dilation_radius_zero_reproduces_the_aliasing_bug():
    """The other half of the proof: without dilation the same query point
    is wrongly flagged foreground, which is exactly what dilation exists
    to prevent."""
    bg = np.array([[0.05, 0.0, 0.0]], dtype=np.float32)
    query = np.array([[0.07, 0.0, 0.0]], dtype=np.float32)
    m = BackgroundModel(voxel=0.06, dilation_radius=0, min_sources=1)
    m.observe(bg, source="a")
    m.finalize()
    assert len(m.foreground_points(query)) == 1


def test_static_replay_produces_no_foreground():
    rng = np.random.default_rng(1)
    m = BackgroundModel(min_sources=1)
    m.observe(_patch(-0.5, 0.5, 0.02, 4.0, 0.01, rng), source="a")
    m.finalize()
    replay = _patch(-0.5, 0.5, 0.02, 4.0, 0.01, rng)
    assert len(m.foreground_points(replay)) == 0


def test_added_object_becomes_foreground():
    """Static patch memorized; a spatially separate new patch survives."""
    rng = np.random.default_rng(2)
    static = _patch(-0.5, 0.5, 0.02, 4.0, 0.01, rng)
    m = BackgroundModel(min_sources=1)
    m.observe(static, source="a")
    m.finalize()
    added = _patch(-0.3, 0.3, 0.02, 2.0, 0.01, rng)
    fg = m.foreground_points(np.concatenate([static, added]))
    assert len(fg) > 0.9 * len(added)
    assert fg[:, 0].min() > 1.5  # nothing from the x=4.0 patch survived


def test_consensus_drops_geometry_seen_by_only_one_source():
    """The claim LOO rests on: shared room (2 sources) stays background,
    a single source's own board (1 source) does not."""
    shared = np.array([[1.0, 0.0, 0.0]], dtype=np.float32)
    only_a = np.array([[5.0, 0.0, 0.0]], dtype=np.float32)
    m = BackgroundModel(voxel=0.06, dilation_radius=0, min_sources=2)
    m.observe(np.concatenate([shared, only_a]), source="a")
    m.observe(shared, source="b")
    m.finalize()
    assert m.n_voxels == 1
    assert m.n_sources == 2
    fg = m.foreground_points(np.concatenate([shared, only_a]))
    assert len(fg) == 1
    assert np.allclose(fg[0], only_a[0])


def test_min_sources_one_is_exactly_a_plain_union():
    """The union ablation must be a flag flip, not a separate code path."""
    shared = np.array([[1.0, 0.0, 0.0]], dtype=np.float32)
    only_a = np.array([[5.0, 0.0, 0.0]], dtype=np.float32)
    m = BackgroundModel(voxel=0.06, dilation_radius=0, min_sources=1)
    m.observe(np.concatenate([shared, only_a]), source="a")
    m.observe(shared, source="b")
    m.finalize()
    assert m.n_voxels == 2
    assert len(m.foreground_points(np.concatenate([shared, only_a]))) == 0
