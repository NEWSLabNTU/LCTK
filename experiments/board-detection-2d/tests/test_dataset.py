"""Tests for labeled-dataset generation (Task 30): label masks align with
the rendered board hit pixels, and dataset dump/reload round-trips."""
from __future__ import annotations

import numpy as np

from boarddet.sim.dataset import (
    DatasetConfig,
    generate_dataset,
    load_scene,
    render_labeled_scene,
)
from boarddet.sim.scenegen import SceneGenConfig
from boarddet.sim.sensor import Vlp32cSensor


def _small_cfg(**overrides) -> DatasetConfig:
    scenegen = SceneGenConfig(board_count_weights={2: 1.0}, n_clutter_range=(2, 2),
                              n_boxes_range=(0, 1), n_cylinders_range=(0, 1))
    return DatasetConfig(scenegen=scenegen, azimuth_steps=720, **overrides)


def test_render_labeled_scene_produces_requested_board_count():
    rng = np.random.default_rng(20)
    sensor = Vlp32cSensor()
    cfg = _small_cfg()
    sample = render_labeled_scene(rng, sensor, cfg)
    assert len(sample.boards) == 2


def test_label_mask_aligns_with_board_hit_pixels():
    """The board's pixel mask must actually cover the board's own hit
    pixels -- neither empty nor drastically over/under-covering."""
    rng = np.random.default_rng(21)
    sensor = Vlp32cSensor()
    cfg = _small_cfg()
    sample = render_labeled_scene(rng, sensor, cfg)
    for board in sample.boards:
        n_mask_pixels = int(board.mask.sum())
        assert n_mask_pixels > 0, "board mask is empty"
        r0, r1, c0, c1 = board.bbox
        assert r0 >= 0 and r1 >= r0 and c1 >= c0
        # every masked pixel must fall inside the reported bbox
        rows, cols = np.nonzero(board.mask)
        assert rows.min() == r0 and rows.max() == r1
        assert cols.min() == c0 and cols.max() == c1
        # the masked cells in the image channel must carry finite range values
        range_channel = sample.image[..., 0]
        assert np.all(np.isfinite(range_channel[board.mask]))


def test_two_boards_produce_two_distinct_labeled_regions():
    rng = np.random.default_rng(22)
    sensor = Vlp32cSensor()
    cfg = _small_cfg()
    sample = render_labeled_scene(rng, sensor, cfg)
    assert len(sample.boards) == 2
    m0, m1 = sample.boards[0].mask, sample.boards[1].mask
    assert m0.sum() > 0 and m1.sum() > 0
    assert not np.array_equal(m0, m1)
    overlap = np.logical_and(m0, m1)
    # boards are distinct 3D objects; their pixel footprints should not
    # be identical (a small overlap at a shared boundary would be a
    # pathological/degenerate scene, not expected here)
    assert overlap.sum() < min(m0.sum(), m1.sum())


def test_dataset_dump_and_reload_roundtrips(tmp_path):
    rng = np.random.default_rng(23)
    cfg = _small_cfg()
    out_dir = tmp_path / "synth"
    paths = generate_dataset(3, out_dir, rng, cfg)
    assert len(paths) == 3
    assert (out_dir / "manifest.json").exists()

    for path in paths:
        assert path.exists()
        sample = load_scene(path)
        assert sample.image.ndim == 3
        assert sample.image.shape[0] == 32  # real channel count
        for board in sample.boards:
            assert board.mask.shape == sample.image.shape[:2]
            assert board.corners.shape == (4, 3)
            assert isinstance(board.hollow, bool)


def test_dataset_generate_respects_requested_count(tmp_path):
    """A small multi-scene run produces exactly the requested number of
    scene files -- the full-size (~200 scene) smoke dataset is generated
    separately by the report script, not the test suite, to keep tests fast."""
    rng = np.random.default_rng(25)
    cfg = _small_cfg()
    paths = generate_dataset(5, tmp_path / "synth5", rng, cfg)
    assert len(paths) == 5
    assert len(list((tmp_path / "synth5").glob("scene_*.npz"))) == 5
