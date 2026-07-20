"""Tests for the Task 31 torch data pipeline
(`boarddet.cnn.data`): input/target shapes and dtypes, value ranges, mask
alignment (a forced 1-board scene's target mask matches the board's own
hit pixels), empty-scene all-zero masks, augmentation (roll/flip apply
identically to input and target), and synth/real structural parity (both
paths share `channels_to_input_array`, so they cannot silently diverge)."""
from __future__ import annotations

import numpy as np
import torch

from boarddet.cnn.data import (
    N_INPUT_CHANNELS,
    R_MAX,
    SynthBoardDataset,
    SynthDataConfig,
    channels_to_input_array,
    real_frame_to_input,
    render_synth_sample,
    roll_and_flip,
)
from boarddet.ingest import Frame
from boarddet.sim.range_image import points_to_rows_cols_ranges
from boarddet.sim.scenegen import SceneGenConfig
from boarddet.sim.sensor import Vlp32cSensor

AZIMUTH_STEPS = 360  # small width keeps the ray-cast fast in tests


def _cfg(**overrides) -> SynthDataConfig:
    scenegen = overrides.pop(
        "scenegen",
        SceneGenConfig(board_count_weights={1: 1.0}, n_clutter_range=(1, 2),
                       n_boxes_range=(0, 1), n_cylinders_range=(0, 1)),
    )
    overrides.setdefault("augment", False)
    return SynthDataConfig(scenegen=scenegen, azimuth_steps=AZIMUTH_STEPS,
                           **overrides)


# ---------------------------------------------------------------------------
# shapes / dtypes / value ranges
# ---------------------------------------------------------------------------


def test_dataset_item_shapes_and_dtypes():
    ds = SynthBoardDataset(_cfg())
    input_t, target_t = ds[0]
    assert input_t.shape == (3, 32, AZIMUTH_STEPS)
    assert target_t.shape == (1, 32, AZIMUTH_STEPS)
    assert input_t.dtype == torch.float32
    assert target_t.dtype == torch.float32


def test_dataset_len_is_configurable_virtual_size():
    ds = SynthBoardDataset(_cfg(virtual_size=42))
    assert len(ds) == 42


def test_range_and_validity_channels_are_within_unit_range():
    ds = SynthBoardDataset(_cfg())
    input_t, _target_t = ds[1]
    ch0, ch1 = input_t[0], input_t[1]
    assert torch.all((ch0 >= 0.0) & (ch0 <= 1.0))
    assert torch.all((ch1 == 0.0) | (ch1 == 1.0))


def test_target_mask_is_binary():
    ds = SynthBoardDataset(_cfg())
    _input_t, target_t = ds[2]
    vals = torch.unique(target_t)
    assert set(vals.tolist()) <= {0.0, 1.0}


# ---------------------------------------------------------------------------
# mask alignment: forced single-board scene
# ---------------------------------------------------------------------------


def test_forced_one_board_scene_mask_matches_board_hit_pixels():
    rng = np.random.default_rng(7)
    sensor = Vlp32cSensor()
    cfg = _cfg()
    input_array, target_array, scene, sim_frame = render_synth_sample(rng, cfg, sensor)
    assert len(scene.boards) == 1
    board = scene.boards[0]

    hit = sim_frame.hit_prim_id == board.prim_index
    board_points = sim_frame.points[hit]
    assert len(board_points) > 0, "the forced board produced no hits at all"
    rows, cols, _ranges = points_to_rows_cols_ranges(board_points, sensor, cfg.azimuth_steps)

    expected_mask = np.zeros_like(target_array, dtype=bool)
    expected_mask[rows, cols] = True

    assert np.array_equal(target_array.astype(bool), expected_mask)
    # every masked pixel actually has a finite normalized range in channel 0
    assert np.all(input_array[0][target_array.astype(bool)] > 0.0)


def test_empty_scene_yields_all_zero_mask():
    rng = np.random.default_rng(3)
    sensor = Vlp32cSensor()
    cfg = _cfg(scenegen=SceneGenConfig(board_count_weights={0: 1.0},
                                       n_clutter_range=(1, 2),
                                       n_boxes_range=(0, 1),
                                       n_cylinders_range=(0, 1)))
    _input_array, target_array, scene, _sim_frame = render_synth_sample(rng, cfg, sensor)
    assert len(scene.boards) == 0
    assert not target_array.any()


# ---------------------------------------------------------------------------
# augmentation: roll + flip apply identically to input and target
# ---------------------------------------------------------------------------


def test_roll_and_flip_shifts_input_and_target_together():
    rng = np.random.default_rng(11)
    sensor = Vlp32cSensor()
    cfg = _cfg()
    input_array, target_array, _scene, _sim_frame = render_synth_sample(rng, cfg, sensor)
    input_t = torch.from_numpy(input_array)
    target_t = torch.from_numpy(target_array[None, :, :])

    shift = 17
    rolled_input, rolled_target = roll_and_flip(input_t, target_t, shift, flip=False)

    expected_input = torch.roll(input_t, shifts=shift, dims=-1)
    expected_target = torch.roll(target_t, shifts=shift, dims=-1)
    assert torch.equal(rolled_input, expected_input)
    assert torch.equal(rolled_target, expected_target)

    # a mask pixel at column c before the roll must appear at (c+shift) % W after
    pre_cols = torch.nonzero(target_t[0].sum(dim=0) > 0).flatten()
    if len(pre_cols):
        post_cols = torch.nonzero(rolled_target[0].sum(dim=0) > 0).flatten()
        expected_post = torch.sort((pre_cols + shift) % target_t.shape[-1]).values
        assert torch.equal(torch.sort(post_cols).values, expected_post)


def test_roll_and_flip_flip_reverses_width_for_both_tensors():
    input_t = torch.arange(2 * 3 * 4, dtype=torch.float32).reshape(2, 3, 4)
    target_t = torch.arange(1 * 3 * 4, dtype=torch.float32).reshape(1, 3, 4)
    flipped_input, flipped_target = roll_and_flip(input_t, target_t, shift=0, flip=True)
    assert torch.equal(flipped_input, torch.flip(input_t, dims=[-1]))
    assert torch.equal(flipped_target, torch.flip(target_t, dims=[-1]))


def test_dataset_augmentation_preserves_mask_input_alignment():
    """With augmentation enabled, the returned (input, target) pair must
    still be mutually consistent: re-deriving the pre-augment sample at the
    same seed and applying the augmentation module's own roll/flip
    independently must reproduce the dataset's augmented output exactly."""
    cfg = _cfg(augment=True)
    ds = SynthBoardDataset(cfg)
    augmented_input, augmented_target = ds[5]

    # Reproduce the same draw sequence by hand: same seed, same rng order
    # (render_synth_sample consumes rng first, THEN shift/flip are drawn).
    rng = np.random.default_rng(ds._seed_for(5))
    input_array, target_array, _scene, _sim_frame = render_synth_sample(rng, cfg, ds.sensor)
    input_t = torch.from_numpy(input_array)
    target_t = torch.from_numpy(target_array[None, :, :])
    width = input_t.shape[-1]
    shift = int(rng.integers(0, width))
    flip = bool(rng.random() < 0.5)
    expected_input, expected_target = roll_and_flip(input_t, target_t, shift, flip)

    assert torch.equal(augmented_input, expected_input)
    assert torch.equal(augmented_target, expected_target)


# ---------------------------------------------------------------------------
# channels_to_input_array normalization unit tests
# ---------------------------------------------------------------------------


def test_channels_to_input_array_normalizes_range_and_clips():
    image = np.array([[[5.0, 1.0], [np.nan, np.nan], [100.0, 0.5]]], dtype=np.float32)
    out = channels_to_input_array(image, r_max=R_MAX)
    assert out.shape == (3, 1, 3)
    # column 0: range 5.0 -> 5/12
    assert np.isclose(out[0, 0, 0], 5.0 / R_MAX)
    assert out[1, 0, 0] == 1.0
    assert np.isclose(out[2, 0, 0], 1.0 / R_MAX)
    # column 1: no return -> zeros, validity 0
    assert out[0, 0, 1] == 0.0
    assert out[1, 0, 1] == 0.0
    assert out[2, 0, 1] == 0.0
    # column 2: range clipped to R_MAX/R_MAX == 1.0
    assert out[0, 0, 2] == 1.0
    assert out[1, 0, 2] == 1.0
    assert np.isclose(out[2, 0, 2], 0.5 / R_MAX)


# ---------------------------------------------------------------------------
# synth/real structural parity
# ---------------------------------------------------------------------------


def test_real_frame_to_input_matches_synth_shape_dtype_and_range():
    sensor = Vlp32cSensor()
    rng = np.random.default_rng(9)
    n = 5000
    az = rng.uniform(-np.pi, np.pi, size=n)
    el = rng.choice(sensor.elevations, size=n)
    r = rng.uniform(1.0, 10.0, size=n)
    horiz = r * np.cos(el)
    xyz = np.stack([horiz * np.cos(az), horiz * np.sin(az), r * np.sin(el)], axis=1)
    fake_frame = Frame(stamp=0.0, xyz=xyz.astype(np.float32),
                       intensity=np.zeros(n, dtype=np.float32),
                       ring=np.zeros(n, dtype=np.uint8))

    real_input = real_frame_to_input(fake_frame, sensor, azimuth_steps=AZIMUTH_STEPS)

    synth_cfg = _cfg()
    synth_ds = SynthBoardDataset(synth_cfg)
    synth_input, _synth_target = synth_ds[0]

    assert real_input.shape == synth_input.shape == (N_INPUT_CHANNELS, 32, AZIMUTH_STEPS)
    assert real_input.dtype == synth_input.dtype == torch.float32
    for name, t in (("real", real_input), ("synth", synth_input)):
        ch0, ch1, ch2 = t[0], t[1], t[2]
        assert torch.all((ch0 >= 0.0) & (ch0 <= 1.0)), f"{name} ch0 out of [0,1]"
        assert torch.all((ch1 == 0.0) | (ch1 == 1.0)), f"{name} ch1 not binary"
        assert torch.all((ch2 >= 0.0) & (ch2 <= 1.0)), f"{name} ch2 out of [0,1]"
