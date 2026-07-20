"""Task 31: torch data pipeline feeding the CNN board detector.

Two entry points share ONE normalization code path
(`channels_to_input_array`), which is the whole point of this module:

- `SynthBoardDataset` -- on-the-fly synthetic training data: random scene
  (`boarddet.sim.scenegen.random_scene`) -> ray cast
  (`boarddet.sim.raycast.render`) -> shared range-image renderer
  (`boarddet.sim.range_image.to_range_image`) -> normalized `(3, 32, W)`
  input + `(1, 32, W)` target mask.
- `real_frame_to_input` -- the IDENTICAL input tensor from a real
  `boarddet.ingest.Frame.xyz`, through the SAME `to_range_image` call and
  the SAME `channels_to_input_array` normalization. If synth and real ever
  diverge here, a synth-trained model fails on real data for a dumb,
  structural reason having nothing to do with board detection -- see
  `range_image.py`'s module docstring for the analogous row-axis linchpin
  this module extends to pixel normalization.

Input channels (in this fixed order):
  0. normalized range: `clip(range, 0, R_MAX) / R_MAX`, 0.0 at no-return.
  1. validity: 1.0 where a return exists, else 0.0.
  2. normalized discontinuity: `clip(discontinuity, 0, R_MAX) / R_MAX`, 0.0
     where undefined (no return, or no left neighbor). Reusing `R_MAX` as
     the discontinuity clip bound is a deliberate simplification, not an
     independent tuned constant: any jump at least as large as the max
     range of interest already means "big depth jump" for a bounded-range
     detector, so nothing is lost by saturating there instead of at the
     sensor's true (much larger) max range.

Target: `(1, 32, W)` float32, the union of the scene's board masks (1.0
where ANY board's own hit points land, via `hit_prim_id == board.prim_index`
rebinned through `points_to_rows_cols_ranges` -- the identical per-board
mask logic `boarddet.sim.dataset.render_labeled_scene` uses). An empty
(0-board) scene yields an all-zero mask.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import torch
from torch.utils.data import Dataset

from ..ingest import Frame
from ..sim.range_image import (
    DEFAULT_AZIMUTH_STEPS,
    points_to_rows_cols_ranges,
    to_range_image,
)
from ..sim.raycast import SimFrame, render
from ..sim.scenegen import BoardMeta, Scene, SceneGenConfig, random_scene
from ..sim.sensor import Vlp32cSensor

R_MAX = 12.0
INPUT_CHANNELS = ("range", "discontinuity")  # fed to to_range_image
N_INPUT_CHANNELS = 3  # normalized-range, validity, normalized-discontinuity


# ---------------------------------------------------------------------------
# THE shared synth/real normalization -- the train/eval consistency linchpin.
# ---------------------------------------------------------------------------


def channels_to_input_array(image: np.ndarray, r_max: float = R_MAX) -> np.ndarray:
    """`image`: `(H, W, 2)` float32 from
    `to_range_image(..., channels=("range", "discontinuity"))` -- NaN where
    a cell has no return. Returns `(3, H, W)` float32: normalized range,
    validity, normalized discontinuity. Called identically by
    `SynthBoardDataset` and `real_frame_to_input`; do not fork this logic.
    """
    range_channel = image[..., 0]
    discontinuity_channel = image[..., 1]

    validity = np.isfinite(range_channel).astype(np.float32)

    range_clipped = np.clip(range_channel, 0.0, r_max)
    normalized_range = np.nan_to_num(range_clipped / r_max, nan=0.0).astype(np.float32)

    discontinuity_clipped = np.clip(discontinuity_channel, 0.0, r_max)
    normalized_discontinuity = np.nan_to_num(
        discontinuity_clipped / r_max, nan=0.0
    ).astype(np.float32)

    return np.stack([normalized_range, validity, normalized_discontinuity], axis=0)


def scene_target_mask(sim_frame: SimFrame, boards: list[BoardMeta],
                      sensor: Vlp32cSensor, azimuth_steps: int) -> np.ndarray:
    """`(n_rows, n_cols)` bool: union of every board's own hit pixels,
    computed the same way `boarddet.sim.dataset.render_labeled_scene`
    computes each individual `BoardLabel.mask` (via `hit_prim_id ==
    board.prim_index`, rebinned through `points_to_rows_cols_ranges`)."""
    n_rows = len(sensor.elevations)
    mask = np.zeros((n_rows, azimuth_steps), dtype=bool)
    for board in boards:
        hit = sim_frame.hit_prim_id == board.prim_index
        board_points = sim_frame.points[hit]
        if len(board_points) == 0:
            continue
        rows, cols, _ranges = points_to_rows_cols_ranges(
            board_points, sensor, azimuth_steps
        )
        mask[rows, cols] = True
    return mask


# ---------------------------------------------------------------------------
# synth: on-the-fly scene -> (input, target) sample
# ---------------------------------------------------------------------------


@dataclass
class SynthDataConfig:
    scenegen: SceneGenConfig = field(default_factory=SceneGenConfig)
    azimuth_steps: int = DEFAULT_AZIMUTH_STEPS
    range_noise_std: float = 0.01
    dropout_grazing: float = 0.1
    dropout_random: float = 0.01
    r_max: float = R_MAX
    augment: bool = True
    base_seed: int = 0
    virtual_size: int = 10000


def render_synth_sample(
    rng: np.random.Generator, cfg: SynthDataConfig, sensor: Vlp32cSensor,
) -> tuple[np.ndarray, np.ndarray, Scene, SimFrame]:
    """One scene, rendered and normalized, WITHOUT augmentation or torch
    conversion -- the numpy core `SynthBoardDataset.__getitem__` wraps, and
    what tests call directly to inspect the scene/sim_frame a sample came
    from (e.g. to check mask-vs-hit-pixel alignment).

    Returns `(input_array (3,H,W) float32, target_mask (H,W) float32 in
    {0,1}, scene, sim_frame)`.
    """
    scene = random_scene(rng, cfg.scenegen, sensor)
    sim_frame = render(
        scene.primitives, sensor,
        azimuth_steps=cfg.azimuth_steps,
        range_noise_std=cfg.range_noise_std,
        dropout_grazing=cfg.dropout_grazing,
        dropout_random=cfg.dropout_random,
        rng=rng,
    )
    image, _grid = to_range_image(
        sim_frame.points, sensor,
        azimuth_steps=cfg.azimuth_steps, channels=INPUT_CHANNELS,
    )
    input_array = channels_to_input_array(image, cfg.r_max)
    mask = scene_target_mask(sim_frame, scene.boards, sensor, cfg.azimuth_steps)
    target_array = mask.astype(np.float32)
    return input_array, target_array, scene, sim_frame


def roll_and_flip(
    input_t: torch.Tensor, target_t: torch.Tensor, shift: int, flip: bool,
) -> tuple[torch.Tensor, torch.Tensor]:
    """Apply the SAME circular azimuth-roll (along the last, width, dim)
    and the SAME optional horizontal flip to `input_t` `(C, H, W)` and
    `target_t` `(1, H, W)` -- deterministic core of the augmentation, kept
    separate from RNG draws so tests can assert exact alignment for a
    chosen `(shift, flip)` without depending on scene generation."""
    if shift:
        input_t = torch.roll(input_t, shifts=shift, dims=-1)
        target_t = torch.roll(target_t, shifts=shift, dims=-1)
    if flip:
        input_t = torch.flip(input_t, dims=[-1])
        target_t = torch.flip(target_t, dims=[-1])
    return input_t, target_t


class SynthBoardDataset(Dataset):
    """On-the-fly synthetic training set. `__len__` is a configurable
    virtual size (`cfg.virtual_size`, default 10000) -- there is no fixed
    underlying data, every `__getitem__(i)` seeds a fresh
    `np.random.default_rng` from `cfg.base_seed + epoch * virtual_size + i`
    and renders a brand-new random scene, so repeated epochs see new scenes
    once `set_epoch` advances past the previous epoch's index range."""

    def __init__(self, cfg: SynthDataConfig | None = None,
                sensor: Vlp32cSensor | None = None):
        self.cfg = cfg if cfg is not None else SynthDataConfig()
        self.sensor = sensor if sensor is not None else Vlp32cSensor()
        self.epoch = 0

    def __len__(self) -> int:
        return self.cfg.virtual_size

    def set_epoch(self, epoch: int) -> None:
        """Advance the seed salt so a new epoch draws different scenes for
        the same index `i` (DataLoader workers/shuffling otherwise reuse
        the same `i -> scene` mapping every epoch)."""
        self.epoch = int(epoch)

    def _seed_for(self, index: int) -> int:
        return self.cfg.base_seed + self.epoch * self.cfg.virtual_size + index

    def __getitem__(self, index: int) -> tuple[torch.Tensor, torch.Tensor]:
        rng = np.random.default_rng(self._seed_for(index))
        input_array, target_array, _scene, _sim_frame = render_synth_sample(
            rng, self.cfg, self.sensor
        )
        input_t = torch.from_numpy(input_array)
        target_t = torch.from_numpy(target_array[None, :, :])

        if self.cfg.augment:
            width = input_t.shape[-1]
            shift = int(rng.integers(0, width))
            flip = bool(rng.random() < 0.5)
            input_t, target_t = roll_and_flip(input_t, target_t, shift, flip)

        return input_t, target_t


# ---------------------------------------------------------------------------
# real: Frame.xyz -> the IDENTICAL input tensor
# ---------------------------------------------------------------------------


def real_frame_to_input(
    frame: Frame, sensor: Vlp32cSensor,
    azimuth_steps: int = DEFAULT_AZIMUTH_STEPS, r_max: float = R_MAX,
) -> torch.Tensor:
    """The train/eval consistency linchpin: a real `Frame.xyz` through the
    SAME `to_range_image` call and the SAME `channels_to_input_array`
    normalization `SynthBoardDataset` uses, so a synth-trained model sees
    pixel-normalization-identical input at real-data eval time. Returns
    `(3, 32, W)` float32, no augmentation (eval-time only)."""
    image, _grid = to_range_image(
        frame.xyz, sensor, azimuth_steps=azimuth_steps, channels=INPUT_CHANNELS,
    )
    input_array = channels_to_input_array(image, r_max)
    return torch.from_numpy(input_array)
