"""Labeled synthetic dataset generation (Task 30): per scene, a range image
(via the SHARED renderer, `range_image.to_range_image` -- the same code
path a real `Frame.xyz` would go through at CNN eval time) + one label per
generated diamond board (pixel mask, bbox in (row, col), and 3D pose).
Dumped to disk as one `.npz` per scene, gitignored (`results/`/`data/synth`
per project convention -- see `pyproject.toml`/`.gitignore`).
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .range_image import points_to_rows_cols_ranges, to_range_image
from .raycast import render
from .scenegen import SceneGenConfig, random_scene
from .sensor import Vlp32cSensor


@dataclass
class DatasetConfig:
    scenegen: SceneGenConfig = field(default_factory=SceneGenConfig)
    azimuth_steps: int = 1800
    channels: tuple[str, ...] = ("range", "discontinuity")
    range_noise_std: float = 0.01
    dropout_grazing: float = 0.1
    dropout_random: float = 0.01


@dataclass
class BoardLabel:
    mask: np.ndarray       # (n_rows, n_cols) bool -- this board's hit pixels
    bbox: tuple[int, int, int, int]  # (row_min, row_max, col_min, col_max), inclusive
    center: np.ndarray     # (3,)
    normal: np.ndarray     # (3,)
    side: float
    corners: np.ndarray    # (4,3)
    hollow: bool


@dataclass
class SceneSample:
    image: np.ndarray            # (n_rows, n_cols, len(channels)) float32
    boards: list[BoardLabel]
    sensor_roll_rad: float
    sensor_pitch_rad: float
    sensor_height_offset_m: float


def render_labeled_scene(rng: np.random.Generator, sensor: Vlp32cSensor,
                         cfg: DatasetConfig) -> SceneSample:
    """Build one domain-randomized scene, render it, and compute a
    per-board label. Uses the SAME shared renderer (`to_range_image`) that
    a real `Frame.xyz` would go through -- not `SimFrame.range_image`
    directly -- so the dataset a CNN trains on matches what it will see at
    real-data eval time pixel-binning-convention-for-convention."""
    scene = random_scene(rng, cfg.scenegen, sensor)
    sim_frame = render(
        scene.primitives, sensor,
        azimuth_steps=cfg.azimuth_steps,
        range_noise_std=cfg.range_noise_std,
        dropout_grazing=cfg.dropout_grazing,
        dropout_random=cfg.dropout_random,
        rng=rng,
    )
    image, grid = to_range_image(sim_frame.points, sensor,
                                 azimuth_steps=cfg.azimuth_steps,
                                 channels=cfg.channels)

    labels: list[BoardLabel] = []
    for board in scene.boards:
        hit = sim_frame.hit_prim_id == board.prim_index
        board_pts = sim_frame.points[hit]
        mask = np.zeros((grid.n_rows, grid.n_cols), dtype=bool)
        if len(board_pts):
            rows, cols, _ = points_to_rows_cols_ranges(board_pts, sensor,
                                                        cfg.azimuth_steps)
            mask[rows, cols] = True
            bbox = (int(rows.min()), int(rows.max()),
                   int(cols.min()), int(cols.max()))
        else:
            bbox = (-1, -1, -1, -1)
        labels.append(BoardLabel(
            mask=mask, bbox=bbox, center=board.center, normal=board.normal,
            side=board.side, corners=board.corners, hollow=board.hollow,
        ))

    return SceneSample(
        image=image, boards=labels,
        sensor_roll_rad=scene.sensor_roll_rad,
        sensor_pitch_rad=scene.sensor_pitch_rad,
        sensor_height_offset_m=scene.sensor_height_offset_m,
    )


def _save_scene(path: Path, sample: SceneSample) -> None:
    n_boards = len(sample.boards)
    arrays: dict[str, np.ndarray] = {
        "image": sample.image,
        "n_boards": np.array(n_boards, dtype=np.int64),
        "sensor_roll_rad": np.array(sample.sensor_roll_rad, dtype=np.float64),
        "sensor_pitch_rad": np.array(sample.sensor_pitch_rad, dtype=np.float64),
        "sensor_height_offset_m": np.array(sample.sensor_height_offset_m,
                                           dtype=np.float64),
    }
    if n_boards:
        arrays["board_mask"] = np.stack([b.mask for b in sample.boards])
        arrays["board_bbox"] = np.array([b.bbox for b in sample.boards],
                                        dtype=np.int64)
        arrays["board_center"] = np.stack([b.center for b in sample.boards])
        arrays["board_normal"] = np.stack([b.normal for b in sample.boards])
        arrays["board_side"] = np.array([b.side for b in sample.boards],
                                        dtype=np.float64)
        arrays["board_corners"] = np.stack([b.corners for b in sample.boards])
        arrays["board_hollow"] = np.array([b.hollow for b in sample.boards],
                                          dtype=bool)
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(path, **arrays)


def load_scene(path: Path) -> SceneSample:
    """Round-trip loader for `_save_scene`'s output -- reconstructs a
    `SceneSample` (image + per-board labels) from disk."""
    with np.load(path) as z:
        n_boards = int(z["n_boards"])
        boards: list[BoardLabel] = []
        for i in range(n_boards):
            boards.append(BoardLabel(
                mask=z["board_mask"][i],
                bbox=tuple(int(v) for v in z["board_bbox"][i]),
                center=z["board_center"][i],
                normal=z["board_normal"][i],
                side=float(z["board_side"][i]),
                corners=z["board_corners"][i],
                hollow=bool(z["board_hollow"][i]),
            ))
        return SceneSample(
            image=z["image"], boards=boards,
            sensor_roll_rad=float(z["sensor_roll_rad"]),
            sensor_pitch_rad=float(z["sensor_pitch_rad"]),
            sensor_height_offset_m=float(z["sensor_height_offset_m"]),
        )


def generate_dataset(n_scenes: int, out_dir: Path | str,
                     rng: np.random.Generator,
                     cfg: DatasetConfig | None = None) -> list[Path]:
    """Generate `n_scenes` labeled synthetic scenes and dump them as
    `scene_00000.npz`, `scene_00001.npz`, ... under `out_dir` (created if
    missing), plus a `manifest.json` recording the count and config. Returns
    the list of written scene paths."""
    cfg = cfg if cfg is not None else DatasetConfig()
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    sensor = Vlp32cSensor()

    paths: list[Path] = []
    for i in range(n_scenes):
        sample = render_labeled_scene(rng, sensor, cfg)
        path = out_dir / f"scene_{i:05d}.npz"
        _save_scene(path, sample)
        paths.append(path)

    manifest = {
        "n_scenes": n_scenes,
        "azimuth_steps": cfg.azimuth_steps,
        "channels": list(cfg.channels),
    }
    (out_dir / "manifest.json").write_text(json.dumps(manifest, indent=2))
    return paths
