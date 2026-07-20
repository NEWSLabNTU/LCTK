"""Scene -> range image + point cloud, by casting the sensor's real VLP-32C
beam directions against a list of primitives and taking the nearest hit per
ray. This is the fidelity fix for Gate-2: every range-image pixel *is* a
cast ray (one per laser x azimuth-step), so there is no object-space
grid-vs-image-bin aliasing step for a moire pattern to come from (contrast
`boarddet.synth`, which samples a regular grid in surface (u, v) coordinates
and only afterwards gets re-binned into image pixels)."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .sensor import Vlp32cSensor

Primitive = object  # duck-typed: anything with .intersect(origins, dirs)


@dataclass
class SimFrame:
    range_image: np.ndarray  # (n_rows, n_cols) float32, NaN = no return
    points: np.ndarray       # (M, 3) float32, unprojected valid hits (sensor frame)
    hit_prim_id: np.ndarray  # (M,) int64, index into `scene` for each point
    rows: np.ndarray         # (M,) int64, range_image row of each point
    cols: np.ndarray         # (M,) int64, range_image col of each point
    n_rows: int
    n_cols: int
    azimuths: np.ndarray     # (n_cols,) rad, nominal azimuth of each column


def render(scene: list[Primitive], sensor: Vlp32cSensor,
          azimuth_steps: int | None = None,
          azimuth_step_deg: float | None = None,
          range_noise_std: float = 0.0,
          dropout_grazing: float = 0.0,
          dropout_random: float = 0.0,
          rng: np.random.Generator | None = None) -> SimFrame:
    """Ray-cast `scene` along `sensor`'s real beam directions.

    Resolution order per ray: nearest finite `t` among all primitives ->
    min/max range clip+reject -> gaussian range noise -> dropout (base
    random rate plus a grazing-incidence term that rises as the incidence
    cosine -> 0). Dropout is evaluated against the *pre-noise* geometry so
    it models sensor/surface physics, not the injected noise.
    """
    rng = rng if rng is not None else np.random.default_rng()

    beam_kwargs = {}
    if azimuth_steps is not None:
        beam_kwargs["azimuth_steps"] = azimuth_steps
    if azimuth_step_deg is not None:
        beam_kwargs["azimuth_step_deg"] = azimuth_step_deg
    grid = sensor.beam_directions(**beam_kwargs)

    n_rays = grid.directions.shape[0]
    origins = np.zeros((n_rays, 3), dtype=np.float64)
    dirs = grid.directions

    t_best = np.full(n_rays, np.inf)
    cos_best = np.full(n_rays, np.nan)
    prim_best = np.full(n_rays, -1, dtype=np.int64)

    for prim_idx, prim in enumerate(scene):
        t, cosang = prim.intersect(origins, dirs)
        closer = t < t_best
        t_best = np.where(closer, t, t_best)
        cos_best = np.where(closer, cosang, cos_best)
        prim_best = np.where(closer, prim_idx, prim_best)

    valid = (
        np.isfinite(t_best)
        & (t_best >= sensor.min_range)
        & (t_best <= sensor.max_range)
    )

    if dropout_grazing > 0.0 or dropout_random > 0.0:
        grazing_factor = 1.0 - np.clip(cos_best, 0.0, 1.0)
        grazing_factor = np.where(np.isnan(grazing_factor), 0.0, grazing_factor)
        p_drop = np.clip(dropout_random + dropout_grazing * grazing_factor, 0.0, 1.0)
        dropped = rng.random(n_rays) < p_drop
        valid = valid & ~dropped

    if range_noise_std > 0.0:
        t_final = t_best + rng.normal(0.0, range_noise_std, size=n_rays)
    else:
        t_final = t_best.copy()
    t_final = np.clip(t_final, sensor.min_range, sensor.max_range)

    range_image = np.full((grid.n_rows, grid.n_cols), np.nan, dtype=np.float32)
    range_image[grid.rows[valid], grid.cols[valid]] = t_final[valid].astype(np.float32)

    points = (origins[valid] + t_final[valid, None] * dirs[valid]).astype(np.float32)

    return SimFrame(
        range_image=range_image,
        points=points,
        hit_prim_id=prim_best[valid],
        rows=grid.rows[valid],
        cols=grid.cols[valid],
        n_rows=grid.n_rows,
        n_cols=grid.n_cols,
        azimuths=grid.azimuths,
    )
