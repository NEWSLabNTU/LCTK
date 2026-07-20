"""Shared range-image renderer -- THE fix for Task 29's review-flagged
row-semantics linchpin.

`boarddet.sim.raycast.render` assembles a range image whose rows are, by
construction, the sensor's 32 REAL per-channel elevations (each ray *is* one
laser). A real decoded point cloud (`boarddet.ingest.Frame.xyz`) has no such
built-in row structure -- it is just (x, y, z). If the CNN's synth-training
renderer and its real-eval renderer used two different row conventions
(e.g. the Stage-9 spike's *uniform* 32-bin elevation split), sim and real
would disagree on what "row 17" means, and a synth-trained model would fail
on real data for a dumb, structural reason having nothing to do with board
detection.

This module is the ONE renderer used for both: every point (sim or real) is
assigned to its NEAREST REAL CHANNEL (`sensor.elevations`, the same 32
sorted elevations `boarddet.sim.sensor.Vlp32cSensor` casts rays along) by
elevation angle, and to an azimuth bin by `atan2(y, x)`. Column binning uses
the identical centered convention as `Vlp32cSensor.beam_directions` (column
`n_cols // 2` is straight ahead, +x) so a forward-facing scene's columns land
contiguously instead of splitting across the 0/2*pi wrap seam.

Row/column semantics, spelled out precisely:
- ROW is exact and unambiguous for sim data: a ray's true elevation is
  *exactly* `sensor.elevations[row]` by construction (see
  `test_sensor_beam_grid_rows_match_sorted_elevation_rank` in
  `tests/test_sim.py`), so nearest-channel binning recovers that same row
  with zero ambiguity, whether we start from `SimFrame.rows` or re-derive it
  from `SimFrame.points`. This identity is what makes "sim and real share a
  row axis" a hard, load-bearing guarantee rather than an approximation.
- COLUMN has one genuine subtlety: `Vlp32cSensor.beam_directions` assigns
  each ray a *nominal* azimuth-step column, then adds that laser's own
  `rot_correction` offset (up to ~4.2 deg) to the ray's true azimuth for
  physical realism -- the ray's true 3D azimuth therefore differs slightly
  from its nominal column's azimuth. `raycast.render` stores each ray's
  value at its *nominal* column index (`SimFrame.cols`), matching how a
  real packet stream would be laid out by firing sequence. This renderer
  instead re-derives the column purely from a point's OWN geometric azimuth
  (`atan2(y, x)`) -- the only information a real, already-decoded point
  cloud ever offers (there is no "nominal firing index" to recover from
  `Frame.xyz`). The net effect: rebinning a `SimFrame`'s own points through
  this renderer reproduces `SimFrame.range_image` up to a small, constant
  per-row column shift (== `round(rot_correction[row] / azimuth_step)`,
  bounded by ~offset_max/step) -- never a per-ray shuffle, because
  `rot_correction` depends only on the row (laser), so every ray in a given
  row shifts by the identical integer number of columns. Verified directly
  in `tests/test_range_image.py`.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .sensor import Vlp32cSensor

DEFAULT_AZIMUTH_STEPS = 1800  # 0.2 deg/col over the full 360 deg, matching
                              # sensor.py's DEFAULT_AZIMUTH_STEP_DEG


@dataclass
class RangeImageGrid:
    """Metadata describing the shared row/column axes of a rendered image."""

    n_rows: int
    n_cols: int
    row_elevations: np.ndarray  # (n_rows,) rad, ascending -- THE shared row axis
    azimuths: np.ndarray        # (n_cols,) rad, nominal per-column azimuth,
                                # centered so col n_cols // 2 ~= 0 (+x, straight ahead)


def _nominal_azimuths(n_cols: int) -> np.ndarray:
    step = 2.0 * np.pi / n_cols
    return (np.arange(n_cols, dtype=np.float64) - n_cols // 2) * step


def elevation_to_row(elevation: np.ndarray,
                     row_elevations: np.ndarray) -> np.ndarray:
    """Nearest-real-channel row assignment: for each elevation angle (rad),
    the index into `row_elevations` (ascending) whose value is closest.

    `row_elevations` must be sorted ascending (as `Vlp32cSensor.elevations`
    is). Vectorized; ties (exactly equidistant) resolve to the lower row.
    """
    elevation = np.asarray(elevation, dtype=np.float64)
    idx_hi = np.searchsorted(row_elevations, elevation)
    idx_hi = np.clip(idx_hi, 1, len(row_elevations) - 1)
    idx_lo = idx_hi - 1
    d_lo = np.abs(elevation - row_elevations[idx_lo])
    d_hi = np.abs(elevation - row_elevations[idx_hi])
    return np.where(d_lo <= d_hi, idx_lo, idx_hi).astype(np.int64)


def azimuth_to_col(azimuth: np.ndarray, n_cols: int) -> np.ndarray:
    """Azimuth (rad, any range) -> column index, using the SAME centered
    convention as `Vlp32cSensor.beam_directions` (col `n_cols // 2` is
    straight ahead / +x)."""
    azimuth = np.asarray(azimuth, dtype=np.float64)
    step = 2.0 * np.pi / n_cols
    # wrap into (-pi, pi] before binning so azimuths near +/-pi don't
    # round to an out-of-range column
    wrapped = (azimuth + np.pi) % (2.0 * np.pi) - np.pi
    col = np.round(wrapped / step).astype(np.int64) + n_cols // 2
    return np.mod(col, n_cols)


def points_to_rows_cols_ranges(points: np.ndarray, sensor: Vlp32cSensor,
                                azimuth_steps: int = DEFAULT_AZIMUTH_STEPS,
                                ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Map (N, 3) points (sensor-frame xyz) to (rows, cols, ranges), each
    (N,). `rows` uses `sensor.elevations` (nearest real channel); `cols`
    uses the centered azimuth convention shared with `sensor.py`."""
    points = np.asarray(points, dtype=np.float64)
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    horiz = np.hypot(x, y)
    elevation = np.arctan2(z, horiz)
    azimuth = np.arctan2(y, x)
    ranges = np.sqrt(horiz**2 + z**2)
    rows = elevation_to_row(elevation, sensor.elevations)
    cols = azimuth_to_col(azimuth, azimuth_steps)
    return rows, cols, ranges


def _horizontal_discontinuity(range_channel: np.ndarray) -> np.ndarray:
    """Absolute range jump to the left neighbor column; NaN where either
    cell has no return, and in column 0 (no left neighbor)."""
    shifted = np.full_like(range_channel, np.nan)
    shifted[:, 1:] = range_channel[:, :-1]
    return np.abs(range_channel - shifted)


def to_range_image(points: np.ndarray, sensor: Vlp32cSensor,
                   azimuth_steps: int = DEFAULT_AZIMUTH_STEPS,
                   channels: tuple[str, ...] = ("range",),
                   ) -> tuple[np.ndarray, RangeImageGrid]:
    """THE shared renderer: bins arbitrary (N, 3) points -- from a real
    `Frame.xyz` OR a sim `SimFrame.points` -- onto the sensor's 32 real
    channels (rows) + an azimuth grid (cols).

    Returns `(image, grid)`: `image` is `(n_rows, n_cols, len(channels))`
    float32, NaN where no point landed in a cell; `channels` may include
    "range" (nearest/closest return per cell) and "discontinuity"
    (abs range jump to the left neighbor column, NaN at either end).
    """
    if not channels:
        raise ValueError("channels must be non-empty")
    n_rows = len(sensor.elevations)
    n_cols = int(azimuth_steps)
    grid = RangeImageGrid(
        n_rows=n_rows, n_cols=n_cols,
        row_elevations=sensor.elevations.copy(),
        azimuths=_nominal_azimuths(n_cols),
    )

    image = np.full((n_rows, n_cols, len(channels)), np.nan, dtype=np.float32)
    if len(points) == 0:
        return image, grid

    rows, cols, ranges = points_to_rows_cols_ranges(points, sensor, n_cols)

    range_channel = np.full((n_rows, n_cols), np.inf, dtype=np.float64)
    np.minimum.at(range_channel, (rows, cols), ranges)
    range_channel[np.isinf(range_channel)] = np.nan

    for c, name in enumerate(channels):
        if name == "range":
            image[..., c] = range_channel.astype(np.float32)
        elif name == "discontinuity":
            image[..., c] = _horizontal_discontinuity(range_channel).astype(np.float32)
        else:
            raise ValueError(f"unknown channel: {name!r}")
    return image, grid


def sim_frame_to_range_image(simframe, sensor: Vlp32cSensor,
                             azimuth_steps: int | None = None,
                             channels: tuple[str, ...] = ("range",),
                             ) -> tuple[np.ndarray, RangeImageGrid]:
    """Convenience: run a `raycast.SimFrame`'s own points back through the
    shared renderer (rather than reading `SimFrame.range_image` directly),
    so sim output goes through the exact same code path a real `Frame.xyz`
    would. Defaults `azimuth_steps` to the frame's own `n_cols` so the
    result is directly comparable to `SimFrame.range_image` (see the
    module docstring for the expected, bounded per-row column-shift
    relationship between the two)."""
    n_cols = azimuth_steps if azimuth_steps is not None else simframe.n_cols
    return to_range_image(simframe.points, sensor, azimuth_steps=n_cols,
                          channels=channels)
