"""VLP-32C beam model.

Parses the vendored VeloView-VLP-32C.yaml (32 per-laser `vert_correction`
elevation + `rot_correction` azimuth-offset pairs, radians) and generates
unit ray directions for a ray-based range-image simulator. This is the
fidelity fix for Gate-2 (see stage9-cnn-spike.md): `boarddet.synth` samples
a regular grid in object-space (u, v) plane coordinates, which aliases
against a range image's row/col binning; casting one ray per real beam
angle instead means the assembled image has no binning step at all -- each
pixel *is* a ray, by construction.

Coordinate convention (matches `boarddet.ingest`'s raw xyz and the spike's
`range_image.py`): x-forward, y-left, z-up; elevation = atan2(z,
hypot(x, y)), azimuth = atan2(y, x).
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

# sim/sensor.py -> src/boarddet/sim -> src/boarddet -> src -> board-detection-2d
_PKG_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_YAML_PATH = _PKG_ROOT / "VeloView-VLP-32C.yaml"

N_LASERS = 32
DEFAULT_AZIMUTH_STEP_DEG = 0.2
DEFAULT_MIN_RANGE = 0.9
DEFAULT_MAX_RANGE = 130.0


@dataclass
class BeamGrid:
    """Flattened (laser x azimuth-step) ray grid.

    `rows`/`cols` map each ray to its pixel in the assembled range image:
    row = laser rank sorted by elevation ascending (0 = lowest beam, 31 =
    highest -- NOT `laser_id`, which VeloView enumerates in an interleaved,
    non-monotonic order), col = azimuth-step index (0..n_cols-1, covering
    [0, 2*pi) before each laser's own `rot_correction` offset is applied).
    """

    directions: np.ndarray       # (32*n_cols, 3) unit vectors, float64
    rows: np.ndarray             # (32*n_cols,) int64, 0..31
    cols: np.ndarray             # (32*n_cols,) int64, 0..n_cols-1
    n_rows: int
    n_cols: int
    row_elevations: np.ndarray   # (32,) rad, ascending -- row i's elevation
    azimuths: np.ndarray         # (n_cols,) rad, nominal (pre-offset) azimuth of each col,
                                 # ascending and centered so col n_cols//2 ~= 0 rad
                                 # (straight ahead, +x) -- keeps a forward-facing scene's
                                 # columns contiguous instead of split across the 0/2pi seam


def _parse_yaml_lasers(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Return (vert_correction, rot_correction), each (32,) float64, in
    laser_id order (0..31) as recorded in the vendored calibration yaml."""
    with path.open() as f:
        doc = yaml.safe_load(f)
    lasers = doc["lasers"]
    if len(lasers) != N_LASERS:
        raise ValueError(
            f"expected {N_LASERS} lasers in {path}, found {len(lasers)}"
        )
    by_id = {int(entry["laser_id"]): entry for entry in lasers}
    missing = set(range(N_LASERS)) - set(by_id)
    if missing:
        raise ValueError(f"{path} is missing laser_id(s) {sorted(missing)}")
    vert = np.array([by_id[i]["vert_correction"] for i in range(N_LASERS)],
                    dtype=np.float64)
    rot = np.array([by_id[i]["rot_correction"] for i in range(N_LASERS)],
                   dtype=np.float64)
    return vert, rot


class Vlp32cSensor:
    """Real VLP-32C beam geometry, parsed once from the vendored VeloView
    calibration yaml."""

    def __init__(self, yaml_path: Path | str | None = None,
                min_range: float = DEFAULT_MIN_RANGE,
                max_range: float = DEFAULT_MAX_RANGE):
        path = Path(yaml_path) if yaml_path is not None else DEFAULT_YAML_PATH
        vert, rot = _parse_yaml_lasers(path)
        order = np.argsort(vert, kind="stable")  # ascending -> monotonic rows
        self.elevations = vert[order]     # (32,) row-sorted elevations (rad)
        self.az_offsets = rot[order]      # (32,) matching per-row az offset (rad)
        self.min_range = float(min_range)
        self.max_range = float(max_range)

    def beam_directions(self, azimuth_steps: int | None = None,
                        azimuth_step_deg: float = DEFAULT_AZIMUTH_STEP_DEG,
                        ) -> BeamGrid:
        """`azimuth_steps` (if given) takes precedence over
        `azimuth_step_deg`; otherwise n_cols = round(360 / azimuth_step_deg)."""
        if azimuth_steps is None:
            n_cols = max(1, round(360.0 / azimuth_step_deg))
        else:
            n_cols = int(azimuth_steps)

        # Center the azimuth origin on column n_cols//2 rather than column 0,
        # so a forward-facing (+x) scene's columns land contiguously in the
        # middle of the image instead of being split across the 0/2*pi wrap.
        base_az = ((np.arange(n_cols, dtype=np.float64) - n_cols // 2)
                  * (2.0 * np.pi / n_cols))

        elev = self.elevations[:, None]      # (32, 1)
        offs = self.az_offsets[:, None]      # (32, 1)
        az = base_az[None, :] + offs         # (32, n_cols)

        cos_e = np.cos(elev)                 # (32, 1), broadcasts below
        x = cos_e * np.cos(az)
        y = cos_e * np.sin(az)
        z = np.broadcast_to(np.sin(elev), az.shape)
        directions = np.stack([x, y, z], axis=-1).reshape(-1, 3)

        rows = np.repeat(np.arange(N_LASERS, dtype=np.int64), n_cols)
        cols = np.tile(np.arange(n_cols, dtype=np.int64), N_LASERS)

        return BeamGrid(
            directions=directions,
            rows=rows,
            cols=cols,
            n_rows=N_LASERS,
            n_cols=n_cols,
            row_elevations=self.elevations.copy(),
            azimuths=base_az,
        )
