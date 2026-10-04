"""Target corner geometry shared by calibration and offline validation."""

import numpy as np
from scipy.spatial.transform import Rotation


def transform_marker_corners(local_corners, position, quaternion, pixels):
    """Return four world corners and observed pixels in their original order.

    The board quaternion is (x, y, z, w), normalized before rotation. Marker
    selection and capture admission belong to callers.
    """
    local = np.asarray(local_corners, dtype=np.float64)
    position = np.asarray(position, dtype=np.float64)
    quaternion = np.asarray(quaternion, dtype=np.float64)
    pixels = np.asarray(pixels, dtype=np.float64)
    if (
        local.shape != (4, 3)
        or position.shape != (3,)
        or quaternion.shape != (4,)
        or pixels.shape != (4, 2)
        or not all(
            np.all(np.isfinite(item)) for item in (local, position, quaternion, pixels)
        )
    ):
        raise ValueError(
            "Marker correspondence requires finite four corners and a board pose"
        )
    norm = float(np.linalg.norm(quaternion))
    if norm < 1e-12 or not np.isfinite(norm):
        raise ValueError("Board quaternion must have a finite nonzero norm")
    world = (Rotation.from_quat(quaternion / norm).as_matrix() @ local.T).T + position
    if not np.all(np.isfinite(world)):
        raise ValueError("Transformed marker corners must be finite")
    return world, pixels.copy()
