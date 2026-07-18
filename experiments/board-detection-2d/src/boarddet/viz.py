"""Overlay renders for eyeballing detections."""
from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from .detector import DetectOutcome  # noqa: E402


def save_overlay(points: np.ndarray, outcome: DetectOutcome,
                 path: Path) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(14, 7))
    ax = axes[0]
    step = max(1, len(points) // 60_000)
    ax.scatter(points[::step, 0], points[::step, 1], s=0.5, c="gray",
               alpha=0.4)
    det = outcome.detection
    if det is not None:
        quad = np.vstack([det.corners_3d, det.corners_3d[:1]])
        ax.plot(quad[:, 0], quad[:, 1], "r-", lw=2)
        ax.plot(det.center[0], det.center[1], "r+", ms=12)
        ax.set_title(f"top-down | score={det.score:.2f}")
    else:
        ax.set_title("top-down | NO DETECTION")
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")

    ax = axes[1]
    if det is not None:
        res = det.result
        ax.imshow(res.raster, cmap="gray", origin="lower")
        # `res.raster`/`res.origin` live in the rotated (up_2d-along-+y)
        # frame when rot_2d is set (Task 16); map the original-plane-frame
        # corners into that frame before converting to raster px.
        corners = (res.corners_2d @ res.rot_2d.T
                  if res.rot_2d is not None else res.corners_2d)
        cell = (corners - res.origin)  # raster-frame coords -> px
        px = cell / res.cell_m
        ax.plot(np.append(px[:, 0], px[0, 0]),
                np.append(px[:, 1], px[0, 1]), "r-", lw=1.5)
        ax.set_title("plane raster + refined quad")
    else:
        ax.axis("off")
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=110)
    plt.close(fig)
