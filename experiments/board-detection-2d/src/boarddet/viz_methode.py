"""Six-panel render of the full Method E pipeline for one frame.

Separate from viz.py (the generator-agnostic 2-panel overlay): this one
needs the background model and the per-rig bbox, and shows the
background-subtraction stages viz.py cannot. Headless Agg, like viz.py.
"""
from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from .background import BackgroundModel  # noqa: E402
from .bbox_ref import BoxRef  # noqa: E402
from .board_config import BoardConfig  # noqa: E402
from .detector import DetectOutcome  # noqa: E402
from .geometry import downsample  # noqa: E402
from .pose import BoardDetection  # noqa: E402

# Fixed layer colors (design spec).
_C_RAW = "0.6"
_C_FG = "tab:blue"
_C_CAND = "tab:orange"
_C_BBOX = "tab:green"
_C_DET = "tab:red"

# The 8 corners of a unit box in its own frame, as (sx, sy, sz) signs.
_BOX_SIGNS = np.array([[sx, sy, sz]
                       for sx in (-1, 1) for sy in (-1, 1)
                       for sz in (-1, 1)], dtype=float)
# The 12 edges of that box, as index pairs into _BOX_SIGNS.
_BOX_EDGES = [(0, 1), (0, 2), (0, 4), (1, 3), (1, 5), (2, 3),
              (2, 6), (3, 7), (4, 5), (4, 6), (5, 7), (6, 7)]


def _box_corners_world(box: BoxRef) -> np.ndarray:
    """(8,3) world-frame corners of the reference box."""
    return box.center + (_BOX_SIGNS * box.half) @ box.rot.T


def _draw_box(ax, corners: np.ndarray, ai: int, bi: int) -> None:
    """Draw the box wireframe projected onto axes (ai, bi) of world coords."""
    for i, j in _BOX_EDGES:
        ax.plot([corners[i, ai], corners[j, ai]],
                [corners[i, bi], corners[j, bi]],
                color=_C_BBOX, lw=1.0, alpha=0.9)


def _draw_quad(ax, det: BoardDetection, ai: int, bi: int, color: str) -> None:
    q = np.vstack([det.corners_3d, det.corners_3d[:1]])
    ax.plot(q[:, ai], q[:, bi], color=color, lw=2)


def _scatter(ax, pts: np.ndarray, ai: int, bi: int, color: str,
             s: float, alpha: float) -> None:
    if len(pts) == 0:
        return
    step = max(1, len(pts) // 60_000)
    ax.scatter(pts[::step, ai], pts[::step, bi], s=s, c=color, alpha=alpha)


# How far beyond the reference box, in metres, the spatial panels show. The
# board is a small object; without a zoom the panels autoscale to distant
# scattered foreground (100 m+ away) and the board region collapses to a few
# invisible pixels. Centre on the box and clip to this margin instead.
_ZOOM_MARGIN_M = 4.0


def _zoom_to_box(ax, box_corners: np.ndarray, ai: int, bi: int) -> None:
    lo = box_corners[:, [ai, bi]].min(axis=0) - _ZOOM_MARGIN_M
    hi = box_corners[:, [ai, bi]].max(axis=0) + _ZOOM_MARGIN_M
    ax.set_xlim(lo[0], hi[0])
    ax.set_ylim(lo[1], hi[1])


def _proj_panel(ax, ai: int, bi: int, labels: tuple[str, str], title: str,
                raw, fg, box_corners, outcome: DetectOutcome) -> None:
    """One orthographic projection panel with all layers, zoomed to the box."""
    _scatter(ax, raw, ai, bi, _C_RAW, 2.0, 0.35)
    _scatter(ax, fg, ai, bi, _C_FG, 4.0, 0.8)
    _draw_box(ax, box_corners, ai, bi)
    if outcome.best_rejected is not None:
        _draw_quad(ax, outcome.best_rejected, ai, bi, _C_CAND)
    if outcome.detection is not None:
        _draw_quad(ax, outcome.detection, ai, bi, _C_DET)
    ax.set_aspect("equal")
    ax.set_xlabel(labels[0])
    ax.set_ylabel(labels[1])
    ax.set_title(title)
    _zoom_to_box(ax, box_corners, ai, bi)


def render_methode(frame_xyz: np.ndarray, board: BoardConfig,
                   background: BackgroundModel, outcome: DetectOutcome,
                   box: BoxRef, path: Path, voxel: float = 0.03) -> None:
    dn = downsample(frame_xyz, voxel)
    fg = background.foreground_points(dn)
    box_corners = _box_corners_world(box)

    det = outcome.detection
    state = (f"score={det.score:.2f}" if det is not None else "NO DETECTION")

    path = Path(path)
    fig, axes = plt.subplots(2, 3, figsize=(19, 11))
    try:
        _render_panels(fig, axes, dn, fg, box_corners, outcome, det, state,
                       path.stem)
        path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(path, dpi=100)
    finally:
        plt.close(fig)


def _render_panels(fig, axes, dn, fg, box_corners, outcome, det, state,
                   suptitle):
    # Panel 1: raw only, top-down (zoomed to the box like the rest)
    _scatter(axes[0, 0], dn, 0, 1, _C_RAW, 2.0, 0.5)
    _draw_box(axes[0, 0], box_corners, 0, 1)
    axes[0, 0].set_aspect("equal")
    axes[0, 0].set_xlabel("x [m]")
    axes[0, 0].set_ylabel("y [m]")
    axes[0, 0].set_title("raw cloud (top-down)")
    _zoom_to_box(axes[0, 0], box_corners, 0, 1)

    # Panel 2: foreground only, top-down. Title carries the TOTAL foreground
    # count (whole scene), but the view is zoomed to the box.
    _scatter(axes[0, 1], fg, 0, 1, _C_FG, 4.0, 0.9)
    _draw_box(axes[0, 1], box_corners, 0, 1)
    axes[0, 1].set_aspect("equal")
    axes[0, 1].set_xlabel("x [m]")
    axes[0, 1].set_ylabel("y [m]")
    axes[0, 1].set_title(f"foreground diff ({len(fg)} pts total)")
    _zoom_to_box(axes[0, 1], box_corners, 0, 1)

    # Panel 3: mix, top-down
    _proj_panel(axes[0, 2], 0, 1, ("x [m]", "y [m]"),
                f"mix (top-down) | {state}", dn, fg, box_corners, outcome)

    # Panel 4: front x-z
    _proj_panel(axes[1, 0], 0, 2, ("x [m]", "z [m]"), "front (x-z)",
                dn, fg, box_corners, outcome)

    # Panel 5: side y-z
    _proj_panel(axes[1, 1], 1, 2, ("y [m]", "z [m]"), "side (y-z)",
                dn, fg, box_corners, outcome)

    # Panel 6: plane raster + refined quad (mirrors viz.py:38-50)
    ax = axes[1, 2]
    if det is not None:
        res = det.result
        ax.imshow(res.raster, cmap="gray", origin="lower")
        corners = (res.corners_2d @ res.rot_2d.T
                   if res.rot_2d is not None else res.corners_2d)
        px = (corners - res.origin) / res.cell_m
        ax.plot(np.append(px[:, 0], px[0, 0]),
                np.append(px[:, 1], px[0, 1]), color=_C_DET, lw=1.5)
        ax.set_title("plane raster + refined quad")
    else:
        ax.axis("off")
        ax.set_title("plane raster (no detection)")

    fig.suptitle(suptitle, fontsize=12)
    fig.tight_layout()
