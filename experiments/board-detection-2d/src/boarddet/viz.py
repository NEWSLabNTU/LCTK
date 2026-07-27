"""Overlay renders for eyeballing detections.

Two renderers live here:
- `save_overlay`  -- the generator-agnostic 2-panel overlay (used by the
  a/b/c benchmark).
- `render_six_panel` + its wrappers `render_methode` / `render_noe` -- the
  full 6-panel pipeline view. The wrappers differ only in how the panel-2
  "foreground" layer is computed (background diff for Method E; RANSAC
  big-plane strip for the no-Method-E baseline).

Frame convention for the spatial panels: x = front, y = left, z = up
(right-handed). The front view projects onto y-z with the horizontal (y)
axis inverted so +y (left) renders on the left; the side view projects onto
x-z with +x (front) to the right. Headless Agg backend throughout.
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
from .candidates.cluster_after_ground import big_plane_residual  # noqa: E402
from .detector import DetectOutcome  # noqa: E402
from .geometry import downsample  # noqa: E402
from .pose import BoardDetection  # noqa: E402


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
        corners = (res.corners_2d @ res.rot_2d.T
                   if res.rot_2d is not None else res.corners_2d)
        px = (corners - res.origin) / res.cell_m
        ax.plot(np.append(px[:, 0], px[0, 0]),
                np.append(px[:, 1], px[0, 1]), "r-", lw=1.5)
        ax.set_title("plane raster + refined quad")
    else:
        ax.axis("off")
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=110)
    plt.close(fig)


# --- Shared 6-panel renderer -------------------------------------------------

# Fixed layer colors.
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

# How far beyond the reference box, in metres, the spatial panels show.
_ZOOM_MARGIN_M = 4.0

# Spatial projection panels other than the top-down mix, as
# (ai, bi, invert_h, xlabel, ylabel, title). Frame convention x=front,
# y=left, z=up: the front view is y-z with y inverted (left renders left);
# the side view is x-z with +x (front) to the right (not inverted).
_FRONT = (1, 2, True, "y [m]", "z [m]", "front (y-z)")
_SIDE = (0, 2, False, "x [m]", "z [m]", "side (x-z)")


def _box_corners_world(box: BoxRef) -> np.ndarray:
    """(8,3) world-frame corners of the reference box."""
    return box.center + (_BOX_SIGNS * box.half) @ box.rot.T


def _draw_box(ax, corners: np.ndarray, ai: int, bi: int) -> None:
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


def _set_limits(ax, box_corners: np.ndarray, ai: int, bi: int,
                invert_h: bool) -> None:
    """Zoom to the box (+margin); invert the horizontal axis when invert_h,
    so a frame direction like +y=left renders on the left instead of right."""
    lo = box_corners[:, [ai, bi]].min(axis=0) - _ZOOM_MARGIN_M
    hi = box_corners[:, [ai, bi]].max(axis=0) + _ZOOM_MARGIN_M
    if invert_h:
        ax.set_xlim(hi[0], lo[0])
    else:
        ax.set_xlim(lo[0], hi[0])
    ax.set_ylim(lo[1], hi[1])


def _proj_panel(ax, ai: int, bi: int, invert_h: bool, labels: tuple[str, str],
                title: str, raw, fg, box_corners, outcome: DetectOutcome
                ) -> None:
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
    _set_limits(ax, box_corners, ai, bi, invert_h)


def render_six_panel(dn: np.ndarray, fg: np.ndarray, box: BoxRef,
                     outcome: DetectOutcome, path: Path,
                     panel2_title: str) -> None:
    """Render the shared 6-panel pipeline view for one frame. `dn` is the
    downsampled cloud (raw layer), `fg` the pipeline's foreground layer
    (background diff for Method E, big-plane residual for no-E); `panel2_title`
    labels the foreground panel. The other five panels are identical."""
    box_corners = _box_corners_world(box)
    det = outcome.detection
    state = f"score={det.score:.2f}" if det is not None else "NO DETECTION"
    path = Path(path)
    fig, axes = plt.subplots(2, 3, figsize=(19, 11))
    try:
        # Panel 1: raw only, top-down.
        _scatter(axes[0, 0], dn, 0, 1, _C_RAW, 2.0, 0.5)
        _draw_box(axes[0, 0], box_corners, 0, 1)
        axes[0, 0].set_aspect("equal")
        axes[0, 0].set_xlabel("x [m]")
        axes[0, 0].set_ylabel("y [m]")
        axes[0, 0].set_title("raw cloud (top-down)")
        _set_limits(axes[0, 0], box_corners, 0, 1, False)

        # Panel 2: foreground only, top-down. Title carries the layer count.
        _scatter(axes[0, 1], fg, 0, 1, _C_FG, 4.0, 0.9)
        _draw_box(axes[0, 1], box_corners, 0, 1)
        axes[0, 1].set_aspect("equal")
        axes[0, 1].set_xlabel("x [m]")
        axes[0, 1].set_ylabel("y [m]")
        axes[0, 1].set_title(panel2_title)
        _set_limits(axes[0, 1], box_corners, 0, 1, False)

        # Panel 3: mix, top-down.
        _proj_panel(axes[0, 2], 0, 1, False, ("x [m]", "y [m]"),
                    f"mix (top-down) | {state}", dn, fg, box_corners, outcome)

        # Panel 4: front (y-z, horizontal inverted). Panel 5: side (x-z).
        for ax, (ai, bi, invert_h, xl, yl, title) in (
                (axes[1, 0], _FRONT), (axes[1, 1], _SIDE)):
            _proj_panel(ax, ai, bi, invert_h, (xl, yl), title,
                        dn, fg, box_corners, outcome)

        # Panel 6: plane raster + refined quad.
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

        fig.suptitle(path.stem, fontsize=12)
        fig.tight_layout()
        path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(path, dpi=100)
    finally:
        plt.close(fig)


def render_methode(frame_xyz: np.ndarray, board: BoardConfig,
                   background: BackgroundModel, outcome: DetectOutcome,
                   box: BoxRef, path: Path, voxel: float = 0.03) -> None:
    """Method E 6-panel view: foreground = background-diff of the frame."""
    dn = downsample(frame_xyz, voxel)
    fg = background.foreground_points(dn)
    render_six_panel(dn, fg, box, outcome, path,
                     f"foreground diff ({len(fg)} pts total)")


def render_noe(frame_xyz: np.ndarray, board: BoardConfig,
               outcome: DetectOutcome, box: BoxRef, path: Path,
               voxel: float = 0.03) -> None:
    """No-Method-E 6-panel view: foreground = generator B's big-plane residual
    (RANSAC-stripped ground/walls), the crop-free analog of a background diff."""
    dn = downsample(frame_xyz, voxel)
    fg = big_plane_residual(dn, board, board.vertical_gap_deg)
    render_six_panel(dn, fg, box, outcome, path,
                     f"after big-plane removal ({len(fg)} pts)")
