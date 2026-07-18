"""Shared 2D scorer: coarse quad from raw points -> refined corners; a
separate closed occupancy raster (rotated to gravity "up" when anisotropic)
is used only to measure fill_ratio."""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from .board_config import BoardConfig

_MIN_POINTS = 60


@dataclass
class ScoreResult:
    score: float
    # (4,2) CCW order, ORIGINAL plane coords. Isotropic path: TLS-refined
    # (_refine_sides). Anisotropic path: raw-point minAreaRect coarse quad,
    # unrefined -- see score_candidate for why refine is skipped there.
    corners_2d: np.ndarray
    side_lengths: np.ndarray  # (4,)
    fill_ratio: float
    angle_err_deg: float
    raster: np.ndarray        # uint8 debug image (rotated frame when rot_2d set)
    origin: np.ndarray        # (2,) raster-frame coords of raster pixel (0,0)
    cell_m: float              # raster cell size used (board.cell_m)
    # (2,2) original plane coords -> rotated (up_2d-along-+y) raster frame.
    # None when the isotropic (stage-3) path was used.
    rot_2d: np.ndarray | None = None


def _rasterize(coords_2d: np.ndarray, cell: float):
    origin = coords_2d.min(axis=0) - 2 * cell
    ij = np.floor((coords_2d - origin) / cell).astype(np.int32)
    h, w = ij[:, 1].max() + 3, ij[:, 0].max() + 3
    img = np.zeros((h, w), dtype=np.uint8)
    img[ij[:, 1], ij[:, 0]] = 255
    return img, origin


def _px_to_plane(pts_px: np.ndarray, origin: np.ndarray, cell: float):
    return origin + (pts_px + 0.5) * cell


def _refine_sides(coords_2d, quad_plane, cell):
    """TLS line fit per side on raw points near it, intersect adjacent lines.

    Only used on the isotropic (stage-3) path, where `quad_plane` comes from
    a raster closed with the small 5x5 square kernel: that quad sits within
    a cell or two of the true edge (not bulged -- the square kernel isn't
    directionally biased), so a tight 2.5*cell search band safely refines it
    with raw, non-quantized points without pulling in an adjacent edge's
    points near a corner.

    Not used on the anisotropic path: there the coarse quad is already fit
    directly to raw points (see score_candidate) and this fit's own
    per-side averaging is systematically biased -- see the comment at its
    call site.
    """
    tol = 2.5 * cell
    lines = []
    for i in range(4):
        a, b = quad_plane[i], quad_plane[(i + 1) % 4]
        ab = b - a
        length = np.linalg.norm(ab)
        t = (coords_2d - a) @ ab / length**2
        perp = np.abs(np.cross(np.append(ab / length, 0.0),
                               np.c_[coords_2d - a, np.zeros(len(coords_2d))]
                               )[:, 2])
        near = (perp < tol) & (t > 0.1) & (t < 0.9)
        side_pts = coords_2d[near]
        if len(side_pts) < 5:
            return None
        centroid = side_pts.mean(axis=0)
        _, _, vt = np.linalg.svd(side_pts - centroid, full_matrices=False)
        lines.append((centroid, vt[0]))  # point + direction
    corners = []
    for i in range(4):
        (p1, d1), (p2, d2) = lines[i - 1], lines[i]
        m = np.array([d1, -d2]).T
        if abs(np.linalg.det(m)) < 1e-9:
            return None
        s = np.linalg.solve(m, p2 - p1)
        corners.append(p1 + s[0] * d1)
    return np.array(corners)


def _closing_kernel(cell: float, close_height_m: float | None) -> np.ndarray:
    """3-px-wide kernel; tall along the rotated-up axis when close_height_m
    is given, otherwise the stage-3 5x5 square."""
    if close_height_m is None:
        return np.ones((5, 5), np.uint8)
    height = int(np.ceil(close_height_m / cell))
    height = max(height, 5)
    if height % 2 == 0:
        height += 1
    height = min(height, 51)
    return np.ones((height, 3), np.uint8)


def score_candidate(coords_2d: np.ndarray, board: BoardConfig,
                    up_2d: np.ndarray | None = None,
                    close_height_m: float | None = None) -> ScoreResult | None:
    if len(coords_2d) < _MIN_POINTS:
        return None
    cell = board.cell_m
    anisotropic = up_2d is not None and close_height_m is not None
    if anisotropic:
        a, b = up_2d
        rot_2d = np.array([[b, -a], [a, b]])
        work_coords = coords_2d @ rot_2d.T
    else:
        rot_2d = None
        work_coords = coords_2d

    # `img`/`closed` (raster, in the work frame, closed with the tall
    # gravity-oriented kernel when anisotropic) exist to compute fill_ratio
    # below -- bridging ring-stripe gaps so a real, sparsely-seen board reads
    # as filled. On the anisotropic path they must never drive corner
    # geometry: closing with a kernel elongated along "up" is extensive
    # (only ever grows the shape, and is not exactly invertible for a
    # tapered region), so near a diamond's pointed corners it permanently
    # bulges the closed blob outward along the up axis. A coarse quad fit to
    # that bulge is offset by up to ~half the kernel height, and would
    # poison _refine_sides into pulling in points from the adjacent edge.
    img, origin = _rasterize(work_coords, cell)
    if img.shape[0] > 4000 or img.shape[1] > 4000:
        return None  # candidate far larger than any board
    kernel = _closing_kernel(cell, close_height_m if anisotropic else None)
    closed = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)

    if anisotropic:
        # Coarse quad: fit directly to the RAW point set, in the ORIGINAL
        # (un-rotated) plane frame -- no rotation needed for this step at
        # all. cv2.minAreaRect on a point set is bulge-free: even sparse
        # ring-stripe endpoints still sit on all four true edges, so the
        # coarse box is ~the true board rectangle regardless of gaps.
        rect = cv2.minAreaRect(coords_2d.astype(np.float32))  # ((cx,cy),(w,h),angle)
        (rw, rh) = rect[1]
        if min(rw, rh) < 3 * cell:
            return None
        quad_plane = cv2.boxPoints(rect)
    else:
        # Isotropic (stage-3) path: unchanged from pre-Task-16. The 5x5
        # square kernel isn't directionally biased, so fitting the coarse
        # quad to its closed contour is safe here and this branch is pinned
        # byte-identical to pre-Task-16 output (see
        # test_isotropic_path_byte_identical_to_pre_task16).
        contours, _ = cv2.findContours(
            closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        contour = max(contours, key=cv2.contourArea)
        rect = cv2.minAreaRect(contour)  # ((cx,cy),(w,h),angle) in px
        (rw, rh) = rect[1]
        if min(rw, rh) < 3:
            return None
        quad_px = cv2.boxPoints(rect)
        quad_plane = _px_to_plane(quad_px, origin, cell)

    # size gate on the coarse quad
    sides = np.linalg.norm(np.roll(quad_plane, -1, axis=0) - quad_plane,
                           axis=1)
    if not (board.side_m * (1 - 2 * board.side_tol)
            < sides.mean()
            < board.side_m * (1 + 2 * board.side_tol)):
        return None

    if anisotropic:
        # _refine_sides fits each side to the mean of raw points within a
        # perpendicular band of the coarse edge. That average is unbiased
        # only when points straddle the true edge on both sides; a solid
        # scanned surface never has points beyond its own true edge, so the
        # band is one-sided and the mean sits systematically ~tol/2 INSIDE
        # the true edge (confirmed empirically: refining a raw-point
        # minAreaRect quad here made corners *worse*, not better). That bias
        # was previously masked on the isotropic path because the raster's
        # closing kernel dilates the coarse quad outward by a similar
        # amount first, so the two roughly cancelled -- a coincidence, not a
        # property to rely on. The anisotropic coarse quad is fit directly
        # to raw points (no dilation to cancel against), so it is already
        # the more accurate estimate; skip refine and use it as-is.
        corners = quad_plane
    else:
        refined = _refine_sides(coords_2d, quad_plane, cell)
        corners = refined if refined is not None else quad_plane
    sides = np.linalg.norm(np.roll(corners, -1, axis=0) - corners, axis=1)

    # angles at each corner
    angs = []
    for i in range(4):
        e1 = corners[(i + 1) % 4] - corners[i]
        e2 = corners[i - 1] - corners[i]
        cosang = e1 @ e2 / (np.linalg.norm(e1) * np.linalg.norm(e2))
        angs.append(np.degrees(np.arccos(np.clip(cosang, -1, 1))))
    angle_err = float(np.mean(np.abs(np.array(angs) - 90.0)))

    # fill ratio: fraction of raster cells inside the quad that are occupied.
    # `closed` lives in the work (rotated, when anisotropic) frame, so the
    # quad has to be mapped there too.
    corners_work = corners @ rot_2d.T if anisotropic else corners
    mask = np.zeros_like(closed)
    quad_px_int = np.round(
        (corners_work - origin) / cell - 0.5).astype(np.int32)
    cv2.fillPoly(mask, [quad_px_int], 255)
    inside = mask > 0
    fill = float((closed[inside] > 0).mean()) if inside.any() else 0.0

    side_err = (float(np.std(sides) / np.mean(sides))
                + abs(float(np.mean(sides)) - board.side_m) / board.side_m)
    if abs(float(np.mean(sides)) - board.side_m) > board.side_tol * board.side_m:
        return None
    # The recorded board is a hollow diamond (3 punched holes, see
    # board_detector.json5's hole_radius/hole_center_shift): even a perfect,
    # fully-observed fit never reaches fill_ratio ~= 1, and on real VLP-32C
    # returns ring-gap sparsity pushes it well below the holes-only ceiling
    # (observed as low as ~0.44 on frames where the outer-border fit and
    # side lengths are otherwise excellent). A linear fill weight then
    # rejected genuine board fits below min_score. sqrt(fill) keeps fill
    # discriminative against low-fill non-board clutter while not punishing
    # the board's structural (not-a-defect) hole area as hard as a linear
    # term does. side_err's weight is loosened to match (-4 -> -2) since it
    # was tuned against the old, more fill-dominated score scale.
    score = float(np.sqrt(fill)) * float(np.exp(-2.0 * side_err)) \
        * float(np.exp(-angle_err / 15.0))

    # CCW order
    c = corners.mean(axis=0)
    order = np.argsort(np.arctan2(*(corners - c).T[::-1]))
    corners = corners[order]
    sides = np.linalg.norm(np.roll(corners, -1, axis=0) - corners, axis=1)

    return ScoreResult(score=float(score), corners_2d=corners,
                       side_lengths=sides, fill_ratio=fill,
                       angle_err_deg=angle_err, raster=closed, origin=origin,
                       cell_m=cell, rot_2d=rot_2d)
