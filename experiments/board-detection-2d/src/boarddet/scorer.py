"""Shared 2D scorer: occupancy raster -> contour -> quad -> refined corners."""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from .board_config import BoardConfig

_MIN_POINTS = 60


@dataclass
class ScoreResult:
    score: float
    corners_2d: np.ndarray   # (4,2) refined, CCW order
    side_lengths: np.ndarray  # (4,)
    fill_ratio: float
    angle_err_deg: float
    raster: np.ndarray        # uint8 debug image
    origin: np.ndarray        # (2,) plane coords of raster pixel (0,0)
    cell_m: float              # raster cell size used (board.cell_m)


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
    """TLS line fit per side on raw points near it, intersect adjacent lines."""
    lines = []
    for i in range(4):
        a, b = quad_plane[i], quad_plane[(i + 1) % 4]
        ab = b - a
        length = np.linalg.norm(ab)
        t = (coords_2d - a) @ ab / length**2
        perp = np.abs(np.cross(np.append(ab / length, 0.0),
                               np.c_[coords_2d - a, np.zeros(len(coords_2d))]
                               )[:, 2])
        near = (perp < 2.5 * cell) & (t > 0.1) & (t < 0.9)
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


def score_candidate(coords_2d: np.ndarray,
                    board: BoardConfig) -> ScoreResult | None:
    if len(coords_2d) < _MIN_POINTS:
        return None
    cell = board.cell_m
    img, origin = _rasterize(coords_2d, cell)
    if img.shape[0] > 4000 or img.shape[1] > 4000:
        return None  # candidate far larger than any board
    closed = cv2.morphologyEx(
        img, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(
        closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    rect = cv2.minAreaRect(contour)  # ((cx,cy),(w,h),angle)
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

    # fill ratio: fraction of raster cells inside the quad that are occupied
    mask = np.zeros_like(closed)
    quad_px_int = np.round(
        (corners - origin) / cell - 0.5).astype(np.int32)
    cv2.fillPoly(mask, [quad_px_int], 255)
    inside = mask > 0
    fill = float((closed[inside] > 0).mean()) if inside.any() else 0.0

    side_err = (float(np.std(sides) / np.mean(sides))
                + abs(float(np.mean(sides)) - board.side_m) / board.side_m)
    if abs(float(np.mean(sides)) - board.side_m) > board.side_tol * board.side_m:
        return None
    score = fill * float(np.exp(-4.0 * side_err)) \
        * float(np.exp(-angle_err / 15.0))

    # CCW order
    c = corners.mean(axis=0)
    order = np.argsort(np.arctan2(*(corners - c).T[::-1]))
    corners = corners[order]
    sides = np.linalg.norm(np.roll(corners, -1, axis=0) - corners, axis=1)

    return ScoreResult(score=float(score), corners_2d=corners,
                       side_lengths=sides, fill_ratio=fill,
                       angle_err_deg=angle_err, raster=closed, origin=origin,
                       cell_m=cell)
