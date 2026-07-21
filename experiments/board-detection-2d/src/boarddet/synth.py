"""Synthetic scenes with a known diamond board pose, for tests + benchmarks."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class SceneTruth:
    center: np.ndarray   # (3,)
    normal: np.ndarray   # (3,) unit
    corners: np.ndarray  # (4,3) top, right, bottom, left


def _plane_basis(normal: np.ndarray, up_hint: np.ndarray):
    n = normal / np.linalg.norm(normal)
    v = up_hint - (up_hint @ n) * n  # in-plane "up"
    v = v / np.linalg.norm(v)
    u = np.cross(v, n)
    return u, v, n


def _sample_plane_patch(extent_u, extent_v, spacing, pattern, rng):
    """2D sample coordinates covering [-eu,eu]x[-ev,ev]."""
    if pattern == "grid":
        us = np.arange(-extent_u, extent_u, spacing)
        vs = np.arange(-extent_v, extent_v, spacing)
        uu, vv = np.meshgrid(us, vs)
        return np.stack([uu.ravel(), vv.ravel()], axis=1)
    if pattern == "uniform":
        area = 4.0 * extent_u * extent_v
        count = int(area / spacing**2)
        return rng.uniform(
            [-extent_u, -extent_v], [extent_u, extent_v], size=(count, 2)
        )
    raise ValueError(f"unknown pattern: {pattern}")


def make_board(side, center, normal, up_hint, spacing, noise, rng,
               pattern="grid", holes=None):
    """holes: optional list of ((hu, hv), radius) circular cutouts in the
    board's own (u, v) plane coords, modelling the recorded hollow-diamond
    board's punched holes (see board_detector.json5 hole_radius /
    hole_center_shift)."""
    u, v, n = _plane_basis(np.asarray(normal, float), np.asarray(up_hint, float))
    center = np.asarray(center, float)
    half_diag = side / np.sqrt(2.0)
    coords = _sample_plane_patch(half_diag, half_diag, spacing, pattern, rng)
    # diamond: |u| + |v| <= half_diag (square rotated 45 deg)
    inside = np.abs(coords[:, 0]) + np.abs(coords[:, 1]) <= half_diag
    coords = coords[inside]
    if holes:
        keep = np.ones(len(coords), dtype=bool)
        for (hu, hv), radius in holes:
            d = np.hypot(coords[:, 0] - hu, coords[:, 1] - hv)
            keep &= d > radius
        coords = coords[keep]
    pts = center + coords[:, :1] * u + coords[:, 1:] * v
    pts = pts + rng.normal(0.0, noise, size=(len(pts), 1)) * n
    corners = np.stack([
        center + half_diag * v,   # top
        center + half_diag * u,   # right
        center - half_diag * v,   # bottom
        center - half_diag * u,   # left
    ])
    return pts.astype(np.float32), SceneTruth(center=center, normal=n,
                                              corners=corners)


def make_scene(board_side=1.0, board_center=(4.0, 0.5, 0.3), spacing=0.03,
               noise=0.01, pattern="grid", rng=None, include_board=True):
    """include_board=False returns the same static geometry (ground, wall,
    clutter blob) with no board and truth=None -- the "background" half of a
    background/reveal pair for Method E's tests. Ground/wall/blob placement
    is fixed regardless of seed (only sampling and noise use rng), so two
    calls with different seeds differ only by noise, like two consecutive
    rotations over a static scene."""
    rng = rng if rng is not None else np.random.default_rng()
    parts = []
    truth = None
    if include_board:
        board_pts, truth = make_board(
            side=board_side,
            center=np.asarray(board_center, float),
            normal=np.array([-1.0, 0.15, 0.05]),
            up_hint=np.array([0.0, 0.0, 1.0]),
            spacing=spacing,
            noise=noise,
            rng=rng,
            pattern=pattern,
        )
        parts.append(board_pts)
    # ground plane z = -1, 12x12 m
    g = _sample_plane_patch(6.0, 6.0, spacing * 3, pattern, rng)
    ground = np.stack([g[:, 0] + 4.0, g[:, 1], np.full(len(g), -1.0)], axis=1)
    parts.append((ground + rng.normal(0, noise, ground.shape))
                 .astype(np.float32))
    # wall x = 8, 12 m wide, 3 m tall
    w = _sample_plane_patch(6.0, 1.5, spacing * 3, pattern, rng)
    wall = np.stack([np.full(len(w), 8.0), w[:, 0], w[:, 1] + 0.5], axis=1)
    parts.append((wall + rng.normal(0, noise, wall.shape)).astype(np.float32))
    # clutter: box-ish blob (not planar) near the board
    blob = rng.normal([3.0, -2.0, 0.0], [0.3, 0.3, 0.5], size=(800, 3))
    parts.append(blob.astype(np.float32))
    return np.concatenate(parts).astype(np.float32), truth
