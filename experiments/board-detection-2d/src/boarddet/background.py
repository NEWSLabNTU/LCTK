"""Accumulated static-scene voxel occupancy for background/foreground diffing.

Method E of `docs/roadmap/side-track_auto-bounding-box.md`: a static-mounted
sensor sees the same room every frame and only the calibration board changes
place, so voxels that are reliably occupied are background and points landing
in never-occupied voxels are foreground.

Stateful by necessity. Unlike `isolation.isolation_density` (a pure function
this module otherwise takes its shape from), a background reference is
cross-frame state -- the only such state in this package. `detect()` never
owns or mutates it; the caller does.

CONSENSUS (`min_sources`): sources are kept apart and a voxel becomes
background only once >= min_sources distinct sources have seen it. With one
source per recorded dataset, the shared static room (seen by every source)
survives while each source's own calibration board (seen by exactly one)
drops out. That is what lets a leave-one-out cross-dataset background avoid
pre-suppressing the held-out dataset's board, whose location may sit within
a board's width of a contributor's. `min_sources=1` is exactly a plain union.
"""
from __future__ import annotations

import itertools
from collections.abc import Hashable

import numpy as np

# Integer voxel indices bit-packed into one int64 key. 21 bits/axis covers
# +/- 2,097,152 cells, i.e. +/- ~125 km at the 0.06 m default voxel -- no
# LiDAR range can overflow it. _KEY_OFFSET biases indices positive so points
# behind/below the sensor pack correctly.
_KEY_BITS = 21
_KEY_MASK = (1 << _KEY_BITS) - 1
_KEY_OFFSET = 1 << (_KEY_BITS - 1)


def _pack(idx: np.ndarray) -> np.ndarray:
    """(..., 3) int64 biased voxel indices -> (...,) int64 keys."""
    return ((idx[..., 0] & _KEY_MASK)
            | ((idx[..., 1] & _KEY_MASK) << _KEY_BITS)
            | ((idx[..., 2] & _KEY_MASK) << (2 * _KEY_BITS)))


def _build_stencil(radius: int) -> np.ndarray:
    """((2r+1)^3, 3) neighbour offsets; radius 0 is the centre cell alone."""
    span = range(-radius, radius + 1)
    return np.array(list(itertools.product(span, span, span)), dtype=np.int64)


class BackgroundModel:
    """Per-source voxel occupancy with a >=min_sources consensus background.

    voxel=0.06 with dilation_radius=1 gives an effective "still counts as
    background" reach of ~0.09-0.12 m from a stored point: above the VLP-32C
    noise floor this codebase repeatedly calibrates against (0.026-0.031 m
    measured plane-fit RMS; cf. `_FLATNESS_RMS_MAX`, `COPLANAR_TOL`) and well
    below the board's own 1.0 m side. Provisional defaults -- sweep them, the
    way `flatness_rms_max` was retuned 0.035 -> 0.045 by measurement.
    """

    def __init__(self, voxel: float = 0.06, dilation_radius: int = 1,
                 min_sources: int = 2) -> None:
        self.voxel = voxel
        self.dilation_radius = dilation_radius
        self.min_sources = min_sources
        self._stencil = _build_stencil(dilation_radius)
        self._sources: dict[Hashable, np.ndarray] = {}
        self._keys: np.ndarray | None = None

    @property
    def n_voxels(self) -> int:
        return 0 if self._keys is None else int(len(self._keys))

    @property
    def n_sources(self) -> int:
        return len(self._sources)

    def _keys_of(self, points: np.ndarray) -> np.ndarray:
        idx = np.floor(
            np.asarray(points, dtype=np.float64) / self.voxel
        ).astype(np.int64) + _KEY_OFFSET
        return np.unique(_pack(idx))

    def observe(self, points: np.ndarray, source: Hashable) -> None:
        """Fold `points` into `source`'s own occupancy set.

        Callable repeatedly for one source (frame by frame or all at once --
        a set union is order- and grouping-independent). Invalidates any
        previous `finalize()`.
        """
        points = np.asarray(points)
        if len(points) == 0:
            return
        keys = self._keys_of(points)
        prev = self._sources.get(source)
        self._sources[source] = keys if prev is None else np.union1d(prev, keys)
        self._keys = None

    def finalize(self) -> None:
        """Collapse the per-source sets into the consensus background."""
        if not self._sources:
            self._keys = np.empty(0, dtype=np.int64)
            return
        stacked = np.concatenate(list(self._sources.values()))
        uniq, counts = np.unique(stacked, return_counts=True)
        self._keys = uniq[counts >= self.min_sources]

    def foreground_points(self, dn: np.ndarray) -> np.ndarray:
        """Points of `dn` whose own voxel AND every dilation-stencil
        neighbour are unoccupied in the consensus background.

        `dn` is the frame's voxel-downsampled cloud -- the same `dn`
        `detect()` builds and hands `isolation_density`.

        Dilation is applied at QUERY time, not at storage time: it keeps the
        stored set small while still absorbing the voxel-boundary aliasing
        that would otherwise report a static surface as new whenever range
        noise nudges a point across a cell edge.
        """
        dn = np.asarray(dn)
        if self._keys is None:
            raise RuntimeError(
                "BackgroundModel.finalize() must be called before "
                "foreground_points(); observe() invalidates it")
        if len(self._keys) == 0 or len(dn) == 0:
            return dn
        base = np.floor(
            dn.astype(np.float64) / self.voxel
        ).astype(np.int64) + _KEY_OFFSET
        cand = base[:, None, :] + self._stencil[None, :, :]
        flat = _pack(cand).ravel()
        # self._keys is sorted (np.unique output), so searchsorted beats
        # np.isin here and stays fully vectorized.
        pos = np.clip(np.searchsorted(self._keys, flat), 0, len(self._keys) - 1)
        hit = self._keys[pos] == flat
        is_background = hit.reshape(cand.shape[:2]).any(axis=1)
        return dn[~is_background]
