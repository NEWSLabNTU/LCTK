"""Ray-intersectable scene primitives for the ray-based LiDAR simulator.

Every primitive exposes `intersect(origins, dirs) -> (t, cos_incidence)`,
vectorized over all rays: `origins`/`dirs` are (n, 3) arrays (`dirs` unit
vectors), `t` is (n,) float64 with `np.inf` where the ray misses, and
`cos_incidence` is (n,) the absolute cosine of the angle between the ray
and the surface normal at the hit point (1.0 = straight-on, 0.0 = grazing;
undefined/NaN where there is no hit). Nearest-hit selection across
multiple primitives happens in `raycast.render`, not here.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

EPS = 1e-9


def _unit(v: np.ndarray) -> np.ndarray:
    v = np.asarray(v, dtype=np.float64)
    n = np.linalg.norm(v)
    if n < EPS:
        raise ValueError("cannot normalize a near-zero-length vector")
    return v / n


@dataclass
class Rect:
    """A planar rectangle: center +/- half_u along u_axis, +/- half_v along
    v_axis (= normal x u_axis, so (u_axis, v_axis, normal) is a
    right-handed orthonormal frame).

    `holes`: list of ((hu, hv), radius) circular cutouts in (u, v) plane
    coordinates, modelling the recorded hollow board's punched holes; empty
    (the default) is a plain solid square/rect.
    """

    center: np.ndarray
    normal: np.ndarray
    u_axis: np.ndarray
    half_u: float
    half_v: float
    holes: list[tuple[tuple[float, float], float]] = field(default_factory=list)

    def __post_init__(self) -> None:
        self.center = np.asarray(self.center, dtype=np.float64)
        self.normal = _unit(self.normal)
        u_raw = np.asarray(self.u_axis, dtype=np.float64)
        u_raw = u_raw - (u_raw @ self.normal) * self.normal  # orthogonalize
        self.u_axis = _unit(u_raw)
        self.v_axis = np.cross(self.normal, self.u_axis)

    def intersect(self, origins: np.ndarray, dirs: np.ndarray):
        denom = dirs @ self.normal  # (n,)
        with np.errstate(divide="ignore", invalid="ignore"):
            t = ((self.center - origins) @ self.normal) / denom
            hit = origins + t[:, None] * dirs
            q = hit - self.center
            u = q @ self.u_axis
            v = q @ self.v_axis

        inside = (
            (np.abs(denom) > EPS)
            & np.isfinite(t)
            & (t > EPS)
            & (np.abs(u) <= self.half_u)
            & (np.abs(v) <= self.half_v)
        )
        for (hu, hv), radius in self.holes:
            inside &= np.hypot(u - hu, v - hv) > radius

        t_out = np.where(inside, t, np.inf)
        cosang = np.where(inside, np.abs(denom), np.nan)
        return t_out, cosang


@dataclass
class Box:
    """Oriented box: `R`'s columns are the box's local x/y/z axes expressed
    in world coordinates (orthonormal), `half_sizes` the half-extent along
    each local axis. Intersected via the slab method; the near (entry)
    face wins, matching a solid opaque object."""

    center: np.ndarray
    R: np.ndarray
    half_sizes: np.ndarray

    def __post_init__(self) -> None:
        self.center = np.asarray(self.center, dtype=np.float64)
        self.R = np.asarray(self.R, dtype=np.float64)
        self.half_sizes = np.asarray(self.half_sizes, dtype=np.float64)

    def intersect(self, origins: np.ndarray, dirs: np.ndarray):
        n = origins.shape[0]
        rel = origins - self.center
        lo = rel @ self.R   # (n, 3) local-frame origin
        ld = dirs @ self.R  # (n, 3) local-frame direction
        h = self.half_sizes

        ld_safe = np.where(np.abs(ld) < EPS, EPS, ld)
        t1 = (-h - lo) / ld_safe
        t2 = (h - lo) / ld_safe
        tmin = np.minimum(t1, t2)  # (n, 3)
        tmax = np.maximum(t1, t2)  # (n, 3)

        # A ray truly parallel to an axis (|ld| ~ 0) and outside that axis's
        # slab never hits, regardless of what the epsilon-division above
        # computed -- force those columns to an unsatisfiable interval.
        parallel_outside = (np.abs(ld) < EPS) & (np.abs(lo) > h)
        tmin = np.where(parallel_outside, np.inf, tmin)
        tmax = np.where(parallel_outside, -np.inf, tmax)

        t_near = tmin.max(axis=1)
        t_far = tmax.min(axis=1)
        near_axis = tmin.argmax(axis=1)

        hit = (t_near <= t_far) & (t_far > EPS) & np.isfinite(t_near)
        t_out = np.where(hit & (t_near > EPS), t_near, np.inf)

        idx = np.arange(n)
        with np.errstate(invalid="ignore"):
            local_hit_coord = lo[idx, near_axis] + t_near * ld[idx, near_axis]
        normal_local = np.zeros((n, 3))
        normal_local[idx, near_axis] = np.sign(local_hit_coord)
        normal_world = normal_local @ self.R.T
        cosang = np.where(
            np.isfinite(t_out),
            np.abs(np.einsum("ij,ij->i", dirs, normal_world)),
            np.nan,
        )
        return t_out, cosang


@dataclass
class Cylinder:
    """Finite cylinder, side surface only (no end caps) -- a pole/pillar of
    clutter, extending from `base` along unit `axis` for `height`."""

    base: np.ndarray
    axis: np.ndarray
    radius: float
    height: float

    def __post_init__(self) -> None:
        self.base = np.asarray(self.base, dtype=np.float64)
        self.axis = _unit(self.axis)

    def intersect(self, origins: np.ndarray, dirs: np.ndarray):
        n = origins.shape[0]
        rel = origins - self.base
        a = self.axis

        rel_para = rel @ a               # (n,)
        rel_perp = rel - rel_para[:, None] * a
        d_para = dirs @ a                # (n,)
        d_perp = dirs - d_para[:, None] * a

        A = np.einsum("ij,ij->i", d_perp, d_perp)
        B = 2.0 * np.einsum("ij,ij->i", d_perp, rel_perp)
        C = np.einsum("ij,ij->i", rel_perp, rel_perp) - self.radius**2

        solvable = A > EPS
        disc = np.where(solvable, B**2 - 4.0 * A * C, -1.0)
        has_root = solvable & (disc >= 0.0)
        sqrt_disc = np.sqrt(np.clip(disc, 0.0, None))
        A_safe = np.where(A > EPS, A, 1.0)
        t_lo = (-B - sqrt_disc) / (2.0 * A_safe)
        t_hi = (-B + sqrt_disc) / (2.0 * A_safe)

        t = np.full(n, np.inf)
        for cand in (t_lo, t_hi):
            axial = rel_para + cand * d_para
            ok = (
                has_root & (cand > EPS)
                & (axial >= 0.0) & (axial <= self.height)
                & (cand < t)
            )
            t = np.where(ok, cand, t)

        valid = np.isfinite(t)
        with np.errstate(invalid="ignore"):
            hit_rel = rel + t[:, None] * dirs
            hit_rel_perp = hit_rel - np.outer(hit_rel @ a, a)
        norm_perp = np.linalg.norm(hit_rel_perp, axis=1)
        norm_safe = np.where(norm_perp > EPS, norm_perp, 1.0)
        radial_dir = hit_rel_perp / norm_safe[:, None]
        cosang = np.where(
            valid, np.abs(np.einsum("ij,ij->i", dirs, radial_dir)), np.nan
        )
        return t, cosang
