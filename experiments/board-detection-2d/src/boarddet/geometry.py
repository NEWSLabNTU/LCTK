"""Plane fitting and 2D plane-coordinate projection (the chosen projection)."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import open3d as o3d


@dataclass
class PlaneModel:
    center: np.ndarray  # (3,)
    normal: np.ndarray  # (3,) unit
    u: np.ndarray       # (3,) in-plane basis
    v: np.ndarray       # (3,) in-plane basis


def fit_plane(points: np.ndarray) -> PlaneModel:
    center = points.mean(axis=0)
    q = points.astype(np.float64) - center
    # smallest singular vector = normal; the two largest span the plane
    _, _, vt = np.linalg.svd(q, full_matrices=False)
    u, v, normal = vt[0], vt[1], vt[2]
    return PlaneModel(center=center, normal=normal, u=u, v=v)


def plane_rms(points: np.ndarray, plane: PlaneModel) -> float:
    d = (points - plane.center) @ plane.normal
    return float(np.sqrt(np.mean(d**2)))


def project_to_plane(points: np.ndarray, plane: PlaneModel) -> np.ndarray:
    q = points - plane.center
    return np.stack([q @ plane.u, q @ plane.v], axis=1)


def unproject(coords_2d: np.ndarray, plane: PlaneModel) -> np.ndarray:
    return (plane.center
            + coords_2d[:, :1] * plane.u
            + coords_2d[:, 1:] * plane.v)


def finite_only(points: np.ndarray) -> np.ndarray:
    """Drop rows with any non-finite (NaN/inf) coordinate.

    Raw LiDAR PointCloud2 encodes invalid returns as NaN; fit_plane's SVD
    would otherwise propagate a NaN normal and poison every downstream pose.
    """
    points = np.asarray(points)
    if len(points) == 0:
        return points
    return points[np.isfinite(points).all(axis=1)]


def downsample(points: np.ndarray, voxel: float = 0.03) -> np.ndarray:
    pc = o3d.geometry.PointCloud(
        o3d.utility.Vector3dVector(points.astype(np.float64)))
    dn = pc.voxel_down_sample(voxel)
    return np.asarray(dn.points, dtype=np.float32)


def extent_2d(coords_2d: np.ndarray) -> float:
    span = coords_2d.max(axis=0) - coords_2d.min(axis=0)
    return float(span.max())
