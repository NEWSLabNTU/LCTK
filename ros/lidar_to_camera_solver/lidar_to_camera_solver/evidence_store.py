"""Stamped camera and LiDAR evidence for assisted capture review."""

from __future__ import annotations

import math
import threading
from collections import OrderedDict
from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np

from lidar_to_camera_solver.preview import annotate, decode_image, encode_jpeg
from lidar_to_camera_solver.stamped_ring import StampedRing

DEFAULT_STAMP_TOLERANCE_S = 0.001
"""Small fallback for conversion jitter; source-stamp equality remains primary."""

_FRAME_RING_MAX_ITEMS = 64
_CLOUD_RING_MAX_ITEMS = 256


@dataclass(frozen=True)
class CaptureEvidence:
    """Review payloads belonging to one retained capture."""

    preview_jpeg: bytes | None
    cloud_xyz: bytes | None
    missing: tuple[str, ...]


@dataclass(frozen=True)
class _StoredFrame:
    image: np.ndarray
    scale_x: float
    scale_y: float


@dataclass(frozen=True)
class _Intrinsics:
    camera_matrix: np.ndarray
    distortion: np.ndarray


@dataclass(frozen=True)
class _PendingCapture:
    camera_stamp: float | None
    cloud_stamp: float | None
    corners: tuple[np.ndarray, ...]
    intrinsics: _Intrinsics | None


class EvidenceStore:
    """Bounded, timestamp-matched camera/cloud snapshots for review.

    Frames are decoded and reduced at subscription rate, but expensive
    rectification, annotation and JPEG encoding happen only for captures.  A
    missing match is represented in :class:`CaptureEvidence`; this store never
    lets a preview failure reject a calibration pair.
    """

    def __init__(
        self,
        *,
        max_previews: int,
        jpeg_quality: int,
        review_evidence_seconds: float = 1.0,
        stamp_tolerance_s: float = DEFAULT_STAMP_TOLERANCE_S,
    ) -> None:
        max_previews = int(max_previews)
        if max_previews < 1:
            raise ValueError(f"max_previews must be at least 1; got {max_previews}")
        jpeg_quality = int(jpeg_quality)
        if not 0 <= jpeg_quality <= 100:
            raise ValueError(
                f"jpeg_quality must be between 0 and 100; got {jpeg_quality}"
            )

        self._max_previews = max_previews
        self._jpeg_quality = jpeg_quality
        self._frames: StampedRing[_StoredFrame] = StampedRing(
            max_age_s=review_evidence_seconds,
            tolerance_s=stamp_tolerance_s,
            max_items=_FRAME_RING_MAX_ITEMS,
        )
        self._clouds: StampedRing[bytes] = StampedRing(
            max_age_s=review_evidence_seconds,
            tolerance_s=stamp_tolerance_s,
            max_items=_CLOUD_RING_MAX_ITEMS,
        )
        self._intrinsics: _Intrinsics | None = None
        self._evidence: OrderedDict[int, CaptureEvidence] = OrderedDict()
        self._pending: dict[int, _PendingCapture] = {}
        self._lock = threading.RLock()

    def observe_frame(
        self,
        stamp: float,
        height: int,
        width: int,
        encoding: str,
        step: int,
        data: bytes,
    ) -> None:
        """Decode and retain one camera frame at half resolution.

        Invalid input clears the frame timeline.  That is intentional: retaining
        a previous valid frame would make a malformed source message appear to be
        evidence for a later pair.
        """
        stamp_s = _finite_stamp(stamp)
        if stamp_s is None:
            with self._lock:
                self._frames.clear()
                self._clouds.clear()
                self._invalidate_pending_locked()
            return
        try:
            height = int(height)
            width = int(width)
            step = int(step)
            if height < 1 or width < 1 or step < 1:
                raise ValueError("image dimensions and step must be positive")
            decoded = decode_image(
                height=height,
                width=width,
                encoding=encoding,
                step=step,
                data=data,
            )
            output_width = max(width // 2, 1)
            output_height = max(height // 2, 1)
            reduced = cv2.resize(
                decoded,
                (output_width, output_height),
                interpolation=cv2.INTER_AREA,
            )
            stored = _StoredFrame(
                image=np.ascontiguousarray(reduced),
                scale_x=output_width / width,
                scale_y=output_height / height,
            )
        except (AttributeError, TypeError, ValueError, OverflowError, cv2.error):
            with self._lock:
                self._frames.clear()
                self._invalidate_pending_locked()
            return

        with self._lock:
            old_epoch = self._frames.epoch
            self._frames.put(stamp_s, stored)
            if self._frames.epoch != old_epoch:
                # A replay/reset seam applies to both source streams.  Clearing
                # only the camera ring would leave an old cloud available for a
                # new camera stamp (and vice versa while the other subscription
                # catches up).
                self._clouds.clear()
                self._invalidate_pending_locked()
            else:
                self._complete_pending_frame_locked(stamp_s)

    def observe_cloud(self, stamp: float, points: Any) -> None:
        """Pack one cloud's XYZ points as little-endian float32 bytes."""
        stamp_s = _finite_stamp(stamp)
        if stamp_s is None:
            with self._lock:
                self._clouds.clear()
                self._frames.clear()
                self._invalidate_pending_locked()
            return
        try:
            array = np.asarray(points, dtype=np.float32)
            if array.size == 0:
                array = np.empty((0, 3), dtype=np.float32)
            elif array.ndim != 2 or array.shape[1] != 3:
                raise ValueError("cloud points must have shape (N, 3)")
            if not np.all(np.isfinite(array)):
                raise ValueError("cloud points must be finite")
            packed = np.ascontiguousarray(array, dtype="<f4").tobytes()
        except (AttributeError, TypeError, ValueError, OverflowError):
            with self._lock:
                self._clouds.clear()
                self._invalidate_pending_locked()
            return

        with self._lock:
            old_epoch = self._clouds.epoch
            self._clouds.put(stamp_s, packed)
            if self._clouds.epoch != old_epoch:
                self._frames.clear()
                self._invalidate_pending_locked()
            else:
                self._complete_pending_cloud_locked(stamp_s, packed)

    def observe_intrinsics(self, camera_matrix: Any, distortion: Any) -> None:
        """Replace the current camera model, refusing malformed values."""
        parsed: _Intrinsics | None
        try:
            matrix = np.asarray(camera_matrix, dtype=np.float64)
            coefficients = np.asarray(distortion, dtype=np.float64).reshape(-1)
            if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
                raise ValueError("camera_matrix must be a finite 3x3 matrix")
            if coefficients.size == 0:
                # Some CameraInfo producers leave D empty for an already
                # rectified stream.  Treat that representation as the inert
                # zero-distortion model rather than carrying a stale model.
                coefficients = np.zeros(5, dtype=np.float64)
            elif coefficients.size not in (4, 5, 8, 12, 14):
                raise ValueError("distortion must contain 4, 5, 8, 12, or 14 values")
            if not np.all(np.isfinite(coefficients)):
                raise ValueError("distortion must be finite")
            parsed = _Intrinsics(
                camera_matrix=np.array(matrix, dtype=np.float64, copy=True),
                distortion=np.array(coefficients, dtype=np.float64, copy=True),
            )
        except (AttributeError, TypeError, ValueError, OverflowError):
            parsed = None

        with self._lock:
            previous = self._intrinsics
            self._intrinsics = parsed
            if (
                parsed is None
                or previous is None
                or not _same_intrinsics(previous, parsed)
            ):
                # Raw frames are only compatible with the model that was active
                # when they were captured.  Do not let a nearest old frame be
                # rectified using a newly published camera model.
                self._frames.clear()
                # A pending capture carries the model observed at capture time.
                # Once the source publishes a new model, a later frame with the
                # same stamp belongs to a different camera timeline and must not
                # complete that old slot.
                self._invalidate_pending_locked()

    def capture(
        self,
        pair_id: int,
        stamp: float,
        corners,
        *,
        cloud_stamp: float | None = None,
    ) -> CaptureEvidence:
        """Capture evidence matched to camera ``stamp`` and optional cloud stamp."""
        camera_stamp = _finite_stamp(stamp)
        cloud_stamp_s = _finite_stamp(stamp if cloud_stamp is None else cloud_stamp)

        with self._lock:
            frame = (
                self._frames.match(camera_stamp) if camera_stamp is not None else None
            )
            cloud = (
                self._clouds.match(cloud_stamp_s) if cloud_stamp_s is not None else None
            )
            intrinsics = self._intrinsics
            preview, preview_missing, owned_corners, retryable = (
                self._render_preview_locked(frame, corners, intrinsics)
            )
            missing = list(preview_missing)
            if cloud is None:
                missing.append("plane inliers")
            evidence = CaptureEvidence(
                preview_jpeg=preview,
                cloud_xyz=cloud,
                missing=tuple(missing),
            )
            pair_id = int(pair_id)
            self._evidence[pair_id] = evidence
            self._evidence.move_to_end(pair_id)
            if retryable or cloud is None:
                self._pending[pair_id] = _PendingCapture(
                    camera_stamp=camera_stamp,
                    cloud_stamp=cloud_stamp_s,
                    corners=owned_corners,
                    intrinsics=intrinsics,
                )
            else:
                self._pending.pop(pair_id, None)
            self._trim_evidence_locked()
            return evidence

    def get(self, pair_id: int) -> CaptureEvidence | None:
        """Return cached evidence and mark it as recently used."""
        with self._lock:
            evidence = self._evidence.get(int(pair_id))
            if evidence is not None:
                self._evidence.move_to_end(int(pair_id))
            return evidence

    def drop(self, pair_id: int) -> None:
        """Forget all evidence associated with one review id."""
        with self._lock:
            pair_id = int(pair_id)
            self._evidence.pop(pair_id, None)
            self._pending.pop(pair_id, None)

    def clear(self) -> None:
        """Forget captures and buffered source payloads, retaining intrinsics."""
        with self._lock:
            self._evidence.clear()
            self._pending.clear()
            self._frames.clear()
            self._clouds.clear()

    def _render_preview_locked(
        self,
        frame: _StoredFrame | None,
        corners,
        intrinsics: _Intrinsics | None,
    ) -> tuple[
        bytes | None,
        tuple[str, ...],
        tuple[np.ndarray, ...],
        bool,
    ]:
        # ``corners`` is usually a list of 4x2 arrays.  Accept one ndarray as a
        # convenience, but never turn malformed corner data into an unannotated
        # success: an operator would reasonably read that as a clean frame.
        try:
            if corners is None:
                source_corners = ()
            else:
                corners_array = np.asarray(corners)
                source_corners = (
                    (corners,) if corners_array.ndim == 2 else tuple(corners)
                )
            owned_corners = tuple(
                np.array(corner, dtype=np.float64, copy=True)
                for corner in source_corners
            )
            if any(
                corner.shape != (4, 2) or not np.all(np.isfinite(corner))
                for corner in owned_corners
            ):
                raise ValueError("corners must contain finite 4x2 quads")
        except (TypeError, ValueError):
            return None, ("camera annotation",), (), False

        if frame is None:
            return None, ("camera frame",), owned_corners, True
        if intrinsics is None:
            return None, ("camera intrinsics",), owned_corners, False
        try:
            scaled_matrix = _scaled_camera_matrix(
                intrinsics.camera_matrix,
                frame.scale_x,
                frame.scale_y,
            )
            pixels = tuple(
                np.column_stack(
                    (
                        corner[:, 0] * frame.scale_x,
                        corner[:, 1] * frame.scale_y,
                    )
                )
                for corner in owned_corners
            )
            image = frame.image
            if np.any(intrinsics.distortion != 0.0):
                image = cv2.undistort(
                    image,
                    scaled_matrix,
                    intrinsics.distortion,
                )
            return (
                encode_jpeg(
                    annotate(image, pixels, reprojected=None), self._jpeg_quality
                ),
                (),
                owned_corners,
                False,
            )
        except (TypeError, ValueError, OverflowError, cv2.error):
            return None, ("camera frame",), owned_corners, False

    def _complete_pending_cloud_locked(self, stamp: float, cloud: bytes) -> None:
        for pair_id, pending in tuple(self._pending.items()):
            if pending.cloud_stamp != stamp:
                continue
            evidence = self._evidence.get(pair_id)
            if evidence is None or evidence.cloud_xyz is not None:
                continue
            missing = tuple(
                name for name in evidence.missing if name != "plane inliers"
            )
            self._evidence[pair_id] = CaptureEvidence(
                preview_jpeg=evidence.preview_jpeg,
                cloud_xyz=cloud,
                missing=missing,
            )
            self._evidence.move_to_end(pair_id)
            if not missing:
                self._pending.pop(pair_id, None)

    def _complete_pending_frame_locked(self, stamp: float) -> None:
        frame = self._frames.match_exact(stamp)
        if frame is None:
            return
        for pair_id, pending in tuple(self._pending.items()):
            if pending.camera_stamp != stamp:
                continue
            evidence = self._evidence.get(pair_id)
            if evidence is None or evidence.preview_jpeg is not None:
                continue
            preview, preview_missing, _, _ = self._render_preview_locked(
                frame, pending.corners, pending.intrinsics
            )
            if preview is None:
                continue
            missing = tuple(
                name for name in evidence.missing if name not in preview_missing
            )
            # If the prior absence was a camera frame/intrinsics failure, remove
            # only that camera-side marker while preserving an unrelated cloud
            # miss.
            missing = tuple(
                name
                for name in missing
                if name not in ("camera frame", "camera intrinsics")
            )
            self._evidence[pair_id] = CaptureEvidence(
                preview_jpeg=preview,
                cloud_xyz=evidence.cloud_xyz,
                missing=missing,
            )
            self._evidence.move_to_end(pair_id)
            if not missing:
                self._pending.pop(pair_id, None)

    def _invalidate_pending_locked(self) -> None:
        """Prevent a new timestamp epoch from completing old evidence slots."""
        self._pending.clear()

    def _trim_evidence_locked(self) -> None:
        while len(self._evidence) > self._max_previews:
            pair_id, _ = self._evidence.popitem(last=False)
            self._pending.pop(pair_id, None)


def _finite_stamp(value: Any) -> float | None:
    try:
        value = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return value if math.isfinite(value) else None


def _scaled_camera_matrix(
    camera_matrix: np.ndarray, scale_x: float, scale_y: float
) -> np.ndarray:
    scaled = np.array(camera_matrix, dtype=np.float64, copy=True)
    scaled[0, :] *= scale_x
    scaled[1, :] *= scale_y
    return scaled


def _same_intrinsics(left: _Intrinsics | None, right: _Intrinsics) -> bool:
    return bool(
        left is not None
        and np.array_equal(left.camera_matrix, right.camera_matrix)
        and np.array_equal(left.distortion, right.distortion)
    )


__all__ = [
    "DEFAULT_STAMP_TOLERANCE_S",
    "CaptureEvidence",
    "EvidenceStore",
]
