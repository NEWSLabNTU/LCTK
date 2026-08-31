"""Owned capture buffer and every estimate derived from it.

This is the domain core of the manual LiDAR-to-camera solver.  It intentionally
knows nothing about ROS nodes, logging, services, files, frame labels, or transform
publication.  Mutations prepare and solve a complete candidate state before one
atomic commit, so an estimate can never outlive the exact captures that produced it.
"""

from __future__ import annotations

import copy
import threading
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass
from enum import Enum, auto

import cv2
import numpy as np
from lctk_quality import (
    QualityReport,
    build_report,
    compute_diversity,
    distinct_placements,
)
from lctk_quality.placements import Placement
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


@dataclass(frozen=True)
class DetectionPair:
    """One deliberately retained synchronized pair of wire messages."""

    aruco: object
    board: object


class RejectionCode(Enum):
    INVALID_INDEX = auto()
    NO_BOARD_DETECTION = auto()
    NO_BOARD_POSE = auto()
    INVALID_BOARD_POSE = auto()
    INVALID_COVARIANCE = auto()
    NO_REAL_ARUCO_CORNERS = auto()
    NO_CONFIGURED_MARKER = auto()
    INSUFFICIENT_CORRESPONDENCES = auto()


@dataclass(frozen=True)
class MutationRejection:
    code: RejectionCode
    detail: str = ""


class FailureCode(Enum):
    PNP_FAILED = auto()
    QUALITY_FAILED = auto()


@dataclass(frozen=True)
class Empty:
    """No captures exist."""


@dataclass(frozen=True)
class NotReady:
    frame_count: int
    required: int


@dataclass(frozen=True)
class Refused:
    normal_spread_deg: float
    depth_range_m: float


@dataclass(frozen=True)
class Failed:
    code: FailureCode
    detail: str = ""


@dataclass(frozen=True)
class SolvedEstimate:
    rvec: np.ndarray
    tvec: np.ndarray
    quality: QualityReport


@dataclass(frozen=True)
class Solved:
    estimate: SolvedEstimate


SolveOutcome = Empty | NotReady | Refused | Failed | Solved


@dataclass(frozen=True)
class BufferSnapshot:
    revision: int
    pairs: tuple[DetectionPair, ...]
    placements: tuple[Placement, ...]
    correspondence_count: int
    outcome: SolveOutcome

    @property
    def frame_count(self) -> int:
        return len(self.pairs)

    @property
    def placement_count(self) -> int:
        return len(self.placements)

    @property
    def estimate(self) -> SolvedEstimate | None:
        return self.outcome.estimate if isinstance(self.outcome, Solved) else None


@dataclass(frozen=True)
class BufferUpdate:
    accepted: bool
    changed: bool
    snapshot: BufferSnapshot
    rejection: MutationRejection | None = None
    added_new_placement: bool | None = None


@dataclass(frozen=True)
class _BoardDetection:
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]
    covariance: np.ndarray | None


@dataclass(frozen=True)
class _PreparedCapture:
    pair: DetectionPair
    object_points: np.ndarray
    image_points: np.ndarray
    board: _BoardDetection
    weight: float


class _AdmissionError(ValueError):
    def __init__(self, code: RejectionCode, detail: str = "") -> None:
        super().__init__(detail)
        self.rejection = MutationRejection(code, detail)


def _readonly_array(value: np.ndarray) -> np.ndarray:
    result = np.array(value, dtype=np.float64, copy=True)
    result.setflags(write=False)
    return result


def _marker_id(value: object) -> int | None:
    try:
        if isinstance(value, str) and value.startswith("aruco_"):
            return int(value.removeprefix("aruco_"))
        return int(value)
    except (TypeError, ValueError):
        return None


class DetectionBuffer:
    """Thread-safe capture collection with atomic, revision-bound solving."""

    def __init__(
        self,
        *,
        camera_matrix: np.ndarray,
        marker_corners_by_id: Mapping[int, Sequence[tuple[float, float, float]]],
        min_frames_required: int,
        min_normal_spread_deg: float,
        min_depth_range_m: float,
        enforce_pose_diversity: bool,
    ) -> None:
        camera_matrix = np.asarray(camera_matrix, dtype=np.float64)
        if camera_matrix.shape != (3, 3) or not np.all(np.isfinite(camera_matrix)):
            raise ValueError("camera_matrix must be a finite 3x3 matrix")
        if min_frames_required < 1:
            raise ValueError("min_frames_required must be at least 1")
        if min_normal_spread_deg < 0.0 or min_depth_range_m < 0.0:
            raise ValueError("pose-diversity thresholds must be non-negative")

        geometry: dict[int, np.ndarray] = {}
        for marker_id, corners in marker_corners_by_id.items():
            array = np.asarray(corners, dtype=np.float64)
            if array.shape != (4, 3) or not np.all(np.isfinite(array)):
                raise ValueError(
                    f"marker {marker_id} geometry must contain four finite 3D corners"
                )
            geometry[int(marker_id)] = _readonly_array(array)
        if not geometry:
            raise ValueError("marker_corners_by_id must not be empty")

        self._camera_matrix = _readonly_array(camera_matrix)
        self._marker_corners_by_id = geometry
        self._min_frames_required = min_frames_required
        self._min_normal_spread_deg = float(min_normal_spread_deg)
        self._min_depth_range_m = float(min_depth_range_m)
        self._enforce_pose_diversity = bool(enforce_pose_diversity)
        self._lock = threading.RLock()
        self._captures: tuple[_PreparedCapture, ...] = ()
        self._placements: tuple[Placement, ...] = ()
        self._outcome: SolveOutcome = Empty()
        self._correspondence_count = 0
        self._revision = 0

    def capture(self, pair: DetectionPair) -> BufferUpdate:
        try:
            prepared = self._prepare_pair(pair)
        except _AdmissionError as error:
            with self._lock:
                return self._rejected(error.rejection)

        with self._lock:
            candidate = (*self._captures, prepared)
            old_count = len(self._placements)
            placements, correspondence_count, outcome = self._derive(candidate)
            self._commit(candidate, placements, correspondence_count, outcome)
            return BufferUpdate(
                accepted=True,
                changed=True,
                snapshot=self._snapshot_locked(),
                added_new_placement=len(placements) > old_count,
            )

    def restore(self, pairs: Iterable[DetectionPair], *, append: bool) -> BufferUpdate:
        try:
            prepared = tuple(self._prepare_pair(pair) for pair in pairs)
        except _AdmissionError as error:
            with self._lock:
                return self._rejected(error.rejection)

        with self._lock:
            candidate = (*self._captures, *prepared) if append else prepared
            placements, correspondence_count, outcome = self._derive(candidate)
            self._commit(candidate, placements, correspondence_count, outcome)
            return BufferUpdate(
                accepted=True,
                changed=True,
                snapshot=self._snapshot_locked(),
            )

    def remove(self, index: int) -> BufferUpdate:
        with self._lock:
            if index < 0 or index >= len(self._captures):
                return self._rejected(
                    MutationRejection(RejectionCode.INVALID_INDEX, str(index))
                )
            candidate = self._captures[:index] + self._captures[index + 1 :]
            placements, correspondence_count, outcome = self._derive(candidate)
            self._commit(candidate, placements, correspondence_count, outcome)
            return BufferUpdate(
                accepted=True,
                changed=True,
                snapshot=self._snapshot_locked(),
            )

    def clear(self) -> BufferUpdate:
        with self._lock:
            if not self._captures:
                return BufferUpdate(
                    accepted=True,
                    changed=False,
                    snapshot=self._snapshot_locked(),
                )
            self._captures = ()
            self._placements = ()
            self._correspondence_count = 0
            self._outcome = Empty()
            self._revision += 1
            return BufferUpdate(
                accepted=True,
                changed=True,
                snapshot=self._snapshot_locked(),
            )

    def snapshot(self) -> BufferSnapshot:
        with self._lock:
            return self._snapshot_locked()

    def _rejected(self, rejection: MutationRejection) -> BufferUpdate:
        return BufferUpdate(
            accepted=False,
            changed=False,
            snapshot=self._snapshot_locked(),
            rejection=rejection,
        )

    def _commit(
        self,
        captures: tuple[_PreparedCapture, ...],
        placements: tuple[Placement, ...],
        correspondence_count: int,
        outcome: SolveOutcome,
    ) -> None:
        self._captures = captures
        self._placements = placements
        self._correspondence_count = correspondence_count
        self._outcome = outcome
        self._revision += 1

    def _snapshot_locked(self) -> BufferSnapshot:
        pairs = tuple(copy.deepcopy(capture.pair) for capture in self._captures)
        placements = copy.deepcopy(self._placements)
        outcome = copy.deepcopy(self._outcome)
        if isinstance(outcome, Solved):
            estimate = outcome.estimate
            outcome = Solved(
                SolvedEstimate(
                    rvec=_readonly_array(estimate.rvec),
                    tvec=_readonly_array(estimate.tvec),
                    quality=estimate.quality,
                )
            )
        return BufferSnapshot(
            revision=self._revision,
            pairs=pairs,
            placements=placements,
            correspondence_count=self._correspondence_count,
            outcome=outcome,
        )

    def _prepare_pair(self, pair: DetectionPair) -> _PreparedCapture:
        detached = DetectionPair(copy.deepcopy(pair.aruco), copy.deepcopy(pair.board))
        board = self._read_board(detached.board)

        object_points: list[np.ndarray] = []
        image_points: list[np.ndarray] = []
        saw_real_corners = False
        saw_configured_marker = False

        rotation = Rotation.from_quat(board.orientation).as_matrix()
        position = np.asarray(board.position, dtype=np.float64)
        for detection in getattr(detached.aruco, "detections", ()):
            results = getattr(detection, "results", ())
            if len(results) < 4:
                continue
            saw_real_corners = True
            marker_id = _marker_id(getattr(detection, "id", None))
            local_corners = self._marker_corners_by_id.get(marker_id)
            if local_corners is None:
                continue
            saw_configured_marker = True
            pixels = np.asarray(
                [
                    (result.pose.pose.position.x, result.pose.pose.position.y)
                    for result in results[:4]
                ],
                dtype=np.float64,
            )
            # Quick fix: rotate the corner order by 1 position (90 degrees)
            pixels = np.roll(pixels, shift=1, axis=0)
            if pixels.shape != (4, 2) or not np.all(np.isfinite(pixels)):
                continue
            world_corners = (rotation @ local_corners.T).T + position
            object_points.extend(world_corners)
            image_points.extend(pixels)

        if not saw_real_corners:
            raise _AdmissionError(RejectionCode.NO_REAL_ARUCO_CORNERS)
        if not saw_configured_marker:
            raise _AdmissionError(RejectionCode.NO_CONFIGURED_MARKER)
        if len(object_points) < 4:
            raise _AdmissionError(
                RejectionCode.INSUFFICIENT_CORRESPONDENCES,
                str(len(object_points)),
            )

        objects = _readonly_array(np.asarray(object_points, dtype=np.float64))
        images = _readonly_array(np.asarray(image_points, dtype=np.float64))
        return _PreparedCapture(
            pair=detached,
            object_points=objects,
            image_points=images,
            board=board,
            weight=self._pose_weight(board, objects),
        )

    @staticmethod
    def _read_board(message: object) -> _BoardDetection:
        detections = getattr(message, "detections", ())
        if not detections:
            raise _AdmissionError(RejectionCode.NO_BOARD_DETECTION)
        results = getattr(detections[0], "results", ())
        if not results:
            raise _AdmissionError(RejectionCode.NO_BOARD_POSE)

        result = results[0]
        pose = result.pose.pose
        position_array = np.asarray(
            (pose.position.x, pose.position.y, pose.position.z), dtype=np.float64
        )
        quaternion = np.asarray(
            (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ),
            dtype=np.float64,
        )
        norm = float(np.linalg.norm(quaternion))
        if (
            not np.all(np.isfinite(position_array))
            or not np.all(np.isfinite(quaternion))
            or norm < 1e-12
        ):
            raise _AdmissionError(RejectionCode.INVALID_BOARD_POSE)
        quaternion /= norm

        covariance_values = np.asarray(result.pose.covariance, dtype=np.float64)
        if covariance_values.size != 36 or not np.all(np.isfinite(covariance_values)):
            raise _AdmissionError(RejectionCode.INVALID_COVARIANCE)
        covariance = covariance_values.reshape(6, 6)
        if not np.any(covariance):
            covariance = None
        else:
            covariance = _readonly_array(covariance)

        return _BoardDetection(
            position=tuple(float(value) for value in position_array),
            orientation=tuple(float(value) for value in quaternion),
            covariance=covariance,
        )

    def _derive(
        self, captures: tuple[_PreparedCapture, ...]
    ) -> tuple[tuple[Placement, ...], int, SolveOutcome]:
        if not captures:
            return (), 0, Empty()

        poses = [
            (capture.board.position, capture.board.orientation) for capture in captures
        ]
        placements = tuple(distinct_placements(poses))
        correspondence_count = sum(len(capture.object_points) for capture in captures)
        if len(captures) < self._min_frames_required:
            return (
                placements,
                correspondence_count,
                NotReady(len(captures), self._min_frames_required),
            )

        diversity = compute_diversity(placements)
        if (
            self._enforce_pose_diversity
            and diversity is not None
            and (
                diversity.normal_span_deg < self._min_normal_spread_deg
                or diversity.depth_range_m < self._min_depth_range_m
            )
        ):
            return (
                placements,
                correspondence_count,
                Refused(diversity.normal_span_deg, diversity.depth_range_m),
            )

        objects = np.vstack([capture.object_points for capture in captures])
        images = np.vstack([capture.image_points for capture in captures])
        weights = self._expanded_weights(captures)
        try:
            ok, rvec, tvec = cv2.solvePnP(
                objects,
                images,
                self._camera_matrix,
                np.zeros(5, dtype=np.float64),
                flags=cv2.SOLVEPNP_SQPNP,
            )
            if not ok:
                return placements, correspondence_count, Failed(FailureCode.PNP_FAILED)
            if weights is None:
                rvec, tvec = cv2.solvePnPRefineLM(
                    objects,
                    images,
                    self._camera_matrix,
                    np.zeros(5, dtype=np.float64),
                    rvec,
                    tvec,
                )
            else:
                rvec, tvec = self._refine_weighted(objects, images, rvec, tvec, weights)
        except (cv2.error, ValueError, np.linalg.LinAlgError) as error:
            return (
                placements,
                correspondence_count,
                Failed(FailureCode.PNP_FAILED, str(error)),
            )

        try:
            quality = build_report(
                [capture.object_points for capture in captures],
                [capture.image_points for capture in captures],
                poses,
                self._camera_matrix,
                rvec,
                tvec,
            )
        except (cv2.error, ValueError, np.linalg.LinAlgError) as error:
            return (
                placements,
                correspondence_count,
                Failed(FailureCode.QUALITY_FAILED, str(error)),
            )
        if quality is None:
            return placements, correspondence_count, Failed(FailureCode.QUALITY_FAILED)

        estimate = SolvedEstimate(
            rvec=_readonly_array(rvec).reshape(3, 1),
            tvec=_readonly_array(tvec).reshape(3, 1),
            quality=quality,
        )
        return placements, correspondence_count, Solved(estimate)

    def _expanded_weights(
        self, captures: tuple[_PreparedCapture, ...]
    ) -> np.ndarray | None:
        if all(capture.weight >= 1.0 for capture in captures):
            return None
        return np.concatenate(
            [
                np.full(len(capture.object_points), capture.weight, dtype=np.float64)
                for capture in captures
            ]
        )

    def _refine_weighted(
        self,
        object_points: np.ndarray,
        image_points: np.ndarray,
        rvec: np.ndarray,
        tvec: np.ndarray,
        weights: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        residual_weights = np.repeat(weights, 2)

        def residuals(parameters: np.ndarray) -> np.ndarray:
            projected, _ = cv2.projectPoints(
                object_points,
                parameters[:3].reshape(3, 1),
                parameters[3:].reshape(3, 1),
                self._camera_matrix,
                np.zeros(5, dtype=np.float64),
            )
            return residual_weights * (projected.reshape(-1, 2) - image_points).ravel()

        seed = np.concatenate((rvec.ravel(), tvec.ravel()))
        result = least_squares(residuals, seed, method="lm", max_nfev=200)
        return result.x[:3].reshape(3, 1), result.x[3:].reshape(3, 1)

    @staticmethod
    def _pose_weight(board: _BoardDetection, object_points: np.ndarray) -> float:
        if board.covariance is None or len(object_points) == 0:
            return 1.0

        total_variance = 0.0
        origin = np.asarray(board.position, dtype=np.float64)
        for corner in object_points:
            lever = corner - origin
            skew = np.array(
                (
                    (0.0, -lever[2], lever[1]),
                    (lever[2], 0.0, -lever[0]),
                    (-lever[1], lever[0], 0.0),
                )
            )
            jacobian = np.hstack((np.eye(3), -skew))
            total_variance += float(np.trace(jacobian @ board.covariance @ jacobian.T))
        sigma = np.sqrt(max(total_variance / len(object_points), 1e-12))
        return float(np.clip(1.0 / (1.0 + sigma / 0.01), 1e-3, 1.0))


__all__ = [
    "BufferSnapshot",
    "BufferUpdate",
    "DetectionBuffer",
    "DetectionPair",
    "Empty",
    "Failed",
    "FailureCode",
    "MutationRejection",
    "NotReady",
    "Refused",
    "RejectionCode",
    "Solved",
    "SolvedEstimate",
]
