"""Camera-side adapters for the shared calibration-target contract.

The physical board and marker layout belong to :mod:`lctk_target`.  This module keeps
the small, ROS-free surface used by the solver and its tests: target loading,
identity validation/gating, and human-readable geometry diagnostics.  Keeping the
identity gate here makes its decision table testable without starting a ROS graph.
"""

from __future__ import annotations

import re
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from lctk_target import TargetIdentity, ValidatedTarget, load_target

# This is the frame in which both observer adapters publish board poses.  The
# value remains exported because the archive codec and migration command still
# use it while archive v5 is being completed in W4-Eb/W4-Ec.
BOARD_FRAME_CONVENTION = "corner_aligned_plate_center_v1"

LIDAR_TARGET_IDENTITY_TOPIC = "lidar_target_identity"
CAMERA_TARGET_IDENTITY_TOPIC = "camera_target_identity"

_IDENTITY_FIELDS = (
    "schema_version",
    "target_id",
    "revision",
    "semantic_sha256",
    "board_frame_convention",
)
_SHA256 = re.compile(r"[0-9a-f]{64}")


def load_target_definition(path: str | Path) -> ValidatedTarget:
    """Load one immutable Target Definition through the shared Python package."""

    return load_target(path)


def marker_geometry_summary(target: ValidatedTarget) -> str:
    """Describe the marker scale derived from a validated target."""

    fiducial = target.fiducial
    square_um = (
        fiducial.paper_side_um - 2 * fiducial.outer_border_um
    ) / fiducial.cells_per_side
    marker_um = square_um * fiducial.marker_fill_ratio
    marker_border_um = (square_um - marker_um) / 2.0
    return (
        f"plate_side={target.plate.side_um / 1000:.1f}mm, "
        f"square_size={square_um / 1000:.1f}mm, "
        f"marker_size={marker_um / 1000:.1f}mm, "
        f"marker_border={marker_border_um / 1000:.1f}mm"
    )


def rotation_matrix_to_quaternion(rotation_matrix: np.ndarray) -> np.ndarray:
    """Convert a 3x3 rotation matrix to ROS quaternion order ``[x,y,z,w]``."""

    rvec, _ = cv2.Rodrigues(np.asarray(rotation_matrix, dtype=np.float64))
    angle = np.linalg.norm(rvec)
    if angle < 1e-6:
        return np.array([0.0, 0.0, 0.0, 1.0])

    axis = rvec.flatten() / angle
    half_angle = angle / 2.0
    return np.array(
        [
            axis[0] * np.sin(half_angle),
            axis[1] * np.sin(half_angle),
            axis[2] * np.sin(half_angle),
            np.cos(half_angle),
        ]
    )


def _field(value: object, name: str) -> object:
    """Read an identity field from either a ROS message or a mapping."""

    if isinstance(value, Mapping):
        if name not in value:
            raise ValueError(f"missing field '{name}'")
        return value[name]
    try:
        return getattr(value, name)
    except AttributeError as error:
        raise ValueError(f"missing field '{name}'") from error


def _positive_uint(value: object, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"{name} must be a positive integer")
    return value


def parse_target_identity(
    value: object, *, label: str = "Target Identity"
) -> TargetIdentity:
    """Validate and detach a wire identity into the shared plain-value type.

    ROS fields are typed, but a default-constructed message is still structurally
    malformed for this contract.  Validation is deliberately strict so malformed
    data can never become an equality match merely because all its fields happen to
    compare equal.
    """

    if value is None:
        raise ValueError(f"{label} is missing")
    if isinstance(value, Mapping):
        unexpected = set(value) - set(_IDENTITY_FIELDS)
        if unexpected:
            raise ValueError(
                f"{label} is malformed: unexpected field '{min(unexpected)}'"
            )
    try:
        fields = {name: _field(value, name) for name in _IDENTITY_FIELDS}
    except ValueError as error:
        raise ValueError(f"{label} is malformed: {error}") from error

    schema_version = _positive_uint(fields["schema_version"], f"{label}.schema_version")
    target_id = fields["target_id"]
    if not isinstance(target_id, str) or not target_id.strip():
        raise ValueError(f"{label}.target_id must be a non-empty string")
    revision = _positive_uint(fields["revision"], f"{label}.revision")
    digest = fields["semantic_sha256"]
    if not isinstance(digest, str) or _SHA256.fullmatch(digest) is None:
        raise ValueError(
            f"{label}.semantic_sha256 must be 64 lowercase hexadecimal characters"
        )
    frame = fields["board_frame_convention"]
    if not isinstance(frame, str) or not frame.strip():
        raise ValueError(f"{label}.board_frame_convention must be a non-empty string")

    return TargetIdentity(schema_version, target_id, revision, digest, frame)


def target_identity_error(
    value: object, *, label: str = "Target Identity"
) -> str | None:
    """Return a structural error, or ``None`` for one valid wire identity."""

    try:
        parse_target_identity(value, label=label)
    except ValueError as error:
        return str(error)
    return None


def identity_gate_error(
    local_identity: TargetIdentity,
    lidar_identity: object | None,
    camera_identity: object | None,
) -> str | None:
    """Apply the LiDAR-camera solver's exact three-way identity gate.

    ``None`` means the pair may be admitted.  Missing, malformed, and mismatched
    identities all return an operator-facing reason and therefore leave the capture
    buffer untouched.  The local identity is already loaded from a validated target,
    but it is parsed here too so this function remains safe as a standalone contract.
    """

    try:
        expected = parse_target_identity(local_identity, label="local Target Identity")
    except ValueError as error:
        return str(error)

    received: list[tuple[str, object | None]] = [
        ("LiDAR", lidar_identity),
        ("camera", camera_identity),
    ]
    parsed: list[tuple[str, TargetIdentity]] = []
    for label, value in received:
        try:
            parsed.append(
                (
                    label,
                    parse_target_identity(value, label=f"{label} Target Identity"),
                )
            )
        except ValueError as error:
            return str(error)

    for label, identity in parsed:
        if identity != expected:
            return (
                f"{label} Target Identity does not exactly match the local Target "
                f"Identity ({identity.target_id}@{identity.revision}, "
                f"{identity.semantic_sha256})"
            )

    first_label, first = parsed[0]
    second_label, second = parsed[1]
    if first != second:
        return (
            f"{first_label} and {second_label} Target Identities disagree; "
            "no Detection Pair will be accepted"
        )
    return None


@dataclass
class TargetIdentityGate:
    """Stateful identity gate for one solver lifetime.

    Observer identities are immutable for a launched graph.  A source changing its
    identity after announcing one is treated as a restart/protocol violation and
    permanently blocks this gate.  Main-node code owns the lock around this object.
    """

    local_identity: TargetIdentity

    def __post_init__(self) -> None:
        # Detach and validate even though ``load_target`` already returned a valid
        # identity; this prevents a caller from mutating a hand-built dataclass into
        # an accepted local identity.
        self.local_identity = parse_target_identity(
            self.local_identity, label="local Target Identity"
        )
        self._received: dict[str, TargetIdentity] = {}
        self._blocked_reason: str | None = None

    @property
    def ready(self) -> bool:
        return self.error is None

    @property
    def error(self) -> str | None:
        if self._blocked_reason is not None:
            return self._blocked_reason
        return identity_gate_error(
            self.local_identity,
            self._received.get("lidar"),
            self._received.get("camera"),
        )

    def update(self, source: str, value: object) -> str | None:
        """Record one source message and return the current gate error."""

        if source not in ("lidar", "camera"):
            raise ValueError(f"unknown Target Identity source '{source}'")
        if self._blocked_reason is not None:
            return self._blocked_reason
        try:
            identity = parse_target_identity(value, label=f"{source} Target Identity")
        except ValueError as error:
            self._blocked_reason = str(error)
            return self._blocked_reason

        previous = self._received.get(source)
        if previous is not None and previous != identity:
            self._blocked_reason = (
                f"{source} Target Identity changed during this solver session; "
                "restart the complete calibration graph"
            )
            return self._blocked_reason
        self._received[source] = identity
        return self.error


def identity_fields(identity: TargetIdentity) -> dict[str, Any]:
    """Return a ROS-message-compatible copy for tests and adapters."""

    parsed = parse_target_identity(identity, label="Target Identity")
    return {name: getattr(parsed, name) for name in _IDENTITY_FIELDS}


__all__ = [
    "BOARD_FRAME_CONVENTION",
    "CAMERA_TARGET_IDENTITY_TOPIC",
    "LIDAR_TARGET_IDENTITY_TOPIC",
    "TargetIdentity",
    "TargetIdentityGate",
    "ValidatedTarget",
    "identity_fields",
    "identity_gate_error",
    "load_target",
    "load_target_definition",
    "marker_geometry_summary",
    "parse_target_identity",
    "rotation_matrix_to_quaternion",
    "target_identity_error",
]
