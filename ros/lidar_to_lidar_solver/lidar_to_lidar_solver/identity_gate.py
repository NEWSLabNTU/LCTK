"""Target Identity comparison and ROS subscription adapter.

The LiDAR-to-LiDAR solver has no local target definition to compare against.  It
therefore accepts a synchronized detection pair only after the two detector
nodes have announced the exact same wire identity.  This module keeps that
policy independent from ROS message delivery and from transform solving.
"""

from __future__ import annotations

import re
from collections.abc import Callable, Mapping
from contextlib import nullcontext
from dataclasses import dataclass
from enum import Enum
from typing import Any

from lctk_interfaces.msg import CalibrationTargetIdentity
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

IDENTITY_FIELDS = (
    "schema_version",
    "target_id",
    "revision",
    "semantic_sha256",
    "board_frame_convention",
)
_SHA256 = re.compile(r"[0-9a-f]{64}\Z")


class IdentityStatus(str, Enum):
    """Outcome of comparing the two detector identities."""

    MISSING = "missing"
    MALFORMED = "malformed"
    MISMATCH = "mismatch"
    MATCH = "match"


@dataclass(frozen=True)
class IdentityComparison:
    """A pure comparison result suitable for logs and a callback gate."""

    status: IdentityStatus
    reason: str

    @property
    def accepted(self) -> bool:
        """Whether a detection pair may mutate solver state."""
        return self.status is IdentityStatus.MATCH


def identity_qos_profile() -> QoSProfile:
    """Return the latched QoS required for Target Identity announcements."""
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


def validate_identity(identity: object, *, label: str) -> str | None:
    """Return a structural error, or ``None`` for a valid identity message.

    The message has a fixed ROS shape, but validation is deliberately structural
    so the pure comparator can also be tested with mappings and small fakes.  A
    malformed message is never partially compared with the other side.
    """
    values = _identity_values(identity)
    if values is None:
        return f"{label} must contain exactly the five Target Identity fields"

    schema_version = values["schema_version"]
    if not _positive_int(schema_version):
        return f"{label}.schema_version must be a non-zero integer"

    target_id = values["target_id"]
    if not isinstance(target_id, str) or not target_id.strip():
        return f"{label}.target_id must be a non-empty string"

    revision = values["revision"]
    if not _positive_int(revision):
        return f"{label}.revision must be a non-zero integer"

    semantic_sha256 = values["semantic_sha256"]
    if (
        not isinstance(semantic_sha256, str)
        or _SHA256.fullmatch(semantic_sha256) is None
    ):
        return f"{label}.semantic_sha256 must be 64 lowercase hexadecimal characters"

    frame = values["board_frame_convention"]
    if not isinstance(frame, str) or not frame.strip():
        return f"{label}.board_frame_convention must be a non-empty string"
    return None


def compare_target_identities(
    lidar1_identity: object | None, lidar2_identity: object | None
) -> IdentityComparison:
    """Compare two wire identities without changing any state.

    ``None`` means the corresponding latched announcement has not arrived yet.
    All five fields are validated before equality is considered; matching only a
    target ID is intentionally not sufficient.
    """
    if lidar1_identity is None or lidar2_identity is None:
        missing = []
        if lidar1_identity is None:
            missing.append("lidar1")
        if lidar2_identity is None:
            missing.append("lidar2")
        return IdentityComparison(
            IdentityStatus.MISSING,
            f"waiting for Target Identity from {', '.join(missing)}",
        )

    error1 = validate_identity(lidar1_identity, label="lidar1 Target Identity")
    if error1 is not None:
        return IdentityComparison(IdentityStatus.MALFORMED, error1)
    error2 = validate_identity(lidar2_identity, label="lidar2 Target Identity")
    if error2 is not None:
        return IdentityComparison(IdentityStatus.MALFORMED, error2)

    values1 = _identity_values(lidar1_identity)
    values2 = _identity_values(lidar2_identity)
    assert values1 is not None and values2 is not None
    if tuple(values1[field] for field in IDENTITY_FIELDS) != tuple(
        values2[field] for field in IDENTITY_FIELDS
    ):
        return IdentityComparison(
            IdentityStatus.MISMATCH,
            "lidar1 and lidar2 Target Identity values do not exactly match",
        )
    return IdentityComparison(
        IdentityStatus.MATCH,
        "lidar1 and lidar2 Target Identity values match exactly",
    )


class TargetIdentityGate:
    """Hold one immutable identity per LiDAR input and expose a pair gate."""

    def __init__(self) -> None:
        self._identities: list[CalibrationTargetIdentity | None] = [None, None]
        self._blocked: IdentityComparison | None = None

    @property
    def identities(self) -> tuple[CalibrationTargetIdentity | None, ...]:
        """Current valid announcements, mainly for diagnostics/tests."""
        return tuple(
            None if value is None else _copy_identity(value)
            for value in self._identities
        )

    def update(self, lidar_index: int, identity: object) -> IdentityComparison:
        """Validate and remember one announcement, preserving lifetime identity.

        A detector is expected to publish once.  If it later announces a
        different identity, the gate remains closed for this solver lifetime;
        silently switching targets would reinterpret already queued detections.
        """
        if lidar_index not in (0, 1):
            raise ValueError("lidar_index must be 0 or 1")
        if self._blocked is not None:
            return self._blocked
        label = f"lidar{lidar_index + 1} Target Identity"
        error = validate_identity(identity, label=label)
        if error is not None:
            self._blocked = IdentityComparison(IdentityStatus.MALFORMED, error)
            return self._blocked

        values = _identity_values(identity)
        assert values is not None
        current = self._identities[lidar_index]
        if current is not None:
            current_values = _identity_values(current)
            assert current_values is not None
            if tuple(current_values[field] for field in IDENTITY_FIELDS) != tuple(
                values[field] for field in IDENTITY_FIELDS
            ):
                self._blocked = IdentityComparison(
                    IdentityStatus.MISMATCH,
                    f"{label} changed during solver lifetime; restart the solver",
                )
                return self._blocked

        # Store a ROS message-shaped copy, not a mutable caller-owned object.
        self._identities[lidar_index] = _copy_identity(identity)
        return self.compare()

    def compare(self) -> IdentityComparison:
        """Return the current gate outcome without mutating state."""
        if self._blocked is not None:
            return self._blocked
        return compare_target_identities(*self._identities)


class TargetIdentitySubscriptions:
    """Create the two relative, remappable identity subscriptions for a node."""

    TOPICS = ("lidar1_target_identity", "lidar2_target_identity")

    def __init__(
        self,
        node: Any,
        gate: TargetIdentityGate,
        *,
        on_update: Callable[[int, IdentityComparison], None] | None = None,
        update_lock: Any | None = None,
    ) -> None:
        """Subscribe to both identities, optionally under one owner lock."""
        qos = identity_qos_profile()
        self._subscriptions = [
            node.create_subscription(
                CalibrationTargetIdentity,
                topic,
                self._callback(gate, index, on_update, update_lock),
                qos,
            )
            for index, topic in enumerate(self.TOPICS)
        ]

    @staticmethod
    def _callback(
        gate: TargetIdentityGate,
        index: int,
        on_update: Callable[[int, IdentityComparison], None] | None,
        update_lock: Any | None,
    ):
        def receive(identity: CalibrationTargetIdentity) -> None:
            context = nullcontext() if update_lock is None else update_lock
            with context:
                result = gate.update(index, identity)
                if on_update is not None:
                    on_update(index, result)

        return receive


def _identity_values(identity: object) -> dict[str, Any] | None:
    if isinstance(identity, Mapping):
        if set(identity) != set(IDENTITY_FIELDS):
            return None
        return {field: identity[field] for field in IDENTITY_FIELDS}
    if all(hasattr(identity, field) for field in IDENTITY_FIELDS):
        return {field: getattr(identity, field) for field in IDENTITY_FIELDS}
    return None


def _copy_identity(identity: object) -> CalibrationTargetIdentity:
    values = _identity_values(identity)
    assert values is not None
    return CalibrationTargetIdentity(**values)


def _positive_int(value: object) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and value > 0


__all__ = [
    "IDENTITY_FIELDS",
    "IdentityComparison",
    "IdentityStatus",
    "TargetIdentityGate",
    "TargetIdentitySubscriptions",
    "compare_target_identities",
    "identity_qos_profile",
    "validate_identity",
]
