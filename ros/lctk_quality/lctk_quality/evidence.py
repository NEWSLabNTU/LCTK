"""Deterministic field-evidence sidecars and reports.

This module is deliberately independent of ROS and ``rosbag2``.  A replay adapter
turns messages from a bag into :class:`EvidenceSample` values; the code here owns
the part that must stay reproducible and reviewable: manifest validation, interval
selection, structured per-frame evidence, denominators, and artifact indexing.

The normalized input boundary is intentional.  ROS diagnostics have changed while
the single-source-target migration is in progress, and silently guessing a topic or a
message field would make a field report impossible to audit.  A future bag adapter
can depend on this module without changing the committed report format.
"""

from __future__ import annotations

import hashlib
import json
import math
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Final, Literal, TypeAlias

MANIFEST_SCHEMA_VERSION: Final = 1
REPORT_SCHEMA_VERSION: Final = 1
INTERVAL_LABELS: Final = ("visible", "absent", "stationary")
PROVENANCES: Final = ("field", "test_only")

IntervalLabel: TypeAlias = Literal["visible", "absent", "stationary"]
Provenance: TypeAlias = Literal["field", "test_only"]
JsonScalar: TypeAlias = None | bool | int | float | str
JsonValue: TypeAlias = JsonScalar | list["JsonValue"] | dict[str, "JsonValue"]


class EvidenceSchemaError(ValueError):
    """Raised when a sidecar, sample, or report violates its wire contract."""


def _fail(field: str, message: str) -> EvidenceSchemaError:
    return EvidenceSchemaError(f"{field}: {message}")


def _is_int(value: object) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _nonnegative_int(value: object, field: str) -> int:
    if not _is_int(value) or value < 0:
        raise _fail(field, "must be a non-negative integer")
    return int(value)


def _positive_int(value: object, field: str) -> int:
    result = _nonnegative_int(value, field)
    if result == 0:
        raise _fail(field, "must be greater than zero")
    return result


def _string(value: object, field: str, *, nonempty: bool = True) -> str:
    if not isinstance(value, str) or (nonempty and not value.strip()):
        suffix = " non-empty" if nonempty else ""
        raise _fail(field, f"must be a{suffix} string")
    return value


def _finite_float(value: object, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise _fail(field, "must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise _fail(field, "must be a finite number")
    return result


def _json_value(value: object, field: str) -> JsonValue:
    """Validate JSON values before they become part of a deterministic report."""
    if value is None or isinstance(value, (bool, int, str)):
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            raise _fail(field, "must not contain NaN or infinity")
        return value
    if isinstance(value, Mapping):
        result: dict[str, JsonValue] = {}
        for key, item in value.items():
            if not isinstance(key, str):
                raise _fail(field, "object keys must be strings")
            result[key] = _json_value(item, f"{field}.{key}")
        return result
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes, bytearray)):
        return [
            _json_value(item, f"{field}[{index}]") for index, item in enumerate(value)
        ]
    raise _fail(field, f"contains unsupported value {type(value).__name__}")


def canonical_json(value: object) -> str:
    """Return stable compact JSON used for report hashes and exact comparisons."""
    checked = _json_value(value, "json")
    try:
        return json.dumps(
            checked,
            ensure_ascii=True,
            allow_nan=False,
            sort_keys=True,
            separators=(",", ":"),
        )
    except (TypeError, ValueError) as error:
        raise EvidenceSchemaError(f"json: cannot serialize value: {error}") from error


def canonical_bytes(value: object) -> bytes:
    return canonical_json(value).encode("utf-8")


def _sha256_hex(value: object, field: str) -> str:
    result = _string(value, field).lower()
    if len(result) != 64 or any(
        character not in "0123456789abcdef" for character in result
    ):
        raise _fail(field, "must be a lowercase or uppercase SHA-256 hex digest")
    return result


def sha256_file(path: str | Path) -> str:
    """Hash a bag or artifact without loading it into memory."""
    digest = hashlib.sha256()
    try:
        with Path(path).open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as error:
        raise EvidenceSchemaError(f"bag.path: cannot hash {path!s}: {error}") from error
    return digest.hexdigest()


@dataclass(frozen=True)
class TargetIdentityRecord:
    """Exact five-field identity copied from ``CalibrationTargetIdentity.msg``."""

    schema_version: int
    target_id: str
    revision: int
    semantic_sha256: str
    board_frame_convention: str

    def __post_init__(self) -> None:
        _positive_int(self.schema_version, "target_identity.schema_version")
        _string(self.target_id, "target_identity.target_id")
        _positive_int(self.revision, "target_identity.revision")
        object.__setattr__(
            self,
            "semantic_sha256",
            _sha256_hex(self.semantic_sha256, "target_identity.semantic_sha256"),
        )
        _string(
            self.board_frame_convention,
            "target_identity.board_frame_convention",
        )

    @classmethod
    def from_mapping(
        cls, value: Mapping[str, object], *, field: str = "target_identity"
    ):
        required = {
            "schema_version",
            "target_id",
            "revision",
            "semantic_sha256",
            "board_frame_convention",
        }
        _exact_keys(value, required, field)
        return cls(
            _positive_int(value["schema_version"], f"{field}.schema_version"),
            _string(value["target_id"], f"{field}.target_id"),
            _positive_int(value["revision"], f"{field}.revision"),
            _sha256_hex(value["semantic_sha256"], f"{field}.semantic_sha256"),
            _string(value["board_frame_convention"], f"{field}.board_frame_convention"),
        )

    @classmethod
    def from_object(cls, value: object, *, field: str = "target_identity"):
        """Copy a ROS-free/domain identity object without importing that package."""
        if isinstance(value, Mapping):
            return cls.from_mapping(value, field=field)
        names = (
            "schema_version",
            "target_id",
            "revision",
            "semantic_sha256",
            "board_frame_convention",
        )
        if not all(hasattr(value, name) for name in names):
            raise _fail(field, "must be an identity mapping or identity object")
        return cls.from_mapping(
            {name: getattr(value, name) for name in names}, field=field
        )

    def to_dict(self) -> dict[str, JsonValue]:
        return {
            "schema_version": self.schema_version,
            "target_id": self.target_id,
            "revision": self.revision,
            "semantic_sha256": self.semantic_sha256,
            "board_frame_convention": self.board_frame_convention,
        }


@dataclass(frozen=True)
class BagFingerprint:
    """Content identity for a bag; paths are optional and never hashed into reports."""

    sha256: str
    size_bytes: int | None = None
    storage_id: str | None = None
    relative_path: str | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "sha256", _sha256_hex(self.sha256, "bag.sha256"))
        if self.size_bytes is not None:
            _nonnegative_int(self.size_bytes, "bag.size_bytes")
        if self.storage_id is not None:
            _string(self.storage_id, "bag.storage_id")
        if self.relative_path is not None:
            _relative_path(self.relative_path, "bag.relative_path")

    @classmethod
    def from_mapping(cls, value: Mapping[str, object], *, field: str = "bag"):
        _exact_keys(
            value, {"sha256", "size_bytes", "storage_id", "relative_path"}, field
        )
        return cls(
            _sha256_hex(value["sha256"], f"{field}.sha256"),
            None
            if value["size_bytes"] is None
            else _nonnegative_int(value["size_bytes"], f"{field}.size_bytes"),
            None
            if value["storage_id"] is None
            else _string(value["storage_id"], f"{field}.storage_id"),
            None
            if value["relative_path"] is None
            else _relative_path(value["relative_path"], f"{field}.relative_path"),
        )

    @classmethod
    def from_file(
        cls,
        path: str | Path,
        *,
        storage_id: str | None = None,
        relative_path: str | None = None,
    ):
        file_path = Path(path)
        try:
            size = file_path.stat().st_size
        except OSError as error:
            raise EvidenceSchemaError(
                f"bag.path: cannot stat {path!s}: {error}"
            ) from error
        return cls(
            sha256_file(file_path),
            size_bytes=size,
            storage_id=storage_id,
            relative_path=relative_path,
        )

    def to_dict(self) -> dict[str, JsonValue]:
        return {
            "sha256": self.sha256,
            "size_bytes": self.size_bytes,
            "storage_id": self.storage_id,
            "relative_path": self.relative_path,
        }


@dataclass(frozen=True)
class EvidenceInterval:
    """Half-open labelled time interval: ``start_ns <= t < end_ns``."""

    label: IntervalLabel
    start_ns: int
    end_ns: int
    name: str | None = None

    def __post_init__(self) -> None:
        if self.label not in INTERVAL_LABELS:
            raise _fail(
                "interval.label",
                f"expected one of {INTERVAL_LABELS}, got {self.label!r}",
            )
        _nonnegative_int(self.start_ns, "interval.start_ns")
        _positive_int(self.end_ns, "interval.end_ns")
        if self.end_ns <= self.start_ns:
            raise _fail("interval.end_ns", "must be greater than interval.start_ns")
        if self.name is not None:
            _string(self.name, "interval.name")

    @classmethod
    def from_mapping(cls, value: Mapping[str, object], *, index: int):
        field = f"intervals[{index}]"
        _exact_keys(value, {"label", "start_ns", "end_ns", "name"}, field)
        label = _string(value["label"], f"{field}.label")
        if label not in INTERVAL_LABELS:
            raise _fail(f"{field}.label", f"expected one of {INTERVAL_LABELS}")
        return cls(
            label,  # type: ignore[arg-type]
            _nonnegative_int(value["start_ns"], f"{field}.start_ns"),
            _positive_int(value["end_ns"], f"{field}.end_ns"),
            None if value["name"] is None else _string(value["name"], f"{field}.name"),
        )

    def to_dict(self) -> dict[str, JsonValue]:
        result: dict[str, JsonValue] = {
            "label": self.label,
            "start_ns": self.start_ns,
            "end_ns": self.end_ns,
            "name": self.name,
        }
        return result


@dataclass(frozen=True)
class EvidenceManifest:
    """Versioned, commit-friendly description of one bag evaluation."""

    bag: BagFingerprint
    target_identity: TargetIdentityRecord
    sensor: str
    preset: str
    topics: Mapping[str, str]
    intervals: tuple[EvidenceInterval, ...]
    provenance: Provenance = "field"
    schema_version: int = MANIFEST_SCHEMA_VERSION

    def __post_init__(self) -> None:
        if self.schema_version != MANIFEST_SCHEMA_VERSION:
            raise _fail("schema_version", f"expected {MANIFEST_SCHEMA_VERSION}")
        _string(self.sensor, "sensor")
        _string(self.preset, "preset")
        if self.provenance not in PROVENANCES:
            raise _fail("provenance", f"expected one of {PROVENANCES}")
        if not self.topics:
            raise _fail("topics", "must contain at least one topic binding")
        for role, topic in self.topics.items():
            _string(role, "topics role")
            _string(topic, f"topics[{role!r}]")
        if not self.intervals:
            raise _fail("intervals", "must contain at least one labelled interval")
        seen: set[tuple[object, ...]] = set()
        for interval in self.intervals:
            key = (interval.label, interval.start_ns, interval.end_ns, interval.name)
            if key in seen:
                raise _fail("intervals", f"duplicate interval {key!r}")
            seen.add(key)
        object.__setattr__(
            self, "topics", {key: self.topics[key] for key in sorted(self.topics)}
        )
        object.__setattr__(
            self,
            "intervals",
            tuple(
                sorted(
                    self.intervals,
                    key=lambda interval: (
                        INTERVAL_LABELS.index(interval.label),
                        interval.start_ns,
                        interval.end_ns,
                        interval.name or "",
                    ),
                )
            ),
        )

    @classmethod
    def from_mapping(cls, value: Mapping[str, object]):
        _exact_keys(
            value,
            {
                "schema_version",
                "bag",
                "target_identity",
                "sensor",
                "preset",
                "topics",
                "intervals",
                "provenance",
            },
            "manifest",
        )
        version = _positive_int(value["schema_version"], "manifest.schema_version")
        bag = _mapping(value["bag"], "manifest.bag")
        identity = _mapping(value["target_identity"], "manifest.target_identity")
        topics_raw = _mapping(value["topics"], "manifest.topics")
        topics = {
            _string(role, "manifest.topics role"): _string(
                topic, f"manifest.topics[{role!r}]"
            )
            for role, topic in topics_raw.items()
        }
        intervals_raw = _list(value["intervals"], "manifest.intervals")
        intervals = tuple(
            EvidenceInterval.from_mapping(
                _mapping(item, f"manifest.intervals[{index}]"), index=index
            )
            for index, item in enumerate(intervals_raw)
        )
        provenance = _string(value["provenance"], "manifest.provenance")
        if provenance not in PROVENANCES:
            raise _fail("manifest.provenance", f"expected one of {PROVENANCES}")
        return cls(
            BagFingerprint.from_mapping(bag, field="manifest.bag"),
            TargetIdentityRecord.from_mapping(
                identity, field="manifest.target_identity"
            ),
            _string(value["sensor"], "manifest.sensor"),
            _string(value["preset"], "manifest.preset"),
            topics,
            intervals,
            provenance,  # type: ignore[arg-type]
            version,
        )

    @classmethod
    def from_json(cls, text: str):
        try:
            value = json.loads(text)
        except json.JSONDecodeError as error:
            raise EvidenceSchemaError(f"manifest: invalid JSON: {error}") from error
        return cls.from_mapping(_mapping(value, "manifest"))

    @classmethod
    def load(cls, path: str | Path):
        try:
            return cls.from_json(Path(path).read_text(encoding="utf-8"))
        except OSError as error:
            raise EvidenceSchemaError(
                f"manifest: cannot read {path!s}: {error}"
            ) from error

    def to_dict(self) -> dict[str, JsonValue]:
        sorted_intervals = sorted(
            self.intervals,
            key=lambda interval: (
                INTERVAL_LABELS.index(interval.label),
                interval.start_ns,
                interval.end_ns,
                interval.name or "",
            ),
        )
        return {
            "schema_version": self.schema_version,
            "bag": self.bag.to_dict(),
            "target_identity": self.target_identity.to_dict(),
            "sensor": self.sensor,
            "preset": self.preset,
            "topics": {key: self.topics[key] for key in sorted(self.topics)},
            "intervals": [interval.to_dict() for interval in sorted_intervals],
            "provenance": self.provenance,
        }

    def canonical_bytes(self) -> bytes:
        return canonical_bytes(self.to_dict())

    def sha256(self) -> str:
        return hashlib.sha256(self.canonical_bytes()).hexdigest()

    def to_json(self) -> str:
        return (
            json.dumps(self.to_dict(), ensure_ascii=True, sort_keys=True, indent=2)
            + "\n"
        )

    def write(self, path: str | Path) -> None:
        _write_text(path, self.to_json(), "manifest")


@dataclass(frozen=True)
class PoseRecord:
    """Board pose plus optional 6x6 covariance, all in the reported frame."""

    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]
    covariance: tuple[float, ...] | None = None

    def __post_init__(self) -> None:
        if len(self.position) != 3:
            raise _fail("pose.position", "must contain three values")
        if len(self.orientation) != 4:
            raise _fail("pose.orientation", "must contain four values")
        for index, value in enumerate(self.position):
            _finite_float(value, f"pose.position[{index}]")
        for index, value in enumerate(self.orientation):
            _finite_float(value, f"pose.orientation[{index}]")
        if self.covariance is not None:
            if len(self.covariance) != 36:
                raise _fail("pose.covariance", "must contain exactly 36 values")
            for index, value in enumerate(self.covariance):
                _finite_float(value, f"pose.covariance[{index}]")

    @classmethod
    def from_mapping(cls, value: Mapping[str, object], *, field: str = "pose"):
        _exact_keys(value, {"position", "orientation", "covariance"}, field)
        position = _numbers(value["position"], f"{field}.position", expected=3)
        orientation = _numbers(value["orientation"], f"{field}.orientation", expected=4)
        covariance_value = value["covariance"]
        covariance = (
            None
            if covariance_value is None
            else _numbers(covariance_value, f"{field}.covariance", expected=36)
        )
        return cls(position, orientation, covariance)

    def to_dict(self) -> dict[str, JsonValue]:
        return {
            "position": list(self.position),
            "orientation": list(self.orientation),
            "covariance": None if self.covariance is None else list(self.covariance),
        }


@dataclass(frozen=True)
class RejectionReason:
    """Typed enough for counting, extensible enough for detector evidence."""

    code: str
    detail: str | None = None
    evidence: Mapping[str, JsonValue] | None = None

    def __post_init__(self) -> None:
        _string(self.code, "rejection.code")
        if self.detail is not None:
            _string(self.detail, "rejection.detail", nonempty=False)
        if self.evidence is not None:
            _json_value(self.evidence, "rejection.evidence")

    @classmethod
    def from_mapping(cls, value: Mapping[str, object], *, field: str = "rejection"):
        _exact_keys(value, {"code", "detail", "evidence"}, field)
        evidence_value = value["evidence"]
        if evidence_value is not None and not isinstance(evidence_value, Mapping):
            raise _fail(f"{field}.evidence", "must be an object or null")
        checked = (
            None
            if evidence_value is None
            else _json_value(evidence_value, f"{field}.evidence")
        )
        assert checked is None or isinstance(checked, dict)
        return cls(
            _string(value["code"], f"{field}.code"),
            None
            if value["detail"] is None
            else _string(value["detail"], f"{field}.detail", nonempty=False),
            checked,
        )

    def to_dict(self) -> dict[str, JsonValue]:
        return {"code": self.code, "detail": self.detail, "evidence": self.evidence}


@dataclass(frozen=True)
class ArucoObservation:
    """One camera-side marker observation preserved in evidence."""

    marker_id: int
    corners: tuple[tuple[float, float], ...]
    score: float | None = None

    def __post_init__(self) -> None:
        _nonnegative_int(self.marker_id, "aruco.marker_id")
        if len(self.corners) != 4:
            raise _fail("aruco.corners", "must contain four corners")
        for corner_index, corner in enumerate(self.corners):
            if len(corner) != 2:
                raise _fail(f"aruco.corners[{corner_index}]", "must contain x and y")
            for axis, value in enumerate(corner):
                _finite_float(value, f"aruco.corners[{corner_index}][{axis}]")
        if self.score is not None:
            _finite_float(self.score, "aruco.score")

    @classmethod
    def from_mapping(cls, value: Mapping[str, object], *, index: int):
        field = f"aruco_observations[{index}]"
        _exact_keys(value, {"marker_id", "corners", "score"}, field)
        marker_id = _nonnegative_int(value["marker_id"], f"{field}.marker_id")
        corners_raw = _list(value["corners"], f"{field}.corners")
        if len(corners_raw) != 4:
            raise _fail(f"{field}.corners", "must contain four corners")
        corners = tuple(
            _numbers(corner, f"{field}.corners[{corner_index}]", expected=2)
            for corner_index, corner in enumerate(corners_raw)
        )
        score = (
            None
            if value["score"] is None
            else _finite_float(value["score"], f"{field}.score")
        )
        return cls(marker_id, corners, score)

    def to_dict(self) -> dict[str, JsonValue]:
        return {
            "marker_id": self.marker_id,
            "corners": [list(corner) for corner in self.corners],
            "score": self.score,
        }


@dataclass(frozen=True)
class ArtifactRef:
    """Commit-friendly pointer to an untracked image, trace, or solver output."""

    artifact_id: str
    kind: str
    relative_path: str
    sha256: str | None = None
    timestamp_ns: int | None = None

    def __post_init__(self) -> None:
        _string(self.artifact_id, "artifact.artifact_id")
        _string(self.kind, "artifact.kind")
        _relative_path(self.relative_path, "artifact.relative_path")
        if self.sha256 is not None:
            object.__setattr__(
                self, "sha256", _sha256_hex(self.sha256, "artifact.sha256")
            )
        if self.timestamp_ns is not None:
            _nonnegative_int(self.timestamp_ns, "artifact.timestamp_ns")

    @classmethod
    def from_mapping(cls, value: Mapping[str, object], *, index: int):
        field = f"artifacts[{index}]"
        _exact_keys(
            value,
            {"artifact_id", "kind", "relative_path", "sha256", "timestamp_ns"},
            field,
        )
        return cls(
            _string(value["artifact_id"], f"{field}.artifact_id"),
            _string(value["kind"], f"{field}.kind"),
            _relative_path(value["relative_path"], f"{field}.relative_path"),
            None
            if value["sha256"] is None
            else _sha256_hex(value["sha256"], f"{field}.sha256"),
            None
            if value["timestamp_ns"] is None
            else _nonnegative_int(value["timestamp_ns"], f"{field}.timestamp_ns"),
        )

    def to_dict(self) -> dict[str, JsonValue]:
        return {
            "artifact_id": self.artifact_id,
            "kind": self.kind,
            "relative_path": self.relative_path,
            "sha256": self.sha256,
            "timestamp_ns": self.timestamp_ns,
        }


@dataclass(frozen=True)
class EvidenceSample:
    """One normalized timestamp from a replay/extraction adapter."""

    timestamp_ns: int
    accepted: bool
    target_identity: TargetIdentityRecord | None = None
    pose: PoseRecord | None = None
    rejection: RejectionReason | None = None
    alignment_dot: float | None = None
    quadrant: int | None = None
    aruco_observations: tuple[ArucoObservation, ...] = ()
    solver_outputs: Mapping[str, JsonValue] | None = None
    artifact_ids: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        _nonnegative_int(self.timestamp_ns, "sample.timestamp_ns")
        if not isinstance(self.accepted, bool):
            raise _fail("sample.accepted", "must be boolean")
        if self.accepted:
            if self.target_identity is None:
                raise _fail(
                    "sample.target_identity", "accepted sample needs target identity"
                )
            if self.pose is None:
                raise _fail("sample.pose", "accepted sample needs pose")
            if self.rejection is not None:
                raise _fail("sample.rejection", "accepted sample cannot have rejection")
        elif self.rejection is None:
            raise _fail("sample.rejection", "rejected sample needs structured reason")
        if self.alignment_dot is not None:
            value = _finite_float(self.alignment_dot, "sample.alignment_dot")
            if value < -1.0 or value > 1.0:
                raise _fail("sample.alignment_dot", "must be in [-1, 1]")
        if self.quadrant is not None:
            quadrant = _nonnegative_int(self.quadrant, "sample.quadrant")
            if quadrant not in range(4):
                raise _fail("sample.quadrant", "must be one of 0, 1, 2, 3")
        if self.solver_outputs is not None:
            _json_value(self.solver_outputs, "sample.solver_outputs")
        for artifact_id in self.artifact_ids:
            _string(artifact_id, "sample.artifact_ids[]")

    @classmethod
    def from_mapping(cls, value: Mapping[str, object], *, index: int):
        field = f"samples[{index}]"
        _exact_keys(
            value,
            {
                "timestamp_ns",
                "accepted",
                "target_identity",
                "pose",
                "rejection",
                "alignment_dot",
                "quadrant",
                "aruco_observations",
                "solver_outputs",
                "artifact_ids",
            },
            field,
        )
        identity_value = value["target_identity"]
        pose_value = value["pose"]
        rejection_value = value["rejection"]
        aruco_raw = _list(value["aruco_observations"], f"{field}.aruco_observations")
        artifacts_raw = _list(value["artifact_ids"], f"{field}.artifact_ids")
        outputs_value = value["solver_outputs"]
        if outputs_value is not None and not isinstance(outputs_value, Mapping):
            raise _fail(f"{field}.solver_outputs", "must be an object or null")
        accepted = value["accepted"]
        if not isinstance(accepted, bool):
            raise _fail(f"{field}.accepted", "must be boolean")
        return cls(
            _nonnegative_int(value["timestamp_ns"], f"{field}.timestamp_ns"),
            accepted,
            None
            if identity_value is None
            else TargetIdentityRecord.from_mapping(
                _mapping(identity_value, f"{field}.target_identity"),
                field=f"{field}.target_identity",
            ),
            None
            if pose_value is None
            else PoseRecord.from_mapping(
                _mapping(pose_value, f"{field}.pose"), field=f"{field}.pose"
            ),
            None
            if rejection_value is None
            else RejectionReason.from_mapping(
                _mapping(rejection_value, f"{field}.rejection"),
                field=f"{field}.rejection",
            ),
            None
            if value["alignment_dot"] is None
            else _finite_float(value["alignment_dot"], f"{field}.alignment_dot"),
            None
            if value["quadrant"] is None
            else _nonnegative_int(value["quadrant"], f"{field}.quadrant"),
            tuple(
                ArucoObservation.from_mapping(
                    _mapping(item, f"{field}.aruco_observations[{aruco_index}]"),
                    index=aruco_index,
                )
                for aruco_index, item in enumerate(aruco_raw)
            ),
            None
            if outputs_value is None
            else _json_value(outputs_value, f"{field}.solver_outputs"),
            tuple(
                _string(item, f"{field}.artifact_ids[{artifact_index}]")
                for artifact_index, item in enumerate(artifacts_raw)
            ),
        )

    def to_dict(
        self, *, labels: Sequence[IntervalLabel] | None = None
    ) -> dict[str, JsonValue]:
        result: dict[str, JsonValue] = {
            "timestamp_ns": self.timestamp_ns,
            "accepted": self.accepted,
            "target_identity": None
            if self.target_identity is None
            else self.target_identity.to_dict(),
            "pose": None if self.pose is None else self.pose.to_dict(),
            "rejection": None if self.rejection is None else self.rejection.to_dict(),
            "alignment_dot": self.alignment_dot,
            "quadrant": self.quadrant,
            "aruco_observations": [item.to_dict() for item in self.aruco_observations],
            "solver_outputs": self.solver_outputs,
            "artifact_ids": list(self.artifact_ids),
        }
        if labels is not None:
            result["labels"] = list(labels)
        return result


@dataclass(frozen=True)
class EvidenceReport:
    """Versioned deterministic output suitable for review or later aggregation."""

    manifest_sha256: str
    bag_sha256: str
    target_identity: TargetIdentityRecord
    sensor: str
    preset: str
    provenance: Provenance
    selected_timestamps_ns: tuple[int, ...]
    frames: tuple[dict[str, JsonValue], ...]
    denominators: Mapping[str, Mapping[str, int]]
    summary: Mapping[str, JsonValue]
    artifacts: tuple[ArtifactRef, ...]
    schema_version: int = REPORT_SCHEMA_VERSION

    def __post_init__(self) -> None:
        if self.schema_version != REPORT_SCHEMA_VERSION:
            raise _fail("report.schema_version", f"expected {REPORT_SCHEMA_VERSION}")
        _sha256_hex(self.manifest_sha256, "report.manifest_sha256")
        _sha256_hex(self.bag_sha256, "report.bag_sha256")
        _string(self.sensor, "report.sensor")
        _string(self.preset, "report.preset")
        if self.provenance not in PROVENANCES:
            raise _fail("report.provenance", f"expected one of {PROVENANCES}")
        if tuple(sorted(self.selected_timestamps_ns)) != self.selected_timestamps_ns:
            raise _fail("report.selected_timestamps_ns", "must be sorted")
        if len(set(self.selected_timestamps_ns)) != len(self.selected_timestamps_ns):
            raise _fail("report.selected_timestamps_ns", "must not contain duplicates")
        if len(self.frames) != len(self.selected_timestamps_ns):
            raise _fail("report.frames", "must be parallel to selected_timestamps_ns")
        for index, timestamp_ns in enumerate(self.selected_timestamps_ns):
            _nonnegative_int(timestamp_ns, f"report.selected_timestamps_ns[{index}]")
        _json_value(self.frames, "report.frames")
        _json_value(self.denominators, "report.denominators")
        _json_value(self.summary, "report.summary")
        ids = [artifact.artifact_id for artifact in self.artifacts]
        if ids != sorted(ids):
            raise _fail("report.artifacts", "must be sorted by artifact_id")
        if len(ids) != len(set(ids)):
            raise _fail("report.artifacts", "artifact_id values must be unique")

    @classmethod
    def from_mapping(cls, value: Mapping[str, object]):
        """Validate and rebuild a report previously written as JSON."""
        _exact_keys(
            value,
            {
                "schema_version",
                "manifest_sha256",
                "bag_sha256",
                "target_identity",
                "sensor",
                "preset",
                "provenance",
                "selected_timestamps_ns",
                "frames",
                "denominators",
                "summary",
                "artifacts",
            },
            "report",
        )
        timestamps_raw = _list(
            value["selected_timestamps_ns"], "report.selected_timestamps_ns"
        )
        timestamps = tuple(
            _nonnegative_int(item, f"report.selected_timestamps_ns[{index}]")
            for index, item in enumerate(timestamps_raw)
        )
        frames_raw = _list(value["frames"], "report.frames")
        frames: list[dict[str, JsonValue]] = []
        for index, frame_value in enumerate(frames_raw):
            frame = _mapping(frame_value, f"report.frames[{index}]")
            frames.append(_report_frame(frame, index=index, timestamps=timestamps))

        denominators = _report_denominators(value["denominators"])
        summary = _mapping(value["summary"], "report.summary")
        artifacts_raw = _list(value["artifacts"], "report.artifacts")
        artifacts = tuple(
            ArtifactRef.from_mapping(
                _mapping(item, f"report.artifacts[{index}]"), index=index
            )
            for index, item in enumerate(artifacts_raw)
        )
        provenance = _string(value["provenance"], "report.provenance")
        if provenance not in PROVENANCES:
            raise _fail("report.provenance", f"expected one of {PROVENANCES}")
        return cls(
            _sha256_hex(value["manifest_sha256"], "report.manifest_sha256"),
            _sha256_hex(value["bag_sha256"], "report.bag_sha256"),
            TargetIdentityRecord.from_mapping(
                _mapping(value["target_identity"], "report.target_identity"),
                field="report.target_identity",
            ),
            _string(value["sensor"], "report.sensor"),
            _string(value["preset"], "report.preset"),
            provenance,  # type: ignore[arg-type]
            timestamps,
            tuple(frames),
            denominators,
            summary,
            artifacts,
            _positive_int(value["schema_version"], "report.schema_version"),
        )

    @classmethod
    def from_json(cls, text: str):
        try:
            value = json.loads(text)
        except json.JSONDecodeError as error:
            raise EvidenceSchemaError(f"report: invalid JSON: {error}") from error
        return cls.from_mapping(_mapping(value, "report"))

    @classmethod
    def load(cls, path: str | Path):
        try:
            return cls.from_json(Path(path).read_text(encoding="utf-8"))
        except OSError as error:
            raise EvidenceSchemaError(
                f"report: cannot read {path!s}: {error}"
            ) from error

    def to_dict(self) -> dict[str, JsonValue]:
        result = {
            "schema_version": self.schema_version,
            "manifest_sha256": self.manifest_sha256,
            "bag_sha256": self.bag_sha256,
            "target_identity": self.target_identity.to_dict(),
            "sensor": self.sensor,
            "preset": self.preset,
            "provenance": self.provenance,
            "selected_timestamps_ns": list(self.selected_timestamps_ns),
            "frames": list(self.frames),
            "denominators": self.denominators,
            "summary": self.summary,
            "artifacts": [artifact.to_dict() for artifact in self.artifacts],
        }
        checked = _json_value(result, "report")
        assert isinstance(checked, dict)
        return checked

    def canonical_bytes(self) -> bytes:
        return canonical_bytes(self.to_dict())

    def sha256(self) -> str:
        return hashlib.sha256(self.canonical_bytes()).hexdigest()

    def to_json(self) -> str:
        return (
            json.dumps(self.to_dict(), ensure_ascii=True, sort_keys=True, indent=2)
            + "\n"
        )

    def write(self, path: str | Path) -> None:
        _write_text(path, self.to_json(), "report")


def _report_frame(
    value: Mapping[str, object], *, index: int, timestamps: tuple[int, ...]
) -> dict[str, JsonValue]:
    _exact_keys(
        value,
        {
            "timestamp_ns",
            "labels",
            "accepted",
            "target_identity",
            "pose",
            "rejection",
            "alignment_dot",
            "quadrant",
            "aruco_observations",
            "solver_outputs",
            "artifact_ids",
        },
        f"report.frames[{index}]",
    )
    labels_raw = _list(value["labels"], f"report.frames[{index}].labels")
    labels: list[IntervalLabel] = []
    for label_index, label_value in enumerate(labels_raw):
        label = _string(label_value, f"report.frames[{index}].labels[{label_index}]")
        if label not in INTERVAL_LABELS:
            raise _fail(
                f"report.frames[{index}].labels[{label_index}]",
                f"expected one of {INTERVAL_LABELS}",
            )
        if label in labels:
            raise _fail(f"report.frames[{index}].labels", f"duplicate label {label!r}")
        labels.append(label)  # type: ignore[arg-type]
    if tuple(labels) != tuple(label for label in INTERVAL_LABELS if label in labels):
        raise _fail(
            f"report.frames[{index}].labels",
            "must use the fixed order visible, absent, stationary",
        )
    if index >= len(timestamps) or value["timestamp_ns"] != timestamps[index]:
        raise _fail(
            f"report.frames[{index}].timestamp_ns",
            "must match selected_timestamps_ns at the same index",
        )
    sample_value = {key: item for key, item in value.items() if key != "labels"}
    sample = EvidenceSample.from_mapping(sample_value, index=index)
    return sample.to_dict(labels=labels)


def _report_denominators(value: object) -> dict[str, dict[str, int]]:
    raw = _mapping(value, "report.denominators")
    if set(raw) != set(INTERVAL_LABELS):
        raise _fail(
            "report.denominators",
            f"must contain exactly {', '.join(INTERVAL_LABELS)}",
        )
    result: dict[str, dict[str, int]] = {}
    for label in INTERVAL_LABELS:
        entry = _mapping(raw[label], f"report.denominators.{label}")
        _exact_keys(
            entry, {"frames", "accepted", "rejected"}, f"report.denominators.{label}"
        )
        result[label] = {
            key: _nonnegative_int(entry[key], f"report.denominators.{label}.{key}")
            for key in ("frames", "accepted", "rejected")
        }
        if (
            result[label]["accepted"] + result[label]["rejected"]
            != result[label]["frames"]
        ):
            raise _fail(
                f"report.denominators.{label}",
                "accepted + rejected must equal frames",
            )
    return result


class EvidenceCollector:
    """Select labelled samples and build a stable report."""

    def __init__(self, manifest: EvidenceManifest):
        self.manifest = manifest

    def collect(
        self,
        samples: Iterable[EvidenceSample],
        artifacts: Iterable[ArtifactRef] = (),
    ) -> EvidenceReport:
        normalized = sorted(samples, key=lambda sample: sample.timestamp_ns)
        timestamps = [sample.timestamp_ns for sample in normalized]
        if len(set(timestamps)) != len(timestamps):
            raise _fail(
                "samples.timestamp_ns",
                "must be unique; merge all topics into one sample",
            )

        artifact_list = sorted(artifacts, key=lambda artifact: artifact.artifact_id)
        artifact_by_id: dict[str, ArtifactRef] = {}
        for artifact in artifact_list:
            if artifact.artifact_id in artifact_by_id:
                raise _fail(
                    "artifacts", f"duplicate artifact_id {artifact.artifact_id!r}"
                )
            artifact_by_id[artifact.artifact_id] = artifact

        selected: list[tuple[EvidenceSample, tuple[IntervalLabel, ...]]] = []
        for sample in normalized:
            if (
                sample.accepted
                and sample.target_identity != self.manifest.target_identity
            ):
                raise _fail(
                    f"sample[{sample.timestamp_ns}].target_identity",
                    "accepted sample identity does not match manifest",
                )
            labels = labels_at(sample.timestamp_ns, self.manifest.intervals)
            for artifact_id in sample.artifact_ids:
                if artifact_id not in artifact_by_id:
                    raise _fail(
                        f"sample[{sample.timestamp_ns}].artifact_ids",
                        f"unknown artifact_id {artifact_id!r}",
                    )
            if labels:
                selected.append((sample, labels))

        denominators: dict[str, dict[str, int]] = {}
        for label in INTERVAL_LABELS:
            labelled = [sample for sample, labels in selected if label in labels]
            accepted = sum(sample.accepted for sample in labelled)
            denominators[label] = {
                "frames": len(labelled),
                "accepted": accepted,
                "rejected": len(labelled) - accepted,
            }

        frames = tuple(sample.to_dict(labels=labels) for sample, labels in selected)
        selected_timestamps = tuple(sample.timestamp_ns for sample, _labels in selected)
        accepted_count = sum(sample.accepted for sample, _labels in selected)
        alignment_count = sum(
            sample.alignment_dot is not None for sample, _labels in selected
        )
        pose_count = sum(
            sample.accepted and sample.pose is not None for sample, _labels in selected
        )
        covariance_count = sum(
            sample.accepted
            and sample.pose is not None
            and sample.pose.covariance is not None
            for sample, _labels in selected
        )
        quadrant_counts = {
            str(quadrant): sum(
                sample.quadrant == quadrant for sample, _labels in selected
            )
            for quadrant in range(4)
        }
        summary: dict[str, JsonValue] = {
            "input_frame_count": len(normalized),
            "selected_frame_count": len(selected),
            "unlabelled_frame_count": len(normalized) - len(selected),
            "accepted_frame_count": accepted_count,
            "rejected_frame_count": len(selected) - accepted_count,
            "accepted_pose_count": pose_count,
            "accepted_covariance_count": covariance_count,
            "alignment_dot_count": alignment_count,
            "quadrant_counts": quadrant_counts,
            "synthetic_test_only": self.manifest.provenance == "test_only",
        }
        return EvidenceReport(
            manifest_sha256=self.manifest.sha256(),
            bag_sha256=self.manifest.bag.sha256,
            target_identity=self.manifest.target_identity,
            sensor=self.manifest.sensor,
            preset=self.manifest.preset,
            provenance=self.manifest.provenance,
            selected_timestamps_ns=selected_timestamps,
            frames=frames,
            denominators=denominators,
            summary=summary,
            artifacts=tuple(artifact_list),
        )

    def collect_jsonl(
        self,
        path: str | Path,
        artifacts: Iterable[ArtifactRef] = (),
    ) -> EvidenceReport:
        """Collect normalized samples from deterministic JSON-lines input.

        This is a headless adapter contract, not a ROS bag reader.  Each non-empty
        line is one :class:`EvidenceSample` object.  It makes schema and selection
        tests reproducible while leaving ROS message/topic interpretation to the
        eventual bag-specific adapter.
        """
        try:
            lines = Path(path).read_text(encoding="utf-8").splitlines()
        except OSError as error:
            raise EvidenceSchemaError(
                f"samples: cannot read {path!s}: {error}"
            ) from error
        samples: list[EvidenceSample] = []
        for line_number, line in enumerate(lines, start=1):
            if not line.strip():
                continue
            try:
                value = json.loads(line)
            except json.JSONDecodeError as error:
                raise EvidenceSchemaError(
                    f"samples line {line_number}: invalid JSON: {error}"
                ) from error
            samples.append(
                EvidenceSample.from_mapping(
                    _mapping(value, f"samples line {line_number}"),
                    index=len(samples),
                )
            )
        return self.collect(samples, artifacts)


def labels_at(
    timestamp_ns: int, intervals: Iterable[EvidenceInterval]
) -> tuple[IntervalLabel, ...]:
    """Return labels in fixed semantic order using half-open interval semantics."""
    _nonnegative_int(timestamp_ns, "timestamp_ns")
    active = {
        interval.label
        for interval in intervals
        if interval.start_ns <= timestamp_ns < interval.end_ns
    }
    return tuple(label for label in INTERVAL_LABELS if label in active)  # type: ignore[return-value]


def _exact_keys(value: Mapping[str, object], expected: set[str], field: str) -> None:
    unknown = set(value) - expected
    missing = expected - set(value)
    if unknown:
        raise _fail(field, f"unknown field(s): {', '.join(sorted(unknown))}")
    if missing:
        raise _fail(field, f"missing field(s): {', '.join(sorted(missing))}")


def _mapping(value: object, field: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise _fail(field, "must be an object")
    return value


def _list(value: object, field: str) -> list[object]:
    if not isinstance(value, list):
        raise _fail(field, "must be an array")
    return value


def _numbers(value: object, field: str, *, expected: int) -> tuple[float, ...]:
    entries = _list(value, field)
    if len(entries) != expected:
        raise _fail(field, f"must contain exactly {expected} values")
    return tuple(
        _finite_float(item, f"{field}[{index}]") for index, item in enumerate(entries)
    )


def _relative_path(value: object, field: str) -> str:
    result = _string(value, field)
    path = Path(result)
    if path.is_absolute() or ".." in path.parts:
        raise _fail(field, "must be a relative path without '..'")
    return result


def _write_text(path: str | Path, text: str, kind: str) -> None:
    try:
        Path(path).write_text(text, encoding="utf-8")
    except OSError as error:
        raise EvidenceSchemaError(f"{kind}: cannot write {path!s}: {error}") from error


__all__ = [
    "INTERVAL_LABELS",
    "MANIFEST_SCHEMA_VERSION",
    "REPORT_SCHEMA_VERSION",
    "ArtifactRef",
    "ArucoObservation",
    "BagFingerprint",
    "EvidenceCollector",
    "EvidenceInterval",
    "EvidenceManifest",
    "EvidenceReport",
    "EvidenceSample",
    "EvidenceSchemaError",
    "PoseRecord",
    "RejectionReason",
    "TargetIdentityRecord",
    "canonical_bytes",
    "canonical_json",
    "labels_at",
    "sha256_file",
]
