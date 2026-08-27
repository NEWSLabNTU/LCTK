"""Pure compatibility rules for solved detection archives.

This module deliberately does not load a Target Definition or import ROS.  The
caller supplies its already-validated local Target Identity when it wants to
restore an archive.  Keeping that comparison here makes the v4/v5 boundary
testable without changing the live solver restore path before its migration
packet lands.
"""

from __future__ import annotations

import re
from collections.abc import Mapping
from typing import Any

ARCHIVE_V4 = 4
ARCHIVE_V5 = 5
MIGRATION_COMMAND = "ros2 run lidar_to_camera_solver migrate_detections"
_IDENTITY_FIELDS = (
    "schema_version",
    "target_id",
    "revision",
    "semantic_sha256",
    "board_frame_convention",
)
_SHA256 = re.compile(r"[0-9a-f]{64}\Z")


def target_identity_error(
    identity: object, *, label: str = "target_identity"
) -> str | None:
    """Return a structural error for a wire Target Identity, else ``None``.

    ``identity`` may be a JSON object or the immutable value supplied by the
    shared target package.  Values are checked structurally here; resolving a
    hash to a target definition is intentionally not this seam's job.
    """
    values = _identity_values(identity)
    if values is None:
        return f"{label} must contain exactly {', '.join(_IDENTITY_FIELDS)}"
    schema_version, target_id, revision, digest, frame = (
        values[field] for field in _IDENTITY_FIELDS
    )
    if not _positive_int(schema_version):
        return f"{label}.schema_version must be a non-zero integer"
    if not isinstance(target_id, str) or not target_id:
        return f"{label}.target_id must be a non-empty string"
    if not _positive_int(revision):
        return f"{label}.revision must be a non-zero integer"
    if not isinstance(digest, str) or _SHA256.fullmatch(digest) is None:
        return f"{label}.semantic_sha256 must be 64 lowercase hexadecimal characters"
    if not isinstance(frame, str) or not frame:
        return f"{label}.board_frame_convention must be a non-empty string"
    return None


def archive_restore_error(data: object, local_identity: object) -> str | None:
    """Return why an archive cannot be restored against ``local_identity``.

    Version 4 has sufficient solved-transform provenance for export, but lacks
    Target Identity and is never restorable after target selection is required.
    Version 5 is restorable only when every identity field exactly matches the
    local validated target.
    """
    if not isinstance(data, Mapping):
        return "Detection archive must be an object"
    version = data.get("version")
    if not isinstance(version, int) or isinstance(version, bool):
        return (
            f"Detection archive version {version!r} is not restorable; "
            "expected integer 5"
        )
    if version == ARCHIVE_V4:
        return (
            "Detection archive version 4 has no Target Identity and cannot be "
            "restored. Explicitly migrate it with: "
            f"{MIGRATION_COMMAND} --input <file> --output <file> "
            "--target-config <target-config>"
        )
    if version < ARCHIVE_V5:
        return (
            f"Detection archive version {version!r} is an unsupported past "
            "version; expected integer 5"
        )
    if version > ARCHIVE_V5:
        return (
            f"Detection archive version {version!r} is an unsupported future "
            "version; expected integer 5"
        )

    archived_identity = data.get("target_identity")
    error = target_identity_error(archived_identity)
    if error is not None:
        return error
    error = target_identity_error(local_identity, label="local target identity")
    if error is not None:
        return error
    archived_values = _identity_values(archived_identity)
    local_values = _identity_values(local_identity)
    convention = data.get("board_frame_convention")
    if not isinstance(convention, str) or not convention:
        return "Detection archive board_frame_convention must be a non-empty string"
    if convention != archived_values["board_frame_convention"]:
        return (
            "Detection archive board_frame_convention conflicts with its "
            "Target Identity"
        )
    if convention != local_values["board_frame_convention"]:
        return (
            "Detection archive board_frame_convention does not match the local target"
        )
    if archived_values != local_values:
        return (
            "Detection archive Target Identity does not exactly match the local target"
        )
    return None


def _identity_values(identity: object) -> dict[str, Any] | None:
    if isinstance(identity, Mapping):
        if set(identity) != set(_IDENTITY_FIELDS):
            return None
        return {field: identity[field] for field in _IDENTITY_FIELDS}
    if all(hasattr(identity, field) for field in _IDENTITY_FIELDS):
        return {field: getattr(identity, field) for field in _IDENTITY_FIELDS}
    return None


def _positive_int(value: object) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and value > 0


__all__ = [
    "ARCHIVE_V4",
    "ARCHIVE_V5",
    "MIGRATION_COMMAND",
    "archive_restore_error",
    "target_identity_error",
]
