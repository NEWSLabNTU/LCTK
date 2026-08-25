"""Pure archive checks used before an Autoware transform export.

The exporter needs only a solved transform's provenance, not a local Target
Definition.  It therefore validates a v5 identity structurally but deliberately
does not attempt to load or compare a target manifest.
"""

from __future__ import annotations

import re
from collections.abc import Mapping

ARCHIVE_V4 = 4
ARCHIVE_V5 = 5
_IDENTITY_FIELDS = {
    "schema_version",
    "target_id",
    "revision",
    "semantic_sha256",
    "board_frame_convention",
}
_SHA256 = re.compile(r"[0-9a-f]{64}\Z")


def target_identity_error(identity: object) -> str | None:
    """Return a structural identity error, without resolving a target file."""
    if not isinstance(identity, Mapping) or set(identity) != _IDENTITY_FIELDS:
        return "target_identity must contain exactly the five Target Identity fields"
    if not _positive_int(identity["schema_version"]):
        return "target_identity.schema_version must be a non-zero integer"
    if not isinstance(identity["target_id"], str) or not identity["target_id"]:
        return "target_identity.target_id must be a non-empty string"
    if not _positive_int(identity["revision"]):
        return "target_identity.revision must be a non-zero integer"
    digest = identity["semantic_sha256"]
    if not isinstance(digest, str) or _SHA256.fullmatch(digest) is None:
        return "target_identity.semantic_sha256 must be 64 lowercase hexadecimal characters"
    frame = identity["board_frame_convention"]
    if not isinstance(frame, str) or not frame:
        return "target_identity.board_frame_convention must be a non-empty string"
    return None


def archive_export_error(data: object, *, expected_frame: str) -> str | None:
    """Return why an archive is unsafe to export, or ``None``.

    Both v4 and v5 solved archives remain exportable.  Only v5 carries a Target
    Identity, and validating it here is intentionally structural: this package
    must remain usable without a target-manifest installation.
    """
    if not isinstance(data, Mapping):
        return "detection archive must be an object"
    version = data.get("version")
    if (
        not isinstance(version, int)
        or isinstance(version, bool)
        or version
        not in (
            ARCHIVE_V4,
            ARCHIVE_V5,
        )
    ):
        return f"detection file version {version!r}, expected 4 or 5"
    convention = data.get("board_frame_convention")
    if not isinstance(convention, str) or convention != expected_frame:
        return (
            f"board-frame convention {convention!r}, expected {expected_frame!r}. "
            "The stored transform means something else; exporting it would put a "
            "wrong extrinsic on a vehicle."
        )
    if version == ARCHIVE_V5:
        identity = data.get("target_identity")
        error = target_identity_error(identity)
        if error is not None:
            return error
        if identity["board_frame_convention"] != convention:
            return (
                "target_identity.board_frame_convention conflicts with the "
                "detection archive board_frame_convention"
            )
    return None


def _positive_int(value: object) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and value > 0


__all__ = ["ARCHIVE_V4", "ARCHIVE_V5", "archive_export_error", "target_identity_error"]
