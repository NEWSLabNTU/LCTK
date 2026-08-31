"""What a calibration session is, and how to find one.

A session is one directory describing one run: where the data comes from, and
everything needed to calibrate against it. It is self-contained and relocatable,
so it can live inside this repo or in an operator's own tree.

This module imports no ROS and no `launch` types, so the rules below -- path
resolution, data-source validation, topic derivation and bag verification -- are
testable without a graph. The launch files are thin wrappers over it.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path

MANIFEST_NAME = "session.yaml"

_FIND_PKG_SHARE = re.compile(r"\$\(find-pkg-share\s+([^)]+)\)")
_SESSION_DIR = re.compile(r"\$\(session-dir\)")


class SessionError(Exception):
    """A session could not be resolved, read, or validated."""


@dataclass(frozen=True)
class SessionPaths:
    """Where a session's manifest is, and the directory it anchors."""

    manifest: Path
    directory: Path


def resolve_session(spec: str) -> SessionPaths:
    """Resolve an explicit `session:=` path to its manifest and directory.

    `spec` is always a path -- absolute, or relative to the working directory.
    There is deliberately no search path: an implicit location would assume both
    where sessions live and where the user is standing, and a session may live
    anywhere. Name lookup, where it is wanted, belongs in the justfile.

    Accepts either the session directory or the manifest file itself.
    """
    candidate = Path(spec).expanduser()
    if not candidate.exists():
        raise SessionError(
            f"no session at '{candidate}'. `session:=` takes an explicit path to a "
            f"session directory or to a {MANIFEST_NAME}; there is no search path."
        )
    candidate = candidate.resolve()
    if candidate.is_dir():
        manifest = candidate / MANIFEST_NAME
        if not manifest.is_file():
            raise SessionError(
                f"'{candidate}' is a directory but contains no {MANIFEST_NAME}"
            )
        return SessionPaths(manifest=manifest, directory=candidate)
    return SessionPaths(manifest=candidate, directory=candidate.parent)


def resolve_config_path(path: str, session_dir: Path | None = None) -> str:
    """Expand `$(find-pkg-share pkg)` and `$(session-dir)` in a config path.

    `$(session-dir)` is what makes a session relocatable: a manifest referring to
    its own directory carries no absolute path into LCTK, so the directory can be
    copied to another machine and still run.
    """

    def replace_package(match: re.Match) -> str:
        package_name = match.group(1).strip()
        from ament_index_python.packages import (
            PackageNotFoundError,
            get_package_share_directory,
        )

        try:
            return get_package_share_directory(package_name)
        except (PackageNotFoundError, ValueError) as error:
            raise ValueError(
                f"Failed to find package '{package_name}': {error}"
            ) from error

    if _SESSION_DIR.search(path):
        if session_dir is None:
            raise SessionError(
                f"'{path}' uses $(session-dir), but this file was not loaded from a "
                "session directory. Use an absolute path or $(find-pkg-share ...) "
                "here."
            )
        path = _SESSION_DIR.sub(str(session_dir), path)
    return _FIND_PKG_SHARE.sub(replace_package, path)
