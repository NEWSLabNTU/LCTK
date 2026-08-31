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


DATA_KINDS = ("pcap_avi", "bag", "live")

_DATA_KEYS = {"kind", "dir", "path", "lidar", "camera"}
_LIDAR_KEYS = {"model", "rpm"}
_CAMERA_KEYS = {"info_url"}


@dataclass(frozen=True)
class DataSource:
    """Where a session's data comes from."""

    kind: str
    directory: Path | None = None  # pcap_avi
    path: Path | None = None  # bag
    lidar_model: str = "vlp32c"
    lidar_rpm: float = 600.0
    camera_info_url: str | None = None


def derived_lidar_topics(device: str) -> dict[str, str]:
    """Topic names LCTK publishes for a driven lidar playback.

    These reproduce `lidar_camera.launch.xml`'s existing defaults, so the
    convention describes behaviour that already exists rather than inventing one.
    """
    return {
        "pointcloud": f"/sensing/lidar/{device}/pointcloud_raw",
        "packets": f"/sensing/lidar/{device}/velodyne_packets",
    }


def derived_camera_topics(device: str) -> dict[str, str]:
    namespace = f"/sensing/camera/{device}"
    return {
        "namespace": namespace,
        "image": f"{namespace}/image_raw",
        "camera_info": f"{namespace}/camera_info",
    }


def _reject_unknown(section: str, raw: dict, known: set[str]) -> None:
    unknown = set(raw) - known
    if unknown:
        raise SessionError(
            f"unknown key(s) in '{section}': {', '.join(sorted(unknown))}. "
            f"Known keys: {', '.join(sorted(known))}"
        )


def parse_data(raw: object, session_dir: Path) -> DataSource:
    """Validate the manifest's `data:` section into a DataSource.

    Refusals here are deliberate. Every failure this design exists to prevent is
    silent at run time -- a wrong topic yields a healthy graph and no data, a
    missing recording yields a detector that never fires -- so they are converted
    into loud refusals before the graph starts.
    """
    if raw is None:
        raise SessionError(
            "missing required 'data' section. A session must say where its data "
            f"comes from: one of {', '.join(DATA_KINDS)}."
        )
    if not isinstance(raw, dict):
        raise SessionError(f"'data' must be a mapping, got {type(raw).__name__}")
    _reject_unknown("data", raw, _DATA_KEYS)

    kind = raw.get("kind")
    if kind not in DATA_KINDS:
        raise SessionError(
            f"unknown data kind {kind!r}; expected one of {', '.join(DATA_KINDS)}"
        )

    lidar = raw.get("lidar") or {}
    _reject_unknown("data.lidar", lidar, _LIDAR_KEYS)
    camera = raw.get("camera") or {}
    _reject_unknown("data.camera", camera, _CAMERA_KEYS)

    info_url = camera.get("info_url")
    if info_url is not None:
        info_url = resolve_config_path(str(info_url), session_dir)

    common = {
        "lidar_model": str(lidar.get("model", "vlp32c")),
        "lidar_rpm": float(lidar.get("rpm", 600.0)),
        "camera_info_url": info_url,
    }

    if kind == "live":
        return DataSource(kind=kind, **common)

    if kind == "pcap_avi":
        if "dir" not in raw:
            raise SessionError("data.kind 'pcap_avi' requires 'dir'")
        directory = Path(resolve_config_path(str(raw["dir"]), session_dir))
        if not directory.is_dir():
            raise SessionError(f"data.dir does not exist: {directory}")
        for required in ("lidar.pcap", "video.avi"):
            if not (directory / required).is_file():
                raise SessionError(f"{directory} has no {required}")
        return DataSource(kind=kind, directory=directory, **common)

    if "path" not in raw:
        raise SessionError("data.kind 'bag' requires 'path'")
    bag = Path(resolve_config_path(str(raw["path"]), session_dir))
    if not bag.is_dir():
        raise SessionError(f"data.path does not exist: {bag}")
    if not (bag / "metadata.yaml").is_file():
        raise SessionError(f"{bag} has no metadata.yaml; is it a rosbag2 directory?")
    return DataSource(kind=kind, path=bag, **common)


def bag_topics(bag: Path) -> list[str]:
    """Every topic name a rosbag2 directory records."""
    import yaml

    metadata = yaml.safe_load((bag / "metadata.yaml").read_text(encoding="utf-8"))
    information = metadata["rosbag2_bagfile_information"]
    return [
        entry["topic_metadata"]["name"]
        for entry in information["topics_with_message_count"]
    ]


def verify_bag_topics(bag: Path, wanted: list[str]) -> None:
    """Refuse a manifest naming a topic the bag does not contain.

    This is M-26 caught at startup: `two_lidar.yaml` named `/velodyne_points`
    while the recording publishes `/lidar/vlp32/velodyne_points`, and the result
    was a pipeline that launched cleanly and sat silent forever. Listing what the
    bag does have turns the error into its own fix.
    """
    available = bag_topics(bag)
    missing = [topic for topic in wanted if topic not in available]
    if missing:
        raise SessionError(
            f"{bag} does not publish {', '.join(missing)}. "
            f"It records: {', '.join(available)}"
        )
