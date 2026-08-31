"""`ros2 run lctk_launch lctk_session` -- list, check and scaffold sessions.

Everything here is available without `just`; the justfile only adds name lookup
and this repo's usual defaults.
"""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

import yaml

from lctk_launch.session import (
    MANIFEST_NAME,
    SessionError,
    bag_topics,
    parse_data,
    resolve_session,
)


def _check(spec: str) -> int:
    """Resolve everything a run needs and report it, without starting a graph."""
    try:
        session = resolve_session(spec)
        manifest = yaml.safe_load(session.manifest.read_text(encoding="utf-8"))
        source = parse_data(manifest.get("data"), session.directory)

        from lctk_launch.config_parser import parse_config

        pipeline = parse_config(str(session.manifest))
    except (SessionError, ValueError, OSError) as error:
        print(f"FAIL {spec}: {error}", file=sys.stderr)
        return 1

    print(f"session:  {session.directory}")
    print(f"manifest: {session.manifest}")
    print(f"data:     {source.kind} {source.directory or source.path or '(live)'}")
    for name, lidar in pipeline.lidars.items():
        print(f"  lidar  {name}: {lidar.pointcloud_topic}  frame={lidar.frame_id}")
    for name, camera in pipeline.cameras.items():
        print(f"  camera {name}: {camera.image_topic}  frame={camera.frame_id}")
    if source.kind == "bag":
        print(f"  bag records: {', '.join(bag_topics(source.path))}")
    print("OK")
    return 0


def _default_roots() -> list[Path]:
    """Where to look for sessions when no directory is given.

    The installed share directory is best-effort: this is a discovery helper, and
    an unbuilt or unsourced workspace is a reason to list fewer roots, not to
    fail. So the lookup swallows whatever ament raises rather than naming a
    growing list of import- and index-time failures.
    """
    roots = [Path.cwd() / "sessions"]
    try:
        from ament_index_python.packages import get_package_share_directory

        roots.append(Path(get_package_share_directory("lctk_launch")) / "sessions")
    except Exception:  # noqa: BLE001,S110 - see the docstring above
        pass
    return roots


def _list(directories: list[str]) -> int:
    roots = [Path(d) for d in directories] or _default_roots()
    for root in roots:
        if not root.is_dir():
            continue
        for child in sorted(root.iterdir()):
            if (child / MANIFEST_NAME).is_file():
                print(f"{child.name:40s} {child}")
    return 0


def _new(target: str, template: str) -> int:
    destination = Path(target).expanduser()
    if destination.exists():
        print(f"{destination} already exists", file=sys.stderr)
        return 1
    try:
        source = resolve_session(template)
    except SessionError as error:
        print(str(error), file=sys.stderr)
        return 1
    shutil.copytree(source.directory, destination, ignore=shutil.ignore_patterns("out"))
    (destination / "out").mkdir(exist_ok=True)
    print(f"created {destination} from {source.directory}")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="lctk_session")
    sub = parser.add_subparsers(dest="command", required=True)

    check = sub.add_parser("check", help="validate a session without launching it")
    check.add_argument("session")

    listing = sub.add_parser("list", help="list sessions in the given directories")
    listing.add_argument("directories", nargs="*")

    new = sub.add_parser("new", help="scaffold a session from an existing one")
    new.add_argument("target")
    new.add_argument("--from", dest="template", required=True)

    args = parser.parse_args(argv)
    if args.command == "check":
        return _check(args.session)
    if args.command == "list":
        return _list(args.directories)
    return _new(args.target, args.template)


if __name__ == "__main__":
    sys.exit(main())
