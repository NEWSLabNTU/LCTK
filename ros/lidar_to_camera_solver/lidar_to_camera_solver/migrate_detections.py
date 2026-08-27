"""Migrate a saved detection archive to the next explicit version.

Two hops exist, and each asserts a different operator fact:

- version 3 to 4 names the board-frame convention the file was CAPTURED in:

      ros2 run lidar_to_camera_solver migrate_detections \\
          --input detections-v3.json --output detections-v4.json \\
          --assume-convention corner_aligned_plate_center_v1

  If the file predates the corner-aligned frame, do not convert it: its board
  poses mean something else, and no field added to the file changes that.
  Re-capture instead.

- version 4 to 5 binds an explicit Target Definition to the archive:

      ros2 run lidar_to_camera_solver migrate_detections \\
          --input detections-v4.json --output detections-v5.json \\
          --target-config hollow_1000_aruco_4_v1.json5

  This checks that every marker ID the archive actually observed belongs to the
  selected target, which catches an obviously wrong selection, but it is not
  and cannot be proof of physical provenance -- which target produced this
  recording remains an operator claim this command cannot verify.

Version 3 is never migrated straight to version 5 in one invocation. Reaching
version 5 from a version-3 file always takes two explicit runs: first the
frame-convention hop above, then the target-binding hop on its version-4
output. Version 4 is never reinterpreted implicitly, so collapsing the two
hops would let one operator claim silently stand in for the other.
"""

import argparse
import json
import os
import sys
import tempfile
from pathlib import Path

from lctk_target import load_target

from lidar_to_camera_solver.archive_contract import ARCHIVE_V4, ARCHIVE_V5
from lidar_to_camera_solver.board_geometry import BOARD_FRAME_CONVENTION
from lidar_to_camera_solver.detection_format import (
    format_version_error,
    migrate_v3_to_v4,
    migrate_v4_to_v5,
)

_ARCHIVE_V3 = 3


def _detected_version(data: object) -> int | None:
    """Return the archive's literal integer ``version``, or ``None`` if absent."""
    if not isinstance(data, dict):
        return None
    version = data.get("version")
    if isinstance(version, bool) or not isinstance(version, int):
        return None
    return version


def _atomic_write_json(destination: Path, data: dict) -> None:
    """Write ``data`` to ``destination`` atomically, or raise ``OSError``.

    Mirrors ``LidarToCameraSolver.dump_detections_callback`` in ``main.py``,
    which fixed the identical class of bug on the save-detections path: write to
    a sibling temp file in the destination's own directory, then ``os.replace``
    it into place, so an unwritable destination or an interrupted write can
    never leave a truncated file where a migrated archive is expected.
    """
    descriptor, temp_name = tempfile.mkstemp(
        dir=str(destination.parent),
        prefix=f".{destination.name}.",
        suffix=".tmp",
    )
    temp_path: Path | None = Path(temp_name)
    try:
        with os.fdopen(descriptor, "w") as file:
            json.dump(data, file, indent=2)
        os.replace(temp_path, destination)
        temp_path = None
    finally:
        if temp_path is not None:
            try:
                temp_path.unlink()
            except OSError as cleanup_error:
                print(
                    f"warning: failed to remove temporary file {temp_path}: "
                    f"{cleanup_error}",
                    file=sys.stderr,
                )


def _migrate_v3(args: argparse.Namespace, data: dict) -> int:
    try:
        migrated = migrate_v3_to_v4(data, convention=args.assume_convention)
    except ValueError as error:
        print(f"{args.input}: {error}", file=sys.stderr)
        return 1

    try:
        _atomic_write_json(Path(args.output), migrated)
    except OSError as error:
        print(f"{args.output}: failed to write: {error}", file=sys.stderr)
        return 1

    remaining = format_version_error(migrated)
    if remaining is not None:
        print(
            f"Wrote {args.output}, but it is still not loadable: {remaining}",
            file=sys.stderr,
        )
        return 1

    print(
        f"Wrote {args.output} (version 4, board_frame_convention="
        f"'{args.assume_convention}').\n"
        "Board-pose covariances are not recoverable from a version-3 file; they stay "
        "all-zero, which the solver reads as unknown rather than exact."
    )
    return 0


def _migrate_v4(args: argparse.Namespace, data: dict) -> int:
    try:
        target = load_target(args.target_config)
    except ValueError as error:
        print(f"{args.target_config}: {error}", file=sys.stderr)
        return 1

    try:
        migrated = migrate_v4_to_v5(data, target=target)
    except ValueError as error:
        print(f"{args.input}: {error}", file=sys.stderr)
        return 1

    try:
        _atomic_write_json(Path(args.output), migrated)
    except OSError as error:
        print(f"{args.output}: failed to write: {error}", file=sys.stderr)
        return 1

    print(
        f"Wrote {args.output} (version 5, target_id="
        f"'{target.target_id}'@{target.revision}).\n"
        "This binding is operator-asserted: every marker ID the archive observed "
        "was checked against the selected target, but that only proves ID "
        "compatibility. It does not and cannot prove which physical target "
        "produced this recording -- that remains your confirmation."
    )
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(
        prog="migrate_detections",
        description=(
            "Migrate a saved detection archive one explicit version forward: "
            "version 3 to 4, or version 4 to 5."
        ),
    )
    parser.add_argument("--input", required=True, help="detection dump JSON to read")
    parser.add_argument("--output", required=True, help="migrated JSON to write")
    parser.add_argument(
        "--assume-convention",
        default=None,
        help=(
            "version 3 to 4 only: the board-frame convention the file was CAPTURED "
            f"in. This build works in '{BOARD_FRAME_CONVENTION}'. Naming anything "
            "else is honest but produces a file this build will still refuse, which "
            "is the correct outcome."
        ),
    )
    parser.add_argument(
        "--target-config",
        default=None,
        help=(
            "version 4 to 5 only: the Target Definition JSON5 the archive's "
            "detections are ASSERTED to have been captured against. Marker IDs "
            "observed in the archive are checked against this target before its "
            "identity is bound; this is not and cannot be proof of physical "
            "provenance."
        ),
    )
    args = parser.parse_args(argv)

    try:
        data = json.loads(Path(args.input).read_text())
    except OSError as error:
        print(f"{args.input}: {error}", file=sys.stderr)
        return 1
    except json.JSONDecodeError as error:
        print(f"{args.input}: not valid JSON: {error}", file=sys.stderr)
        return 1

    if args.assume_convention is not None and args.target_config is not None:
        print(
            f"{args.input}: supply exactly one of --assume-convention (for a "
            "version 3 input) or --target-config (for a version 4 input) per "
            "invocation. Migrating version 3 straight to version 5 in one command "
            "is not supported -- each hop asserts a different operator fact. Run "
            "the version 3 to 4 step first, then run this command again with "
            "--target-config on that step's output.",
            file=sys.stderr,
        )
        return 1

    version = _detected_version(data)

    if version == ARCHIVE_V5:
        print(
            f"{args.input}: already version {ARCHIVE_V5}; there is nothing to migrate.",
            file=sys.stderr,
        )
        return 1

    if version == ARCHIVE_V4:
        if args.assume_convention is not None:
            print(
                f"{args.input}: is already version 4; --assume-convention only "
                "applies to a version 3 input. Supply --target-config to migrate "
                "this file to version 5.",
                file=sys.stderr,
            )
            return 1
        if args.target_config is None:
            print(
                f"{args.input}: is version 4; migrating to version 5 requires "
                "--target-config.",
                file=sys.stderr,
            )
            return 1
        return _migrate_v4(args, data)

    if version == _ARCHIVE_V3:
        if args.target_config is not None:
            print(
                f"{args.input}: is version 3, not version 4; --target-config only "
                "applies once a file is already version 4. Run this command with "
                "--assume-convention first to reach version 4, then run it again "
                "with --target-config to reach version 5.",
                file=sys.stderr,
            )
            return 1
        if args.assume_convention is None:
            print(
                f"{args.input}: is version 3; migrating to version 4 requires "
                "--assume-convention.",
                file=sys.stderr,
            )
            return 1
        return _migrate_v3(args, data)

    raw_version = data.get("version") if isinstance(data, dict) else None
    print(
        f"{args.input}: unsupported detection file version {raw_version!r}; this "
        "command migrates version 3 to 4 or version 4 to 5 only.",
        file=sys.stderr,
    )
    return 1


if __name__ == "__main__":
    sys.exit(main())
