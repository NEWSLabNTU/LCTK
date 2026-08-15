"""Convert a version-3 detection dump to version 4.

`lidar_to_camera_solver` refuses to load a version-3 file, because version 3 records no
board-frame convention: a file captured before the corner-aligned board frame and one
captured after are byte-indistinguishable, and reinterpreting one on load would make its
meaning depend on which build opened it.

This is the explicit way out, for a calibration you still trust. You must name the
convention the file was captured in — the tool cannot know it, and guessing is the
failure this whole change exists to remove.

    ros2 run lidar_to_camera_solver migrate_detections \\
        --input ~/detections.json --output ~/detections-v4.json \\
        --assume-convention corner_aligned_plate_center_v1

If the file predates the corner-aligned frame, do not convert it: its board poses mean
something else, and no field added to the file changes that. Re-capture.
"""

import argparse
import json
import sys
from pathlib import Path

from lidar_to_camera_solver.board_geometry import BOARD_FRAME_CONVENTION
from lidar_to_camera_solver.detection_format import (
    format_version_error,
    migrate_v3_to_v4,
)


def main(argv=None):
    parser = argparse.ArgumentParser(
        prog="migrate_detections",
        description="Convert a version-3 detection dump to version 4.",
    )
    parser.add_argument("--input", required=True, help="version-3 dump JSON to read")
    parser.add_argument("--output", required=True, help="version-4 JSON to write")
    parser.add_argument(
        "--assume-convention",
        required=True,
        help=(
            "the board-frame convention the file was CAPTURED in. This build works in "
            f"'{BOARD_FRAME_CONVENTION}'. Naming anything else is honest but produces a "
            "file this build will still refuse, which is the correct outcome."
        ),
    )
    args = parser.parse_args(argv)

    data = json.loads(Path(args.input).read_text())

    try:
        migrated = migrate_v3_to_v4(data, convention=args.assume_convention)
    except ValueError as error:
        print(f"{args.input}: {error}", file=sys.stderr)
        return 1

    Path(args.output).write_text(json.dumps(migrated, indent=2))

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


if __name__ == "__main__":
    sys.exit(main())
