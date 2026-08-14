"""Patch an Autoware ``sensor_kit_calibration.yaml`` with an LCTK-solved extrinsic.

Input is ``lidar_to_camera_solver``'s ``dump_detections`` JSON (version 4), whose
``transform`` holds the raw solver rvec/tvec (``T_optical<-lidar``). The re-labeled
TF topic is deliberately not an input — see M-01 and the Phase 6 design doc.
"""

import argparse
import json
import shutil
import sys
from pathlib import Path

import numpy as np
from ruamel.yaml import YAML

from .frames import entry_to_transform, kit_to_camera_link, transform_to_entry

DEFAULT_KIT_FRAME = "sensor_kit_base_link"


class ExportError(Exception):
    """Refuse-to-guess failure; message tells the operator what to fix."""


#: The dump format this exporter understands, and the board-frame convention its poses
#: must have been produced in. Kept as literals rather than imported from
#: `lidar_to_camera_solver` so this package stays independently installable; the pytest
#: suite runs both and would catch a divergence.
SUPPORTED_FORMAT_VERSION = 4
SUPPORTED_FRAME_CONVENTION = "corner_aligned_plate_center_v1"


def check_format_version(path, data):
    """H-11: refuse a dump whose format or frame convention this build cannot vouch for.

    This exporter writes into a `sensor_kit_calibration.yaml` that ends up on a vehicle,
    which makes it the single most important place for the check to exist. It had none:
    it read only `transform.rvec`/`transform.tvec`, and its own fixtures declared
    `"version": 2` and passed.
    """
    version = data.get("version", 0)
    if version != SUPPORTED_FORMAT_VERSION:
        raise ExportError(
            f"{path}: detection file version {version}, expected "
            f"{SUPPORTED_FORMAT_VERSION}. Versions below 4 record no board-frame "
            "convention, so their transform may be wrong by a silent 45-degree "
            "in-plane rotation (the 2x2 ArUco grid is symmetric, so the reprojection "
            "error stays low) plus a ~707 mm origin shift. Re-capture, or convert a "
            "file you still trust with: ros2 run lidar_to_camera_solver "
            "migrate_detections --help"
        )

    convention = data.get("board_frame_convention")
    if convention is None or convention.strip() != SUPPORTED_FRAME_CONVENTION:
        raise ExportError(
            f"{path}: board-frame convention {convention!r}, expected "
            f"'{SUPPORTED_FRAME_CONVENTION}'. The stored transform means something "
            "else; exporting it would put a wrong extrinsic on a vehicle."
        )


def load_solver_transform(path):
    """Read rvec/tvec from a dump_detections JSON file."""
    data = json.loads(Path(path).read_text())
    check_format_version(path, data)
    transform = data.get("transform")
    if not transform or "rvec" not in transform or "tvec" not in transform:
        raise ExportError(
            f"{path}: no solved transform found. Produce the file with the "
            "lidar_to_camera_solver's dump_detections service after a successful solve."
        )
    rvec = np.asarray(transform["rvec"], dtype=np.float64).reshape(3)
    tvec = np.asarray(transform["tvec"], dtype=np.float64).reshape(3)
    return rvec, tvec


def patch_calibration(
    target,
    *,
    rvec,
    tvec,
    camera_frame,
    lidar_frame,
    kit_frame=DEFAULT_KIT_FRAME,
    dry_run=False,
):
    """Replace ``[kit_frame][camera_frame]`` in the target YAML, preserving
    everything else (comments, order). Returns the written entry dict."""
    target = Path(target)
    yaml = YAML()  # round-trip mode: keeps comments and key order
    yaml.preserve_quotes = True
    doc = yaml.load(target.read_text())

    if kit_frame not in doc:
        raise ExportError(
            f"{target}: no '{kit_frame}' key. Top-level keys: {list(doc.keys())}"
        )
    kit = doc[kit_frame]
    if lidar_frame not in kit:
        raise ExportError(
            f"{target}: no '{lidar_frame}' entry under '{kit_frame}' to anchor the "
            f"chain. Available children: {list(kit.keys())}"
        )

    T_kit_lidar = entry_to_transform(dict(kit[lidar_frame]))
    entry = transform_to_entry(kit_to_camera_link(T_kit_lidar, rvec, tvec))

    if dry_run:
        return entry

    backup = target.with_suffix(target.suffix + ".bak")
    if not backup.exists():
        shutil.copy2(target, backup)

    if camera_frame in kit:
        for key, value in entry.items():
            kit[camera_frame][key] = value
    else:
        kit[camera_frame] = entry
    with target.open("w") as f:
        yaml.dump(doc, f)
    return entry


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Export an LCTK LiDAR-camera extrinsic into an Autoware "
        "sensor_kit_calibration.yaml (patches one entry, preserves the rest)."
    )
    parser.add_argument(
        "--detections",
        required=True,
        help="dump_detections JSON from lidar_to_camera_solver (source of rvec/tvec)",
    )
    parser.add_argument(
        "--target", required=True, help="sensor_kit_calibration.yaml to patch"
    )
    parser.add_argument(
        "--camera-frame",
        required=True,
        help="child key to write, e.g. camera0/camera_link",
    )
    parser.add_argument(
        "--lidar-frame",
        required=True,
        help="existing child entry used as the kit->lidar anchor, "
        "e.g. velodyne_top_base_link",
    )
    parser.add_argument("--kit-frame", default=DEFAULT_KIT_FRAME)
    parser.add_argument(
        "--dry-run", action="store_true", help="print the entry, write nothing"
    )
    args = parser.parse_args(argv)

    try:
        rvec, tvec = load_solver_transform(args.detections)
        entry = patch_calibration(
            args.target,
            rvec=rvec,
            tvec=tvec,
            camera_frame=args.camera_frame,
            lidar_frame=args.lidar_frame,
            kit_frame=args.kit_frame,
            dry_run=args.dry_run,
        )
    except ExportError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1

    action = "would write" if args.dry_run else "wrote"
    print(f"{action} {args.kit_frame} -> {args.camera_frame} in {args.target}:")
    for key in ("x", "y", "z", "roll", "pitch", "yaw"):
        print(f"  {key}: {entry[key]:.9f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
