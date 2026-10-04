"""Patch an Autoware ``sensor_kit_calibration.yaml`` with an LCTK-solved extrinsic.

Input is ``lidar_to_camera_solver``'s ``dump_detections`` JSON (version 6), whose
``transform`` holds the raw solver rvec/tvec (``T_optical<-lidar``). The re-labeled
TF topic is deliberately not an input — see M-01 and the Phase 6 design doc.
"""

import argparse
import json
import shutil
import sys
from pathlib import Path

import numpy as np
from lctk_quality.projection_metadata import (
    archive_frames,
    normalize_camera_projection,
)
from ruamel.yaml import YAML

from .archive_contract import archive_export_error
from .frames import entry_to_transform, kit_to_camera_link, transform_to_entry

DEFAULT_KIT_FRAME = "sensor_kit_base_link"


class ExportError(Exception):
    """Refuse-to-guess failure; message tells the operator what to fix."""


#: The dump format this exporter understands, and the board-frame convention its poses
#: must have been produced in. The identity check is structural so this package stays
#: independently installable without loading a target file.
SUPPORTED_FORMAT_VERSION = 6
SUPPORTED_FRAME_CONVENTION = "corner_aligned_plate_center_v1"


def check_format_version(path, data):
    """Refuse archives whose format, projection, or frames cannot be validated."""
    error = archive_export_error(data, expected_frame=SUPPORTED_FRAME_CONVENTION)
    if error is not None:
        raise ExportError(f"{path}: {error}")
    try:
        projection = normalize_camera_projection(data.get("camera_projection"))
        archive_frames(data, projection)
    except ValueError as error:
        raise ExportError(f"{path}: {error}") from error


def load_solver_transform(path):
    """Read rvec/tvec from a dump_detections JSON file."""
    data = json.loads(Path(path).read_text())
    check_format_version(path, data)
    transform = data.get("transform")
    if (
        not isinstance(transform, dict)
        or "rvec" not in transform
        or "tvec" not in transform
    ):
        raise ExportError(
            f"{path}: no solved transform found. Produce the file with the "
            "lidar_to_camera_solver's dump_detections service after a successful solve."
        )
    rvec = _load_transform_vector(path, transform["rvec"], "rvec")
    tvec = _load_transform_vector(path, transform["tvec"], "tvec")
    return rvec, tvec


def _load_transform_vector(path, value, name):
    """Parse one saved 3-vector without accepting coercions or reshaping."""
    if (
        not isinstance(value, list)
        or len(value) != 3
        or any(
            isinstance(component, bool) or not isinstance(component, (int, float))
            for component in value
        )
    ):
        raise ExportError(
            f"{path}: transform.{name} must be a three-element numeric vector"
        )
    try:
        vector = np.asarray(value, dtype=np.float64)
    except (OverflowError, TypeError, ValueError) as error:
        raise ExportError(
            f"{path}: transform.{name} must contain representable numeric values"
        ) from error
    if not np.all(np.isfinite(vector)):
        raise ExportError(f"{path}: transform.{name} must contain only finite values")
    return vector


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
