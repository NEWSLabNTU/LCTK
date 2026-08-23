"""The saved-detection file format: serialization, version 4, and its version check.

A dump file stores board *poses*, and Phase 1 changed what a board pose means. Version
3 recorded no convention — board-local marker corners are recomputed at load time from
``aruco_pattern.json5`` — so files written before and after that change are
indistinguishable, and either reloads under whatever convention the loading build
believes in.

Version 4 fixes two things:

- it records the frame convention that produced it, using the same identifier the
  detector publishes, so there is one vocabulary rather than two;
- it stores the board pose's 6x6 covariance, which version 3 dropped. Without it a
  reloaded buffer solves with uniform weight 1.0 and quietly differs from the live
  buffer it was saved from (M-13).

Version 3 files are **rejected**, not migrated on load. Automatic migration would make
a file's meaning depend on which build opened it — the same class of silent difference
this phase exists to remove. `migrate_v3_to_v4` is the explicit way out, and it makes
the operator name the convention they are asserting.

Like `board_geometry`, this module imports nothing from ``rclpy``: the whole format is
functions over plain values.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from lidar_to_camera_solver.board_geometry import BOARD_FRAME_CONVENTION

#: Bumped from 3 in the same change that altered what a stored pose means, so version
#: and meaning stay in step.
FORMAT_VERSION = 4

#: The one-shot conversion command named in the rejection message.
MIGRATION_COMMAND = "ros2 run lidar_to_camera_solver migrate_detections"


@dataclass(frozen=True)
class ArchivedQuality:
    """Operator-facing quality verdict stored beside a calibration."""

    status: str
    is_degenerate: bool
    n_frames: int
    n_distinct_placements: int
    reprojection_rms_px: float
    reprojection_max_px: float
    per_pose_rms_px: tuple[float, ...]
    cond_jtj: float
    normal_span_deg: float
    depth_range_m: float
    lateral_span_m: float
    uncertainty_rot_deg: float | None
    uncertainty_trans_mm: float | None
    uncertainty_n_subsets: int | None
    warnings: tuple[str, ...]


@dataclass(frozen=True)
class AdjustedTransform:
    """Node-owned adjusted transform restored only against a current solve."""

    rvec: np.ndarray
    tvec: np.ndarray


@dataclass(frozen=True)
class DetectionArchive:
    pairs: tuple[object, ...]
    quality: ArchivedQuality | None
    adjusted_transform: AdjustedTransform | None


def format_version_error(data: dict) -> str | None:
    """Decide whether a loaded dump file may be used by this build.

    Returns ``None`` when it may, or an operator-facing failure message when it may
    not. Pure over the parsed JSON, so the whole decision table is testable without a
    ROS graph. The version says how the file is laid out; the convention tag says what
    the poses in it mean — both must agree with this build.
    """
    version = data.get("version", 0)

    if version != FORMAT_VERSION:
        detail = ""
        if version == 3:
            detail = (
                " Version 3 predates the corner-aligned board frame, so its poses may "
                "be wrong by a silent 45-degree in-plane rotation and a ~707 mm origin "
                "shift."
            )
        elif version in (1, 2):
            detail = (
                f" Version {version} also carries no real ArUco corners, only the "
                "axis-aligned bounding box (C-01)."
            )
        return (
            f"Unsupported detection file version {version}; this build reads version "
            f"{FORMAT_VERSION}.{detail} Convert a file you still trust with: "
            f"{MIGRATION_COMMAND} --input <file> --output <file> "
            f"--assume-convention {BOARD_FRAME_CONVENTION}"
        )

    convention = data.get("board_frame_convention")
    if convention is None:
        return (
            f"Detection file declares version {FORMAT_VERSION} but carries no "
            f"'board_frame_convention'. Expected '{BOARD_FRAME_CONVENTION}'."
        )

    if convention.strip() != BOARD_FRAME_CONVENTION:
        return (
            f"Detection file was produced in board-frame convention "
            f"'{convention}', but this build works in '{BOARD_FRAME_CONVENTION}'. "
            f"The stored poses mean something else; re-capture rather than reuse them."
        )

    return None


def migrate_v3_to_v4(data: dict, *, convention: str) -> dict:
    """Convert a parsed version-3 dump to version 4.

    The operator names the convention rather than the tool assuming one: converting is
    a claim about where the file came from, and only the person who captured it knows.
    The board-pose covariances version 3 discarded cannot be recovered; they stay
    all-zero, which every reader already treats as "unknown", NOT as "exact".
    """
    version = data.get("version", 0)
    if version != 3:
        raise ValueError(
            f"migrate_v3_to_v4 expects a version 3 file, got version {version}"
        )

    migrated = dict(data)
    # This is intentionally literal.  v3 data has the v4 layout after migration;
    # a later current format must not relabel it as carrying fields it never gained.
    migrated["version"] = 4
    migrated["board_frame_convention"] = convention
    return migrated


def serialize_detection2d_array(msg) -> dict:
    """Serialize a ``Detection2DArray`` to a JSON-compatible dict."""
    return {
        "header": {
            "stamp": {
                "sec": msg.header.stamp.sec,
                "nanosec": msg.header.stamp.nanosec,
            },
            "frame_id": msg.header.frame_id,
        },
        "detections": [
            {
                "id": d.id,
                "bbox": {
                    "center": {
                        "x": d.bbox.center.position.x,
                        "y": d.bbox.center.position.y,
                    },
                    "size_x": d.bbox.size_x,
                    "size_y": d.bbox.size_y,
                },
                # H-10: the real ArUco corner pixels, one per result. Without them a
                # reload falls back to the axis-aligned bbox and reintroduces C-01.
                "results": [
                    {
                        "class_id": r.hypothesis.class_id,
                        "score": r.hypothesis.score,
                        "position": {
                            "x": r.pose.pose.position.x,
                            "y": r.pose.pose.position.y,
                            "z": r.pose.pose.position.z,
                        },
                    }
                    for r in d.results
                ],
            }
            for d in msg.detections
        ],
    }


def serialize_detection3d_array(msg) -> dict:
    """Serialize a ``Detection3DArray`` to a JSON-compatible dict.

    Version 4 keeps the 6x6 pose covariance, which the version-3 serializer dropped.
    """
    return {
        "header": {
            "stamp": {
                "sec": msg.header.stamp.sec,
                "nanosec": msg.header.stamp.nanosec,
            },
            "frame_id": msg.header.frame_id,
        },
        "detections": [
            {
                "results": [
                    {
                        "pose": {
                            "position": {
                                "x": r.pose.pose.position.x,
                                "y": r.pose.pose.position.y,
                                "z": r.pose.pose.position.z,
                            },
                            "orientation": {
                                "x": r.pose.pose.orientation.x,
                                "y": r.pose.pose.orientation.y,
                                "z": r.pose.pose.orientation.z,
                                "w": r.pose.pose.orientation.w,
                            },
                        },
                        "covariance": [float(v) for v in r.pose.covariance],
                    }
                    for r in d.results
                ]
            }
            for d in msg.detections
        ],
    }


def deserialize_detection2d_array(data: dict):
    """Rebuild a ``Detection2DArray`` from `serialize_detection2d_array` output."""
    from geometry_msgs.msg import Pose, PoseWithCovariance
    from vision_msgs.msg import (
        BoundingBox2D,
        Detection2D,
        Detection2DArray,
        ObjectHypothesisWithPose,
    )

    msg = Detection2DArray()
    msg.header.stamp.sec = data["header"]["stamp"]["sec"]
    msg.header.stamp.nanosec = data["header"]["stamp"]["nanosec"]
    msg.header.frame_id = data["header"]["frame_id"]

    for d_data in data["detections"]:
        detection = Detection2D()
        if "id" in d_data:
            detection.id = d_data["id"]
        detection.bbox = BoundingBox2D()
        detection.bbox.center.position.x = d_data["bbox"]["center"]["x"]
        detection.bbox.center.position.y = d_data["bbox"]["center"]["y"]
        detection.bbox.size_x = d_data["bbox"]["size_x"]
        detection.bbox.size_y = d_data["bbox"]["size_y"]

        for r_data in d_data.get("results", []):
            result = ObjectHypothesisWithPose()
            result.hypothesis.class_id = r_data.get("class_id", "")
            result.hypothesis.score = r_data.get("score", 1.0)
            result.pose = PoseWithCovariance()
            result.pose.pose = Pose()
            pos = r_data["position"]
            result.pose.pose.position.x = pos["x"]
            result.pose.pose.position.y = pos["y"]
            result.pose.pose.position.z = pos.get("z", 0.0)
            result.pose.pose.orientation.w = 1.0
            detection.results.append(result)

        msg.detections.append(detection)

    return msg


def deserialize_detection3d_array(data: dict):
    """Rebuild a ``Detection3DArray`` from `serialize_detection3d_array` output.

    A file without covariances (or with all-zero ones) yields all-zero covariance,
    which downstream code reads as "the detector did not compute one" — unknown, not
    exact.
    """
    from geometry_msgs.msg import Pose, PoseWithCovariance
    from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

    msg = Detection3DArray()
    msg.header.stamp.sec = data["header"]["stamp"]["sec"]
    msg.header.stamp.nanosec = data["header"]["stamp"]["nanosec"]
    msg.header.frame_id = data["header"]["frame_id"]

    for d_data in data["detections"]:
        detection = Detection3D()
        for r_data in d_data["results"]:
            result = ObjectHypothesisWithPose()
            result.pose = PoseWithCovariance()
            result.pose.pose = Pose()
            result.pose.pose.position.x = r_data["pose"]["position"]["x"]
            result.pose.pose.position.y = r_data["pose"]["position"]["y"]
            result.pose.pose.position.z = r_data["pose"]["position"]["z"]
            result.pose.pose.orientation.x = r_data["pose"]["orientation"]["x"]
            result.pose.pose.orientation.y = r_data["pose"]["orientation"]["y"]
            result.pose.pose.orientation.z = r_data["pose"]["orientation"]["z"]
            result.pose.pose.orientation.w = r_data["pose"]["orientation"]["w"]

            covariance = r_data.get("covariance")
            if covariance:
                result.pose.covariance = [float(v) for v in covariance]

            detection.results.append(result)
        msg.detections.append(detection)

    return msg


def encode_detection_archive(
    snapshot,
    *,
    adjusted_rvec: np.ndarray | None,
    adjusted_tvec: np.ndarray | None,
) -> dict:
    """Encode one complete version-4 archive from a detached buffer snapshot."""
    data = {
        "version": FORMAT_VERSION,
        "board_frame_convention": BOARD_FRAME_CONVENTION,
        "num_detections": snapshot.frame_count,
        "detections": [
            {
                "aruco": serialize_detection2d_array(pair.aruco),
                "board": serialize_detection3d_array(pair.board),
            }
            for pair in snapshot.pairs
        ],
    }

    if (adjusted_rvec is None) != (adjusted_tvec is None):
        raise ValueError("adjusted rvec and tvec must be present together")
    if adjusted_rvec is not None:
        rvec = np.asarray(adjusted_rvec, dtype=np.float64)
        tvec = np.asarray(adjusted_tvec, dtype=np.float64)
        if (
            rvec.size != 3
            or tvec.size != 3
            or not (np.all(np.isfinite(rvec)) and np.all(np.isfinite(tvec)))
        ):
            raise ValueError("adjusted transform must contain finite 3-vectors")
        data["transform"] = {
            "rvec": rvec.reshape(3).tolist(),
            "tvec": tvec.reshape(3).tolist(),
        }

    estimate = snapshot.estimate
    if estimate is not None:
        quality = estimate.quality
        data["quality"] = {
            "status": quality.status_line(),
            "is_degenerate": quality.is_degenerate,
            "n_frames": quality.n_frames,
            "n_distinct_placements": quality.n_placements,
            "reprojection_rms_px": quality.residuals.rms_px,
            "reprojection_max_px": quality.residuals.max_px,
            "per_pose_rms_px": quality.residuals.per_pose_rms_px,
            "cond_JtJ": quality.conditioning.cond,
            "diversity": {
                "normal_span_deg": quality.diversity.normal_span_deg,
                "depth_range_m": quality.diversity.depth_range_m,
                "lateral_span_m": quality.diversity.lateral_span_m,
            },
            "uncertainty": (
                {
                    "rot_deg": quality.spread.rot_deg,
                    "trans_mm": quality.spread.trans_mm,
                    "n_subsets": quality.spread.n_subsets,
                }
                if quality.spread is not None
                else None
            ),
            "warnings": quality.warnings(),
        }
    return data


def decode_detection_archive(data: dict) -> DetectionArchive:
    """Validate and decode a complete archive without touching live state."""
    error = format_version_error(data)
    if error is not None:
        raise ValueError(error)

    detections = data.get("detections")
    if not isinstance(detections, list):
        raise TypeError("Detection archive 'detections' must be a list")
    declared_count = data.get("num_detections")
    if declared_count != len(detections):
        raise ValueError(
            "Detection archive count mismatch: "
            f"declares {declared_count}, contains {len(detections)}"
        )

    from lidar_to_camera_solver.detection_buffer import DetectionPair

    pairs = []
    for index, item in enumerate(detections):
        if not isinstance(item, dict) or "aruco" not in item or "board" not in item:
            raise ValueError(f"Detection archive pair {index} is malformed")
        try:
            pairs.append(
                DetectionPair(
                    aruco=deserialize_detection2d_array(item["aruco"]),
                    board=deserialize_detection3d_array(item["board"]),
                )
            )
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(
                f"Detection archive pair {index} is malformed: {error}"
            ) from error

    quality = _decode_quality(data.get("quality"))
    transform = _decode_transform(data.get("transform"))
    return DetectionArchive(tuple(pairs), quality, transform)


def select_loaded_adjustment(
    archive: DetectionArchive, snapshot, *, append: bool
) -> AdjustedTransform | None:
    """Apply version-4 adjustment anchoring rules after a successful restore."""
    estimate = snapshot.estimate
    if estimate is None:
        return None
    if not append and archive.adjusted_transform is not None:
        return archive.adjusted_transform

    rvec = np.array(estimate.rvec, dtype=np.float64, copy=True)
    tvec = np.array(estimate.tvec, dtype=np.float64, copy=True)
    rvec.setflags(write=False)
    tvec.setflags(write=False)
    return AdjustedTransform(rvec, tvec)


def _decode_transform(data: object) -> AdjustedTransform | None:
    if data is None:
        return None
    if not isinstance(data, dict):
        raise TypeError("Detection archive transform must be an object")
    try:
        rvec = np.asarray(data["rvec"], dtype=np.float64).reshape(3, 1)
        tvec = np.asarray(data["tvec"], dtype=np.float64).reshape(3, 1)
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(
            f"Detection archive transform is malformed: {error}"
        ) from error
    if not (np.all(np.isfinite(rvec)) and np.all(np.isfinite(tvec))):
        raise ValueError("Detection archive transform must be finite")
    rvec.setflags(write=False)
    tvec.setflags(write=False)
    return AdjustedTransform(rvec, tvec)


def _decode_quality(data: object) -> ArchivedQuality | None:
    if data is None:
        return None
    if not isinstance(data, dict):
        raise TypeError("Detection archive quality must be an object")
    try:
        diversity = data["diversity"]
        uncertainty = data.get("uncertainty")
        quality = ArchivedQuality(
            status=str(data["status"]),
            is_degenerate=bool(data["is_degenerate"]),
            n_frames=int(data["n_frames"]),
            n_distinct_placements=int(data["n_distinct_placements"]),
            reprojection_rms_px=float(data["reprojection_rms_px"]),
            reprojection_max_px=float(data["reprojection_max_px"]),
            per_pose_rms_px=tuple(float(value) for value in data["per_pose_rms_px"]),
            cond_jtj=float(data["cond_JtJ"]),
            normal_span_deg=float(diversity["normal_span_deg"]),
            depth_range_m=float(diversity["depth_range_m"]),
            lateral_span_m=float(diversity["lateral_span_m"]),
            uncertainty_rot_deg=(
                float(uncertainty["rot_deg"]) if uncertainty is not None else None
            ),
            uncertainty_trans_mm=(
                float(uncertainty["trans_mm"]) if uncertainty is not None else None
            ),
            uncertainty_n_subsets=(
                int(uncertainty["n_subsets"]) if uncertainty is not None else None
            ),
            warnings=tuple(str(value) for value in data.get("warnings", ())),
        )
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"Detection archive quality is malformed: {error}") from error

    numeric = (
        quality.reprojection_rms_px,
        quality.reprojection_max_px,
        *quality.per_pose_rms_px,
        quality.cond_jtj,
        quality.normal_span_deg,
        quality.depth_range_m,
        quality.lateral_span_m,
    )
    optional_numeric = (
        quality.uncertainty_rot_deg,
        quality.uncertainty_trans_mm,
    )
    if not all(np.isfinite(value) for value in numeric) or not all(
        value is None or np.isfinite(value) for value in optional_numeric
    ):
        raise ValueError("Detection archive quality must be finite")
    return quality


__all__ = [
    "FORMAT_VERSION",
    "MIGRATION_COMMAND",
    "AdjustedTransform",
    "ArchivedQuality",
    "DetectionArchive",
    "decode_detection_archive",
    "deserialize_detection2d_array",
    "deserialize_detection3d_array",
    "encode_detection_archive",
    "format_version_error",
    "migrate_v3_to_v4",
    "select_loaded_adjustment",
    "serialize_detection2d_array",
    "serialize_detection3d_array",
]
