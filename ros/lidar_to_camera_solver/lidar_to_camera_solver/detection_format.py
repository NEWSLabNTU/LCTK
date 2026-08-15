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

from lidar_to_camera_solver.board_geometry import BOARD_FRAME_CONVENTION

#: Bumped from 3 in the same change that altered what a stored pose means, so version
#: and meaning stay in step.
FORMAT_VERSION = 4

#: The one-shot conversion command named in the rejection message.
MIGRATION_COMMAND = "ros2 run lidar_to_camera_solver migrate_detections"


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
    migrated["version"] = FORMAT_VERSION
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


__all__ = [
    "FORMAT_VERSION",
    "MIGRATION_COMMAND",
    "deserialize_detection2d_array",
    "deserialize_detection3d_array",
    "format_version_error",
    "migrate_v3_to_v4",
    "serialize_detection2d_array",
    "serialize_detection3d_array",
]
