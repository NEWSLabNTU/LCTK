#!/usr/bin/env python3
"""H-10 verification: dump->load preserves the real ArUco corners (no C-01 regression).

Builds a Detection2DArray whose 4 results carry known *rotated* corner pixels (not an
axis-aligned box), runs it through the node's serialize -> JSON -> deserialize path,
and asserts the corners survive exactly. Also checks a v2-style dict (no results)
deserializes to an empty results list (the documented fallback).

Requires: source install/setup.bash
"""
import json
import sys

from advanced_extrinsic_solver.main import AdvancedExtrinsicSolver
from geometry_msgs.msg import Pose, PoseWithCovariance
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

# Rotated (trapezoidal) corners -- the case the axis-aligned bbox cannot represent.
CORNERS = [(100.0, 200.0), (340.0, 190.0), (360.0, 430.0), (90.0, 450.0)]


def build_array():
    arr = Detection2DArray()
    arr.header.frame_id = "camera"
    det = Detection2D()
    det.id = "aruco_696"
    det.bbox = BoundingBox2D()
    det.bbox.center.position.x = 225.0
    det.bbox.center.position.y = 320.0
    det.bbox.size_x = 270.0
    det.bbox.size_y = 260.0
    for (x, y) in CORNERS:
        r = ObjectHypothesisWithPose()
        r.hypothesis.class_id = "696"
        r.hypothesis.score = 1.0
        r.pose = PoseWithCovariance()
        r.pose.pose = Pose()
        r.pose.pose.position.x = x
        r.pose.pose.position.y = y
        det.results.append(r)
    arr.detections.append(det)
    return arr


def main():
    node = object.__new__(AdvancedExtrinsicSolver)  # skip ROS __init__

    # Round trip through JSON, as dump/load does.
    serialized = json.loads(json.dumps(node._serialize_detection2d_array(build_array())))
    restored = node._deserialize_detection2d_array(serialized)

    det = restored.detections[0]
    assert det.id == "aruco_696", f"id lost: {det.id!r}"
    assert len(det.results) == 4, f"corners dropped: {len(det.results)} results (H-10 regression)"
    got = [(r.pose.pose.position.x, r.pose.pose.position.y) for r in det.results]
    assert got == CORNERS, f"corners changed: {got} != {CORNERS}"
    print(f"[1] round-trip preserved all 4 rotated corners: {got}")

    # A v2-style file (no results) -> empty results (fallback path), not a crash.
    v2 = json.loads(json.dumps(node._serialize_detection2d_array(build_array())))
    for d in v2["detections"]:
        d.pop("results", None)
    restored_v2 = node._deserialize_detection2d_array(v2)
    assert len(restored_v2.detections[0].results) == 0, "v2 should have no results"
    print("[2] v2 file (no results) deserializes to empty results (fallback)")

    print("\nH-10 PASS: dump->load preserves real corners; C-01 no longer regresses")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:  # noqa: BLE001
        print(f"H-10 FAIL: {e}")
        sys.exit(1)
