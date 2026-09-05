"""The review scene wire format stays tied to solver-owned geometry."""

import threading
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from lctk_target import load_target
from lidar_to_camera_solver.detection_buffer import DetectionBuffer, DetectionPair
from lidar_to_camera_solver.main import LidarToCameraSolver

ROOT = Path(__file__).resolve().parents[3]
TARGET = load_target(ROOT / "ros/lctk_launch/config/targets/solid_600_aruco_1_v1.json5")


def _board_message(position=(0.7, -0.4, 4.2)):
    pose = SimpleNamespace(
        position=SimpleNamespace(x=position[0], y=position[1], z=position[2]),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    result = SimpleNamespace(pose=SimpleNamespace(pose=pose, covariance=[0.0] * 36))
    return SimpleNamespace(detections=[SimpleNamespace(results=[result])])


def _aruco_message():
    results = [
        SimpleNamespace(
            pose=SimpleNamespace(
                pose=SimpleNamespace(position=SimpleNamespace(x=x, y=y))
            )
        )
        for x, y in ((120.0, 100.0), (220.0, 100.0), (220.0, 200.0), (120.0, 200.0))
    ]
    return SimpleNamespace(detections=[SimpleNamespace(id="aruco_24", results=results)])


def _camera_info():
    return SimpleNamespace(width=640, height=480)


def _solver():
    matrix = np.array(
        [[500.0, 0.0, 320.0], [0.0, 500.0, 240.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    solver = object.__new__(LidarToCameraSolver)
    solver.state_lock = threading.RLock()
    solver._scene_revision = 0
    solver.parent_frame = "velodyne"
    solver.target = TARGET
    solver.camera_info = _camera_info()
    solver._camera_matrix = matrix
    solver.current_rvec = np.zeros((3, 1), dtype=np.float64)
    solver.current_tvec = np.array([[1.0], [2.0], [3.0]], dtype=np.float64)
    solver.detection_buffer = DetectionBuffer(
        camera_matrix=matrix,
        marker_corners_by_id=TARGET.marker_corners_by_id,
        min_frames_required=1,
        min_normal_spread_deg=0.0,
        min_depth_range_m=0.0,
        enforce_pose_diversity=False,
    )
    update = solver.detection_buffer.capture(
        DetectionPair(aruco=_aruco_message(), board=_board_message())
    )
    assert update.accepted
    return solver


def test_scene_quads_are_the_detection_buffers_world_points():
    solver = _solver()
    payload = solver.scene()
    snapshot = solver.detection_buffer.snapshot()

    assert payload["scene_revision"] == 0
    assert payload["world_frame_id"] == "velodyne"
    assert [capture["id"] for capture in payload["captures"]] == list(
        snapshot.capture_ids
    )
    expected = snapshot.scene_captures[0].marker_corners_world[0]
    actual = np.asarray(payload["captures"][0]["marker_quads_world"][0])
    assert np.allclose(actual, expected, atol=1e-12, rtol=0.0)

    outline = np.asarray(payload["captures"][0]["board_outline_world"])
    assert outline.shape == (5, 3)
    assert np.allclose(outline[0], [0.4, -0.7, 4.2], atol=1e-12, rtol=0.0)


def test_scene_camera_uses_one_inverted_optical_pose():
    solver = _solver()
    camera = solver.scene()["camera"]

    assert camera is not None
    assert camera["optical_pose_world"]["position"] == [-1.0, -2.0, -3.0]
    assert camera["optical_pose_world"]["orientation"] == [0.0, 0.0, 0.0, 1.0]
    assert camera["image_size"] == {"width": 640, "height": 480}


def test_scene_revision_advances_when_node_marks_a_mutation():
    solver = _solver()
    solver._mark_scene_mutation_locked()
    assert solver.scene()["scene_revision"] == 1
