"""The review scene wire format stays tied to solver-owned geometry."""

import threading
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from lctk_target import load_target
from lidar_to_camera_solver.detection_buffer import (
    BufferSnapshot,
    DetectionBuffer,
    DetectionPair,
    Empty,
)
from lidar_to_camera_solver.main import LidarToCameraSolver

ROOT = Path(__file__).resolve().parents[3]
TARGET = load_target(ROOT / "ros/lctk_launch/config/targets/solid_600_aruco_1_v1.json5")


def _board_message(position=(0.7, -0.4, 4.2), orientation=(0.0, 0.0, 0.0, 1.0)):
    pose = SimpleNamespace(
        position=SimpleNamespace(x=position[0], y=position[1], z=position[2]),
        orientation=SimpleNamespace(
            x=orientation[0],
            y=orientation[1],
            z=orientation[2],
            w=orientation[3],
        ),
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


def _solver(*, orientation=(0.0, 0.0, 0.0, 1.0)):
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
        DetectionPair(
            aruco=_aruco_message(), board=_board_message(orientation=orientation)
        )
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
    radius = TARGET.plate.side_um * 1e-6 / np.sqrt(2.0)
    assert np.allclose(
        outline,
        [
            [0.7, -0.4 - radius, 4.2],
            [0.7 + radius, -0.4, 4.2],
            [0.7, -0.4 + radius, 4.2],
            [0.7 - radius, -0.4, 4.2],
            [0.7, -0.4 - radius, 4.2],
        ],
        atol=1e-12,
        rtol=0.0,
    )


def test_scene_outline_uses_the_same_board_pose_as_marker_geometry():
    angle = np.deg2rad(31.0)
    solver = _solver(orientation=(0.0, 0.0, np.sin(angle / 2.0), np.cos(angle / 2.0)))
    payload = solver.scene()
    outline = np.asarray(payload["captures"][0]["board_outline_world"])
    radius = TARGET.plate.side_um * 1e-6 / np.sqrt(2.0)
    local = np.asarray(
        [
            [0.0, -radius, 0.0],
            [radius, 0.0, 0.0],
            [0.0, radius, 0.0],
            [-radius, 0.0, 0.0],
            [0.0, -radius, 0.0],
        ]
    )
    rotation = np.asarray(
        [
            [np.cos(angle), -np.sin(angle), 0.0],
            [np.sin(angle), np.cos(angle), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    expected = (rotation @ local.T).T + np.asarray([0.7, -0.4, 4.2])
    assert np.allclose(outline, expected, atol=1e-12, rtol=0.0)


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


def test_state_reports_the_camera_source_stamp_and_export_availability():
    solver = _solver()
    solver.solver_mode = "assisted"
    stamp = SimpleNamespace(sec=12, nanosec=345_000_000)
    pair = DetectionPair(
        aruco=SimpleNamespace(header=SimpleNamespace(stamp=stamp)),
        board=SimpleNamespace(),
    )
    snapshot = BufferSnapshot(
        revision=1,
        pairs=(pair,),
        placements=(),
        correspondence_count=4,
        outcome=Empty(),
        capture_ids=(7,),
    )
    solver._snapshot = lambda: snapshot
    solver._evidence_store = None
    solver._last_stillness = None
    solver._stillness = None
    solver._stability_params = {}
    solver._review_params_writable = False
    solver._review_params_detail = ""
    solver.pair_source = SimpleNamespace(status_line=lambda: "waiting")
    solver.identity_gate = SimpleNamespace(error=None)
    parameters = {
        "review_archive_path": "",
        "export_autoware_target": "",
        "export_camera_frame": "camera",
        "export_lidar_frame": "",
    }
    solver._string_parameter = lambda name: parameters[name]

    state = solver.state()

    assert state["pairs"] == [
        {
            "id": 7,
            "rms_px": None,
            "has_preview": False,
            "missing": ["camera frame", "plane inliers"],
            "stamp_s": 12.345,
            "evidence_revision": 0,
        }
    ]
    assert state["export"]["autoware_ready"] is False
    assert state["export_availability"]["autoware"]["missing"] == [
        "export_autoware_target",
        "export_lidar_frame",
    ]


def test_review_scene_reuses_snapshot_until_buffer_or_pose_changes(monkeypatch):
    solver = _solver()
    original = solver.detection_buffer.snapshot
    copies = []

    def counted_snapshot():
        copies.append(True)
        return original()

    monkeypatch.setattr(solver.detection_buffer, "snapshot", counted_snapshot)
    first = solver.scene()
    assert solver.scene() is first
    assert len(copies) == 1

    solver.current_tvec[0, 0] += 1
    moved = solver.scene()
    assert moved["scene_revision"] > first["scene_revision"]
    assert moved["camera"] != first["camera"]
    assert len(copies) == 1

    solver.detection_buffer.remove(0)
    empty = solver.scene()
    assert empty["captures"] == []
    assert empty["scene_revision"] > moved["scene_revision"]
    assert len(copies) == 2


def test_review_scene_republishes_revision_when_geometry_returns_to_an_old_key():
    solver = _solver()
    first = solver.scene()
    solver.current_tvec[0, 0] += 1.0
    moved = solver.scene()
    solver.current_tvec[0, 0] -= 1.0
    returned = solver.scene()

    assert moved["scene_revision"] > first["scene_revision"]
    assert returned["scene_revision"] > moved["scene_revision"]
