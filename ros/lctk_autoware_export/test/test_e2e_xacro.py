"""Phase 6 deliverable 3: exported YAML -> real xacro -> URDF joint == solver transform.

Runs the exporter on a copy of the fixture calibration, processes a minimal replica of
Autoware's sensor_kit.xacro (same xacro.load_yaml mechanism) with the patched file, and
checks the emitted joint origin reproduces the composed kit->camera_link transform.
"""

import json
import math
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
from lctk_autoware_export.export import load_solver_transform, patch_calibration
from lctk_autoware_export.frames import (
    OPTICAL_IN_CAMERA_LINK,
    entry_to_transform,
    inv_transform,
    kit_to_camera_link,
    make_transform,
    rpy_to_matrix,
)

xacro = pytest.importorskip("xacro", reason="ROS xacro not on PYTHONPATH")

FIXTURES = Path(__file__).parent / "fixtures"
ARCHIVE_FIXTURES = (
    Path(__file__).resolve().parents[3] / "fixtures" / "detection_archives"
)


def test_exported_yaml_round_trips_through_xacro(tmp_path):
    # A non-trivial solve: camera 0.4 m ahead, 0.1 m left of the lidar, yawed 0.2 rad.
    T_lidar_optical = make_transform(
        rpy_to_matrix(0.0, 0.0, 0.2) @ OPTICAL_IN_CAMERA_LINK[:3, :3],
        [0.4, 0.1, -0.05],
    )
    T_solve = inv_transform(T_lidar_optical)
    R = T_solve[:3, :3]
    angle = math.acos(max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0)))
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (
        2.0 * math.sin(angle)
    )
    detections = tmp_path / "detections.json"
    detections.write_text(
        json.dumps(
            {
                "version": 4,
                "board_frame_convention": "corner_aligned_plate_center_v1",
                "transform": {
                    "rvec": (axis * angle).tolist(),
                    "tvec": T_solve[:3, 3].tolist(),
                },
            }
        )
    )

    config_dir = tmp_path / "config"
    config_dir.mkdir()
    target = config_dir / "sensor_kit_calibration.yaml"
    shutil.copy(FIXTURES / "sensor_kit_calibration.yaml", target)

    rvec, tvec = load_solver_transform(detections)
    patch_calibration(
        target,
        rvec=rvec,
        tvec=tvec,
        camera_frame="camera0/camera_link",
        lidar_frame="velodyne_top_base_link",
    )

    # Real xacro pipeline, exactly as tier4_vehicle_launch invokes it.
    urdf = xacro.process_file(
        str(FIXTURES / "sensor_kit.xacro"),
        mappings={"config_dir": str(config_dir)},
    ).toxml()
    origin = ET.fromstring(urdf).find("joint[@name='camera0/camera_link_joint']/origin")
    assert origin is not None, urdf
    xyz = [float(v) for v in origin.get("xyz").split()]
    roll, pitch, yaw = (float(v) for v in origin.get("rpy").split())
    T_urdf = entry_to_transform(
        {
            "x": xyz[0],
            "y": xyz[1],
            "z": xyz[2],
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
        }
    )

    T_kit_lidar = entry_to_transform(
        {"x": 0.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 1.575}
    )
    T_expected = kit_to_camera_link(T_kit_lidar, rvec, tvec)
    np.testing.assert_allclose(T_urdf, T_expected, atol=1e-9)


def test_paired_archives_produce_identical_xacro_transforms(tmp_path):
    """The v5 identity must not change an already-solved export transform."""
    transforms = []
    for name in ("solved_v4.json", "solved_v5.json"):
        detections = tmp_path / name
        detections.write_text((ARCHIVE_FIXTURES / name).read_text())
        config_dir = tmp_path / name.removesuffix(".json")
        config_dir.mkdir()
        target = config_dir / "sensor_kit_calibration.yaml"
        shutil.copy(FIXTURES / "sensor_kit_calibration.yaml", target)

        rvec, tvec = load_solver_transform(detections)
        patch_calibration(
            target,
            rvec=rvec,
            tvec=tvec,
            camera_frame="camera0/camera_link",
            lidar_frame="velodyne_top_base_link",
        )
        urdf = xacro.process_file(
            str(FIXTURES / "sensor_kit.xacro"),
            mappings={"config_dir": str(config_dir)},
        ).toxml()
        origin = ET.fromstring(urdf).find(
            "joint[@name='camera0/camera_link_joint']/origin"
        )
        assert origin is not None, urdf
        transforms.append(
            np.array(
                [
                    *(float(v) for v in origin.get("xyz").split()),
                    *(float(v) for v in origin.get("rpy").split()),
                ]
            )
        )

    np.testing.assert_array_equal(transforms[0], transforms[1])
