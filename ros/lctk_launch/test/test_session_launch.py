"""Graph contracts for the session launch files."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import ModuleType

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DATA_LAUNCH = PACKAGE_ROOT / "launch" / "session_data.launch.py"

# Keep source-tree execution consistent with the existing lctk_launch tests.
sys.path.insert(0, str(PACKAGE_ROOT))


def load(path: Path, name: str) -> ModuleType:
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class _Context:
    def __init__(self, session: Path):
        self.launch_configurations = {"session": str(session)}

    def perform_substitution(self, substitution):
        return substitution.perform(self)


@pytest.fixture(scope="module")
def data_launch() -> ModuleType:
    return load(DATA_LAUNCH, "session_data_launch")


def make_pcap_session(tmp_path):
    directory = tmp_path / "rig"
    (directory / "data").mkdir(parents=True)
    (directory / "data" / "lidar.pcap").write_bytes(b"")
    (directory / "data" / "video.avi").write_bytes(b"")
    (directory / "session.yaml").write_text(
        """
name: rig
data:
  kind: pcap_avi
  dir: $(session-dir)/data
devices:
  lidars:
    top: { frame_id: velodyne_top }
  cameras:
    front_center: { frame_id: camera_front_center }
markers:
  calibration_board:
    target_config: $(find-pkg-share lctk_launch)/config/targets/hollow_1000_aruco_4_v1.json5
    detector_config: $(find-pkg-share lctk_launch)/config/board/hollow_1000/velodyne.json5
    pairs:
      - [top, front_center]
sync: { tolerance_ms: 100, queue_size: 100, drop_policy: reject_new }
""",
        encoding="utf-8",
    )
    return directory


def test_pcap_avi_includes_the_playback_launch_with_derived_topics(
    data_launch, tmp_path
):
    directory = make_pcap_session(tmp_path)
    actions = data_launch.generate_data_source(_Context(directory))
    arguments = {}
    for action in actions:
        if hasattr(action, "launch_arguments"):
            arguments.update({k: v for k, v in action.launch_arguments})
    assert arguments["pointcloud_topic"] == "/sensing/lidar/top/pointcloud_raw"
    assert arguments["camera_namespace"] == "/sensing/camera/front_center"
    assert arguments["lidar_frame_id"] == "velodyne_top"
    assert arguments["camera_frame_id"] == "camera_front_center"
    assert arguments["pcap_file"].endswith("/data/lidar.pcap")
    assert arguments["video_file"].endswith("/data/video.avi")


def test_live_generates_no_data_nodes(data_launch, tmp_path):
    directory = tmp_path / "live"
    directory.mkdir()
    (directory / "session.yaml").write_text(
        """
name: live
data: { kind: live }
devices:
  lidars:
    top: { frame_id: velodyne_top, pointcloud_topic: /points }
  cameras:
    front_center: { frame_id: cam, image_topic: /image }
markers:
  calibration_board:
    target_config: $(find-pkg-share lctk_launch)/config/targets/hollow_1000_aruco_4_v1.json5
    detector_config: $(find-pkg-share lctk_launch)/config/board/hollow_1000/velodyne.json5
    pairs:
      - [top, front_center]
sync: { tolerance_ms: 100, queue_size: 100, drop_policy: reject_new }
""",
        encoding="utf-8",
    )
    actions = data_launch.generate_data_source(_Context(directory))
    assert not [a for a in actions if hasattr(a, "launch_arguments")]


def test_a_missing_session_is_refused(data_launch, tmp_path):
    with pytest.raises(Exception, match="no session"):
        data_launch.generate_data_source(_Context(tmp_path / "absent"))
