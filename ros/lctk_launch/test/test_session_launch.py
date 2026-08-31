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
    """A LaunchContext stand-in holding only the configurations under test.

    ``**configurations`` stands in for what ``DeclareLaunchArgument`` puts into a
    real context before the ``OpaqueFunction`` runs; a launch file performing an
    argument it declares needs that argument present here too.
    """

    def __init__(self, session: Path, **configurations: str):
        self.launch_configurations = {"session": str(session), **configurations}

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


SESSION_LAUNCH = PACKAGE_ROOT / "launch" / "session.launch.py"

# What DeclareLaunchArgument puts in the context before the OpaqueFunction runs.
# The names are pinned independently by the declaration test below.
FORWARDED_DEFAULTS = {
    "debug_mode": "false",
    "log_level": "info",
    "mode": "offline",
    "enable_rviz": "true",
    "solver_mode": "continuous",
    "enable_overlay": "false",
    "enable_judge": "false",
}


def test_session_launch_declares_session_and_the_calibrate_arguments():
    module = load(SESSION_LAUNCH, "session_launch")
    description = module.generate_launch_description()
    names = {
        action.name
        for action in description.entities
        if hasattr(action, "name") and action.name
    }
    assert "session" in names
    for expected in (
        "solver_mode",
        "mode",
        "enable_rviz",
        "enable_overlay",
        "log_level",
    ):
        assert expected in names, f"{expected} must still be settable end to end"


def test_session_launch_feeds_the_same_manifest_to_both_halves(tmp_path):
    """The data source and the calibration graph must read one file.

    If they can be pointed at different files the design's guarantee is gone --
    that is the two-sources-of-truth bug this whole change exists to remove.
    """
    module = load(SESSION_LAUNCH, "session_launch2")
    directory = make_pcap_session(tmp_path)
    includes = module.generate_session(_Context(directory, **FORWARDED_DEFAULTS))
    argument_sets = [
        {k: v for k, v in action.launch_arguments}
        for action in includes
        if hasattr(action, "launch_arguments")
    ]
    sessions = {a["session"] for a in argument_sets if "session" in a}
    configs = {a["config_file"] for a in argument_sets if "config_file" in a}
    assert sessions == {str(directory)}
    assert configs == {str(directory / "session.yaml")}
