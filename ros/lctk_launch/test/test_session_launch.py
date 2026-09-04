"""Graph contracts for the session launch files."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import ModuleType

import pytest
from lctk_launch.session import DEFAULT_RVIZ_CONFIG_PARTS

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
    "enable_rviz": "true",
    "solver_mode": "continuous",
    "enable_overlay": "false",
    "enable_judge": "false",
    "rviz_config": "",
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
        "enable_rviz",
        "enable_overlay",
        "log_level",
        "rviz_config",
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


def _calibrate_arguments(includes) -> dict:
    """The arguments session.launch.py hands to calibrate.launch.py."""
    for action in includes:
        if not hasattr(action, "launch_arguments"):
            continue
        arguments = {k: v for k, v in action.launch_arguments}
        if "config_file" in arguments:
            return arguments
    raise AssertionError("no calibrate include found")


def test_a_session_local_rviz_layout_is_forwarded(tmp_path):
    """An RViz layout is per-experiment, so a session that ships one gets it.

    Without this the layout had to be named again on every command line, which
    is how the justfile ended up repeating the same --rviz_config in four
    recipes -- a session detail living outside the session.
    """
    module = load(SESSION_LAUNCH, "session_launch_rviz")
    directory = make_pcap_session(tmp_path)
    (directory / module.SESSION_RVIZ).write_text("Panels: []\n", encoding="utf-8")
    includes = module.generate_session(_Context(directory, **FORWARDED_DEFAULTS))
    arguments = _calibrate_arguments(includes)
    assert arguments["rviz_config"] == str(directory / module.SESSION_RVIZ)


def test_a_session_without_a_layout_names_the_default_rather_than_omitting_it(tmp_path):
    """A session with no rviz.rviz must still reach RViz with a real layout.

    This test used to assert the opposite -- that `rviz_config` is *absent*
    from the forwarded arguments, on the reasoning that saying nothing leaves
    `calibrate.launch.py` the default's only owner. That reasoning is sound and
    the assertion held, but the behaviour it was standing in for did not:
    `session.launch.py` declares `rviz_config` so it can distinguish an untyped
    argument from a typed one, and a launch configuration set in a parent scope
    is inherited by every launch file it includes. The included
    `DeclareLaunchArgument` therefore never applied its default, and RViz was
    started as `-d ""`, opening its stock layout instead of this repo's.

    Observed on `sessions/solid600-handheld-seyond` (which ships no rviz.rviz)
    as `rviz2 -d  --ros-args`, with an empty path.

    So the assertion now checks the reachable outcome -- a concrete default is
    named -- and `DEFAULT_RVIZ_CONFIG_PARTS` in `lctk_launch.session` keeps the
    single-owner property the old docstring wanted.
    """
    module = load(SESSION_LAUNCH, "session_launch_rviz_absent")
    directory = make_pcap_session(tmp_path)
    includes = module.generate_session(_Context(directory, **FORWARDED_DEFAULTS))
    chosen = _calibrate_arguments(includes)["rviz_config"]
    assert chosen != "", "an empty rviz_config is what opens RViz's stock layout"
    # The value is a PathJoinSubstitution over [FindPackageShare(...), *parts].
    # Its `.substitutions` is a list of lists (each element is normalised to a
    # list of substitutions), so flatten before reading the literal tail -- that
    # tail is what identifies the layout without performing a launch context.
    literals = [
        part.text
        for group in chosen.substitutions
        for part in group
        if getattr(part, "text", None) is not None
    ]
    assert literals == list(DEFAULT_RVIZ_CONFIG_PARTS), literals


def test_an_explicit_rviz_config_beats_the_session_layout(tmp_path):
    """An operator who types rviz_config:= means it, session file or not."""
    module = load(SESSION_LAUNCH, "session_launch_rviz_explicit")
    directory = make_pcap_session(tmp_path)
    (directory / module.SESSION_RVIZ).write_text("Panels: []\n", encoding="utf-8")
    chosen = tmp_path / "mine.rviz"
    context = _Context(directory, **{**FORWARDED_DEFAULTS, "rviz_config": str(chosen)})
    arguments = _calibrate_arguments(module.generate_session(context))
    assert arguments["rviz_config"] == str(chosen)


def _bag_session(directory: Path) -> Path:
    """A minimal `kind: bag` session whose bag directory looks real enough to parse."""
    directory.mkdir(parents=True, exist_ok=True)
    bag = directory / "bag"
    bag.mkdir(exist_ok=True)
    (bag / "metadata.yaml").write_text(
        """
rosbag2_bagfile_information:
  version: 5
  storage_identifier: sqlite3
  relative_file_paths: [bag_0.db3]
  duration: { nanoseconds: 1000000000 }
  starting_time: { nanoseconds_since_epoch: 0 }
  message_count: 1
  topics_with_message_count:
    - topic_metadata:
        name: /points
        type: sensor_msgs/msg/PointCloud2
        serialization_format: cdr
      message_count: 1
""",
        encoding="utf-8",
    )
    (bag / "bag_0.db3").write_bytes(b"")
    (directory / "session.yaml").write_text(
        """
name: bagged
data: { kind: bag, path: $(session-dir)/bag }
devices:
  lidars:
    top: { frame_id: velodyne_top, pointcloud_topic: /points }
markers:
  calibration_board:
    target_config: $(find-pkg-share lctk_launch)/config/targets/hollow_1000_aruco_4_v1.json5
    detector_config: $(find-pkg-share lctk_launch)/config/board/hollow_1000/velodyne.json5
    pairs: []
sync: { tolerance_ms: 100, queue_size: 100, drop_policy: reject_new }
""",
        encoding="utf-8",
    )
    return directory


def _player_arguments(actions) -> list[str]:
    """The bag player's argv, read before the action is executed.

    `node_name` raises until launch has run the action, so the declared name is read
    from the attribute the constructor stored.
    """
    for action in actions:
        if getattr(action, "_Node__node_name", None) == "bag_player":
            return [str(a) for a in action._Node__arguments]
    raise AssertionError("no bag_player node in the generated actions")


def test_the_player_is_given_no_qos_override(data_launch, tmp_path):
    """The player replays each topic with the QoS the recording offers.

    `play_args` used to exist so an operator could override that, because the
    graph-wide `mode` argument could not express what the bag already knows.
    Subscribers now adapt per topic instead (lctk_launch/transport.py), so
    nothing overrides the player and `--clock` is the only argument it takes.
    """
    directory = _bag_session(tmp_path / "bagged")
    arguments = _player_arguments(data_launch.generate_data_source(_Context(directory)))
    assert [a for a in arguments if a.startswith("--play-arg=")] == [
        "--play-arg=--clock"
    ]
