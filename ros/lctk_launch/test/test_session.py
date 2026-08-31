"""Session resolution and path substitution.

A session directory must work wherever it is -- inside this repo or in an
operator's own tree -- so nothing here may assume a location, a working
directory, or a search path.
"""

from pathlib import Path

import pytest
import yaml
from lctk_launch.session import (
    MANIFEST_NAME,
    SessionError,
    bag_topics,
    derived_camera_topics,
    derived_lidar_topics,
    parse_data,
    resolve_config_path,
    resolve_session,
    verify_bag_topics,
)


def make_session(tmp_path, name="rig-a", body="name: rig-a\n"):
    directory = tmp_path / name
    directory.mkdir(parents=True)
    (directory / MANIFEST_NAME).write_text(body, encoding="utf-8")
    return directory


def test_a_directory_resolves_to_the_manifest_inside_it(tmp_path):
    directory = make_session(tmp_path)
    paths = resolve_session(str(directory))
    assert paths.manifest == directory / MANIFEST_NAME
    assert paths.directory == directory


def test_a_manifest_file_resolves_to_its_own_directory(tmp_path):
    directory = make_session(tmp_path)
    paths = resolve_session(str(directory / MANIFEST_NAME))
    assert paths.manifest == directory / MANIFEST_NAME
    assert paths.directory == directory


def test_a_relative_path_resolves_against_the_working_directory(tmp_path, monkeypatch):
    make_session(tmp_path)
    monkeypatch.chdir(tmp_path)
    paths = resolve_session("rig-a")
    assert paths.directory == tmp_path / "rig-a"


def test_the_resolved_paths_are_absolute(tmp_path, monkeypatch):
    make_session(tmp_path)
    monkeypatch.chdir(tmp_path)
    paths = resolve_session("rig-a")
    assert paths.manifest.is_absolute()
    assert paths.directory.is_absolute()


def test_a_missing_path_is_refused_and_names_what_was_tried(tmp_path):
    with pytest.raises(SessionError) as excinfo:
        resolve_session(str(tmp_path / "nope"))
    assert "nope" in str(excinfo.value)


def test_a_directory_without_a_manifest_is_refused_by_name(tmp_path):
    empty = tmp_path / "empty"
    empty.mkdir()
    with pytest.raises(SessionError) as excinfo:
        resolve_session(str(empty))
    assert MANIFEST_NAME in str(excinfo.value)
    assert "empty" in str(excinfo.value)


def test_there_is_no_implicit_search_path(tmp_path, monkeypatch):
    """A bare name must not be looked up anywhere but the working directory.

    An implicit ./sessions would assume both where sessions live and where the
    user is standing; the whole point is that a session can live anywhere.
    """
    sessions = tmp_path / "sessions"
    sessions.mkdir()
    make_session(sessions, name="rig-a")
    monkeypatch.chdir(tmp_path)
    with pytest.raises(SessionError):
        resolve_session("rig-a")


def test_session_dir_substitution(tmp_path):
    resolved = resolve_config_path("$(session-dir)/bbox.json5", tmp_path)
    assert resolved == str(tmp_path / "bbox.json5")


def test_session_dir_without_a_directory_context_is_refused(tmp_path):
    with pytest.raises(SessionError, match=r"session-dir"):
        resolve_config_path("$(session-dir)/bbox.json5", None)


def test_find_pkg_share_still_works(tmp_path):
    resolved = resolve_config_path("$(find-pkg-share lctk_launch)/config", tmp_path)
    assert resolved.endswith("/lctk_launch/config")
    assert "$(" not in resolved


def test_both_substitutions_in_one_string(tmp_path):
    resolved = resolve_config_path(
        "$(session-dir)/x:$(find-pkg-share lctk_launch)/y", tmp_path
    )
    assert str(tmp_path) in resolved
    assert "$(" not in resolved


def test_an_unknown_package_is_refused(tmp_path):
    with pytest.raises(Exception, match="no_such_package"):
        resolve_config_path("$(find-pkg-share no_such_package)/x", tmp_path)


def test_a_plain_path_is_returned_unchanged(tmp_path):
    assert resolve_config_path("/abs/path.json5", tmp_path) == "/abs/path.json5"


def write_bag(tmp_path, topics, name="bag"):
    bag = tmp_path / name
    bag.mkdir(parents=True)
    (bag / "metadata.yaml").write_text(
        yaml.safe_dump(
            {
                "rosbag2_bagfile_information": {
                    "topics_with_message_count": [
                        {"topic_metadata": {"name": t}} for t in topics
                    ]
                }
            }
        ),
        encoding="utf-8",
    )
    return bag


def test_derived_topics_match_the_existing_playback_defaults():
    """These are lidar_camera.launch.xml's defaults, so dataset 3 keeps its names."""
    assert derived_lidar_topics("top") == {
        "pointcloud": "/sensing/lidar/top/pointcloud_raw",
        "packets": "/sensing/lidar/top/velodyne_packets",
    }
    assert derived_camera_topics("front_center") == {
        "image": "/sensing/camera/front_center/image_raw",
        "camera_info": "/sensing/camera/front_center/camera_info",
        "namespace": "/sensing/camera/front_center",
    }


def test_pcap_avi_resolves_its_directory(tmp_path):
    data_dir = tmp_path / "3"
    data_dir.mkdir()
    (data_dir / "lidar.pcap").write_bytes(b"")
    (data_dir / "video.avi").write_bytes(b"")
    source = parse_data(
        {"kind": "pcap_avi", "dir": "$(session-dir)/3"}, session_dir=tmp_path
    )
    assert source.kind == "pcap_avi"
    assert source.directory == data_dir


def test_pcap_avi_missing_directory_names_the_resolved_path(tmp_path):
    with pytest.raises(SessionError) as excinfo:
        parse_data({"kind": "pcap_avi", "dir": "$(session-dir)/gone"}, tmp_path)
    assert str(tmp_path / "gone") in str(excinfo.value)
    assert "$(" not in str(excinfo.value), "must name the resolved path, not the source"


def test_pcap_avi_missing_lidar_pcap_is_refused(tmp_path):
    data_dir = tmp_path / "3"
    data_dir.mkdir()
    (data_dir / "video.avi").write_bytes(b"")
    with pytest.raises(SessionError, match="lidar.pcap"):
        parse_data({"kind": "pcap_avi", "dir": "$(session-dir)/3"}, tmp_path)


def test_pcap_avi_defaults_are_the_vlp32c(tmp_path):
    data_dir = tmp_path / "3"
    data_dir.mkdir()
    (data_dir / "lidar.pcap").write_bytes(b"")
    (data_dir / "video.avi").write_bytes(b"")
    source = parse_data({"kind": "pcap_avi", "dir": "$(session-dir)/3"}, tmp_path)
    assert source.lidar_model == "vlp32c"
    assert source.lidar_rpm == 600.0


def test_bag_resolves_and_requires_metadata(tmp_path):
    bag = write_bag(tmp_path, ["/a", "/b"])
    source = parse_data({"kind": "bag", "path": "$(session-dir)/bag"}, tmp_path)
    assert source.kind == "bag"
    assert source.path == bag


def test_bag_without_metadata_is_refused(tmp_path):
    bare = tmp_path / "bare"
    bare.mkdir()
    with pytest.raises(SessionError, match="metadata.yaml"):
        parse_data({"kind": "bag", "path": "$(session-dir)/bare"}, tmp_path)


def test_bag_topics_are_read_from_metadata(tmp_path):
    bag = write_bag(
        tmp_path, ["/lidar/vlp32/velodyne_points", "/lidar/falcon/iv_points"]
    )
    assert bag_topics(bag) == [
        "/lidar/vlp32/velodyne_points",
        "/lidar/falcon/iv_points",
    ]


def test_verify_bag_topics_accepts_a_match(tmp_path):
    bag = write_bag(tmp_path, ["/x", "/y"])
    verify_bag_topics(bag, ["/x"])


def test_verify_bag_topics_refuses_and_lists_what_the_bag_has(tmp_path):
    """This is M-26: two_lidar.yaml names /velodyne_points, the bag publishes
    /lidar/vlp32/velodyne_points. A silent hang becomes a startup error that
    tells the operator the answer."""
    bag = write_bag(tmp_path, ["/lidar/vlp32/velodyne_points"])
    with pytest.raises(SessionError) as excinfo:
        verify_bag_topics(bag, ["/velodyne_points"])
    message = str(excinfo.value)
    assert "/velodyne_points" in message
    assert "/lidar/vlp32/velodyne_points" in message


def test_live_needs_no_paths():
    source = parse_data({"kind": "live"}, Path("/tmp"))
    assert source.kind == "live"
    assert source.directory is None and source.path is None


def test_an_unknown_kind_lists_the_known_ones(tmp_path):
    with pytest.raises(SessionError) as excinfo:
        parse_data({"kind": "rosbag"}, tmp_path)
    for kind in ("pcap_avi", "bag", "live"):
        assert kind in str(excinfo.value)


def test_a_missing_data_section_is_refused(tmp_path):
    with pytest.raises(SessionError, match="data"):
        parse_data(None, tmp_path)


def test_an_unknown_key_in_data_is_refused(tmp_path):
    with pytest.raises(SessionError, match="pcap_dir"):
        parse_data({"kind": "live", "pcap_dir": "x"}, tmp_path)
