"""The lctk_session tool: list, check, new.

`check` is the piece that earns its keep -- it answers "why is nothing being
detected" before the run rather than after, without starting a graph.
"""

from lctk_launch.session_cli import main

MANIFEST = """
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
"""


def make_session(tmp_path, with_data=True, name="rig"):
    directory = tmp_path / name
    directory.mkdir(parents=True)
    (directory / "session.yaml").write_text(MANIFEST, encoding="utf-8")
    if with_data:
        (directory / "data").mkdir()
        (directory / "data" / "lidar.pcap").write_bytes(b"")
        (directory / "data" / "video.avi").write_bytes(b"")
    return directory


def test_check_passes_on_a_complete_session(tmp_path, capsys):
    directory = make_session(tmp_path)
    assert main(["check", str(directory)]) == 0
    assert "top" in capsys.readouterr().out


def test_check_reports_the_derived_topics(tmp_path, capsys):
    directory = make_session(tmp_path)
    main(["check", str(directory)])
    assert "/sensing/lidar/top/pointcloud_raw" in capsys.readouterr().out


def test_check_fails_when_the_data_is_missing(tmp_path, capsys):
    directory = make_session(tmp_path, with_data=False)
    assert main(["check", str(directory)]) != 0
    assert "data" in capsys.readouterr().err.lower()


def test_check_names_the_resolved_path_not_the_substitution(tmp_path, capsys):
    directory = make_session(tmp_path, with_data=False)
    main(["check", str(directory)])
    assert "$(session-dir)" not in capsys.readouterr().err


def test_list_finds_sessions_in_a_given_directory(tmp_path, capsys):
    make_session(tmp_path, name="rig-a")
    make_session(tmp_path, name="rig-b")
    assert main(["list", str(tmp_path)]) == 0
    out = capsys.readouterr().out
    assert "rig-a" in out and "rig-b" in out


def test_list_ignores_directories_without_a_manifest(tmp_path, capsys):
    make_session(tmp_path, name="rig-a")
    (tmp_path / "not-a-session").mkdir()
    main(["list", str(tmp_path)])
    assert "not-a-session" not in capsys.readouterr().out


def test_new_scaffolds_a_session(tmp_path):
    template = make_session(tmp_path, name="template")
    target = tmp_path / "fresh"
    assert main(["new", str(target), "--from", str(template)]) == 0
    assert (target / "session.yaml").is_file()
    assert (target / "out").is_dir()


def test_new_refuses_to_overwrite(tmp_path, capsys):
    template = make_session(tmp_path, name="template")
    target = make_session(tmp_path, name="existing")
    assert main(["new", str(target), "--from", str(template)]) != 0
    assert "exists" in capsys.readouterr().err.lower()


def test_list_reports_each_session_once_across_overlapping_roots(tmp_path, capsys):
    """`--symlink-install` makes ./sessions/x and <share>/sessions/x the same
    files. Listing both roots must not imply there are two of the session."""
    # Mirror what --symlink-install actually does: a real directory in the share,
    # with the files inside it symlinked back to the source. Symlinking the
    # directory itself would be an easier case than the one that occurs.
    real = make_session(tmp_path, name="rig-a")
    mirror = tmp_path / "share" / "rig-a"
    mirror.mkdir(parents=True)
    (mirror / "session.yaml").symlink_to(real / "session.yaml")

    assert main(["list", str(tmp_path), str(tmp_path / "share")]) == 0

    lines = [ln for ln in capsys.readouterr().out.splitlines() if "rig-a" in ln]
    assert len(lines) == 1, f"listed {len(lines)} times: {lines}"
