"""Session resolution and path substitution.

A session directory must work wherever it is -- inside this repo or in an
operator's own tree -- so nothing here may assume a location, a working
directory, or a search path.
"""

import pytest
from lctk_launch.session import (
    MANIFEST_NAME,
    SessionError,
    resolve_config_path,
    resolve_session,
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
