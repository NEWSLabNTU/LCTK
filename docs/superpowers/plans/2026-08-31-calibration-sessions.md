# Calibration Sessions Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make one directory describe one calibration run — data source and calibration config together, relocatable, runnable from anywhere with a plain `ros2 launch`.

**Architecture:** A new ROS-free `session.py` in `lctk_launch` owns manifest resolution, the `$(session-dir)` substitution, and the derive/verify/state rules for topics. `config_parser.py` gains a `data:` section and passes the session directory to path resolution. Two thin launch files (`session_data.launch.py`, `session.launch.py`) and one console script (`lctk_session`) sit on top. The justfile becomes pure aliases.

**Tech Stack:** Python 3.10, ROS 2 Humble (`launch`, `launch_ros`, `ament_index_python`), PyYAML, pytest, just.

**Spec:** [`docs/superpowers/specs/2026-08-31-calibration-sessions-design.md`](../specs/2026-08-31-calibration-sessions-design.md)

## Global Constraints

- **Never `pip3 install --user` anything.** `CLAUDE.md` Known Issue 3 — pip installs of `setuptools`, `numpy`, `scipy` and `anyio` have shadowed apt packages and broken this build four separate times.
- **Build with `just build`**, never a raw `colcon build`.
- **Run tests as `python3 -m pytest`, never bare `pytest`** — apt's `python3-pytest` ships no `pytest` executable, which is L-28.
- **`just lint-py` must stay clean** (`ruff check ros/` + `ruff format --check ros/`, line length 88).
- **`session.py` imports no ROS and no `launch` types.** Its tests must run without a graph. If a test there needs `rclpy`, the seam has leaked.
- **Every malformed-manifest case is a startup refusal, not a warning.** The failures this design exists to prevent are all silent; converting them into loud ones is the point.
- **Error messages name the resolved absolute path**, never the unresolved `$(…)` string — an operator debugging a missing file needs the path that was actually tried.
- **`session:=` is always an explicit path.** No search path, no implicit `./sessions`, no `LCTK_SESSION_PATH`.
- **After adding or changing a test recipe, break an assertion deliberately and confirm a non-zero exit** before trusting it. Do not read `$?` through a pipe.

### Reference: facts verified against the tree

- `resolve_package_path(path)` is `ros/lctk_launch/lctk_launch/config_parser.py:36`; call sites are lines **481, 486, 543, 545, 548, 551, 722**.
- `parse_config(config_path) -> PipelineConfig` is at `config_parser.py:857`.
- `PipelineConfig` already carries `sync: SyncSettings | None` and `assisted: AssistedSettings`; `_parse_sync` / `_parse_assisted` are the pattern to copy.
- `lidar_camera.launch.xml` arguments: `pcap_file`, `pointcloud_topic`, `velodyne_packets_topic`, `lidar_frame_id`, `rpm`, `port`, `read_fast`, `video_file`, `camera_name`, `camera_namespace`, `camera_info_url`, `camera_frame_id`, `use_sensor_data_qos`.
- Its defaults are `/sensing/lidar/top/pointcloud_raw`, `/sensing/lidar/top/velodyne_packets`, `velodyne_top`, `/sensing/camera/front_center`, `camera_front_center` — the derived convention below reproduces these exactly when the lidar device is named `top` and the camera `front_center`.
- A rosbag2 `metadata.yaml` lists topics at `rosbag2_bagfile_information.topics_with_message_count[].topic_metadata.name`. Verified on `ros/lctk_sample_data/bags/TWO_LIDAR_1`: `['/lidar/falcon/iv_points', '/lidar/vlp32/velodyne_points']`.
- `lctk_launch/setup.py` already has `console_scripts` (`tf_tree_broadcaster`) and installs `config/` by walking the tree — `sessions/` needs the same walk.

---

### Task 1: `session.py` — resolve a session, resolve its paths

**Files:**
- Create: `ros/lctk_launch/lctk_launch/session.py`
- Test: `ros/lctk_launch/test/test_session.py`

**Interfaces:**
- Consumes: nothing.
- Produces:
  - `class SessionError(Exception)`
  - `@dataclass(frozen=True) class SessionPaths: manifest: Path; directory: Path`
  - `resolve_session(spec: str) -> SessionPaths`
  - `resolve_config_path(path: str, session_dir: Path | None = None) -> str`
  - `MANIFEST_NAME = "session.yaml"`

- [ ] **Step 1: Write the failing tests**

```python
"""Session resolution and path substitution.

A session directory must work wherever it is -- inside this repo or in an
operator's own tree -- so nothing here may assume a location, a working
directory, or a search path.
"""

from pathlib import Path

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
```

- [ ] **Step 2: Run the tests and confirm they fail**

```bash
cd /home/jetson/LCTK && source /opt/ros/humble/setup.bash
PYTHONPATH="$PWD/ros/lctk_launch:$PYTHONPATH" \
  python3 -m pytest ros/lctk_launch/test/test_session.py -q --no-header
```

Expected: `ModuleNotFoundError: No module named 'lctk_launch.session'`.

- [ ] **Step 3: Implement resolution and substitution**

```python
"""What a calibration session is, and how to find one.

A session is one directory describing one run: where the data comes from, and
everything needed to calibrate against it. It is self-contained and relocatable,
so it can live inside this repo or in an operator's own tree.

This module imports no ROS and no `launch` types, so the rules below -- path
resolution, data-source validation, topic derivation and bag verification -- are
testable without a graph. The launch files are thin wrappers over it.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path

MANIFEST_NAME = "session.yaml"

_FIND_PKG_SHARE = re.compile(r"\$\(find-pkg-share\s+([^)]+)\)")
_SESSION_DIR = re.compile(r"\$\(session-dir\)")


class SessionError(Exception):
    """A session could not be resolved, read, or validated."""


@dataclass(frozen=True)
class SessionPaths:
    """Where a session's manifest is, and the directory it anchors."""

    manifest: Path
    directory: Path


def resolve_session(spec: str) -> SessionPaths:
    """Resolve an explicit `session:=` path to its manifest and directory.

    `spec` is always a path -- absolute, or relative to the working directory.
    There is deliberately no search path: an implicit location would assume both
    where sessions live and where the user is standing, and a session may live
    anywhere. Name lookup, where it is wanted, belongs in the justfile.

    Accepts either the session directory or the manifest file itself.
    """
    candidate = Path(spec).expanduser()
    if not candidate.exists():
        raise SessionError(
            f"no session at '{candidate}'. `session:=` takes an explicit path to a "
            f"session directory or to a {MANIFEST_NAME}; there is no search path."
        )
    candidate = candidate.resolve()
    if candidate.is_dir():
        manifest = candidate / MANIFEST_NAME
        if not manifest.is_file():
            raise SessionError(
                f"'{candidate}' is a directory but contains no {MANIFEST_NAME}"
            )
        return SessionPaths(manifest=manifest, directory=candidate)
    return SessionPaths(manifest=candidate, directory=candidate.parent)


def resolve_config_path(path: str, session_dir: Path | None = None) -> str:
    """Expand `$(find-pkg-share pkg)` and `$(session-dir)` in a config path.

    `$(session-dir)` is what makes a session relocatable: a manifest referring to
    its own directory carries no absolute path into LCTK, so the directory can be
    copied to another machine and still run.
    """

    def replace_package(match: re.Match) -> str:
        package_name = match.group(1).strip()
        from ament_index_python.packages import (
            PackageNotFoundError,
            get_package_share_directory,
        )

        try:
            return get_package_share_directory(package_name)
        except (PackageNotFoundError, ValueError) as error:
            raise ValueError(
                f"Failed to find package '{package_name}': {error}"
            ) from error

    if _SESSION_DIR.search(path):
        if session_dir is None:
            raise SessionError(
                f"'{path}' uses $(session-dir), but this file was not loaded from a "
                "session directory. Use an absolute path or $(find-pkg-share ...) here."
            )
        path = _SESSION_DIR.sub(str(session_dir), path)
    return _FIND_PKG_SHARE.sub(replace_package, path)
```

- [ ] **Step 4: Run the tests and confirm they pass**

```bash
cd /home/jetson/LCTK && source /opt/ros/humble/setup.bash
PYTHONPATH="$PWD/ros/lctk_launch:$PYTHONPATH" \
  python3 -m pytest ros/lctk_launch/test/test_session.py -q --no-header
```

- [ ] **Step 5: Confirm the suite can actually fail**

Delete the `if session_dir is None:` guard so `$(session-dir)` silently expands to
`None`. Run with output redirected to a file (not a pipe) and read `$?` directly:
expect a non-zero exit and
`test_session_dir_without_a_directory_context_is_refused` failing. Restore it.

- [ ] **Step 6: Lint and commit**

```bash
cd /home/jetson/LCTK && just lint-py
git add ros/lctk_launch/lctk_launch/session.py ros/lctk_launch/test/test_session.py
git commit -m "feat(sessions): resolve a session path and its \$(session-dir)" -- \
  ros/lctk_launch/lctk_launch/session.py ros/lctk_launch/test/test_session.py
```

---

### Task 2: The `data:` section — derive, verify, state

**Files:**
- Modify: `ros/lctk_launch/lctk_launch/session.py`
- Modify: `ros/lctk_launch/test/test_session.py`

**Interfaces:**
- Consumes: `SessionError` from Task 1.
- Produces:
  - `@dataclass(frozen=True) class DataSource: kind: str; directory: Path | None; path: Path | None; lidar_model: str; lidar_rpm: float; camera_info_url: str | None`
  - `DATA_KINDS = ("pcap_avi", "bag", "live")`
  - `derived_lidar_topics(device: str) -> dict[str, str]` — keys `pointcloud`, `packets`
  - `derived_camera_topics(device: str) -> dict[str, str]` — keys `image`, `camera_info`, `namespace`
  - `parse_data(raw: object, session_dir: Path) -> DataSource`
  - `bag_topics(bag: Path) -> list[str]`
  - `verify_bag_topics(bag: Path, wanted: list[str]) -> None`

The derived convention reproduces `lidar_camera.launch.xml`'s existing defaults exactly, so
dataset 3 keeps its current topic names when its lidar device is named `top` and its camera
`front_center`.

- [ ] **Step 1: Write the failing tests**

```python
# appended to ros/lctk_launch/test/test_session.py

import yaml

from lctk_launch.session import (
    DataSource,
    bag_topics,
    derived_camera_topics,
    derived_lidar_topics,
    parse_data,
    verify_bag_topics,
)


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
    bag = write_bag(tmp_path, ["/lidar/vlp32/velodyne_points", "/lidar/falcon/iv_points"])
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
```

- [ ] **Step 2: Run and confirm they fail**

```bash
cd /home/jetson/LCTK && source /opt/ros/humble/setup.bash
PYTHONPATH="$PWD/ros/lctk_launch:$PYTHONPATH" \
  python3 -m pytest ros/lctk_launch/test/test_session.py -q --no-header
```

Expected: `ImportError: cannot import name 'parse_data'`.

- [ ] **Step 3: Implement the data section**

```python
# appended to ros/lctk_launch/lctk_launch/session.py

DATA_KINDS = ("pcap_avi", "bag", "live")

_DATA_KEYS = {"kind", "dir", "path", "lidar", "camera"}
_LIDAR_KEYS = {"model", "rpm"}
_CAMERA_KEYS = {"info_url"}


@dataclass(frozen=True)
class DataSource:
    """Where a session's data comes from."""

    kind: str
    directory: Path | None = None      # pcap_avi
    path: Path | None = None           # bag
    lidar_model: str = "vlp32c"
    lidar_rpm: float = 600.0
    camera_info_url: str | None = None


def derived_lidar_topics(device: str) -> dict[str, str]:
    """Topic names LCTK publishes for a driven lidar playback.

    These reproduce `lidar_camera.launch.xml`'s existing defaults, so the
    convention describes behaviour that already exists rather than inventing one.
    """
    return {
        "pointcloud": f"/sensing/lidar/{device}/pointcloud_raw",
        "packets": f"/sensing/lidar/{device}/velodyne_packets",
    }


def derived_camera_topics(device: str) -> dict[str, str]:
    namespace = f"/sensing/camera/{device}"
    return {
        "namespace": namespace,
        "image": f"{namespace}/image_raw",
        "camera_info": f"{namespace}/camera_info",
    }


def _reject_unknown(section: str, raw: dict, known: set[str]) -> None:
    unknown = set(raw) - known
    if unknown:
        raise SessionError(
            f"unknown key(s) in '{section}': {', '.join(sorted(unknown))}. "
            f"Known keys: {', '.join(sorted(known))}"
        )


def parse_data(raw: object, session_dir: Path) -> DataSource:
    """Validate the manifest's `data:` section into a DataSource.

    Refusals here are deliberate. Every failure this design exists to prevent is
    silent at run time -- a wrong topic yields a healthy graph and no data, a
    missing recording yields a detector that never fires -- so they are converted
    into loud refusals before the graph starts.
    """
    if raw is None:
        raise SessionError(
            "missing required 'data' section. A session must say where its data "
            f"comes from: one of {', '.join(DATA_KINDS)}."
        )
    if not isinstance(raw, dict):
        raise SessionError(f"'data' must be a mapping, got {type(raw).__name__}")
    _reject_unknown("data", raw, _DATA_KEYS)

    kind = raw.get("kind")
    if kind not in DATA_KINDS:
        raise SessionError(
            f"unknown data kind {kind!r}; expected one of {', '.join(DATA_KINDS)}"
        )

    lidar = raw.get("lidar") or {}
    _reject_unknown("data.lidar", lidar, _LIDAR_KEYS)
    camera = raw.get("camera") or {}
    _reject_unknown("data.camera", camera, _CAMERA_KEYS)

    info_url = camera.get("info_url")
    if info_url is not None:
        info_url = resolve_config_path(str(info_url), session_dir)

    common = {
        "lidar_model": str(lidar.get("model", "vlp32c")),
        "lidar_rpm": float(lidar.get("rpm", 600.0)),
        "camera_info_url": info_url,
    }

    if kind == "live":
        return DataSource(kind=kind, **common)

    if kind == "pcap_avi":
        if "dir" not in raw:
            raise SessionError("data.kind 'pcap_avi' requires 'dir'")
        directory = Path(resolve_config_path(str(raw["dir"]), session_dir))
        if not directory.is_dir():
            raise SessionError(f"data.dir does not exist: {directory}")
        for required in ("lidar.pcap", "video.avi"):
            if not (directory / required).is_file():
                raise SessionError(f"{directory} has no {required}")
        return DataSource(kind=kind, directory=directory, **common)

    if "path" not in raw:
        raise SessionError("data.kind 'bag' requires 'path'")
    bag = Path(resolve_config_path(str(raw["path"]), session_dir))
    if not bag.is_dir():
        raise SessionError(f"data.path does not exist: {bag}")
    if not (bag / "metadata.yaml").is_file():
        raise SessionError(f"{bag} has no metadata.yaml; is it a rosbag2 directory?")
    return DataSource(kind=kind, path=bag, **common)


def bag_topics(bag: Path) -> list[str]:
    """Every topic name a rosbag2 directory records."""
    import yaml

    metadata = yaml.safe_load((bag / "metadata.yaml").read_text(encoding="utf-8"))
    information = metadata["rosbag2_bagfile_information"]
    return [
        entry["topic_metadata"]["name"]
        for entry in information["topics_with_message_count"]
    ]


def verify_bag_topics(bag: Path, wanted: list[str]) -> None:
    """Refuse a manifest naming a topic the bag does not contain.

    This is M-26 caught at startup: `two_lidar.yaml` named `/velodyne_points`
    while the recording publishes `/lidar/vlp32/velodyne_points`, and the result
    was a pipeline that launched cleanly and sat silent forever. Listing what the
    bag does have turns the error into its own fix.
    """
    available = bag_topics(bag)
    missing = [topic for topic in wanted if topic not in available]
    if missing:
        raise SessionError(
            f"{bag} does not publish {', '.join(missing)}. "
            f"It records: {', '.join(available)}"
        )
```

- [ ] **Step 4: Run and confirm they pass**

- [ ] **Step 5: Confirm the suite can actually fail**

Make `verify_bag_topics` return unconditionally. Expect a non-zero exit with
`test_verify_bag_topics_refuses_and_lists_what_the_bag_has` failing. Restore it.

- [ ] **Step 6: Lint and commit**

```bash
cd /home/jetson/LCTK && just lint-py
git commit -m "feat(sessions): validate data sources; derive, verify and state topics" -- \
  ros/lctk_launch/lctk_launch/session.py ros/lctk_launch/test/test_session.py
```

---

### Task 3: Wire sessions into `config_parser.py`

**Files:**
- Modify: `ros/lctk_launch/lctk_launch/config_parser.py` (lines 36, 481, 486, 543, 545, 548, 551, 722, and the parse flow)
- Modify: `ros/lctk_launch/test/test_config_parser.py`

**Interfaces:**
- Consumes: everything from Tasks 1 and 2.
- Produces: `PipelineConfig.data: DataSource | None`; `resolve_package_path` delegating to `session.resolve_config_path`.

Behaviour:
- The parser records the manifest's directory and passes it to every path resolution.
- Under `kind: pcap_avi`, each device's topic is **derived** from its name; a stated
  `pointcloud_topic` / `image_topic` is **refused**.
- Under `bag` and `live`, a topic is **required**; under `bag` the set is verified.
- A config with no `data:` section still parses, so `calibrate.launch.py` keeps working
  against plain configs and live rigs.

- [ ] **Step 1: Write the failing tests**

```python
# appended to ros/lctk_launch/test/test_config_parser.py

import pytest

from lctk_launch.config_parser import parse_config
from lctk_launch.session import SessionError


def write_session(tmp_path, data_block, devices_block, name="rig"):
    directory = tmp_path / name
    directory.mkdir(parents=True, exist_ok=True)
    target = "$(find-pkg-share lctk_launch)/config/targets/hollow_1000_aruco_4_v1.json5"
    detector = "$(find-pkg-share lctk_launch)/config/board/hollow_1000/velodyne.json5"
    (directory / "session.yaml").write_text(
        f"""
name: {name}
{data_block}
{devices_block}
markers:
  calibration_board:
    target_config: {target}
    detector_config: {detector}
    pairs:
      - [top, front_center]
sync:
  tolerance_ms: 100
  queue_size: 100
  drop_policy: reject_new
""",
        encoding="utf-8",
    )
    return directory / "session.yaml"


def make_pcap_dir(tmp_path, name="data"):
    directory = tmp_path / name
    directory.mkdir(parents=True, exist_ok=True)
    (directory / "lidar.pcap").write_bytes(b"")
    (directory / "video.avi").write_bytes(b"")
    return directory


def test_pcap_avi_derives_device_topics(tmp_path):
    make_pcap_dir(tmp_path / "rig")
    manifest = write_session(
        tmp_path,
        "data:\n  kind: pcap_avi\n  dir: $(session-dir)/data\n",
        "devices:\n  lidars:\n    top:\n      frame_id: velodyne_top\n"
        "  cameras:\n    front_center:\n      frame_id: camera_front_center\n",
    )
    pipeline = parse_config(str(manifest))
    assert pipeline.lidars["top"].pointcloud_topic == "/sensing/lidar/top/pointcloud_raw"
    assert (
        pipeline.cameras["front_center"].image_topic
        == "/sensing/camera/front_center/image_raw"
    )


def test_pcap_avi_refuses_a_stated_topic(tmp_path):
    """Accepting one would reinstate the two-sources-of-truth bug the manifest
    exists to remove."""
    make_pcap_dir(tmp_path / "rig")
    manifest = write_session(
        tmp_path,
        "data:\n  kind: pcap_avi\n  dir: $(session-dir)/data\n",
        "devices:\n  lidars:\n    top:\n      frame_id: velodyne_top\n"
        "      pointcloud_topic: /my/topic\n"
        "  cameras:\n    front_center:\n      frame_id: camera_front_center\n",
    )
    with pytest.raises((ValueError, SessionError), match="derived"):
        parse_config(str(manifest))


def test_live_requires_a_stated_topic(tmp_path):
    manifest = write_session(
        tmp_path,
        "data:\n  kind: live\n",
        "devices:\n  lidars:\n    top:\n      frame_id: velodyne_top\n"
        "  cameras:\n    front_center:\n      frame_id: camera_front_center\n",
    )
    with pytest.raises((ValueError, SessionError), match="pointcloud_topic"):
        parse_config(str(manifest))


def test_session_dir_resolves_in_a_marker_path(tmp_path):
    make_pcap_dir(tmp_path / "rig")
    bbox = tmp_path / "rig" / "bbox.json5"
    bbox.write_text("{}", encoding="utf-8")
    manifest = write_session(
        tmp_path,
        "data:\n  kind: pcap_avi\n  dir: $(session-dir)/data\n",
        "devices:\n  lidars:\n    top:\n      frame_id: velodyne_top\n"
        "  cameras:\n    front_center:\n      frame_id: camera_front_center\n",
    )
    text = manifest.read_text(encoding="utf-8").replace(
        "    pairs:", "    bbox_config: $(session-dir)/bbox.json5\n    pairs:"
    )
    manifest.write_text(text, encoding="utf-8")
    pipeline = parse_config(str(manifest))
    assert pipeline.lidar_board_detectors[0].bbox_config == str(bbox)


def test_a_config_without_a_data_section_still_parses(tmp_path):
    """calibrate.launch.py must keep working against plain configs and live rigs."""
    manifest = write_session(
        tmp_path,
        "",
        "devices:\n  lidars:\n    top:\n      frame_id: velodyne_top\n"
        "      pointcloud_topic: /points\n"
        "  cameras:\n    front_center:\n      frame_id: camera_front_center\n"
        "      image_topic: /image\n",
    )
    pipeline = parse_config(str(manifest))
    assert pipeline.data is None
    assert pipeline.lidars["top"].pointcloud_topic == "/points"
```

- [ ] **Step 2: Run and confirm they fail**

```bash
cd /home/jetson/LCTK && source /opt/ros/humble/setup.bash
PYTHONPATH="$PWD/ros/lctk_launch:$PYTHONPATH" \
  python3 -m pytest ros/lctk_launch/test/test_config_parser.py -q --no-header
```

- [ ] **Step 3: Delegate path resolution and record the session directory**

Replace the body of `resolve_package_path` (`config_parser.py:36`) so there is one
implementation, and thread the session directory through:

```python
from lctk_launch.session import (
    DataSource,
    SessionError,
    derived_camera_topics,
    derived_lidar_topics,
    parse_data,
    resolve_config_path,
    verify_bag_topics,
)


def resolve_package_path(path: str, session_dir=None) -> str:
    """Kept for its existing call sites; the rules live in session.py."""
    return resolve_config_path(path, session_dir)
```

In `CalibrationConfigParser.__init__`, record `self._session_dir = Path(config_path).resolve().parent`
and `self._data: DataSource | None = None`. Pass `self._session_dir` as the second
argument at every `resolve_package_path(...)` call site — lines 481, 486, 543, 545, 548,
551 and 722.

Add `data: DataSource | None = None` to `PipelineConfig`, parse it next to
`self._parse_sync(...)`:

```python
        raw_data = raw_config.get("data")
        if raw_data is not None:
            self._data = parse_data(raw_data, self._session_dir)
```

and pass `data=self._data` in `_derive_pipeline`.

- [ ] **Step 4: Apply the topic rules**

Where devices are parsed, branch on the data kind:

```python
    def _device_topic(self, kind_of: str, name: str, stated: str | None) -> str:
        """Derive, or require, a device's topic according to the data source.

        What is knowable differs by source, so the rule does. Under `pcap_avi`
        LCTK drives the playback, so one source feeds both the player and the
        calibration graph and a mismatch becomes unrepresentable. Under `bag` and
        `live` the topic is a fact about the recording or the rig, so it is stated.
        """
        derived = (
            derived_lidar_topics(name)["pointcloud"]
            if kind_of == "lidar"
            else derived_camera_topics(name)["image"]
        )
        key = "pointcloud_topic" if kind_of == "lidar" else "image_topic"
        if self._data is not None and self._data.kind == "pcap_avi":
            if stated is not None:
                raise SessionError(
                    f"device '{name}' states {key}, but under data.kind 'pcap_avi' the "
                    f"topic is derived ({derived}). Stating it would reintroduce the "
                    "two-sources-of-truth mismatch the manifest exists to remove."
                )
            return derived
        if stated is None:
            raise ValueError(f"device '{name}' requires {key}")
        return stated
```

After devices are parsed, verify bag topics:

```python
        if self._data is not None and self._data.kind == "bag":
            wanted = [lidar.pointcloud_topic for lidar in self.lidars.values()]
            wanted += [camera.image_topic for camera in self.cameras.values()]
            verify_bag_topics(self._data.path, wanted)
```

- [ ] **Step 5: Run the whole launch suite**

```bash
cd /home/jetson/LCTK && source /opt/ros/humble/setup.bash
PYTHONPATH="$PWD/ros/lctk_launch:$PYTHONPATH" \
  python3 -m pytest ros/lctk_launch/test/ -q --no-header
```

Expected: all pass, including the pre-existing tests — this task must not change how a
plain config parses.

- [ ] **Step 6: Confirm the suite can actually fail**

Make `_device_topic` always return `derived`. Expect a non-zero exit with
`test_live_requires_a_stated_topic` failing. Restore it.

- [ ] **Step 7: Lint and commit**

```bash
cd /home/jetson/LCTK && just lint-py
git commit -m "feat(sessions): read data: and \$(session-dir) in the config parser" -- \
  ros/lctk_launch/lctk_launch/config_parser.py ros/lctk_launch/test/test_config_parser.py
```

---

### Task 4: `session_data.launch.py`

**Files:**
- Create: `ros/lctk_launch/launch/session_data.launch.py`
- Test: `ros/lctk_launch/test/test_session_launch.py`

**Interfaces:**
- Consumes: `resolve_session`, `parse_data`, `derived_lidar_topics`, `derived_camera_topics`.
- Produces: a launch file taking `session:=<path>`, generating the data source for a manifest.

For `pcap_avi` it includes the existing `lidar_camera.launch.xml` with derived arguments —
reuse, not reimplementation. For `bag` it runs `ros2 bag play`. For `live` it generates
nothing and says so.

- [ ] **Step 1: Write the failing test**

```python
"""Graph contracts for the session launch files."""

from __future__ import annotations

import importlib.util
from pathlib import Path
from types import ModuleType

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DATA_LAUNCH = PACKAGE_ROOT / "launch" / "session_data.launch.py"


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
```

- [ ] **Step 2: Run and confirm it fails** (`session_data.launch.py` does not exist).

- [ ] **Step 3: Implement `session_data.launch.py`**

```python
"""Start the data source a session manifest describes.

Thin by design: the rules live in `lctk_launch.session`, and for `pcap_avi` this
includes the existing `lidar_camera.launch.xml` rather than reimplementing the
velodyne driver and gscam wiring. The topics it passes are the same derived
values `config_parser` gives the calibration graph, which is the whole point --
one source, so the two halves cannot disagree.
"""

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from lctk_launch.session import (
    derived_camera_topics,
    derived_lidar_topics,
    parse_data,
    resolve_session,
)


def generate_data_source(context, *args, **kwargs) -> list:
    session = resolve_session(LaunchConfiguration("session").perform(context))
    manifest = yaml.safe_load(session.manifest.read_text(encoding="utf-8"))
    source = parse_data(manifest.get("data"), session.directory)

    devices = manifest.get("devices") or {}
    lidars = list((devices.get("lidars") or {}).items())
    cameras = list((devices.get("cameras") or {}).items())

    if source.kind == "live":
        return [
            LogInfo(msg=f"  session '{session.directory.name}': live sensors, "
                        "no playback started")
        ]

    if source.kind == "bag":
        return [
            LogInfo(msg=f"  session '{session.directory.name}': playing {source.path}"),
            ExecuteProcess(
                cmd=["ros2", "bag", "play", str(source.path), "--clock"],
                output="screen",
            ),
        ]

    # pcap_avi: one lidar and one camera, played by lctk_sample_data.
    if len(lidars) != 1 or len(cameras) != 1:
        raise RuntimeError(
            f"data.kind 'pcap_avi' plays exactly one lidar and one camera; this "
            f"session declares {len(lidars)} lidar(s) and {len(cameras)} camera(s)"
        )
    lidar_name, lidar_config = lidars[0]
    camera_name, camera_config = cameras[0]
    lidar_topics = derived_lidar_topics(lidar_name)
    camera_topics = derived_camera_topics(camera_name)

    info_url = source.camera_info_url
    if info_url and not info_url.startswith("file://"):
        info_url = f"file://{info_url}"

    return [
        LogInfo(msg=f"  session '{session.directory.name}': playing {source.directory}"),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("lctk_sample_data"),
                        "launch",
                        "lidar_camera.launch.xml",
                    ]
                )
            ),
            launch_arguments={
                "pcap_file": str(source.directory / "lidar.pcap"),
                "video_file": str(source.directory / "video.avi"),
                "pointcloud_topic": lidar_topics["pointcloud"],
                "velodyne_packets_topic": lidar_topics["packets"],
                "lidar_frame_id": lidar_config["frame_id"],
                "rpm": str(source.lidar_rpm),
                "camera_name": camera_name,
                "camera_namespace": camera_topics["namespace"],
                "camera_frame_id": camera_config["frame_id"],
                **({"camera_info_url": info_url} if info_url else {}),
            }.items(),
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "session",
                description=(
                    "Explicit path to a session directory or its session.yaml. "
                    "There is no search path: a session may live anywhere."
                ),
            ),
            OpaqueFunction(function=generate_data_source),
        ]
    )
```

- [ ] **Step 4: Run and confirm the tests pass**

- [ ] **Step 5: Confirm the suite can actually fail**

Change `derived_lidar_topics(lidar_name)["pointcloud"]` to a literal `"/points"`. Expect a
non-zero exit with `test_pcap_avi_includes_the_playback_launch_with_derived_topics`
failing. Restore it.

- [ ] **Step 6: Lint and commit**

```bash
cd /home/jetson/LCTK && just lint-py
git commit -m "feat(sessions): start a session's data source" -- \
  ros/lctk_launch/launch/session_data.launch.py ros/lctk_launch/test/test_session_launch.py
```

---

### Task 5: `session.launch.py`, and retire `demo.launch.py`

**Files:**
- Create: `ros/lctk_launch/launch/session.launch.py`
- Delete: `ros/lctk_launch/launch/demo.launch.py`
- Modify: `ros/lctk_launch/test/test_session_launch.py`

**Interfaces:**
- Consumes: `session_data.launch.py` and `calibrate.launch.py`.
- Produces: the end-to-end entry point, `session:=<path>` plus every existing calibrate argument.

- [ ] **Step 1: Write the failing test**

```python
# appended to ros/lctk_launch/test/test_session_launch.py

SESSION_LAUNCH = PACKAGE_ROOT / "launch" / "session.launch.py"


def test_session_launch_declares_session_and_the_calibrate_arguments():
    module = load(SESSION_LAUNCH, "session_launch")
    description = module.generate_launch_description()
    names = {
        action.name
        for action in description.entities
        if hasattr(action, "name") and action.name
    }
    assert "session" in names
    for expected in ("solver_mode", "mode", "enable_rviz", "enable_overlay", "log_level"):
        assert expected in names, f"{expected} must still be settable end to end"


def test_session_launch_feeds_the_same_manifest_to_both_halves(tmp_path):
    """The data source and the calibration graph must read one file.

    If they can be pointed at different files the design's guarantee is gone --
    that is the two-sources-of-truth bug this whole change exists to remove.
    """
    module = load(SESSION_LAUNCH, "session_launch2")
    directory = make_pcap_session(tmp_path)
    includes = module.generate_session(_Context(directory))
    argument_sets = [
        {k: v for k, v in action.launch_arguments}
        for action in includes
        if hasattr(action, "launch_arguments")
    ]
    sessions = {a["session"] for a in argument_sets if "session" in a}
    configs = {a["config_file"] for a in argument_sets if "config_file" in a}
    assert sessions == {str(directory)}
    assert configs == {str(directory / "session.yaml")}
```

- [ ] **Step 2: Run and confirm it fails.**

- [ ] **Step 3: Implement `session.launch.py`**

```python
"""Run a whole calibration session: its data source, then the calibration graph.

Generalises the deleted `demo.launch.py`, which hard-coded dataset 3 and
`sample_data.yaml` in its body and took no argument to change either.

Both halves are handed the *same* manifest -- the data launch by directory, the
calibration launch by file. That is the design's one guarantee: the topics the
player publishes and the topics the graph subscribes to come from one source.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from lctk_launch.session import resolve_session

_FORWARDED = (
    ("debug_mode", "false", "Enable debug topics"),
    ("log_level", "info", "ROS log level"),
    ("mode", "offline", "Transport QoS: 'offline' (RELIABLE) or 'realtime' (BEST_EFFORT)"),
    ("enable_rviz", "true", "Launch RViz alongside the pipeline"),
    ("solver_mode", "continuous", "'continuous', 'manual' or 'assisted'"),
    ("enable_overlay", "false", "Launch pointcloud_image_overlay"),
    ("enable_judge", "false", "Launch the calibration quality judge"),
)


def _share(*parts):
    return PathJoinSubstitution([FindPackageShare("lctk_launch"), *parts])


def generate_session(context, *args, **kwargs) -> list:
    session = resolve_session(LaunchConfiguration("session").perform(context))
    forwarded = {
        name: LaunchConfiguration(name).perform(context) for name, _, _ in _FORWARDED
    }
    return [
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(_share("launch", "session_data.launch.py")),
            launch_arguments={"session": str(session.directory)}.items(),
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(_share("launch", "calibrate.launch.py")),
            launch_arguments={
                "config_file": str(session.manifest),
                **forwarded,
            }.items(),
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    arguments = [
        DeclareLaunchArgument(
            "session",
            description=(
                "Explicit path to a session directory or its session.yaml. "
                "There is no search path: a session may live anywhere."
            ),
        )
    ]
    arguments += [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, default, description in _FORWARDED
    ]
    return LaunchDescription([*arguments, OpaqueFunction(function=generate_session)])
```

- [ ] **Step 4: Delete `demo.launch.py`**

```bash
cd /home/jetson/LCTK && git rm ros/lctk_launch/launch/demo.launch.py
```

- [ ] **Step 5: Run the whole launch suite and confirm it passes**

- [ ] **Step 6: Confirm the suite can actually fail**

Change the `config_file` argument to a literal path. Expect a non-zero exit with
`test_session_launch_feeds_the_same_manifest_to_both_halves` failing. Restore it.

- [ ] **Step 7: Lint and commit**

```bash
cd /home/jetson/LCTK && just lint-py
git commit -m "feat(sessions): run a session end to end; retire demo.launch.py" -- \
  ros/lctk_launch/launch ros/lctk_launch/test/test_session_launch.py
```

---

### Task 6: The `lctk_session` console script

**Files:**
- Create: `ros/lctk_launch/lctk_launch/session_cli.py`
- Modify: `ros/lctk_launch/setup.py` (add to `console_scripts`)
- Test: `ros/lctk_launch/test/test_session_cli.py`

**Interfaces:**
- Consumes: Tasks 1–3.
- Produces: `main(argv: list[str] | None = None) -> int` with subcommands `list`, `check`, `new`.

- [ ] **Step 1: Write the failing tests**

```python
"""The lctk_session tool: list, check, new.

`check` is the piece that earns its keep -- it answers "why is nothing being
detected" before the run rather than after, without starting a graph.
"""

from pathlib import Path

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
```

- [ ] **Step 2: Run and confirm they fail.**

- [ ] **Step 3: Implement `session_cli.py`**

```python
"""`ros2 run lctk_launch lctk_session` -- list, check and scaffold sessions.

Everything here is available without `just`; the justfile only adds name lookup
and this repo's usual defaults.
"""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

import yaml

from lctk_launch.session import (
    MANIFEST_NAME,
    SessionError,
    bag_topics,
    parse_data,
    resolve_session,
)


def _check(spec: str) -> int:
    """Resolve everything a run needs and report it, without starting a graph."""
    try:
        session = resolve_session(spec)
        manifest = yaml.safe_load(session.manifest.read_text(encoding="utf-8"))
        source = parse_data(manifest.get("data"), session.directory)

        from lctk_launch.config_parser import parse_config

        pipeline = parse_config(str(session.manifest))
    except (SessionError, ValueError, OSError) as error:
        print(f"FAIL {spec}: {error}", file=sys.stderr)
        return 1

    print(f"session:  {session.directory}")
    print(f"manifest: {session.manifest}")
    print(f"data:     {source.kind} {source.directory or source.path or '(live)'}")
    for name, lidar in pipeline.lidars.items():
        print(f"  lidar  {name}: {lidar.pointcloud_topic}  frame={lidar.frame_id}")
    for name, camera in pipeline.cameras.items():
        print(f"  camera {name}: {camera.image_topic}  frame={camera.frame_id}")
    if source.kind == "bag":
        print(f"  bag records: {', '.join(bag_topics(source.path))}")
    print("OK")
    return 0


def _list(directories: list[str]) -> int:
    roots = [Path(d) for d in directories] or _default_roots()
    for root in roots:
        if not root.is_dir():
            continue
        for child in sorted(root.iterdir()):
            if (child / MANIFEST_NAME).is_file():
                print(f"{child.name:40s} {child}")
    return 0


def _default_roots() -> list[Path]:
    roots = [Path.cwd() / "sessions"]
    try:
        from ament_index_python.packages import get_package_share_directory

        roots.append(Path(get_package_share_directory("lctk_launch")) / "sessions")
    except Exception:  # noqa: BLE001 - discovery helper; an unbuilt workspace is not an error
        pass
    return roots


def _new(target: str, template: str) -> int:
    destination = Path(target).expanduser()
    if destination.exists():
        print(f"{destination} already exists", file=sys.stderr)
        return 1
    try:
        source = resolve_session(template)
    except SessionError as error:
        print(str(error), file=sys.stderr)
        return 1
    shutil.copytree(source.directory, destination, ignore=shutil.ignore_patterns("out"))
    (destination / "out").mkdir(exist_ok=True)
    print(f"created {destination} from {source.directory}")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="lctk_session")
    sub = parser.add_subparsers(dest="command", required=True)

    check = sub.add_parser("check", help="validate a session without launching it")
    check.add_argument("session")

    listing = sub.add_parser("list", help="list sessions in the given directories")
    listing.add_argument("directories", nargs="*")

    new = sub.add_parser("new", help="scaffold a session from an existing one")
    new.add_argument("target")
    new.add_argument("--from", dest="template", required=True)

    args = parser.parse_args(argv)
    if args.command == "check":
        return _check(args.session)
    if args.command == "list":
        return _list(args.directories)
    return _new(args.target, args.template)


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 4: Register the console script**

In `ros/lctk_launch/setup.py`, beside the existing entry:

```python
        "console_scripts": [
            "tf_tree_broadcaster = lctk_launch.tf_tree_broadcaster:main",
            "lctk_session = lctk_launch.session_cli:main",
        ],
```

- [ ] **Step 5: Run the tests, then confirm the built script works**

```bash
cd /home/jetson/LCTK && just build
source install/setup.bash
ros2 run lctk_launch lctk_session list
```

- [ ] **Step 6: Confirm the suite can actually fail**

Make `_check` return `0` unconditionally. Expect a non-zero exit with
`test_check_fails_when_the_data_is_missing` failing. Restore it.

- [ ] **Step 7: Lint and commit**

```bash
cd /home/jetson/LCTK && just lint-py
git commit -m "feat(sessions): add the lctk_session list/check/new tool" -- \
  ros/lctk_launch/lctk_launch/session_cli.py ros/lctk_launch/test/test_session_cli.py \
  ros/lctk_launch/setup.py
```

---

### Task 7: Ship `sessions/`, and the justfile aliases

**Files:**
- Modify: `ros/lctk_launch/setup.py` (install `sessions/` like `config/`)
- Modify: `justfile`
- Modify: `.gitignore`
- Create: `sessions/README.md`

**Interfaces:**
- Consumes: Tasks 4–6.
- Produces: `just run|check|sessions|new`, with `just demo` as an alias.

The shipped sessions live at the repo root in `sessions/`; `experiments/` is already taken by
the `board-detection-2d` benchmark project.

- [ ] **Step 1: Install `sessions/` into the package share**

`setup.py` already walks `config/`; add the same walk for the repo-root `sessions/`
directory, installing to `share/lctk_launch/sessions/…` and skipping any `out/` directory.

- [ ] **Step 2: Ignore session outputs**

In `.gitignore`, beside the other generated-output entries:

```
# Per-session run outputs: detections archives, exports, logs.
/sessions/*/out/
```

- [ ] **Step 3: Add the justfile recipes**

```make
# List available sessions
sessions:
    #!/usr/bin/env bash
    set -eo pipefail
    source install/setup.bash
    ros2 run lctk_launch lctk_session list

# Validate a session without launching it: just check <path-or-name>
check SESSION:
    #!/usr/bin/env bash
    set -eo pipefail
    source install/setup.bash
    ros2 run lctk_launch lctk_session check "$(just _session-path {{ SESSION }})"

# Scaffold a new session: just new <path> [FROM=<path-or-name>]
new TARGET FROM='sample3-hollow-velodyne':
    #!/usr/bin/env bash
    set -eo pipefail
    source install/setup.bash
    ros2 run lctk_launch lctk_session new {{ TARGET }} \
        --from "$(just _session-path {{ FROM }})"

# Run a session end to end: just run <path-or-name>
run SESSION:
    #!/usr/bin/env bash
    set -eo pipefail
    source install/setup.bash
    play_launch launch \
        --web-addr 0.0.0.0:8000 \
        lctk_launch session.launch.py \
        session:="$(just _session-path {{ SESSION }})" \
        debug_mode:={{ debug_mode }} \
        log_level:={{ log_level }} \
        mode:={{ mode }} \
        enable_rviz:={{ rviz_enabled }} \
        solver_mode:={{ solver_mode }} \
        enable_overlay:={{ enable_overlay }} \
        enable_judge:={{ enable_judge }}

# Run the shipped sample-data session end to end
demo:
    #!/usr/bin/env bash
    set -eo pipefail
    just run sample3-hollow-velodyne

# Resolve a session name to a path. Name lookup lives HERE, in the alias layer --
# the ros2 interface takes an explicit path and makes no assumption about where
# sessions live or where the user is standing.
_session-path SESSION:
    #!/usr/bin/env bash
    set -eo pipefail
    if [[ -e "{{ SESSION }}" ]]; then
        realpath "{{ SESSION }}"
    elif [[ -e "sessions/{{ SESSION }}" ]]; then
        realpath "sessions/{{ SESSION }}"
    else
        source install/setup.bash
        SHARE=$(ros2 pkg prefix lctk_launch --share)
        if [[ -e "$SHARE/sessions/{{ SESSION }}" ]]; then
            echo "$SHARE/sessions/{{ SESSION }}"
        else
            echo "no session '{{ SESSION }}' as a path, in ./sessions/, or in $SHARE/sessions/" >&2
            exit 1
        fi
    fi
```

- [ ] **Step 4: Write `sessions/README.md`**

Explain what a session is, the directory contents, that `session:=` is always an explicit
path so a session may live anywhere, and how to make one with
`ros2 run lctk_launch lctk_session new`.

- [ ] **Step 5: Build and check the recipes resolve**

```bash
cd /home/jetson/LCTK && just build
just sessions
just --list | grep -E "run|check|sessions|new|demo"
```

- [ ] **Step 6: Commit**

```bash
git commit -m "feat(sessions): ship sessions/ and add the justfile aliases" -- \
  ros/lctk_launch/setup.py justfile .gitignore sessions/README.md
```

---

### Task 8: Migrate the six examples

**Files:**
- Create: six directories under `sessions/`
- Delete: `ros/lctk_launch/config/examples/`, and the orphaned crop boxes under `config/board/`
- Modify: `ros/lctk_launch/test/test_calibrate_launch_graph.py`, `ros/lctk_launch/test/test_target_presets.py` (they read `CONFIG_ROOT`)
- Test: `ros/lctk_launch/test/test_sessions_shipped.py`

**Interfaces:**
- Consumes: Tasks 1–7.

| example | session | data kind |
|---|---|---|
| `sample_data.yaml` | `sample3-hollow-velodyne` | `pcap_avi`, `lctk_sample_data/data/3` |
| `seyond_left.yaml` | `seyond-left` | `live` |
| `seyond_right.yaml` | `seyond-right` | `live` |
| `solid_600_handheld.yaml` | `solid600-handheld-zed` | `live` |
| `two_lidar.yaml` | `twolidar-vlp32-falcon` | `bag`, `TWO_LIDAR_1` |
| `vehicle.yaml` | `vehicle-multisensor` | `live` |

Session-local files move with their session: `config/judge/ground_truth_config.yaml` and
`ground_truth_transform.txt` describe one rig, and `config/rviz/*.rviz` are per-experiment
layouts. Each goes to the session that uses it, as `judge_ground_truth.yaml` and `rviz.rviz`;
`config/rviz/calibration.rviz` stays as the shared default for a session that ships none.

Two things change meaning during migration and must be done deliberately:

- **`sample3-hollow-velodyne` renames its lidar device `top_lidar` → `top`**, so the derived
  topics reproduce today's `/sensing/lidar/top/pointcloud_raw` exactly. `config/board/sample_data_bbox.json5`
  moves into the session as `bbox.json5`, and `config/camera/front_center_camera_info.yaml`
  as `camera_info.yaml`.
- **`twolidar-vlp32-falcon` gets the topics the bag actually records** —
  `/lidar/vlp32/velodyne_points` and `/lidar/falcon/iv_points`, verified live against
  `TWO_LIDAR_1/metadata.yaml`. The old `/velodyne_points` and `/iv_points` are what M-26
  filed; `verify_bag_topics` now refuses them.

- [ ] **Step 1: Write the failing test**

```python
"""Every shipped session parses, and the sample-data one keeps its topics."""

from pathlib import Path

import pytest

from lctk_launch.config_parser import parse_config
from lctk_launch.session import MANIFEST_NAME

SESSIONS = Path(__file__).resolve().parents[3] / "sessions"

NAMES = [
    "sample3-hollow-velodyne",
    "seyond-left",
    "seyond-right",
    "solid600-handheld-zed",
    "twolidar-vlp32-falcon",
    "vehicle-multisensor",
]


@pytest.mark.parametrize("name", NAMES)
def test_every_shipped_session_parses(name):
    parse_config(str(SESSIONS / name / MANIFEST_NAME))


def test_the_sample_session_keeps_the_topics_the_playback_publishes():
    """The migration must not move dataset 3's topics; the playback defaults
    and the calibration graph have to keep meeting at the same names."""
    pipeline = parse_config(str(SESSIONS / "sample3-hollow-velodyne" / MANIFEST_NAME))
    assert pipeline.lidars["top"].pointcloud_topic == "/sensing/lidar/top/pointcloud_raw"
    assert (
        pipeline.cameras["front_center"].image_topic
        == "/sensing/camera/front_center/image_raw"
    )


def test_the_two_lidar_session_names_the_topics_the_bag_records():
    """M-26: the old config named /velodyne_points; the bag records
    /lidar/vlp32/velodyne_points. Parsing verifies against metadata.yaml, so this
    test failing means the manifest is wrong, not the test."""
    pipeline = parse_config(str(SESSIONS / "twolidar-vlp32-falcon" / MANIFEST_NAME))
    topics = {lidar.pointcloud_topic for lidar in pipeline.lidars.values()}
    assert topics == {"/lidar/vlp32/velodyne_points", "/lidar/falcon/iv_points"}
```

`twolidar-vlp32-falcon` needs the gitignored bag present. Guard that test with
`pytest.mark.skipif(not bag_path.exists())` and say so in the skip reason, following the
repo's existing treatment of `bags/`.

- [ ] **Step 2: Run and confirm it fails.**

- [ ] **Step 3: Create the six sessions**

For each, write `session.yaml` carrying the old file's `markers:`, `sync:` and `pairs:`
verbatim, plus the new `data:` section, and move its session-local files in. Give each a
`README.md` saying what the recording is and whether the data ships.

- [ ] **Step 4: Delete `config/examples/` and the orphaned crop boxes**

```bash
cd /home/jetson/LCTK
git rm -r ros/lctk_launch/config/examples
git rm ros/lctk_launch/config/board/bbox.json5 \
       ros/lctk_launch/config/board/bbox_v1.json5 \
       ros/lctk_launch/config/board/bbox-seyond.json5 \
       ros/lctk_launch/config/board/bbox-vlp.json5 \
       ros/lctk_launch/config/board/bbox_2_lidar_seyond.json5 \
       ros/lctk_launch/config/board/bbox_2_lidar_vlp32.json5 \
       ros/lctk_launch/config/board/sample_data_bbox.json5
```

Before deleting each crop box, `grep -rn` its name across the tree and move it into the
session that uses it. Anything with no user is deleted rather than kept as a mystery file —
that ambiguity is what M-29 was made of. The `board-detection-2d` experiment references
`config/board/bbox.json5` as its pcap reference; repoint it at
`sessions/sample3-hollow-velodyne/bbox.json5`.

- [ ] **Step 5: Repoint the tests that read `config/examples/`**

`test_calibrate_launch_graph.py` and `test_target_presets.py` use `CONFIG_ROOT`; change it
to the `sessions/` tree and the manifest filename.

- [ ] **Step 6: Run the full suite**

```bash
cd /home/jetson/LCTK && just test
```

- [ ] **Step 7: Commit**

```bash
git commit -m "refactor(sessions): migrate the six examples; close M-26 and M-27"
```

---

### Task 9: Documentation

**Files:**
- Create: `book/src/user-guide/sessions.md`
- Modify: `book/src/SUMMARY.md`, `CLAUDE.md`, `README.md`, `ros/lctk_launch/README.md`, `ros/lctk_launch/config/README.md`
- Modify: `docs/issues/README.md`, `docs/issues/M-26-*.md`, `docs/issues/M-27-*.md` (close them)
- Move: both issue files to `docs/issues/archive/`

- [ ] **Step 1: Write the user guide**

Lead with the `ros2 launch` and `ros2 run` commands; show the `just` shorthand second. Cover:
what a session is, the directory contents, the manifest with every key, the three data kinds
and why the topic rule differs between them, `session:=` always being an explicit path,
preparing a new experiment, and where outputs land.

- [ ] **Step 2: Update `SUMMARY.md`, `CLAUDE.md` and the READMEs**

Every place that names `config/examples/…` or `just demo`'s pinned dataset changes. In
`CLAUDE.md`, the "Config-Driven Calibration" section gains the `data:` section and
`$(session-dir)`, and the example-config list becomes the session list.

- [ ] **Step 3: Close M-26 and M-27**

Both are fixed by construction now — M-26's topics are verified against the bag, M-27's
placeholder topics are replaced by a declared source. Add a resolution section to each,
move to `docs/issues/archive/`, repair the links that cross the move, and update the tracker
table.

- [ ] **Step 4: Verify docs**

```bash
cd /home/jetson/LCTK
python3 setup/scripts/check-doc-links.py
cd book && just build
```

- [ ] **Step 5: Commit**

```bash
git commit -m "docs(sessions): document sessions; close M-26 and M-27"
```

---

### Task 10: Full verification

- [ ] **Step 1: Build, test, lint**

```bash
cd /home/jetson/LCTK
just build
just test
just lint
python3 setup/scripts/check-doc-links.py
```

Record the before/after test counts explicitly rather than assuming.

- [ ] **Step 2: Run the demo end to end and confirm detections actually flow**

```bash
just demo
```

In another terminal, confirm the pipeline is alive rather than merely launched:

```bash
source install/setup.bash
ros2 topic hz /calibration/top_front_center/extrinsic_transform
ros2 topic echo --once /calibration/top_calibration_board/calibration_board_detections
```

Expected: a non-empty `detections:` array and a published transform. An empty array means
the pipeline launched and is doing nothing — the exact silent failure M-29 recorded, and
the reason this step asserts on data rather than on the graph.

- [ ] **Step 3: Run the same session with plain ROS 2, from a different directory**

```bash
cd /tmp && source /home/jetson/LCTK/install/setup.bash
ros2 launch lctk_launch session.launch.py \
    session:=$(ros2 pkg prefix lctk_launch --share)/sessions/sample3-hollow-velodyne \
    enable_rviz:=false
```

This is the claim that sessions work without `just` and without standing in the repo.

- [ ] **Step 4: Run a session from outside the repo**

```bash
ros2 run lctk_launch lctk_session new ~/calib/scratch --from \
    $(ros2 pkg prefix lctk_launch --share)/sessions/sample3-hollow-velodyne
ros2 run lctk_launch lctk_session check ~/calib/scratch
ros2 launch lctk_launch session.launch.py session:=~/calib/scratch enable_rviz:=false
```

This is the claim that a session directory is relocatable. If `$(session-dir)` is wrong
anywhere, this is what catches it.

- [ ] **Step 5: Confirm `check` catches a broken session**

Point the copy's `data.dir` at a path that does not exist and confirm
`lctk_session check` exits non-zero and names the resolved absolute path.

- [ ] **Step 6: Commit any fixes and push**

```bash
git add -A && git commit -m "fix(sessions): address full-verification findings"
git push origin feat/selectable-calibration-targets
```
