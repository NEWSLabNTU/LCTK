# TWO_LIDAR Bag Dataset Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the four `TWO_LIDAR_*` ROS 2 bags usable as `boarddet` benchmark datasets, so Method E and the rest of the phase-7 detector can be measured on a second rig and — for the first time — on a real solid-state LiDAR.

**Architecture:** A ROS-side export tool converts each bag topic into the same `.npz` cache format `ingest.py` already produces for the pcaps, bridging the ROS 2 Humble (Python 3.10) and `boarddet` (Python 3.11 `uv` venv) environments without coupling them. `boarddet` gains a bag frame loader, a rotation-aware bbox-reference loader that replaces the LOO harness's hardcoded reference box, and a generalized harness that takes an arbitrary set of named frame sources plus a sensor and a bbox path.

**Tech Stack:** ROS 2 Humble (`rosbag2_py`, `rclpy`, Python 3.10, system) for export only; `boarddet` (Python 3.11, `uv`, numpy/open3d/scipy) for everything else.

## Context

`experiments/board-detection-2d` (`boarddet`) is the phase-7 crop-box-free board detector. Its
current best result is Method E's background subtraction at **88.4% recall / 100% precision**
across the five sample pcaps
([side-track_method-e-background-subtraction.md](../../roadmap/side-track_method-e-background-subtraction.md)),
achieved by leave-one-out cross-dataset background construction: hold out one capture, build a
consensus background from the others, and the held-out board is the one thing that background has
never seen.

Everything measured so far comes from one rig and one sensor. The new bags change both:

| | existing pcaps | new bags |
|---|---|---|
| count | 5 | 4 (`TWO_LIDAR_1`..`4`) |
| sensors | VLP-32C only | VLP-32C **and** Innovusion/Seyond Falcon |
| format | raw pcap, decoded by `velodyne_decoder` | ROS 2 bag (sqlite3), `sensor_msgs/PointCloud2` |
| frames | 103–113 per dataset | ~199 per sensor per bag |
| rig | one shared room, `bbox.json5` reference | different rig, reference **to be supplied** |

Confirmed bag contents (`ros2 bag info`, and a `PointCloud2` field dump):

- `/lidar/vlp32/velodyne_points` — `frame_id: velodyne`, ~51,429 pts/frame, `point_step` 32,
  fields `x,y,z` (float32 @ 0,4,8), `intensity` (uint8 @ 12), `return_type` (uint8 @ 13),
  `channel` (uint16 @ 14), plus `azimuth`/`elevation`/`distance`/`time_stamp`.
- `/lidar/falcon/iv_points` — `frame_id: seyond`, ~92,322 pts/frame, `point_step` 16, fields
  `x,y,z` (float32 @ 0,4,8), `intensity` (uint8 @ 12), `return_type` (uint8 @ 13),
  `channel` (uint16 @ 14).

The first 16 bytes are laid out identically for both sensors, so one parser serves both.

**The board is static within every bag** (confirmed by the user), exactly like the pcap captures —
so cross-bag LOO is the right harness and no within-session multi-pose claim is unlocked by this
data. Whether the board *moves between* bags is **not yet confirmed** and is the single biggest
risk here: if all four bags place the board identically, consensus background subtraction absorbs
it and cross-bag LOO returns 0%. Task 6 measures this before any benchmark is run.

## Global Constraints

- All `boarddet` work runs through `uv` from `experiments/board-detection-2d/`: `uv run pytest`, `uv run python -m boarddet.<module>`.
- The export tool is the **only** code that may import ROS. It runs under system Python 3.10 with `/opt/ros/humble/setup.bash` sourced, and is never imported by `boarddet` or its tests.
- **Never `pip3 install --user` numpy, scipy, or setuptools** (CLAUDE.md Known Issue 3). Dependencies go in via `uv add`.
- Bag data is **not** committed to git. 2.4 GB of `.db3` plus 1.9 GB of `.zip` stays local.
- Existing behavior must stay byte-identical when new code paths are unused: the five pcap datasets, generators a/b/c/e, and every current test keep working unchanged.
- Exported `.npz` files use the **existing** `ingest.py` cache schema — `stamps` plus per-frame `xyz_{i}`, `intensity_{i}`, `ring_{i}` — so both sources produce identical `Frame` objects downstream.
- `intensity` and `ring`/`channel` are **diagnostics only**. Algorithm code must never read them (solid-state compatibility; `ingest.py` module docstring).
- The board is static within each bag. Do not build within-session motion logic.
- Commit per task. Branch off the current `feat/method-e-background-subtraction` work or `main` as the repo state requires.

---

### Task 1: Keep bag data out of git, and correct the "no rosbags" claim

**Files:**
- Modify: `.gitignore`
- Modify: `CLAUDE.md`
- Create: `ros/lctk_sample_data/bags/README.md`

**Interfaces:**
- Consumes: nothing.
- Produces: the documented on-disk layout every later task assumes — `ros/lctk_sample_data/bags/TWO_LIDAR_{1,2,3,4}/` each containing `metadata.yaml` and `TWO_LIDAR_{n}_0.db3`.

- [ ] **Step 1: Confirm the bags are not already tracked**

```bash
cd /home/jetson/LCTK
git ls-files ros/lctk_sample_data/bags/ | head
```
Expected: **no output**. If any files are listed, untrack them without deleting the local copies:
```bash
git rm -r --cached ros/lctk_sample_data/bags/
```

- [ ] **Step 2: Add the ignore rule**

Append to `.gitignore`:

```gitignore

# Recorded ROS 2 bags used as boarddet benchmark data (TWO_LIDAR_*).
# ~2.4 GB of .db3 plus ~1.9 GB of redundant .zip archives -- far too large to
# put in git history, and re-derivable only by re-recording. See
# ros/lctk_sample_data/bags/README.md for the expected layout.
/ros/lctk_sample_data/bags/
```

- [ ] **Step 3: Document the expected layout**

Create `ros/lctk_sample_data/bags/README.md`:

```markdown
# Recorded bags (not tracked in git)

This directory is gitignored: the bags are ~2.4 GB of `.db3` plus ~1.9 GB of
`.zip`, which must not enter git history. Obtain them from the project's data
share and unpack them here so the layout is:

```
ros/lctk_sample_data/bags/
  TWO_LIDAR_1/
    metadata.yaml
    TWO_LIDAR_1_0.db3
  TWO_LIDAR_2/ ...
  TWO_LIDAR_3/ ...
  TWO_LIDAR_4/ ...
```

Each bag is a ~20 s, ~199-frame-per-sensor static capture of the calibration
board from a two-LiDAR rig:

| topic | sensor | frame_id | points/frame |
|---|---|---|---|
| `/lidar/vlp32/velodyne_points` | Velodyne VLP-32C (spinning) | `velodyne` | ~51,400 |
| `/lidar/falcon/iv_points` | Innovusion/Seyond Falcon (solid-state) | `seyond` | ~92,300 |

The board is held static within each bag.

To use them in the `boarddet` experiment, export to its `.npz` cache first —
see `experiments/board-detection-2d/README.md`.

Verify a bag with:

```bash
source /opt/ros/humble/setup.bash
ros2 bag info ros/lctk_sample_data/bags/TWO_LIDAR_1
```
```

- [ ] **Step 4: Correct CLAUDE.md**

`CLAUDE.md` currently asserts twice that the repo has no rosbags. Both statements are now wrong.

In the `lctk_sample_data/` bullet under **Project Structure**, replace:

```markdown
  - `lctk_sample_data/` - Sample data playback (pcap + avi; there are **no rosbags** in this repo)
```

with:

```markdown
  - `lctk_sample_data/` - Sample data playback (pcap + avi), plus gitignored recorded
    `bags/TWO_LIDAR_*` (two-LiDAR: VLP-32C + solid-state Falcon; see `bags/README.md`)
```

In the **Processing Modes** section, replace:

```markdown
Note: the repo ships no rosbags — the only recorded data is `lctk_sample_data`'s pcap + avi
(datasets 1–5; dataset 3 is the lidar-camera default, dataset 4 the second lidar). To get a real
bag, record one during playback: `ros2 bag record -a` alongside `just sample-data`.
```

with:

```markdown
Note: `lctk_sample_data` ships pcap + avi in git (datasets 1–5; dataset 3 is the lidar-camera
default, dataset 4 the second lidar). Recorded two-LiDAR bags live in
`ros/lctk_sample_data/bags/TWO_LIDAR_*` but are **gitignored** — see that directory's README to
obtain them. To record more: `ros2 bag record -a` alongside `just sample-data`.
```

- [ ] **Step 5: Verify the ignore rule works, then commit**

```bash
cd /home/jetson/LCTK
git status --porcelain ros/lctk_sample_data/bags/
```
Expected: **no output** (the directory is ignored; `bags/README.md` is inside an ignored directory, so force-add it).

```bash
git add -f ros/lctk_sample_data/bags/README.md
git add .gitignore CLAUDE.md
git commit -m "chore(sample-data): gitignore recorded TWO_LIDAR bags, document layout

The bags are ~2.4 GB of .db3 plus ~1.9 GB of .zip, which must not enter git
history. Document the expected on-disk layout and both topics instead, and
correct CLAUDE.md's two now-false claims that the repo ships no rosbags."
```

---

### Task 2: Bag → npz export tool

**Files:**
- Create: `experiments/board-detection-2d/tools/export_bag_npz.py`
- Test: none automated — this file cannot be imported by the `boarddet` test suite (it needs ROS, which lives in a different Python). Verified by running it in Step 4.

**Interfaces:**
- Consumes: the bag layout from Task 1.
- Produces: `experiments/board-detection-2d/cache/bag_{bag}_{sensor}.npz` files using `ingest.py`'s existing cache schema — `stamps` (float64, N) plus `xyz_{i}` (float32, M×3), `intensity_{i}` (float32, M), `ring_{i}` (uint8, M) for each frame `i`. Sensor keys are `vlp32` and `falcon`.

- [ ] **Step 1: Write the export tool**

Create `experiments/board-detection-2d/tools/export_bag_npz.py`:

```python
#!/usr/bin/env python3
"""Export a recorded ROS 2 bag's PointCloud2 topic to boarddet's npz cache.

This is the ONLY file in this experiment that imports ROS. It runs under
system Python 3.10 with /opt/ros/humble/setup.bash sourced -- NOT inside the
uv venv, which is Python 3.11 and deliberately ROS-free. boarddet never
imports this module; it only reads the .npz files it writes.

    source /opt/ros/humble/setup.bash
    python3 experiments/board-detection-2d/tools/export_bag_npz.py \
        --bags TWO_LIDAR_1 TWO_LIDAR_2 TWO_LIDAR_3 TWO_LIDAR_4 \
        --sensors vlp32 falcon

Output schema matches ingest.py's pcap cache exactly (stamps + per-frame
xyz_i/intensity_i/ring_i), so both sources yield identical Frame objects.
`channel` is stored as `ring`; like intensity it is DIAGNOSTIC ONLY and
algorithm code must never read it.
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2

_REPO_ROOT = Path(__file__).resolve().parents[3]
_BAG_DIR = _REPO_ROOT / "ros" / "lctk_sample_data" / "bags"
_CACHE_DIR = Path(__file__).resolve().parents[1] / "cache"

SENSOR_TOPICS = {
    "vlp32": "/lidar/vlp32/velodyne_points",
    "falcon": "/lidar/falcon/iv_points",
}

# Both sensors lay out the first 16 bytes identically: x,y,z float32 at
# offsets 0/4/8, intensity uint8 at 12, return_type uint8 at 13, channel
# uint16 at 14. Assert rather than assume -- a silently mis-parsed cloud
# would look like plausible noise downstream.
_EXPECTED_OFFSETS = {"x": 0, "y": 4, "z": 8, "intensity": 12, "channel": 14}


def _check_layout(msg: PointCloud2) -> None:
    offsets = {f.name: f.offset for f in msg.fields}
    for name, want in _EXPECTED_OFFSETS.items():
        if offsets.get(name) != want:
            raise ValueError(
                f"unexpected PointCloud2 layout: field {name!r} at offset "
                f"{offsets.get(name)}, expected {want}. Fields present: "
                f"{[(f.name, f.offset) for f in msg.fields]}")


def _decode(msg: PointCloud2):
    """-> (xyz float32 (M,3), intensity float32 (M,), ring uint8 (M,))."""
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
    xyz = raw[:, 0:12].copy().view(np.float32).reshape(-1, 3)
    intensity = raw[:, 12].astype(np.float32)
    ring = raw[:, 14:16].copy().view(np.uint16).reshape(-1)
    # is_dense is advertised True, but a NaN here would poison every plane
    # fit downstream, so filter rather than trust the flag.
    keep = np.isfinite(xyz).all(axis=1)
    return xyz[keep], intensity[keep], ring[keep].astype(np.uint8)


def export(bag: str, sensor: str, overwrite: bool = False) -> Path:
    topic = SENSOR_TOPICS[sensor]
    uri = _BAG_DIR / bag
    if not uri.exists():
        raise FileNotFoundError(f"bag not found: {uri} (see bags/README.md)")
    out = _CACHE_DIR / f"bag_{bag}_{sensor}.npz"
    if out.exists() and not overwrite:
        print(f"  {out.name} exists, skipping (use --overwrite to redo)")
        return out

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(uri), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""))

    stamps: list[float] = []
    arrays: dict[str, np.ndarray] = {}
    i = 0
    while reader.has_next():
        got_topic, data, _ = reader.read_next()
        if got_topic != topic:
            continue
        msg = deserialize_message(data, PointCloud2)
        if i == 0:
            _check_layout(msg)
        xyz, intensity, ring = _decode(msg)
        stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        arrays[f"xyz_{i}"] = xyz
        arrays[f"intensity_{i}"] = intensity
        arrays[f"ring_{i}"] = ring
        i += 1

    if i == 0:
        raise ValueError(f"no messages on {topic!r} in {bag}")
    arrays["stamps"] = np.array(stamps, dtype=np.float64)
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(out, **arrays)
    pts = int(np.mean([len(arrays[f"xyz_{j}"]) for j in range(i)]))
    print(f"  {out.name}: {i} frames, ~{pts} pts/frame")
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--bags", nargs="+",
                    default=["TWO_LIDAR_1", "TWO_LIDAR_2",
                             "TWO_LIDAR_3", "TWO_LIDAR_4"])
    ap.add_argument("--sensors", nargs="+", default=list(SENSOR_TOPICS),
                    choices=list(SENSOR_TOPICS))
    ap.add_argument("--overwrite", action="store_true")
    args = ap.parse_args()
    for bag in args.bags:
        print(bag)
        for sensor in args.sensors:
            export(bag, sensor, overwrite=args.overwrite)


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Export one bag's VLP32 topic as a smoke test**

```bash
cd /home/jetson/LCTK
source /opt/ros/humble/setup.bash
python3 experiments/board-detection-2d/tools/export_bag_npz.py \
    --bags TWO_LIDAR_1 --sensors vlp32
```
Expected: `bag_TWO_LIDAR_1_vlp32.npz: 199 frames, ~51000 pts/frame`

- [ ] **Step 3: Verify the npz is readable and sane from the boarddet venv**

```bash
cd /home/jetson/LCTK/experiments/board-detection-2d
uv run python -c "
import numpy as np
z = np.load('cache/bag_TWO_LIDAR_1_vlp32.npz')
print('frames', len(z['stamps']))
print('xyz_0', z['xyz_0'].shape, z['xyz_0'].dtype)
print('range', np.linalg.norm(z['xyz_0'][:, :2], axis=1).max().round(1), 'm')
print('finite', np.isfinite(z['xyz_0']).all())
"
```
Expected: 199 frames, `xyz_0 (N, 3) float32`, a plausible max range (tens of metres), `finite True`.

- [ ] **Step 4: Export everything**

```bash
cd /home/jetson/LCTK && source /opt/ros/humble/setup.bash
python3 experiments/board-detection-2d/tools/export_bag_npz.py
```
Expected: 8 files (4 bags × 2 sensors). This reads ~2.4 GB and will take a few minutes.

- [ ] **Step 5: Confirm the cache stays out of git, then commit the tool**

`cache/` is already gitignored by the experiment; confirm:
```bash
cd /home/jetson/LCTK && git status --porcelain experiments/board-detection-2d/cache/
```
Expected: no output.

```bash
git add experiments/board-detection-2d/tools/export_bag_npz.py
git commit -m "feat(boarddet): export recorded bags to the npz frame cache

The only file in this experiment that imports ROS: it runs under system
Python 3.10 with Humble sourced, while boarddet itself stays ROS-free on
3.11. Output uses ingest.py's existing cache schema, so bag-sourced and
pcap-sourced frames are indistinguishable downstream.

Asserts the PointCloud2 field layout rather than assuming it -- a
mis-parsed cloud would look like plausible noise to every later stage."
```

---

### Task 3: Bag frame loader in `boarddet`

**Files:**
- Modify: `experiments/board-detection-2d/src/boarddet/ingest.py`
- Test: `experiments/board-detection-2d/tests/test_ingest_bags.py`

**Interfaces:**
- Consumes: the `.npz` files from Task 2.
- Produces: `load_bag_frames(bag: str, sensor: str, max_frames: int | None = None) -> list[Frame]` and `BAG_SENSORS: tuple[str, ...]` in `boarddet.ingest`.

- [ ] **Step 1: Write the failing tests**

Create `experiments/board-detection-2d/tests/test_ingest_bags.py`:

```python
"""Bag-sourced frames must be indistinguishable from pcap-sourced ones."""
from __future__ import annotations

import numpy as np
import pytest

from boarddet.ingest import Frame, load_bag_frames


def _write_cache(path, n_frames=3, n_pts=50):
    rng = np.random.default_rng(0)
    arrays = {"stamps": np.arange(n_frames, dtype=np.float64)}
    for i in range(n_frames):
        arrays[f"xyz_{i}"] = rng.normal(size=(n_pts, 3)).astype(np.float32)
        arrays[f"intensity_{i}"] = rng.random(n_pts).astype(np.float32)
        arrays[f"ring_{i}"] = rng.integers(0, 32, n_pts).astype(np.uint8)
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(path, **arrays)


def test_loads_frames_from_an_exported_cache(tmp_path, monkeypatch):
    import boarddet.ingest as ingest
    monkeypatch.setattr(ingest, "CACHE_DIR", tmp_path)
    _write_cache(tmp_path / "bag_TESTBAG_vlp32.npz", n_frames=3)

    frames = load_bag_frames("TESTBAG", "vlp32")
    assert len(frames) == 3
    assert all(isinstance(f, Frame) for f in frames)
    assert frames[0].xyz.shape == (50, 3)
    assert frames[0].xyz.dtype == np.float32
    assert frames[1].stamp == 1.0


def test_max_frames_truncates(tmp_path, monkeypatch):
    import boarddet.ingest as ingest
    monkeypatch.setattr(ingest, "CACHE_DIR", tmp_path)
    _write_cache(tmp_path / "bag_TESTBAG_vlp32.npz", n_frames=5)
    assert len(load_bag_frames("TESTBAG", "vlp32", max_frames=2)) == 2


def test_missing_export_names_the_tool_that_creates_it(tmp_path, monkeypatch):
    """A missing cache is a workflow step not yet run, not a crash -- the
    error must say how to fix it."""
    import boarddet.ingest as ingest
    monkeypatch.setattr(ingest, "CACHE_DIR", tmp_path)
    with pytest.raises(FileNotFoundError, match="export_bag_npz"):
        load_bag_frames("NOPE", "vlp32")


def test_unknown_sensor_is_rejected(tmp_path, monkeypatch):
    import boarddet.ingest as ingest
    monkeypatch.setattr(ingest, "CACHE_DIR", tmp_path)
    with pytest.raises(ValueError, match="sensor"):
        load_bag_frames("TESTBAG", "lidar-that-does-not-exist")
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
cd /home/jetson/LCTK/experiments/board-detection-2d
uv run pytest tests/test_ingest_bags.py -q
```
Expected: FAIL — `ImportError: cannot import name 'load_bag_frames'`

- [ ] **Step 3: Implement the loader**

In `experiments/board-detection-2d/src/boarddet/ingest.py`, append after `load_frames`:

```python
# Sensors exported from the recorded TWO_LIDAR bags by
# tools/export_bag_npz.py. "falcon" is solid-state (no ring structure) --
# see BoardConfig.vertical_gap_deg before benchmarking it.
BAG_SENSORS = ("vlp32", "falcon")


def _bag_cache_path(bag: str, sensor: str) -> Path:
    return CACHE_DIR / f"bag_{bag}_{sensor}.npz"


def load_bag_frames(bag: str, sensor: str,
                    max_frames: int | None = None) -> list[Frame]:
    """Load frames exported from a recorded ROS 2 bag.

    Unlike `load_frames`, this never decodes anything itself: bags are read
    by `tools/export_bag_npz.py`, which needs ROS and therefore a different
    Python than this package runs on. The export is a prerequisite, and a
    missing one is a workflow step not yet taken.
    """
    if sensor not in BAG_SENSORS:
        raise ValueError(
            f"unknown sensor {sensor!r}; expected one of {BAG_SENSORS}")
    cached = _bag_cache_path(bag, sensor)
    if not cached.exists():
        raise FileNotFoundError(
            f"no exported cache at {cached}. Create it with:\n"
            f"  source /opt/ros/humble/setup.bash\n"
            f"  python3 tools/export_bag_npz.py --bags {bag} "
            f"--sensors {sensor}")
    frames = _load_cache(cached)
    if max_frames is not None:
        frames = frames[:max_frames]
    return frames
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
uv run pytest tests/test_ingest_bags.py -q
```
Expected: PASS, 4 tests.

- [ ] **Step 5: Verify against a real exported bag**

```bash
uv run python -c "
from boarddet.ingest import load_bag_frames
f = load_bag_frames('TWO_LIDAR_1', 'vlp32')
print(len(f), 'frames;', f[0].xyz.shape, 'pts in frame 0')
"
```
Expected: `199 frames; (N, 3) pts in frame 0`

- [ ] **Step 6: Run the full suite and commit**

```bash
uv run pytest -q
```
Expected: all pass (181 existing + 4 new).

```bash
cd /home/jetson/LCTK
git add experiments/board-detection-2d/src/boarddet/ingest.py \
        experiments/board-detection-2d/tests/test_ingest_bags.py
git commit -m "feat(boarddet): load frames exported from recorded bags

Bag-sourced frames use the same cache schema and yield the same Frame
objects as the pcap path, so everything downstream is source-agnostic. A
missing export is a workflow step not yet run, so the error names the tool
and the exact command that creates it."
```

---

### Task 4: Rotation-aware bbox reference loader

**Files:**
- Create: `experiments/board-detection-2d/src/boarddet/bbox_ref.py`
- Modify: `experiments/board-detection-2d/src/boarddet/benchmark_e_loo.py`
- Modify: `experiments/board-detection-2d/pyproject.toml` (via `uv add json5`)
- Test: `experiments/board-detection-2d/tests/test_bbox_ref.py`

**Interfaces:**
- Consumes: nothing from earlier tasks.
- Produces: `BoxRef` (dataclass with `center: np.ndarray`, `half: np.ndarray`, `rot: np.ndarray`, method `contains(point) -> bool`) and `load_bbox(path: str | Path) -> BoxRef` in `boarddet.bbox_ref`. `benchmark_e_loo.in_bbox` becomes `BoxRef.contains`; `benchmark_e_loo.DEFAULT_BBOX_PATH` points at the existing pcap-rig reference.

The reference file format is the existing `bbox.json5` schema — JSON5 (comments allowed), with
`pose.translation` `[x, y, z]`, `pose.rotation` a quaternion in **`[x, y, z, w]` order (w last,
nalgebra's serde convention)**, and `size_xyz` `[x, y, z]` giving the box's **full** extent.

- [ ] **Step 1: Add the json5 dependency**

```bash
cd /home/jetson/LCTK/experiments/board-detection-2d
uv add json5
```
The existing `bbox.json5` contains `//` comments, so `json.load` cannot read it.

- [ ] **Step 2: Write the failing tests**

Create `experiments/board-detection-2d/tests/test_bbox_ref.py`:

```python
"""The true-board reference box, loaded from a bbox.json5 rather than
hardcoded, and honouring the box's own rotation."""
from __future__ import annotations

import numpy as np
import pytest

import pathlib

from boarddet.bbox_ref import load_bbox

# Confirmed true-board centres, phase-7 doc "Pose sanity" table.
_BOARDS = [
    (2.256, -0.059, 0.074), (2.147, 0.420, 0.076), (2.101, -0.314, 0.074),
    (2.077, -0.605, 0.066), (2.090, -0.829, 0.039),
]
# Documented static clutter attractors.
_CLUTTER = [(-1.83, -2.89, -0.1), (4.7, 2.6, -0.1), (-3.3, 3.4, 0.5)]

# The pcap rig's reference, as used by stages 3-8 and Method E.
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_PCAP_BBOX = _REPO_ROOT / "ros/lctk_launch/config/board/bbox.json5"


def test_reads_json5_with_comments():
    """The real reference file has // comments; strict json cannot parse it."""
    box = load_bbox(_PCAP_BBOX)
    assert np.allclose(box.center, [2.6, 0.0, 0.35])
    assert np.allclose(box.half, [1.55, 1.97, 1.1])


@pytest.mark.parametrize("center", _BOARDS)
def test_confirmed_boards_are_inside(center):
    assert load_bbox(_PCAP_BBOX).contains(np.array(center))


@pytest.mark.parametrize("center", _CLUTTER)
def test_confirmed_clutter_is_outside(center):
    assert not load_bbox(_PCAP_BBOX).contains(np.array(center))


def test_rotation_is_applied(tmp_path):
    """A box rotated 90 deg about z swaps which points fall inside. Without
    rotation handling this test's second assertion passes wrongly."""
    p = tmp_path / "rot.json5"
    p.write_text("""{
        // 90 deg about z, quaternion in (x, y, z, w) order
        "pose": {"translation": [0.0, 0.0, 0.0],
                 "rotation": [0.0, 0.0, 0.7071067811865476, 0.7071067811865476]},
        "size_xyz": [4.0, 1.0, 1.0]
    }""")
    box = load_bbox(p)
    # The long axis now points along world y, not world x.
    assert box.contains(np.array([0.0, 1.5, 0.0]))
    assert not box.contains(np.array([1.5, 0.0, 0.0]))


def test_identity_rotation_is_axis_aligned(tmp_path):
    p = tmp_path / "ident.json5"
    p.write_text('{"pose": {"translation": [0,0,0], "rotation": [0,0,0,1]},'
                 ' "size_xyz": [2.0, 2.0, 2.0]}')
    box = load_bbox(p)
    assert box.contains(np.array([0.9, 0.9, 0.9]))
    assert not box.contains(np.array([1.1, 0.0, 0.0]))


def test_missing_file_raises(tmp_path):
    with pytest.raises(FileNotFoundError):
        load_bbox(tmp_path / "nope.json5")
```

- [ ] **Step 3: Run tests to verify they fail**

```bash
uv run pytest tests/test_bbox_ref.py -q
```
Expected: FAIL — `ModuleNotFoundError: No module named 'boarddet.bbox_ref'`

- [ ] **Step 4: Implement the loader**

Create `experiments/board-detection-2d/src/boarddet/bbox_ref.py`:

```python
"""The true-board reference box, read from a `bbox.json5`.

Benchmarks classify each accepted detection as true-board or clutter by
where its centre falls. Stages 3-8 and Method E did that against one
hardcoded box -- the pcap rig's -- which is wrong for any other rig. Each
recording rig supplies its own reference file in the same schema the
detector's crop box already uses:

    {
      "pose": {"translation": [x, y, z],
               "rotation": [x, y, z, w]},   // quaternion, w LAST
      "size_xyz": [x, y, z]                 // FULL extent, not half
    }

The rotation is nalgebra's serde order (w last) -- the same trap
`bbox.json5`'s own comment documents, where [1,0,0,0] looks like identity
but is a 180 deg rotation about x.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import json5
import numpy as np


def _quat_xyzw_to_matrix(q: np.ndarray) -> np.ndarray:
    """Quaternion (x, y, z, w) -> 3x3 rotation matrix (box -> world)."""
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


@dataclass
class BoxRef:
    center: np.ndarray  # (3,) box centre in world coords
    half: np.ndarray    # (3,) half extents along the box's own axes
    rot: np.ndarray     # (3,3) box -> world

    def contains(self, point: np.ndarray) -> bool:
        """Is `point` inside the box? Tested in the BOX's frame, so a
        rotated reference is handled correctly rather than being treated as
        its axis-aligned bounding box."""
        local = self.rot.T @ (np.asarray(point, dtype=np.float64) - self.center)
        return bool(np.all(np.abs(local) <= self.half))


def load_bbox(path: str | Path) -> BoxRef:
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"bbox reference not found: {path}")
    raw = json5.loads(path.read_text())
    pose = raw["pose"]
    return BoxRef(
        center=np.asarray(pose["translation"], dtype=np.float64),
        half=np.asarray(raw["size_xyz"], dtype=np.float64) / 2.0,
        rot=_quat_xyzw_to_matrix(
            np.asarray(pose["rotation"], dtype=np.float64)),
    )
```

- [ ] **Step 5: Run tests to verify they pass**

```bash
uv run pytest tests/test_bbox_ref.py -q
```
Expected: PASS, 12 tests.

- [ ] **Step 6: Replace the harness's hardcoded box**

In `experiments/board-detection-2d/src/boarddet/benchmark_e_loo.py`, delete the `_BBOX_CENTER` /
`_BBOX_HALF` constants and the `in_bbox` function, and replace them with a module-level default
path plus use of `BoxRef`:

```python
from .bbox_ref import BoxRef, load_bbox

# The pcap rig's reference, used by stages 3-8 and Method E. Other rigs
# (e.g. the recorded TWO_LIDAR bags) supply their own via --bbox.
DEFAULT_BBOX_PATH = (Path(__file__).resolve().parents[3]
                     / "ros" / "lctk_launch" / "config" / "board"
                     / "bbox.json5")
```

`run_loo` gains a `box: BoxRef` parameter, and every `in_bbox(d.center)` call becomes
`box.contains(d.center)`. `main()` gains:

```python
    ap.add_argument("--bbox", type=Path, default=DEFAULT_BBOX_PATH,
                    help="true-board reference box (bbox.json5 schema); "
                         "each recording rig has its own")
```
and passes `box=load_bbox(args.bbox)` into `run_loo`.

Update `tests/test_benchmark_e_loo.py`: its `test_confirmed_board_centers_are_in_bbox`,
`test_confirmed_clutter_is_outside_bbox` and any other `loo.in_bbox` use now load the box
explicitly:

```python
from boarddet.bbox_ref import load_bbox
from boarddet.benchmark_e_loo import DEFAULT_BBOX_PATH

_BOX = load_bbox(DEFAULT_BBOX_PATH)
```
and call `_BOX.contains(...)`. `near_known_clutter` is unchanged. `run_loo` calls in those tests
gain `box=_BOX`.

- [ ] **Step 7: Verify the refactor changed no behavior**

```bash
uv run pytest -q
```
Expected: all pass. The confirmed-coordinate tests are the regression pin — they assert the same
in/out classification as the hardcoded constants did.

- [ ] **Step 8: Commit**

```bash
cd /home/jetson/LCTK
git add experiments/board-detection-2d/src/boarddet/bbox_ref.py \
        experiments/board-detection-2d/src/boarddet/benchmark_e_loo.py \
        experiments/board-detection-2d/tests/test_bbox_ref.py \
        experiments/board-detection-2d/tests/test_benchmark_e_loo.py \
        experiments/board-detection-2d/pyproject.toml \
        experiments/board-detection-2d/uv.lock
git commit -m "refactor(boarddet): load the true-board reference from bbox.json5

The LOO harness hardcoded the pcap rig's reference box, which is wrong for
any other rig -- and the recorded TWO_LIDAR bags are a different one. Read
it from a bbox.json5 instead, in the same schema the detector's crop box
already uses, and honour the box's rotation rather than treating a rotated
reference as its axis-aligned bound.

The confirmed board/clutter coordinates from the phase-7 pose-sanity
tables serve as the regression pin: classification is unchanged."
```

---

### Task 5: Generalize the LOO harness to named sources

**Files:**
- Modify: `experiments/board-detection-2d/src/boarddet/benchmark_e_loo.py`
- Test: `experiments/board-detection-2d/tests/test_benchmark_e_loo.py`

**Interfaces:**
- Consumes: `load_bag_frames` (Task 3), `BoxRef`/`load_bbox` (Task 4).
- Produces: `run_loo(sources: dict[str, list[Frame]], board, out_dir, *, box, background_voxel=0.06, dilation_radius=1, min_sources=2) -> dict` — now taking already-loaded frames keyed by label instead of integer dataset ids — plus `load_sources(kind: str, names: list[str], sensor: str, max_frames: int | None) -> dict[str, list[Frame]]`.

- [ ] **Step 1: Write the failing tests**

Add to `experiments/board-detection-2d/tests/test_benchmark_e_loo.py`:

```python
def test_run_loo_accepts_named_sources(tmp_path):
    """Folds are keyed by label, so pcap datasets ('3') and bags
    ('TWO_LIDAR_1') can both be held out by the same harness."""
    from boarddet.board_config import BoardConfig
    sources = {"A": _frames(1.0), "B": _frames(1.0),
               "C": _frames(1.0), "D": _frames(9.0)}
    out = loo.run_loo(sources, BoardConfig(side_m=1.0), tmp_path,
                      box=_BOX, min_sources=2, dilation_radius=0)
    assert set(out["folds"]) == {"A", "B", "C", "D"}


def test_unreachable_min_sources_still_rejected(tmp_path):
    from boarddet.board_config import BoardConfig
    sources = {"A": _frames(1.0), "B": _frames(9.0)}
    with pytest.raises(ValueError, match="unreachable"):
        loo.run_loo(sources, BoardConfig(side_m=1.0), tmp_path,
                    box=_BOX, min_sources=2)


def test_load_sources_rejects_unknown_kind():
    with pytest.raises(ValueError, match="kind"):
        loo.load_sources("floppy-disk", ["1"], "vlp32", None)
```

Replace the existing `test_build_background_excludes_the_held_out_dataset` and
`test_consensus_drops_a_single_contributors_unique_geometry` dict keys with the same string labels
(`"A"`, `"B"`, …) so they match the new signature; their assertions are unchanged.

- [ ] **Step 2: Run tests to verify they fail**

```bash
uv run pytest tests/test_benchmark_e_loo.py -q
```
Expected: FAIL — `run_loo` still expects `datasets: list[int]`.

- [ ] **Step 3: Generalize `run_loo` and add `load_sources`**

In `benchmark_e_loo.py`, replace the signature and the loading/guard block:

```python
def load_sources(kind: str, names: list[str], sensor: str,
                 max_frames: int | None) -> dict[str, list[Frame]]:
    """Load frames for each named capture. `kind` selects the reader:
    "pcap" for the sample datasets 1-5, "bag" for exported TWO_LIDAR bags
    (see tools/export_bag_npz.py). Labels are the names as given, so folds
    are readable in the output either way."""
    if kind == "pcap":
        return {n: load_frames(int(n), max_frames=max_frames) for n in names}
    if kind == "bag":
        return {n: load_bag_frames(n, sensor, max_frames=max_frames)
                for n in names}
    raise ValueError(f"unknown source kind {kind!r}; expected 'pcap' or 'bag'")


def run_loo(sources: dict[str, list[Frame]], board: BoardConfig,
            out_dir: Path, *, box: BoxRef, background_voxel: float = 0.06,
            dilation_radius: int = 1, min_sources: int = 2) -> dict:
    n_contributors = len(sources) - 1
    if n_contributors < min_sources:
        raise ValueError(
            f"min_sources={min_sources} is unreachable with {len(sources)} "
            f"captures: each fold has only {n_contributors} contributing "
            f"source(s), so the background would be empty and every point "
            f"would count as foreground. Use at least {min_sources + 1} "
            f"captures, or lower --min-sources.")
    out_dir.mkdir(parents=True, exist_ok=True)
    folds: dict[str, dict] = {}
    for held_out in sources:
        model = build_background(sources, held_out, background_voxel,
                                 dilation_radius, min_sources)
        outcomes = [detect(f.xyz, board, generator="e", background=model)
                    for f in sources[held_out]]
        ...
```

The body of the loop is otherwise unchanged except that `in_bbox(d.center)` becomes
`box.contains(d.center)` and the fold key is the string label. `build_background`'s signature
changes only in its type hint (`dict[str, list[Frame]]`, `held_out: str`); its body is unchanged.

Add to the summary dict: `"source_labels": list(sources)`.

`main()` gains source selection and passes through:

```python
    ap.add_argument("--source", choices=["pcap", "bag"], default="pcap",
                    help="pcap = sample datasets 1-5; bag = exported "
                         "TWO_LIDAR bags (run tools/export_bag_npz.py first)")
    ap.add_argument("--names", nargs="+", default=None,
                    help="capture names; defaults to 1..5 for pcap and "
                         "TWO_LIDAR_1..4 for bag")
    ap.add_argument("--sensor", choices=["vlp32", "falcon"], default="vlp32",
                    help="bag sources only; falcon is solid-state, so "
                         "consider --vertical-gap-deg 0")
    ap.add_argument("--vertical-gap-deg", type=float, default=3.0,
                    help="anisotropic clustering tolerance; 3.0 suits the "
                         "VLP-32C's ring gaps, 0 disables it for a "
                         "solid-state sensor with no ring structure")
```

with defaults resolved as:

```python
    names = args.names
    if names is None:
        names = (["1", "2", "3", "4", "5"] if args.source == "pcap"
                 else ["TWO_LIDAR_1", "TWO_LIDAR_2", "TWO_LIDAR_3",
                       "TWO_LIDAR_4"])
    sources = load_sources(args.source, names, args.sensor, args.max_frames)
```

and `vertical_gap_deg=args.vertical_gap_deg` added to the `BoardConfig(...)` construction.

- [ ] **Step 4: Run tests to verify they pass**

```bash
uv run pytest tests/test_benchmark_e_loo.py -q
```
Expected: PASS.

- [ ] **Step 5: Confirm the pcap result is unchanged**

The refactor must not move the published Method E numbers:

```bash
uv run python -m boarddet.benchmark_e_loo --source pcap \
  --side 1.0 --stance-gate --flatness-rms-max 0.045 --min-sources 3 \
  --isolation --isolation-max-density 0.3 --out results/regress-ms3-iso
```
Expected, matching the published table exactly: ds1 99.0%, ds2 100%, ds3 99.1%, ds4 99.1%,
ds5 42.7%; 0 clutter on every fold.

- [ ] **Step 6: Full suite and commit**

```bash
uv run pytest -q
```

```bash
cd /home/jetson/LCTK
git add experiments/board-detection-2d/src/boarddet/benchmark_e_loo.py \
        experiments/board-detection-2d/tests/test_benchmark_e_loo.py
git commit -m "refactor(boarddet): LOO harness takes named capture sources

Folds are keyed by label instead of integer dataset id, and frames are
loaded by a --source selector, so pcap datasets and exported bags run
through one harness. Adds --sensor and --vertical-gap-deg, the latter
because a solid-state sensor has no ring gaps for the anisotropic
clustering to bridge.

Verified the published pcap numbers are unchanged by the refactor."
```

---

### Task 6: Cross-bag board-motion diagnostic, then the VLP32 benchmark

**Files:**
- Create: `experiments/board-detection-2d/tools/bag_motion_probe.py`
- Modify: `docs/roadmap/side-track_method-e-background-subtraction.md`

**Interfaces:**
- Consumes: `load_bag_frames` (Task 3), `BackgroundModel` (existing), the generalized harness (Task 5).
- Produces: measured numbers and a new results section. No new library code.

**This task is gated on the board actually moving between bags.** Method E's whole premise is that
the held-out capture's board sits where the others' do not. Measure that before benchmarking, and
if it fails, stop and report rather than producing a meaningless 0%.

- [ ] **Step 1: Write the diagnostic**

Create `experiments/board-detection-2d/tools/bag_motion_probe.py`:

```python
"""Does the board sit somewhere different in each bag?

Method E's cross-capture premise: the held-out capture contains an object
the others do not, at a location they do not occupy. If every bag places
the board identically, a consensus background absorbs it and LOO recall is
0 by construction -- a fact worth knowing BEFORE running a benchmark and
misreading the result as a detector failure.

For each bag, build a consensus background from the OTHER bags and report
what survives: how many points, and where their largest cluster sits.
"""
from __future__ import annotations

import numpy as np

from boarddet.background import BackgroundModel
from boarddet.geometry import downsample
from boarddet.ingest import load_bag_frames

BAGS = ["TWO_LIDAR_1", "TWO_LIDAR_2", "TWO_LIDAR_3", "TWO_LIDAR_4"]


def main() -> None:
    frames = {b: load_bag_frames(b, "vlp32", max_frames=40) for b in BAGS}
    dn = {b: [downsample(f.xyz, 0.03) for f in fr] for b, fr in frames.items()}
    for min_sources in (1, 2, 3):
        print(f"--- min_sources={min_sources}")
        for held in BAGS:
            m = BackgroundModel(voxel=0.06, dilation_radius=1,
                                min_sources=min_sources)
            for b in BAGS:
                if b == held:
                    continue
                m.observe(np.concatenate(dn[b], axis=0), source=b)
            m.finalize()
            fg = m.foreground_points(dn[held][0])
            if len(fg) == 0:
                print(f"  {held}: 0 foreground points "
                      f"(bg {m.n_voxels} voxels)")
                continue
            c = fg.mean(axis=0)
            print(f"  {held}: {len(fg):6d} fg pts  centroid "
                  f"({c[0]:6.2f},{c[1]:6.2f},{c[2]:6.2f})  "
                  f"bg {m.n_voxels} voxels")


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Run it and decide whether to continue**

```bash
cd /home/jetson/LCTK/experiments/board-detection-2d
uv run python tools/bag_motion_probe.py
```

Read the result before going further:

- **Foreground survives at some threshold, with per-bag centroids that differ** → the board moves
  between bags. Continue to Step 3, using the smallest threshold that leaves foreground.
- **Foreground is ~0 at every threshold** → the board is in the same place in all four bags.
  **Stop.** Record the finding in the results doc as a measured limitation, note that cross-bag LOO
  cannot work on this capture set, and report to the human — do not run a benchmark whose result is
  predetermined.

Note the four bags give only **three** contributors per fold, so `min_sources` can be at most 3
(the harness rejects more).

- [ ] **Step 3: Sweep the consensus threshold**

```bash
for MS in 1 2 3; do
  echo "===== min_sources=$MS ====="
  uv run python -m boarddet.benchmark_e_loo --source bag --sensor vlp32 \
    --side 1.0 --stance-gate --flatness-rms-max 0.045 --min-sources $MS \
    --bbox <PATH TO THE BAG RIG'S bbox.json5> \
    --out results/bagE-vlp32-ms$MS
done
```

**`--bbox` is required and has no default that fits this rig.** The bag rig's reference file is
supplied separately by the user. If it is not yet available, run the sweep anyway and read
`n_detections` and the per-fold centroids from `loo_summary.json` — recall/precision will be
meaningless (every detection classified against the wrong box), so report only detection counts and
pose clusters until the reference arrives.

- [ ] **Step 4: Run the recommended operating point with isolation**

```bash
uv run python -m boarddet.benchmark_e_loo --source bag --sensor vlp32 \
  --side 1.0 --stance-gate --flatness-rms-max 0.045 \
  --min-sources <BEST FROM STEP 3> --isolation --isolation-max-density 0.3 \
  --bbox <PATH TO THE BAG RIG'S bbox.json5> \
  --out results/bagE-vlp32-best
```

- [ ] **Step 5: Write up the VLP32 bag results**

Append a section to `docs/roadmap/side-track_method-e-background-subtraction.md`, after the
Verdict, titled `## Second rig: recorded TWO_LIDAR bags (VLP-32C)`. Include, filled from the runs
above and **no number written before its run produced it**:

- The motion-probe result from Step 2, stated first — it conditions everything after it.
- The consensus-threshold sweep, per fold and in total, in the same table shape as the pcap results.
- Whether `n_known_clutter_survived` is meaningful here: the documented attractor coordinates belong
  to the **pcap** rig, so on the bag rig that counter is not a valid sanity check. Say so explicitly
  and do not present it as one.
- A comparison against the pcap result (88.4% / 100%), with the caveat that only three contributors
  are available per fold versus four, so the thresholds are not directly equivalent.
- Timing, against the 100 ms/frame budget. Bag frames carry ~51k points versus the pcaps' clouds, so
  do not assume the pcap timings transfer.

- [ ] **Step 6: Commit**

```bash
cd /home/jetson/LCTK
git add experiments/board-detection-2d/tools/bag_motion_probe.py \
        docs/roadmap/side-track_method-e-background-subtraction.md
git commit -m "feat(boarddet): cross-bag motion probe and VLP-32C bag results

Method E's premise is that the held-out capture's board sits where the
others' do not, so measure that first: a consensus background built from
the other bags, and what survives it. Running a benchmark without checking
would risk reading a structural 0% as a detector failure.

Results for the recorded TWO_LIDAR bags on the VLP-32C topic follow the
same reporting shape as the pcap results, including what does NOT transfer
between rigs."
```

---

### Task 7: Falcon (solid-state) pass

**Files:**
- Modify: `docs/roadmap/side-track_method-e-background-subtraction.md`
- Modify: `experiments/board-detection-2d/README.md`

**Interfaces:**
- Consumes: everything from Tasks 2–6.
- Produces: measured numbers and documentation. No new code.

The phase-7 doc's only solid-state evidence is synthetic: *"All three generators detect 5/5
uniform-pattern (Livox-like, no ring structure) synthetic scenes"*, with the explicit caveat that
*"Real spinning-LiDAR data is the hard case here, not the easy one."* The Falcon topic is the first
chance to test that claim on a real solid-state sensor.

- [ ] **Step 1: Sanity-check the Falcon clouds**

```bash
cd /home/jetson/LCTK/experiments/board-detection-2d
uv run python -c "
import numpy as np
from boarddet.ingest import load_bag_frames
from boarddet.geometry import downsample
f = load_bag_frames('TWO_LIDAR_1', 'falcon', max_frames=3)
for i, fr in enumerate(f):
    d = downsample(fr.xyz, 0.03)
    r = np.linalg.norm(fr.xyz[:, :2], axis=1)
    print(f'frame {i}: {len(fr.xyz)} pts -> {len(d)} at 0.03 voxel, '
          f'range {r.min():.1f}-{r.max():.1f} m')
"
```
Expected: ~92k points per frame, a plausible range span, and a voxel-downsampled count well below
the raw count. A downsampled count nearly equal to the raw count would mean the cloud is far
sparser than assumed, and the 0.03 m voxel needs revisiting before anything else.

- [ ] **Step 2: Run the sweep with anisotropic clustering disabled**

The anisotropic vertical scaling exists to bridge a spinning LiDAR's ring gaps
(`_anisotropic_scaled`'s docstring). A solid-state sensor has no rings, so run it off and on to
measure whether that reasoning holds in practice rather than assuming it:

```bash
for VG in 0 3.0; do
  echo "===== vertical_gap_deg=$VG ====="
  uv run python -m boarddet.benchmark_e_loo --source bag --sensor falcon \
    --side 1.0 --stance-gate --flatness-rms-max 0.045 \
    --min-sources <BEST FROM TASK 6> --vertical-gap-deg $VG \
    --bbox <PATH TO THE BAG RIG'S bbox.json5> \
    --out results/bagE-falcon-vg$VG
done
```

- [ ] **Step 3: Run the best configuration with isolation**

```bash
uv run python -m boarddet.benchmark_e_loo --source bag --sensor falcon \
  --side 1.0 --stance-gate --flatness-rms-max 0.045 \
  --min-sources <BEST> --vertical-gap-deg <BEST> \
  --isolation --isolation-max-density 0.3 \
  --bbox <PATH TO THE BAG RIG'S bbox.json5> \
  --out results/bagE-falcon-best
```

- [ ] **Step 4: Write up the Falcon results**

Append `## Solid-state: the Falcon topic` to
`docs/roadmap/side-track_method-e-background-subtraction.md`, covering:

- Recall/precision per fold and in total, versus the VLP-32C numbers on the **same bags** — the
  cleanest sensor-to-sensor comparison available anywhere in this phase, since the scene, rig, and
  board pose are identical and only the sensor differs.
- Whether disabling anisotropic clustering helped, hurt, or did nothing, and what that says about
  the ring-gap reasoning behind it.
- Timing at ~92k points/frame against the 100 ms budget — nearly double the VLP-32C's point count,
  so this is where the pipeline's cost scaling shows.
- Whether phase 7's synthetic solid-state claim holds on real data. **If it does not, say so
  plainly** — a refuted claim is the more valuable finding, and stages 2, 6, 7 and 8 all record
  refutations in full.

- [ ] **Step 5: Document the bag workflow in the experiment README**

Add to `experiments/board-detection-2d/README.md`, after the Method E LOO section:

````markdown
### Recorded bags (two-LiDAR: VLP-32C + solid-state Falcon)

The `TWO_LIDAR_*` bags are gitignored — see
[`ros/lctk_sample_data/bags/README.md`](../../ros/lctk_sample_data/bags/README.md).
Export them once (needs ROS, runs outside this venv):

```bash
source /opt/ros/humble/setup.bash
python3 tools/export_bag_npz.py
```

Then benchmark either sensor. The bag rig has its own reference box, so
`--bbox` is required:

```bash
uv run python -m boarddet.benchmark_e_loo --source bag --sensor vlp32 \
  --side 1.0 --stance-gate --flatness-rms-max 0.045 --min-sources 3 \
  --bbox /path/to/bag-rig-bbox.json5 --out results/bagE-vlp32

uv run python -m boarddet.benchmark_e_loo --source bag --sensor falcon \
  --vertical-gap-deg 0 --side 1.0 --stance-gate --flatness-rms-max 0.045 \
  --min-sources 3 --bbox /path/to/bag-rig-bbox.json5 \
  --out results/bagE-falcon
```

`--vertical-gap-deg 0` disables the anisotropic clustering that bridges a
spinning LiDAR's ring gaps; the Falcon is solid-state and has none.
````

- [ ] **Step 6: Verify docs links resolve, then commit**

```bash
cd /home/jetson/LCTK
python3 - <<'EOF'
import re, pathlib
bad = []
files = list(pathlib.Path('docs').rglob('*.md')) + [
    pathlib.Path('experiments/board-detection-2d/README.md'),
    pathlib.Path('ros/lctk_sample_data/bags/README.md')]
for f in files:
    for m in re.finditer(r'\]\(([^)]+\.md)\)', f.read_text()):
        t = (f.parent / m.group(1)).resolve()
        if not t.exists():
            bad.append(f"{f}: {m.group(1)}")
print("BROKEN:", len(bad))
for b in bad: print("  ", b)
EOF
```
Expected: only the two pre-existing breaks in `docs/roadmap/phase-1-message-synchronization.md`
(they point at a `rust/multi-stream-synchronizer/` that does not exist and are unrelated to this
work).

```bash
uv run pytest -q   # from experiments/board-detection-2d
git add docs/roadmap/side-track_method-e-background-subtraction.md \
        experiments/board-detection-2d/README.md
git commit -m "docs(boarddet): solid-state Falcon results and bag workflow

First test of the projection pipeline on a real solid-state LiDAR. Phase 7
had only synthetic uniform-sampling evidence for that case, with its own
caveat that real spinning data was the hard case -- this measures the claim
against the same scene, rig, and board pose as the VLP-32C run, with only
the sensor differing."
```

---

## Out of scope

- **ROS-side use of the bags.** The `lidar_to_lidar_solver` is documented as "not yet tested", and these two-LiDAR bags are exactly what would test it — but that is a separate subsystem with its own pipeline, and it deserves its own plan.
- **Committing bag data**, in git or LFS. Task 1 gitignores it.
- **Within-session multi-pose detection.** The board is static within each bag, so this data does not unlock it.
- **Re-tuning detector gates for the new rig.** Run the established stage-6/stage-8 operating points first and report what they give; tuning against a second rig is a follow-on once there is a baseline.
- **Camera topics.** The bags contain LiDAR only.
