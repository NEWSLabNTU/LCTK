# Assisted Extrinsic Solver Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a third `solver_mode=assisted` to `lidar_to_camera_solver` that auto-queues board poses when they are held still and geometrically new, and serves a browser page for reviewing those pairs with image previews, dropping bad ones, and exporting.

**Architecture:** Three new modules in the existing `lidar_to_camera_solver` package — `stability.py` (ROS-free stillness detector), `preview.py` (ROS-free image decode/annotate/encode plus a latest-frame holder), `review_server.py` (ROS-free Flask app behind a `NodeFacade` protocol). `main.py` gains wiring only. `continuous` and `manual` keep their exact current behaviour.

**Tech Stack:** Python 3.10, rclpy (Humble), OpenCV 4.5 (apt `python3-opencv`), NumPy 1.21, Flask 2.0.1 (apt `python3-flask`), pytest.

**Spec:** [`docs/superpowers/specs/2026-08-31-assisted-extrinsic-solver-design.md`](../specs/2026-08-31-assisted-extrinsic-solver-design.md)

## Global Constraints

- **Never `pip3 install --user` anything.** `CLAUDE.md` Known Issue 3: pip installs of `setuptools`, `numpy`, `scipy` and `anyio` have shadowed apt packages and broken the build four separate times. Flask comes from apt (`python3-flask`, 2.0.1, already installed). Declare it in `package.xml` as `<depend>python3-flask</depend>`, following the existing `python3-json5` pattern.
- **Build with `just build`**, never a raw `colcon build`.
- **Run tests from the repo root** with `python3 -m pytest` (never bare `pytest` — apt's `python3-pytest` ships no `pytest` executable, which is L-28).
- **No hardcoded node defaults for physical or operational values.** Every new tunable is a declared ROS parameter fed from the calibration config.
- **New modules under `ros/` are linted** by `just lint-py` (`ruff check ros/` and `ruff format --check ros/`).
- **`setup.py` uses `find_packages`**, so new `.py` files in `ros/lidar_to_camera_solver/lidar_to_camera_solver/` are picked up with no packaging change.
- **Format strings use named parameters** (`f"{e}"`), per the repo coding guidelines.
- **Threading model:** the node runs on a plain single-threaded `rclpy.spin` with no callback groups. The review server's thread is the *only* other thread that will touch node state. `DetectionBuffer` is internally locked and safe to call from that thread; node-level state (`current_rvec`, `current_tvec`, `last_transform`, `publishing_enabled`, `camera_info`, `_camera_matrix`, `_identity_generation`) requires `self.state_lock`, which is *also* the `DetectionPairSource` admission lock — so never hold it across HTTP or disk I/O.
- **After adding or changing any test recipe, break an assertion deliberately and confirm a non-zero exit** before trusting it (`CLAUDE.md` Testing Practices). Do not read `$?` through a pipe.

---

### Task 1: Replace the hardcoded corner roll with the real convention fix

Filled in below from the corner-order investigation.

---

### Task 2: `StillnessTracker`

**Files:**
- Create: `ros/lidar_to_camera_solver/lidar_to_camera_solver/stability.py`
- Test: `ros/lidar_to_camera_solver/test/test_stability.py`

**Interfaces:**
- Consumes: nothing from earlier tasks.
- Produces:
  - `StillnessState` — frozen dataclass: `is_still: bool`, `should_capture: bool`, `translation_span_m: float`, `rotation_span_deg: float`, `frames: int`, `reason: str`
  - `StillnessTracker(window_frames: int, max_translation_m: float, max_rotation_deg: float, cooldown_s: float)`
  - `StillnessTracker.push(position: tuple[float,float,float], quaternion: tuple[float,float,float,float], stamp_s: float) -> StillnessState`
  - `StillnessTracker.reset() -> None`

Design notes for the implementer: `translation_span_m` is the largest pairwise distance between positions in the window (max minus min per axis is wrong for a diagonal drift; use the max distance from the window mean, doubled, or simply `max` over pairwise norms — the test below pins the value). `rotation_span_deg` is the largest pairwise angle between the window's quaternions. `should_capture` is `is_still` AND the cooldown has elapsed AND the tracker has not already reported a capture for this uninterrupted still stretch; it latches false until an `is_still=False` sample resets it.

- [ ] **Step 1: Write the failing tests**

```python
"""StillnessTracker: is the board being held still enough to capture?

The gate is a span over a sliding window, not a frame-to-frame delta. A board
drifting steadily at 1 mm per frame has a tiny per-frame delta and is not still;
only the span across the whole window sees it.
"""

import math

import pytest

from lidar_to_camera_solver.stability import StillnessState, StillnessTracker

IDENTITY = (0.0, 0.0, 0.0, 1.0)


def make_tracker(**overrides):
    kwargs = dict(
        window_frames=5,
        max_translation_m=0.005,
        max_rotation_deg=0.5,
        cooldown_s=1.0,
    )
    kwargs.update(overrides)
    return StillnessTracker(**kwargs)


def quaternion_about_z(degrees):
    half = math.radians(degrees) / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


def test_window_must_fill_before_any_verdict():
    tracker = make_tracker()
    for index in range(4):
        state = tracker.push((0.0, 0.0, 0.0), IDENTITY, float(index))
        assert not state.is_still
        assert state.frames == index + 1
        assert "filling" in state.reason


def test_a_perfectly_still_board_captures_once():
    tracker = make_tracker()
    states = [tracker.push((1.0, 2.0, 3.0), IDENTITY, float(i)) for i in range(10)]
    captured = [s for s in states if s.should_capture]
    assert len(captured) == 1, "a single uninterrupted hold must capture exactly once"
    assert captured[0].is_still
    assert captured[0].translation_span_m == pytest.approx(0.0)


def test_a_steadily_drifting_board_never_captures():
    tracker = make_tracker()
    # 2 mm per frame: each frame delta is under the 5 mm gate, the window span is not.
    states = [tracker.push((0.002 * i, 0.0, 0.0), IDENTITY, float(i)) for i in range(20)]
    assert not any(s.should_capture for s in states)
    assert not any(s.is_still for s in states)


def test_translation_span_is_measured_across_the_window():
    tracker = make_tracker(max_translation_m=1.0)
    for index in range(4):
        tracker.push((0.0, 0.0, 0.0), IDENTITY, float(index))
    state = tracker.push((0.3, 0.4, 0.0), IDENTITY, 4.0)
    assert state.translation_span_m == pytest.approx(0.5)


def test_rotation_span_is_measured_across_the_window():
    tracker = make_tracker(max_rotation_deg=90.0)
    for index in range(4):
        tracker.push((0.0, 0.0, 0.0), IDENTITY, float(index))
    state = tracker.push((0.0, 0.0, 0.0), quaternion_about_z(30.0), 4.0)
    assert state.rotation_span_deg == pytest.approx(30.0, abs=1e-6)


def test_rotation_alone_breaks_stillness():
    tracker = make_tracker()
    states = [
        tracker.push((0.0, 0.0, 0.0), quaternion_about_z(1.0 * i), float(i))
        for i in range(20)
    ]
    assert not any(s.should_capture for s in states)


def test_exactly_at_the_tolerance_is_still():
    tracker = make_tracker(max_translation_m=0.005)
    for index in range(4):
        tracker.push((0.0, 0.0, 0.0), IDENTITY, float(index))
    state = tracker.push((0.005, 0.0, 0.0), IDENTITY, 4.0)
    assert state.is_still, "the tolerance is inclusive; 5 mm with a 5 mm gate is still"


def test_capture_re_arms_only_after_the_board_moves():
    tracker = make_tracker(cooldown_s=0.0)
    for index in range(10):
        tracker.push((0.0, 0.0, 0.0), IDENTITY, float(index))
    # Move far enough to break the window, then settle again.
    for index in range(10, 20):
        tracker.push((5.0, 0.0, 0.0), IDENTITY, float(index))
    states = [tracker.push((5.0, 0.0, 0.0), IDENTITY, float(i)) for i in range(20, 30)]
    assert not any(s.should_capture for s in states), (
        "the second hold already captured while settling; it must not capture again"
    )


def test_cooldown_suppresses_a_second_capture():
    tracker = make_tracker(cooldown_s=10.0)
    for index in range(10):
        tracker.push((0.0, 0.0, 0.0), IDENTITY, float(index))
    for index in range(10, 20):
        tracker.push((5.0, 0.0, 0.0), IDENTITY, float(index))
    states = [tracker.push((5.0, 0.0, 0.0), IDENTITY, float(i)) for i in range(20, 30)]
    assert not any(s.should_capture for s in states)


def test_reason_names_the_measurement_when_not_still():
    tracker = make_tracker()
    for index in range(5):
        tracker.push((0.02 * index, 0.0, 0.0), IDENTITY, float(index))
    state = tracker.push((0.2, 0.0, 0.0), IDENTITY, 5.0)
    assert "mm" in state.reason
    assert str(state.frames) in state.reason


def test_reset_clears_the_window():
    tracker = make_tracker()
    for index in range(10):
        tracker.push((0.0, 0.0, 0.0), IDENTITY, float(index))
    tracker.reset()
    state = tracker.push((0.0, 0.0, 0.0), IDENTITY, 20.0)
    assert state.frames == 1
    assert not state.is_still


def test_state_is_frozen():
    state = StillnessState(
        is_still=True,
        should_capture=False,
        translation_span_m=0.0,
        rotation_span_deg=0.0,
        frames=5,
        reason="",
    )
    with pytest.raises(Exception):
        state.is_still = False
```

- [ ] **Step 2: Run the tests and confirm they fail**

```bash
cd /home/jetson/LCTK && source /opt/ros/humble/setup.bash
PYTHONPATH="$PWD/ros/lidar_to_camera_solver:$PYTHONPATH" \
  python3 -m pytest ros/lidar_to_camera_solver/test/test_stability.py -q --no-header
```

Expected: collection error, `ModuleNotFoundError: No module named 'lidar_to_camera_solver.stability'`.

- [ ] **Step 3: Implement `stability.py`**

```python
"""Is the calibration board being held still?

A capture session's quality depends on catching the board when it is stationary,
which an operator currently judges by eye before reaching for a key. This module
is that judgement, made from the board pose the LiDAR detector already publishes.

The gate is a **span across a sliding window**, not a frame-to-frame delta. A
board drifting steadily at 1 mm per frame has a negligible per-frame delta and is
plainly not still; only the span over the whole window sees it. Getting this wrong
would auto-capture a slow drift, which is exactly the motion blur the reviewer
would then have to find by hand.

Pure numpy: no ROS, no OpenCV, so the policy is unit-testable without a graph.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class StillnessState:
    """What the tracker saw, and what it wants done about it."""

    is_still: bool
    should_capture: bool
    translation_span_m: float
    rotation_span_deg: float
    frames: int
    reason: str


def _quaternion_angle_deg(a: np.ndarray, b: np.ndarray) -> float:
    """Angle between two unit quaternions, in degrees.

    ``|dot|`` rather than ``dot``: q and -q name the same rotation, so the sign
    must not turn a zero-degree difference into 180.
    """
    dot = float(np.clip(abs(float(np.dot(a, b))), 0.0, 1.0))
    return math.degrees(2.0 * math.acos(dot))


class StillnessTracker:
    """Sliding-window stillness gate with a one-shot capture latch."""

    def __init__(
        self,
        *,
        window_frames: int,
        max_translation_m: float,
        max_rotation_deg: float,
        cooldown_s: float,
    ):
        if window_frames < 2:
            raise ValueError(
                f"window_frames must be at least 2 to have a span; got {window_frames}"
            )
        self._window_frames = window_frames
        self._max_translation_m = max_translation_m
        self._max_rotation_deg = max_rotation_deg
        self._cooldown_s = cooldown_s
        self._positions: deque[np.ndarray] = deque(maxlen=window_frames)
        self._quaternions: deque[np.ndarray] = deque(maxlen=window_frames)
        self._armed = True
        self._last_capture_s: float | None = None

    def reset(self) -> None:
        """Forget the window. Used when the recording restarts under the node."""
        self._positions.clear()
        self._quaternions.clear()
        self._armed = True

    def push(self, position, quaternion, stamp_s: float) -> StillnessState:
        self._positions.append(np.asarray(position, dtype=np.float64))
        quaternion_array = np.asarray(quaternion, dtype=np.float64)
        norm = float(np.linalg.norm(quaternion_array))
        if norm > 0.0:
            quaternion_array = quaternion_array / norm
        self._quaternions.append(quaternion_array)

        frames = len(self._positions)
        if frames < self._window_frames:
            return StillnessState(
                is_still=False,
                should_capture=False,
                translation_span_m=0.0,
                rotation_span_deg=0.0,
                frames=frames,
                reason=f"filling the window: {frames}/{self._window_frames} frames",
            )

        stacked = np.stack(self._positions)
        translation_span = float(
            np.max(np.linalg.norm(stacked[:, None, :] - stacked[None, :, :], axis=-1))
        )
        rotation_span = max(
            _quaternion_angle_deg(a, b)
            for i, a in enumerate(self._quaternions)
            for b in list(self._quaternions)[i + 1 :]
        )

        is_still = (
            translation_span <= self._max_translation_m
            and rotation_span <= self._max_rotation_deg
        )

        if not is_still:
            # The board left the placement, so the next hold is a new one.
            self._armed = True
            return StillnessState(
                is_still=False,
                should_capture=False,
                translation_span_m=translation_span,
                rotation_span_deg=rotation_span,
                frames=frames,
                reason=(
                    f"board moving: {translation_span * 1000:.0f} mm / "
                    f"{rotation_span:.1f} deg over {frames} frames"
                ),
            )

        cooled = (
            self._last_capture_s is None
            or (stamp_s - self._last_capture_s) >= self._cooldown_s
        )
        should_capture = self._armed and cooled
        if should_capture:
            self._armed = False
            self._last_capture_s = stamp_s

        return StillnessState(
            is_still=True,
            should_capture=should_capture,
            translation_span_m=translation_span,
            rotation_span_deg=rotation_span,
            frames=frames,
            reason="held still" if should_capture else "still, already captured",
        )
```

- [ ] **Step 4: Run the tests and confirm they pass**

```bash
cd /home/jetson/LCTK && source /opt/ros/humble/setup.bash
PYTHONPATH="$PWD/ros/lidar_to_camera_solver:$PYTHONPATH" \
  python3 -m pytest ros/lidar_to_camera_solver/test/test_stability.py -q --no-header
```

Expected: all pass.

- [ ] **Step 5: Confirm the suite can actually fail**

Change `max_translation_m=0.005` to `0.5` in `make_tracker`, rerun, confirm a non-zero exit and a failing drift test, then change it back.

- [ ] **Step 6: Lint and commit**

```bash
cd /home/jetson/LCTK
just lint-py
git add ros/lidar_to_camera_solver/lidar_to_camera_solver/stability.py \
        ros/lidar_to_camera_solver/test/test_stability.py
git commit -m "feat(assisted): add the stillness tracker"
```

---

### Task 3: `PreviewStore`

**Files:**
- Create: `ros/lidar_to_camera_solver/lidar_to_camera_solver/preview.py`
- Test: `ros/lidar_to_camera_solver/test/test_preview.py`

**Interfaces:**
- Consumes: nothing from earlier tasks.
- Produces:
  - `decode_image(height: int, width: int, encoding: str, step: int, data: bytes) -> np.ndarray` — BGR, raises `ValueError` on an unsupported encoding
  - `annotate(frame: np.ndarray, corners: Sequence[np.ndarray], reprojected: Sequence[np.ndarray] | None) -> np.ndarray`
  - `encode_jpeg(frame: np.ndarray, quality: int) -> bytes`
  - `PreviewStore(max_previews: int, jpeg_quality: int)` with `set_latest(frame: np.ndarray | None)`, `capture(pair_id: int, corners, reprojected) -> bool`, `get(pair_id: int) -> bytes | None`, `drop(pair_id: int)`, `clear()`

`decode_image` deliberately takes the `sensor_msgs/Image` **fields**, not the message, so the whole module is ROS-free and testable without a graph. This also avoids adding `cv_bridge` as a dependency for what is ten lines of reshaping.

- [ ] **Step 1: Write the failing tests**

```python
"""PreviewStore: the picture a queued pair was measured in.

No solver in this tree subscribes to an image, which is why a bad capture is
currently undiagnosable. These tests pin the two properties that matter: a
preview must never be able to break a capture, and the bytes must be a real JPEG.
"""

import numpy as np
import pytest

from lidar_to_camera_solver.preview import (
    PreviewStore,
    annotate,
    decode_image,
    encode_jpeg,
)


def bgr_frame(height=16, width=24):
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    frame[:, :, 2] = 255  # red in BGR
    return frame


def test_decode_bgr8_round_trips():
    frame = bgr_frame()
    decoded = decode_image(
        height=16, width=24, encoding="bgr8", step=24 * 3, data=frame.tobytes()
    )
    assert decoded.shape == (16, 24, 3)
    assert np.array_equal(decoded, frame)


def test_decode_rgb8_swaps_channels():
    rgb = np.zeros((4, 4, 3), dtype=np.uint8)
    rgb[:, :, 0] = 255  # red in RGB
    decoded = decode_image(
        height=4, width=4, encoding="rgb8", step=12, data=rgb.tobytes()
    )
    assert decoded[0, 0, 2] == 255, "red must land in the BGR red channel"
    assert decoded[0, 0, 0] == 0


def test_decode_mono8_expands_to_three_channels():
    mono = np.full((4, 4), 128, dtype=np.uint8)
    decoded = decode_image(
        height=4, width=4, encoding="mono8", step=4, data=mono.tobytes()
    )
    assert decoded.shape == (4, 4, 3)
    assert np.all(decoded == 128)


def test_decode_honours_row_padding():
    # step larger than width*channels: rows are padded, which naive reshaping ignores.
    padded = np.zeros((4, 10 * 3), dtype=np.uint8)
    padded[:, : 4 * 3] = 7
    decoded = decode_image(
        height=4, width=4, encoding="bgr8", step=30, data=padded.tobytes()
    )
    assert decoded.shape == (4, 4, 3)
    assert np.all(decoded == 7)


def test_decode_rejects_an_unsupported_encoding():
    with pytest.raises(ValueError, match="bayer_rggb8"):
        decode_image(height=2, width=2, encoding="bayer_rggb8", step=2, data=b"\x00" * 4)


def test_encode_jpeg_produces_jpeg_magic_bytes():
    data = encode_jpeg(bgr_frame(), quality=80)
    assert data[:2] == b"\xff\xd8", "JPEG SOI marker"
    assert data[-2:] == b"\xff\xd9", "JPEG EOI marker"


def test_annotate_does_not_mutate_the_input_frame():
    frame = bgr_frame()
    original = frame.copy()
    annotate(frame, [np.array([[1.0, 1.0], [5.0, 1.0], [5.0, 5.0], [1.0, 5.0]])], None)
    assert np.array_equal(frame, original)


def test_annotate_draws_something():
    frame = bgr_frame()
    marked = annotate(
        frame, [np.array([[1.0, 1.0], [5.0, 1.0], [5.0, 5.0], [1.0, 5.0]])], None
    )
    assert not np.array_equal(marked, frame)


def test_capture_without_a_frame_reports_failure_and_does_not_raise():
    store = PreviewStore(max_previews=4, jpeg_quality=80)
    assert store.capture(1, corners=[], reprojected=None) is False
    assert store.get(1) is None


def test_capture_stores_retrievable_jpeg_bytes():
    store = PreviewStore(max_previews=4, jpeg_quality=80)
    store.set_latest(bgr_frame())
    assert store.capture(7, corners=[], reprojected=None) is True
    assert store.get(7)[:2] == b"\xff\xd8"


def test_store_evicts_the_oldest_beyond_the_bound():
    store = PreviewStore(max_previews=2, jpeg_quality=80)
    store.set_latest(bgr_frame())
    for pair_id in (1, 2, 3):
        store.capture(pair_id, corners=[], reprojected=None)
    assert store.get(1) is None, "oldest must be evicted"
    assert store.get(2) is not None
    assert store.get(3) is not None


def test_drop_and_clear():
    store = PreviewStore(max_previews=4, jpeg_quality=80)
    store.set_latest(bgr_frame())
    store.capture(1, corners=[], reprojected=None)
    store.drop(1)
    assert store.get(1) is None
    store.capture(2, corners=[], reprojected=None)
    store.clear()
    assert store.get(2) is None
```

- [ ] **Step 2: Run the tests and confirm they fail**

```bash
cd /home/jetson/LCTK && source /opt/ros/humble/setup.bash
PYTHONPATH="$PWD/ros/lidar_to_camera_solver:$PYTHONPATH" \
  python3 -m pytest ros/lidar_to_camera_solver/test/test_preview.py -q --no-header
```

Expected: `ModuleNotFoundError: No module named 'lidar_to_camera_solver.preview'`.

- [ ] **Step 3: Implement `preview.py`**

```python
"""The camera frame a queued pair was measured in.

No solver in this tree subscribes to an image, so when a capture turns out bad --
motion blur, an occluded marker, glare across the plate -- nothing downstream can
say why. This module keeps the latest frame, and snapshots it when a pair is
queued, with the detected corners drawn on.

``decode_image`` takes the ``sensor_msgs/Image`` **fields** rather than the
message, so this module imports no ROS and its tests need no graph. It also means
``cv_bridge`` is not a dependency for what amounts to a reshape.

**A preview must never be able to break a capture.** Every failure path here
returns falsy rather than raising; calibration correctness does not depend on a
picture being available.
"""

from __future__ import annotations

from collections import OrderedDict
from collections.abc import Sequence
import threading

import cv2
import numpy as np

_CORNER_COLOR = (0, 255, 0)
_REPROJECTED_COLOR = (0, 128, 255)


def decode_image(
    *, height: int, width: int, encoding: str, step: int, data: bytes
) -> np.ndarray:
    """One ``sensor_msgs/Image`` as an HxWx3 BGR array.

    ``step`` is honoured rather than assumed: a padded row stride silently
    shears the image if you reshape by ``width`` alone.
    """
    encoding = encoding.lower()
    if encoding in ("bgr8", "rgb8"):
        channels = 3
    elif encoding == "mono8":
        channels = 1
    else:
        raise ValueError(
            f"unsupported image encoding '{encoding}'; "
            "preview supports bgr8, rgb8 and mono8"
        )

    buffer = np.frombuffer(data, dtype=np.uint8)
    expected = step * height
    if buffer.size < expected:
        raise ValueError(
            f"image data is {buffer.size} bytes; {expected} required for "
            f"{height} rows of stride {step}"
        )
    rows = buffer[:expected].reshape(height, step)
    frame = rows[:, : width * channels].reshape(height, width, channels)

    if encoding == "rgb8":
        return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    if encoding == "mono8":
        return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    return frame.copy()


def annotate(
    frame: np.ndarray,
    corners: Sequence[np.ndarray],
    reprojected: Sequence[np.ndarray] | None,
) -> np.ndarray:
    """A copy of ``frame`` with detected corners and reprojected points drawn.

    Never mutates the input: the latest-frame buffer is shared, and drawing into
    it would corrupt the next capture.
    """
    canvas = frame.copy()
    for quad in corners:
        points = np.asarray(quad, dtype=np.int32).reshape(-1, 1, 2)
        cv2.polylines(canvas, [points], isClosed=True, color=_CORNER_COLOR, thickness=1)
        # Mark corner 0 so a quarter-turn in the correspondence order is visible.
        first = tuple(int(v) for v in np.asarray(quad, dtype=np.float64)[0])
        cv2.circle(canvas, first, 3, _CORNER_COLOR, -1)
    for point in reprojected or ():
        x, y = (int(v) for v in np.asarray(point, dtype=np.float64).ravel()[:2])
        cv2.drawMarker(
            canvas, (x, y), _REPROJECTED_COLOR, cv2.MARKER_CROSS, markerSize=6
        )
    return canvas


def encode_jpeg(frame: np.ndarray, quality: int) -> bytes:
    ok, buffer = cv2.imencode(
        ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)]
    )
    if not ok:
        raise ValueError("cv2.imencode refused the frame")
    return buffer.tobytes()


class PreviewStore:
    """Latest camera frame, plus a bounded cache of per-pair JPEG snapshots."""

    def __init__(self, *, max_previews: int, jpeg_quality: int):
        self._max_previews = max_previews
        self._jpeg_quality = jpeg_quality
        self._latest: np.ndarray | None = None
        self._previews: OrderedDict[int, bytes] = OrderedDict()
        self._lock = threading.Lock()

    def set_latest(self, frame: np.ndarray | None) -> None:
        """Store the newest frame. Called from the subscription callback, so it
        does no work beyond the assignment."""
        with self._lock:
            self._latest = frame

    def capture(self, pair_id: int, corners, reprojected) -> bool:
        """Snapshot the latest frame against ``pair_id``. False if there is none."""
        with self._lock:
            frame = self._latest
        if frame is None:
            return False
        try:
            data = encode_jpeg(annotate(frame, corners, reprojected), self._jpeg_quality)
        except (ValueError, cv2.error):
            return False
        with self._lock:
            self._previews[pair_id] = data
            self._previews.move_to_end(pair_id)
            while len(self._previews) > self._max_previews:
                self._previews.popitem(last=False)
        return True

    def get(self, pair_id: int) -> bytes | None:
        with self._lock:
            return self._previews.get(pair_id)

    def drop(self, pair_id: int) -> None:
        with self._lock:
            self._previews.pop(pair_id, None)

    def clear(self) -> None:
        with self._lock:
            self._previews.clear()
```

- [ ] **Step 4: Run the tests and confirm they pass**

```bash
cd /home/jetson/LCTK && source /opt/ros/humble/setup.bash
PYTHONPATH="$PWD/ros/lidar_to_camera_solver:$PYTHONPATH" \
  python3 -m pytest ros/lidar_to_camera_solver/test/test_preview.py -q --no-header
```

Expected: all pass.

- [ ] **Step 5: Confirm the suite can actually fail**

Delete the `[:, : width * channels]` slice in `decode_image`, rerun, confirm `test_decode_honours_row_padding` fails and the exit code is non-zero, then restore it.

- [ ] **Step 6: Lint and commit**

```bash
cd /home/jetson/LCTK
just lint-py
git add ros/lidar_to_camera_solver/lidar_to_camera_solver/preview.py \
        ros/lidar_to_camera_solver/test/test_preview.py
git commit -m "feat(assisted): add the preview store"
```

---

### Task 4: The review server

**Files:**
- Create: `ros/lidar_to_camera_solver/lidar_to_camera_solver/review_server.py`
- Test: `ros/lidar_to_camera_solver/test/test_review_server.py`
- Modify: `ros/lidar_to_camera_solver/package.xml` (add `<depend>python3-flask</depend>`)

**Interfaces:**
- Consumes: nothing from earlier tasks (the facade is a protocol, not an import).
- Produces:
  - `NodeFacade` — `Protocol` with `state() -> dict`, `preview(pair_id) -> bytes | None`, `drop(pair_id) -> tuple[bool, str]`, `export_archive(path) -> tuple[bool, str]`, `export_autoware(dry_run: bool) -> tuple[bool, str, dict | None]`
  - `create_app(facade: NodeFacade) -> flask.Flask`
  - `ReviewServer(facade, host: str, port: int)` with `start()`, `shutdown()`, `port` property

There is deliberately **no `/api/resolve`**: `DetectionBuffer` re-derives the solve inside every mutation, so dropping a pair already re-solves. Adding a separate endpoint would imply a second, unrelated code path that does not exist.

- [ ] **Step 1: Write the failing tests**

```python
"""The review server, exercised through Flask's test client.

The whole point of the NodeFacade seam is that these tests need no ROS graph, no
node, and no camera. If a test here needs rclpy, the seam has leaked.
"""

import json

import pytest

from lidar_to_camera_solver.review_server import create_app


class FakeFacade:
    def __init__(self):
        self.dropped = []
        self.exported = []
        self.autoware_calls = []
        self._state = {
            "mode": "assisted",
            "sync": "sync: groups=12",
            "stillness": {"is_still": True, "reason": "held still", "frames": 5},
            "diversity": {"n_placements": 2, "shortfalls": ["move the board"]},
            "solve": {"status": "solved", "rms_px": 0.5},
            "pairs": [{"id": 1, "rms_px": 0.5, "has_preview": True}],
            "export": {"archive_path": "/tmp/detections.json", "autoware_ready": True},
        }
        self._previews = {1: b"\xff\xd8fakejpeg\xff\xd9"}

    def state(self):
        return self._state

    def preview(self, pair_id):
        return self._previews.get(pair_id)

    def drop(self, pair_id):
        if pair_id not in self._previews:
            return False, f"no pair {pair_id}"
        self.dropped.append(pair_id)
        return True, "dropped"

    def export_archive(self, path):
        self.exported.append(path)
        return True, f"wrote {path}"

    def export_autoware(self, dry_run):
        self.autoware_calls.append(dry_run)
        return True, "ok", {"x": 1.0, "y": 2.0}


@pytest.fixture
def client():
    facade = FakeFacade()
    app = create_app(facade)
    app.config["TESTING"] = True
    with app.test_client() as test_client:
        test_client.facade = facade
        yield test_client


def test_index_serves_a_self_contained_page(client):
    response = client.get("/")
    assert response.status_code == 200
    body = response.data.decode()
    assert "<html" in body.lower()
    assert "http://" not in body.replace("http://www.w3.org", ""), (
        "the page must not reference any external host; the rig has no internet"
    )


def test_state_is_returned_verbatim(client):
    response = client.get("/api/state")
    assert response.status_code == 200
    assert json.loads(response.data) == client.facade.state()


def test_preview_returns_jpeg(client):
    response = client.get("/api/pair/1/preview.jpg")
    assert response.status_code == 200
    assert response.mimetype == "image/jpeg"
    assert response.data.startswith(b"\xff\xd8")


def test_missing_preview_is_404_not_500(client):
    assert client.get("/api/pair/99/preview.jpg").status_code == 404


def test_drop_calls_the_facade(client):
    response = client.post("/api/pair/1/drop")
    assert response.status_code == 200
    assert json.loads(response.data)["ok"] is True
    assert client.facade.dropped == [1]


def test_drop_of_an_unknown_pair_reports_failure_without_raising(client):
    response = client.post("/api/pair/99/drop")
    assert response.status_code == 200
    payload = json.loads(response.data)
    assert payload["ok"] is False
    assert "99" in payload["detail"]


def test_export_archive_passes_the_path(client):
    response = client.post(
        "/api/export/archive",
        data=json.dumps({"path": "/tmp/out.json"}),
        content_type="application/json",
    )
    assert json.loads(response.data)["ok"] is True
    assert client.facade.exported == ["/tmp/out.json"]


def test_export_archive_requires_a_path(client):
    response = client.post(
        "/api/export/archive", data=json.dumps({}), content_type="application/json"
    )
    payload = json.loads(response.data)
    assert payload["ok"] is False
    assert "path" in payload["detail"]


def test_autoware_preview_does_not_write(client):
    response = client.post("/api/export/autoware/preview")
    payload = json.loads(response.data)
    assert payload["ok"] is True
    assert payload["entry"] == {"x": 1.0, "y": 2.0}
    assert client.facade.autoware_calls == [True], "preview must be a dry run"


def test_autoware_write_is_refused_before_a_preview(client):
    response = client.post("/api/export/autoware/write")
    payload = json.loads(response.data)
    assert payload["ok"] is False
    assert "preview" in payload["detail"].lower()
    assert client.facade.autoware_calls == [], "nothing may be written unseen"


def test_autoware_write_is_allowed_after_a_preview(client):
    client.post("/api/export/autoware/preview")
    response = client.post("/api/export/autoware/write")
    assert json.loads(response.data)["ok"] is True
    assert client.facade.autoware_calls == [True, False]


def test_a_drop_invalidates_a_pending_autoware_confirmation(client):
    client.post("/api/export/autoware/preview")
    client.post("/api/pair/1/drop")
    response = client.post("/api/export/autoware/write")
    payload = json.loads(response.data)
    assert payload["ok"] is False, (
        "the buffer changed after the diff was shown, so the confirmation is stale"
    )
    assert client.facade.autoware_calls == [True]
```

- [ ] **Step 2: Run the tests and confirm they fail**

```bash
cd /home/jetson/LCTK && source /opt/ros/humble/setup.bash
PYTHONPATH="$PWD/ros/lidar_to_camera_solver:$PYTHONPATH" \
  python3 -m pytest ros/lidar_to_camera_solver/test/test_review_server.py -q --no-header
```

Expected: `ModuleNotFoundError: No module named 'lidar_to_camera_solver.review_server'`.

- [ ] **Step 3: Implement `review_server.py`**

```python
"""The capture-review page and its JSON API.

Everything here talks to the node through :class:`NodeFacade`, never to ROS
types, so the whole module is testable with Flask's test client and a fake. The
server runs on its own thread; the facade is what makes that safe, because the
node implements it with the correct locking on the other side.

The Autoware export is deliberately two requests. It writes a file that reaches a
vehicle, so the operator sees the diff first and confirms second, and a buffer
change between the two invalidates the confirmation.
"""

from __future__ import annotations

import threading
from typing import Any, Protocol

from flask import Flask, Response, jsonify, request
from werkzeug.serving import make_server

_PAGE = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>LCTK assisted capture</title>
<style>
 :root { color-scheme: light dark; }
 body { font: 14px/1.45 system-ui, sans-serif; margin: 0; padding: 1rem; }
 h1 { font-size: 1.1rem; margin: 0 0 .75rem; }
 .banner { padding: .6rem .8rem; border-radius: .4rem; margin-bottom: .8rem;
           background: #7773; }
 .banner.still { background: #2b8a3e33; }
 .shortfall { margin: .15rem 0; opacity: .85; }
 .pair { display: flex; gap: .8rem; align-items: center;
         border-top: 1px solid #8884; padding: .5rem 0; }
 .pair img { width: 220px; border-radius: .25rem; background: #8882; }
 .worst { outline: 2px solid #e0348044; }
 button { font: inherit; padding: .35rem .7rem; border-radius: .3rem; }
 .sync { opacity: .7; font-size: .85em; }
 pre { background: #8882; padding: .5rem; border-radius: .3rem; overflow-x: auto; }
</style>
</head>
<body>
<h1>LCTK assisted capture <span class="sync" id="sync"></span></h1>
<div class="banner" id="banner">connecting…</div>
<div id="diversity"></div>
<div id="solve"></div>
<div id="pairs"></div>
<p>
  <button onclick="exportArchive()">Export archive</button>
  <button onclick="autowarePreview()">Export to Autoware…</button>
</p>
<div id="autoware"></div>
<script>
let archivePath = "";
async function refresh() {
  const state = await (await fetch("/api/state")).json();
  archivePath = state.export.archive_path || "";
  document.getElementById("sync").textContent = state.sync || "";
  const banner = document.getElementById("banner");
  banner.textContent = state.stillness.reason || "";
  banner.className = "banner" + (state.stillness.is_still ? " still" : "");
  document.getElementById("diversity").innerHTML =
    (state.diversity.shortfalls || [])
      .map(s => '<div class="shortfall">· ' + s + "</div>").join("")
    || '<div class="shortfall">diversity targets met</div>';
  const solve = state.solve || {};
  document.getElementById("solve").textContent =
    "solve: " + (solve.status || "?") +
    (solve.rms_px != null ? "  RMS " + solve.rms_px.toFixed(2) + " px" : "") +
    (solve.detail ? "  " + solve.detail : "");
  const pairs = (state.pairs || []).slice().sort(
    (a, b) => (b.rms_px || 0) - (a.rms_px || 0));
  document.getElementById("pairs").innerHTML = pairs.map((p, i) =>
    '<div class="pair' + (i === 0 && pairs.length > 1 ? ' worst' : '') + '">' +
    (p.has_preview
      ? '<img src="/api/pair/' + p.id + '/preview.jpg?v=' + p.id + '">'
      : '<img alt="no frame">') +
    '<div>#' + p.id +
    (p.rms_px != null ? '<br>' + p.rms_px.toFixed(2) + ' px' : '') + '</div>' +
    '<button onclick="dropPair(' + p.id + ')">drop</button></div>').join("");
}
async function post(url, body) {
  const response = await fetch(url, {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify(body || {}),
  });
  return response.json();
}
async function dropPair(id) { await post("/api/pair/" + id + "/drop"); refresh(); }
async function exportArchive() {
  const result = await post("/api/export/archive", {path: archivePath});
  alert(result.detail);
}
async function autowarePreview() {
  const result = await post("/api/export/autoware/preview");
  const box = document.getElementById("autoware");
  if (!result.ok) { box.textContent = result.detail; return; }
  box.innerHTML = "<pre>" + JSON.stringify(result.entry, null, 2) + "</pre>" +
    '<button onclick="autowareWrite()">Confirm write</button>';
}
async function autowareWrite() {
  const result = await post("/api/export/autoware/write");
  document.getElementById("autoware").textContent = result.detail;
}
setInterval(refresh, 500);
refresh();
</script>
</body>
</html>
"""


class NodeFacade(Protocol):
    """What the server needs from the node. Plain data in, plain data out.

    No method raises: failures come back as ``(False, reason)`` so an operator
    sees the reason on the page instead of a stack trace in a log.
    """

    def state(self) -> dict[str, Any]: ...

    def preview(self, pair_id: int) -> bytes | None: ...

    def drop(self, pair_id: int) -> tuple[bool, str]: ...

    def export_archive(self, path: str) -> tuple[bool, str]: ...

    def export_autoware(self, dry_run: bool) -> tuple[bool, str, dict | None]: ...


def create_app(facade: NodeFacade) -> Flask:
    app = Flask(__name__)
    # The confirmation token for the Autoware write: the buffer revision the
    # operator was shown a diff for. Any mutation clears it.
    pending: dict[str, Any] = {"previewed": False}

    @app.get("/")
    def index() -> Response:
        return Response(_PAGE, mimetype="text/html")

    @app.get("/api/state")
    def state() -> Response:
        return jsonify(facade.state())

    @app.get("/api/pair/<int:pair_id>/preview.jpg")
    def preview(pair_id: int) -> Response:
        data = facade.preview(pair_id)
        if data is None:
            return Response("no preview for that pair", status=404)
        return Response(data, mimetype="image/jpeg")

    @app.post("/api/pair/<int:pair_id>/drop")
    def drop(pair_id: int) -> Response:
        ok, detail = facade.drop(pair_id)
        if ok:
            # The diff the operator was shown described a different buffer.
            pending["previewed"] = False
        return jsonify({"ok": ok, "detail": detail})

    @app.post("/api/export/archive")
    def export_archive() -> Response:
        payload = request.get_json(silent=True) or {}
        path = payload.get("path")
        if not path:
            return jsonify({"ok": False, "detail": "no 'path' given for the archive"})
        ok, detail = facade.export_archive(path)
        return jsonify({"ok": ok, "detail": detail})

    @app.post("/api/export/autoware/preview")
    def autoware_preview() -> Response:
        ok, detail, entry = facade.export_autoware(dry_run=True)
        pending["previewed"] = ok
        return jsonify({"ok": ok, "detail": detail, "entry": entry})

    @app.post("/api/export/autoware/write")
    def autoware_write() -> Response:
        if not pending["previewed"]:
            return jsonify(
                {
                    "ok": False,
                    "detail": "preview the Autoware diff first; nothing is written "
                    "unseen, and a buffer change invalidates an earlier preview",
                    "entry": None,
                }
            )
        ok, detail, entry = facade.export_autoware(dry_run=False)
        pending["previewed"] = False
        return jsonify({"ok": ok, "detail": detail, "entry": entry})

    return app


class ReviewServer:
    """The Flask app on a daemon thread, startable and stoppable by the node."""

    def __init__(self, facade: NodeFacade, *, host: str, port: int):
        self._server = make_server(host, port, create_app(facade), threaded=True)
        self._thread = threading.Thread(
            target=self._server.serve_forever, name="lctk-review", daemon=True
        )

    @property
    def port(self) -> int:
        return self._server.server_port

    def start(self) -> None:
        self._thread.start()

    def shutdown(self) -> None:
        self._server.shutdown()
        self._thread.join(timeout=2.0)
```

- [ ] **Step 4: Run the tests and confirm they pass**

```bash
cd /home/jetson/LCTK && source /opt/ros/humble/setup.bash
PYTHONPATH="$PWD/ros/lidar_to_camera_solver:$PYTHONPATH" \
  python3 -m pytest ros/lidar_to_camera_solver/test/test_review_server.py -q --no-header
```

Expected: all pass.

- [ ] **Step 5: Declare the Flask dependency**

In `ros/lidar_to_camera_solver/package.xml`, add alongside the other `python3-*` entries:

```xml
  <depend>python3-flask</depend>
```

- [ ] **Step 6: Confirm the suite can actually fail**

Make `autoware_write` skip the `pending["previewed"]` check, rerun, confirm `test_autoware_write_is_refused_before_a_preview` fails with a non-zero exit, then restore it.

- [ ] **Step 7: Lint and commit**

```bash
cd /home/jetson/LCTK
just lint-py
git add ros/lidar_to_camera_solver/lidar_to_camera_solver/review_server.py \
        ros/lidar_to_camera_solver/test/test_review_server.py \
        ros/lidar_to_camera_solver/package.xml
git commit -m "feat(assisted): add the review server and its two-step Autoware export"
```

---

### Task 5: Wire `assisted` into the node

**Files:**
- Modify: `ros/lidar_to_camera_solver/lidar_to_camera_solver/main.py`
- Modify: `ros/lidar_to_camera_solver/test/test_detection_buffer.py:127-135`
- Modify: `ros/lidar_to_camera_solver/test/test_identity_node_contract.py` (harness state)
- Test: `ros/lidar_to_camera_solver/test/test_assisted_mode.py`

**Interfaces:**
- Consumes: `StillnessTracker`, `StillnessState` (Task 2); `PreviewStore`, `decode_image` (Task 3); `ReviewServer`, `NodeFacade` (Task 4).
- Produces: `SOLVER_MODES = ("continuous", "manual", "assisted")`; `LidarToCameraSolver._assisted_pair_callback`; `LidarToCameraSolver` implementing the `NodeFacade` methods.

Reference points in the current file, from the plumbing survey:
`SOLVER_MODES` at `:65`; `parse_solver_mode` at `:104-109`; `solve_min_frames` branch at `:137-139`; `DetectionPairSource(on_pair=...)` at `:222-225`; service branch at `:238-241`; parameter table at `:250-274`; `state_lock` at `:174`; `camera_info` subscription at `:231-236`; reusable renderers `_status_text` at `:483-503` and `_rejection_text` at `:505-524`; dump/load bodies at `:708-908` (the archive-writing code the facade reuses).

Behaviour required of `assisted`:
- `solve_min_frames` uses `min_frames_required`, exactly like `manual` (it is a multi-pose buffer).
- `on_pair=self._assisted_pair_callback`, so pairs are pushed, not polled.
- The manual services **are** created too, so `interactive_solver_controller` still attaches and the existing dump/load path is reachable.
- The callback: push the board pose into the `StillnessTracker`; if `should_capture`, check novelty via the buffer's `added_new_placement` on `capture`, and if the placement was not new, undo with `remove`. Snapshot a preview against the new pair index.
- `pair_source.epoch_resets` increasing calls `tracker.reset()`.

- [ ] **Step 1: Write the failing tests**

```python
"""assisted mode: the third solver_mode.

These tests build the node with ``object.__new__`` and set only the attributes
under test, the same way test_identity_node_contract.py does -- no ROS graph.
"""

import pytest

from lidar_to_camera_solver.main import SOLVER_MODES, parse_solver_mode


def test_assisted_is_a_valid_mode():
    assert parse_solver_mode("assisted") == "assisted"


def test_the_three_modes_are_exactly_these():
    assert SOLVER_MODES == ("continuous", "manual", "assisted")


def test_an_unknown_mode_names_all_three():
    with pytest.raises(ValueError, match="continuous', 'manual', 'assisted"):
        parse_solver_mode("automatic")
```

Also update the two existing tests the survey identified:

```python
# ros/lidar_to_camera_solver/test/test_detection_buffer.py:127
@pytest.mark.parametrize("mode", ["continuous", "manual", "assisted"])
def test_solver_mode_accepts_only_named_behaviours(mode):
    assert parse_solver_mode(mode) == mode


# ros/lidar_to_camera_solver/test/test_detection_buffer.py:132
@pytest.mark.parametrize("mode", ["", "standard", "advanced", "true"])
def test_solver_mode_rejects_removed_or_unknown_values(mode):
    with pytest.raises(ValueError, match="expected 'continuous', 'manual', 'assisted'"):
        parse_solver_mode(mode)
```

- [ ] **Step 2: Run the tests and confirm they fail**

```bash
cd /home/jetson/LCTK && source /opt/ros/humble/setup.bash
PYTHONPATH="$PWD/ros/lidar_to_camera_solver:$PYTHONPATH" \
  python3 -m pytest ros/lidar_to_camera_solver/test/test_assisted_mode.py \
                   ros/lidar_to_camera_solver/test/test_detection_buffer.py -q --no-header
```

Expected: `test_assisted_is_a_valid_mode` fails with `Invalid solver_mode 'assisted'`.

- [ ] **Step 3: Extend the mode tuple and the parameter table**

In `main.py`, change `:65`:

```python
SOLVER_MODES = ("continuous", "manual", "assisted")
```

Add to the `_declare_parameters` tuple list (`:250-274`), following the existing `(name, default)` shape:

```python
        ("stability_window_frames", 10),
        ("stability_max_translation_m", 0.005),
        ("stability_max_rotation_deg", 0.5),
        ("stability_cooldown_s", 1.0),
        ("novelty_position_tol_m", 0.05),
        ("novelty_orientation_tol_deg", 5.0),
        ("review_bind_host", "127.0.0.1"),
        ("review_port", 8080),
        ("review_jpeg_quality", 80),
        ("review_max_previews", 64),
        ("review_archive_path", ""),
        ("export_autoware_target", ""),
        ("export_camera_frame", ""),
        ("export_lidar_frame", ""),
```

- [ ] **Step 4: Run the mode tests and confirm they pass**

```bash
cd /home/jetson/LCTK && source /opt/ros/humble/setup.bash
PYTHONPATH="$PWD/ros/lidar_to_camera_solver:$PYTHONPATH" \
  python3 -m pytest ros/lidar_to_camera_solver/test/test_assisted_mode.py -q --no-header
```

Expected: all pass.

- [ ] **Step 5: Wire the subsystems into `__init__`**

Insert after the `camera_info` subscription (`:236`) and before the service branch (`:238`):

```python
        self._stillness = None
        self._preview_store = None
        self._review_server = None
        if self.solver_mode == "assisted":
            self._stillness = StillnessTracker(
                window_frames=int(self._parameter("stability_window_frames")),
                max_translation_m=float(self._parameter("stability_max_translation_m")),
                max_rotation_deg=float(self._parameter("stability_max_rotation_deg")),
                cooldown_s=float(self._parameter("stability_cooldown_s")),
            )
            self._preview_store = PreviewStore(
                max_previews=int(self._parameter("review_max_previews")),
                jpeg_quality=int(self._parameter("review_jpeg_quality")),
            )
            # The image is for the reviewer, never for the solve, so it takes the
            # same QoS as the detections and keeps only the newest frame.
            self.image_subscription = self.create_subscription(
                Image, self.camera_topic, self._image_callback, qos_profile
            )
            host = self._string_parameter("review_bind_host")
            self._review_server = ReviewServer(
                self, host=host, port=int(self._parameter("review_port"))
            )
            self._review_server.start()
            if host not in ("127.0.0.1", "localhost"):
                self.get_logger().warning(
                    f"review server bound to {host}:{self._review_server.port} -- "
                    "the queue, the camera previews and the solved extrinsic are "
                    "readable by anyone who can reach that port, and there is no "
                    "authentication"
                )
            else:
                self.get_logger().info(
                    f"review server on http://{host}:{self._review_server.port}"
                )
```

Change the service branch (`:238-241`) so assisted also gets the services:

```python
        if self.solver_mode in ("manual", "assisted"):
            self._create_services()
        else:
            self._services = []
```

Change the `on_pair` selection (`:222-225`):

```python
                on_pair=(
                    self._continuous_pair_callback
                    if self.solver_mode == "continuous"
                    else self._assisted_pair_callback
                    if self.solver_mode == "assisted"
                    else None
                ),
```

The image callback stays trivial, per the `ArcSwap` guidance in `CLAUDE.md`:

```python
    def _image_callback(self, message):
        """Store the newest frame and return. Decoding happens at capture time."""
        try:
            frame = decode_image(
                height=message.height,
                width=message.width,
                encoding=message.encoding,
                step=message.step,
                data=bytes(message.data),
            )
        except ValueError as error:
            self.get_logger().warning(f"preview disabled for this frame: {error}")
            return
        self._preview_store.set_latest(frame)
```

- [ ] **Step 6: Implement the assisted capture callback**

```python
    def _assisted_pair_callback(self, messages):
        """Auto-capture a pair when the board is held still in a new placement.

        Two gates, and both are load-bearing. Stillness stops motion blur.
        Novelty stops the degenerate capture -- one placement filmed forty times --
        that lctk_quality.diversity exists to detect and that every residual-based
        number rates as excellent.
        """
        aruco, board = messages
        pose = self._board_pose(board)
        if pose is None:
            return
        position, orientation = pose
        stamp = self.get_clock().now().nanoseconds * 1e-9
        state = self._stillness.push(position, orientation, stamp)
        self._last_stillness = state
        if not state.should_capture:
            return

        with self.state_lock:
            generation = self._identity_generation
            buffer = self.detection_buffer
            if buffer is None or not self.identity_gate.is_open:
                return
            update = buffer.capture(DetectionPair(aruco=aruco, board=board))
            if not update.accepted:
                self.get_logger().debug(
                    f"assisted capture refused: {self._rejection_text(update)}"
                )
                return
            if update.added_new_placement is False:
                # Still, but not a new placement. Undo rather than pad the buffer.
                buffer.remove(update.snapshot.frame_count - 1)
                return
            pair_id = update.snapshot.frame_count - 1

        self._preview_store.capture(
            pair_id, corners=self._aruco_corners(aruco), reprojected=None
        )
        self._apply_update(update, expected_generation=generation)
```

- [ ] **Step 7: Implement the `NodeFacade` methods on the node**

```python
    # --- NodeFacade -----------------------------------------------------------
    # The review server calls these from its own thread. DetectionBuffer is
    # internally locked, so snapshots need no node lock; anything touching node
    # state takes state_lock, and nothing holds it across disk I/O.

    def state(self) -> dict:
        snapshot = self._snapshot()
        stillness = self._last_stillness
        diversity = compute_diversity(snapshot.placements)
        estimate = snapshot.estimate
        return {
            "mode": self.solver_mode,
            "sync": self.pair_source.status_line(),
            "stillness": {
                "is_still": bool(stillness and stillness.is_still),
                "reason": stillness.reason if stillness else "waiting for detections",
                "frames": stillness.frames if stillness else 0,
            },
            "diversity": (
                {
                    "n_placements": diversity.n_placements,
                    "normal_span_deg": diversity.normal_span_deg,
                    "depth_range_m": diversity.depth_range_m,
                    "lateral_span_m": diversity.lateral_span_m,
                    "is_degenerate": diversity.is_degenerate,
                    "shortfalls": diversity.shortfalls(),
                }
                if diversity is not None
                else {"n_placements": 0, "shortfalls": ["no placements yet"]}
            ),
            "solve": {
                "status": self._status_text(snapshot),
                "rms_px": (
                    estimate.quality.residuals.rms_px if estimate is not None else None
                ),
            },
            "pairs": [
                {
                    "id": index,
                    "rms_px": (
                        estimate.quality.residuals.per_pose_rms_px[index]
                        if estimate is not None
                        and index < len(estimate.quality.residuals.per_pose_rms_px)
                        else None
                    ),
                    "has_preview": self._preview_store.get(index) is not None,
                }
                for index in range(snapshot.frame_count)
            ],
            "export": {
                "archive_path": self._string_parameter("review_archive_path"),
                "autoware_ready": bool(
                    self._string_parameter("export_autoware_target")
                    and self._string_parameter("export_camera_frame")
                    and self._string_parameter("export_lidar_frame")
                ),
            },
        }

    def preview(self, pair_id: int) -> bytes | None:
        return self._preview_store.get(pair_id)

    def drop(self, pair_id: int) -> tuple[bool, str]:
        with self.state_lock:
            buffer = self.detection_buffer
            if buffer is None:
                return False, "no buffer"
            generation = self._identity_generation
            update = buffer.remove(pair_id)
        if not update.accepted:
            return False, self._rejection_text(update)
        self._preview_store.drop(pair_id)
        self._apply_update(update, expected_generation=generation)
        return True, f"dropped pair {pair_id}"

    def export_archive(self, path: str) -> tuple[bool, str]:
        request = DumpDetections.Request()
        request.file_path = path
        response = self.dump_detections_callback(request, DumpDetections.Response())
        return bool(response.success), response.message

    def export_autoware(self, dry_run: bool) -> tuple[bool, str, dict | None]:
        target = self._string_parameter("export_autoware_target")
        camera_frame = self._string_parameter("export_camera_frame")
        lidar_frame = self._string_parameter("export_lidar_frame")
        missing = [
            name
            for name, value in (
                ("export_autoware_target", target),
                ("export_camera_frame", camera_frame),
                ("export_lidar_frame", lidar_frame),
            )
            if not value
        ]
        if missing:
            return False, f"unset parameter(s): {', '.join(missing)}", None
        snapshot = self._snapshot()
        estimate = snapshot.estimate
        if estimate is None:
            return False, "no solved estimate to export", None
        try:
            entry = patch_calibration(
                target,
                rvec=estimate.rvec,
                tvec=estimate.tvec,
                camera_frame=camera_frame,
                lidar_frame=lidar_frame,
                dry_run=dry_run,
            )
        except ExportError as error:
            return False, str(error), None
        verb = "would write" if dry_run else "wrote"
        return True, f"{verb} {camera_frame} under {target}", dict(entry)
```

Add the imports at the top of `main.py`:

```python
from sensor_msgs.msg import CameraInfo, Image
from lctk_quality.diversity import compute_diversity
from lctk_autoware_export.export import ExportError, patch_calibration
from lidar_to_camera_solver.preview import PreviewStore, decode_image
from lidar_to_camera_solver.review_server import ReviewServer
from lidar_to_camera_solver.stability import StillnessTracker
```

Initialise `self._last_stillness = None` alongside the other node state at `:164-179`, and add it to the `solver_harness()` fixture in `test_identity_node_contract.py` next to `solver._continuous_solve_count = 0`.

Add `<depend>lctk_autoware_export</depend>` to `package.xml`.

- [ ] **Step 8: Run the whole solver suite**

```bash
cd /home/jetson/LCTK && source /opt/ros/humble/setup.bash
PYTHONPATH="$PWD/ros/lidar_to_camera_solver:$PWD/ros/lctk_quality:$PWD/ros/lctk_autoware_export:$PWD/ros/lctk_target:$PWD/ros/lctk_sync:$PYTHONPATH" \
  python3 -m pytest ros/lidar_to_camera_solver/test/ -q --no-header
```

Expected: all pass; `continuous` and `manual` tests unchanged.

- [ ] **Step 9: Commit**

```bash
cd /home/jetson/LCTK
just lint-py
git add ros/lidar_to_camera_solver/
git commit -m "feat(assisted): wire the third solver mode into the node"
```

---

### Task 6: Launch and justfile plumbing

**Files:**
- Modify: `ros/lctk_launch/launch/calibrate.launch.py:72-76` (validation), `:445-449` (argument description), `:221-222` (parameters)
- Modify: `ros/lctk_launch/launch/demo.launch.py:50-54` (description)
- Modify: `justfile` (a recipe for the assisted flow)
- Test: `ros/lctk_launch/test/test_calibrate_launch_graph.py`

**Interfaces:**
- Consumes: `SOLVER_MODES` from Task 5.
- Produces: `calibrate.launch.py` accepting `solver_mode:=assisted` and forwarding the new `stability_*`, `novelty_*`, `review_*` and `export_*` parameters from the config's optional `assisted:` section.

- [ ] **Step 1: Write the failing test**

```python
def test_assisted_is_an_accepted_solver_mode():
    context = _LaunchContext()
    context.launch_configurations["solver_mode"] = "assisted"
    actions = launch_setup(context)
    solver_nodes = [a for a in actions if getattr(a, "_package", None)
                    == "lidar_to_camera_solver"]
    assert solver_nodes, "assisted must still generate a solver node"


def test_an_unknown_solver_mode_is_refused_by_name():
    context = _LaunchContext()
    context.launch_configurations["solver_mode"] = "automatic"
    with pytest.raises(RuntimeError, match="'continuous', 'manual' or 'assisted'"):
        launch_setup(context)
```

- [ ] **Step 2: Run it and confirm it fails**

```bash
cd /home/jetson/LCTK && source /opt/ros/humble/setup.bash
PYTHONPATH="$PWD/ros/lctk_launch:$PYTHONPATH" \
  python3 -m pytest ros/lctk_launch/test/test_calibrate_launch_graph.py -q --no-header
```

Expected: `RuntimeError: Invalid solver_mode 'assisted'`.

- [ ] **Step 3: Update the launch validation and argument**

`calibrate.launch.py:72-76`:

```python
    solver_mode = LaunchConfiguration("solver_mode").perform(context)
    if solver_mode not in ("continuous", "manual", "assisted"):
        raise RuntimeError(
            f"Invalid solver_mode '{solver_mode}'; "
            "expected 'continuous', 'manual' or 'assisted'."
        )
```

`:445-449`:

```python
            DeclareLaunchArgument(
                "solver_mode",
                default_value="continuous",
                description=(
                    "LiDAR-camera solver behaviour: 'continuous' (auto-publishes the "
                    "latest pair), 'manual' (service-driven multi-pose buffer), or "
                    "'assisted' (auto-captures still, novel poses and serves a review "
                    "page)"
                ),
            ),
```

Mirror the description in `demo.launch.py:50-54`.

- [ ] **Step 4: Forward the assisted parameters**

In the `params` dict at `:221-222`, add the values parsed from the config's optional `assisted:` section, defaulting to the node's own defaults when the section is absent:

```python
        **_assisted_params(config),
```

with, near the other helpers in the same file:

```python
_ASSISTED_DEFAULTS = {
    "stability_window_frames": 10,
    "stability_max_translation_m": 0.005,
    "stability_max_rotation_deg": 0.5,
    "stability_cooldown_s": 1.0,
    "novelty_position_tol_m": 0.05,
    "novelty_orientation_tol_deg": 5.0,
    "review_bind_host": "127.0.0.1",
    "review_port": 8080,
    "review_jpeg_quality": 80,
    "review_max_previews": 64,
    "review_archive_path": "",
    "export_autoware_target": "",
    "export_camera_frame": "",
    "export_lidar_frame": "",
}


def _assisted_params(config):
    """The assisted-mode parameters, from the config's optional `assisted:` block.

    Unlike `sync:`, this section is optional: continuous and manual do not read
    any of it, and refusing a config that omits it would break both.
    """
    section = (config or {}).get("assisted") or {}
    unknown = set(section) - set(_ASSISTED_DEFAULTS)
    if unknown:
        raise ValueError(
            f"unknown key(s) in the 'assisted:' section: {', '.join(sorted(unknown))}"
        )
    return {name: section.get(name, default)
            for name, default in _ASSISTED_DEFAULTS.items()}
```

- [ ] **Step 5: Run the launch tests and confirm they pass**

```bash
cd /home/jetson/LCTK && source /opt/ros/humble/setup.bash
PYTHONPATH="$PWD/ros/lctk_launch:$PYTHONPATH" \
  python3 -m pytest ros/lctk_launch/test/ -q --no-header
```

- [ ] **Step 6: Add the justfile recipe**

```make
# Launch assisted calibration: auto-captures still, novel poses; review at
# http://localhost:8080
assisted CONFIG='seyond_left.yaml':
    #!/usr/bin/env bash
    set -eo pipefail
    source install/setup.bash
    SHARE=$(ros2 pkg prefix lctk_launch --share)
    play_launch launch \
        --web-addr 0.0.0.0:8000 \
        lctk_launch calibrate.launch.py \
        config_file:=$SHARE/config/examples/{{ CONFIG }} \
        debug_mode:={{ debug_mode }} \
        log_level:={{ log_level }} \
        mode:={{ mode }} \
        enable_rviz:={{ rviz_enabled }} \
        rviz_config:=$SHARE/config/rviz/lidar_camera.rviz \
        solver_mode:=assisted \
        enable_overlay:={{ enable_overlay }} \
        enable_judge:={{ enable_judge }}
```

`solver_mode` stays the switch, so `just solver_mode=continuous lidar-camera` and
`just solver_mode=manual lidar-camera` still select the original paths for comparison.

- [ ] **Step 7: Build and commit**

```bash
cd /home/jetson/LCTK
just build
just lint-py
git add justfile ros/lctk_launch/
git commit -m "feat(assisted): plumb the third mode through launch and the justfile"
```

---

### Task 7: Documentation

**Files:**
- Create: `book/src/user-guide/assisted-capture.md`
- Modify: `book/src/SUMMARY.md`
- Modify: `CLAUDE.md` (the solver-mode list and the generated-nodes section)
- Modify: `ros/lidar_to_camera_solver/README.md:21-35` (which currently says the accepted values are "exactly `continuous` and `manual`")
- Modify: `README.md:184`, `ros/lctk_launch/README.md:67`

- [ ] **Step 1: Write the user-guide page**

Cover: what assisted mode does, the two gates and why both exist, the workflow (launch, open the page, walk the board, watch the diversity meter, review, drop, export), every new config key with its meaning and default, and an explicit note that the review server is unauthenticated and binds loopback unless changed.

- [ ] **Step 2: Update `SUMMARY.md`**

```markdown
- [Assisted Capture](./user-guide/assisted-capture.md)
```

placed after the LiDAR-Camera Calibration entry.

- [ ] **Step 3: Update `CLAUDE.md` and the three READMEs**

Everywhere the two modes are enumerated, add the third with a one-line description. `ros/lidar_to_camera_solver/README.md` needs its "exactly `continuous` and `manual`" sentence corrected.

- [ ] **Step 4: Verify the docs build and the links resolve**

```bash
cd /home/jetson/LCTK
python3 setup/scripts/check-doc-links.py
cd book && just build
```

Expected: "all relative documentation links resolve", and a clean mdbook build.

- [ ] **Step 5: Commit**

```bash
cd /home/jetson/LCTK
git add book/ CLAUDE.md README.md ros/lidar_to_camera_solver/README.md ros/lctk_launch/README.md
git commit -m "docs(assisted): document the assisted capture mode"
```

---

### Task 8: Full verification

- [ ] **Step 1: Full build**

```bash
cd /home/jetson/LCTK && just build
```

- [ ] **Step 2: Full test suite**

```bash
cd /home/jetson/LCTK && just test
```

Expected: the Rust suite green, and the Python suite green apart from any failure that was already present on the branch before this work — record the before/after counts explicitly rather than assuming.

- [ ] **Step 3: Full lint**

```bash
cd /home/jetson/LCTK && just lint
```

- [ ] **Step 4: Confirm the original paths still run**

```bash
just solver_mode=continuous lidar-camera   # then Ctrl-C
just solver_mode=manual lidar-camera       # then Ctrl-C
just assisted                              # open http://localhost:8080
```

Expected: all three start; continuous and manual behave exactly as before; assisted additionally logs the review server URL.

- [ ] **Step 5: Commit any fixes and push**

```bash
cd /home/jetson/LCTK
git add -A && git commit -m "fix(assisted): address full-suite findings"
git push origin feat/selectable-calibration-targets
```
