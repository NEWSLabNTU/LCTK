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
