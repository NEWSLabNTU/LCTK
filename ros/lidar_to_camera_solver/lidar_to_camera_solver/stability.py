"""Is the calibration board being held still?

A capture session's quality depends on catching the board when it is stationary,
which an operator currently judges by eye before reaching for a key. This module
is that judgement, made from the board pose the LiDAR detector already publishes.

Two things about the window are load-bearing.

**The gate is a span across the window, not a frame-to-frame delta.** A board
drifting steadily at 1 mm per frame has a negligible per-frame delta and is
plainly not still; only the span over the whole window sees it. Getting this
wrong would auto-capture a slow drift, which is exactly the motion blur the
reviewer would then have to find by hand.

**The window is a duration, not a count of detection pairs.** Pairs are the
output of a synchronizer over LiDAR board detections and ArUco detections, and
they arrive irregularly: the board leaves the field of view, an ICP fit is
rejected, a sweep returns too few points. Ten consecutive pairs therefore say
nothing about how long the board held. Measured on the 58 s recording behind
`sessions/solid600-handheld-zed`, a ten-pair window ran from 0.48 s to 19.42 s
(median 1.30 s), with 71 of 195 windows shorter than one second -- so the
frame-count window was simultaneously too permissive during a dense burst and
unboundedly stale across a dropout.

Pure numpy: no ROS, no OpenCV, so the policy is unit-testable without a graph.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import numpy as np

#: Samples required *inside* the window before any ``is_still=True`` verdict.
#:
#: This is a correctness guard against sparse detections, not a quality bar --
#: the review page is where an operator prunes marginal captures -- so it is the
#: smallest count that does the job rather than the most evidence obtainable.
#:
#: A time window on its own is not evidence: two pairs whose stamps happen to
#: straddle a second satisfy the window while saying nothing about what the
#: board did in between. Two would not fix that here, because the bracketing
#: sample ``_evict`` retains is not counted: a stream arriving every 0.8 s puts
#: exactly two samples inside a one-second window, and that stream -- ten pairs
#: covering eight seconds -- is the case this window change exists to reject.
#: Three is therefore the smallest floor that rejects it, and it costs nothing
#: on real data: on the solid600 recording the median one-second window holds
#: nine detections, and dropping the floor to two changes neither the captures
#: nor the placements at any threshold tried.
DEFAULT_MIN_SAMPLES = 3


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
    """Sliding time-window stillness gate with a one-shot capture latch."""

    def __init__(
        self,
        *,
        window_s: float,
        max_translation_m: float,
        max_rotation_deg: float,
        cooldown_s: float,
        min_samples: int = DEFAULT_MIN_SAMPLES,
    ):
        window_s = float(window_s)
        if not math.isfinite(window_s) or window_s <= 0.0:
            raise ValueError(
                f"window_s must be finite and strictly positive to have a span; "
                f"got {window_s}"
            )
        if min_samples < 2:
            raise ValueError(
                f"min_samples must be at least 2 to have a span; got {min_samples}"
            )
        self._window_s = window_s
        self._min_samples = min_samples
        self._max_translation_m = max_translation_m
        self._max_rotation_deg = max_rotation_deg
        self._cooldown_s = cooldown_s
        self._stamps: deque[float] = deque()
        self._positions: deque[np.ndarray] = deque()
        self._quaternions: deque[np.ndarray] = deque()
        self._armed = True
        self._last_capture_s: float | None = None

    def reset(self) -> None:
        """Forget the window. Used when the recording restarts under the node."""
        self._stamps.clear()
        self._positions.clear()
        self._quaternions.clear()
        self._armed = True

    def _evict(self, horizon: float) -> None:
        """Drop everything older than the window, keeping one bracketing sample.

        The oldest retained sample is the *newest* one at or before ``horizon``,
        when such a sample exists. Retaining it is what lets the measured span
        cover the full ``window_s``: with strict eviction the retained samples
        could only ever span a little under the window, so "the board held for a
        second" would never be provable. The bracket can only widen a span, never
        narrow one, so keeping it is the conservative choice.
        """
        while len(self._stamps) > 1 and self._stamps[1] <= horizon:
            self._stamps.popleft()
            self._positions.popleft()
            self._quaternions.popleft()

    def push(self, position, quaternion, stamp_s: float) -> StillnessState:
        stamp_s = float(stamp_s)
        if self._stamps and stamp_s < self._stamps[-1]:
            # Replayed data and a `--clock` restart can hand us a stamp from
            # before the last one. The samples already in the window belong to an
            # abandoned timeline and cannot be compared against this one, so the
            # window goes. The cooldown anchor goes with it: left in place it
            # would be compared against smaller stamps and would suppress every
            # capture until the new clock caught up to the old one.
            self.reset()
            self._last_capture_s = None

        self._stamps.append(stamp_s)
        self._positions.append(np.asarray(position, dtype=np.float64))
        quaternion_array = np.asarray(quaternion, dtype=np.float64)
        norm = float(np.linalg.norm(quaternion_array))
        if norm > 0.0:
            quaternion_array = quaternion_array / norm
        self._quaternions.append(quaternion_array)

        horizon = stamp_s - self._window_s
        self._evict(horizon)

        covered_s = self._stamps[-1] - self._stamps[0]
        frames = sum(1 for stamp in self._stamps if stamp > horizon)

        if covered_s < self._window_s:
            return StillnessState(
                is_still=False,
                should_capture=False,
                translation_span_m=0.0,
                rotation_span_deg=0.0,
                frames=frames,
                reason=(f"filling the window: {covered_s:.2f}/{self._window_s:.2f} s"),
            )

        if frames < self._min_samples:
            return StillnessState(
                is_still=False,
                should_capture=False,
                translation_span_m=0.0,
                rotation_span_deg=0.0,
                frames=frames,
                reason=(
                    f"too few detections: {frames} in the last "
                    f"{self._window_s:.2f} s (need {self._min_samples})"
                ),
            )

        stacked = np.stack(self._positions)
        translation_span = float(
            np.max(np.linalg.norm(stacked[:, None, :] - stacked[None, :, :], axis=-1))
        )
        quaternions = list(self._quaternions)
        rotation_span = max(
            _quaternion_angle_deg(a, b)
            for i, a in enumerate(quaternions)
            for b in quaternions[i + 1 :]
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
                    f"{rotation_span:.1f} deg over {frames} detections in "
                    f"{covered_s:.2f} s"
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
