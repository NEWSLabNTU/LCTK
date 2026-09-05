"""Small, bounded stores for messages that are matched by source timestamp.

The assisted review path receives camera frames and detector side-channel data on
independent subscriptions.  A bounded timestamp ring gives both adapters the same
matching policy without making the detector pair synchronizer carry another stream.
"""

from __future__ import annotations

import math
import threading
from collections import deque
from dataclasses import dataclass
from typing import Generic, TypeVar

T = TypeVar("T")


@dataclass(frozen=True)
class _StampedValue(Generic[T]):
    stamp: float
    value: T


class StampedRing(Generic[T]):
    """A thread-safe, time/count bounded ring keyed by monotonically rising stamps.

    ``match`` first looks for exact floating-point stamp equality.  The source
    adapters both derive their stamps from ROS integer seconds/nanoseconds, so
    exact equality is the normal path; nearest matching is only a small allowance
    for an adapter that has converted a source timestamp differently.

    A stamp rollback starts a new timeline and clears the old entries.  This is
    deliberately the same epoch rule used by the synchronizer and stillness
    tracker: comparing samples from before and after a replay restart would make
    an otherwise valid review image look like it belongs to the new pair.
    """

    def __init__(
        self,
        *,
        max_age_s: float,
        tolerance_s: float,
        max_items: int,
    ) -> None:
        max_age_s = float(max_age_s)
        tolerance_s = float(tolerance_s)
        max_items = int(max_items)
        if not math.isfinite(max_age_s) or max_age_s <= 0.0:
            raise ValueError(
                f"max_age_s must be finite and strictly positive; got {max_age_s}"
            )
        if not math.isfinite(tolerance_s) or tolerance_s < 0.0:
            raise ValueError(
                f"tolerance_s must be finite and non-negative; got {tolerance_s}"
            )
        if max_items < 1:
            raise ValueError(f"max_items must be at least 1; got {max_items}")
        self._max_age_s = max_age_s
        self._tolerance_s = tolerance_s
        self._max_items = max_items
        self._values: deque[_StampedValue[T]] = deque()
        self._epoch = 0
        self._lock = threading.RLock()

    def put(self, stamp: float, value: T) -> None:
        """Insert or replace one value, silently ignoring an invalid stamp."""
        try:
            stamp = float(stamp)
        except (TypeError, ValueError, OverflowError):
            self.clear()
            return
        if not math.isfinite(stamp):
            self.clear()
            return

        with self._lock:
            if self._values and stamp < self._values[-1].stamp:
                self._values.clear()
                self._epoch += 1

            # A duplicate source stamp is one sample being refreshed, not a new
            # timeline entry.  Keeping one value also ensures an exact match can
            # never accidentally select a stale duplicate after a count eviction.
            for index in range(len(self._values) - 1, -1, -1):
                if self._values[index].stamp == stamp:
                    del self._values[index]
                    break

            self._values.append(_StampedValue(stamp, value))
            self._prune_locked(stamp)
            while len(self._values) > self._max_items:
                self._values.popleft()

    def match(self, stamp: float) -> T | None:
        """Return an exact or nearby value, or ``None`` when no value is usable."""
        try:
            stamp = float(stamp)
        except (TypeError, ValueError, OverflowError):
            return None
        if not math.isfinite(stamp):
            return None

        with self._lock:
            # ``put`` already maintains the time bound against the newest source
            # stamp.  Do not prune against an arbitrary query stamp: a late pair
            # from a new topic may legitimately ask about an older entry, and a
            # failed lookup must not erase that entry before the other topic can
            # finish matching it.
            if self._values:
                self._prune_locked(self._values[-1].stamp)

            exact = self._match_exact_locked(stamp)
            if exact is not _NO_MATCH:
                return exact
            if not self._values:
                return None
            nearest = min(
                reversed(self._values),
                key=lambda entry: abs(entry.stamp - stamp),
            )
            if abs(nearest.stamp - stamp) <= self._tolerance_s:
                return nearest.value
            return None

    def match_exact(self, stamp: float) -> T | None:
        """Return only a value carrying exactly ``stamp``.

        This narrow helper is used to complete an evidence slot when a cloud or
        frame arrives after its pair.  It intentionally cannot borrow a nearby
        sweep during that late-arrival repair.
        """
        try:
            stamp = float(stamp)
        except (TypeError, ValueError, OverflowError):
            return None
        if not math.isfinite(stamp):
            return None
        with self._lock:
            exact = self._match_exact_locked(stamp)
            return None if exact is _NO_MATCH else exact

    def clear(self) -> None:
        """Forget all values and begin a fresh timestamp epoch."""
        with self._lock:
            self._values.clear()
            self._epoch += 1

    @property
    def epoch(self) -> int:
        """Monotonic timeline generation, incremented when the ring is cleared."""
        with self._lock:
            return self._epoch

    def __len__(self) -> int:
        with self._lock:
            return len(self._values)

    def _match_exact_locked(self, stamp: float) -> object:
        for entry in reversed(self._values):
            if entry.stamp == stamp:
                return entry.value
        return _NO_MATCH

    def _prune_locked(self, newest_stamp: float) -> None:
        horizon = newest_stamp - self._max_age_s
        while self._values and self._values[0].stamp < horizon:
            self._values.popleft()


_NO_MATCH = object()


__all__ = ["StampedRing"]
