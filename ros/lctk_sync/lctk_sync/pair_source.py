"""The synchronized detection pair, as one module.

A solver node wants one thing from message synchronization: *the freshest pair of
detections that were genuinely simultaneous, or the reason there isn't one.* Everything
else -- the synchronizer, its window, its buffers, its counters, what a rewound
recording does to it, and how stale a cached pair may be -- is this module's business.

## Why this exists

Three solver nodes each wired conflux by hand, and each held a large interface to do it:
nine parameters, two topics, four counters, and (once the epoch fix landed) conflux's
private `_sync` handle. Two defects found on 2026-08-15 were therefore fixed in exactly
one of the three, and `lidar_to_lidar_solver` still paired by arrival order and still
stopped dead when a bag was replayed. See
`docs/adr/2026-08-15-architecture-review.md`.

## What it knows that a caller should not have to

- **The window is a correctness setting.** Conflux matches by time only when a finite
  window is set; with an infinite one it pairs by arrival order and two streams at
  different rates drift apart without bound. `PairSourceConfig` refuses it.
- **A replayed recording is a new epoch.** Conflux is strictly time-ordered: it rejects
  any message stamped at or before the group it last emitted, and that commit time only
  moves forward. Both detectors copy the stamp of the message they consumed, so every
  new bag -- or every `--loop` wrap -- sends the stamps backward and conflux stops
  pairing permanently. This module notices and starts a fresh synchronizer. Conflux's
  rule is right for a live sensor and is left alone.
- **A cached pair goes stale.** Playback stops; the pair does not. Handing it out
  minutes later buffers a pose from an unknown moment while looking like success.
- **The skew inside a pair is the number that matters.** It is what says whether
  "synchronized" meant anything, so it is measured and reported rather than assumed.
"""

import time
from collections.abc import Callable, Iterator, Sequence
from contextlib import contextmanager
from dataclasses import dataclass
from typing import Any, Protocol

from conflux_py import DropPolicy, ROS2Synchronizer, SyncGroup

from lctk_sync.config import PairSourceConfig
from lctk_sync.diagnosis import (
    SyncGroupSummary,
    format_sync_stats,
    should_reset_for_new_epoch,
    sync_health_warning,
    sync_pair_staleness_error,
    sync_wait_diagnosis,
)


@dataclass(frozen=True)
class PairOutcome:
    """Either a pair of messages, or the reason there isn't one.

    Both cases carry an operator-facing `reason` when the answer is no, so a caller can
    hand it straight to a service response without composing an explanation of its own.
    """

    messages: tuple[Any, ...] | None = None
    reason: str | None = None

    @property
    def ok(self) -> bool:
        return self.messages is not None


class ReentrantLock(Protocol):
    """Lock protocol for a consumer that nests admission and invalidation."""

    def __enter__(self) -> Any: ...

    def __exit__(self, exc_type, exc_value, traceback) -> bool | None: ...


class DetectionPairSource:
    """Freshest simultaneous pair of detections from two (or more) topics.

    Interface: construct it, optionally register `admit_pair` for a pre-cache
    consumer gate and `on_pair` for push consumers, and call `take_fresh_pair()`
    when you need the newest usable pair. `admission_lock` can serialize the
    admission predicate and cache mutation with a consumer's invalidation; the
    predicate runs while that context manager is held. The lock must be reentrant:
    consumers may invalidate the source while already holding their state lock.
    `status_line()` reports what the synchronization is doing. Everything else is
    internal.
    """

    def __init__(
        self,
        node,
        topics: Sequence[str],
        msg_types: Sequence[type],
        *,
        config: PairSourceConfig | None = None,
        qos=None,
        on_pair: Callable[[tuple[Any, ...]], None] | None = None,
        admit_pair: Callable[[tuple[Any, ...]], str | None] | None = None,
        admission_lock: ReentrantLock | None = None,
    ):
        if len(topics) != len(msg_types):
            raise ValueError("topics and msg_types must be the same length")

        self._node = node
        self._topics = list(topics)
        self._config = config or PairSourceConfig()
        self._on_pair = on_pair
        self._admit_pair = admit_pair
        # Optional consumer-owned lock.  When supplied, admission and cache
        # mutation are one atomic operation from the consumer's point of view.
        # The protocol is intentionally structural: Python cannot reliably
        # determine whether an arbitrary context manager is reentrant at runtime.
        # Keeping this opt-in preserves the source's generic, lock-free API.
        self._admission_lock = admission_lock

        self._latest: tuple[Any, ...] | None = None
        self._latest_at: float | None = None
        self._last_group: SyncGroupSummary | None = None
        self._last_group_at: float | None = None
        self._last_skew_ms: float | None = None
        self._max_skew_ms: float = 0.0
        self._epoch_resets = 0
        self._last_epoch_reset_at = 0.0
        self._last_epoch_received: dict = {}
        self._started_at = time.monotonic()
        self._last_received: dict = {}
        self._last_stats_line: str | None = None

        drop_policy = (
            DropPolicy.DROP_OLDEST
            if self._config.drop_policy == "drop_oldest"
            else DropPolicy.REJECT_NEW
        )

        self._sync = ROS2Synchronizer(
            node,
            window_size_ms=int(self._config.window_ms),
            buffer_size=self._config.queue_size,
            drop_policy=drop_policy,
            qos=qos,
        )
        for msg_type, topic in zip(msg_types, self._topics):
            self._sync.add_subscription(msg_type, topic)

        @self._sync.on_synchronized
        def _handle(group: SyncGroup):
            self._handle_group(group)

        if self._config.stats_interval_s > 0.0:
            self._stats_timer = node.create_timer(
                self._config.stats_interval_s, self._log_stats
            )
        if self._config.epoch_check_interval_s > 0.0:
            self._epoch_timer = node.create_timer(
                self._config.epoch_check_interval_s, self._check_for_new_epoch
            )

        node.get_logger().info(
            f"Synchronizing {', '.join(self._topics)} "
            f"(window={self._config.window_ms:.0f}ms, buffer={self._config.queue_size}, "
            f"policy={self._config.drop_policy})"
        )

    # ---- interface -------------------------------------------------------------

    def take_fresh_pair(self) -> PairOutcome:
        """The newest usable pair, or the reason there is none.

        Refuses a pair older than `max_pair_age_s`: playback stops, the cached pair does
        not, and handing it out later buffers detections from an unknown moment.
        """
        with self._cache_context():
            if self._latest is None:
                return PairOutcome(reason=sync_wait_diagnosis(self._current_summary()))

            staleness = sync_pair_staleness_error(
                age_s=time.monotonic() - (self._latest_at or 0.0),
                max_age_s=self._config.max_pair_age_s,
            )
            if staleness is not None:
                return PairOutcome(reason=f"{staleness} [{self.status_line()}]")

            return PairOutcome(messages=self._latest)

    def status_line(self) -> str:
        """What the synchronization is doing, for a log line or a refusal message."""
        stats = self._sync.statistics
        skew = (
            None
            if self._last_skew_ms is None
            else (self._last_skew_ms, self._max_skew_ms)
        )
        return format_sync_stats(
            received=stats.messages_received,
            dropped=stats.messages_dropped,
            rejected=stats.messages_rejected,
            groups=stats.groups_synchronized,
            skew_ms=skew,
        )

    def discard_cached_pair(self) -> None:
        """Forget the cached pair.

        For a caller whose own state has been reset -- the solver's `clear_buffer`
        service means "start over", and a pair captured before the clear must not be
        addable after it, even while it is still inside the freshness window.
        """
        with self._cache_context():
            self._clear_cached_pair()

    def is_cached_pair(self, messages: tuple[Any, ...]) -> bool:
        """Whether ``messages`` is still the exact cached pair.

        Push consumers receive a pair after the cache critical section.  A consumer
        that may invalidate its own state while a callback is delayed must call this
        before restoring the pair: tuple identity distinguishes that callback's
        admission from a newer pair or a cache that was discarded during a reset.
        The check shares ``admission_lock`` with cache writes and invalidation when
        one is supplied, so it is atomic with those operations from the consumer's
        point of view.  The supplied lock must be reentrant because callers normally
        perform this check while holding their state lock.
        """
        with self._cache_context():
            return self._latest is messages

    @property
    def epoch_resets(self) -> int:
        """How many times the recording has restarted under this source."""
        return self._epoch_resets

    # ---- implementation --------------------------------------------------------

    @contextmanager
    def _cache_context(self) -> Iterator[None]:
        """Serialize cache access with the optional consumer admission lock."""
        if self._admission_lock is None:
            yield
            return
        with self._admission_lock:
            yield

    def _clear_cached_pair(self) -> None:
        """Clear the latest pair; caller owns the cache context."""
        self._latest = None
        self._latest_at = None

    @staticmethod
    def _stamp_s(msg) -> float:
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _handle_group(self, group: SyncGroup):
        messages = tuple(group.get(topic) for topic in self._topics)
        if any(msg is None for msg in messages):
            self._node.get_logger().warn(
                f"Incomplete sync group: {group.topics()}", throttle_duration_sec=5.0
            )
            return

        counts = tuple(len(getattr(msg, "detections", ())) for msg in messages)
        stamps = [self._stamp_s(msg) for msg in messages]
        skew_ms = (max(stamps) - min(stamps)) * 1000.0

        now = time.monotonic()
        self._last_skew_ms = skew_ms
        self._max_skew_ms = max(self._max_skew_ms, skew_ms)
        self._last_group = SyncGroupSummary(
            aruco_count=counts[0], board_count=counts[-1], age_s=0.0
        )
        self._last_group_at = now

        if self._config.require_non_empty and not all(counts):
            # Both sides warn, at the same level and both throttled. These used to be
            # asymmetric -- the empty-board case warned while the empty-ArUco case only
            # logged at debug -- so a recording whose two detectors never succeed at the
            # same moment looked like silence at log_level=info.
            for topic, count in zip(self._topics, counts):
                if count == 0:
                    self._node.get_logger().warn(
                        f"Ignoring sync group: '{topic}' carried no detections "
                        f"(counts={counts})",
                        throttle_duration_sec=5.0,
                    )
            return

        # An optional consumer-owned admission gate runs in the same critical
        # section as the latest-pair cache mutation.  The LiDAR-to-LiDAR
        # solver supplies its state RLock here, so a target-identity update
        # cannot clear the cache between an accepted predicate and this write.
        rejection = None
        with self._cache_context():
            if self._admit_pair is not None:
                rejection = self._admit_pair(messages)
                if rejection is not None:
                    self._clear_cached_pair()
                else:
                    self._latest = messages
                    self._latest_at = now
            else:
                self._latest = messages
                self._latest_at = now

        if rejection is not None:
            self._node.get_logger().warn(
                f"Ignoring sync group before cache admission: {rejection}",
                throttle_duration_sec=5.0,
            )
            return

        # Push callbacks intentionally run after the cache critical section:
        # solver computation may be expensive.  A callback that can be delayed
        # must re-check `is_cached_pair` before restoring this tuple, so a reset
        # or newer pair cannot be interpreted as current-session input.
        if self._on_pair is not None:
            self._on_pair(messages)

    def _current_summary(self) -> SyncGroupSummary | None:
        if self._last_group is None or self._last_group_at is None:
            return None
        return SyncGroupSummary(
            aruco_count=self._last_group.aruco_count,
            board_count=self._last_group.board_count,
            age_s=time.monotonic() - self._last_group_at,
        )

    def _last_group_age_s(self) -> float | None:
        if self._last_group_at is None:
            return None
        return time.monotonic() - self._last_group_at

    def _check_for_new_epoch(self):
        received = dict(self._sync.statistics.messages_received)
        if should_reset_for_new_epoch(
            previous_received=self._last_epoch_received,
            current_received=received,
            last_group_age_s=self._last_group_age_s(),
            age_since_start_s=time.monotonic() - self._started_at,
        ):
            self._reset_for_new_epoch()
        self._last_epoch_received = received

    def _reset_for_new_epoch(self):
        """Start a fresh synchronizer after the recording restarted.

        Swaps conflux's matching engine for a new one, leaving the subscriptions (and
        the statistics, which are the evidence) in place. The buffered messages all
        belong to the previous recording and cannot pair with anything arriving now, so
        they go with it -- as does any cached pair.

        `ROS2Synchronizer.reset()` keeps the ROS subscriptions, callback and cumulative
        statistics while replacing the matching engine.
        """
        self._epoch_resets += 1
        self._last_epoch_reset_at = time.monotonic()
        self._sync.reset()
        with self._cache_context():
            self._clear_cached_pair()
        self._max_skew_ms = 0.0
        self._node.get_logger().warn(
            f"Nothing has paired while both streams keep arriving: the recording "
            f"changed under the synchronizer (a new bag, a --loop wrap, or a stream "
            f"that started late holding a previous recording's timestamps). conflux is "
            f"strictly time-ordered and cannot recover from either on its own, so it "
            f"had stopped pairing. Started a fresh synchronizer (reset "
            f"#{self._epoch_resets}); detections already buffered by the solver are "
            f"untouched."
        )

    def _log_stats(self):
        line = self.status_line()
        if line != self._last_stats_line:
            self._last_stats_line = line
            self._node.get_logger().info(line)

        received = dict(self._sync.statistics.messages_received)
        warning = sync_health_warning(
            previous=self._last_received,
            current=received,
            last_group_age_s=self._last_group_age_s(),
        )
        self._last_received = received
        # A reset in the last few seconds explains the silence and has already been
        # reported; saying it twice in different words helps nobody.
        if warning is not None and time.monotonic() - self._last_epoch_reset_at > 10.0:
            self._node.get_logger().warn(warning)
