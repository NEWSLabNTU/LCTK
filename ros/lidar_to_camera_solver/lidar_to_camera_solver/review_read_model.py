"""Revision tokens and bounded projection cache for assisted review.

The read model is deliberately smaller than the solver.  ``DetectionBuffer``
still owns captures, estimates, and quality; this module only gives the review
adapter a stable session epoch, change tokens, and one coherent cache seam.
"""

from __future__ import annotations

import logging
import secrets
import threading
from collections.abc import Callable, Hashable, Mapping
from dataclasses import dataclass
from typing import Any, TypeVar

T = TypeVar("T")
_LOGGER = logging.getLogger(__name__)


@dataclass(frozen=True)
class ReviewRevisions:
    """Immutable revision snapshot sent to the browser."""

    session_epoch: str
    state_revision: int
    capture_revision: int
    scene_revision: int
    evidence_revisions: Mapping[int, int]
    # ``capture_revision`` retains its old meaning: it only advances when the
    # DetectionBuffer quality projection changes.  ``captures_revision`` is the
    # aggregate token for the split captures representation and also advances
    # when asynchronous evidence (a preview or cloud) becomes available.
    live_revision: int = 0
    captures_revision: int = 0

    def event_vector(self) -> dict[str, int | str]:
        """Return the compact vector used by the review SSE stream.

        The event deliberately contains no pair list, evidence map, or scene
        geometry.  It is a hint telling the browser which representation to
        fetch, not a second transport for the representation itself.
        """

        return {
            "session_epoch": self.session_epoch,
            "live_revision": self.live_revision,
            "captures_revision": self.captures_revision,
            "scene_revision": self.scene_revision,
        }


class ReviewReadModel:
    """Own review change tokens and the expensive projection cache.

    ``observe_*`` methods are idempotent.  Callers may invoke them from every
    heartbeat; a token advances only when the corresponding source changes.
    ``observe_captures`` records the buffer and evidence observations as one
    atomic projection, so one request cannot publish two aggregate revisions.
    ``cached`` stores one projection per named slot. Review requests hold
    ``lock`` while taking their source snapshot and publishing its revisions.
    """

    def __init__(self, *, session_epoch: str | None = None) -> None:
        self._lock = threading.RLock()
        self.lock = self._lock
        self._session_epoch = session_epoch or secrets.token_hex(16)
        self._state_revision = 0
        self._capture_revision = 0
        self._live_revision = 0
        self._captures_revision = 0
        self._scene_revision = 0
        self._last_buffer_revision: Hashable | None = None
        self._last_live_signature: Hashable | None = None
        self._last_scene_revision: Hashable | None = None
        self._last_evidence: dict[int, Hashable] = {}
        self._evidence_revisions: dict[int, int] = {}
        self._last_state_signature: Hashable | None = None
        self._slots: dict[str, tuple[Hashable, Any]] = {}
        # Explicit marks are used by the assisted node's mutation callbacks.
        # The next observation consumes a corresponding mark, so the regular
        # read path does not advance the same token a second time.
        self._live_dirty = False
        self._capture_dirty = False
        self._captures_dirty = False
        self._scene_dirty = False
        self._listeners: set[Callable[[ReviewRevisions], None]] = set()

    @property
    def session_epoch(self) -> str:
        return self._session_epoch

    def add_listener(
        self, callback: Callable[[ReviewRevisions], None]
    ) -> Callable[[], None]:
        """Subscribe to revision-vector changes and return an unsubscribe hook.

        Callbacks are invoked after the read-model lock is released.  A
        callback must therefore treat the supplied immutable snapshot as the
        complete input and should return quickly; the SSE hub uses this seam to
        publish a bounded latest-value notification.
        """

        with self._lock:
            self._listeners.add(callback)

        def remove() -> None:
            with self._lock:
                self._listeners.discard(callback)

        return remove

    # ``subscribe`` is a convenient alias for callers that use the usual
    # observer vocabulary.  The return value remains an explicit unsubscribe
    # function rather than a long-lived queue owned by the read model.
    subscribe = add_listener

    def _revision_snapshot_locked(self) -> ReviewRevisions:
        return ReviewRevisions(
            session_epoch=self._session_epoch,
            state_revision=self._state_revision,
            capture_revision=self._capture_revision,
            scene_revision=self._scene_revision,
            evidence_revisions=dict(self._evidence_revisions),
            live_revision=self._live_revision,
            captures_revision=self._captures_revision,
        )

    def _notify(self, snapshot: ReviewRevisions | None) -> None:
        """Invoke listeners outside the model lock, isolating bad listeners."""

        if snapshot is None:
            return
        with self._lock:
            listeners = tuple(self._listeners)
        for callback in listeners:
            try:
                callback(snapshot)
            except (
                AttributeError,
                KeyError,
                OSError,
                RuntimeError,
                TypeError,
                ValueError,
            ) as error:
                # A transport subscriber must never be able to break the
                # solver's capture path.  The next revision gives it another
                # opportunity to catch up.
                _LOGGER.debug("review revision listener failed: %s", error)

    def observe_live(self, signature: Hashable) -> int:
        """Advance the live token when stillness/sync status changes."""

        snapshot = None
        with self._lock:
            changed = self._last_live_signature != signature
            self._last_live_signature = signature
            if changed:
                if self._live_dirty:
                    self._live_dirty = False
                else:
                    self._live_revision += 1
                    snapshot = self._revision_snapshot_locked()
            elif self._live_dirty:
                # An explicit mark can arrive before a heartbeat observes the
                # source.  It has already advanced the revision and only needs
                # its dirty flag consumed.
                self._live_dirty = False
        self._notify(snapshot)
        return self._live_revision

    def mark_live_changed(self) -> int:
        """Advance the live token from an assisted-mode mutation callback."""

        with self._lock:
            self._live_revision += 1
            self._live_dirty = True
            snapshot = self._revision_snapshot_locked()
        self._notify(snapshot)
        return self._live_revision

    mark_live = mark_live_changed

    def observe_capture(self, buffer_revision: Hashable) -> int:
        """Advance the capture/quality token when the buffer revision changes."""

        revision = buffer_revision
        snapshot = None
        with self._lock:
            if self._last_buffer_revision != revision:
                self._last_buffer_revision = revision
                if self._capture_dirty:
                    self._capture_dirty = False
                else:
                    self._capture_revision += 1
                    self._captures_revision += 1
                    snapshot = self._revision_snapshot_locked()
            elif self._capture_dirty:
                self._capture_dirty = False
            if self._captures_dirty:
                # A mutation callback already advanced the aggregate token.
                # Clear the marker even when this standalone observation is
                # not followed by an evidence observation.
                self._captures_dirty = False
        self._notify(snapshot)
        return self._capture_revision

    def mark_capture_changed(self) -> int:
        """Mark a DetectionBuffer mutation before the next review read."""

        with self._lock:
            self._capture_revision += 1
            self._captures_revision += 1
            self._capture_dirty = True
            self._captures_dirty = True
            snapshot = self._revision_snapshot_locked()
        self._notify(snapshot)
        return self._capture_revision

    mark_capture = mark_capture_changed

    def mark_captures_changed(self) -> int:
        """Mark an evidence/list representation mutation.

        This token is separate from ``capture_revision`` because a delayed
        image or plane-inlier message changes the review list without changing
        the solver's DetectionBuffer quality calculation.
        """

        with self._lock:
            self._captures_revision += 1
            self._captures_dirty = True
            snapshot = self._revision_snapshot_locked()
        self._notify(snapshot)
        return self._captures_revision

    mark_captures = mark_captures_changed

    def observe_scene(self, scene_revision: Hashable) -> int:
        """Remember the node's world-geometry revision."""

        revision = scene_revision
        snapshot = None
        with self._lock:
            if self._last_scene_revision != revision:
                node_revision = (
                    revision[0] if isinstance(revision, tuple) else int(revision)
                )
                if self._scene_dirty:
                    self._scene_dirty = False
                    previous_revision = self._scene_revision
                    self._scene_revision = max(self._scene_revision, node_revision)
                    if self._scene_revision != previous_revision:
                        snapshot = self._revision_snapshot_locked()
                else:
                    self._scene_revision = (
                        node_revision
                        if self._last_scene_revision is None
                        else max(self._scene_revision + 1, node_revision)
                    )
                    snapshot = self._revision_snapshot_locked()
                self._last_scene_revision = revision
            elif self._scene_dirty:
                self._scene_dirty = False
        self._notify(snapshot)
        return self._scene_revision

    def mark_scene_changed(self) -> int:
        """Mark a world-scene mutation before its next scene read."""

        with self._lock:
            self._scene_revision += 1
            self._scene_dirty = True
            snapshot = self._revision_snapshot_locked()
        self._notify(snapshot)
        return self._scene_revision

    mark_scene = mark_scene_changed

    def observe_evidence(self, values: Mapping[int, Hashable]) -> dict[int, int]:
        """Advance only the Capture IDs whose evidence signature changed."""

        current = {int(pair_id): signature for pair_id, signature in values.items()}
        snapshot = None
        with self._lock:
            changed = self._observe_evidence_locked(current)
            if changed:
                if self._captures_dirty:
                    self._captures_dirty = False
                else:
                    self._captures_revision += 1
                    snapshot = self._revision_snapshot_locked()
            elif self._captures_dirty:
                self._captures_dirty = False
        self._notify(snapshot)
        return dict(self._evidence_revisions)

    def observe_captures(
        self,
        buffer_revision: Hashable,
        values: Mapping[int, Hashable],
    ) -> tuple[int, dict[int, int]]:
        """Observe buffer and evidence state as one aggregate projection.

        The assisted node reads both sources under its state lock.  Keeping the
        two comparisons in one model operation prevents a single HTTP request
        from incrementing ``captures_revision`` once for the buffer and again
        for the evidence map.  Explicit mutation marks have already published
        their revisions; the dirty flags suppress the corresponding lazy
        observation and are then consumed.
        """

        revision = buffer_revision
        current = {int(pair_id): signature for pair_id, signature in values.items()}
        snapshot = None
        with self._lock:
            buffer_changed = self._last_buffer_revision != revision
            self._last_buffer_revision = revision
            capture_marked = self._capture_dirty
            self._capture_dirty = False
            evidence_changed = self._observe_evidence_locked(current)
            aggregate_changed = buffer_changed or evidence_changed

            public_changed = False
            if buffer_changed and not capture_marked:
                self._capture_revision += 1
                public_changed = True
            if aggregate_changed:
                if self._captures_dirty:
                    self._captures_dirty = False
                else:
                    self._captures_revision += 1
                    public_changed = True
            elif self._captures_dirty:
                self._captures_dirty = False

            if public_changed:
                snapshot = self._revision_snapshot_locked()
        self._notify(snapshot)
        return self._capture_revision, dict(self._evidence_revisions)

    def _observe_evidence_locked(self, current: Mapping[int, Hashable]) -> bool:
        """Apply an evidence map under ``_lock`` and report whether it changed."""

        changed = False
        for pair_id, signature in current.items():
            if self._last_evidence.get(pair_id) == signature:
                continue
            changed = True
            self._last_evidence[pair_id] = signature
            self._evidence_revisions[pair_id] = (
                self._evidence_revisions.get(pair_id, 0) + 1
            )
        for pair_id in tuple(self._last_evidence):
            if pair_id in current:
                continue
            changed = True
            self._last_evidence.pop(pair_id, None)
            self._evidence_revisions.pop(pair_id, None)
        return changed

    def observe_state(self, signature: Hashable) -> int:
        """Advance the complete-state token when any rendered field changes."""

        with self._lock:
            if self._last_state_signature != signature:
                self._last_state_signature = signature
                self._state_revision += 1
            return self._state_revision

    def cached(self, key: Hashable, builder: Callable[[], T], *, slot="quality") -> T:
        """Return the projection for ``key``, building it once when possible."""

        with self._lock:
            cached = self._slots.get(slot)
            if cached is not None and cached[0] == key:
                return cached[1]
            value = builder()
            self._slots[slot] = (key, value)
            return value

    def revisions(self) -> ReviewRevisions:
        """Return a copy safe to include in a JSON response."""

        with self._lock:
            return self._revision_snapshot_locked()

    def revision_vector(self) -> dict[str, int | str]:
        """Return the compact SSE/fallback-polling revision vector."""

        return self.revisions().event_vector()

    def reset(self, *, session_epoch: str | None = None) -> str:
        """Start a new review session and invalidate every old token/cache."""

        with self._lock:
            self._session_epoch = session_epoch or secrets.token_hex(16)
            self._state_revision = 0
            self._capture_revision = 0
            self._live_revision = 0
            self._captures_revision = 0
            self._scene_revision = 0
            self._last_buffer_revision = None
            self._last_live_signature = None
            self._last_scene_revision = None
            self._last_evidence.clear()
            self._evidence_revisions.clear()
            self._last_state_signature = None
            self._slots.clear()
            self._live_dirty = False
            self._capture_dirty = False
            self._captures_dirty = False
            self._scene_dirty = False
            snapshot = self._revision_snapshot_locked()
        self._notify(snapshot)
        return snapshot.session_epoch


__all__ = ["ReviewReadModel", "ReviewRevisions"]
