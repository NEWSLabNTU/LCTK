"""Revision tokens and bounded projection cache for assisted review.

The read model is deliberately smaller than the solver.  ``DetectionBuffer``
still owns captures, estimates, and quality; this module only gives the review
adapter a stable session epoch, change tokens, and one coherent cache seam.
"""

from __future__ import annotations

import secrets
import threading
from collections.abc import Callable, Hashable, Mapping
from dataclasses import dataclass
from typing import Any, TypeVar

T = TypeVar("T")


@dataclass(frozen=True)
class ReviewRevisions:
    """Immutable revision snapshot sent to the browser."""

    session_epoch: str
    state_revision: int
    capture_revision: int
    scene_revision: int
    evidence_revisions: Mapping[int, int]


class ReviewReadModel:
    """Own review change tokens and the expensive projection cache.

    ``observe_*`` methods are idempotent.  Callers may invoke them from every
    heartbeat; a token advances only when the corresponding source changes.
    ``cached`` stores one projection per named slot. Review requests hold
    ``lock`` while taking their source snapshot and publishing its revisions.
    """

    def __init__(self, *, session_epoch: str | None = None) -> None:
        self._lock = threading.RLock()
        self.lock = self._lock
        self._session_epoch = session_epoch or secrets.token_hex(16)
        self._state_revision = 0
        self._capture_revision = 0
        self._scene_revision = 0
        self._last_buffer_revision: Hashable | None = None
        self._last_scene_revision: Hashable | None = None
        self._last_evidence: dict[int, Hashable] = {}
        self._evidence_revisions: dict[int, int] = {}
        self._last_state_signature: Hashable | None = None
        self._slots: dict[str, tuple[Hashable, Any]] = {}

    @property
    def session_epoch(self) -> str:
        return self._session_epoch

    def observe_capture(self, buffer_revision: Hashable) -> int:
        """Advance the capture/quality token when the buffer revision changes."""

        revision = buffer_revision
        with self._lock:
            if self._last_buffer_revision != revision:
                self._last_buffer_revision = revision
                self._capture_revision += 1
            return self._capture_revision

    def observe_scene(self, scene_revision: Hashable) -> int:
        """Remember the node's world-geometry revision."""

        revision = scene_revision
        with self._lock:
            if self._last_scene_revision != revision:
                node_revision = (
                    revision[0] if isinstance(revision, tuple) else int(revision)
                )
                self._scene_revision = (
                    node_revision
                    if self._last_scene_revision is None
                    else max(self._scene_revision + 1, node_revision)
                )
                self._last_scene_revision = revision
            return self._scene_revision

    def observe_evidence(self, values: Mapping[int, Hashable]) -> dict[int, int]:
        """Advance only the Capture IDs whose evidence signature changed."""

        current = {int(pair_id): signature for pair_id, signature in values.items()}
        with self._lock:
            for pair_id, signature in current.items():
                if self._last_evidence.get(pair_id) == signature:
                    continue
                self._last_evidence[pair_id] = signature
                self._evidence_revisions[pair_id] = (
                    self._evidence_revisions.get(pair_id, 0) + 1
                )
            for pair_id in tuple(self._last_evidence):
                if pair_id in current:
                    continue
                self._last_evidence.pop(pair_id, None)
                self._evidence_revisions.pop(pair_id, None)
            return dict(self._evidence_revisions)

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
            return ReviewRevisions(
                session_epoch=self._session_epoch,
                state_revision=self._state_revision,
                capture_revision=self._capture_revision,
                scene_revision=self._scene_revision,
                evidence_revisions=dict(self._evidence_revisions),
            )


__all__ = ["ReviewReadModel", "ReviewRevisions"]
