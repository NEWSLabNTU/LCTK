"""Stateful replay-epoch recovery for synchronized detection streams."""

from collections.abc import Mapping

from lctk_sync.diagnosis import should_reset_for_new_epoch


class EpochRecovery:
    """Make the existing replay-reset heuristic a one-shot state machine.

    The tracker deliberately consumes only evidence already exposed by the LCTK
    synchronizer adapter. It does not inspect Conflux internals or alter the
    synchronizer itself.

    A reset is destructive because it discards buffered messages. Once the
    heuristic requests one, the tracker enters recovery and suppresses further
    reset requests until a synchronized group proves that the fresh engine is
    working again. A later timer sample may still update the receipt baseline,
    but it cannot erase the fresh buffers a second time.
    """

    def __init__(self):
        self._recovering = False
        self._previous_received: dict[str, int] = {}

    @property
    def recovering(self) -> bool:
        """Whether a reset has happened and recovery is still unproven."""
        return self._recovering

    def observe(
        self,
        *,
        received: Mapping[str, int],
        last_group_age_s: float | None,
        age_since_start_s: float | None,
        quiet_after_s: float = 2.0,
    ) -> bool:
        """Record one timer sample and return whether one reset is warranted."""
        current_received = dict(received)
        if self._recovering:
            self._previous_received = current_received
            return False

        reset = should_reset_for_new_epoch(
            previous_received=self._previous_received,
            current_received=current_received,
            last_group_age_s=last_group_age_s,
            age_since_start_s=age_since_start_s,
            quiet_after_s=quiet_after_s,
        )
        self._previous_received = current_received
        if reset:
            self._recovering = True
        return reset

    def mark_group(self) -> bool:
        """Record that matching works; return whether this completed recovery."""
        recovered = self._recovering
        self._recovering = False
        return recovered
