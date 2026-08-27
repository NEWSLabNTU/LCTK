"""Synchronized detection pairs for the LCTK solver nodes.

`DetectionPairSource` is the module; everything else exported here is the pure decision
layer beneath it, kept public because it is the test surface and because a caller
occasionally wants to render one of these messages itself.
"""

from lctk_sync.config import DROP_POLICIES, PairSourceConfig
from lctk_sync.diagnosis import (
    SyncGroupSummary,
    format_sync_stats,
    should_reset_for_new_epoch,
    sync_health_warning,
    sync_pair_staleness_error,
    sync_wait_diagnosis,
)
from lctk_sync.pair_source import DetectionPairSource, PairOutcome, ReentrantLock

__all__ = [
    "DROP_POLICIES",
    "DetectionPairSource",
    "PairOutcome",
    "PairSourceConfig",
    "ReentrantLock",
    "SyncGroupSummary",
    "format_sync_stats",
    "should_reset_for_new_epoch",
    "sync_health_warning",
    "sync_pair_staleness_error",
    "sync_wait_diagnosis",
]
