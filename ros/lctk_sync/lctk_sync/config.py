"""How a `DetectionPairSource` is configured, and which settings it refuses.

The window is the one setting that decides whether "synchronized" means anything, so
it is validated here rather than trusted from a launch file -- see the module docstring
of `pair_source.py`.
"""

from dataclasses import dataclass

DROP_POLICIES = ("reject_new", "drop_oldest")


@dataclass(frozen=True)
class PairSourceConfig:
    """Settings for one synchronized pair of detection streams.

    Defaults are the offline (recorded playback) settings, which is what every solver
    in this repository runs by default.
    """

    #: Time window in milliseconds. Two messages pair only if their header stamps sit
    #: within this of each other.
    window_ms: float = 100.0

    #: Messages buffered per stream before `drop_policy` applies.
    queue_size: int = 100

    #: What to do when a stream's buffer is full: "reject_new" (preserve what is
    #: buffered) or "drop_oldest" (always accept the newest).
    drop_policy: str = "reject_new"

    #: Whether a group holding an empty detection array counts as a pair. Solvers want
    #: False here: an empty array means the detector found nothing, not that nothing
    #: was detected at that instant.
    require_non_empty: bool = True

    #: How old the newest pair may be when a caller asks for it, in seconds. 0 disables
    #: the check. Without it a cached pair lives forever and a solver keeps "succeeding"
    #: long after playback stopped, buffering a pose from an unknown moment.
    max_pair_age_s: float = 2.0

    #: How often the counters are logged. 0 disables.
    stats_interval_s: float = 10.0

    #: How often to check whether the recording restarted. A looping bag wraps every
    #: bag-length, and every second spent unpaired after a wrap is a second the operator
    #: cannot add, so this runs far more often than the stats log.
    epoch_check_interval_s: float = 1.0

    def __post_init__(self):
        if self.window_ms <= 0.0:
            raise ValueError(
                f"window_ms must be positive, got {self.window_ms}. An infinite window "
                f"(0) makes conflux pair messages by ARRIVAL ORDER rather than by time: "
                f"it skips the pruning step that aligns the buffers, so two streams at "
                f"different rates drift apart without bound (measured: 53s inside one "
                f"'synchronized' group at 10Hz vs 1Hz). The drift is silent, because a "
                f"solve from mismatched detections still reports a low reprojection "
                f"error. Pick a window a little wider than one frame interval instead."
            )
        if self.drop_policy not in DROP_POLICIES:
            raise ValueError(
                f"drop_policy must be one of {DROP_POLICIES}, got {self.drop_policy!r}"
            )
        if self.queue_size < 2:
            raise ValueError(f"queue_size must be at least 2, got {self.queue_size}")
        if self.max_pair_age_s < 0.0:
            raise ValueError(
                f"max_pair_age_s must be zero (disabled) or positive, got "
                f"{self.max_pair_age_s}"
            )
