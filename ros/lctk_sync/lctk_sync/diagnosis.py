"""How a solver decides whether it has a usable pair of detections, and why not.

Pure functions over plain values: no `rclpy`, no conflux, no node. Every decision the
synchronized-pair machinery makes is expressed here so that the whole decision table is
testable without a ROS graph, and so that the reason an operator is shown is written
once rather than per solver node.
"""

from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class SyncGroupSummary:
    """What the last synchronized group actually contained, and how old it is.

    Kept for EVERY group, including the ones that are ignored, because "nothing to add"
    and "the LiDAR side stopped detecting" are different problems and the operator can
    only act on the second one.
    """

    aruco_count: int
    board_count: int
    age_s: float


def sync_pair_staleness_error(*, age_s: float, max_age_s: float) -> Optional[str]:
    """Refuse a cached detection pair that is too old to be what the operator sees.

    The cached pair used to live forever, so `add_detection` kept succeeding long after
    playback stopped — buffering a board pose from an unknown moment while looking like
    it worked. ``max_age_s <= 0`` disables the gate.
    """
    if max_age_s <= 0.0 or age_s < max_age_s:
        return None

    return (
        f"The newest synchronized detection pair is {age_s:.1f}s old (limit "
        f"{max_age_s:.1f}s), so it is not what you are looking at now. Playback has "
        f"probably stopped, or one of the detectors has. Adding it would buffer a "
        f"board pose from an unknown moment. Set max_pair_age_s to 0 to disable this "
        f"check."
    )


def should_reset_for_new_epoch(
    *,
    previous_dropped: dict,
    current_dropped: dict,
    last_group_age_s: Optional[float],
    quiet_after_s: float = 2.0,
) -> bool:
    """Has the message source restarted (a new bag, or a `--loop` wrap)?

    Conflux is strictly time-ordered: it rejects any message stamped at or before the
    newest group it emitted, and that commit time only moves forward. Correct for a
    live sensor; wrong for this workflow, where the operator replays several recorded
    bags and both detectors copy the stamp of the message they consumed. Every new bag
    sends the stamps backward, and conflux then rejects everything, forever.

    Rather than weaken conflux's rule, recognise the restart here. Its signature needs
    no access to the raw stamps: groups have stopped, and EVERY stream is having its
    messages thrown away. One stream being dropped alone is a detector problem, and no
    drops at all is a dead stream -- neither should trigger a silent reset.
    """
    if last_group_age_s is None or last_group_age_s < quiet_after_s:
        return False
    if not current_dropped:
        return False

    return all(
        count > previous_dropped.get(topic, 0)
        for topic, count in current_dropped.items()
    )


def format_sync_stats(
    *,
    received: dict,
    dropped: dict,
    rejected: dict,
    groups: int,
    skew_ms: Optional[Tuple[float, float]] = None,
) -> str:
    """One line saying what each input stream did, from this node's point of view.

    When synchronized groups stop arriving while both detectors are visibly
    publishing, the question is which stream stopped reaching THIS node — and
    whether the synchronizer threw it away (buffer full, or too late to group).
    `received` answers the first; `rejected` and `dropped` answer the second.
    """
    topics = sorted(set(received) | set(dropped) | set(rejected))
    if not topics:
        return f"sync: groups={groups}, no input streams registered"

    # The skew INSIDE a group is what says whether "synchronized" means anything: the
    # solver pairs ArUco corners with a board pose on the assumption both saw the board
    # at one instant. Conflux's infinite window let this grow without bound while every
    # other counter stayed clean, so report it where it cannot be missed.
    skew = ""
    if skew_ms is not None:
        skew = f" pair skew last={skew_ms[0]:.1f}ms max={skew_ms[1]:.1f}ms;"

    parts = [
        f"{topic}: received={received.get(topic, 0)} "
        f"rejected={rejected.get(topic, 0)} dropped={dropped.get(topic, 0)}"
        for topic in topics
    ]
    return f"sync: groups={groups};{skew} " + "; ".join(parts)


def sync_health_warning(
    *,
    previous: dict,
    current: dict,
    last_group_age_s: Optional[float],
    quiet_after_s: float = 10.0,
) -> Optional[str]:
    """Warn when synchronized groups have stopped, and say what the evidence points at.

    Two failures look identical from the operator's chair — the TUI just says there is
    no pair — but have opposite causes:

    - a stream stopped reaching this node (its `received` count froze), which is a
      detector or a topic-wiring problem;
    - both streams keep arriving but no group comes out, which is a synchronizer
      problem.

    Returns ``None`` while groups are flowing, and before the first group ever arrives
    (that is waiting for playback, not a stall).
    """
    if last_group_age_s is None or last_group_age_s < quiet_after_s:
        return None

    silent = sorted(
        topic
        for topic, count in current.items()
        if count == previous.get(topic, -1) and count > 0
    )
    if silent:
        return (
            f"No synchronized group for {last_group_age_s:.0f}s: no new messages on "
            f"{', '.join(silent)}. That stream stopped reaching this node -- check the "
            f"node that publishes it, and that the topic is wired to this one."
        )

    arriving = [topic for topic, count in current.items() if count > previous.get(topic, 0)]
    if arriving:
        return (
            f"No synchronized group for {last_group_age_s:.0f}s, yet messages are still "
            f"arriving on both streams ({', '.join(sorted(arriving))}). The "
            f"synchronizer is not pairing them -- compare their header stamps."
        )

    return None


def sync_wait_diagnosis(summary: Optional[SyncGroupSummary]) -> str:
    """Explain WHY there is no usable detection pair, in terms of what to fix.

    A pair needs both arrays non-empty in the same synchronized group. Both detectors
    publish empty arrays when they fail, so a steady stream of groups can still yield
    nothing usable — which is what a bag whose ArUco and LiDAR detections never overlap
    in time looks like from here.
    """
    if summary is None:
        return (
            "No synchronized detection pair available: no synchronized group has "
            "arrived at all. Check that both aruco_detections and "
            "calibration_board_detections are publishing."
        )

    parts = [
        f"No usable synchronized detection pair. The last synchronized group "
        f"({summary.age_s:.1f}s ago) held {summary.aruco_count} ArUco marker(s) and "
        f"{summary.board_count} board detection(s); both must be non-empty."
    ]
    if summary.aruco_count == 0:
        parts.append(
            "The camera side is empty: aruco_locator_node is detecting no markers "
            "(board too far, out of frame, or motion-blurred)."
        )
    if summary.board_count == 0:
        parts.append(
            "The LiDAR side is empty: lidar_board_detector is not detecting the board "
            "(check its crop box / bbox_free candidates and its own log)."
        )
    return " ".join(parts)
