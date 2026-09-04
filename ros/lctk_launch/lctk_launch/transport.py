"""Which reliability each sensor topic is received with.

A subscriber that asks for RELIABLE and meets a BEST_EFFORT publisher receives
nothing at all -- no error, no rejection, just a graph that launches cleanly and
sits silent. That is M-30, and it shipped: `twolidar-vlp32-falcon` ran with a
RELIABLE subscriber against a recording whose VLP-32 offers BEST_EFFORT, so the
Falcon detector warmed up and the VLP-32 one never received a cloud.

This module makes that decision a property of the session rather than of a
command-line flag. Three rules, in falling order of authority:

1. What the manifest states, per device or as a session-wide default.
2. What the recording offers, read from the bag's own `metadata.yaml`.
3. BEST_EFFORT, which is compatible with every publisher.

A stated value is not merely trusted. Under `kind: bag` the recording knows the
answer, so a claim that contradicts it is refused at parse time with the topic
named -- turning the operator's knowledge into a checked claim rather than a
silent one.

Reliability is the only axis here. Queue depth used to travel with it -- the
`mode` argument selected two whole QoS profiles that differed in depth as well,
10 against 1 -- but the nodes already discard stale frames with the store-latest
ArcSwap pattern, so a depth of 1 only cost frames. Depth is now fixed in the
nodes and is not a session concern.
"""

from __future__ import annotations

from pathlib import Path

RELIABLE = "reliable"
BEST_EFFORT = "best_effort"
RELIABILITIES = (RELIABLE, BEST_EFFORT)

# rmw_qos_reliability_policy_t, as rosbag2 writes it into metadata.yaml.
_RMW_RELIABILITY = {
    0: None,  # SYSTEM_DEFAULT -- says nothing, so treat the topic as unknown
    1: RELIABLE,
    2: BEST_EFFORT,
}


class TransportError(Exception):
    """A stated reliability cannot receive the data it is pointed at."""


def parse_reliability(value: object, where: str) -> str:
    """Validate one manifest `qos:` value."""
    if not isinstance(value, str) or value not in RELIABILITIES:
        raise ValueError(
            f"{where} sets qos {value!r}; expected one of "
            f"{', '.join(RELIABILITIES)}. This is the reliability the "
            "subscriber asks for, not a profile name."
        )
    return value


def bag_offered_reliability(bag: Path) -> dict[str, str]:
    """Read each recorded topic's offered reliability from `metadata.yaml`.

    A topic with several publishers gets the most permissive answer that can
    receive all of them: one BEST_EFFORT publisher among many is enough to make
    a RELIABLE subscriber miss that publisher entirely.

    Topics whose profile is absent, empty, or SYSTEM_DEFAULT are omitted rather
    than guessed at -- an older recording simply does not carry the answer, and
    the caller falls back to BEST_EFFORT, which can receive either.
    """
    import yaml

    metadata = yaml.safe_load((bag / "metadata.yaml").read_text(encoding="utf-8"))
    information = metadata["rosbag2_bagfile_information"]

    offered: dict[str, str] = {}
    for entry in information["topics_with_message_count"]:
        topic_metadata = entry["topic_metadata"]
        name = topic_metadata["name"]
        # rosbag2 has written this field both as an embedded YAML string and as
        # a real list, depending on version. Accept both.
        raw = topic_metadata.get("offered_qos_profiles")
        if isinstance(raw, str):
            raw = yaml.safe_load(raw) if raw.strip() else None
        if not raw:
            continue

        policies = {
            _RMW_RELIABILITY.get(profile.get("reliability"))
            for profile in raw
            if isinstance(profile, dict)
        }
        if None in policies or not policies:
            continue
        offered[name] = BEST_EFFORT if BEST_EFFORT in policies else RELIABLE

    return offered


def resolve_reliability(
    topic: str,
    stated: str | None,
    offered: dict[str, str] | None,
) -> str:
    """Decide how one sensor topic is subscribed to.

    `offered` is what the recording says, or `None` when there is no recording
    to ask -- a live rig, the pcap/avi playback, or a plain calibration config.
    """
    recorded = offered.get(topic) if offered else None

    if stated is None:
        return recorded or BEST_EFFORT

    if stated == RELIABLE and recorded == BEST_EFFORT:
        raise TransportError(
            f"'{topic}' is set to qos 'reliable', but the recording offers it "
            "'best_effort'. A RELIABLE subscriber receives nothing at all from "
            "a BEST_EFFORT publisher -- the graph would launch cleanly and stay "
            "silent (M-30). Use 'best_effort' here, or omit qos and let the "
            "recording decide."
        )
    return stated


def resolve_topics(
    topics: dict[str, str | None],
    offered: dict[str, str] | None,
) -> dict[str, str]:
    """Resolve a whole `{topic: stated-or-None}` mapping at once."""
    return {
        topic: resolve_reliability(topic, stated, offered)
        for topic, stated in topics.items()
    }
