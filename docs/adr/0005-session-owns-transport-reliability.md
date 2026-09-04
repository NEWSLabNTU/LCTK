# 0005. The session owns transport reliability, resolved per device

- **Date:** 2026-09-04
- **Status:** accepted

## Context

A ROS 2 subscriber that requests RELIABLE and meets a BEST_EFFORT publisher receives nothing.
Neither side errors. The graph launches, every node reports healthy, and no message arrives.

Until now one launch argument, `mode`, chose that reliability for the whole graph: `offline` meant
RELIABLE, `realtime` meant BEST_EFFORT. Three measurements showed the argument cannot hold the
decision:

- **It is per topic, not per graph.** `ros/lctk_sample_data/bags/TWO_LIDAR_1` records
  `/lidar/falcon/iv_points` with `reliability: 1` (RELIABLE) beside
  `/lidar/vlp32/velodyne_points` with `reliability: 2` (BEST_EFFORT). No single value serves both.
  `just mode=offline run twolidar-vlp32-falcon` — the documented invocation — left the VLP-32
  detector without a single cloud while the Falcon detector warmed up normally, unnoticed until
  2026-09-04.
- **It is not derivable from `data.kind`.** `twolidar-vlp32-falcon` is a `bag` that ran at
  `offline`; `vlp32-zed-hollow` is `live` and required `realtime`, because its operator replays
  bags by hand. Both directions have a counter-example among shipped sessions.
- **Nothing tested it.** No test in the repository asserted `use_best_effort_qos` in either mode on
  any node, and the L-05 validity guard on `mode` was untested. The behaviour could have been
  deleted silently.

`play_args` (added 2026-09-04, `a705584`) existed only to override playback QoS from the command
line, because `mode` could not express what the recording already knew. It reached the data half
while `mode` reached the calibration half, and no code compared them: the two could contradict each
other and nothing noticed.

Tracked as [M-30](../issues/archive/M-30-bag-playback-qos-mismatch-is-silent.md), whose candidate
fixes (1) "check it at parse time" and (2) "let the manifest state it" this decision adopts
together.

## Decision

**Transport reliability is a property of the session, resolved per device**, by
`ros/lctk_launch/lctk_launch/transport.py`, in three steps:

1. What the manifest states — `qos:` on a device, or `qos:` at the top level as a session default.
   Values are `reliable` and `best_effort`.
2. What the recording offers — `offered_qos_profiles` in the bag's `metadata.yaml`. `kind: bag`
   only.
3. `best_effort`, the only value compatible with a publisher of either kind.

A stated value is checked, not trusted: under `kind: bag`, stating `reliable` for a topic the
recording offers `best_effort` is refused at parse time with the topic named.

**Only sensor subscriptions take an answer from the session.** LCTK's own detection and transform
topics are ours on both ends and are pinned RELIABLE inside the nodes. That is what allows two
detectors with different input reliability to feed one lidar-to-lidar solver, which subscribes to
both with a single profile. `pointcloud_image_overlay` is a viewer and stays BEST_EFFORT
unconditionally.

**Queue depth is not a session concern** and is fixed at 10. It previously travelled with
reliability — the two `mode` branches selected whole different rclrs profiles differing 10 against
1, which no document mentioned — but the nodes already discard stale frames with the store-latest
ArcSwap pattern, so a depth of 1 only cost frames during a burst.

The `mode` argument and `play_args` are deleted, from the justfile, `session.launch.py`,
`session_data.launch.py` and `calibrate.launch.py`.

## Consequences

**Easy.** A session runs correctly with no transport flag, including a recording whose topics
disagree with each other. The one silent pairing becomes a refusal naming the topic. An operator
who knows their rig can still state the answer, and gets it verified against the recording rather
than merely accepted. The player is no longer overridden, so what a bag claims about itself is
never rewritten.

**Hard.** A rig whose driver publishes RELIABLE and whose operator wants every message must now say
`qos: reliable` on that device; the default is the lossy-but-compatible one. Nothing infers it,
because a live publisher's offered QoS cannot be read before the graph starts.

**Ruled out.** Reliability as a command-line argument, in any form. It is not a property of the
run; it is a property of what publishes each topic, and the two shipped counter-examples above are
what a graph-wide flag gets wrong. Also ruled out: overriding the player's QoS, which masks a
genuine mismatch elsewhere by rewriting the recording's own claim (M-30's candidate 3).

**Follow-on.** A mistyped `qos` would have been discarded silently, because `config_parser.py`
accepted unknown keys everywhere except `assisted:`. That is closed by
[ADR-0006](./0006-one-manifest-one-strictness.md).
