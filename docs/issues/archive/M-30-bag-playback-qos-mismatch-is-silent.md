# M-30 · A bag session in the default mode receives no LiDAR, and says so only once

- **Severity:** Medium
- **Area:** lctk_launch (sessions), calibrate.launch.py
- **Status:** 🟢 Fixed 2026-09-04 — the session owns transport reliability, resolved per
  device from the recording, and the one silent pairing is refused at parse time
- **Found:** 2026-09-02, first run of `solid600-handheld-zed` against its recording
- **Related:** [M-26 (archived)](./M-26-two-lidar-example-topics-unreachable.md), [M-29](../M-29-sample-data-path-dead-shared-bbox-and-icp-gate.md)

## Problem

Launching a `kind: bag` session with the default `mode:=offline` produces **zero
board detections**. The camera half is healthy — 1610 ArUco detections, none
dropped — so the synchronizer, the solver and the review page all look alive
while the pipeline is dead:

```
sync: groups=0; aruco_detections: received=1610 rejected=1510 dropped=0;
      calibration_board_detections: received=0 rejected=0 dropped=0
```

The cause appears exactly once, in the first second of the log, from a node
nobody is watching:

```
[rosbag2_player] New subscription discovered on topic '/velodyne_points',
requesting incompatible QoS. No messages will be sent to it.
Last incompatible policy: RELIABILITY_QOS_POLICY
```

`mode` selects transport QoS: `offline` is RELIABLE, `realtime` is BEST_EFFORT.
`ros2 bag play` republishes with the QoS profile the topic was **recorded** with,
and a LiDAR driver publishing BEST_EFFORT records BEST_EFFORT. A RELIABLE
subscriber cannot match a BEST_EFFORT publisher, so the detector's subscription
is never fed.

The name is the trap. `offline` reads as *the mode for recorded data*, and its
documented use case in `CLAUDE.md` is "Recorded data (rosbags or the pcap/avi
sample playback)". For the pcap/avi path that is right — LCTK's own driver
publishes RELIABLE. For a bag it is wrong whenever the original publisher was
BEST_EFFORT, which is the common case for LiDAR.

## Why nothing caught it

Same shape as M-29 and M-26: every component reports success and only the
composition is dead. Worse than both, because here the diagnosis *is* printed —
once, at startup, by `rosbag2_player`, above hundreds of lines of healthy sync
statistics. `just smoke` covers only `pcap_avi` sessions, where the mismatch
cannot occur.

## Workaround (historical)

`just mode=realtime run <session>` for any bag session, documented in the affected
sessions' READMEs. Both the flag and the workaround are gone; see the resolution below.

## Candidate fixes

1. **Check it at parse time.** `session.py` already reads the bag's
   `metadata.yaml`, which records each topic's `offered_qos_profiles`. Comparing
   that against the reliability `mode` implies turns a silent dead pipeline into
   a startup refusal that names the fix — the same move M-26 made for topic
   names, using a field already being parsed.
2. **Let the manifest state it.** A `data.qos:` key, so the session owns the
   judgement rather than a command-line flag the operator must remember.
3. **Override the player.** `ros2 bag play --qos-profile-overrides-path` can
   force RELIABLE, but that silently rewrites what the recording claims and
   would mask a genuine mismatch elsewhere.

(1) is preferred: it is a check, not a behaviour change, and it cannot make an
existing session behave differently.

## Resolution

Candidate fixes (1) and (2) landed together, because neither is sufficient alone: a check
needs something to check *against*, and a manifest key that nothing verifies is one more
thing to get wrong silently.

`lctk_launch/transport.py` resolves the reliability of each **sensor** topic in three steps:
what the manifest states (`qos:` on a device, or top-level as a session default), else what
the recording offers (`offered_qos_profiles` from `metadata.yaml`, which `session.py` was
already opening for M-26's topic check), else `best_effort` — the only value compatible with
a publisher of either kind. Stating `reliable` for a topic a recording offers `best_effort`
is refused at parse time with the topic named, which is (1).

The `mode` argument is deleted, and with it `play_args`, which existed only to work around
`mode` being unable to express what the bag already knew. Candidate (3) is therefore moot:
nothing overrides the player, and the recording's own claim is never rewritten.

**A second bug fell out of the same measurement.** `TWO_LIDAR_1` records a RELIABLE Falcon
beside a BEST_EFFORT VLP-32, so no single graph-wide answer could serve both. `just
mode=offline run twolidar-vlp32-falcon` — the documented invocation, with no mode override —
left the VLP-32 detector without a single cloud while the Falcon detector warmed up
normally, and had been doing so unnoticed. After the change both detectors reach
`background warmup 19/20` within milliseconds of each other.

That is also why LCTK's own detection and transform topics are now pinned RELIABLE inside
the nodes rather than following the sensor answer: two detectors with different input
reliability feed one lidar-to-lidar solver, which can only ask for one thing.

Queue depth stopped travelling with reliability at the same time. The two `mode` branches
selected whole different rclrs profiles that differed 10 against 1 — undocumented, and
noticed only while tracing this — but the nodes discard stale frames with the store-latest
ArcSwap pattern, so a depth of 1 only cost frames during a burst. It is fixed at 10.

Guarded by `ros/lctk_launch/test/test_transport.py` and four tests in `test_config_parser.py`,
including the mixed-reliability recording that produced the two-lidar failure.
