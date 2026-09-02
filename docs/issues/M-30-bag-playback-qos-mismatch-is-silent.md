# M-30 · A bag session in the default mode receives no LiDAR, and says so only once

- **Severity:** Medium
- **Area:** lctk_launch (sessions), calibrate.launch.py
- **Status:** 🔴 Open
- **Found:** 2026-09-02, first run of `solid600-handheld-zed` against its recording
- **Related:** [M-26 (archived)](./archive/M-26-two-lidar-example-topics-unreachable.md), [M-29](./M-29-sample-data-path-dead-shared-bbox-and-icp-gate.md)

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

## Workaround

`just mode=realtime run <session>` for any bag session. Documented in
`sessions/solid600-handheld-zed/README.md`.

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
