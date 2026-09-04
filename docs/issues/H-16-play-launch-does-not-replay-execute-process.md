# H-16 · `play_launch` never starts an `ExecuteProcess`, so a bag session plays into an empty graph

- **Severity:** High
- **Area:** lctk_launch (session_data.launch.py), justfile
- **Status:** 🟢 Fixed (2026-09-02)
- **Found:** by an operator running `just mode=realtime assisted solid600-handheld-zed` and reporting that the nodes had not started before the recording ended
- **Related:** [M-30](./archive/M-30-bag-playback-qos-mismatch-is-silent.md), [M-29](./M-29-sample-data-path-dead-shared-bbox-and-icp-gate.md)

## Problem

Every `just` recipe drives the graph through `play_launch`, which runs the launch
tree **twice**:

```
Step 1/2: Recording launch execution...
Step 2/2: Replaying launch execution...
```

The recording pass captures `Node` actions; the replay is what actually starts
them. `play_log/<run>/node/` contains one directory per node and there is no
category for anything else.

`ExecuteProcess` is not recorded. It runs during the **recording** pass — when
no node exists yet — and then does not appear in the replay at all.
`session_data.launch.py` started `ros2 bag play` that way, so under any `just`
recipe the entire recording played into an empty graph and the detectors came up
after it had finished.

## Why nothing caught it

Three layers of concealment:

- **Launch reports success.** Both passes complete, every node starts, the
  playback exits 0. Nothing is in an error state at any point.
- **`just demo` cannot reproduce it.** The `pcap_avi` path is built entirely from
  `Node` actions (the velodyne driver, the decoder), so it replays correctly.
  Only `kind: bag` used `ExecuteProcess`, and no shipped bag session had a
  recording present until now.
- **`just smoke` cannot reproduce it either** — it invokes `ros2 launch`
  directly, where `ExecuteProcess` behaves normally. The bug exists only on the
  path humans use.

This is the third member of the same family as M-29 and M-30: every component
healthy, correct, and reporting success, with only the composition dead.

## Resolution

The player is now a `Node` — `lctk_bag_play`, a console script in `lctk_launch`
— so `play_launch` records and replays it alongside everything else. Verified
under the exact failing command: `play_log/latest/node/bag_player/` now exists,
its log reads `graph is listening after 0.5s; playing`, it exits 0, and the run
produces 385 board detections and 1140 ArUco detections where it previously
produced none.

`lctk_bag_play` also fixes a second ordering problem that raw `ros2 bag play` has
regardless of `play_launch`: it is ready in about a second while the Rust
detectors are still loading, and the bag topics replay BEST_EFFORT and VOLATILE,
so anything published into that gap is lost to a subscriber that has not
appeared yet. It waits for the subscriptions to exist rather than for a guessed
number of seconds — a `sleep` long enough for a loaded Jetson is wasted time
elsewhere and a silent data-loss bug on the next slower machine. Timing out
after 60 s warns and plays anyway: refusing to play the recording would be a
worse failure than playing it into a partly-assembled graph.

A regression guard in `test_launch_files_are_well_formed.py` fails if any launch
file this repo owns constructs an `ExecuteProcess`. It parses rather than greps,
because the comment in `session_data.launch.py` explaining this rule names the
class and a substring check flagged its own documentation.

## The rule this leaves behind

**Anything in a launch file that produces or consumes data must be a `Node`.** If
some future action genuinely cannot be one, it needs its own answer to "what
starts this during the replay pass?" before the guard is relaxed.
