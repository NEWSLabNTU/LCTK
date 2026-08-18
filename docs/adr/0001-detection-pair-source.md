# 0001. The synchronized detection pair is one module, and it refuses an infinite window

- **Date:** 2026-08-15
- **Status:** accepted
- **Review that produced it:** [2026-08-15 architecture review](./2026-08-15-architecture-review.md), candidate 1

## Context

Three solver nodes — `lidar_to_camera_solver`, `lidar_to_lidar_solver` and the superseded
`extrinsic_solver_node` — each wired conflux by hand. Each held a large interface to do it: nine
parameters, two topics, four counters, and the question of what an absent pair means. The
implementation behind that interface, in each node, was a couple of caches. Shallow, three times over.

Two defects were found on 2026-08-15 while debugging a calibration run that could not add a single
detection pair:

1. **Pairing by arrival order.** Conflux matches by time only when a finite window is set: with an
   infinite window, `State::try_match` skips the pruning step that aligns the buffers and pops the
   front of each one. Two streams at different rates then drift apart without bound. Measured against
   this repository's conflux build: camera 10 Hz + LiDAR 1 Hz reached a **53 s** gap inside one
   "synchronized" group; 30 Hz + 10 Hz saturated at 10 s; the seyond rig (5.4 Hz / 4.4 Hz) passed
   **11 s** and was still climbing. `calibrate.launch.py` shipped `sync_tolerance_ms: 0.0` — the
   infinite window — for offline playback, the default for every recorded run.

   This is worse than a stall because it succeeds. The solver pairs ArUco corners with a board pose on
   the assumption both sensors saw the board at one instant. Pair frames 11 s apart and the board has
   moved, so the extrinsic is wrong while the reprojection error still looks fine.

2. **A replayed recording stops it permanently.** Conflux is strictly time-ordered: `State::push`
   rejects any message stamped at or before the group it last emitted, and that commit time only moves
   forward. Both detectors copy the stamp of the message they consumed (`aruco_locator_node` from the
   image, `lidar_board_detector` from the point cloud), so every new bag — and every `--loop` wrap —
   sends the stamps backward. Measured on a looping 19.8 s bag: groups froze at 32 while `dropped`
   climbed 1:1 with `received` on both streams for the next four minutes.

A third failure surfaced after this module shipped, and it is the sharpest of the three.
`State::try_match` runs its readiness check (`inf_ts + window > sup_ts`) **before** the pruning step
that drops stale messages. When one stream's buffer holds only a previous recording's stamps — the
operator plays a background bag with no camera in it, then a calibration bag — the new recording's
stamps never overlap the old ones, the check always returns early, the prune never runs, and under
`reject_new` the buffer never drains. Measured: `groups=0`, permanently, with no group having ever
been emitted. That state is unrecoverable from inside conflux and deserves an upstream fix; the
reset below is a workaround from outside.

The first two were fixed in `lidar_to_camera_solver` alone, because there was nowhere else for the fix
to live.
At the moment of the review `lidar_to_lidar_solver` still had the hardcoded `0.0` at
`calibrate.launch.py:285` and still had no epoch reset. That is the cost of the missing module, stated
as a bug rather than as a principle — and adopting this decision is what closed it: that node now
takes the mode-derived window preset and inherits the epoch reset from the module.

## Decision

**The synchronized detection pair becomes one deep module, `lctk_sync.DetectionPairSource`, and every
maintained solver node consumes it rather than conflux directly.** The superseded
`extrinsic_solver_node` is excluded because Stage 3 of the
[diamond-frame plan](../superpowers/specs/2026-08-14-lidar-to-camera-solver-diamond-frame.md)
deletes it.

Its interface is: construct it with a node, topics and message types; optionally register `on_pair`
for consumers that act on every pair; call `take_fresh_pair()` for the newest usable pair or the
reason there is none; call `status_line()` for what synchronization is doing. Behind it sit the
synchronizer, the window policy, the epoch reset, the staleness gate, the skew measurement and the
refusal diagnosis.

**`PairSourceConfig` refuses `window_ms <= 0`**, with a message naming arrival-order pairing and the
measured drift. No launch file, node, or future caller can request the setting that caused defect 1.

**Conflux's own rule is not changed.** Strict time ordering is correct for a live sensor; what is
wrong is asking it to serve a workflow that replays recordings. The module recognises that the source
has changed underneath it — nothing has paired for a couple of seconds *while every stream is still
delivering* — and starts a fresh matching engine. A stream that has gone quiet is a detector fault, or
simply a background bag with no camera in it, and must not reset anything: clearing the buffers would
hide it.

`lctk_sync` is a new ament_python package rather than an upstream change to
`jerry73204/conflux`, because the epoch policy encodes LCTK's workflow (replaying many bags to collect
board placements), not a general synchronization rule.

## Consequences

**Easier.** A sync defect is fixed once. A solver learns two calls instead of nine parameters plus
conflux's ordering contract. Stage 2's `continuous` mode gets the fixed behaviour by construction. The
pure decisions (`should_reset_for_new_epoch`, `sync_wait_diagnosis`, `sync_pair_staleness_error`,
`sync_health_warning`, `format_sync_stats`) are the module's own test surface, exercised through the
interface rather than beside it in a 2187-line node.

**Harder.** A caller that genuinely wants order-based pairing — two streams in true 1:1 lockstep —
must now say so some other way; the escape hatch was removed deliberately.

`ROS2Synchronizer.reset()` now provides the epoch seam directly. It replaces only the matching
engine, preserving ROS subscriptions, the synchronized callback and cumulative statistics.
`DetectionPairSource` therefore no longer imports the bare Conflux engine or mutates the wrapper's
private `_sync` handle.

**Ruled out.** Restamping detections with wall-clock time to dodge the backward jumps. It makes stamps
monotonic but destroys what the pairing is for: after restamping, a stamp says when a result finished
computing, so pairing would reflect processing latency (ArUco ~5.4 Hz behind a backlogged republish,
ICP 200–600 ms) rather than observation time — reintroducing defect 1 in a form no window can catch.

## Implementation status — 2026-08-18

`lidar_to_camera_solver` and `lidar_to_lidar_solver` both consume
`lctk_sync.DetectionPairSource`. They inherit its finite-window rule, replay recovery, skew/status
reporting and empty-group policy. The pull-based LiDAR-camera solver additionally uses its freshness
gate and refusal diagnosis; the push-based LiDAR-to-LiDAR solver receives each pair through
`on_pair`. The only direct Conflux caller left is the legacy `extrinsic_solver_node`, which is
scheduled for deletion rather than migration.

The production migration and public reset seam are complete. The remaining work under this ADR is
real-ROS contract coverage through the module interface: in-window delivery, outside-window refusal,
empty-group diagnosis, stale-pair refusal and autonomous recovery after a timestamp rewind.
