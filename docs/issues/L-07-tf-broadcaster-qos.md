# L-07 · tf_tree_broadcaster QoS may be incompatible with realtime solver publishers

- **Severity:** Low (needs runtime confirmation)
- **Area:** lctk_launch tf_tree_broadcaster
- **Status:** Fixed (2026-07-11)
- **Verified:** Static review — needs verification of solver publisher QoS
- **Location:** `ros/lctk_launch/lctk_launch/tf_tree_broadcaster.py:38-50` (RELIABLE + TRANSIENT_LOCAL)

## Problem

The broadcaster subscribes to the solver `extrinsic_transform` topics with RELIABLE + TRANSIENT_LOCAL QoS. If the solvers publish VOLATILE / BEST_EFFORT (as realtime mode might), the QoS is incompatible and no transform is delivered — so the TF tree is never broadcast, with no error.

## Failure scenario

In realtime mode, `/tf_static` is silently never populated; downstream TF consumers see nothing.

## Suggested fix

Confirm the solver publisher QoS and align the subscription (or use a compatible / matched QoS profile). Log a QoS-incompatibility warning if ROS reports one.

## Resolution (2026-07-11)
Confirmed real: the solvers publish `extrinsic_transform` BEST_EFFORT in realtime,
while the broadcaster subscribed RELIABLE + TRANSIENT_LOCAL (incompatible → no
delivery). Changed the broadcaster subscription to BEST_EFFORT + VOLATILE, which
is compatible with the publisher QoS in both offline and realtime modes.
