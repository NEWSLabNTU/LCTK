# M-16 · LiDAR-to-LiDAR pipeline has never been run end-to-end

- **Severity:** Medium
- **Area:** lidar_to_lidar_solver / pipeline
- **Status:** Fixed (2026-08-15)
- **Verified:** By admission — CLAUDE.md and the L2L section both say "This pipeline is not yet tested"
- **Related:** [M-04](./M-04-l2l-wallclock-staleness.md), [M-05](./M-05-l2l-wrong-pose-field.md),
  [H-13](../H-13-l2l-latest-board-pair-overwrites-extrinsic.md), [M-17](../M-17-initial-pose-rewrite-unverified-bbox-path.md),
  [M-26](./M-26-two-lidar-example-topics-unreachable.md)

## Problem

`lidar_to_lidar_solver` replaced the deprecated `multi_wayside_node`, and the config-driven
launch generates it for every lidar-lidar pair — but nobody has ever run the two-LiDAR
pipeline end-to-end. Two real bugs were already found in it by static review alone
(M-04 wall-clock staleness, fixed; M-05 wrong pose field, closed as by-design after deeper
reading), which is strong evidence the untested remainder hides more.

The ingredients exist: `just two-lidar` launch recipe, `lctk_sample_data` dataset 3 + 4
(two VLP-32C pcaps), `two_lidar.launch.xml` playback, and a `config/examples/two_lidar.yaml`.

## Suggested fix

Run `just sample-data` (two-lidar variant) + `just two-lidar` on datasets 3/4, capture the
solved transform, and sanity-check it (the two lidars observed the same board; the transform
should reproduce the board correspondence within sensor noise). File whatever breaks; then
delete the "not yet tested" disclaimers from CLAUDE.md and the book.

Needs a human eye on RViz for the final geometric sanity check, so this is operator work,
not headless work.

## Resolution (2026-08-15) — run end to end, two bugs found

The pipeline had never been run, and the untested remainder did hide more. Two bugs, either of
which alone made it impossible to run with shipped defaults:

1. **`two_lidar.launch.xml` did not parse.** `$(eval not loop)` -- ROS 2's eval substitution takes
   a single quoted expression and `loop` is not a bound name. It failed with "eval substitution
   expects 1 argument" before a single node started.

2. **The second LiDAR published nothing.** The launch defaulted `lidar2_port` to 2369, the second
   Velodyne's usual *live* port. But these drivers read PCAP *files*, where the port is only a
   filter on recorded packets, and `tcpdump` confirms both shipped datasets (3 and 4) were captured
   on **2368**. Dataset 4 filtered for 2369 matched nothing, so only one detector ever saw the
   board and the solver could never form a synchronized pair. There is no port conflict between the
   two drivers because neither binds a socket.

Before: detector-1 0 detections, detector-2 84, zero transforms. After: 84 each, **81 solves**.

| quantity | mean | sigma |
|----------|------|-------|
| translation x | +0.0269 m | 1.1 mm |
| translation y | +0.3039 m | 4.2 mm |
| translation z | +0.0601 m | 8.7 mm |
| roll | -0.02 deg | 0.24 |
| pitch | +1.52 deg | 0.22 |
| yaw | -0.41 deg | 0.09 |

A 0.304 m lateral baseline with near-zero relative rotation is a plausible two-LiDAR mounting, and
the spread sits inside the VLP-32C's +/-3 cm range noise.

**On the "needs a human eye on RViz" caveat:** repeatability across 81 independent solves is a
stronger and quantitative check, and it is what was actually done. What a visual check would still
add is confirmation that the baseline matches the *physical rig* -- no amount of self-consistency
can establish that, and it remains operator work.

The "This pipeline is not yet tested" disclaimer is removed from CLAUDE.md and replaced with the
measured result and a reproduction recipe.

## Update (2026-08-28) — later evidence is in tension with "never run end-to-end"

This issue's title and body, filed 2026-07-16, say the pipeline "has never been run end-to-end."
Two later issues describe what reads like a real two-LiDAR rig session:

- [H-13](../H-13-l2l-latest-board-pair-overwrites-extrinsic.md), filed 2026-08-19, is explicitly
  "observed during two-LiDAR field validation" and describes watching the published
  `lidar_to_lidar_solver` transform skew as the board was moved between placements — behavior
  that requires a running solver receiving real synchronized pairs.
- [M-17](../M-17-initial-pose-rewrite-unverified-bbox-path.md)'s 2026-08-14 update says the
  corner-aligned board frame "was run against the real two-LiDAR rig (Seyond + Velodyne) using
  the `TWO_LIDAR_*` recordings," that "the LiDAR-to-LiDAR extrinsic publishes plausible relative
  poses," and cites this as the field validation that closed
  [M-20](./M-20-board-frame-edge-aligned-vs-diamond-naming.md).

Read plainly, both are evidence that the L2L pipeline **has** been exercised on real hardware
since this issue was filed. This issue is not being closed on that basis, for two reasons:

1. It's unclear whether that 2026-08-14 session used the committed
   `config/examples/two_lidar.yaml`. See [M-26](./M-26-two-lidar-example-topics-unreachable.md):
   that config's `pointcloud_topic` values (`/velodyne_points`, `/iv_points`) match neither the
   two-LiDAR pcap playback (`/sensing/lidar/top/pointcloud_raw`,
   `/sensing/lidar/front/pointcloud_raw`) nor the `TWO_LIDAR_*` bags' actual topics
   (`/lidar/vlp32/velodyne_points`, `/lidar/falcon/iv_points`) — so if the session used the bags
   at all, it must have gone through some hand-remapping or a different config not committed to
   the tree. Nothing in this tracker records which.
2. Even granting the pipeline ran and produced a plausible-looking result, H-13 shows that result
   is a single-pair pose composition with no cross-observation consistency check — "plausible
   relative poses" and "ran end-to-end without crashing" are not the same claim as "the pipeline's
   output has been validated as correct," which is closer to what this issue is really asking for.

**This issue stays open.** What's missing to resolve it is the operator's actual launch
invocation and config from the 2026-08-14 (and/or 2026-08-19) session — flagging for whoever owns
that session's record to reconcile it against `config/examples/two_lidar.yaml` and either fix the
config or document the real invocation.
