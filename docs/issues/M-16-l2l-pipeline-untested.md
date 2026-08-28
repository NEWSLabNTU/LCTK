# M-16 · LiDAR-to-LiDAR pipeline has never been run end-to-end

- **Severity:** Medium
- **Area:** lidar_to_lidar_solver / pipeline
- **Status:** Open
- **Verified:** By admission — CLAUDE.md and the L2L section both say "This pipeline is not yet tested"
- **Related:** [M-04](./archive/M-04-l2l-wallclock-staleness.md), [M-05](./archive/M-05-l2l-wrong-pose-field.md),
  [H-13](./H-13-l2l-latest-board-pair-overwrites-extrinsic.md), [M-17](./M-17-initial-pose-rewrite-unverified-bbox-path.md),
  [M-23](./M-23-two-lidar-example-topics-unreachable.md)

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

## Update (2026-08-28) — later evidence is in tension with "never run end-to-end"

This issue's title and body, filed 2026-07-16, say the pipeline "has never been run end-to-end."
Two later issues describe what reads like a real two-LiDAR rig session:

- [H-13](./H-13-l2l-latest-board-pair-overwrites-extrinsic.md), filed 2026-08-19, is explicitly
  "observed during two-LiDAR field validation" and describes watching the published
  `lidar_to_lidar_solver` transform skew as the board was moved between placements — behavior
  that requires a running solver receiving real synchronized pairs.
- [M-17](./M-17-initial-pose-rewrite-unverified-bbox-path.md)'s 2026-08-14 update says the
  corner-aligned board frame "was run against the real two-LiDAR rig (Seyond + Velodyne) using
  the `TWO_LIDAR_*` recordings," that "the LiDAR-to-LiDAR extrinsic publishes plausible relative
  poses," and cites this as the field validation that closed
  [M-20](./archive/M-20-board-frame-edge-aligned-vs-diamond-naming.md).

Read plainly, both are evidence that the L2L pipeline **has** been exercised on real hardware
since this issue was filed. This issue is not being closed on that basis, for two reasons:

1. It's unclear whether that 2026-08-14 session used the committed
   `config/examples/two_lidar.yaml`. See [M-23](./M-23-two-lidar-example-topics-unreachable.md):
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
