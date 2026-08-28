# M-26 · `two_lidar.yaml` names topics no in-repo data source ever publishes

- **Severity:** Medium
- **Area:** lctk_launch / config/examples, lctk_sample_data
- **Status:** Open
- **Verified:** By code trace (2026-08-28) — the config, both sample-data launch files, the
  recorded-bag README, and the justfile were all read directly
- **Related:** [M-16](./M-16-l2l-pipeline-untested.md), [H-13](./H-13-l2l-latest-board-pair-overwrites-extrinsic.md)

## Problem

`ros/lctk_launch/config/examples/two_lidar.yaml`, the config `just two-lidar` launches, subscribes:

```yaml
devices:
  lidars:
    top_lidar:
      pointcloud_topic: /velodyne_points
      frame_id: velodyne
    front_lidar:
      pointcloud_topic: /iv_points
      frame_id: seyond
```

No in-repo data source publishes either topic:

- `ros/lctk_sample_data/launch/two_lidar.launch.xml` — the two-LiDAR pcap playback — publishes
  `/sensing/lidar/top/pointcloud_raw` (`frame_id: velodyne_top`) and
  `/sensing/lidar/front/pointcloud_raw` (`frame_id: velodyne_front`), from
  `data/3/lidar.pcap` and `data/4/lidar.pcap` respectively.
- The recorded rig bags described in `ros/lctk_sample_data/bags/README.md` (gitignored, obtained
  out-of-band) carry `/lidar/vlp32/velodyne_points` (`frame_id: velodyne`) and
  `/lidar/falcon/iv_points` (`frame_id: seyond`) — the frame IDs match `two_lidar.yaml`, but the
  topics still don't: the config is missing the bags' `/lidar/vlp32/` and `/lidar/falcon/` topic
  prefixes.

Worse, **no justfile recipe launches `two_lidar.launch.xml` at all.** `just sample-data` only runs
`lctk_sample_data lidar_camera.launch.xml` (single-lidar + camera playback); there is no
`two-lidar`-flavored sample-data recipe. So `just two-lidar` starts a full calibration graph
(2× `lidar_board_detector`, 1× `lidar_to_lidar_solver`) with nothing in the repo that can feed it
data — not even in principle, since the launch recipe to produce the closest-matching topics
doesn't exist — short of an operator hand-remapping topics or replaying a rig bag with `ros2 bag
play` and manual `-remap` flags.

This predates the 2026-08-28 selectable-targets cutover (commit `24224c8`, "cut the maintained
examples over to selectable targets", rewrote only the `markers:` section of this file); the
`devices.lidars.*.pointcloud_topic` values were already `/velodyne_points` / `/iv_points`
beforehand, confirmed via `git show 24224c8^:ros/lctk_launch/config/examples/two_lidar.yaml`.

## Consequence

The two-LiDAR path is unrunnable out of the box via the one documented recipe (`just two-lidar`),
silently: the graph starts, nodes come up, and nothing ever arrives — there is no error, just an
indefinitely empty pipeline. This compounds [M-16](./M-16-l2l-pipeline-untested.md): if the
"two-lidar variant" of `just sample-data` that M-16's suggested fix refers to doesn't exist, M-16's
prescribed verification procedure can't be followed as written either. It's also possible this is
part of why the L2L pipeline's field validation ([H-13](./H-13-l2l-latest-board-pair-overwrites-extrinsic.md),
M-17's 2026-08-14 update) was run some other way than `just two-lidar` — see the note added to
M-16.

## Suggested fix

Pick one and make `just two-lidar` actually work end-to-end:

1. Retopic `two_lidar.yaml` to match `two_lidar.launch.xml`'s defaults
   (`/sensing/lidar/top/pointcloud_raw` / `/sensing/lidar/front/pointcloud_raw`,
   `velodyne_top` / `velodyne_front`), and add a `just` recipe that launches
   `lctk_sample_data two_lidar.launch.xml` (mirroring how `sample-data` wraps
   `lidar_camera.launch.xml`), or fold it into `two-lidar` directly.
2. Or, if the intent was always the recorded rig bags rather than the pcap playback, retopic to
   `/lidar/vlp32/velodyne_points` / `/lidar/falcon/iv_points` and document that `just two-lidar`
   requires `ros2 bag play` against a `TWO_LIDAR_*` bag first, the way `two_lidar.yaml`'s header
   comment currently implies pcap playback but doesn't say so explicitly.

Either way, add a check (or at minimum a doc note) that keeps the example's topics and the data
source's topics from drifting apart again.
