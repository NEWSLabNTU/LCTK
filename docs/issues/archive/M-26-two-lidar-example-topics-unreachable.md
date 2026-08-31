# M-26 · `two_lidar.yaml` names topics no in-repo data source ever publishes

- **Severity:** Medium
- **Area:** lctk_launch / sessions (was `config/examples`), lctk_sample_data
- **Status:** Fixed (2026-09-01)
- **Verified:** By code trace (2026-08-28) — the config, both sample-data launch files, the
  recorded-bag README, and the justfile were all read directly
- **Related:** [M-16](./M-16-l2l-pipeline-untested.md), [H-13](../H-13-l2l-latest-board-pair-overwrites-extrinsic.md)

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
part of why the L2L pipeline's field validation ([H-13](../H-13-l2l-latest-board-pair-overwrites-extrinsic.md),
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

## Resolution (2026-09-01)

Fixed by the calibration-sessions change, taking suggested fix 2 — the recorded rig bags
were always the intent — and adding the drift check the closing paragraph asked for.

**The config now names the topics the bag actually records.** `two_lidar.yaml` is gone;
`sessions/twolidar-vlp32-falcon/session.yaml` replaces it and declares
`/lidar/vlp32/velodyne_points` and `/lidar/falcon/iv_points`, read off
`TWO_LIDAR_1/metadata.yaml` rather than guessed.

**The data source is declared, not assumed.** The session carries a `data:` section:

```yaml
data:
  kind: bag
  path: $(session-dir)/bag
```

so `ros2 launch lctk_launch session.launch.py session:=<...>/twolidar-vlp32-falcon` plays the
recording and starts the graph together. The missing recipe this issue complained about is no
longer needed: a session says where its data comes from, so there is nothing left to pair up
by hand.

**The drift is now impossible to reintroduce silently.** Under `kind: bag` the stated topic
set is verified against the recording's own `metadata.yaml` at parse time
(`lctk_launch/session.py`'s `verify_bag_topics`), and a name the bag does not publish is
refused with the list of names it does:

```
<bag> does not publish /velodyne_points. It records: /lidar/vlp32/velodyne_points, ...
```

That converts this issue's exact failure — a graph that launches cleanly and sits empty
forever — into a startup error that states its own fix. It is pinned by two tests:
`test_verify_bag_topics_refuses_and_lists_what_the_bag_has` in
`ros/lctk_launch/test/test_session.py`, and
`test_a_bag_session_naming_a_topic_the_bag_lacks_is_refused_at_parse_time` in
`test_config_parser.py`.

`ros2 run lctk_launch lctk_session check <session>` runs the same verification without
starting a graph, so the answer arrives before the run rather than after.

**Still true, and not this issue's to fix:** the `TWO_LIDAR_*` bags remain gitignored, so the
session needs one placed or symlinked at `$(session-dir)/bag` before it can run. That is
documented in the session's README and in `ros/lctk_sample_data/bags/README.md`. `just
two-lidar` still starts the calibration half only, for an operator playing the bag themselves.
