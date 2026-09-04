# vlp32-zed-hollow

VLP-32C plus a ZED against the **static hollow 1000 mm** target, recorded in an
underground car park (`calib_sey_vlp_zed/`).

## Why this is a `live` session and not a `bag` one

The capture is eight bags of about 9.6 s, each holding **one board placement for
its whole length**. A multi-pose calibration therefore means replaying several of
them, by hand, into a graph that stays up between bags — so no single bag belongs
in the manifest. `kind: live` is how a session says *someone else supplies the
data*.

The `data.republish:` bridge is still declared and still launched, because the
ZED records `CompressedImage` and nothing in this tree subscribes to one. No
second terminal running `image_transport republish` by hand.

## Running it

Two terminals. First the graph:

```bash
just solver_mode=manual run vlp32-zed-hollow
```

Then feed it bags, one at a time:

```bash
ros2 bag play /path/to/calib_sey_vlp_zed/calib_1 --clock
```

And drive the buffer from the TUI — `Space` adds the current pair, `p` saves:

```bash
just extrinsic-solver-controller
```

One detection per bag; eight bags give eight placements. `lctk_quality` wants
ten, so expect the diversity check to report a shortfall.

The reliability each sensor topic is subscribed with is resolved per device from
the recording itself (`offered_qos_profiles` in the bag's `metadata.yaml`), so
nothing has to be remembered on the command line. This session used to require
`mode=realtime` because the graph-wide flag could not say what the bag already
knew: its LiDAR replays BEST_EFFORT, and a RELIABLE subscriber receives nothing
from it while the camera half keeps working, which made the failure look like a
broken detector. See
[M-30](../../docs/issues/archive/M-30-bag-playback-qos-mismatch-is-silent.md) and
`lctk_launch/transport.py`.

## Not for assisted mode

The board never moves within a bag, so the novelty gate would capture one pair
and then correctly refuse the rest. For assisted mode use
[`solid600-handheld-vlp`](../solid600-handheld-vlp/), where the board is carried
around.

## Two LiDARs are in the recordings

Both `/velodyne_points` (VLP-32C, frame `velodyne`) and `/iv_points` (Seyond
Falcon, frame `seyond`). This manifest uses the Velodyne; to exercise the Seyond
preset, change `pointcloud_topic`, `frame_id` and the marker's `detector_config`
to `config/board/hollow_1000/seyond.json5`.

## RViz

`rviz.rviz` is the old `lidar_camera.rviz` — deleted from
`ros/lctk_launch/config/rviz/` in `7769cd7`, which moved it into
`sessions/seyond-left/`. This copy is retargeted from that rig's device names
(`seyond_lidar`/`left_camera`) to this one's (`vlp32_lidar`/`zed_camera`) and
refixed to the `velodyne` frame.
