# solid600-handheld-vlp

Solid 600 mm target, hand-held, walked around an underground car park. VLP-32C
plus a ZED.

**This is the assisted-mode session.** The board moves, stops, and moves again,
which is what the stillness and novelty gates are built for. Every other
recording in this repo holds the board still for its whole length, so the
novelty gate captures one placement and then correctly refuses everything after.

## The recording does not ship

It is a field capture, gitignored like the `TWO_LIDAR_*` bags. Symlink one in:

```bash
ln -sfn /path/to/new_LCTK_board/newtype_1 sessions/solid600-handheld-vlp/bag
```

`newtype_1` is 58 s; `newtype_2` is a second take. A separate
`newtype_background` bag exists but is **not needed** — the detector builds its
background model from the opening frames of `newtype_1` itself, before the
operator carries the board into the scene.

Without a bag, `lctk_session check` refuses the session rather than launching a
graph that would sit silent: a `kind: bag` manifest is verified against the
recording's `metadata.yaml` at parse time (M-26).

## Running it

```bash
just mode=realtime assisted solid600-handheld-vlp
```

Then open <http://localhost:8080>. *Export archive* writes `out/detections.json`.

**`mode=realtime` is required, and this is not a typo.** `mode` selects transport
QoS: `offline` is RELIABLE, `realtime` is BEST_EFFORT. The recording replays with
the QoS its publishers used, which for this rig's LiDAR driver is BEST_EFFORT, so
a RELIABLE subscriber is simply incompatible and receives nothing:

```
[rosbag2_player] New subscription discovered on topic '/velodyne_points',
requesting incompatible QoS. No messages will be sent to it.
```

The camera half keeps working, so the failure looks like a broken LiDAR detector
rather than a QoS mismatch. See
[M-30](../../docs/issues/M-30-bag-playback-qos-mismatch-is-silent.md).

## The ZED records compressed images only

Nothing in this tree subscribes to `sensor_msgs/CompressedImage` —
`aruco_locator_node` takes `sensor_msgs/Image`. The manifest's `data.republish:`
section puts the `image_transport` bridge in the launched graph, so there is no
second terminal to remember.

## Live sensors

This manifest was a `kind: live` session before the recording existed. To drive
the rig live again, replace the `data:` block with `kind: live` and drop the
`republish:` entry if the camera publishes raw.
