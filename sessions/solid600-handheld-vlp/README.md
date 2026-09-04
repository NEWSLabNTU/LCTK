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
just solver_mode=assisted run solid600-handheld-vlp
```

Then open <http://localhost:8080>. *Export archive* writes `out/detections.json`.

The reliability each sensor topic is subscribed with is resolved per device from
the recording itself (`offered_qos_profiles` in the bag's `metadata.yaml`), so
nothing has to be remembered on the command line. This session used to require
`mode=realtime` because the graph-wide flag could not say what the bag already
knew: its LiDAR replays BEST_EFFORT, and a RELIABLE subscriber receives nothing
from it while the camera half keeps working, which made the failure look like a
broken detector. See
[M-30](../../docs/issues/archive/M-30-bag-playback-qos-mismatch-is-silent.md) and
`lctk_launch/transport.py`.

## The ZED records compressed images only

Nothing in this tree subscribes to `sensor_msgs/CompressedImage` —
`aruco_locator_node` takes `sensor_msgs/Image`. The manifest's `data.republish:`
section puts the `image_transport` bridge in the launched graph, so there is no
second terminal to remember.

## Live sensors

This manifest was a `kind: live` session before the recording existed. To drive
the rig live again, replace the `data:` block with `kind: live` and drop the
`republish:` entry if the camera publishes raw.
