# M-27 · `solid_600_handheld.yaml`'s placeholder topics alias the hollow-board sample-data playback

- **Severity:** Medium
- **Area:** lctk_launch / sessions (was `config/examples`)
- **Status:** Fixed (2026-09-01)
- **Verified:** By code trace (2026-08-28) against `lctk_sample_data/launch/lidar_camera.launch.xml`
- **Related:** [L-19 (archived)](./L-19-aruco-config-required-but-unused-for-lidar.md)

## Problem

`ros/lctk_launch/config/examples/solid_600_handheld.yaml` documents its topics as placeholders:

```yaml
devices:
  lidars:
    top_lidar:
      pointcloud_topic: /sensing/lidar/top/pointcloud_raw
      frame_id: velodyne_top
  cameras:
    front_center:
      image_topic: /sensing/camera/front_center/image_raw
      frame_id: camera_front_center
```

The file's own header comment says: *"the topics below are placeholders matching the sample-data
naming convention, so a bag can arrive later without a config change."* That's true, but it's also
exactly the problem: `ros/lctk_sample_data/launch/lidar_camera.launch.xml` — the launch file behind
`just sample-data` — publishes those **exact** topics by default:

- `pointcloud_topic` default: `/sensing/lidar/top/pointcloud_raw`
- `camera_namespace` default: `/sensing/camera/front_center` → `image_raw` under it resolves to
  `/sensing/camera/front_center/image_raw`

`lidar_camera.launch.xml` plays back `data/3/` — the **hollow 1000 mm** board (dataset 3 is
CLAUDE.md's stated lidar-camera default). `solid_600_handheld.yaml` selects the **solid 600 mm**
Target Definition (`config/targets/solid_600_aruco_1_v1.json5`) and its EXPERIMENTAL detector
tuning (`config/board/solid_600/velodyne.json5`).

So `just sample-data` run alongside `solid_600_handheld.yaml` connects a solid-600 Target
Definition to a recording of the hollow-1000 board, with no config change required to make that
happen — the aliasing is silent and automatic. The header comment frames the topic reuse as a
*feature* ("a bag can arrive later without a config change") without noting it's simultaneously a
footgun against the one recording that already exists in the repo and answers to the same topics
today.

## Why the identity gate does not catch this

The pipeline has a runtime target-identity gate (`TargetIdentityGate` / the
`lidar_target_identity_topic` / `camera_target_identity_topic` mechanism referenced in
`lidar_to_camera_solver`). It compares the identity the LiDAR-side detector reports against the
identity the camera-side detector reports, and rejects a pair when they disagree.

That defends against exactly one failure mode: *two detectors, each observing something real,
disagreeing about what target they're looking at.* It cannot defend against this one, because both
observers are configured — by `detector_config` — to expect the **solid 600 mm** target and report
that identity regardless of what physical board is actually in the frame. The mismatch here is
between the **config's declared target** and the **physical board in the data stream**; nothing in
the running software observes the physical board's true identity independently of what the config
told the detector to look for. Both sides would agree (wrongly) on "solid 600," or the solid-tuned
detector would simply fail to find a solid board in hollow-board data — either way, no cross-check
fires, because the gate only ever compares two configured expectations against each other, never a
configured expectation against physical ground truth.

## Suggested fix (do not implement — for the record)

Pick one:

1. Give the example unmistakably-placeholder topic names (e.g. `/PLACEHOLDER/lidar/pointcloud`,
   or a topic namespace no shipped playback uses, such as `/sensing/lidar/handheld/...`) so it
   cannot silently alias an existing recording.
2. Or namespace it explicitly, e.g. `/sensing/lidar/solid_600_handheld/pointcloud_raw`, so a
   collision requires the operator to type the same topic twice on purpose rather than getting it
   for free from following the stated naming convention.

Either way, note in the header comment that "placeholder matching the sample-data convention" is a
double-edged property until a real solid-600 recording exists to fill the aliased topics
correctly.

## Resolution (2026-09-01)

Fixed by the calibration-sessions change, and fixed at the level this issue's "why the
identity gate does not catch this" section identified: the collision was never really about
topic *names*, it was about a config that declared no data source at all and so could be
pointed at anyone's.

**A session declares where its data comes from.** `solid_600_handheld.yaml` is gone;
`sessions/solid600-handheld-zed/session.yaml` replaces it and states `data.kind: live` — no
recording ships for this rig, and the manifest now says so rather than leaving it to a header
comment. Its camera topic is the ZED's real one
(`/sensing/camera/zed/rgb/color/rect/image`), which no shipped playback publishes.

**The aliasing is structurally gone, not merely renamed.** The sample-data recording is now
its own session, `sample3-hollow-velodyne`, with `data.kind: pcap_avi` — and under `pcap_avi`
topics are **derived** from the device names, never stated. Stating one is refused at parse
time. So the hollow-board playback's topics are a consequence of that session's own device
names, and a second session cannot inherit them by following a naming convention. Running
either session starts its own data source; there is no longer a "run this config alongside
that playback" step in which the two could be mismatched.

That is why suggested fix 1 and 2 — both of which were about choosing less collidable
placeholder names — were not taken. Renaming a placeholder makes the collision less likely;
making each session own its data source makes the pairing explicit, so there is no free
placeholder to collide.

**What is still true.** As the "why the identity gate does not catch this" section explains,
nothing in the running software observes the physical board's true identity independently of
what the config told the detector to look for, so pointing a solid-600 session at
hollow-board data by hand remains possible. What has changed is that it now takes a
deliberate edit to the manifest's `data:` section instead of happening for free.
`ros2 run lctk_launch lctk_session check <session>` prints the data source and every device
topic before the run, which is where such an edit becomes visible.
