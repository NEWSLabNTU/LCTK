# solid600-handheld-seyond

Solid 600 mm target, hand-held, walked around an underground car park. **Seyond
Falcon** plus a ZED.

This is [`solid600-handheld-zed`](../solid600-handheld-zed/) read through the
other LiDAR. Every `newtype_*` bag records both sensors — `/velodyne_points` and
`/iv_points` — so the two sessions see the same scene, the same board and the
same operator, and differ only in which sensor the detector reads.

## Why a second session rather than a device override

A VLP-32C at 7–8 m crosses a 600 mm plate with roughly four rings: about 2.8 cm
between points within a ring, but ~15 cm between rings. That is the measurement
behind [H-17](../../docs/issues/H-17-solid-600-preset-detects-nothing.md), and it
caps how many frames yield a usable board cluster. Assisted mode selects still,
geometrically-novel placements out of that stream, so a thin stream gives it
little to choose from.

The Falcon samples the same plate far more densely, which is why its preset can
carry `icp_min_inlier_points: 300` where the Velodyne's carries `100`. More
surviving frames means more candidate placements reach the review page.

## The recording does not ship

Field capture, gitignored like the `TWO_LIDAR_*` bags. Symlink one in — either
take works, both contain both sensors:

```bash
ln -sfn /path/to/new_LCTK_board/newtype_1 sessions/solid600-handheld-seyond/bag
```

`newtype_1` is 58 s (577 `/iv_points` messages); `newtype_2` is a second take.

## Running it

```bash
just mode=realtime assisted solid600-handheld-seyond
```

Then open <http://localhost:8080>. *Export archive* writes `out/detections.json`.

`mode=realtime` is required for the same reason as the Velodyne session: the
recording replays with the QoS its publishers used, which is BEST_EFFORT, and a
RELIABLE subscriber receives nothing from it while the camera half keeps working
— so the failure looks like a broken LiDAR detector. See
[M-30](../../docs/issues/M-30-bag-playback-qos-mismatch-is-silent.md).

## What is not yet tuned

- The `solid_600/seyond.json5` preset is **EXPERIMENTAL** by its own header —
  sensor-specific starting values awaiting a real-bag evidence report.
- The `assisted:` stillness thresholds are inherited verbatim from the Velodyne
  session and were fitted to *its* pose noise. See the comment in
  `session.yaml`; they want re-measuring against the Falcon's pose stream.
