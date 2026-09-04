# solid600-handheld-seyond

Solid 600 mm target, hand-held, walked around an underground car park. **Seyond
Falcon** plus a ZED.

This is [`solid600-handheld-vlp`](../solid600-handheld-vlp/) read through the
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

## Stillness tuning, and what it is measured from

`stability_max_translation_m` is `0.120` here against `solid600-handheld-vlp`'s
`0.050`. Measured 2026-09-04 on both takes: the inherited 50 mm produced 3-4
distinct placements, and 120 mm produces 6 on `newtype_1` and 9 on `newtype_2`.

**Measure it from the node, not from the detection topic.** The stillness gate
runs on *synchronized pairs*, so a board detection whose ArUco partner missed
the 50 ms sync window never reaches it. Replaying the board detection topic
through `StillnessTracker` offline feeds it a denser stream than it ever sees,
and overstates the result -- it predicted 7 placements where the node produced
3. `lidar_to_camera_solver` now logs every stillness verdict at debug level:

```bash
just mode=realtime solver_mode=assisted log_level=debug run solid600-handheld-seyond
grep "stillness:" play_log/<run>/node/lidar_to_camera_solver/err
```

Each line carries the reason, the 1 s translation and rotation spans, and how
many pairs were in the window, which is what these thresholds are set against.

## Known: the solve is DEGENERATE on both recordings

Every solve on these bags reports `DEGENERATE` with a reprojection RMS around
42 px, at 6 and at 9 placements alike. More placements did not improve it, so
this is not the diversity problem the assisted mode's gates exist to prevent --
the extra captures this tuning buys are necessary but not sufficient. The cause
is not diagnosed here. Note the solid 600 target carries a single ArUco marker,
which gives the camera side far less to constrain a pose with than the hollow
1000's four.
