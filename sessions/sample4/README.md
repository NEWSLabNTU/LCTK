# sample4

Formerly dataset 4 of `lctk_sample_data`: a VLP-32C `lidar.pcap` and a camera
`video.avi`, shipped in git.

**The one thing actually known about this recording** is that its `lidar.pcap` is
the *second* LiDAR of a two-LiDAR capture. `ros/lctk_sample_data/README.md` calls
dataset 4 the "secondary LiDAR dataset for two-LiDAR calibration",
`two_lidar.launch.xml` pairs it with dataset 3 as `/sensing/lidar/front/...`, and
[M-26](../../docs/issues/archive/M-26-two-lidar-example-topics-unreachable.md)
records the same pairing. Both were captured on UDP port 2368 (see the `lidar2_port`
comment in `two_lidar.launch.xml`, which is where M-16 was found).

That says only which *role* the point cloud played. It says nothing about the
board, the crop box, or the camera — and this session, being a LiDAR-camera
session, uses the `video.avi` that the two-LiDAR path never touched.

## Verified 2026-09-01

This session **runs and detects the board.** It was launched headless with
`ros2 launch lctk_launch session.launch.py session:=sessions/sample4` and produced
a non-empty `calibration_board_detections` array with **zero detector rejections**.

That settles the two things this file previously listed as assumptions: the board
really is the hollow 1000 mm target, and the rig geometry matches
`sample3-hollow-velodyne` closely enough for its crop box to work here.

**What the first run got wrong, and why it is worth recording.** This session
originally shipped with the *bbox-free* preset and no crop box, on the reasoning
that borrowing another recording's box is what silenced the shipped demo in M-29.
Run that way it detected nothing, and the detector said exactly why:

```
no board selected — no candidate clusters survived foreground extraction;
candidates=0, foreground_pts=0
```

Background subtraction was absorbing a board that barely moves in these
recordings. Refusing to guess was still right; what was wrong was assuming the
bbox-free preset would work without checking. Switching to
`hollow_1000/velodyne_bbox.json5` with a crop box made it detect on the first try.

## What is still not known

- The **extrinsic has not been validated**. Detections flow and the solver runs;
  nobody has checked the result against a measurement of the physical rig.
- The crop box and camera intrinsics are **copies of sample3's**, now living here
  so the session is self-contained. They work, which is evidence they are close
  to right, not proof they are exact for this recording.
- The 100 ms sync window is inherited from sample3 and has not been examined
  against how fast the board moves here.

## Session-local files

- `data/` — the recording: `lidar.pcap` and `video.avi`. It moved here from
  `ros/lctk_sample_data/data/4` so the session is self-contained.

