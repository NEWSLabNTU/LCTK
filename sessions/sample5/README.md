# sample5

Formerly dataset 5 of `lctk_sample_data`: a VLP-32C `lidar.pcap` and a camera
`video.avi`, shipped in git and now a self-contained session.

## Verified 2026-09-01

This session **runs and detects the board.** It was launched headless with
`ros2 launch lctk_launch session.launch.py session:=sessions/sample5` and produced
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
  `ros/lctk_sample_data/data/5` so the session is self-contained.

