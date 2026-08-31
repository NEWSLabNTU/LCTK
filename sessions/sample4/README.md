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

## What is actually known

Very little. The recording ships in git and plays back — that is the whole of it.

- **It has never been run through the calibration pipeline.** No detection, no
  solve, no extrinsic. Nobody has looked at a frame of it in this repo.
- **The board is an assumption.** `hollow_1000_aruco_4_v1.json5` is selected
  because it is what the one verified session (`sample3-hollow-velodyne`) uses,
  and these recordings were captured alongside it. Which physical target was in
  front of the sensors here is not recorded anywhere.
- **The detector preset is an assumption too.** `hollow_1000/velodyne.json5` is
  the bbox-free preset, chosen because the LiDAR is a VLP-32C. It has not been
  tuned against this data.
- **There is no verified crop box, so this session has none.** The manifest uses
  the bbox-free detector deliberately. A crop box describes where the board sat
  during one specific recording; borrowing another recording's box is what
  silenced the shipped demo in M-29, so an invented one would be worse than
  nothing.
- **There are no camera intrinsics for this session.** No `camera_info.yaml`
  ships here, and the manifest names none. The playback falls back to the
  `camera_info_url` default in `lctk_sample_data`'s `lidar_camera.launch.xml`,
  which is `sample3-hollow-velodyne`'s file — the same camera, but that is an
  inference, not a measurement.
- The rig geometry — where the LiDAR and the camera sat relative to each other —
  is unknown. The device names, frames and 100 ms sync window are all copied
  from sample3.

Treat every value in `session.yaml` as a starting point to be checked, not as a
description of the rig.

## Session-local files

- `data/` — the recording: `lidar.pcap` and `video.avi`. It moved here from
  `ros/lctk_sample_data/data/4` so the session is self-contained.

## Verification

**Pending.** Nobody has run this session. Replace this section with what a real
run produced: detection counts, detector rejections, solved extrinsic, and
whether the target and detector preset above turned out to be right.
