# sample5

Formerly dataset 5 of `lctk_sample_data`: a VLP-32C `lidar.pcap` and a camera
`video.avi`, shipped in git and never used for anything.

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
  `ros/lctk_sample_data/data/5` so the session is self-contained.

## Verification

**Pending.** Nobody has run this session. Replace this section with what a real
run produced: detection counts, detector rejections, solved extrinsic, and
whether the target and detector preset above turned out to be right.
