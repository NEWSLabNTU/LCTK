# sample3-hollow-velodyne

Dataset 3 of `lctk_sample_data`: a VLP-32C `lidar.pcap` and a camera `video.avi`
recorded together against the hollow 1000 mm board, with one board placement.

**The data ships in git**, at `ros/lctk_sample_data/data/3`, so this session runs
on a fresh clone with no extra downloads. It is what `just demo` runs.

| | |
|---|---|
| Data | `pcap_avi`, `$(find-pkg-share lctk_sample_data)/data/3` |
| LiDAR | VLP-32C at 600 rpm, device `top`, frame `velodyne_top` |
| Camera | device `front_center`, frame `camera_front_center` |
| Target | hollow 1000 mm, four ArUco markers |
| Detector | `hollow_1000/velodyne_bbox.json5` — the one shipped preset still in bbox mode |

The device is named `top` (not `top_lidar`) on purpose: under `pcap_avi` the topics
are derived from the device name, and `top` reproduces the playback's long-standing
`/sensing/lidar/top/pointcloud_raw` and `/sensing/camera/front_center/image_raw`
exactly.

## Session-local files

- `bbox.json5` — the crop box for **this** recording. It was split out from the
  shared `config/board/bbox.json5` because that file had been retuned for a Seyond
  rosbag, which moved the box somewhere this recording's board never is; the
  detector then reported "only 0 finite points in the configured box" on every
  frame and published nothing (M-29). Verified 2026-08-31: with this box the
  detector finds ~2080 correspondences per frame.
- `camera_info.yaml` — the intrinsics for the avi. Also the default
  `camera_info_url` of `lctk_sample_data`'s `lidar_camera.launch.xml`.
