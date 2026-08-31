# LCTK Sample Data Package

This package holds the **playback launch files** that turn a recorded pcap and
avi into ROS 2 topics. It no longer holds any recordings.

## Where the data went

The five pcap/avi datasets used to live here as `data/1` … `data/5`, reached with
`$(find-pkg-share lctk_sample_data)/data/<N>`. Each one now lives inside the
calibration session that describes it, so a session directory is self-contained
on disk and can be copied elsewhere whole:

| was | is now |
|---|---|
| `data/1` | `sessions/sample1/data` |
| `data/2` | `sessions/sample2/data` |
| `data/3` | `sessions/sample3-hollow-velodyne/data` |
| `data/4` | `sessions/sample4/data` |
| `data/5` | `sessions/sample5/data` |

`sessions/` is installed into `share/lctk_launch/sessions/`, so after a build the
recordings are reachable as
`$(find-pkg-share lctk_launch)/sessions/<name>/data/`. Only
`sample3-hollow-velodyne` has ever been run end to end; read the other four
sessions' READMEs before trusting anything in their manifests.

The gitignored two-LiDAR rosbags are unaffected and still live in `bags/` — see
[bags/README.md](bags/README.md).

## What this package still contains

- `launch/lidar_camera.launch.xml` — a Velodyne driver plus gscam, publishing one
  LiDAR and one camera from a pcap/avi pair.
- `launch/two_lidar.launch.xml` — two Velodyne drivers from two pcaps.

Both take the file paths as arguments. Their defaults name the sample3 (and, for
the second LiDAR, sample4) session copies, purely so a bare invocation still plays
what it always played. **`session_data.launch.py` never relies on those
defaults**: it derives `pcap_file`, `video_file`, the topics and the frame ids
from the session manifest and passes them explicitly, which is what keeps the
playback and the calibration graph from drifting apart.

## Usage

Prefer running a session, which starts the playback and the calibration graph
from one manifest:

```bash
just run sample3-hollow-velodyne          # or: just demo
just run sessions/sample1                 # a path works too
```

Playback only, from the session manifest:

```bash
ros2 launch lctk_launch session_data.launch.py \
  session:=$(ros2 pkg prefix lctk_launch --share)/sessions/sample3-hollow-velodyne
```

The launch files below are the layer underneath, and can still be driven
directly:

```bash
# Defaults (the sample3 recording)
ros2 launch lctk_sample_data lidar_camera.launch.xml

# Any other files
ros2 launch lctk_sample_data lidar_camera.launch.xml \
  pcap_file:=/path/to/your.pcap \
  video_file:=/path/to/your.avi \
  loop:=true

# Two LiDARs (defaults: the sample3 and sample4 recordings)
ros2 launch lctk_sample_data two_lidar.launch.xml \
  lidar1_pcap:=/path/to/lidar1.pcap \
  lidar2_pcap:=/path/to/lidar2.pcap
```

## Topics Published

### LiDAR-Camera Sample Data
- `/sensing/lidar/top/pointcloud_raw`: LiDAR point cloud
- `/sensing/lidar/top/velodyne_packets`: raw Velodyne packets
- `/sensing/camera/front_center/image_raw`: Camera image
- `/sensing/camera/front_center/camera_info`: Camera calibration info

Under a `pcap_avi` session these names are derived from the device names in the
manifest, so they follow the session rather than being fixed here.

### Two-LiDAR Sample Data
- `/sensing/lidar/top/pointcloud_raw`: First LiDAR point cloud
- `/sensing/lidar/front/pointcloud_raw`: Second LiDAR point cloud

## Parameters

### Common Parameters
- `loop`: Loop playback when reaching end of files (default: false)
- `read_fast`: Read PCAP as fast as possible or preserve timing (default: false)

### LiDAR Parameters
- `rpm`: Device rotation rate in RPM (default: 600.0)
- `model`: Velodyne model (default: 32C)

### Camera Parameters
- `camera_info_url`: Path to camera calibration file (defaults to the sample3
  session's `camera_info.yaml`)
- `camera_frame_id`: Frame ID for image messages
