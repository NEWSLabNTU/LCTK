# LCTK Sample Data Package

This package contains sample data and launch files for demonstrating LCTK calibration capabilities.

## Contents

### Sample Data
- `data/3/`: Primary dataset for LiDAR-camera calibration
  - `lidar.pcap`: Velodyne VLP-32C point cloud data
  - `video.avi`: Camera video synchronized with LiDAR data
- `data/4/`: Secondary LiDAR dataset for two-LiDAR calibration
  - `lidar.pcap`: Second Velodyne VLP-32C point cloud data
- `data/1-5/`: Complete collection of sample datasets

### Launch Files
- `lidar_camera.launch.xml`: Publishes synchronized LiDAR and camera data for LiDAR-camera calibration
- `two_lidar.launch.xml`: Publishes two LiDAR data streams for two-LiDAR calibration

## Usage

### LiDAR-Camera Sample Data
```bash
# Using default sample data (dataset 3)
ros2 launch lctk_sample_data lidar_camera.launch.xml

# Using custom data files
ros2 launch lctk_sample_data lidar_camera.launch.xml \
  pcap_file:=/path/to/your.pcap \
  video_file:=/path/to/your.avi \
  loop:=true
```

### Two-LiDAR Sample Data
```bash
# Using default sample data (datasets 3 and 4)
ros2 launch lctk_sample_data two_lidar.launch.xml

# Using custom PCAP files
ros2 launch lctk_sample_data two_lidar.launch.xml \
  lidar1_pcap:=/path/to/lidar1.pcap \
  lidar2_pcap:=/path/to/lidar2.pcap \
  loop:=true
```

## Topics Published

### LiDAR-Camera Sample Data
- `/sensing/lidar/top/pointcloud_raw`: LiDAR point cloud
- `/sensing/lidar/top/synchronized_pointcloud`: Synchronized LiDAR point cloud
- `/sensing/camera/front_center/image_raw`: Camera image
- `/sensing/camera/front_center/synchronized_image`: Synchronized camera image
- `/sensing/camera/front_center/camera_info`: Camera calibration info

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
- `camera_info_url`: Path to camera calibration file
- `camera_frame_id`: Frame ID for image messages