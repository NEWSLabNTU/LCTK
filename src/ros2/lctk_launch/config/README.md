# LCTK Configuration Files

This directory contains centralized configuration files for all LCTK calibration packages.

## Directory Structure

```
config/
├── aruco/          # ArUco marker configurations
├── board/          # Calibration board configurations
├── camera/         # Camera intrinsics and settings
├── lidar/          # LiDAR sensor configurations
└── calibration/    # Calibration results and parameters
```

## Configuration Files

### ArUco Configurations (`aruco/`)
- `aruco_pattern.json5` - ArUco marker pattern definitions for detection

### Board Configurations (`board/`)
- `board_pattern.json5` - Physical calibration board specifications
- `board_detector.json5` - Board detection algorithm parameters

### Camera Configurations (`camera/`)
- `front_center_camera_info.yaml` - Camera intrinsic parameters (focal length, distortion, etc.)

### LiDAR Configurations (`lidar/`)
- LiDAR sensor specific configuration files (if any)

### Calibration Results (`calibration/`)
- Stores extrinsic calibration results and transformation matrices

## Usage in Launch Files

Configuration files are referenced in launch files using the ROS 2 package share directory:

```xml
<arg name="aruco_config_file"
     default="$(find-pkg-share calib_launch)/config/aruco/aruco_pattern.json5"/>

<arg name="board_config_file"
     default="$(find-pkg-share calib_launch)/config/board/board_pattern.json5"/>
```

Or accessed via command line:
```bash
ros2 pkg prefix calib_launch
# Returns: /path/to/install/calib_launch
# Config files at: /path/to/install/calib_launch/share/calib_launch/config/
```

## File Formats

- **JSON5** (`.json5`): Used for complex structured configurations with comments
- **YAML** (`.yaml`/`.yml`): Used for ROS-compatible camera parameters

## Adding New Configurations

1. Place configuration files in the appropriate subdirectory
2. Update CMakeLists.txt if adding new file extensions
3. Reference the file in launch files using `$(find-pkg-share calib_launch)/config/<subdir>/<file>`

## Notes

- All paths in launch files should use the installed share directory location
- Configuration files are installed during the build process (`make build`)
- Changes to config files require rebuilding to update the installed versions