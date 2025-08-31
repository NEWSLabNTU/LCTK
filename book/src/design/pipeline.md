# Calibration Pipeline

LCTK supports multiple calibration pipelines for different sensor configurations. Each pipeline is designed to be modular, allowing components to be easily replaced or extended.

## Supported Pipelines

1. **LiDAR-Camera Calibration**: Calibrating a single LiDAR with a single camera
2. **Two-LiDAR Calibration**: Calibrating two LiDARs relative to each other
3. **Multi-Sensor Calibration**: Extended configurations with multiple sensors

## Pipeline Architecture

Each calibration pipeline follows a similar pattern:

1. **Data Acquisition**: Sensor data is captured or played back from recordings
2. **Detection**: Calibration targets are detected in each sensor's data
3. **Synchronization**: Detections are temporally aligned
4. **Optimization**: Calibration parameters are computed
5. **Validation**: Results are verified through visualization

## Common Components

### Detection Stage
- ArUco marker detection for cameras
- Calibration board detection for LiDARs
- Feature extraction and correspondence matching

### Synchronization Stage
- Timestamp-based alignment
- Configurable tolerance windows
- Buffering for real-time processing

### Optimization Stage
- PnP solving for camera-based calibration
- ICP refinement for point cloud registration
- Bundle adjustment for multi-sensor scenarios

### Visualization Stage
- Real-time monitoring in RViz
- Overlay visualization for verification
- Error metrics and statistics

## Configuration

Pipelines are configured through:
- Launch file parameters
- JSON5 configuration files
- ROS 2 parameter server
- Dynamic reconfiguration at runtime