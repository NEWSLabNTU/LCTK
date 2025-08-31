# ROS 2 Nodes

The ROS 2 nodes are located in the `src/bin` directory. Each node is a separate Rust crate that compiles to an executable.

## Detection Nodes

### aruco_locator_node
Detects ArUco markers in camera image streams.
- **Input**: Camera images (`sensor_msgs/Image`)
- **Output**: 2D detections (`vision_msgs/Detection2DArray`)
- **Features**: Real-time detection, configurable marker dictionaries, camera calibration support

### calibration_board_locator
Detects calibration boards in point cloud streams.
- **Input**: Point clouds (`sensor_msgs/PointCloud2`)
- **Output**: 3D detections (`vision_msgs/Detection3DArray`)
- **Features**: Plane fitting, geometric pattern matching, noise filtering

## Calibration Nodes

### extrinsic_solver
Solves for extrinsic calibration between LiDAR and camera sensors.
- **Input**: 2D and 3D detections
- **Output**: Transform (`geometry_msgs/TransformStamped`)
- **Features**: Multiple PnP algorithms, iterative refinement, uncertainty estimation

### multi_wayside_node
Handles multi-LiDAR calibration scenarios.
- **Features**: Real-time processing, automatic detection synchronization, TF broadcasting
- **Use Case**: Calibrating multiple LiDAR sensors in wayside installations

## Utility Nodes

### synchronizer
Synchronizes outputs from multiple detection nodes.
- **Purpose**: Ensures temporal alignment of ArUco and board detections
- **Method**: Timestamp-based synchronization with configurable tolerance

### pointcloud_image_overlay
Overlays point clouds onto camera images for visualization.
- **Purpose**: Visual verification of calibration accuracy
- **Output**: Colored point clouds projected onto image plane

### aruco_detection_overlay
Visualizes ArUco detections on camera images.
- **Purpose**: Real-time monitoring of marker detection
- **Output**: Annotated images with bounding boxes and marker IDs

## Data Management

### rosbag_deck
Advanced tool for working with ROS 2 bag files.
- **Features**: 
  - Multi-bag playback synchronization
  - Rate control and looping
  - Topic filtering and remapping
  - Frame extraction for offline processing