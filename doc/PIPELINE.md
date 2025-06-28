# Calibration Pipeline Refactoring Plan

This document outlines the refactoring plan for the LCTK calibration launch files to support two distinct calibration pipelines:

1.  **LiDAR-Camera Calibration:** Calibrating a single LiDAR with a single camera.
2.  **Two-LiDAR Calibration:** Calibrating two LiDARs using the `multi_wayside_node`.

## Current State

The existing `src/bin/calib_launch/launch/calibration_pipeline.launch.yaml` file currently implements the LiDAR-Camera calibration pipeline.

## Proposed Structure

To improve modularity and clarity, the launch files will be reorganized as follows:

-   `src/bin/calib_launch/launch/lidar_camera_calibration.launch.yaml`: This file will contain the nodes and configurations specific to the LiDAR-Camera calibration pipeline. This will be a renamed version of the current `calibration_pipeline.launch.yaml`.
-   `src/bin/calib_launch/launch/two_lidar_calibration.launch.yaml`: This new file will contain the nodes and configurations for the two-LiDAR calibration pipeline.

Users will directly select the appropriate launch file based on their calibration needs.

## Pipeline Details

### 1. LiDAR-Camera Calibration (`lidar_camera_calibration.launch.yaml`)

This pipeline will remain largely similar to the existing `calibration_pipeline.launch.yaml`.

**Nodes and Data Flow:**

-   **`aruco_locator_node`**:
    -   **Input:**
        -   `/camera/image_raw` (sensor_msgs/Image)
        -   `/camera/camera_info` (sensor_msgs/CameraInfo)
    -   **Output:** `/calibration/aruco_locator/aruco_detections` (vision_msgs/Detection2DArray)
-   **`calibration_board_locator`**:
    -   **Input:** `/lidar/pointcloud` (sensor_msgs/PointCloud2)
    -   **Output:** `/calibration/calibration_board_locator/board_detections` (vision_msgs/Detection3DArray)
-   **`synchronizer`**:
    -   **Input:**
        -   `/calibration/aruco_locator/aruco_detections` (vision_msgs/Detection2DArray)
        -   `/calibration/calibration_board_locator/board_detections` (vision_msgs/Detection3DArray)
    -   **Output:**
        -   `/calibration/synchronizer/synchronized_detections` (custom message type, containing both 2D and 3D detections)
        -   `/calibration/synchronizer/synchronized_pointcloud` (sensor_msgs/PointCloud2)
        -   `/calibration/synchronizer/synchronized_image` (sensor_msgs/Image)
-   **`extrinsic_solver`**:
    -   **Input:**
        -   `/calibration/synchronizer/synchronized_detections` (custom message type)
        -   `/camera/camera_info` (sensor_msgs/CameraInfo)
    -   **Output:** `/calibration/extrinsic_solver/extrinsic_transform` (geometry_msgs/TransformStamped)
-   **`pointcloud_image_overlay`** (Optional visualization node):
    -   **Input:**
        -   `/calibration/synchronizer/synchronized_pointcloud` (sensor_msgs/PointCloud2)
        -   `/calibration/synchronizer/synchronized_image` (sensor_msgs/Image)
        -   `/camera/camera_info` (sensor_msgs/CameraInfo)
        -   `/calibration/extrinsic_solver/extrinsic_transform` (geometry_msgs/TransformStamped)
    -   **Output:** (Visualization in Rviz/Rerun, no direct ROS topic output for other nodes)

**Launch Arguments:**

-   `aruco_config_file`: Path to ArUco pattern configuration JSON5 file.
-   `board_config_file`: Path to calibration board configuration JSON5 file.
-   `camera_topic`: Input camera image topic.
-   `camera_info_topic`: Input camera info topic.
-   `pointcloud_topic`: Input point cloud topic.
-   `debug_mode`: Enable debug logging and visualization.
-   `sync_window_ms`: Synchronization window size in milliseconds.
-   `max_distance`: Maximum distance for point cloud filtering (meters).
-   `min_distance`: Minimum distance for point cloud filtering (meters).

**Example Usage:**
```bash
ros2 launch calib_launch lidar_camera_calibration.launch.yaml
```

### 2. Two-LiDAR Calibration (`two_lidar_calibration.launch.yaml`)

This new pipeline will focus on calibrating two LiDARs.

**Nodes and Data Flow:**

-   **`calibration_board_locator_1`**:
    -   **Input:** `/lidar1/pointcloud` (sensor_msgs/PointCloud2) - Point cloud data from the first LiDAR.
    -   **Output:** `/lidar1/board_detections` (vision_msgs/Detection3DArray) - Detections of calibration boards in the first LiDAR's point cloud.
-   **`calibration_board_locator_2`**:
    -   **Input:** `/lidar2/pointcloud` (sensor_msgs/PointCloud2) - Point cloud data from the second LiDAR.
    -   **Output:** `/lidar2/board_detections` (vision_msgs/Detection3DArray) - Detections of calibration boards in the second LiDAR's point cloud.
-   **`multi_wayside_node`**:
    -   **Input:**
        -   `/lidar1/board_detections` (vision_msgs/Detection3DArray) - Board detections from the first LiDAR.
        -   `/lidar2/board_detections` (vision_msgs/Detection3DArray) - Board detections from the second LiDAR.
    -   **Output:** `/lidar1_to_lidar2_transform` (geometry_msgs/TransformStamped) - The computed extrinsic transform from LiDAR 1 to LiDAR 2.

**Launch Arguments (Proposed):**

-   `lidar1_pointcloud_topic`: Input point cloud topic for the first LiDAR.
-   `lidar2_pointcloud_topic`: Input point cloud topic for the second LiDAR.
-   `board_config_file`: Path to a configuration file for `calibration_board_locator` (if required).
-   `debug_mode`: Enable debug logging.

**Example Usage:**
```bash
ros2 launch calib_launch two_lidar_calibration.launch.yaml
```

## Implementation Steps

1.  Rename `src/bin/calib_launch/launch/calibration_pipeline.launch.yaml` to `src/bin/calib_launch/launch/lidar_camera_calibration.launch.yaml`.
2.  Create `src/bin/calib_launch/launch/two_lidar_calibration.launch.yaml` with the necessary nodes and arguments for two-LiDAR calibration.
3.  Update `CMakeLists.txt` in `src/bin/calib_launch/` to ensure all new launch files are installed correctly.

## Progress Table

| Task                                                              | Status    | Notes                                      |
| :---------------------------------------------------------------- | :-------- | :----------------------------------------- |
| Rename `calibration_pipeline.launch.yaml`                         | To Do     |                                            |
| Create `lidar_camera_calibration.launch.yaml`                     | To Do     | Content will be from renamed file          |
| Create `two_lidar_calibration.launch.yaml`                        | To Do     | New file with two-LiDAR pipeline           |
| Update `CMakeLists.txt` for `calib_launch`                        | To Do     | Ensure new launch files are installed      |

## Phase-by-Phase Action Items

### Phase 1: File Renaming and Initial Setup

-   **Action:** Rename `src/bin/calib_launch/launch/calibration_pipeline.launch.yaml` to `src/bin/calib_launch/launch/lidar_camera_calibration.launch.yaml`.
-   **Verification:** Confirm the file has been renamed and its content is intact.

### Phase 2: Two-LiDAR Calibration Launch File Creation

-   **Action:** Create `src/bin/calib_launch/launch/two_lidar_calibration.launch.yaml`.
-   **Details:**
    -   Include `calibration_board_locator` nodes for both LiDARs.
    -   Include the `multi_wayside_node`.
    -   Define the necessary launch arguments (`lidar1_pointcloud_topic`, `lidar2_pointcloud_topic`, `board_config_file`, `debug_mode`).
    -   Map the input/output topics as described in the "Nodes and Data Flow" section.
-   **Verification:** Ensure the YAML syntax is correct and all required nodes and arguments are present.

### Phase 3: CMakeLists.txt Update

-   **Action:** Modify `src/bin/calib_launch/CMakeLists.txt`.
-   **Details:**
    -   Update the `install(DIRECTORY ...)` command to include `lidar_camera_calibration.launch.yaml` and `two_lidar_calibration.launch.yaml`.
-   **Verification:** Build the `calib_launch` package and verify that the new launch files are installed in the ROS 2 install space.

### Phase 4: Testing and Validation

-   **Action:** Run both calibration pipelines using `ros2 launch` commands.
-   **Details:**
    -   Test `ros2 launch calib_launch lidar_camera_calibration.launch.yaml` with appropriate data.
    -   Test `ros2 launch calib_launch two_lidar_calibration.launch.yaml` with appropriate data.
-   **Verification:** Observe the node outputs and ensure the pipelines are functioning as expected. Confirm that transforms are being published for both pipelines.