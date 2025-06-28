# Architecture

This document outlines the architecture of the LCTK project.

## Overview

The project is structured as a collection of Rust libraries and binaries, with a strong emphasis on ROS 2 for communication and workflow orchestration. The architecture can be broadly divided into three layers:

1.  **Core Libraries (`src/lib`)**: These are the fundamental building blocks of the system, providing reusable functionalities for various tasks related to LiDAR and camera calibration.
2.  **ROS 2 Nodes (`src/bin`)**: These are executable applications that use the core libraries to perform specific tasks. They communicate with each other using ROS 2 topics and services.
3.  **Launch Files (`src/bin/calib_launch/launch`)**: These files define the overall workflow by launching and configuring the ROS 2 nodes in the correct sequence.

## Core Libraries (`src/lib`)

The core libraries are located in the `src/lib` directory. They are organized by functionality:

*   **ArUco Marker Handling**:
    *   `aruco-config`: Defines the data structures for ArUco marker patterns.
    *   `aruco-detector`: Detects ArUco markers in images.
    *   `aruco-generator`: Generates ArUco marker board images.
*   **Calibration Board Handling**:
    *   `hollow-board-config`: Defines the data structures for the hollow calibration board.
    *   `hollow-board-detector`: Detects the hollow calibration board in point clouds.
    *   `board-fitter-config`: Advanced board shape configurations (rectangles, circles, polygons).
    *   `board-fitter`: Advanced board detection using small_gicp with SVD-based ICP refinement.
*   **Point Cloud Processing**:
    *   `plane-estimator`: Fits planes to point cloud data.
    *   `small_gicp_rust`: A Rust wrapper for the small_gicp library for point cloud registration.
*   **Calibration**:
    *   `pnp-solver`: Solves the Perspective-n-Point (PnP) problem to determine object pose.
*   **Utilities**:
    *   `multi-stream-synchronizer`: Synchronizes data from multiple sensor streams.

## ROS 2 Nodes (`src/bin`)

The ROS 2 nodes are located in the `src/bin` directory. Each node is a separate Rust crate that compiles to an executable. The key nodes are:

*   `aruco_locator_node`: Detects ArUco markers in a camera image stream.
*   `calibration_board_locator`: Detects the calibration board in a point cloud stream.
*   `extrinsic_solver`: Solves for the extrinsic calibration between the LiDAR and camera.
*   `synchronizer`: Synchronizes the outputs of the `aruco_locator_node` and `calibration_board_locator`.
*   `pointcloud_image_overlay`: Overlays the point cloud onto the camera image for visualization.
*   `multi_wayside_node`: Handles multi-LiDAR calibration scenarios with real-time processing, automatic detection synchronization, and TF broadcasting.
*   `rosbag_deck`: A tool for working with ROS 2 bag files with advanced playback features.

## Communication

The ROS 2 nodes communicate with each other using a set of defined topics. The primary topics are:

### LiDAR-Camera Calibration Pipeline
*   `/camera/image_raw`: The input camera image stream.
*   `/lidar/pointcloud`: The input LiDAR point cloud stream.
*   `/calibration/aruco_locator/aruco_detections`: The output of the ArUco marker detector.
*   `/calibration/calibration_board_locator/board_detections`: The output of the calibration board detector.
*   `/calibration/synchronizer/synchronized_detections`: The synchronized detections from the `synchronizer` node.
*   `/calibration/extrinsic_solver/extrinsic_transform`: The final extrinsic calibration transform.

### Multi-LiDAR Calibration (multi_wayside_node)
*   `/lidar1/points`, `/lidar2/points`: Input point cloud streams from multiple LiDARs.
*   `/lidar1/board_detection`, `/lidar2/board_detection`: Board detection results from each LiDAR.
*   `/lidar1/board_pose_adjustment`, `/lidar2/board_pose_adjustment`: Manual pose adjustment inputs.
*   `/calibration_transform`: Real-time LiDAR-to-LiDAR calibration transform.
*   `/calibration_markers`: RViz visualization markers for detected boards and calibration status.

### Advanced Features
*   **ROS Services**: ROI configuration, calibration triggering, and system control.
*   **TF2 Integration**: Automatic transform broadcasting for coordinate frame management.
*   **Parameter Server**: Dynamic reconfiguration of detection thresholds and calibration parameters.

This modular, message-based architecture allows for flexibility and easy replacement of individual components.
