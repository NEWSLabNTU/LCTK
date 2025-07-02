# Rerun Visualization Example Design

## Executive Summary

This document outlines the design for a revised `rerun_visualization.rs` example for the `board-fitter` library. The goal is to create a powerful, interactive, and intuitive debugging and demonstration tool that clearly visualizes each stage of the board detection pipeline. The revision will transform the example from a simple data dump into an insightful, user-driven experience, leveraging the full capabilities of the Rerun SDK.

## Current State & Problems

The existing `rerun_visualization.rs` example successfully demonstrates a basic detection and visualization loop. However, it suffers from several limitations:

1.  **Lack of Clarity**: All visualization data (planes, squares, final detections) is logged to the same Rerun entity path, making it difficult to distinguish between different stages of the pipeline.
2.  **Limited Interactivity**: The user has no control over the visualization. They cannot toggle the visibility of intermediate steps (e.g., hide planes to see squares more clearly).
3.  **Monolithic Code**: The visualization logic is contained within a single `main` function, making it hard to maintain, extend, or reuse.
4.  **Inefficient Data Handling**: It doesn't leverage the `board-fitter` library's built-in `DebugContext`, which is designed specifically for capturing pipeline data. This leads to redundant data handling and a disconnect from the library's own debugging mechanisms.
5.  **Suboptimal Data Representation**: Information like confidence scores or detection parameters is not clearly visualized, requiring the user to inspect raw log output.

## Proposed Revision & Design Goals

The revised example will be refactored into a modular and extensible application that provides a rich, interactive view of the detection process.

### 1. Modular Code Architecture

The example will be restructured for clarity and reusability:

-   **Argument Parsing (`args.rs`)**: A dedicated module using `clap` for robust command-line argument parsing. This will allow easy configuration of the input PCD file, board configuration, and detector settings.
-   **Visualization Logic (`viewer.rs`)**: A new `Viewer` struct will encapsulate all Rerun-related logic. It will manage the Rerun `RecordingStream` and provide dedicated methods for logging different types of data (`log_point_cloud`, `log_planes`, `log_detections`, etc.).
-   **Main Logic (`main.rs`)**: The `main` function will be streamlined to handle application setup, orchestrate the detection, and delegate all visualization tasks to the `Viewer`.

### 2. Structured Rerun Entity Paths

To bring clarity to the visualization, a hierarchical entity path structure will be used:

-   `/`: Root path for global information and controls.
-   `/input/point_cloud`: The original, unprocessed point cloud.
-   `/input/roi`: The configured Region of Interest (ROI).
-   `/processing/planes`: Visualization of all detected planar surfaces.
-   `/processing/diamonds`: Visualization of fitted diamond squares on the planes.
-   `/processing/holes`: Detected holes within the diamond squares.
-   `/detections/final`: The final, validated board detections.
-   `/detections/final/{id}/pose`: The 3D pose (transform) of a specific board.
-   `/detections/final/{id}/bbox`: The 3D bounding box of a specific board.
-   `/detections/final/{id}/points`: The point cloud inliers for a specific board.

### 3. Integration with `DebugContext`

The revised example will fully integrate with the `board-fitter` library's `DebugContext`.

-   The `BoardDetector` will be initialized with a `DebugContext` that uses a custom callback.
-   This callback will receive `DebugData` from each stage of the pipeline (`PLANE_DETECTION`, `DIAMOND_FITTING`, etc.) and pass it directly to the `Viewer` for logging.
-   This ensures that the visualization is always in sync with the internal state of the detector and eliminates redundant data processing in the example itself.

### 4. Enhanced Visualization and Interactivity

The `Viewer` will leverage Rerun's features to create a more insightful visualization:

-   **Color by Confidence**: Final board detections will be colored based on their confidence score (e.g., green for high confidence, yellow for medium, red for low).
-   **Interactive Labels**: Hovering over a detected board will display its ID, confidence score, and other relevant metadata.
-   **3D Transforms and Bounding Boxes**: Each final detection will be visualized with its 3D bounding box and a coordinate system axis (TF frame) representing its pose.
-   **Per-Detection Point Clouds**: The specific point cloud inliers that contributed to a detection will be logged to a dedicated entity path (`/detections/final/{id}/points`), allowing users to isolate and inspect the points for a single board.
-   **Timeline Control**: The Rerun timeline will naturally correspond to the stages of the detection pipeline, allowing users to scrub through the process.

### 5. User Controls

The example will log Rerun Blueprints and controls to allow users to interact with the visualization:

-   **Checkboxes**: To toggle the visibility of each processing stage (e.g., "Show Planes", "Show Diamonds").
-   **Sliders**: To filter detections based on a confidence threshold.

## Implementation Plan

1.  **Refactor `main.rs`**: Create the new file structure (`args.rs`, `viewer.rs`). Implement argument parsing with `clap`.
2.  **Create `Viewer` Struct**: Implement the `Viewer` struct in `viewer.rs`. It will initialize the Rerun `RecordingStream`.
3.  **Implement Logging Functions**: Add methods to `Viewer` for each data type (e.g., `fn log_planes(&self, planes: &[DetectedPlane])`). Use the new hierarchical entity paths.
4.  **Integrate `DebugContext`**:
    -   In `main.rs`, create a `DebugContext` with a callback.
    -   The callback will pattern-match on the `DebugData` enum and call the appropriate `Viewer` logging method.
    -   Pass this `DebugContext` to the `BoardDetectorBuilder`.
5.  **Add Interactivity**: Implement the coloring, labels, and bounding boxes for the final detections.
6.  **Add UI Controls**: Use `rerun::log_blueprint` to add checkboxes and sliders to the Rerun UI for controlling the visualization.
7.  **Update `Cargo.toml`**: Ensure all necessary dependencies (`rerun`, `clap`) are included for the example.
8.  **Documentation**: Add comments to the example code explaining the new structure and how to use the interactive features.

## Example Usage

The revised example will be run from the command line with clear, configurable arguments:

```bash
cargo run --example rerun_visualization -- \
    --pcd-file /path/to/data.pcd \
    --board-config /path/to/board.json5 \
    --min-confidence 0.7 \
    --timeout 3000
```

## Success Metrics

-   A new user can easily understand the step-by-step process of the board detection algorithm just by interacting with the Rerun visualization.
-   The visualization clearly distinguishes between intermediate processing artifacts (planes, squares) and final, validated detections.
-   Key quality metrics, like confidence scores, are immediately apparent through visual cues like color.
-   The example code is modular, easy to read, and serves as a clear template for users who want to integrate `board-fitter` into their own applications.
