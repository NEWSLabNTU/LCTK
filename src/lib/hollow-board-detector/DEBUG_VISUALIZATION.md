# Debug Visualization System for Hollow Board Detection

This document describes the debug visualization system implemented for the hollow board detection algorithm, specifically for the ICP (Iterative Closest Point) iterations.

## Overview

The debug visualization system allows you to visualize the ICP algorithm's progress in real-time by publishing debug topics that can be viewed in RViz2. This is particularly useful for:

- Understanding how the board model converges during ICP iterations
- Debugging issues with board detection
- Analyzing the quality of point correspondences
- Visualizing the board model geometry (corners, holes, coordinate frame)

## Configuration

### Enable Debug Visualization

To enable debug visualization, set the `enable_debug_visualization` parameter to `true` in your board detector configuration file:

```json5
{
    // ... other parameters ...
    "enable_debug_visualization": true
}
```

### Configuration File

The parameter is added to `config/board_detector.json5`:

```json5
{
    // max number of RANSAC steps
    "plane_ransac_max_iterations": 500,
    // the loss threshold that a point is considered an inlier
    "plane_ransac_inlier_threshold": 0.05,
    // max number of ICP iterations
    "max_icp_iterations": 20000,
    // pose weight is amount of pose change per iteration
    // the ICP terminates if pose weight is blow this threshold several times
    "icp_pose_weight_threshold": 5e-13,
    // the maximum accepted ICP loss
    "icp_rejection_threshold": 1.0,
    // the length of border margin
    "board_width": "1000mm",    // mm
    // the radius of circle holes
    "hole_radius": "150mm",     // mm
    // suppose the center of board is (0, 0)
    // the center of hole will be at (+/- shift, +/- shift)
    "hole_center_shift": "200mm", // mm
    // enable debug visualization topics for ICP iterations
    "enable_debug_visualization": false
}
```

## Debug Topics

When debug visualization is enabled, the following topics are published:

### 1. Board Model Markers
- **Topic**: `/debug/board_model_markers`
- **Type**: `visualization_msgs/MarkerArray`
- **Description**: Visualizes the current board model including:
  - Board outline (yellow rectangle)
  - Hole circles (magenta)
  - Coordinate frame (RGB arrows: X=red, Y=green, Z=blue)

### 2. Input Point Cloud
- **Topic**: `/debug/input_point_cloud`
- **Type**: `sensor_msgs/PointCloud2`
- **Description**: The current inlier points being processed in the ICP iteration

### 3. Corresponding Points
- **Topic**: `/debug/corresponding_points`
- **Type**: `sensor_msgs/PointCloud2`
- **Description**: The corresponding model points found for the input points

### 4. ICP Iteration Data
- **Topic**: `/debug/icp_iteration_data`
- **Type**: Custom message (to be defined)
- **Description**: Numerical data about the current ICP iteration including:
  - Iteration number
  - Current loss
  - Pose weight
  - Board model pose

## Visualization in RViz2

### Setting up RViz2

1. Launch your board detection node with debug visualization enabled
2. Start RViz2: `rviz2`
3. Add the following displays:

#### Board Model Markers
- **Type**: MarkerArray
- **Topic**: `/debug/board_model_markers`
- **Description**: Shows the board model geometry

#### Input Points
- **Type**: PointCloud2
- **Topic**: `/debug/input_point_cloud`
- **Color**: Green
- **Size**: 0.01

#### Corresponding Points
- **Type**: PointCloud2
- **Topic**: `/debug/corresponding_points`
- **Color**: Red
- **Size**: 0.01

### What to Look For

1. **Board Model Convergence**: Watch how the board model (yellow rectangle with magenta holes) moves and rotates to align with the point cloud
2. **Point Correspondences**: Observe how the green input points align with the red corresponding points
3. **Coordinate Frame**: The RGB arrows show the board's orientation (X=red, Y=green, Z=blue)
4. **ICP Progress**: Monitor the console output for iteration numbers, losses, and pose weights

## Implementation Details

### Architecture

The debug visualization system is implemented using a trait-based approach:

```rust
pub trait DebugVisualizationPublisher: Send + Sync {
    fn publish_icp_debug_data(&self, data: &DebugVisualizationData) -> anyhow::Result<()>;
    fn publish_board_model_markers(&self, board_model: &BoardModel, iteration: usize) -> anyhow::Result<()>;
    fn publish_point_cloud_debug(&self, points: &[na::Point3<f64>], topic_suffix: &str) -> anyhow::Result<()>;
}
```

### Key Components

1. **DebugVisualizationData**: Contains all the data for a single ICP iteration
2. **DebugMarker**: Represents different types of visualization markers
3. **ROS2DebugPublisher**: ROS2 implementation of the debug publisher
4. **NoOpDebugPublisher**: No-op implementation when debug is disabled

### Integration Points

The debug visualization is integrated into the ICP loop in `algo.rs`:

1. **Before ICP iteration**: Publishes initial board model and input points
2. **After correspondence finding**: Publishes corresponding points
3. **After loss calculation**: Publishes complete debug data with calculated values

## Performance Considerations

- Debug visualization adds computational overhead
- Point cloud publishing can be bandwidth-intensive
- Consider disabling for production use
- The system gracefully handles publisher failures with warning messages

## Future Enhancements

1. **Custom ROS2 Messages**: Define proper custom message types for ICP debug data
2. **Real ROS2 Integration**: Replace placeholder implementations with actual ROS2 publishers
3. **Configurable Topics**: Allow topic names to be configured via parameters
4. **Selective Debugging**: Allow enabling/disabling specific debug topics
5. **Performance Metrics**: Add timing and performance data to debug output

## Troubleshooting

### Common Issues

1. **No debug topics appearing**: Check that `enable_debug_visualization` is set to `true`
2. **Missing markers**: Ensure the board detection is running and finding boards
3. **Performance issues**: Consider reducing the frequency of debug publishing

### Debug Output

The system provides console output for debugging:

```
Publishing ICP debug data for iteration 5: loss=0.012345, pose_weight=0.000001
Publishing board model markers for iteration 5: 6 markers
Publishing 1234 points to topic '/debug/input_point_cloud'
```

This output helps verify that the debug system is working correctly.

