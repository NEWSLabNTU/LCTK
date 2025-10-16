# Calibration Judge

ROS2 node for evaluating calibration quality by comparing estimated extrinsic transforms against ground truth.

## Overview

The `calibration_judge` package subscribes to the `/calibration/extrinsic_solver/extrinsic_transform` topic (or a custom topic) and compares each received transform against a ground truth transformation matrix. It computes and logs various error metrics in real-time.

## Usage

### Running the Node

```bash
# Source the workspace
source install/setup.bash

# Run with default parameters (no ground truth)
ros2 run calibration_judge judge_node

# Run with ground truth file
ros2 run calibration_judge judge_node --ros-args \
  -p ground_truth_file:=/path/to/ground_truth.txt

# Run with custom transform topic
ros2 run calibration_judge judge_node --ros-args \
  -p ground_truth_file:=/path/to/ground_truth.txt \
  -p transform_topic:=/custom/transform/topic
```

### Ground Truth File Format

The ground truth file should contain a 4x4 transformation matrix in plain text format (space or comma separated):

```
1.0 0.0 0.0 0.5
0.0 1.0 0.0 0.3
0.0 0.0 1.0 0.2
0.0 0.0 0.0 1.0
```

Where:
- Top-left 3x3 block is the rotation matrix
- Top-right 3x1 column is the translation vector (in meters)
- Bottom row should be [0, 0, 0, 1]

## Error Metrics

The node currently computes the following error metrics:

1. **Translation Error**: Euclidean distance between estimated and ground truth translation vectors (meters)
2. **Rotation Frobenius Error**: Frobenius norm of the difference between rotation matrices
3. **Rotation Angle Error**: Angle of the rotation difference (degrees)

## Future Work

The `_compute_score()` function is designed to be extended with additional metrics and an overall scoring function. Discuss and implement custom scoring algorithms as needed.

## Parameters

- `ground_truth_file` (string, default: ''): Path to the ground truth transformation matrix file
- `transform_topic` (string, default: '/calibration/extrinsic_solver/extrinsic_transform'): Topic name for extrinsic transform messages

## Topics

### Subscribed Topics

- `transform_topic` (geometry_msgs/TransformStamped): Extrinsic transformation from calibration solver
