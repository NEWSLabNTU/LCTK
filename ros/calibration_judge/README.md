# Calibration Judge

ROS2 node for evaluating calibration quality by comparing estimated extrinsic transforms against ground truth.

## Overview

The `calibration_judge` package subscribes to the `/calibration/extrinsic_solver/extrinsic_transform` topic (or a custom topic) and compares each received transform against a ground truth transformation matrix. It computes and logs calibration quality scores in real-time using a configurable scoring system with linear interpolation between min/max error thresholds.

## Usage

### Running the Node

**IMPORTANT**: The `ground_truth_file` parameter is **mandatory**. The node will fail to start without it.

```bash
# Source the workspace
source install/setup.bash

# Run with ground truth config file (REQUIRED)
ros2 run calibration_judge judge_node --ros-args \
  -p ground_truth_file:=/path/to/ground_truth_config.yaml

# Run with custom transform topic
ros2 run calibration_judge judge_node --ros-args \
  -p ground_truth_file:=/path/to/ground_truth_config.yaml \
  -p transform_topic:=/custom/transform/topic
```

### Ground Truth Configuration File Format

The configuration file is a YAML file containing the ground truth transformation matrix and scoring parameters:

```yaml
ground_truth:
  # 4x4 transformation matrix (LiDAR to Camera)
  matrix:
    - [-0.008120, -0.999550, +0.028668, -0.016307]
    - [-0.007539, -0.028730, -0.999556, -0.146427]
    - [+0.999939, -0.008541, -0.014490, -0.153410]
    - [0.0, 0.0, 0.0, 1.0]

scoring:
  total_score: 100.0  # Maximum possible score

  translation:
    weight: 0.5         # 50% of total score
    min_error_m: 0.01   # Errors below this get full points
    max_error_m: 0.10   # Errors above this get zero points

  rotation:
    weight: 0.5         # 50% of total score
    min_error_deg: 0.5  # Errors below this get full points
    max_error_deg: 5.0  # Errors above this get zero points
```

**Scoring System**:
- Translation and rotation each contribute 50% to the total score (configurable via `weight`)
- Errors below `min_error` threshold receive full points for that component
- Errors above `max_error` threshold receive zero points for that component
- Errors between min and max are linearly interpolated

## Scoring Output

The node logs calibration scores in real-time for each incoming transform:

```
======================================================================
Calibration Quality Score:

  Translation Error: 0.0234 m
    → Translation Score: 35.22/50.00 (70.4%)

  Rotation Angle Error: 1.234 degrees
    → Rotation Score: 41.85/50.00 (83.7%)

  FINAL SCORE: 77.07/100.00 (77.1%)
======================================================================
```

## Error Metrics

The node computes the following error metrics:

1. **Translation Error**: Euclidean distance between estimated and ground truth translation vectors (meters)
2. **Rotation Angle Error**: Angular difference between estimated and ground truth rotation matrices (degrees)
   - Computed using the trace of R_gt^T @ R_est

## Customizing Scoring Thresholds

Edit the `ground_truth_config.yaml` file to adjust scoring parameters:

- Modify `min_error` thresholds for stricter grading (smaller values = harder to get full points)
- Modify `max_error` thresholds for pass/fail boundary (larger values = more forgiving)
- Adjust `weight` values to change the relative importance of translation vs rotation (must sum to 1.0)
- Change `total_score` to scale the final output (e.g., 10.0 for a 0-10 scale)

## Parameters

- `ground_truth_file` (string, **REQUIRED**, no default): Path to the ground truth configuration YAML file
- `transform_topic` (string, default: '/calibration/extrinsic_solver/extrinsic_transform'): Topic name for extrinsic transform messages

## Topics

### Subscribed Topics

- `transform_topic` (geometry_msgs/TransformStamped): Extrinsic transformation from calibration solver
