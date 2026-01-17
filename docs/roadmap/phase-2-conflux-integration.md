# Phase 2: Conflux Message Synchronization Integration

## Overview

Integrate the `conflux` message synchronization library into LCTK's calibration pipeline. The synchronizer will pair ArUco detection and LiDAR board detection messages by timestamp, enabling accurate multi-sensor calibration.

## Current State

### LCTK Calibration Pipeline
- `aruco_locator_node` publishes `Detection2DArray` (ArUco markers in image)
- `lidar_board_detector` publishes `Detection3DArray` (board pose in point cloud)
- `advanced_extrinsic_solver` caches latest messages independently (**no synchronization**)
- User manually triggers `add_detection` service to capture detection pairs

### Conflux Library (ros/conflux/)
- **conflux-core**: Pure Rust sync algorithm (time-window based)
- **conflux-ros2**: ROS2 utilities (dynamic subscriptions, timestamp extraction)
- **conflux_node**: Standalone ROS2 synchronizer node
- **Status**: Synchronization works, but **publishing is not implemented**

### rclrs Version Mismatch
| Component | rclrs Version | Source |
|-----------|---------------|--------|
| LCTK crates | 0.6.0 | crates.io |
| conflux | main branch (commit 562e815) | git |

**Risk**: Potential API incompatibilities between versions.

## Target Architecture

```
┌─────────────────────┐     ┌─────────────────────┐
│   aruco_locator     │     │  lidar_board_       │
│       node          │     │     detector        │
└──────────┬──────────┘     └──────────┬──────────┘
           │                           │
   Detection2DArray             Detection3DArray
           │                           │
           └───────────┬───────────────┘
                       ▼
           ┌───────────────────────┐
           │     conflux_node      │
           │  (message synchronizer)│
           └───────────┬───────────┘
                       │
         ┌─────────────┴─────────────┐
         ▼                           ▼
  Detection2DArray_sync      Detection3DArray_sync
  (aruco, synchronized)      (board, synchronized)
         │                           │
         └───────────┬───────────────┘
                     ▼
           ┌───────────────────────┐
           │ advanced_extrinsic_   │
           │       solver          │
           └───────────────────────┘
```

## Implementation Tasks

### Task 1: Verify rclrs Compatibility

**Goal**: Determine if conflux's rclrs (main) and LCTK's rclrs (0.6.0) can coexist.

**Steps**:
1. Check conflux's rclrs commit for API differences vs 0.6.0
2. Test building conflux within LCTK workspace
3. Document any breaking changes or required patches

**Decision Point**:
- If compatible: Proceed with integration
- If incompatible: Either upgrade LCTK to main branch rclrs, or pin conflux to 0.6.0

### Task 2: Implement Dynamic Publishing in conflux_node

**Goal**: Enable conflux_node to republish synchronized messages.

**Files to modify**:
- `ros/conflux/conflux_node/src/config.rs`
- `ros/conflux/conflux_node/src/node.rs`
- `ros/conflux/crates/conflux-ros2/src/lib.rs` (if needed)

**Config Changes**:
```yaml
# Before (single unused output)
output:
  topic: /synchronized

# After (per-input derived outputs)
inputs:
  - topic: /aruco_detections
    type: vision_msgs/msg/Detection2DArray
    # Output: /aruco_detections_sync (auto-derived)
  - topic: /board_detections
    type: vision_msgs/msg/Detection3DArray
    # Output: /board_detections_sync (auto-derived)

output:
  suffix: _sync  # Applied to each input topic
```

**Node Changes**:
1. Create dynamic publisher for each input topic at startup
2. Store publishers in a HashMap keyed by input topic
3. In `handle_synchronized_group()`:
   - For each message in the synchronized group
   - Look up corresponding publisher
   - Publish the message data

**Technical Approach for Publishing**:
```rust
// Option A: Raw publishing (if rclrs supports)
publisher.publish_raw(&msg.data)?;

// Option B: DynamicMessage reconstruction
let dynamic_msg = DynamicMessage::from_serialized(&msg.data, type_support)?;
publisher.publish(dynamic_msg)?;
```

### Task 3: Add vision_msgs Support to conflux

**Goal**: Ensure conflux can handle `Detection2DArray` and `Detection3DArray`.

**Steps**:
1. Add `vision_msgs` dependency to conflux workspace
2. Verify timestamp extraction works for Detection messages
3. Test with sample detection data

### Task 4: Create Calibration Sync Configuration

**Goal**: Provide ready-to-use config for LCTK calibration.

**File**: `ros/conflux/conflux_node/config/examples/lctk_calibration.yaml`

```yaml
# LCTK Calibration Board Detection Synchronization
#
# Synchronizes ArUco detections (camera) with board detections (LiDAR)
# for extrinsic calibration.

inputs:
  - topic: /calibration/aruco_locator/aruco_detections
    type: vision_msgs/msg/Detection2DArray
  - topic: /calibration/lidar_board_detector/calibration_board_detections
    type: vision_msgs/msg/Detection3DArray

output:
  suffix: _sync

sync:
  # 100ms window accommodates:
  # - Camera at 30Hz (33ms intervals)
  # - LiDAR at 10Hz (100ms intervals)
  window_size: 100ms
  buffer_size: 32

staleness:
  preset: high_frequency

qos:
  reliability: best_effort
  history_depth: 1
```

### Task 5: Update advanced_extrinsic_solver

**Goal**: Subscribe to synchronized topics instead of raw topics.

**Changes**:
1. Add parameter `use_synchronized_input` (default: false for backward compat)
2. When enabled, subscribe to `*_sync` topics
3. Remove independent caching - use synchronized pairs directly

**Modified subscription logic**:
```python
if self.use_synchronized_input:
    # Subscribe to synchronized topics
    self.aruco_subscription = self.create_subscription(
        Detection2DArray,
        "aruco_detections_sync",  # Synchronized
        self.synced_aruco_callback,
        qos_profile
    )
    # Similar for board detections
else:
    # Original behavior (cache latest independently)
    ...
```

### Task 6: Update Launch Files

**Goal**: Integrate conflux_node into calibration launch.

**File**: `ros/lctk_launch/launch/lidar_camera_calibration.launch.xml`

**Add**:
```xml
<!-- Message Synchronizer -->
<node pkg="conflux_node"
      exec="conflux_node"
      name="detection_sync"
      namespace="$(var namespace)"
      output="screen">
    <param name="config_file"
           value="$(find-pkg-share conflux_node)/config/examples/lctk_calibration.yaml"/>
</node>
```

### Task 7: Integration Testing

**Goal**: Verify end-to-end synchronization works.

**Test Cases**:
1. **Basic sync**: Play rosbag, verify synchronized pairs are published
2. **Timing accuracy**: Check timestamp differences in synchronized pairs
3. **Staleness**: Verify old messages are dropped appropriately
4. **Calibration accuracy**: Compare calibration results with/without sync

## Dependencies

```
Task 1 (rclrs compat) ──┬──► Task 2 (publishing)
                        │
                        └──► Task 3 (vision_msgs)
                                    │
                                    ▼
                        Task 4 (config) ──► Task 5 (solver) ──► Task 6 (launch)
                                                                      │
                                                                      ▼
                                                              Task 7 (testing)
```

## Risk Mitigation

| Risk | Impact | Mitigation |
|------|--------|------------|
| rclrs incompatibility | High | Test early; have fallback to upgrade or pin version |
| Dynamic publishing not supported | Medium | Implement DynamicMessage reconstruction |
| Performance overhead | Low | Benchmark sync latency; use high_frequency preset |
| Message type changes | Low | Use dynamic introspection, no hardcoded types |

## Success Criteria

1. conflux_node builds within LCTK workspace
2. Synchronized detection messages publish correctly
3. Timestamp difference between paired messages < window_size
4. advanced_extrinsic_solver works with synchronized input
5. Calibration accuracy maintained or improved

## Future Enhancements

- Add sync status topic for monitoring
- Support exact timestamp matching mode
- Integrate with calibration_orchestrator for multi-sensor scenarios
