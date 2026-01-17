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

### Task 1: Verify rclrs Compatibility ✅ DONE

**Goal**: Determine if conflux's rclrs (main) and LCTK's rclrs (0.6.0) can coexist.

**Resolution**: Use separate workspaces. LCTK stays on crates.io rclrs 0.6.0, conflux uses git rclrs with DynamicMessage support. Communication happens via wire-compatible ROS2 topics.

**Changes Made**:
- Added `ros/conflux` and `external/rclrs` to LCTK's workspace exclude list
- Added `test-release` profile to external/rclrs

### Task 2: Implement Dynamic Publishing in conflux_node ✅ DONE

**Goal**: Enable conflux_node to republish synchronized messages.

**Implementation** (see Phase 2a for details):
- Created `ros2_message.rs` - wrapper owning DynamicMessage
- Created `ros2_sync_state.rs` - sync state with move semantics
- Created `ros2_publisher.rs` - dynamic publisher manager
- Created `ros2_sync_node.rs` - complete sync runner
- Updated `subscriber.rs` - added `create_ros2_subscription()`
- Updated `config.rs` - changed output from `topic` to `suffix`
- Updated `node.rs` - uses new Ros2SyncRunner

**Config format**:
```yaml
inputs:
  - topic: /input_topic
    type: some_msgs/msg/SomeType

output:
  suffix: _sync  # Output: /input_topic_sync
```

### Task 3: ~~Add vision_msgs Support~~ NOT NEEDED

Conflux uses `DynamicMessage` with runtime type introspection. It works with **any** message type that has `header.stamp` - no special handling needed.

### Task 4: Create Calibration Sync Configuration ✅ DONE

**Goal**: Provide ready-to-use config for LCTK calibration.

**File**: `ros/lctk_launch/config/detection_sync.yaml`

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

**Output topics**:
- `/calibration/aruco_locator/aruco_detections_sync`
- `/calibration/lidar_board_detector/calibration_board_detections_sync`

### Task 5: ~~Update advanced_extrinsic_solver~~ → Launch File Remapping

**Original Goal**: Modify solver code to subscribe to synchronized topics.

**Simpler Approach**: No code changes needed. The extrinsic solvers simply subscribe to input topics and process messages on demand. We remap topics in the launch file so they receive pre-synchronized messages transparently.

**How it works**:
1. conflux_node subscribes to original detection topics
2. conflux_node publishes synchronized messages to `*_sync` topics
3. Launch file remaps solver subscriptions to `*_sync` topics
4. Solvers receive pre-synchronized data without knowing about conflux

### Task 6: Update Launch Files ✅ DONE

**Goal**: Integrate conflux_node into calibration launch with topic remapping.

**Files modified**:
- `ros/lctk_launch/launch/extrinsic_calibration.launch.xml`
- `ros/lctk_launch/launch/lidar_camera_calibration.launch.xml`

**Changes**:
- Added `use_synchronized_input` argument (default: false for backward compatibility)
- When enabled, launches conflux_node with detection_sync.yaml config
- Solver nodes conditionally remap to `*_sync` topics

**Usage**:
```bash
# Without synchronization (default, backward compatible)
ros2 launch lctk_launch lidar_camera_calibration.launch.xml

# With synchronization
ros2 launch lctk_launch lidar_camera_calibration.launch.xml use_synchronized_input:=true
```

### Task 7: Integration Testing

**Goal**: Verify end-to-end synchronization works.

**Test Cases**:
1. **Basic sync**: Play rosbag, verify synchronized pairs are published
2. **Timing accuracy**: Check timestamp differences in synchronized pairs
3. **Staleness**: Verify old messages are dropped appropriately
4. **Calibration accuracy**: Compare calibration results with/without sync

### Task 8: Add Timestamp Validation to Extrinsic Solvers (Future)

**Goal**: Add optional timestamp checking in extrinsic solvers to warn about desynchronized inputs.

**Note**: This is a future enhancement, not required for initial integration. The solvers will work correctly with synchronized inputs from conflux. This task adds defensive validation to detect if timestamps are unexpectedly far apart.

**Scope**:
- Add timestamp difference check when processing detection pairs
- Log warning if timestamps differ by more than expected window
- No pairing logic - just validation

## Dependencies

```
Task 1 (rclrs compat) ✅
        │
        ▼
Task 2 (publishing) ✅
        │
        ▼
Task 4 (config) ✅ ──► Task 6 (launch) ✅ ──► Task 7 (testing)
                                                    │
                                                    ▼
                                            Task 8 (timestamp validation) [Future]
```

## Risk Mitigation

| Risk | Impact | Mitigation | Status |
|------|--------|------------|--------|
| rclrs incompatibility | High | Separate workspaces, wire-compatible topics | ✅ Resolved |
| Dynamic publishing not supported | Medium | Implemented move semantics for DynamicMessage | ✅ Resolved |
| Performance overhead | Low | Benchmark sync latency; use high_frequency preset | To verify |
| Message type changes | Low | Use dynamic introspection, no hardcoded types | ✅ Resolved |

## Success Criteria

1. ✅ conflux_node builds (in separate workspace with git rclrs)
2. ✅ Config and launch files created for synchronized input
3. ⏳ Synchronized detection messages publish to `*_sync` topics (needs testing)
4. ⏳ Timestamp difference between paired messages < window_size (needs testing)
5. ⏳ Calibration workflow functions correctly end-to-end (needs testing)

## Future Enhancements

- Add sync status topic for monitoring
- Support exact timestamp matching mode
- Integrate with calibration_orchestrator for multi-sensor scenarios
