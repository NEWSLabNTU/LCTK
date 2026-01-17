# Phase 1: Message Synchronization

## Overview

This document outlines the implementation plan for adding robust message synchronization across LCTK nodes. The goal is to ensure that sensor data from multiple sources (LiDAR point clouds, camera images, detection messages) are properly time-aligned before processing.

## Problem Statement

Current nodes like `extrinsic_solver_node` and `advanced_extrinsic_solver` receive data from multiple topics:
- ArUco marker detections (from camera/image processing)
- Board detections (from LiDAR point cloud processing)
- Camera info

These messages arrive asynchronously and may have different:
- Publishing rates (e.g., camera at 30Hz, LiDAR at 10Hz)
- Latencies (processing time, network delays)
- Timestamp sources (different clocks, hardware triggers)

Without proper synchronization, the solver may process mismatched data pairs, leading to calibration errors.

## Real-Time Constraints

### Non-1:1 Frame Correspondence

Sensors operate at different rates and are not necessarily synchronized:

```
Camera (30 Hz):   |C1|C2|C3|C4|C5|C6|C7|C8|C9|...
LiDAR  (10 Hz):   |  L1  |  L2  |  L3  |  ...
Time (ms):        0  33  66 100 133 166 200 ...
```

**Key insight**: Not every camera frame has a corresponding LiDAR frame, and vice versa. The synchronizer must:
- Accept that some frames will be unmatched
- Select the best available pair within tolerance
- Never assume 1:1 correspondence

### Wall Clock Latency Bounds

For real-time applications, messages cannot be queued indefinitely waiting for a match:

```
Wall Clock Latency Budget:
┌─────────────────────────────────────────────────────────┐
│ Sensor capture → Processing → Sync wait → Output       │
│     ~10ms          ~50ms        ≤100ms      ~10ms      │
│                                   ↑                     │
│                          Must be bounded!               │
└─────────────────────────────────────────────────────────┘
```

**Requirements:**
1. **Maximum wait time**: A message waiting for sync must be dropped after `max_wait_ms`
2. **Maximum age**: Messages older than `max_age_ms` (wall clock) are stale and dropped
3. **No unbounded queues**: Buffer sizes are strictly limited

### Dropped Message Scenarios

Messages will be intentionally dropped when:

| Scenario                       | Behavior                    | Rationale                  |
|--------------------------------|-----------------------------|----------------------------|
| No match within `max_wait_ms`  | Drop oldest waiting message | Prevent unbounded latency  |
| Message age > `max_age_ms`     | Drop immediately            | Stale data is useless      |
| Buffer full, no match possible | Drop oldest                 | Prevent memory growth      |
| Rate mismatch (faster stream)  | Drop excess frames          | Keep up with slower stream |

### Matching Strategy

When multiple candidates exist within the tolerance window, use **nearest timestamp**:

```
LiDAR frame at t=100ms
Camera candidates: C3(t=90ms), C4(t=110ms), C5(t=130ms)
Tolerance: 50ms

→ All three are within tolerance
→ Select C4 (nearest to LiDAR timestamp: |110-100| = 10ms)
→ C3 and C5 are dropped (already matched or will be matched to other LiDAR frames)
```

## Goals

1. **Unified synchronization strategy** across Python and Rust nodes
2. **Configurable matching modes**: exact timestamp and approximate matching
3. **Latency-aware**: drop stale messages to prevent processing outdated data
4. **Minimal latency overhead**: synchronization should not significantly delay processing

## Implementation Strategy

### Python Nodes: `message_filters`

Use the standard ROS 2 `message_filters` package for Python nodes.

**Target Nodes:**
- `ros/extrinsic_solver_node/`
- `ros/advanced_extrinsic_solver/`
- `ros/pointcloud_image_overlay/`

### Rust Nodes: `multi-stream-synchronizer`

Use the existing `rust/multi-stream-synchronizer/` library for Rust nodes.

**Target Nodes:**
- `ros/lidar_board_detector/`
- `ros/aruco_locator_node/`
- `ros/multi_wayside_node/`

---

## Configuration Design

### ROS Parameters

Each synchronized node will expose the following parameters:

```yaml
# Synchronization mode
sync_mode: "approximate"  # Options: "exact", "approximate", "disabled"

# Timestamp matching settings
sync_tolerance_ms: 50.0   # Maximum timestamp difference (ms) for approximate matching
sync_queue_size: 10       # Number of messages to buffer per topic

# Real-time latency constraints (wall clock based)
max_wait_ms: 100.0        # Maximum time to wait for a match before dropping
max_message_age_ms: 200.0 # Drop messages older than this (wall clock age)

# Behavior settings
drop_policy: "oldest"     # Options: "oldest", "newest", "all"
enable_statistics: true   # Log match/drop statistics

# Advanced settings (optional)
rate_tolerance_ratio: 3.0 # Max rate difference before warning (e.g., 30Hz vs 10Hz = 3.0)
```

### Synchronization Modes

| Mode          | Description                                | Use Case                               |
|---------------|--------------------------------------------|----------------------------------------|
| `exact`       | Messages must have identical timestamps    | Hardware-triggered sensors, simulation |
| `approximate` | Messages within `sync_tolerance_ms` window | Real sensors with timing jitter        |
| `disabled`    | No synchronization, use latest available   | Debugging, backward compatibility      |

---

## Detailed Implementation Plan

### Task 1: Create Shared Configuration Types

**Files to create/modify:**
- `ros/lctk_common/sync_config.py` (Python)
- `rust/lctk-sync-config/` (Rust crate)

**Configuration struct:**

```python
@dataclass
class SyncConfig:
    mode: Literal["exact", "approximate", "disabled"]
    tolerance_ms: float        # Timestamp matching tolerance
    queue_size: int            # Buffer size per stream
    max_wait_ms: float         # Wall clock: max time waiting for match
    max_age_ms: float          # Wall clock: max message age
    drop_policy: Literal["oldest", "newest", "all"]
    enable_statistics: bool

@dataclass
class SyncStatistics:
    matched_count: int
    dropped_timeout: int       # Dropped due to max_wait exceeded
    dropped_stale: int         # Dropped due to max_age exceeded
    dropped_overflow: int      # Dropped due to buffer full
    avg_match_latency_ms: float
    avg_timestamp_diff_ms: float
```

```rust
pub struct SyncConfig {
    pub mode: SyncMode,
    pub tolerance: Duration,
    pub queue_size: usize,
    pub max_wait: Duration,    // Wall clock limit
    pub max_age: Duration,     // Wall clock limit
    pub drop_policy: DropPolicy,
    pub enable_statistics: bool,
}

pub enum SyncMode {
    Exact,
    Approximate,
    Disabled,
}

pub enum DropPolicy {
    Oldest,   // Drop oldest unmatched message
    Newest,   // Drop newest (keep older for better chance of match)
    All,      // Drop all pending when timeout
}

pub struct SyncStatistics {
    pub matched_count: u64,
    pub dropped_timeout: u64,
    pub dropped_stale: u64,
    pub dropped_overflow: u64,
    pub avg_match_latency: Duration,
    pub avg_timestamp_diff: Duration,
}
```

---

### Task 2: Python Synchronizer Wrapper

**File:** `ros/lctk_common/synchronizer.py`

Create a wrapper that abstracts `message_filters` with our configuration:

```python
class LctkSynchronizer:
    """
    Unified synchronizer for LCTK Python nodes.

    Supports:
    - ExactTime: Requires identical timestamps
    - ApproximateTime: Allows configurable timestamp tolerance
    - Disabled: Passthrough mode (no synchronization)
    """

    def __init__(
        self,
        node: Node,
        subscribers: List[Tuple[type, str]],  # [(msg_type, topic), ...]
        callback: Callable,
        config: SyncConfig,
    ):
        ...

    def update_config(self, config: SyncConfig) -> None:
        """Runtime configuration update."""
        ...
```

**Features:**
- Automatic selection of `TimeSynchronizer` or `ApproximateTimeSynchronizer`
- Staleness checking via message age validation in callback
- Runtime reconfiguration support
- Logging of dropped/matched messages for debugging

---

### Task 3: Rust Synchronizer Integration

**File:** `rust/lctk-ros-sync/src/lib.rs`

Create a ROS 2-aware wrapper around `multi-stream-synchronizer`:

```rust
pub struct RosSynchronizer<K, T> {
    inner: MultiStreamSynchronizer<K, T>,
    config: SyncConfig,
}

impl<K: Key, T: WithTimestamp> RosSynchronizer<K, T> {
    pub fn new(stream_names: Vec<K>, config: SyncConfig) -> Result<Self>;

    /// Add a message from a subscription callback
    pub fn push(&mut self, key: K, message: T, ros_time: Time) -> Result<()>;

    /// Try to get a synchronized group
    pub fn try_pop(&mut self) -> Option<IndexMap<K, T>>;

    /// Check and expire stale messages
    pub fn expire_stale(&mut self);
}
```

**Exact Mode Implementation:**
- Set `window_size` to `Duration::ZERO` or very small value
- Only emit groups where all timestamps match exactly

**Approximate Mode Implementation:**
- Set `window_size` to `config.tolerance`
- Use staleness config based on `max_age` and `enable_staleness`

---

### Task 4: Update `advanced_extrinsic_solver`

**File:** `ros/advanced_extrinsic_solver/advanced_extrinsic_solver/main.py`

**Changes:**

1. Add sync parameters to node declaration:
```python
self.declare_parameter("sync_mode", "approximate")
self.declare_parameter("sync_tolerance_ms", 100.0)
self.declare_parameter("sync_queue_size", 10)
self.declare_parameter("max_message_age_ms", 500.0)
```

2. Replace manual caching with `LctkSynchronizer`:
```python
# Before: separate callbacks caching latest messages
# After: synchronized callback receiving matched pairs

self.synchronizer = LctkSynchronizer(
    node=self,
    subscribers=[
        (Detection2DArray, "aruco_detections"),
        (Detection3DArray, "calibration_board_detections"),
    ],
    callback=self.synced_detection_callback,
    config=sync_config,
)

def synced_detection_callback(self, aruco_msg, board_msg):
    """Called only when synchronized pair is available."""
    with self.lock:
        self.latest_aruco_detection = aruco_msg
        self.latest_board_detection = board_msg
```

3. Add status reporting for sync statistics

---

### Task 5: Update `extrinsic_solver_node`

**File:** `ros/extrinsic_solver_node/extrinsic_solver_node/main.py`

Same pattern as Task 4.

---

### Task 6: Update `lidar_board_detector` (Rust)

**File:** `ros/lidar_board_detector/src/main.rs`

If this node needs to synchronize multiple inputs (e.g., multiple LiDAR streams):

1. Add sync configuration parameters
2. Integrate `RosSynchronizer` for input handling
3. Process only synchronized message groups

---

### Task 7: Update `multi_wayside_node` (Rust)

**File:** `ros/multi_wayside_node/src/detection/synchronizer.rs`

The node already has `DefaultDetectionSynchronizer`. Enhance it to:

1. Support exact mode (currently only approximate)
2. Add configurable parameters via ROS params
3. Use `multi-stream-synchronizer` library directly instead of custom implementation

---

### Task 8: Documentation and Examples

**Files to create:**
- `docs/user-guide/message-synchronization.md`
- `ros/lctk_launch/config/sync_examples.yaml`

**Content:**
- Explanation of synchronization modes
- Configuration examples for common scenarios
- Troubleshooting guide for sync issues
- Performance tuning recommendations

---

## Testing Plan

### Unit Tests

1. **Python synchronizer tests:**
   - Exact mode: only identical timestamps pass
   - Approximate mode: messages within tolerance pass
   - Staleness: old messages are dropped
   - Queue overflow handling

2. **Rust synchronizer tests:**
   - Leverage existing `multi-stream-synchronizer` tests
   - Add ROS-specific wrapper tests

### Integration Tests

1. **Rosbag replay tests:**
   - Record synchronized sensor data
   - Verify sync behavior with known timestamps
   - Measure synchronization latency

2. **Live sensor tests:**
   - Test with actual camera + LiDAR setup
   - Verify calibration accuracy with sync enabled vs disabled

### Performance Tests

1. Measure added latency from synchronization
2. Memory usage under sustained load
3. Behavior under message rate mismatch

---

## Migration Guide

### For Existing Launch Files

Add sync parameters to node configurations:

```xml
<node pkg="advanced_extrinsic_solver" exec="advanced_extrinsic_solver">
    <!-- Existing parameters -->
    <param name="parent_frame" value="lidar"/>
    <param name="child_frame" value="camera"/>

    <!-- New sync parameters -->
    <param name="sync_mode" value="approximate"/>
    <param name="sync_tolerance_ms" value="100.0"/>
    <param name="max_message_age_ms" value="500.0"/>
</node>
```

### Backward Compatibility

- Default `sync_mode: "disabled"` for backward compatibility
- Nodes function identically when sync is disabled
- Deprecation warnings for nodes not using synchronization

---

## Timeline Estimate

| Task | Description | Complexity |
|------|-------------|------------|
| 1 | Shared configuration types | Low |
| 2 | Python synchronizer wrapper | Medium |
| 3 | Rust synchronizer integration | Medium |
| 4 | Update advanced_extrinsic_solver | Medium |
| 5 | Update extrinsic_solver_node | Low |
| 6 | Update lidar_board_detector | Medium |
| 7 | Update multi_wayside_node | Low |
| 8 | Documentation | Low |

---

## Design Decisions

### Resolved

1. **Frame correspondence**: Explicitly support non-1:1 matching. Faster streams will have frames dropped.

2. **Latency bounds**: Wall clock based limits are mandatory. Messages are dropped, not queued indefinitely.

3. **Drop policy**: Configurable via `drop_policy` parameter. Default is `oldest`.

### Open Questions

1. **Camera info synchronization**: Should `CameraInfo` be included in sync group, or is it acceptable to use cached value (rarely changes)?

2. **Multi-LiDAR sync**: For `multi_wayside_node`, should we sync across LiDARs or just between LiDAR and camera per unit?

3. **TF synchronization**: Should we also synchronize TF lookups with message timestamps?

4. **Statistics publishing**: Should sync statistics be:
   - Published as a ROS topic?
   - Logged periodically?
   - Available only via service call?

5. **Graceful degradation**: When one stream stops publishing:
   - Timeout and drop all pending from other streams?
   - Switch to single-stream passthrough mode?
   - Emit error and stop processing?

---

## Example Scenarios

### Scenario 1: Normal Operation (Camera 30Hz, LiDAR 10Hz)

```
Config: tolerance=50ms, max_wait=100ms, max_age=200ms

Timeline (wall clock):
  0ms: Camera C1 arrives (stamp=0ms)     → Buffer: [C1]
 33ms: Camera C2 arrives (stamp=33ms)    → Buffer: [C1, C2]
 66ms: Camera C3 arrives (stamp=66ms)    → Buffer: [C1, C2, C3]
100ms: LiDAR L1 arrives (stamp=100ms)    → Match C3 with L1 (diff=34ms < 50ms)
                                         → Output: (C3, L1)
                                         → Drop C1, C2 (older than match)
                                         → Buffer: []
```

### Scenario 2: LiDAR Delayed (exceeds max_wait)

```
Config: tolerance=50ms, max_wait=100ms

Timeline:
  0ms: Camera C1 arrives     → Buffer: [C1]
 33ms: Camera C2 arrives     → Buffer: [C1, C2]
 66ms: Camera C3 arrives     → Buffer: [C1, C2, C3]
100ms: C1 waited 100ms       → Drop C1 (max_wait exceeded)
                             → Buffer: [C2, C3]
133ms: Camera C4 arrives     → Buffer: [C2, C3, C4]
133ms: C2 waited 100ms       → Drop C2 (max_wait exceeded)
150ms: LiDAR L1 arrives      → Match C4 with L1
                             → Output: (C4, L1)
```

### Scenario 3: Stale Message Rejected

```
Config: max_age=200ms

Timeline:
  0ms: Camera C1 arrives (stamp=0ms, captured at wall=-50ms due to processing)
      → Wall clock age = 50ms, OK
      → Buffer: [C1]

200ms: Camera C2 arrives (stamp=100ms, captured at wall=100ms)
      → Wall clock age = 100ms, OK
      → C1 wall clock age now = 250ms > 200ms
      → Drop C1 (stale)
      → Buffer: [C2]
```

### Scenario 4: Exact Mode

```
Config: mode=exact

Timeline:
  0ms: Camera C1 arrives (stamp=1000000000ns)  → Buffer: [C1]
 10ms: LiDAR L1 arrives (stamp=1000000000ns)   → Exact match!
                                               → Output: (C1, L1)

 50ms: Camera C2 arrives (stamp=1050000000ns)  → Buffer: [C2]
 60ms: LiDAR L2 arrives (stamp=1050000001ns)   → No match (1ns diff)
                                               → Buffer: [C2], [L2]
100ms: Both waiting > max_wait                 → Drop both
```

---

## References

- [ROS 2 message_filters Documentation](https://docs.ros.org/en/humble/p/message_filters/doc/index.html)
- [ApproximateTime Synchronizer Tutorial (C++)](https://docs.ros.org/en/rolling/p/message_filters/doc/Tutorials/Approximate-Synchronizer-Cpp.html)
- [ApproximateTime Synchronizer Tutorial (Python)](https://docs.ros.org/en/rolling/p/message_filters/doc/Tutorials/Approximate-Synchronizer-Python.html)
- [multi-stream-synchronizer README](../../rust/multi-stream-synchronizer/README.md)
- [multi-stream-synchronizer Algorithm](../../rust/multi-stream-synchronizer/ALGORITHM.md)
