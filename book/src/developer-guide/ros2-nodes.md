# ROS 2 Nodes

ROS 2 nodes in `src/bin/` wrap core libraries and handle ROS communication. Each node is a separate Rust crate that compiles to an executable.

## Node Architecture Pattern

**Standard node structure:**
```rust
use rclrs::{Node, Publisher, Subscription};
use arc_swap::ArcSwap;

pub struct NodeState {
    // Core algorithm (from src/lib/)
    detector: ArUcoDetector,

    // ROS publishers
    publisher: Publisher<Detection2DArray>,

    // Lock-free configuration updates
    config: Arc<ArcSwap<Config>>,
}

impl NodeState {
    fn callback(&self, msg: Image) {
        // Use core library
        let detections = self.detector.detect(&msg.data)?;

        // Publish ROS message
        self.publisher.publish(&detections)?;
    }
}
```

## Detection Nodes

### aruco_locator_node
**Location:** `src/bin/aruco_locator_node/`

**Purpose:** Detect ArUco markers in camera images

**Topics:**
- Input: `/image` (sensor_msgs/Image)
- Input: `/camera_info` (sensor_msgs/CameraInfo) - auto-derived from image topic
- Output: `/aruco_detections` (vision_msgs/Detection2DArray)

**Parameters:**
```bash
ros2 run aruco_locator_node aruco_locator_node \
    --ros-args \
    -p aruco_config_file:=/path/to/aruco_pattern.json5
```

**Key feature:** Waits for camera_info before processing images

### lidar_board_detector
**Location:** `src/bin/lidar_board_detector/` (actually in `src/ros2/`)

**Purpose:** Detect calibration boards in point clouds

**Topics:**
- Input: `/input_pointcloud` (sensor_msgs/PointCloud2)
- Output: `/calibration_board_detections` (vision_msgs/Detection3DArray)

**Parameters:**
```bash
ros2 run lidar_board_detector lidar_board_detector \
    --ros-args \
    -p board_detector_file:=/path/to/board_detector.json5 \
    -p bbox_file:=/path/to/bbox.json5
```

**Debug mode:** Set `debug_mode:=true` to publish intermediate detection steps

## Calibration Nodes

### extrinsic_solver_node
**Location:** `src/ros2/extrinsic_solver_node/` (Python)

**Purpose:** Compute LiDAR-to-camera transformation

**Topics:**
- Input: `/aruco_detections` (synchronized 2D detections)
- Input: `/calibration_board_detections` (synchronized 3D detections)
- Output: `/calibration_transform` (geometry_msgs/TransformStamped)

**Algorithm:** PnP solver with multi-marker support

### multi_wayside_node
**Location:** `src/ros2/multi_wayside_node/` (Rust)

**Purpose:** Multi-LiDAR calibration

**Topics:**
- Input: `/lidar1/board_detections`, `/lidar2/board_detections`
- Output: `/calibration_transform`
- Output: `/calibration_markers` (visualization)

**Services:**
- `/trigger_calibration`: Start calibration
- `/set_roi_bounds`: Adjust detection region

**Key feature:** Real-time detection synchronization with TF broadcasting

## Synchronization Node

### detection_synchronizer
**Location:** `src/ros2/detection_synchronizer/` (Rust)

**Purpose:** Time-align detections from multiple sources

**Implementation:**
```rust
pub struct SynchronizerState {
    aruco_buffer: VecDeque<Detection2DArray>,
    board_buffer: VecDeque<Detection3DArray>,
    window_size: Duration,
}

// Keep subscriptions alive as struct members!
pub struct SynchronizerNode {
    _aruco_subscription: Subscription<Detection2DArray>,
    _board_subscription: Subscription<Detection3DArray>,
}
```

**Configuration:**
- `window_size`: 500ms (default)
- `buffer_size`: 200 messages
- `quality_threshold`: 50

## Visualization Nodes

### pointcloud_image_overlay
**Location:** `src/bin/pointcloud_image_overlay/` (Python)

**Purpose:** Project calibrated point clouds onto images

**Topics:**
- Input: `/image`, `/pointcloud`, `/calibration_transform`
- Output: `/overlay_image`

**Feature:** Auto-derives camera_info topic from image topic

## Node Development Workflow

### 1. Create Node Package

```bash
cd src/bin/
cargo new --bin my_node
```

**Cargo.toml:**
```toml
[package]
name = "my_node"
version = "0.1.0"

[dependencies]
rclrs = { workspace = true }
sensor_msgs = { workspace = true }
my_detector = { path = "../../lib/my-detector" }
arc-swap = "1.7"
```

### 2. Implement Node

```rust
use rclrs::{create_node, spin, Context, Node};
use sensor_msgs::msg::Image;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let context = Context::new(std::env::args())?;
    let node = create_node(&context, "my_node")?;

    let state = Arc::new(MyNodeState::new(&node)?);

    let subscription = {
        let state = Arc::clone(&state);
        node.create_subscription::<Image, _>(
            "/input_topic",
            move |msg: Image| {
                state.process(msg);
            },
        )?
    };

    spin(&node)?;
    Ok(())
}
```

### 3. Add ROS Package Metadata

**CMakeLists.txt:**
```cmake
cmake_minimum_required(VERSION 3.10)
project(my_node)

find_package(ament_cmake REQUIRED)
install(PROGRAMS
    ${CMAKE_CURRENT_BINARY_DIR}/my_node
    DESTINATION lib/${PROJECT_NAME})

ament_package()
```

**package.xml:**
```xml
<package format="3">
  <name>my_node</name>
  <version>0.1.0</version>
  <description>My custom ROS 2 node</description>

  <depend>rclrs</depend>
  <depend>sensor_msgs</depend>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>colcon_cargo</buildtool_depend>
  <export>
    <build_type>ament_cargo</build_type>
  </export>
</package>
```

### 4. Build and Run

```bash
# Build
make build_packages

# Source workspace
source install/setup.bash

# Run node
ros2 run my_node my_node
```

## Common Node Patterns

### Parameter Loading

```rust
let config_file: String = node.declare_parameter("config_file")?;
let debug_mode: bool = node.declare_parameter("debug_mode", false)?;
```

### Topic Remapping

```xml
<node pkg="my_node" exec="my_node">
  <remap from="input" to="/sensing/camera/image"/>
  <remap from="output" to="/detection/results"/>
</node>
```

### Service Handlers

```rust
let service = node.create_service::<SetBool, _>(
    "enable_detection",
    move |_req_id, request| {
        // Handle request
        Response { success: true, message: "OK".to_string() }
    },
)?;
```

### Lock-Free Configuration Updates

```rust
use arc_swap::ArcSwap;

let config = Arc::new(ArcSwap::from_pointee(initial_config));

// Service updates config atomically
service_handler: {
    let config = Arc::clone(&config);
    move |new_config| {
        config.store(Arc::new(new_config));
    }
}

// Detection thread reads without blocking
detection_thread: {
    let current_config = config.load();
    detector.detect_with_config(&current_config);
}
```

## Debugging Nodes

### Enable Logging

```bash
export RCUTILS_LOGGING_LEVEL=DEBUG
ros2 run my_node my_node
```

### Inspect Node

```bash
# List nodes
ros2 node list

# Node info
ros2 node info /my_node

# View parameters
ros2 param list /my_node
ros2 param get /my_node config_file
```

### Monitor Topics

```bash
# Check topic rate
ros2 topic hz /output_topic

# View messages
ros2 topic echo /output_topic --no-arr

# Topic info
ros2 topic info /output_topic
```

## Performance Optimization

**Minimize message copies:**
```rust
subscription.create_subscription::<Image, _>(
    "topic",
    move |msg: Arc<Image>| {  // Use Arc for zero-copy
        process(&msg);
    },
)?;
```

**Batch processing:**
```rust
let mut buffer = Vec::with_capacity(10);
// Accumulate messages
// Process batch at once
```

**Preallocate publishers:**
```rust
// Create publisher once, reuse many times
let publisher = node.create_publisher::<Image>("topic")?;
```

## Next Steps

- [Build System](./build-system.md) - Building ROS packages
- [Testing](./testing.md) - Unit and integration testing
- [Advanced Topics](./advanced-topics.md) - Performance tuning
