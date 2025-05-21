# ArUco Locator Messages

This package contains ROS 2 message and service definitions for ArUco marker detection.

## Services

### DetectAruco.srv

Service for detecting ArUco markers in images.

**Request:**
- `sensor_msgs/Image image` - Input image to process

**Response:**
- `bool success` - Whether the detection was successful
- `string message` - Human-readable status message
- `bool markers_found` - Whether any markers were detected
- `int32[] marker_ids` - Array of detected marker IDs
- `string detection_data` - Detailed marker information (JSON serialized)
- `float64 processing_time_ms` - Processing time in milliseconds

## Usage

After building this package, you can use the service definitions in your ROS 2 nodes:

```cpp
// C++
#include "aruco_locator_msgs/srv/detect_aruco.hpp"
```

```python
# Python
from aruco_locator_msgs.srv import DetectAruco
```

```rust
// Rust
use aruco_locator_msgs::srv::DetectAruco;
```

## Building

This package generates ROS 2 interfaces and should be built using colcon:

```bash
colcon build --packages-select aruco_locator_msgs
```