# Multi-Wayside Node Architecture

## System Architecture Overview

The multi_wayside_node implements a modular, trait-based architecture for real-time LiDAR-to-LiDAR calibration using ROS 2. The system is designed with clear separation of concerns, dependency injection, and comprehensive testing support.

## Core Architecture Principles

### 1. Trait-Based Dependency Injection
All major components implement traits to enable:
- **Testability**: Mock implementations for unit testing
- **Modularity**: Swappable implementations
- **Maintainability**: Clear interface contracts

### 2. Layered Architecture
```
┌─────────────────────────────────────────┐
│           ROS 2 Interface Layer         │
│  (Publishers, Subscribers, Services)    │
├─────────────────────────────────────────┤
│          Application Layer              │
│     (Main orchestration logic)          │
├─────────────────────────────────────────┤
│          Processing Layer               │
│  (Detection pipeline, ROI management)   │
├─────────────────────────────────────────┤
│           Core Library Layer            │
│   (Point cloud parsing, filtering)      │
└─────────────────────────────────────────┘
```

### 3. Module Organization
Each module has a single responsibility with well-defined interfaces:

```
src/
├── main.rs              # Application orchestration
├── node/                # ROS 2 interface layer
├── detection/           # Board detection pipeline
├── pointcloud/          # Point cloud processing
├── roi/                 # Region of Interest management
├── visualization/       # Marker generation
├── calibration/         # Transform computation
├── config/              # Configuration handling
├── types/               # Shared data structures
└── utils/               # Utility functions
```

## Core Traits and Interfaces

### Point Cloud Processing
```rust
pub trait PointCloudParser: Send + Sync {
    fn parse(&self, msg: &PointCloud2) -> Result<Vec<LidarPoint>>;
    fn to_nalgebra_points(&self, points: &[LidarPoint]) -> Vec<Point3<f64>>;
}

pub trait PointCloudFilter: Send + Sync {
    fn filter_nalgebra(&self, points: &[Point3<f64>]) -> Vec<Point3<f64>>;
}
```

### ROI Management
```rust
pub trait RoiManager: Send + Sync {
    fn get_bounds(&self, lidar_id: u8) -> Option<RoiBounds>;
    fn set_bounds(&self, lidar_id: u8, bounds: RoiBounds) -> Result<()>;
    fn apply_crop(&self, points: &[Point3<f64>], lidar_id: u8) -> Vec<Point3<f64>>;
}
```

### Detection Processing
```rust
pub trait DetectionProcessor: Send + Sync {
    fn process(&self, points: &[Point3<f64>]) -> Result<Option<BoardDetection>>;
}
```

### Visualization
```rust
pub trait RoiMarkerGenerator: Send + Sync {
    fn generate_roi_marker(&self, bounds: &RoiBounds, lidar_id: u8, header: Header) -> MarkerArray;
}

pub trait TextMarkerGenerator: Send + Sync {
    fn generate_status_text(&self, text: &str, position: Point, header: Header) -> Marker;
    fn generate_detection_status(&self, lidar1_detected: bool, lidar2_detected: bool, sync_status: &str, header: Header) -> MarkerArray;
}
```

## Data Flow Architecture

### Processing Pipeline
```
PointCloud2 Input
       ↓
[PointCloudParser] → Parse to LidarPoint
       ↓
[PointCloudFilter] → Apply range filtering
       ↓
[RoiManager] → Apply ROI cropping
       ↓
[DetectionProcessor] → Detect calibration board
       ↓
[DetectionSynchronizer] → Match detections across LiDARs
       ↓
[CalibrationProcessor] → Compute transform
       ↓
Transform Output
```

### Message Flow
```
Input Topics → Subscribers → Processing Pipeline → Publishers → Output Topics
     ↓              ↓              ↓              ↓              ↓
/lidar1/points → pointcloud → detection → markers → /calibration_markers
/lidar2/points → processing → sync → transform → /calibration_transform
```

## Component Architecture Details

### 1. ROS 2 Interface Layer (`node/`)

#### Publishers (`node/publishers.rs`)
- Manages all ROS 2 publishers
- Publishes detection results, markers, and transforms
- Thread-safe publishing with Arc<Publisher<T>>

#### Subscribers (`node/subscribers.rs`)
- Handles incoming point cloud messages
- Manages subscription callbacks
- Coordinates with processing pipeline

#### Services (`node/services.rs`)
- ROI configuration services
- Parameter update services
- Status query services

### 2. Detection Pipeline (`detection/`)

#### Detection Processor (`detection/processor.rs`)
Complete point cloud processing pipeline:
```rust
pub struct DetectionPipeline<P, F, R, D> {
    parser: Arc<P>,
    filter: Arc<F>,
    roi_manager: Arc<R>,
    detector: Arc<D>,
}
```

#### Synchronizer (`detection/synchronizer.rs`)
Multi-LiDAR detection synchronization:
- Time-based matching with configurable tolerance
- Thread-safe access with Arc<Mutex<>>
- Automatic calibration triggering

### 3. Point Cloud Processing (`pointcloud/`)

#### Parser (`pointcloud/parser.rs`)
- PointCloud2 to internal format conversion
- Handles different point cloud field layouts
- Nalgebra integration for geometric operations

#### Filter (`pointcloud/filter.rs`)
- Range-based filtering
- Statistical outlier removal
- Configurable filter parameters

### 4. ROI Management (`roi/`)

#### Manager (`roi/manager.rs`)
- Per-LiDAR ROI bounds management
- Thread-safe state updates
- Parameter-driven configuration

#### Service Handlers (`roi/service.rs`)
- ROI bounds update services
- Interactive ROI adjustment
- Validation and error handling

### 5. Visualization (`visualization/`)

#### ROI Markers (`visualization/roi_markers.rs`)
- 3D ROI box visualization
- Color-coded per LiDAR
- Text labels with dimensions

#### Board Markers (`visualization/board_markers.rs`)
- Detected board visualization
- Pose and orientation display
- ArUco marker visualization

#### Text Markers (`visualization/text_markers.rs`)
- Status display
- Detection indicators
- Real-time feedback

### 6. Calibration (`calibration/`)

#### Transform Computation (`calibration/transform.rs`)
- LiDAR-to-LiDAR transform calculation
- Quality assessment metrics
- Validation and filtering

#### Validator (`calibration/validator.rs`)
- Transform reasonableness checking
- Quality thresholds
- Error reporting

## Thread Safety and Concurrency

### Shared State Management
All shared state uses Arc<Mutex<T>> or Arc<RwLock<T>>:
```rust
// Detection synchronizer with thread-safe access
Arc<Mutex<DefaultDetectionSynchronizer>>

// ROI manager with concurrent read/write
Arc<RwLock<DefaultRoiManager>>
```

### Message Passing
- Subscribers run in separate threads
- Processing pipeline is thread-safe
- Publishers handle concurrent access

### Resource Management
- RAII patterns for automatic cleanup
- Smart pointers for memory safety
- Error propagation with eyre::Result

## Configuration Architecture

### Parameter System
```rust
pub struct NodeParameters {
    pub board_config_file: String,
    pub detector_config_file: String,
    pub aruco_pattern_file: String,
    pub max_queue_size: usize,
    pub sync_tolerance_ms: u64,
    pub same_face_mode: bool,
    pub apply_bug_fix: bool,
    pub roi_box_size_x: f64,
    pub roi_box_size_y: f64,
    pub roi_box_size_z: f64,
    pub roi_box_position_x: f64,
    pub roi_box_position_y: f64,
    pub roi_box_position_z: f64,
    pub min_range: f64,
    pub max_range: f64,
}
```

### Configuration Loading
- ROS 2 parameter system integration
- YAML configuration file support
- Runtime parameter updates via services

## Error Handling Architecture

### Error Propagation
Consistent error handling using eyre::Result:
```rust
pub fn process_pointcloud(&self, msg: &PointCloud2, lidar_id: u8) -> Result<ProcessingResult> {
    let parsed_points = self.parser.parse(msg)?;
    let filtered_points = self.filter.filter_nalgebra(&nalgebra_points);
    let cropped_points = self.roi_manager.apply_crop(&filtered_points, lidar_id);
    let detection = self.detector.process(&cropped_points)?;
    
    Ok(ProcessingResult {
        original_points: nalgebra_points,
        filtered_points,
        cropped_points,
        detection,
    })
}
```

### Error Recovery
- Graceful degradation on detection failures
- Timeout handling for blocked operations
- Logging and diagnostics for debugging

## Testing Architecture

### Unit Testing Strategy
Each module has comprehensive unit tests:
- Mock implementations of all traits
- Isolated testing of individual components
- Property-based testing for edge cases

### Integration Testing
- End-to-end pipeline testing
- ROS 2 message flow validation
- Performance benchmarking

### Test Infrastructure
```
tests/
├── unit/                    # Per-module unit tests
├── integration/             # End-to-end tests
├── mocks/                   # Mock implementations
└── fixtures/                # Test data and configurations
```

## Performance Considerations

### Real-Time Processing
- Target: >10 Hz processing rate
- Memory-efficient point cloud handling
- Optimized geometric computations

### Resource Usage
- <50% CPU on target hardware
- <500MB memory usage
- Bounded queue sizes to prevent memory leaks

### Scalability
- Configurable buffer sizes
- Adaptive processing based on load
- Graceful degradation under high load

## Extension Points

### Adding New Filters
Implement the PointCloudFilter trait:
```rust
struct CustomFilter;

impl PointCloudFilter for CustomFilter {
    fn filter_nalgebra(&self, points: &[Point3<f64>]) -> Vec<Point3<f64>> {
        // Custom filtering logic
    }
}
```

### Adding New Detectors
Implement the DetectionProcessor trait:
```rust
struct CustomDetector;

impl DetectionProcessor for CustomDetector {
    fn process(&self, points: &[Point3<f64>]) -> Result<Option<BoardDetection>> {
        // Custom detection logic
    }
}
```

### Adding New Visualizations
Implement marker generation traits:
```rust
struct CustomMarkerGenerator;

impl RoiMarkerGenerator for CustomMarkerGenerator {
    fn generate_roi_marker(&self, bounds: &RoiBounds, lidar_id: u8, header: Header) -> MarkerArray {
        // Custom marker generation
    }
}
```

This architecture provides a solid foundation for the multi_wayside_node with clear separation of concerns, comprehensive testing support, and extensibility for future enhancements.