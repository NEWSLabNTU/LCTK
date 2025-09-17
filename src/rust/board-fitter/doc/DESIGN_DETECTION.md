# Detection Pipeline Design

## Overview

This document details the design of the core detection pipeline in the board-fitter library. The pipeline transforms raw LiDAR point clouds into validated board detections through a series of specialized processing stages.

## Pipeline Architecture

### Stage Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    Detection Pipeline                         │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  Input: PointCloud<f64>                                       │
│    ↓                                                          │
│  1. Preprocessing & ROI Management                           │
│    ↓                                                          │
│  2. Plane Detection (RANSAC)                                 │
│    ↓                                                          │
│  3. Diamond Square Fitting                                   │
│    ↓                                                          │
│  4. Hole Detection (Hybrid)                                  │
│    ↓                                                          │
│  5. Coordinate Transformation                                │
│    ↓                                                          │
│  6. Pattern Matching & Validation                           │
│    ↓                                                          │
│  Output: Vec<BoardDetection>                                 │
│                                                               │
└─────────────────────────────────────────────────────────────┘
```

## Stage 1: Preprocessing & ROI Management

### Purpose
Reduce computational load and improve detection quality by focusing on relevant data.

### Design Components

#### Voxel Filtering
```rust
pub struct VoxelFilter {
    resolution: f64,  // Default: 5mm
    min_points_per_voxel: usize,  // Default: 1
}

impl VoxelFilter {
    pub fn downsample(&self, cloud: &PointCloud) -> PointCloud {
        // Grid-based downsampling preserving point distribution
        let mut voxel_map: HashMap<VoxelKey, Vec<usize>> = HashMap::new();

        for (idx, point) in cloud.points.iter().enumerate() {
            let key = self.compute_voxel_key(point);
            voxel_map.entry(key).or_default().push(idx);
        }

        // Select representative point from each voxel
        self.select_representatives(cloud, voxel_map)
    }
}
```

#### ROI Bounds
```rust
pub struct RoiBounds {
    pub center: Point3<f64>,
    pub half_extents: Vector3<f64>,
    pub orientation: Option<UnitQuaternion<f64>>,
}

impl RoiBounds {
    pub fn contains(&self, point: &Point3<f64>) -> bool {
        let local = self.to_local_coordinates(point);
        local.x.abs() <= self.half_extents.x &&
        local.y.abs() <= self.half_extents.y &&
        local.z.abs() <= self.half_extents.z
    }
}
```

### Adaptive Preprocessing
- Dynamic voxel resolution based on point density
- Automatic ROI expansion if too few points
- Statistical outlier removal for noisy data

## Stage 2: Plane Detection

### Purpose
Identify planar surfaces that could contain calibration boards.

### RANSAC Algorithm Design

```rust
pub struct RansacPlaneDetector {
    pub min_inliers: usize,          // Default: 100
    pub distance_threshold: f64,      // Default: 0.02m
    pub max_iterations: usize,        // Default: 1000
    pub probability: f64,             // Default: 0.99
    pub parallel_angle_threshold: f64, // Default: 10°
}
```

### Multi-Plane Detection Strategy

```rust
pub fn detect_planes(&self, points: &[Point3<f64>]) -> Vec<PlaneCandidate> {
    let mut remaining_points = points.to_vec();
    let mut planes = Vec::new();

    while remaining_points.len() > self.min_inliers {
        match self.fit_plane(&remaining_points) {
            Some(plane) => {
                // Remove inliers from remaining points
                remaining_points = self.remove_inliers(&remaining_points, &plane);

                // Merge with existing parallel planes
                if let Some(merged) = self.try_merge_parallel(&planes, &plane) {
                    *merged = self.merge_planes(merged, &plane);
                } else {
                    planes.push(plane);
                }
            }
            None => break,
        }
    }

    planes
}
```

### Plane Quality Metrics

```rust
pub struct PlaneQuality {
    pub inlier_ratio: f64,      // Inliers / total points
    pub planarity: f64,         // Eigenvalue ratio
    pub point_distribution: f64, // Spatial coverage
}
```

## Stage 3: Diamond Square Fitting

### Purpose
Extract diamond-oriented squares from planar point sets.

### Algorithm Pipeline

```
Planar Points
     ↓
Convex Hull Extraction
     ↓
PCA Analysis (2D projection)
     ↓
Rectangle Fitting
     ↓
45° Rotation Validation
     ↓
Diamond Square
```

### Convex Hull Processing

```rust
pub fn extract_convex_hull(points: &[Point2<f64>]) -> Vec<Point2<f64>> {
    // Graham scan algorithm for 2D convex hull
    let mut sorted = points.to_vec();
    sorted.sort_by(|a, b| {
        a.x.partial_cmp(&b.x).unwrap()
            .then(a.y.partial_cmp(&b.y).unwrap())
    });

    let mut hull = Vec::new();

    // Lower hull
    for point in &sorted {
        while hull.len() >= 2 &&
              !is_counter_clockwise(&hull[hull.len()-2], &hull[hull.len()-1], point) {
            hull.pop();
        }
        hull.push(*point);
    }

    // Upper hull (similar process)
    // ...

    hull
}
```

### PCA-Based Orientation

```rust
pub fn compute_square_orientation(points: &[Point2<f64>]) -> DiamondSquare {
    // Compute centroid
    let centroid = compute_centroid(points);

    // Build covariance matrix
    let cov = compute_covariance(points, &centroid);

    // Eigendecomposition for principal axes
    let eigen = cov.symmetric_eigen();
    let principal_axis = eigen.eigenvectors.column(0);

    // Compute rotation angle
    let angle = principal_axis.y.atan2(principal_axis.x);

    // Validate diamond orientation (45° ± 15°)
    if !is_diamond_oriented(angle) {
        return Err(NotDiamondOriented);
    }

    // Fit bounding box in principal coordinates
    fit_oriented_bbox(points, centroid, angle)
}
```

### Diamond Validation

```rust
fn is_diamond_oriented(angle: f64) -> bool {
    let angle_deg = angle.to_degrees().abs();
    // Check if angle is near 45° or 135°
    (angle_deg - 45.0).abs() < 15.0 ||
    (angle_deg - 135.0).abs() < 15.0
}
```

## Stage 4: Hole Detection

### Purpose
Detect circular holes within the diamond square region.

### Hybrid Detection Strategy

#### Method 1: Intensity-Based Detection

```rust
pub struct IntensityHoleDetector {
    pub min_intensity_drop: f32,     // Default: 50%
    pub gradient_threshold: f32,     // Default: 20
    pub grid_resolution: f64,        // Default: 5mm
}

impl IntensityHoleDetector {
    pub fn detect_holes(&self, points: &[LidarPoint]) -> Vec<DetectedHole> {
        // Build intensity grid
        let grid = self.build_intensity_grid(points);

        // Find low-intensity regions
        let dark_regions = self.find_dark_regions(&grid);

        // Fit circles to region boundaries
        dark_regions.into_iter()
            .filter_map(|region| self.fit_circle_to_region(region))
            .collect()
    }
}
```

#### Method 2: Geometric Detection

```rust
pub struct GeometricHoleDetector {
    pub min_hole_radius: f64,        // Default: 10mm
    pub max_hole_radius: f64,        // Default: 50mm
    pub density_threshold: f64,      // Points per area
}

impl GeometricHoleDetector {
    pub fn detect_holes(&self, points: &[Point3<f64>]) -> Vec<DetectedHole> {
        // Project to 2D plane
        let points_2d = project_to_plane(points);

        // Build occupancy grid
        let occupancy = self.build_occupancy_grid(&points_2d);

        // Find empty regions
        let empty_regions = self.find_empty_regions(&occupancy);

        // Validate circular shape
        empty_regions.into_iter()
            .filter_map(|region| self.validate_circular_region(region))
            .collect()
    }
}
```

### Hole Fusion and Ranking

```rust
pub fn fuse_hole_detections(
    intensity_holes: Vec<DetectedHole>,
    geometric_holes: Vec<DetectedHole>,
) -> Vec<DetectedHole> {
    let mut fused = Vec::new();
    let mut used_geometric = HashSet::new();

    // Match and fuse nearby detections
    for int_hole in intensity_holes {
        let matching = find_matching_hole(&int_hole, &geometric_holes);

        if let Some((idx, geo_hole)) = matching {
            used_geometric.insert(idx);
            fused.push(merge_holes(&int_hole, &geo_hole));
        } else {
            fused.push(int_hole);
        }
    }

    // Add unmatched geometric holes
    for (idx, geo_hole) in geometric_holes.iter().enumerate() {
        if !used_geometric.contains(&idx) {
            fused.push(geo_hole.clone());
        }
    }

    // Sort by confidence
    fused.sort_by(|a, b| b.confidence.partial_cmp(&a.confidence).unwrap());
    fused
}
```

## Stage 5: Coordinate Transformation

### Purpose
Transform detected features from sensor coordinates to board-centric coordinates.

### Transformation Pipeline

```rust
pub struct CoordinateTransformer {
    pub board_to_world: Isometry3<f64>,
    pub sensor_to_world: Isometry3<f64>,
}

impl CoordinateTransformer {
    pub fn to_board_coordinates(&self, point: &Point3<f64>) -> Point3<f64> {
        let world_point = self.sensor_to_world * point;
        self.board_to_world.inverse() * world_point
    }

    pub fn compute_board_pose(
        &self,
        square: &DiamondSquare,
        plane: &Plane,
    ) -> Isometry3<f64> {
        // Build coordinate frame
        let z_axis = plane.normal;
        let x_axis = square.compute_x_axis();
        let y_axis = z_axis.cross(&x_axis);

        let rotation = Rotation3::from_matrix_unchecked(
            Matrix3::from_columns(&[x_axis, y_axis, z_axis])
        );

        let translation = square.center;

        Isometry3::from_parts(translation.into(), rotation.into())
    }
}
```

## Stage 6: Pattern Matching & Validation

### Purpose
Validate detected features against expected board geometry.

### Validation Criteria

```rust
pub struct PatternValidator {
    pub expected_holes: Vec<Point2<f64>>,
    pub hole_radius: f64,
    pub tolerance: f64,
}

impl PatternValidator {
    pub fn validate(&self, detection: &BoardDetection) -> ValidationResult {
        let mut result = ValidationResult::default();

        // Check hole count
        result.hole_count_valid = detection.holes.len() == self.expected_holes.len();

        // Check hole positions
        let (matches, unmatched) = self.match_holes(&detection.holes);
        result.hole_match_ratio = matches.len() as f64 / self.expected_holes.len() as f64;

        // Check hole spacing
        result.spacing_error = self.compute_spacing_error(&matches);

        // Check hole sizes
        result.size_consistency = self.compute_size_consistency(&detection.holes);

        // Overall score
        result.confidence = self.compute_confidence(&result);

        result
    }
}
```

### Geometric Constraints

```rust
pub fn validate_board_geometry(detection: &BoardDetection) -> bool {
    // Check aspect ratio (should be square)
    let aspect_ratio = detection.width / detection.height;
    if (aspect_ratio - 1.0).abs() > 0.1 {
        return false;
    }

    // Check hole grid regularity
    let grid_score = compute_grid_regularity(&detection.holes);
    if grid_score < 0.8 {
        return false;
    }

    // Check co-planarity
    let planarity = compute_hole_planarity(&detection.holes);
    if planarity > 0.01 { // 1cm threshold
        return false;
    }

    true
}
```

## Performance Optimization

### Parallel Processing

```rust
pub struct ParallelDetector {
    thread_pool: ThreadPool,
    num_workers: usize,
}

impl ParallelDetector {
    pub fn detect_parallel(&self, planes: Vec<PlaneCandidate>) -> Vec<BoardDetection> {
        let (tx, rx) = channel();

        for plane in planes {
            let tx = tx.clone();
            self.thread_pool.execute(move || {
                let detection = process_plane(plane);
                tx.send(detection).unwrap();
            });
        }

        rx.iter().take(planes.len()).filter_map(|d| d).collect()
    }
}
```

### Early Termination

```rust
pub fn detect_with_early_termination(&self, cloud: &PointCloud) -> Option<BoardDetection> {
    for plane in self.detect_planes(cloud) {
        if let Some(square) = self.fit_diamond_square(&plane) {
            let holes = self.detect_holes(&square, &plane);

            // Early termination on high confidence
            if holes.len() >= self.min_holes {
                let detection = self.create_detection(square, holes);
                if detection.confidence > self.early_termination_threshold {
                    return Some(detection);
                }
            }
        }
    }

    None
}
```

## Error Recovery

### Partial Detection Handling

```rust
pub enum PartialDetection {
    PlaneOnly(Plane),
    SquareNoHoles(DiamondSquare),
    PartialHoles(DiamondSquare, Vec<DetectedHole>),
}

impl PartialDetection {
    pub fn try_complete(&self, additional_data: &PointCloud) -> Option<BoardDetection> {
        match self {
            PartialDetection::SquareNoHoles(square) => {
                // Try to detect holes with relaxed parameters
                let holes = detect_holes_relaxed(square, additional_data);
                if holes.len() >= MIN_HOLES_FOR_DETECTION {
                    Some(create_detection(square, holes))
                } else {
                    None
                }
            }
            // Handle other cases...
        }
    }
}
```

## Quality Metrics

### Detection Quality Assessment

```rust
pub struct DetectionQuality {
    pub geometric_score: f64,     // Shape regularity
    pub intensity_score: f64,     // Intensity consistency
    pub completeness_score: f64,  // Detected vs expected features
    pub confidence_score: f64,    // Overall confidence

    pub warnings: Vec<QualityWarning>,
}

pub enum QualityWarning {
    LowPointDensity { points_per_sqm: f64 },
    HighNoise { noise_level: f64 },
    PartialOcclusion { occluded_ratio: f64 },
    PoorLighting { intensity_variance: f64 },
}
```