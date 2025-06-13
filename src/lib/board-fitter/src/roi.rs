//! Region of Interest (ROI) management and adaptive preprocessing

use anyhow::Result;
use nalgebra::{Point3, Vector3};
use std::{
    collections::HashMap,
    time::{Duration, Instant},
};

use crate::{
    tracking::{TrackedBoard, TrackingStateInfo},
    types::{BoardId, BoundingBox, PointCloud, RegionOfInterest, RoiType},
};

/// Configuration for ROI management
#[derive(Debug, Clone)]
pub struct RoiConfig {
    /// Base ROI size for local tracking (meters)
    pub base_roi_size: Vector3<f64>,
    /// Expansion factor for tracking ROI based on velocity
    pub velocity_expansion_factor: f64,
    /// Expansion factor for uncertainty-based ROI padding
    pub uncertainty_expansion_factor: f64,
    /// Maximum allowed ROI size
    pub max_roi_size: Vector3<f64>,
    /// Minimum ROI size
    pub min_roi_size: Vector3<f64>,
    /// ROI expansion rate for lost boards (per second)
    pub lost_board_expansion_rate: f64,
    /// Maximum expansion for lost boards
    pub max_lost_board_expansion: f64,
    /// Workspace bounds for global search
    pub workspace_bounds: BoundingBox,
}

impl Default for RoiConfig {
    fn default() -> Self {
        Self {
            base_roi_size: Vector3::new(2.0, 2.0, 1.0), // 2m x 2m x 1m
            velocity_expansion_factor: 1.5,
            uncertainty_expansion_factor: 3.0, // 3-sigma bounds
            max_roi_size: Vector3::new(5.0, 5.0, 2.0),
            min_roi_size: Vector3::new(0.5, 0.5, 0.2),
            lost_board_expansion_rate: 1.0, // 1 m/s expansion
            max_lost_board_expansion: 3.0,  // 3m maximum expansion
            workspace_bounds: BoundingBox {
                min: Point3::new(-5.0, -5.0, 0.0),
                max: Point3::new(10.0, 5.0, 3.0),
            },
        }
    }
}

/// Adaptive ROI manager
pub struct RoiManager {
    config: RoiConfig,
    /// Cache of computed ROIs for performance
    roi_cache: HashMap<BoardId, (RegionOfInterest, Instant)>,
    /// Cache timeout
    cache_timeout: Duration,
}

impl RoiManager {
    /// Create a new ROI manager
    pub fn new(config: RoiConfig) -> Self {
        Self {
            config,
            roi_cache: HashMap::new(),
            cache_timeout: Duration::from_millis(100), // 100ms cache
        }
    }

    /// Create with default configuration
    pub fn default() -> Self {
        Self::new(RoiConfig::default())
    }

    /// Compute ROIs based on current tracking state
    pub fn compute_rois(
        &mut self,
        tracking_info: &TrackingStateInfo,
        tracked_boards: &[TrackedBoard],
        prediction_time: Duration,
    ) -> Result<Vec<RegionOfInterest>> {
        let now = Instant::now();
        self.cleanup_cache(now);

        match tracking_info.roi_mode {
            RoiType::GlobalSearch => Ok(vec![self.create_global_search_roi()]),
            RoiType::LocalTracking => {
                self.compute_local_tracking_rois(tracked_boards, prediction_time)
            }
            RoiType::ExpandingSearch => {
                self.compute_expanding_search_rois(tracked_boards, prediction_time)
            }
        }
    }

    /// Apply ROI filtering to point cloud
    pub fn filter_point_cloud(
        &self,
        point_cloud: &PointCloud,
        rois: &[RegionOfInterest],
    ) -> Result<Vec<(PointCloud, BoardId)>> {
        let mut filtered_clouds = Vec::new();

        for roi in rois {
            let filtered_points = self.extract_points_in_roi(point_cloud, &roi.bbox)?;

            if !filtered_points.is_empty() {
                let filtered_cloud = PointCloud {
                    points: filtered_points
                        .iter()
                        .map(|&i| point_cloud.points[i])
                        .collect(),
                    intensities: point_cloud.intensities.as_ref().map(|intensities| {
                        filtered_points.iter().map(|&i| intensities[i]).collect()
                    }),
                    colors: point_cloud
                        .colors
                        .as_ref()
                        .map(|colors| filtered_points.iter().map(|&i| colors[i]).collect()),
                    timestamp: point_cloud.timestamp,
                    frame_id: point_cloud.frame_id.clone(),
                };

                filtered_clouds.push((
                    filtered_cloud,
                    roi.board_id.unwrap_or_else(|| BoardId::nil()),
                ));
            }
        }

        Ok(filtered_clouds)
    }

    /// Create global search ROI covering the entire workspace
    fn create_global_search_roi(&self) -> RegionOfInterest {
        RegionOfInterest {
            bbox: self.config.workspace_bounds.clone(),
            priority: 1.0,
            board_id: None,
            roi_type: RoiType::GlobalSearch,
        }
    }

    /// Compute ROIs for local tracking mode
    fn compute_local_tracking_rois(
        &mut self,
        tracked_boards: &[TrackedBoard],
        prediction_time: Duration,
    ) -> Result<Vec<RegionOfInterest>> {
        let mut rois = Vec::new();

        for board in tracked_boards {
            if matches!(board.state, crate::tracking::TrackingState::Active) {
                let roi = self.compute_board_roi(board, prediction_time, RoiType::LocalTracking)?;
                rois.push(roi);
            }
        }

        Ok(rois)
    }

    /// Compute ROIs for expanding search mode
    fn compute_expanding_search_rois(
        &mut self,
        tracked_boards: &[TrackedBoard],
        prediction_time: Duration,
    ) -> Result<Vec<RegionOfInterest>> {
        let mut rois = Vec::new();

        for board in tracked_boards {
            if matches!(board.state, crate::tracking::TrackingState::Lost) {
                let roi = self.compute_expanding_roi(board, prediction_time)?;
                rois.push(roi);
            }
        }

        // If no lost boards, fall back to global search
        if rois.is_empty() {
            rois.push(self.create_global_search_roi());
        }

        Ok(rois)
    }

    /// Compute ROI for a specific tracked board
    fn compute_board_roi(
        &mut self,
        board: &TrackedBoard,
        prediction_time: Duration,
        roi_type: RoiType,
    ) -> Result<RegionOfInterest> {
        let now = Instant::now();

        // Check cache first
        if let Some((cached_roi, cache_time)) = self.roi_cache.get(&board.id) {
            if now.duration_since(*cache_time) < self.cache_timeout {
                return Ok(cached_roi.clone());
            }
        }

        // Predict board position
        let mut mutable_board = board.clone();
        let predicted_pose =
            mutable_board.predict(prediction_time, &crate::tracking::TrackingConfig::default());
        let center = Point3::from(predicted_pose.translation.vector);

        // Compute ROI size based on multiple factors
        let base_size = self.config.base_roi_size;
        let velocity_padding = self.compute_velocity_padding(board, prediction_time);
        let uncertainty_padding = self.compute_uncertainty_padding(board);

        let total_size = base_size + velocity_padding + uncertainty_padding;
        let clamped_size = self.clamp_roi_size(total_size);

        let bbox = BoundingBox::from_center(center, clamped_size);

        // Compute priority based on tracking confidence and history
        let priority = self.compute_roi_priority(board);

        let roi = RegionOfInterest {
            bbox,
            priority,
            board_id: Some(board.id),
            roi_type,
        };

        // Cache the result
        self.roi_cache.insert(board.id, (roi.clone(), now));

        Ok(roi)
    }

    /// Compute expanding ROI for lost boards
    fn compute_expanding_roi(
        &mut self,
        board: &TrackedBoard,
        prediction_time: Duration,
    ) -> Result<RegionOfInterest> {
        // Start with normal ROI computation
        let mut roi = self.compute_board_roi(board, prediction_time, RoiType::ExpandingSearch)?;

        // Add expansion based on time since last detection
        let time_since_detection = Instant::now().duration_since(board.last_detection_time);
        let expansion_factor = (time_since_detection.as_secs_f64()
            * self.config.lost_board_expansion_rate)
            .min(self.config.max_lost_board_expansion);

        let expansion = Vector3::new(expansion_factor, expansion_factor, expansion_factor / 2.0);
        let expanded_size = roi.bbox.size() + expansion;
        let clamped_size = self.clamp_roi_size(expanded_size);

        roi.bbox = BoundingBox::from_center(roi.bbox.center(), clamped_size);
        roi.priority *= 0.8; // Lower priority for lost boards

        Ok(roi)
    }

    /// Compute velocity-based padding for ROI
    fn compute_velocity_padding(
        &self,
        board: &TrackedBoard,
        prediction_time: Duration,
    ) -> Vector3<f64> {
        let linear_velocity = Vector3::new(board.velocity[0], board.velocity[1], board.velocity[2]);
        let velocity_magnitude = linear_velocity.norm();

        if velocity_magnitude > 0.001 {
            // Avoid division by very small numbers
            let predicted_displacement = velocity_magnitude * prediction_time.as_secs_f64();
            let padding = predicted_displacement * self.config.velocity_expansion_factor;
            Vector3::new(padding, padding, padding / 2.0)
        } else {
            Vector3::zeros()
        }
    }

    /// Compute uncertainty-based padding for ROI
    fn compute_uncertainty_padding(&self, board: &TrackedBoard) -> Vector3<f64> {
        // Extract position uncertainty (3x3 submatrix)
        let pos_uncertainty = board.uncertainty.fixed_view::<3, 3>(0, 0);

        Vector3::new(
            self.config.uncertainty_expansion_factor * pos_uncertainty[(0, 0)].sqrt(),
            self.config.uncertainty_expansion_factor * pos_uncertainty[(1, 1)].sqrt(),
            self.config.uncertainty_expansion_factor * pos_uncertainty[(2, 2)].sqrt(),
        )
    }

    /// Clamp ROI size to configured bounds
    fn clamp_roi_size(&self, size: Vector3<f64>) -> Vector3<f64> {
        Vector3::new(
            size.x
                .clamp(self.config.min_roi_size.x, self.config.max_roi_size.x),
            size.y
                .clamp(self.config.min_roi_size.y, self.config.max_roi_size.y),
            size.z
                .clamp(self.config.min_roi_size.z, self.config.max_roi_size.z),
        )
    }

    /// Compute ROI priority based on board properties
    fn compute_roi_priority(&self, board: &TrackedBoard) -> f64 {
        let confidence_factor = board.confidence.value();
        let history_factor = (board.detection_count as f64).ln() / 10.0; // Logarithmic history bonus
        let freshness_factor = 1.0 / (1.0 + board.consecutive_misses as f64);

        (confidence_factor + history_factor) * freshness_factor
    }

    /// Extract point indices that fall within the ROI
    fn extract_points_in_roi(
        &self,
        point_cloud: &PointCloud,
        bbox: &BoundingBox,
    ) -> Result<Vec<usize>> {
        let indices = (0..point_cloud.len())
            .filter(|&i| bbox.contains(&point_cloud.points[i]))
            .collect();

        Ok(indices)
    }

    /// Clean up expired cache entries
    fn cleanup_cache(&mut self, now: Instant) {
        self.roi_cache
            .retain(|_, (_, cache_time)| now.duration_since(*cache_time) < self.cache_timeout);
    }

    /// Update configuration
    pub fn update_config(&mut self, config: RoiConfig) {
        self.config = config;
        self.roi_cache.clear(); // Clear cache when config changes
    }

    /// Get current configuration
    pub fn config(&self) -> &RoiConfig {
        &self.config
    }
}

/// Adaptive preprocessing based on ROI context
pub struct AdaptivePreprocessor {
    /// Different preprocessing configurations for different ROI types
    configs: HashMap<RoiType, PreprocessingConfig>,
}

impl AdaptivePreprocessor {
    /// Create a new adaptive preprocessor
    pub fn new() -> Self {
        let mut configs = HashMap::new();

        // Global search: aggressive filtering to reduce computation
        configs.insert(
            RoiType::GlobalSearch,
            PreprocessingConfig {
                voxel_size: 0.05, // 5cm voxels
                outlier_removal_neighbors: 20,
                outlier_removal_std_dev: 2.0,
                enable_normal_estimation: false,
            },
        );

        // Local tracking: fine-grained processing for accuracy
        configs.insert(
            RoiType::LocalTracking,
            PreprocessingConfig {
                voxel_size: 0.01, // 1cm voxels
                outlier_removal_neighbors: 10,
                outlier_removal_std_dev: 1.0,
                enable_normal_estimation: true,
            },
        );

        // Expanding search: balanced processing
        configs.insert(
            RoiType::ExpandingSearch,
            PreprocessingConfig {
                voxel_size: 0.02, // 2cm voxels
                outlier_removal_neighbors: 15,
                outlier_removal_std_dev: 1.5,
                enable_normal_estimation: false,
            },
        );

        Self { configs }
    }

    /// Apply preprocessing to point cloud based on ROI type
    pub fn preprocess(&self, point_cloud: &PointCloud, roi_type: RoiType) -> Result<PointCloud> {
        let default_config = PreprocessingConfig::default();
        let config = self.configs.get(&roi_type).unwrap_or(&default_config);

        let mut processed_cloud = point_cloud.clone();

        // Apply voxel grid filtering
        if config.voxel_size > 0.0 {
            processed_cloud = self.apply_voxel_filter(&processed_cloud, config.voxel_size)?;
        }

        // Apply statistical outlier removal
        if config.outlier_removal_neighbors > 0 {
            processed_cloud = self.apply_outlier_removal(
                &processed_cloud,
                config.outlier_removal_neighbors,
                config.outlier_removal_std_dev,
            )?;
        }

        // Apply normal estimation if enabled
        if config.enable_normal_estimation {
            // Normal estimation not implemented yet - could use PCA on local neighborhoods
            // This would typically add normal vectors to the point cloud
        }

        Ok(processed_cloud)
    }

    /// Apply voxel grid downsampling
    fn apply_voxel_filter(&self, point_cloud: &PointCloud, voxel_size: f64) -> Result<PointCloud> {
        if voxel_size <= 0.0 || point_cloud.is_empty() {
            return Ok(point_cloud.clone());
        }

        // Create voxel grid hash map
        let mut voxel_map: HashMap<(i32, i32, i32), Vec<usize>> = HashMap::new();

        // Assign points to voxels
        for (idx, point) in point_cloud.points.iter().enumerate() {
            let voxel_x = (point.x / voxel_size).floor() as i32;
            let voxel_y = (point.y / voxel_size).floor() as i32;
            let voxel_z = (point.z / voxel_size).floor() as i32;

            voxel_map
                .entry((voxel_x, voxel_y, voxel_z))
                .or_insert_with(Vec::new)
                .push(idx);
        }

        // Downsample by taking centroid of each voxel
        let mut downsampled_points = Vec::new();
        let mut downsampled_intensities = point_cloud.intensities.as_ref().map(|_| Vec::new());
        let mut downsampled_colors = point_cloud.colors.as_ref().map(|_| Vec::new());

        for point_indices in voxel_map.values() {
            if point_indices.is_empty() {
                continue;
            }

            // Compute centroid
            let mut centroid = Point3::origin();
            for &idx in point_indices {
                centroid += point_cloud.points[idx].coords;
            }
            centroid /= point_indices.len() as f64;
            downsampled_points.push(centroid);

            // Average intensity if available
            if let (Some(intensities), Some(ref mut downsampled_int)) =
                (&point_cloud.intensities, &mut downsampled_intensities)
            {
                let avg_intensity = point_indices
                    .iter()
                    .map(|&idx| intensities[idx])
                    .sum::<f32>()
                    / point_indices.len() as f32;
                downsampled_int.push(avg_intensity);
            }

            // Average color if available
            if let (Some(colors), Some(ref mut downsampled_col)) =
                (&point_cloud.colors, &mut downsampled_colors)
            {
                let avg_r = point_indices.iter().map(|&idx| colors[idx][0]).sum::<u8>()
                    / point_indices.len() as u8;
                let avg_g = point_indices.iter().map(|&idx| colors[idx][1]).sum::<u8>()
                    / point_indices.len() as u8;
                let avg_b = point_indices.iter().map(|&idx| colors[idx][2]).sum::<u8>()
                    / point_indices.len() as u8;
                downsampled_col.push([avg_r, avg_g, avg_b]);
            }
        }

        Ok(PointCloud {
            points: downsampled_points,
            intensities: downsampled_intensities,
            colors: downsampled_colors,
            timestamp: point_cloud.timestamp,
            frame_id: point_cloud.frame_id.clone(),
        })
    }

    /// Apply statistical outlier removal
    fn apply_outlier_removal(
        &self,
        point_cloud: &PointCloud,
        neighbors: usize,
        std_dev_threshold: f64,
    ) -> Result<PointCloud> {
        if neighbors == 0 || point_cloud.points.len() < neighbors + 1 {
            return Ok(point_cloud.clone());
        }

        let mut mean_distances = Vec::new();

        // For each point, find k nearest neighbors and compute mean distance
        for (i, point) in point_cloud.points.iter().enumerate() {
            let mut distances = Vec::new();

            // Compute distances to all other points
            for (j, other_point) in point_cloud.points.iter().enumerate() {
                if i != j {
                    let distance = (point - other_point).norm();
                    distances.push(distance);
                }
            }

            // Sort distances and take k nearest neighbors
            distances.sort_by(|a, b| a.partial_cmp(b).unwrap());
            let k_nearest = distances.into_iter().take(neighbors).collect::<Vec<_>>();

            // Compute mean distance to k nearest neighbors
            let mean_distance = k_nearest.iter().sum::<f64>() / k_nearest.len() as f64;
            mean_distances.push(mean_distance);
        }

        // Compute global mean and standard deviation of mean distances
        let global_mean = mean_distances.iter().sum::<f64>() / mean_distances.len() as f64;
        let variance = mean_distances
            .iter()
            .map(|&d| (d - global_mean).powi(2))
            .sum::<f64>()
            / mean_distances.len() as f64;
        let std_dev = variance.sqrt();

        // Filter points based on statistical threshold
        let threshold = global_mean + std_dev_threshold * std_dev;
        let mut filtered_points = Vec::new();
        let mut filtered_intensities = point_cloud.intensities.as_ref().map(|_| Vec::new());
        let mut filtered_colors = point_cloud.colors.as_ref().map(|_| Vec::new());

        for (i, &mean_distance) in mean_distances.iter().enumerate() {
            if mean_distance <= threshold {
                filtered_points.push(point_cloud.points[i]);

                // Include intensity if available
                if let (Some(intensities), Some(ref mut filtered_int)) =
                    (&point_cloud.intensities, &mut filtered_intensities)
                {
                    filtered_int.push(intensities[i]);
                }

                // Include color if available
                if let (Some(colors), Some(ref mut filtered_col)) =
                    (&point_cloud.colors, &mut filtered_colors)
                {
                    filtered_col.push(colors[i]);
                }
            }
        }

        Ok(PointCloud {
            points: filtered_points,
            intensities: filtered_intensities,
            colors: filtered_colors,
            timestamp: point_cloud.timestamp,
            frame_id: point_cloud.frame_id.clone(),
        })
    }
}

/// Configuration for adaptive preprocessing
#[derive(Debug, Clone)]
pub struct PreprocessingConfig {
    /// Voxel size for downsampling (0.0 = disabled)
    pub voxel_size: f64,
    /// Number of neighbors for outlier removal (0 = disabled)
    pub outlier_removal_neighbors: usize,
    /// Standard deviation threshold for outlier removal
    pub outlier_removal_std_dev: f64,
    /// Enable normal vector estimation
    pub enable_normal_estimation: bool,
}

impl Default for PreprocessingConfig {
    fn default() -> Self {
        Self {
            voxel_size: 0.02,
            outlier_removal_neighbors: 10,
            outlier_removal_std_dev: 1.0,
            enable_normal_estimation: false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        tracking::{TrackedBoard, TrackingState},
        types::DetectionConfidence,
    };
    use nalgebra::{Isometry3, Point3, Vector3};

    fn create_test_tracked_board() -> TrackedBoard {
        let detection = crate::types::BoardDetection::new(
            Isometry3::identity(),
            DetectionConfidence::new(0.8),
            Vector3::new(1.0, 1.0, 0.02),
        );
        TrackedBoard::new(detection)
    }

    #[test]
    fn test_roi_config_default() {
        let config = RoiConfig::default();
        assert_eq!(config.base_roi_size, Vector3::new(2.0, 2.0, 1.0));
        assert_eq!(config.velocity_expansion_factor, 1.5);
    }

    #[test]
    fn test_roi_manager_creation() {
        let manager = RoiManager::default();
        assert_eq!(manager.roi_cache.len(), 0);
    }

    #[test]
    fn test_global_search_roi() {
        let manager = RoiManager::default();
        let roi = manager.create_global_search_roi();

        assert_eq!(roi.roi_type, RoiType::GlobalSearch);
        assert!(roi.board_id.is_none());
        assert_eq!(roi.priority, 1.0);
    }

    #[test]
    fn test_roi_size_clamping() {
        let manager = RoiManager::default();

        // Test oversized ROI gets clamped
        let oversized = Vector3::new(10.0, 10.0, 5.0);
        let clamped = manager.clamp_roi_size(oversized);
        assert!(clamped.x <= manager.config.max_roi_size.x);
        assert!(clamped.y <= manager.config.max_roi_size.y);
        assert!(clamped.z <= manager.config.max_roi_size.z);

        // Test undersized ROI gets clamped
        let undersized = Vector3::new(0.1, 0.1, 0.05);
        let clamped = manager.clamp_roi_size(undersized);
        assert!(clamped.x >= manager.config.min_roi_size.x);
        assert!(clamped.y >= manager.config.min_roi_size.y);
        assert!(clamped.z >= manager.config.min_roi_size.z);
    }

    #[test]
    fn test_roi_priority_computation() {
        let manager = RoiManager::default();
        let board = create_test_tracked_board();

        let priority = manager.compute_roi_priority(&board);
        assert!(priority > 0.0);
        assert!(priority <= 1.0); // Should be reasonable range
    }

    #[test]
    fn test_velocity_padding_computation() {
        let manager = RoiManager::default();
        let mut board = create_test_tracked_board();

        // Set some velocity
        board.velocity[0] = 1.0; // 1 m/s in x direction

        let prediction_time = Duration::from_secs(1);
        let padding = manager.compute_velocity_padding(&board, prediction_time);

        assert!(padding.x > 0.0); // Should have padding due to velocity
    }

    #[test]
    fn test_adaptive_preprocessor_creation() {
        let preprocessor = AdaptivePreprocessor::new();

        // Check that all ROI types have configurations
        assert!(preprocessor.configs.contains_key(&RoiType::GlobalSearch));
        assert!(preprocessor.configs.contains_key(&RoiType::LocalTracking));
        assert!(preprocessor.configs.contains_key(&RoiType::ExpandingSearch));
    }

    #[test]
    fn test_preprocessing_config_default() {
        let config = PreprocessingConfig::default();
        assert_eq!(config.voxel_size, 0.02);
        assert_eq!(config.outlier_removal_neighbors, 10);
        assert!(!config.enable_normal_estimation);
    }
}
