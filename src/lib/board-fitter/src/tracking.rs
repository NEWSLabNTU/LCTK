//! Board tracking and temporal consistency management

use anyhow::Result;
use nalgebra::{Isometry3, Matrix6, Point3, Vector3, Vector6};
use std::{
    collections::HashMap,
    time::{Duration, Instant},
};

use crate::{
    debug::{stages, AlgorithmStats, DebugContext, DebugData, StageMetrics},
    refinement::{temporal_alignment::TemporalTrackingState, IcpRefinement},
    types::{BoardDetection, BoardId, BoundingBox, DetectionConfidence, RoiType},
};

/// Configuration for board tracking
#[derive(Debug, Clone)]
pub struct TrackingConfig {
    /// Maximum distance for associating detections with tracks (meters)
    pub max_association_distance: f64,
    /// Time after which a track is considered lost (seconds)
    pub track_timeout: Duration,
    /// Minimum confidence for initializing new tracks
    pub min_init_confidence: f64,
    /// Minimum confidence for maintaining existing tracks
    pub min_track_confidence: f64,
    /// Maximum number of consecutive misses before track removal
    pub max_consecutive_misses: u32,
    /// Enable motion prediction using Kalman filtering
    pub enable_prediction: bool,
    /// Process noise for motion model
    pub process_noise: f64,
    /// Measurement noise for observations
    pub measurement_noise: f64,
}

impl Default for TrackingConfig {
    fn default() -> Self {
        Self {
            max_association_distance: 0.5, // 50cm
            track_timeout: Duration::from_secs(2),
            min_init_confidence: 0.7,
            min_track_confidence: 0.5,
            max_consecutive_misses: 5,
            enable_prediction: true,
            process_noise: 0.01,
            measurement_noise: 0.05,
        }
    }
}

/// State of a tracked board
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrackingState {
    /// Board currently visible and detected
    Active,
    /// Board position predicted from motion model
    Predicted,
    /// Board lost, searching in expanded region
    Lost,
    /// Track marked for removal
    Terminated,
}

/// Tracked board with motion history
#[derive(Debug, Clone)]
pub struct TrackedBoard {
    /// Unique identifier
    pub id: BoardId,
    /// Current 6DOF pose
    pub pose: Isometry3<f64>,
    /// Linear and angular velocity [vx, vy, vz, wx, wy, wz]
    pub velocity: Vector6<f64>,
    /// Pose uncertainty covariance matrix
    pub uncertainty: Matrix6<f64>,
    /// Current tracking state
    pub state: TrackingState,
    /// Confidence in the current pose estimate
    pub confidence: DetectionConfidence,
    /// Time of last detection update
    pub last_detection_time: Instant,
    /// Time when track was first created
    pub creation_time: Instant,
    /// Number of consecutive detection misses
    pub consecutive_misses: u32,
    /// Total number of detections for this track
    pub detection_count: usize,
    /// Board dimensions
    pub dimensions: Vector3<f64>,
    /// Motion prediction filter (Kalman filter state)
    pub filter_state: Option<KalmanFilterState>,
    /// ICP temporal tracking state
    pub temporal_state: Option<TemporalTrackingState>,
}

impl TrackedBoard {
    /// Create a new tracked board from detection
    pub fn new(detection: BoardDetection) -> Self {
        let now = Instant::now();
        Self {
            id: detection.id,
            pose: detection.pose,
            velocity: Vector6::zeros(),
            uncertainty: Matrix6::identity() * 0.1, // Initial uncertainty
            state: TrackingState::Active,
            confidence: detection.confidence,
            last_detection_time: now,
            creation_time: now,
            consecutive_misses: 0,
            detection_count: 1,
            dimensions: detection.dimensions,
            filter_state: None,
            temporal_state: None,
        }
    }

    /// Update track with new detection
    pub fn update(&mut self, detection: &BoardDetection, config: &TrackingConfig) {
        self.update_with_points(detection, config, None);
    }

    /// Update track with new detection and optional point cloud for temporal ICP
    pub fn update_with_points(
        &mut self,
        detection: &BoardDetection,
        config: &TrackingConfig,
        board_points: Option<&[Point3<f64>]>,
    ) {
        let now = Instant::now();
        let dt = now.duration_since(self.last_detection_time).as_secs_f64();

        // Update pose and confidence
        self.pose = detection.pose;
        self.confidence = detection.confidence;
        self.last_detection_time = now;
        self.consecutive_misses = 0;
        self.detection_count += 1;
        self.state = TrackingState::Active;

        // Update velocity estimation
        if dt > 0.001 {
            // Avoid division by very small numbers
            self.update_velocity(detection, dt);
        }

        // Update Kalman filter if enabled
        if config.enable_prediction {
            self.update_filter(detection, dt, config);
        }

        // Update temporal ICP state if points are provided
        if let Some(_points) = board_points {
            if self.temporal_state.is_none() {
                self.temporal_state = Some(TemporalTrackingState {
                    previous_voxelmap: None,
                    previous_cloud: None,
                    motion_prediction: None,
                });
            }
        }
    }

    /// Predict current pose based on motion model
    pub fn predict(&mut self, dt: Duration, config: &TrackingConfig) -> Isometry3<f64> {
        let dt_secs = dt.as_secs_f64();

        if config.enable_prediction && dt_secs > 0.0 {
            // Use Kalman filter prediction if available
            if let Some(ref mut filter) = self.filter_state {
                return filter.predict(dt_secs, config);
            }
        }

        // Simple linear motion prediction
        let linear_vel = Vector3::new(self.velocity[0], self.velocity[1], self.velocity[2]);
        let predicted_translation = self.pose.translation.vector + linear_vel * dt_secs;

        // For angular velocity, we'd need more complex rotation prediction
        // For now, assume constant orientation
        Isometry3::from_parts(predicted_translation.into(), self.pose.rotation)
    }

    /// Mark track as missed (no associated detection)
    pub fn mark_missed(&mut self, config: &TrackingConfig) {
        self.consecutive_misses += 1;

        self.state = if self.consecutive_misses >= config.max_consecutive_misses {
            TrackingState::Lost
        } else {
            TrackingState::Predicted
        };

        // Increase uncertainty over time
        self.uncertainty *= 1.1;

        // Decrease confidence
        let confidence_decay = 0.9_f64.powi(self.consecutive_misses as i32);
        self.confidence = DetectionConfidence::new(self.confidence.value() * confidence_decay);
    }

    /// Check if track should be terminated
    pub fn should_terminate(&self, config: &TrackingConfig) -> bool {
        let time_since_detection = Instant::now().duration_since(self.last_detection_time);

        time_since_detection > config.track_timeout
            || self.confidence.value() < config.min_track_confidence
            || self.state == TrackingState::Terminated
    }

    /// Get predicted bounding box for ROI generation
    pub fn predicted_bbox(
        &self,
        prediction_time: Duration,
        config: &TrackingConfig,
    ) -> BoundingBox {
        let mut mutable_self = self.clone();
        let predicted_pose = mutable_self.predict(prediction_time, config);
        let center = Point3::from(predicted_pose.translation.vector);

        // Add uncertainty padding
        let uncertainty_padding = self.compute_uncertainty_padding();
        let size = self.dimensions + uncertainty_padding;

        BoundingBox::from_center(center, size)
    }

    /// Update velocity estimation using finite differences
    fn update_velocity(&mut self, detection: &BoardDetection, dt: f64) {
        let position_diff = detection.pose.translation.vector - self.pose.translation.vector;
        let linear_velocity = position_diff / dt;

        // Simple velocity smoothing (exponential moving average)
        let alpha = 0.3; // Smoothing factor
        self.velocity[0] = alpha * linear_velocity.x + (1.0 - alpha) * self.velocity[0];
        self.velocity[1] = alpha * linear_velocity.y + (1.0 - alpha) * self.velocity[1];
        self.velocity[2] = alpha * linear_velocity.z + (1.0 - alpha) * self.velocity[2];

        // Angular velocity estimation using rotation difference
        let rotation_diff = detection.pose.rotation * self.pose.rotation.inverse();
        if let Some(axis_angle) = rotation_diff.axis_angle() {
            let angular_velocity = axis_angle.1 / dt; // Radians per second

            // Smooth angular velocity (simple 1D for rotation around z-axis)
            self.velocity[3] = alpha * angular_velocity + (1.0 - alpha) * self.velocity[3];
        }
    }

    /// Update Kalman filter with new measurement
    fn update_filter(&mut self, detection: &BoardDetection, dt: f64, config: &TrackingConfig) {
        if self.filter_state.is_none() {
            self.filter_state = Some(KalmanFilterState::new(detection.pose, config));
        }

        if let Some(ref mut filter) = self.filter_state {
            filter.update(detection.pose, dt, config);
            self.pose = filter.get_pose();
            self.uncertainty = filter.get_covariance();
        }
    }

    /// Compute uncertainty padding for bounding boxes
    fn compute_uncertainty_padding(&self) -> Vector3<f64> {
        // Extract position uncertainty (3x3 submatrix)
        let pos_uncertainty = self.uncertainty.fixed_view::<3, 3>(0, 0);

        // Use 2-sigma bounds for padding
        Vector3::new(
            2.0 * pos_uncertainty[(0, 0)].sqrt(),
            2.0 * pos_uncertainty[(1, 1)].sqrt(),
            2.0 * pos_uncertainty[(2, 2)].sqrt(),
        )
    }

    /// Get the age of this track in seconds
    pub fn age(&self) -> f64 {
        self.creation_time.elapsed().as_secs_f64()
    }

    /// Get time since last detection in seconds
    pub fn time_since_last_detection(&self) -> f64 {
        self.last_detection_time.elapsed().as_secs_f64()
    }

    /// Check if this track should be considered lost
    pub fn is_lost(&self, config: &TrackingConfig) -> bool {
        self.consecutive_misses >= config.max_consecutive_misses
            || self.time_since_last_detection() > config.track_timeout.as_secs_f64()
    }

    /// Get confidence decay based on consecutive misses
    pub fn effective_confidence(&self) -> DetectionConfidence {
        let decay_factor = 0.8_f64.powi(self.consecutive_misses as i32);
        DetectionConfidence::new(self.confidence.value() * decay_factor)
    }

    /// Get linear velocity magnitude
    pub fn speed(&self) -> f64 {
        Vector3::new(self.velocity[0], self.velocity[1], self.velocity[2]).norm()
    }
}

/// Simple Kalman filter state for board tracking
#[derive(Debug, Clone)]
pub struct KalmanFilterState {
    /// State vector [x, y, z, vx, vy, vz] (position + velocity)
    state: Vector6<f64>,
    /// State covariance matrix
    covariance: Matrix6<f64>,
}

impl KalmanFilterState {
    /// Initialize Kalman filter
    pub fn new(initial_pose: Isometry3<f64>, config: &TrackingConfig) -> Self {
        let position = initial_pose.translation.vector;
        let state = Vector6::new(position.x, position.y, position.z, 0.0, 0.0, 0.0);
        let covariance = Matrix6::identity() * config.measurement_noise;

        Self { state, covariance }
    }

    /// Predict next state
    pub fn predict(&mut self, dt: f64, config: &TrackingConfig) -> Isometry3<f64> {
        // Simple constant velocity model
        // x_{k+1} = x_k + v_k * dt
        // v_{k+1} = v_k

        let f = self.state_transition_matrix(dt);
        self.state = f * self.state;

        // Update covariance: P = F * P * F^T + Q
        let q = self.process_noise_matrix(dt, config.process_noise);
        self.covariance = f * self.covariance * f.transpose() + q;

        self.get_pose()
    }

    /// Update with measurement
    pub fn update(&mut self, measurement_pose: Isometry3<f64>, _dt: f64, config: &TrackingConfig) {
        // Simple position update - just update position states directly
        let measured_position = measurement_pose.translation.vector;

        // Update position states with measurement
        self.state[0] = measured_position.x;
        self.state[1] = measured_position.y;
        self.state[2] = measured_position.z;

        // Reduce uncertainty after measurement
        self.covariance *= 0.9;

        // Add measurement noise back
        let noise = config.measurement_noise;
        self.covariance[(0, 0)] += noise;
        self.covariance[(1, 1)] += noise;
        self.covariance[(2, 2)] += noise;
    }

    /// Get current pose estimate
    pub fn get_pose(&self) -> Isometry3<f64> {
        let position = Vector3::new(self.state[0], self.state[1], self.state[2]);
        Isometry3::from_parts(position.into(), nalgebra::UnitQuaternion::identity())
    }

    /// Get current covariance estimate
    pub fn get_covariance(&self) -> Matrix6<f64> {
        self.covariance
    }

    /// State transition matrix for constant velocity model
    fn state_transition_matrix(&self, dt: f64) -> Matrix6<f64> {
        let mut f = Matrix6::identity();
        f[(0, 3)] = dt; // x += vx * dt
        f[(1, 4)] = dt; // y += vy * dt
        f[(2, 5)] = dt; // z += vz * dt
        f
    }

    /// Measurement matrix (observe position only)
    fn _measurement_matrix(&self) -> nalgebra::Matrix3x6<f64> {
        let mut h = nalgebra::Matrix3x6::zeros();
        h[(0, 0)] = 1.0; // Observe x
        h[(1, 1)] = 1.0; // Observe y
        h[(2, 2)] = 1.0; // Observe z
        h
    }

    /// Process noise matrix
    fn process_noise_matrix(&self, dt: f64, noise: f64) -> Matrix6<f64> {
        let dt2 = dt * dt;
        let dt3 = dt2 * dt;
        let dt4 = dt3 * dt;

        let mut q = Matrix6::zeros();

        // Position-position covariance
        q[(0, 0)] = dt4 / 4.0 * noise;
        q[(1, 1)] = dt4 / 4.0 * noise;
        q[(2, 2)] = dt4 / 4.0 * noise;

        // Position-velocity covariance
        q[(0, 3)] = dt3 / 2.0 * noise;
        q[(1, 4)] = dt3 / 2.0 * noise;
        q[(2, 5)] = dt3 / 2.0 * noise;
        q[(3, 0)] = dt3 / 2.0 * noise;
        q[(4, 1)] = dt3 / 2.0 * noise;
        q[(5, 2)] = dt3 / 2.0 * noise;

        // Velocity-velocity covariance
        q[(3, 3)] = dt2 * noise;
        q[(4, 4)] = dt2 * noise;
        q[(5, 5)] = dt2 * noise;

        q
    }
}

/// Board tracker managing multiple tracked boards
pub struct BoardTracker {
    /// Active tracked boards
    tracked_boards: HashMap<BoardId, TrackedBoard>,
    /// Tracking configuration
    config: TrackingConfig,
    /// Current ROI mode
    roi_mode: RoiType,
    /// ICP refinement for temporal alignment
    icp_refiner: Option<IcpRefinement>,
}

impl Default for BoardTracker {
    fn default() -> Self {
        Self::new(TrackingConfig::default())
    }
}

impl BoardTracker {
    /// Create a new board tracker
    pub fn new(config: TrackingConfig) -> Self {
        Self {
            tracked_boards: HashMap::new(),
            config,
            roi_mode: RoiType::GlobalSearch,
            icp_refiner: None,
        }
    }

    /// Create a new board tracker with ICP refinement
    pub fn new_with_icp(config: TrackingConfig, icp_refiner: IcpRefinement) -> Self {
        Self {
            tracked_boards: HashMap::new(),
            config,
            roi_mode: RoiType::GlobalSearch,
            icp_refiner: Some(icp_refiner),
        }
    }

    /// Update tracker with new detections
    pub fn update(&mut self, detections: Vec<BoardDetection>) -> Result<Vec<TrackedBoard>> {
        self.update_with_debug(detections, None)
    }

    /// Update tracker with new detections and optional debug context
    pub fn update_with_debug(
        &mut self,
        detections: Vec<BoardDetection>,
        mut debug_ctx: Option<&mut DebugContext>,
    ) -> Result<Vec<TrackedBoard>> {
        let start_time = Instant::now();
        let now = Instant::now();

        if let Some(ref mut ctx) = debug_ctx {
            ctx.start_stage(stages::BOARD_TRACKING);

            // Emit input detection data
            let debug_data = DebugData::DetectionResult {
                detections: detections.clone(),
                confidence_scores: detections.iter().map(|d| d.confidence.value()).collect(),
                metadata: {
                    let mut metadata = HashMap::new();
                    metadata.insert("input_detections".to_string(), detections.len().to_string());
                    metadata.insert(
                        "existing_tracks".to_string(),
                        self.tracked_boards.len().to_string(),
                    );
                    metadata
                },
            };
            ctx.emit_data(stages::BOARD_TRACKING, &debug_data);
        }

        let initial_track_count = self.tracked_boards.len();

        // Predict current positions for all tracks
        self.predict_all_tracks(now);

        if let Some(ref mut ctx) = debug_ctx {
            let mut metadata = HashMap::new();
            metadata.insert("stage".to_string(), "prediction".to_string());
            metadata.insert(
                "predicted_tracks".to_string(),
                self.tracked_boards.len().to_string(),
            );

            let debug_data = DebugData::Generic {
                data: {
                    let mut data = HashMap::new();
                    data.insert(
                        "predicted_tracks_count".to_string(),
                        self.tracked_boards.len().into(),
                    );
                    data
                },
            };
            ctx.emit_data(stages::BOARD_TRACKING, &debug_data);
        }

        // Associate detections with existing tracks
        let associations = self.associate_detections(&detections);

        if let Some(ref mut ctx) = debug_ctx {
            let mut metadata = HashMap::new();
            metadata.insert("stage".to_string(), "association".to_string());
            metadata.insert(
                "matches".to_string(),
                associations.matches.len().to_string(),
            );
            metadata.insert(
                "unmatched_tracks".to_string(),
                associations.unmatched_tracks.len().to_string(),
            );
            metadata.insert(
                "unmatched_detections".to_string(),
                associations.unmatched_detections.len().to_string(),
            );

            let debug_data = DebugData::Generic {
                data: {
                    let mut data = HashMap::new();
                    data.insert(
                        "matches_count".to_string(),
                        associations.matches.len().into(),
                    );
                    data.insert(
                        "unmatched_tracks_count".to_string(),
                        associations.unmatched_tracks.len().into(),
                    );
                    data.insert(
                        "unmatched_detections_count".to_string(),
                        associations.unmatched_detections.len().into(),
                    );
                    data
                },
            };
            ctx.emit_data(stages::BOARD_TRACKING, &debug_data);
        }

        // Update associated tracks
        let mut updated_tracks = 0;
        for (track_id, detection_idx) in associations.matches {
            if let Some(track) = self.tracked_boards.get_mut(&track_id) {
                let detection = &detections[detection_idx];

                // Apply ICP temporal refinement if available
                let refined_detection = if let Some(ref icp_refiner) = self.icp_refiner {
                    Self::refine_detection_with_temporal_icp(track, detection, icp_refiner)
                        .unwrap_or_else(|_| detection.clone())
                } else {
                    detection.clone()
                };

                track.update(&refined_detection, &self.config);
                updated_tracks += 1;
            }
        }

        // Mark unassociated tracks as missed
        let mut missed_tracks = 0;
        for track_id in associations.unmatched_tracks {
            if let Some(track) = self.tracked_boards.get_mut(&track_id) {
                track.mark_missed(&self.config);
                missed_tracks += 1;
            }
        }

        // Initialize new tracks for unassociated detections
        let mut new_tracks = 0;
        for detection_idx in associations.unmatched_detections {
            let detection = &detections[detection_idx];
            if detection
                .confidence
                .above_threshold(self.config.min_init_confidence)
            {
                let track = TrackedBoard::new(detection.clone());
                self.tracked_boards.insert(track.id, track);
                new_tracks += 1;
            }
        }

        // Remove terminated tracks
        let tracks_before_removal = self.tracked_boards.len();
        self.remove_terminated_tracks();
        let removed_tracks = tracks_before_removal - self.tracked_boards.len();

        // Update ROI mode
        let old_roi_mode = self.roi_mode;
        self.update_roi_mode();
        let roi_mode_changed = old_roi_mode != self.roi_mode;

        let final_tracks: Vec<TrackedBoard> = self.tracked_boards.values().cloned().collect();
        let duration = start_time.elapsed();

        if let Some(ref mut ctx) = debug_ctx {
            // Emit final tracking state
            let state_info = self.get_tracking_state();
            let debug_data = DebugData::Generic {
                data: {
                    let mut data = HashMap::new();
                    data.insert("total_tracks".to_string(), state_info.total_tracks.into());
                    data.insert("active_tracks".to_string(), state_info.active_tracks.into());
                    data.insert(
                        "predicted_tracks".to_string(),
                        state_info.predicted_tracks.into(),
                    );
                    data.insert("lost_tracks".to_string(), state_info.lost_tracks.into());
                    data.insert(
                        "roi_mode".to_string(),
                        format!("{:?}", state_info.roi_mode).into(),
                    );
                    data.insert("roi_mode_changed".to_string(), roi_mode_changed.into());
                    data
                },
            };
            ctx.emit_data(stages::BOARD_TRACKING, &debug_data);

            // Emit metrics
            let metrics = StageMetrics::new(detections.len(), final_tracks.len(), duration);
            ctx.emit_metrics(stages::BOARD_TRACKING, &metrics);

            // Emit algorithm statistics
            let mut algo_stats = AlgorithmStats::new("Board_Tracking", 1, !final_tracks.is_empty());
            algo_stats.add_stat("initial_tracks", initial_track_count as f64);
            algo_stats.add_stat("input_detections", detections.len() as f64);
            algo_stats.add_stat("updated_tracks", updated_tracks as f64);
            algo_stats.add_stat("missed_tracks", missed_tracks as f64);
            algo_stats.add_stat("new_tracks", new_tracks as f64);
            algo_stats.add_stat("removed_tracks", removed_tracks as f64);
            algo_stats.add_stat("final_tracks", final_tracks.len() as f64);
            algo_stats.add_stat("roi_mode_changed", if roi_mode_changed { 1.0 } else { 0.0 });
            ctx.emit_algorithm_stats(stages::BOARD_TRACKING, &algo_stats);

            ctx.end_stage(stages::BOARD_TRACKING);
        }

        Ok(final_tracks)
    }

    /// Get current tracking state
    pub fn get_tracking_state(&self) -> TrackingStateInfo {
        let active_tracks = self
            .tracked_boards
            .values()
            .filter(|t| t.state == TrackingState::Active)
            .count();

        let predicted_tracks = self
            .tracked_boards
            .values()
            .filter(|t| t.state == TrackingState::Predicted)
            .count();

        let lost_tracks = self
            .tracked_boards
            .values()
            .filter(|t| t.state == TrackingState::Lost)
            .count();

        TrackingStateInfo {
            total_tracks: self.tracked_boards.len(),
            active_tracks,
            predicted_tracks,
            lost_tracks,
            roi_mode: self.roi_mode,
        }
    }

    /// Get ROIs for next frame processing
    pub fn get_rois(
        &self,
        prediction_time: Duration,
    ) -> Vec<(BoundingBox, RoiType, Option<BoardId>)> {
        match self.roi_mode {
            RoiType::GlobalSearch => {
                // Return workspace-wide ROI
                vec![(self.get_workspace_roi(), RoiType::GlobalSearch, None)]
            }
            RoiType::LocalTracking => {
                // Return ROIs around tracked boards
                self.tracked_boards
                    .values()
                    .filter(|t| t.state == TrackingState::Active)
                    .map(|t| {
                        (
                            t.predicted_bbox(prediction_time, &self.config),
                            RoiType::LocalTracking,
                            Some(t.id),
                        )
                    })
                    .collect()
            }
            RoiType::ExpandingSearch => {
                // Return expanded ROIs around lost boards
                self.tracked_boards
                    .values()
                    .filter(|t| t.state == TrackingState::Lost)
                    .map(|t| {
                        let mut bbox = t.predicted_bbox(prediction_time, &self.config);
                        // Expand the bounding box for lost tracks
                        let expansion = Vector3::new(1.0, 1.0, 0.5);
                        bbox = BoundingBox::from_center(bbox.center(), bbox.size() + expansion);
                        (bbox, RoiType::ExpandingSearch, Some(t.id))
                    })
                    .collect()
            }
        }
    }

    /// Predict all track positions to current time
    fn predict_all_tracks(&mut self, current_time: Instant) {
        for track in self.tracked_boards.values_mut() {
            if track.last_detection_time < current_time {
                let dt = current_time.duration_since(track.last_detection_time);
                track.predict(dt, &self.config);
            }
        }
    }

    /// Associate detections with existing tracks using Hungarian algorithm (simplified)
    fn associate_detections(&self, detections: &[BoardDetection]) -> AssociationResult {
        let mut associations = HashMap::new();
        let mut unmatched_tracks = Vec::new();
        let mut unmatched_detections: Vec<usize> = (0..detections.len()).collect();

        if self.tracked_boards.is_empty() {
            // No existing tracks, all detections are unmatched
            return AssociationResult {
                matches: Vec::new(),
                unmatched_tracks,
                unmatched_detections,
            };
        }

        if detections.is_empty() {
            // No detections, all tracks are unmatched
            unmatched_tracks = self.tracked_boards.keys().copied().collect();
            unmatched_detections.clear();
            return AssociationResult {
                matches: Vec::new(),
                unmatched_tracks,
                unmatched_detections,
            };
        }

        // Compute cost matrix for Hungarian algorithm
        let track_ids: Vec<BoardId> = self.tracked_boards.keys().copied().collect();
        let cost_matrix = self.compute_association_costs(&track_ids, detections);

        // Solve assignment problem using simplified Hungarian algorithm
        let assignments = self.solve_assignment_problem(&cost_matrix);

        // Process assignments
        for (track_idx, detection_idx) in assignments {
            if track_idx < track_ids.len() && detection_idx < detections.len() {
                let track_id = track_ids[track_idx];
                let cost = cost_matrix[track_idx][detection_idx];

                // Only accept assignment if cost is below threshold
                if cost < self.config.max_association_distance {
                    associations.insert(track_id, detection_idx);
                    // Remove from unmatched
                    unmatched_detections.retain(|&idx| idx != detection_idx);
                } else {
                    unmatched_tracks.push(track_id);
                }
            }
        }

        // Add tracks that weren't assigned
        for &track_id in &track_ids {
            if !associations.contains_key(&track_id) {
                unmatched_tracks.push(track_id);
            }
        }

        AssociationResult {
            matches: associations.into_iter().collect(),
            unmatched_tracks,
            unmatched_detections,
        }
    }

    /// Remove tracks that should be terminated
    fn remove_terminated_tracks(&mut self) {
        self.tracked_boards
            .retain(|_, track| !track.should_terminate(&self.config));
    }

    /// Update ROI mode based on tracking state
    fn update_roi_mode(&mut self) {
        let active_tracks = self
            .tracked_boards
            .values()
            .filter(|t| t.state == TrackingState::Active)
            .count();

        let lost_tracks = self
            .tracked_boards
            .values()
            .filter(|t| t.state == TrackingState::Lost)
            .count();

        self.roi_mode = match (active_tracks, lost_tracks) {
            (0, 0) => RoiType::GlobalSearch,
            (_, 0) => RoiType::LocalTracking,
            _ => RoiType::ExpandingSearch,
        };
    }

    /// Get workspace-wide ROI for global search
    fn get_workspace_roi(&self) -> BoundingBox {
        // Default workspace bounds - should be configurable
        BoundingBox {
            min: Point3::new(-5.0, -5.0, 0.0),
            max: Point3::new(10.0, 5.0, 3.0),
        }
    }

    // Helper methods for data association

    /// Compute cost matrix for track-detection association
    fn compute_association_costs(
        &self,
        track_ids: &[BoardId],
        detections: &[BoardDetection],
    ) -> Vec<Vec<f64>> {
        let mut cost_matrix = Vec::new();

        for &track_id in track_ids {
            let mut row = Vec::new();
            if let Some(track) = self.tracked_boards.get(&track_id) {
                for detection in detections {
                    let cost = self.compute_association_cost(track, detection);
                    row.push(cost);
                }
            }
            cost_matrix.push(row);
        }

        cost_matrix
    }

    /// Compute association cost between a track and detection
    fn compute_association_cost(&self, track: &TrackedBoard, detection: &BoardDetection) -> f64 {
        // Primary cost: Euclidean distance between predicted and detected positions
        let predicted_pos = if let Some(ref filter_state) = track.filter_state {
            filter_state.position()
        } else {
            // If no filter state, use current pose position
            track.pose.translation.vector
        };
        let detected_pos = detection.pose.translation.vector;
        let position_distance = (predicted_pos - detected_pos).norm();

        // Secondary cost: Orientation difference
        let predicted_orientation = if let Some(ref filter_state) = track.filter_state {
            filter_state.orientation()
        } else {
            // If no filter state, extract orientation from current pose
            track
                .pose
                .rotation
                .axis_angle()
                .map_or(0.0, |(_, angle)| angle)
        };
        let detected_orientation = detection
            .pose
            .rotation
            .axis_angle()
            .map_or(0.0, |(_, angle)| angle);
        let orientation_diff = (predicted_orientation - detected_orientation).abs();
        let normalized_orientation_diff = (orientation_diff % (2.0 * std::f64::consts::PI))
            .min(2.0 * std::f64::consts::PI - orientation_diff);

        // Combine costs with weights
        let position_weight = 1.0;
        let orientation_weight = 0.3;

        position_weight * position_distance + orientation_weight * normalized_orientation_diff
    }

    /// Simplified Hungarian algorithm for assignment problem
    fn solve_assignment_problem(&self, cost_matrix: &[Vec<f64>]) -> Vec<(usize, usize)> {
        if cost_matrix.is_empty() || cost_matrix[0].is_empty() {
            return Vec::new();
        }

        let n_tracks = cost_matrix.len();
        let n_detections = cost_matrix[0].len();

        // For simplicity, use greedy assignment instead of full Hungarian algorithm
        // This is suboptimal but much simpler to implement
        let mut assignments = Vec::new();
        let mut used_tracks = vec![false; n_tracks];
        let mut used_detections = vec![false; n_detections];

        // Create sorted list of all possible assignments by cost
        let mut candidates = Vec::new();
        for (i, track_costs) in cost_matrix.iter().enumerate().take(n_tracks) {
            for (j, &cost) in track_costs.iter().enumerate().take(n_detections) {
                candidates.push((cost, i, j));
            }
        }
        candidates.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());

        // Greedily assign lowest cost assignments first
        for (cost, track_idx, detection_idx) in candidates {
            if !used_tracks[track_idx]
                && !used_detections[detection_idx]
                && cost < self.config.max_association_distance
            {
                assignments.push((track_idx, detection_idx));
                used_tracks[track_idx] = true;
                used_detections[detection_idx] = true;
            }
        }

        assignments
    }

    /// Refine detection using temporal ICP alignment
    fn refine_detection_with_temporal_icp(
        track: &TrackedBoard,
        detection: &BoardDetection,
        refiner: &IcpRefinement,
    ) -> Result<BoardDetection> {
        use crate::refinement::temporal_alignment;

        // Check if track has temporal state, otherwise initialize
        if track.temporal_state.is_none() {
            return Ok(detection.clone());
        }

        let temporal_state = track.temporal_state.as_ref().unwrap();

        // Extract board region points (using supporting points if available)
        let board_points: Vec<Point3<f64>> = if !detection.supporting_points.is_empty() {
            detection
                .supporting_points
                .iter()
                .map(|&idx| Point3::new(idx as f64, 0.0, 0.0)) // Placeholder - would need actual points
                .collect()
        } else {
            // Generate points from board dimensions
            let half_x = detection.dimensions.x / 2.0;
            let half_y = detection.dimensions.y / 2.0;
            vec![
                Point3::new(-half_x, -half_y, 0.0),
                Point3::new(half_x, -half_y, 0.0),
                Point3::new(half_x, half_y, 0.0),
                Point3::new(-half_x, half_y, 0.0),
            ]
        };

        // Transform points to world coordinates
        let world_points: Vec<Point3<f64>> =
            board_points.iter().map(|p| detection.pose * p).collect();

        // Apply temporal ICP alignment
        let initial_guess = Some(&detection.pose);
        let refinement =
            refiner.align_temporal(&world_points, temporal_state, initial_guess, None)?;

        // Apply smoothing if refinement was successful
        let refined_pose = if refinement.converged {
            temporal_alignment::smooth_transformation(
                &refinement.transformation,
                &track.pose,
                0.3, // Smoothing factor
            )
        } else {
            detection.pose
        };

        // Create refined detection
        let mut refined = detection.clone();
        refined.pose = refined_pose;
        refined.confidence =
            DetectionConfidence::new(detection.confidence.score() * refinement.fitness);

        Ok(refined)
    }
}

// Additional helper methods for KalmanFilterState
impl KalmanFilterState {
    /// Get current position estimate
    fn position(&self) -> nalgebra::Vector3<f64> {
        self.state.fixed_rows::<3>(0).into()
    }

    /// Get current orientation estimate (simplified as single angle)
    fn orientation(&self) -> f64 {
        // For simplicity, extract a single orientation angle
        // In a full implementation, this would handle quaternions properly
        self.state[3] // Assuming orientation is stored in state[3]
    }
}

/// Result of detection-to-track association
#[derive(Debug)]
struct AssociationResult {
    /// Matched pairs: (track_id, detection_index)
    matches: Vec<(BoardId, usize)>,
    /// Unmatched track IDs
    unmatched_tracks: Vec<BoardId>,
    /// Unmatched detection indices
    unmatched_detections: Vec<usize>,
}

/// Information about current tracking state
#[derive(Debug, Clone)]
pub struct TrackingStateInfo {
    pub total_tracks: usize,
    pub active_tracks: usize,
    pub predicted_tracks: usize,
    pub lost_tracks: usize,
    pub roi_mode: RoiType,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::DetectionConfidence;
    use nalgebra::{Isometry3, Vector3};

    fn create_test_detection() -> BoardDetection {
        BoardDetection::new(
            Isometry3::identity(),
            DetectionConfidence::new(0.8),
            Vector3::new(1.0, 1.0, 0.02),
        )
    }

    #[test]
    fn test_tracked_board_creation() {
        let detection = create_test_detection();
        let track = TrackedBoard::new(detection);

        assert_eq!(track.state, TrackingState::Active);
        assert_eq!(track.consecutive_misses, 0);
        assert_eq!(track.detection_count, 1);
    }

    #[test]
    fn test_track_missed_updates() {
        let detection = create_test_detection();
        let mut track = TrackedBoard::new(detection);
        let config = TrackingConfig::default();

        // Mark as missed multiple times
        for i in 1..=3 {
            track.mark_missed(&config);
            assert_eq!(track.consecutive_misses, i);
            assert_eq!(track.state, TrackingState::Predicted);
        }

        // Mark as missed enough times to be lost
        for _ in 4..=config.max_consecutive_misses {
            track.mark_missed(&config);
        }
        assert_eq!(track.state, TrackingState::Lost);
    }

    #[test]
    fn test_kalman_filter_initialization() {
        let pose = Isometry3::identity();
        let config = TrackingConfig::default();
        let filter = KalmanFilterState::new(pose, &config);

        assert_eq!(filter.state[0], 0.0); // Initial x position
        assert_eq!(filter.state[1], 0.0); // Initial y position
        assert_eq!(filter.state[2], 0.0); // Initial z position
    }

    #[test]
    fn test_board_tracker_initialization() {
        let tracker = BoardTracker::default();
        assert_eq!(tracker.tracked_boards.len(), 0);
        assert_eq!(tracker.roi_mode, RoiType::GlobalSearch);
    }

    #[test]
    fn test_tracking_state_info() {
        let tracker = BoardTracker::default();
        let state_info = tracker.get_tracking_state();

        assert_eq!(state_info.total_tracks, 0);
        assert_eq!(state_info.active_tracks, 0);
        assert_eq!(state_info.roi_mode, RoiType::GlobalSearch);
    }
}
