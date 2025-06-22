use crate::{
    calibration::{
        CalibrationSolver, CalibrationValidator, DefaultCalibrationSolver,
        DefaultCalibrationValidator,
    },
    detection::{DefaultDetectionSynchronizer, DetectionSynchronizer},
    types::{BoardDetection, TimestampedDetection},
};
use builtin_interfaces::msg::Time;
use eyre::Result;
use geometry_msgs::msg::TransformStamped;
use nalgebra::Isometry3;
use rclrs::{log_error, log_info, log_warn, ToLogParams};
use std::{
    sync::{Arc, Mutex},
    time::{Duration, SystemTime, UNIX_EPOCH},
};

/// Calibration state for tracking the calibration process
#[derive(Debug, Clone)]
pub struct CalibrationState {
    pub is_calibrated: bool,
    pub transform: Option<Isometry3<f64>>,
    pub quality_score: Option<f64>,
    pub last_calibration_time: Option<SystemTime>,
    pub successful_calibrations: u32,
    pub failed_calibrations: u32,
}

impl Default for CalibrationState {
    fn default() -> Self {
        Self {
            is_calibrated: false,
            transform: None,
            quality_score: None,
            last_calibration_time: None,
            successful_calibrations: 0,
            failed_calibrations: 0,
        }
    }
}

/// Trait for managing automatic calibration
pub trait CalibrationManager: Send + Sync {
    /// Add a detection from a LiDAR sensor
    fn add_detection(&self, detection: BoardDetection, timestamp: Time, lidar_id: u8);

    /// Check if calibration is available and get the transform
    fn get_current_calibration(&self) -> Option<(Isometry3<f64>, f64)>;

    /// Get calibration state for monitoring
    fn get_calibration_state(&self) -> CalibrationState;

    /// Reset calibration state
    fn reset_calibration(&self);

    /// Check if automatic calibration should be attempted
    fn should_attempt_calibration(&self) -> bool;

    /// Manually trigger calibration attempt
    fn trigger_calibration(&self) -> Result<Option<Isometry3<f64>>>;
}

/// Default implementation of CalibrationManager
pub struct DefaultCalibrationManager {
    synchronizer: Arc<Mutex<DefaultDetectionSynchronizer>>,
    solver: Arc<DefaultCalibrationSolver>,
    validator: Arc<DefaultCalibrationValidator>,
    state: Arc<Mutex<CalibrationState>>,
    config: CalibrationConfig,
}

/// Configuration for calibration manager
#[derive(Debug, Clone)]
pub struct CalibrationConfig {
    pub auto_calibrate: bool,
    pub min_detections_for_calibration: usize,
    pub calibration_timeout_seconds: u64,
    pub quality_threshold: f64,
    pub same_face_mode: bool,
    pub apply_bug_fix: bool,
    pub max_queue_size: usize,
    pub sync_tolerance_ms: u64,
}

impl Default for CalibrationConfig {
    fn default() -> Self {
        Self {
            auto_calibrate: true,
            min_detections_for_calibration: 10,
            calibration_timeout_seconds: 30,
            quality_threshold: 0.7,
            same_face_mode: true,
            apply_bug_fix: false,
            max_queue_size: 100,
            sync_tolerance_ms: 100,
        }
    }
}

impl DefaultCalibrationManager {
    pub fn new(config: CalibrationConfig) -> Self {
        let synchronizer = Arc::new(Mutex::new(DefaultDetectionSynchronizer::new(
            config.max_queue_size,
            config.sync_tolerance_ms,
        )));
        let solver = Arc::new(DefaultCalibrationSolver);
        let validator = Arc::new(DefaultCalibrationValidator::new());
        let state = Arc::new(Mutex::new(CalibrationState::default()));

        Self {
            synchronizer,
            solver,
            validator,
            state,
            config,
        }
    }

    /// Attempt automatic calibration using available detections
    fn attempt_auto_calibration(&self) -> Result<Option<Isometry3<f64>>> {
        if !self.config.auto_calibrate {
            return Ok(None);
        }

        let mut sync = self.synchronizer.lock().unwrap();

        // Clean up old detections
        sync.clear_old_detections(Duration::from_secs(self.config.calibration_timeout_seconds));

        // Check if we have enough detections
        let (q1_size, q2_size) = sync.get_queue_sizes();
        if q1_size < self.config.min_detections_for_calibration
            || q2_size < self.config.min_detections_for_calibration
        {
            return Ok(None);
        }

        // Try to find synchronized pairs and compute calibration
        if let Some((det1, det2)) = sync.find_synchronized_pair() {
            log_info!(
                "multi_wayside_node",
                "Found synchronized detection pair, attempting calibration"
            );

            match self.solver.compute_transform(
                &det1.detection,
                &det2.detection,
                self.config.same_face_mode,
            ) {
                Ok(transform) => {
                    // Validate the computed transform
                    let quality = self.validator.validate_transform(&transform);

                    if self.validator.is_acceptable(&quality) {
                        log_info!(
                            "multi_wayside_node",
                            "Calibration successful! Quality score: {:.3}, Translation: {:.3}m, Rotation: {:.3}°",
                            quality.confidence_score,
                            quality.translation_magnitude,
                            quality.rotation_angle.to_degrees()
                        );

                        // Update calibration state
                        let mut state = self.state.lock().unwrap();
                        state.is_calibrated = true;
                        state.transform = Some(transform);
                        state.quality_score = Some(quality.confidence_score);
                        state.last_calibration_time = Some(SystemTime::now());
                        state.successful_calibrations += 1;

                        Ok(Some(transform))
                    } else {
                        log_warn!(
                            "multi_wayside_node",
                            "Calibration rejected due to poor quality. Score: {:.3} (threshold: {:.3})",
                            quality.confidence_score,
                            self.config.quality_threshold
                        );

                        let mut state = self.state.lock().unwrap();
                        state.failed_calibrations += 1;

                        Ok(None)
                    }
                }
                Err(e) => {
                    log_error!("multi_wayside_node", "Transform computation failed: {}", e);

                    let mut state = self.state.lock().unwrap();
                    state.failed_calibrations += 1;

                    Err(e)
                }
            }
        } else {
            Ok(None)
        }
    }
}

impl CalibrationManager for DefaultCalibrationManager {
    fn add_detection(&self, detection: BoardDetection, timestamp: Time, lidar_id: u8) {
        {
            let mut sync = self.synchronizer.lock().unwrap();
            sync.add_detection(detection, timestamp, lidar_id);
        }

        // Attempt automatic calibration if enabled and conditions are met
        if self.should_attempt_calibration() {
            if let Err(e) = self.attempt_auto_calibration() {
                log_error!("multi_wayside_node", "Auto-calibration failed: {}", e);
            }
        }
    }

    fn get_current_calibration(&self) -> Option<(Isometry3<f64>, f64)> {
        let state = self.state.lock().unwrap();
        if state.is_calibrated {
            Some((state.transform.unwrap(), state.quality_score.unwrap_or(0.0)))
        } else {
            None
        }
    }

    fn get_calibration_state(&self) -> CalibrationState {
        self.state.lock().unwrap().clone()
    }

    fn reset_calibration(&self) {
        let mut state = self.state.lock().unwrap();
        *state = CalibrationState::default();

        let mut sync = self.synchronizer.lock().unwrap();
        sync.clear_old_detections(Duration::from_secs(0)); // Clear all detections

        log_info!("multi_wayside_node", "Calibration state reset");
    }

    fn should_attempt_calibration(&self) -> bool {
        if !self.config.auto_calibrate {
            return false;
        }

        let state = self.state.lock().unwrap();

        // Don't attempt if already calibrated (unless it's been a while)
        if state.is_calibrated {
            if let Some(last_calibration) = state.last_calibration_time {
                let elapsed = SystemTime::now()
                    .duration_since(last_calibration)
                    .unwrap_or_default();

                // Only re-calibrate if it's been more than 10 minutes
                return elapsed > Duration::from_secs(600);
            }
        }

        true
    }

    fn trigger_calibration(&self) -> Result<Option<Isometry3<f64>>> {
        log_info!("multi_wayside_node", "Manual calibration triggered");
        self.attempt_auto_calibration()
    }
}

/// Convert Isometry3 to ROS TransformStamped message
pub fn isometry_to_transform_stamped(
    transform: &Isometry3<f64>,
    frame_id: &str,
    child_frame_id: &str,
) -> TransformStamped {
    let now = SystemTime::now();
    let duration = now.duration_since(UNIX_EPOCH).unwrap_or_default();

    let stamp = Time {
        sec: duration.as_secs() as i32,
        nanosec: duration.subsec_nanos(),
    };

    let translation = geometry_msgs::msg::Vector3 {
        x: transform.translation.x,
        y: transform.translation.y,
        z: transform.translation.z,
    };

    let rotation = geometry_msgs::msg::Quaternion {
        x: transform.rotation.i,
        y: transform.rotation.j,
        z: transform.rotation.k,
        w: transform.rotation.w,
    };

    let transform_msg = geometry_msgs::msg::Transform {
        translation,
        rotation,
    };

    TransformStamped {
        header: std_msgs::msg::Header {
            stamp,
            frame_id: frame_id.to_string(),
        },
        child_frame_id: child_frame_id.to_string(),
        transform: transform_msg,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Translation3, UnitQuaternion};
    use std::time::SystemTime;

    fn create_test_detection() -> BoardDetection {
        BoardDetection {
            pose: Isometry3::identity(),
            confidence: 0.8,
            inlier_count: 100,
            timestamp: SystemTime::now(),
        }
    }

    fn create_test_time(sec: i32, nanosec: u32) -> Time {
        Time { sec, nanosec }
    }

    #[test]
    fn test_calibration_manager_creation() {
        let config = CalibrationConfig::default();
        let manager = DefaultCalibrationManager::new(config);

        let state = manager.get_calibration_state();
        assert!(!state.is_calibrated);
        assert_eq!(state.successful_calibrations, 0);
    }

    #[test]
    fn test_add_detections() {
        let config = CalibrationConfig {
            auto_calibrate: false, // Disable auto calibration for this test
            ..CalibrationConfig::default()
        };
        let manager = DefaultCalibrationManager::new(config);

        // Add detections from both LiDARs
        let detection1 = create_test_detection();
        let timestamp1 = create_test_time(1000, 0);
        manager.add_detection(detection1, timestamp1, 1);

        let detection2 = create_test_detection();
        let timestamp2 = create_test_time(1000, 50_000_000); // 50ms later
        manager.add_detection(detection2, timestamp2, 2);

        // State should still be uncalibrated since auto_calibrate is disabled
        let state = manager.get_calibration_state();
        assert!(!state.is_calibrated);
    }

    #[test]
    fn test_manual_calibration() {
        let config = CalibrationConfig {
            auto_calibrate: false,
            min_detections_for_calibration: 1, // Lower threshold for test
            ..CalibrationConfig::default()
        };
        let manager = DefaultCalibrationManager::new(config);

        // Add synchronized detections
        let detection1 = BoardDetection {
            pose: Isometry3::identity(),
            confidence: 0.9,
            inlier_count: 150,
            timestamp: SystemTime::now(),
        };
        let timestamp1 = create_test_time(1000, 0);
        manager.add_detection(detection1, timestamp1, 1);

        let detection2 = BoardDetection {
            pose: Isometry3::from_parts(
                Translation3::new(1.0, 0.0, 0.0),
                UnitQuaternion::identity(),
            ),
            confidence: 0.9,
            inlier_count: 150,
            timestamp: SystemTime::now(),
        };
        let timestamp2 = create_test_time(1000, 50_000_000);
        manager.add_detection(detection2, timestamp2, 2);

        // Manually trigger calibration
        let result = manager.trigger_calibration();
        assert!(result.is_ok());

        // Check if calibration was successful
        if result.unwrap().is_some() {
            let state = manager.get_calibration_state();
            assert!(state.is_calibrated);
            assert_eq!(state.successful_calibrations, 1);
        }
    }

    #[test]
    fn test_isometry_to_transform_stamped() {
        let transform =
            Isometry3::from_parts(Translation3::new(1.0, 2.0, 3.0), UnitQuaternion::identity());

        let transform_msg = isometry_to_transform_stamped(&transform, "lidar1", "lidar2");

        assert_eq!(transform_msg.header.frame_id, "lidar1");
        assert_eq!(transform_msg.child_frame_id, "lidar2");
        assert_eq!(transform_msg.transform.translation.x, 1.0);
        assert_eq!(transform_msg.transform.translation.y, 2.0);
        assert_eq!(transform_msg.transform.translation.z, 3.0);
    }

    #[test]
    fn test_calibration_reset() {
        let config = CalibrationConfig::default();
        let manager = DefaultCalibrationManager::new(config);

        // Manually set calibrated state
        {
            let mut state = manager.state.lock().unwrap();
            state.is_calibrated = true;
            state.successful_calibrations = 5;
        }

        // Reset calibration
        manager.reset_calibration();

        // Check that state is reset
        let state = manager.get_calibration_state();
        assert!(!state.is_calibrated);
        assert_eq!(state.successful_calibrations, 0);
    }
}
