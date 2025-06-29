//! Temporal alignment for smooth tracking between frames
//!
//! This module provides frame-to-frame ICP alignment for stable board tracking
//! using VGICP for efficient processing of consecutive detections.

use super::{
    register_vgicp, GaussianVoxelMap, GaussianVoxelMapConfig, IcpRefinement, IcpStageConfig,
    PointCloud, RefinementResult, RegistrationSettings, SmallGicpConvergenceCriteria,
};
use crate::types::DetectionError;
use anyhow::Result;
use nalgebra::{Isometry3, Point3, Vector3};

/// Temporal tracking state
#[derive(Debug, Clone)]
pub struct TemporalTrackingState {
    /// Previous frame's voxel map for efficient matching
    pub previous_voxelmap: Option<GaussianVoxelMap>,
    /// Previous frame's point cloud
    pub previous_cloud: Option<Vec<Point3<f64>>>,
    /// Motion prediction from Kalman filter
    pub motion_prediction: Option<Isometry3<f64>>,
}

impl IcpRefinement {
    /// Refine board pose using temporal coherence
    ///
    /// Uses VGICP to efficiently align current detection with previous frame,
    /// providing smooth tracking and reducing jitter.
    pub fn align_temporal(
        &self,
        current_points: &[Point3<f64>],
        tracking_state: &TemporalTrackingState,
        initial_guess: Option<&Isometry3<f64>>,
        config: Option<&IcpStageConfig>,
    ) -> Result<RefinementResult> {
        let stage_config = config.unwrap_or(&self.config.temporal_alignment);

        if !stage_config.enabled {
            return Ok(RefinementResult {
                transformation: initial_guess.cloned().unwrap_or_else(Isometry3::identity),
                fitness: 1.0,
                num_inliers: current_points.len() as i32,
                covariance: None,
                converged: true,
                iterations: 0,
            });
        }

        // Need previous frame data
        let previous_voxelmap =
            tracking_state
                .previous_voxelmap
                .as_ref()
                .ok_or(DetectionError::InsufficientData(
                    "No previous voxelmap".to_string(),
                ))?;

        // Create current point cloud
        let current = PointCloud::from_points(current_points.to_vec());

        // Use motion prediction if available, otherwise use provided guess
        let initial_transform = tracking_state
            .motion_prediction
            .as_ref()
            .or(initial_guess)
            .cloned()
            .unwrap_or_else(Isometry3::identity);

        // Create settings for VGICP
        let settings = RegistrationSettings {
            registration_type: self.convert_registration_type(&stage_config.registration_type),
            num_threads: self.config.num_threads,
            max_iterations: stage_config.max_iterations,
            convergence_criteria: SmallGicpConvergenceCriteria {
                rotation_epsilon: stage_config.convergence_criteria.rotation_epsilon,
                translation_epsilon: stage_config.convergence_criteria.translation_epsilon,
            },
            initial_guess: Some(initial_transform),
        };

        // Perform VGICP registration
        let result = register_vgicp(previous_voxelmap, &current, &settings)?;

        // Convert result
        Ok(self.convert_registration_result(result))
    }

    /// Update temporal tracking state with new detection
    pub fn update_temporal_state(
        &self,
        state: &mut TemporalTrackingState,
        points: &[Point3<f64>],
        voxel_resolution: f64,
    ) -> Result<()> {
        // Create point cloud
        let cloud = PointCloud::from_points(points.to_vec());

        // Create voxel map for next frame
        let voxel_config = GaussianVoxelMapConfig {
            voxel_resolution,
            num_threads: self.config.num_threads,
        };

        let voxelmap = GaussianVoxelMap::new(&cloud, &voxel_config)?;

        // Update state
        state.previous_cloud = Some(cloud.points);
        state.previous_voxelmap = Some(voxelmap);

        Ok(())
    }
}

/// Apply temporal smoothing to transformation
pub fn smooth_transformation(
    current: &Isometry3<f64>,
    previous: &Isometry3<f64>,
    smoothing_factor: f64,
) -> Isometry3<f64> {
    // Simple exponential smoothing
    // For rotation, we use quaternion slerp
    let current_rot = current.rotation.quaternion();
    let previous_rot = previous.rotation.quaternion();
    let smoothed_rot = previous_rot.lerp(current_rot, 1.0 - smoothing_factor);

    // For translation, linear interpolation
    let current_trans = current.translation.vector;
    let previous_trans = previous.translation.vector;
    let smoothed_trans =
        previous_trans * smoothing_factor + current_trans * (1.0 - smoothing_factor);

    Isometry3::from_parts(
        smoothed_trans.into(),
        nalgebra::Unit::new_normalize(smoothed_rot),
    )
}

/// Compute motion prediction for next frame
pub fn predict_motion(history: &[Isometry3<f64>], window_size: usize) -> Option<Isometry3<f64>> {
    if history.len() < 2 {
        return None;
    }

    // Simple velocity-based prediction
    let start_idx = history.len().saturating_sub(window_size);
    let recent_history = &history[start_idx..];

    if recent_history.len() < 2 {
        return None;
    }

    // Compute average velocity
    let mut total_translation = Vector3::zeros();
    let mut total_rotation = Vector3::zeros(); // Axis-angle representation

    for i in 1..recent_history.len() {
        let prev = &recent_history[i - 1];
        let curr = &recent_history[i];

        // Translation velocity
        let trans_vel = curr.translation.vector - prev.translation.vector;
        total_translation += trans_vel;

        // Rotation velocity (using axis-angle)
        let relative_rot = prev.rotation.inverse() * curr.rotation;
        let (axis, angle) = relative_rot
            .axis_angle()
            .unwrap_or((Vector3::z_axis(), 0.0));
        total_rotation += axis.into_inner() * angle;
    }

    let n = (recent_history.len() - 1) as f64;
    let avg_trans_vel = total_translation / n;
    let avg_rot_vel = total_rotation / n;

    // Predict next pose
    let last_pose = recent_history.last().unwrap();
    let predicted_trans = last_pose.translation.vector + avg_trans_vel;

    let rot_angle = avg_rot_vel.norm();
    let predicted_rot = if rot_angle > 1e-6 {
        let rot_axis = avg_rot_vel / rot_angle;
        last_pose.rotation
            * nalgebra::UnitQuaternion::from_axis_angle(
                &nalgebra::Unit::new_normalize(rot_axis),
                rot_angle,
            )
    } else {
        last_pose.rotation
    };

    Some(Isometry3::from_parts(predicted_trans.into(), predicted_rot))
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_smooth_transformation() {
        let prev = Isometry3::translation(0.0, 0.0, 0.0);
        let curr = Isometry3::translation(1.0, 1.0, 1.0);

        let smoothed = smooth_transformation(&curr, &prev, 0.5);

        assert_relative_eq!(smoothed.translation.vector.x, 0.5, epsilon = 1e-6);
        assert_relative_eq!(smoothed.translation.vector.y, 0.5, epsilon = 1e-6);
        assert_relative_eq!(smoothed.translation.vector.z, 0.5, epsilon = 1e-6);
    }

    #[test]
    fn test_predict_motion() {
        let history = vec![
            Isometry3::translation(0.0, 0.0, 0.0),
            Isometry3::translation(1.0, 0.0, 0.0),
            Isometry3::translation(2.0, 0.0, 0.0),
        ];

        let prediction = predict_motion(&history, 3).unwrap();

        // Should predict next position at x=3.0
        assert_relative_eq!(prediction.translation.vector.x, 3.0, epsilon = 1e-6);
        assert_relative_eq!(prediction.translation.vector.y, 0.0, epsilon = 1e-6);
        assert_relative_eq!(prediction.translation.vector.z, 0.0, epsilon = 1e-6);
    }
}
