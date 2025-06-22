use crate::calibration::{isometry_to_transform_stamped, CalibrationManager};
use builtin_interfaces::msg::Time;
use eyre::Result;
use geometry_msgs::msg::TransformStamped;
use nalgebra::Isometry3;
use rclrs::{log_info, Node, Publisher, ToLogParams};
use std::{
    sync::{Arc, Mutex},
    time::{SystemTime, UNIX_EPOCH},
};

/// Trait for broadcasting TF2 transforms
pub trait TfBroadcaster: Send + Sync {
    /// Broadcast a single transform
    fn broadcast_transform(&self, transform: &TransformStamped) -> Result<()>;

    /// Broadcast calibration transform between LiDAR frames
    fn broadcast_calibration_transform(
        &self,
        transform: &Isometry3<f64>,
        frame_id: &str,
        child_frame_id: &str,
    ) -> Result<()>;

    /// Check if transform is being broadcasted
    fn is_broadcasting(&self) -> bool;

    /// Stop broadcasting transforms
    fn stop_broadcasting(&self);
}

/// Default implementation using ROS 2 TF2
pub struct DefaultTfBroadcaster {
    tf_publisher: Arc<Publisher<TransformStamped>>,
    is_active: Arc<Mutex<bool>>,
}

impl DefaultTfBroadcaster {
    pub fn new(node: &Node) -> Result<Self> {
        // Create calibration transform publisher
        let tf_publisher = Arc::new(
            node.create_publisher::<TransformStamped>("/calibration_transform")
                .map_err(|e| {
                    eyre::eyre!("Failed to create calibration transform publisher: {}", e)
                })?,
        );

        let is_active = Arc::new(Mutex::new(false));

        log_info!("multi_wayside_node", "TF2 broadcaster initialized");

        Ok(Self {
            tf_publisher,
            is_active,
        })
    }
}

impl TfBroadcaster for DefaultTfBroadcaster {
    fn broadcast_transform(&self, transform: &TransformStamped) -> Result<()> {
        self.tf_publisher
            .publish(transform)
            .map_err(|e| eyre::eyre!("Failed to publish calibration transform: {}", e))?;

        Ok(())
    }

    fn broadcast_calibration_transform(
        &self,
        transform: &Isometry3<f64>,
        frame_id: &str,
        child_frame_id: &str,
    ) -> Result<()> {
        // Convert Isometry3 to TransformStamped
        let transform_stamped = isometry_to_transform_stamped(transform, frame_id, child_frame_id);

        // Broadcast the transform
        self.broadcast_transform(&transform_stamped)?;

        // Mark as active
        {
            let mut active = self.is_active.lock().unwrap();
            *active = true;
        }

        log_info!(
            "multi_wayside_node",
            "Broadcasting calibration transform from {} to {} on /calibration_transform",
            frame_id,
            child_frame_id
        );

        Ok(())
    }

    fn is_broadcasting(&self) -> bool {
        *self.is_active.lock().unwrap()
    }

    fn stop_broadcasting(&self) {
        let mut active = self.is_active.lock().unwrap();
        *active = false;
        log_info!("multi_wayside_node", "TF2 broadcasting stopped");
    }
}

/// Enhanced calibration manager with TF2 broadcasting
pub struct CalibrationManagerWithTf<T: TfBroadcaster> {
    manager: Arc<crate::calibration::DefaultCalibrationManager>,
    tf_broadcaster: Arc<T>,
    frame_config: TfFrameConfig,
}

/// Configuration for TF frame names
#[derive(Debug, Clone)]
pub struct TfFrameConfig {
    pub lidar1_frame: String,
    pub lidar2_frame: String,
    pub broadcast_interval_ms: u64,
}

impl Default for TfFrameConfig {
    fn default() -> Self {
        Self {
            lidar1_frame: "lidar1".to_string(),
            lidar2_frame: "lidar2".to_string(),
            broadcast_interval_ms: 100, // 10 Hz
        }
    }
}

impl<T: TfBroadcaster> CalibrationManagerWithTf<T> {
    pub fn new(
        manager: Arc<crate::calibration::DefaultCalibrationManager>,
        tf_broadcaster: Arc<T>,
        frame_config: TfFrameConfig,
    ) -> Self {
        Self {
            manager,
            tf_broadcaster,
            frame_config,
        }
    }

    /// Get the underlying calibration manager
    pub fn get_manager(&self) -> &Arc<crate::calibration::DefaultCalibrationManager> {
        &self.manager
    }

    /// Broadcast current calibration if available
    pub fn broadcast_current_calibration(&self) -> Result<Option<()>> {
        if let Some((transform, _quality)) = self.manager.get_current_calibration() {
            self.tf_broadcaster.broadcast_calibration_transform(
                &transform,
                &self.frame_config.lidar1_frame,
                &self.frame_config.lidar2_frame,
            )?;
            Ok(Some(()))
        } else {
            Ok(None)
        }
    }

    /// Check if calibration is being broadcasted
    pub fn is_broadcasting_calibration(&self) -> bool {
        self.tf_broadcaster.is_broadcasting() && self.manager.get_current_calibration().is_some()
    }

    /// Update frame configuration
    pub fn update_frame_config(&mut self, config: TfFrameConfig) {
        self.frame_config = config;
    }

    /// Get frame configuration
    pub fn get_frame_config(&self) -> &TfFrameConfig {
        &self.frame_config
    }
}

/// Utility function to create current timestamp
pub fn current_ros_time() -> Time {
    let now = SystemTime::now();
    let duration = now.duration_since(UNIX_EPOCH).unwrap_or_default();

    Time {
        sec: duration.as_secs() as i32,
        nanosec: duration.subsec_nanos(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Translation3, UnitQuaternion};
    use std::sync::atomic::{AtomicBool, Ordering};

    // Mock TF broadcaster for testing
    struct MockTfBroadcaster {
        broadcast_called: AtomicBool,
        is_active: AtomicBool,
    }

    impl MockTfBroadcaster {
        fn new() -> Self {
            Self {
                broadcast_called: AtomicBool::new(false),
                is_active: AtomicBool::new(false),
            }
        }

        fn was_broadcast_called(&self) -> bool {
            self.broadcast_called.load(Ordering::Relaxed)
        }
    }

    impl TfBroadcaster for MockTfBroadcaster {
        fn broadcast_transform(&self, _transform: &TransformStamped) -> Result<()> {
            self.broadcast_called.store(true, Ordering::Relaxed);
            Ok(())
        }

        fn broadcast_calibration_transform(
            &self,
            _transform: &Isometry3<f64>,
            _frame_id: &str,
            _child_frame_id: &str,
        ) -> Result<()> {
            self.broadcast_called.store(true, Ordering::Relaxed);
            self.is_active.store(true, Ordering::Relaxed);
            Ok(())
        }

        fn is_broadcasting(&self) -> bool {
            self.is_active.load(Ordering::Relaxed)
        }

        fn stop_broadcasting(&self) {
            self.is_active.store(false, Ordering::Relaxed);
        }
    }

    #[test]
    fn test_tf_frame_config_default() {
        let config = TfFrameConfig::default();
        assert_eq!(config.lidar1_frame, "lidar1");
        assert_eq!(config.lidar2_frame, "lidar2");
        assert_eq!(config.broadcast_interval_ms, 100);
    }

    #[test]
    fn test_current_ros_time() {
        let time = current_ros_time();
        assert!(time.sec > 0); // Should be a valid timestamp
    }

    #[test]
    fn test_calibration_manager_with_tf() {
        let calibration_config = crate::calibration::CalibrationConfig::default();
        let manager = Arc::new(crate::calibration::DefaultCalibrationManager::new(
            calibration_config,
        ));
        let tf_broadcaster = Arc::new(MockTfBroadcaster::new());
        let frame_config = TfFrameConfig::default();

        let manager_with_tf =
            CalibrationManagerWithTf::new(manager, tf_broadcaster.clone(), frame_config);

        // Initially should not be broadcasting
        assert!(!manager_with_tf.is_broadcasting_calibration());

        // Test frame config access
        assert_eq!(manager_with_tf.get_frame_config().lidar1_frame, "lidar1");
    }

    #[test]
    fn test_mock_tf_broadcaster() {
        let broadcaster = MockTfBroadcaster::new();

        // Initially not broadcasting
        assert!(!broadcaster.is_broadcasting());
        assert!(!broadcaster.was_broadcast_called());

        // Test transform broadcasting
        let transform =
            Isometry3::from_parts(Translation3::new(1.0, 0.0, 0.0), UnitQuaternion::identity());

        let result = broadcaster.broadcast_calibration_transform(&transform, "lidar1", "lidar2");

        assert!(result.is_ok());
        assert!(broadcaster.was_broadcast_called());
        assert!(broadcaster.is_broadcasting());

        // Test stop broadcasting
        broadcaster.stop_broadcasting();
        assert!(!broadcaster.is_broadcasting());
    }
}
