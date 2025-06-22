use crate::types::RoiBounds;
use eyre::Result;
use rclrs::{log_info, Node, ToLogParams};
use std::{collections::HashMap, path::Path, sync::Arc};

/// Configuration parameters for the multi-wayside node
#[derive(Debug, Clone)]
pub struct MultiWaysideConfig {
    // File paths
    pub board_config_file: String,
    pub detector_config_file: String,
    pub aruco_pattern_file: String,

    // Detection parameters
    pub max_queue_size: usize,
    pub sync_tolerance_ms: u64,
    pub same_face_mode: bool,
    pub apply_bug_fix: bool,

    // ROI parameters
    pub roi_box_size_x: f64,
    pub roi_box_size_y: f64,
    pub roi_box_size_z: f64,
    pub roi_box_position_x: f64,
    pub roi_box_position_y: f64,
    pub roi_box_position_z: f64,

    // Filter parameters
    pub min_range: f32,
    pub max_range: f32,
}

impl MultiWaysideConfig {
    /// Create default configuration
    pub fn default() -> Self {
        Self {
            board_config_file: "config/hollow_board.yaml".to_string(),
            detector_config_file: "config/detector.yaml".to_string(),
            aruco_pattern_file: "config/aruco_pattern.json5".to_string(),
            max_queue_size: 100,
            sync_tolerance_ms: 100,
            same_face_mode: true,
            apply_bug_fix: false,
            roi_box_size_x: 4.0,
            roi_box_size_y: 4.0,
            roi_box_size_z: 2.0,
            roi_box_position_x: 2.0,
            roi_box_position_y: 0.0,
            roi_box_position_z: 0.0,
            min_range: 0.5,
            max_range: 50.0,
        }
    }

    /// Load configuration from ROS parameters
    pub fn from_node(node: &Node) -> Result<Self> {
        let default_config = Self::default();

        // Declare and load file path parameters
        let board_config_param = node
            .declare_parameter::<Arc<str>>("board_config_file")
            .default(default_config.board_config_file.clone().into())
            .description("Path to hollow board configuration file")
            .mandatory()
            .map_err(|e| eyre::eyre!("Failed to declare board_config_file parameter: {}", e))?;

        let detector_config_param = node
            .declare_parameter::<Arc<str>>("detector_config_file")
            .default(default_config.detector_config_file.clone().into())
            .description("Path to detector configuration file")
            .mandatory()
            .map_err(|e| eyre::eyre!("Failed to declare detector_config_file parameter: {}", e))?;

        let aruco_pattern_param = node
            .declare_parameter::<Arc<str>>("aruco_pattern_file")
            .default(default_config.aruco_pattern_file.clone().into())
            .description("Path to ArUco pattern file")
            .mandatory()
            .map_err(|e| eyre::eyre!("Failed to declare aruco_pattern_file parameter: {}", e))?;

        // Declare and load detection parameters
        let max_queue_size_param = node
            .declare_parameter::<i64>("max_queue_size")
            .default(default_config.max_queue_size as i64)
            .description("Maximum size of detection queues")
            .range(rclrs::ParameterRange {
                lower: Some(1i64),
                upper: Some(10000i64),
                step: None,
            })
            .mandatory()
            .map_err(|e| eyre::eyre!("Failed to declare max_queue_size parameter: {}", e))?;

        let sync_tolerance_ms_param = node
            .declare_parameter::<i64>("sync_tolerance_ms")
            .default(default_config.sync_tolerance_ms as i64)
            .description("Synchronization tolerance in milliseconds")
            .range(rclrs::ParameterRange {
                lower: Some(1i64),
                upper: Some(10000i64),
                step: None,
            })
            .mandatory()
            .map_err(|e| eyre::eyre!("Failed to declare sync_tolerance_ms parameter: {}", e))?;

        let same_face_mode_param = node
            .declare_parameter::<bool>("same_face_mode")
            .default(default_config.same_face_mode)
            .description("Enable same face calibration mode")
            .mandatory()
            .map_err(|e| eyre::eyre!("Failed to declare same_face_mode parameter: {}", e))?;

        let apply_bug_fix_param = node
            .declare_parameter::<bool>("apply_bug_fix")
            .default(default_config.apply_bug_fix)
            .description("Apply VLP-16 coordinate system bug fix")
            .mandatory()
            .map_err(|e| eyre::eyre!("Failed to declare apply_bug_fix parameter: {}", e))?;

        // Declare and load ROI parameters
        let roi_box_size_x_param = node
            .declare_parameter::<f64>("roi_box_size_x")
            .default(default_config.roi_box_size_x)
            .description("ROI box size in X direction (meters)")
            .range(rclrs::ParameterRange {
                lower: Some(0.1f64),
                upper: Some(100.0f64),
                step: None,
            })
            .mandatory()
            .map_err(|e| eyre::eyre!("Failed to declare roi_box_size_x parameter: {}", e))?;

        let roi_box_size_y_param = node
            .declare_parameter::<f64>("roi_box_size_y")
            .default(default_config.roi_box_size_y)
            .description("ROI box size in Y direction (meters)")
            .range(rclrs::ParameterRange {
                lower: Some(0.1f64),
                upper: Some(100.0f64),
                step: None,
            })
            .mandatory()
            .map_err(|e| eyre::eyre!("Failed to declare roi_box_size_y parameter: {}", e))?;

        let roi_box_size_z_param = node
            .declare_parameter::<f64>("roi_box_size_z")
            .default(default_config.roi_box_size_z)
            .description("ROI box size in Z direction (meters)")
            .range(rclrs::ParameterRange {
                lower: Some(0.1f64),
                upper: Some(100.0f64),
                step: None,
            })
            .mandatory()
            .map_err(|e| eyre::eyre!("Failed to declare roi_box_size_z parameter: {}", e))?;

        let roi_box_position_x_param = node
            .declare_parameter::<f64>("roi_box_position_x")
            .default(default_config.roi_box_position_x)
            .description("ROI box center position in X direction (meters)")
            .range(rclrs::ParameterRange {
                lower: Some(-100.0f64),
                upper: Some(100.0f64),
                step: None,
            })
            .mandatory()
            .map_err(|e| eyre::eyre!("Failed to declare roi_box_position_x parameter: {}", e))?;

        let roi_box_position_y_param = node
            .declare_parameter::<f64>("roi_box_position_y")
            .default(default_config.roi_box_position_y)
            .description("ROI box center position in Y direction (meters)")
            .range(rclrs::ParameterRange {
                lower: Some(-100.0f64),
                upper: Some(100.0f64),
                step: None,
            })
            .mandatory()
            .map_err(|e| eyre::eyre!("Failed to declare roi_box_position_y parameter: {}", e))?;

        let roi_box_position_z_param = node
            .declare_parameter::<f64>("roi_box_position_z")
            .default(default_config.roi_box_position_z)
            .description("ROI box center position in Z direction (meters)")
            .range(rclrs::ParameterRange {
                lower: Some(-100.0f64),
                upper: Some(100.0f64),
                step: None,
            })
            .mandatory()
            .map_err(|e| eyre::eyre!("Failed to declare roi_box_position_z parameter: {}", e))?;

        // Declare and load filter parameters
        let min_range_param = node
            .declare_parameter::<f64>("min_range")
            .default(default_config.min_range as f64)
            .description("Minimum range for point cloud filtering (meters)")
            .range(rclrs::ParameterRange {
                lower: Some(0.0f64),
                upper: Some(1000.0f64),
                step: None,
            })
            .mandatory()
            .map_err(|e| eyre::eyre!("Failed to declare min_range parameter: {}", e))?;

        let max_range_param = node
            .declare_parameter::<f64>("max_range")
            .default(default_config.max_range as f64)
            .description("Maximum range for point cloud filtering (meters)")
            .range(rclrs::ParameterRange {
                lower: Some(0.1f64),
                upper: Some(1000.0f64),
                step: None,
            })
            .mandatory()
            .map_err(|e| eyre::eyre!("Failed to declare max_range parameter: {}", e))?;

        // Create configuration from parameter values
        let config = Self {
            board_config_file: board_config_param.get().to_string(),
            detector_config_file: detector_config_param.get().to_string(),
            aruco_pattern_file: aruco_pattern_param.get().to_string(),
            max_queue_size: max_queue_size_param.get() as usize,
            sync_tolerance_ms: sync_tolerance_ms_param.get() as u64,
            same_face_mode: same_face_mode_param.get(),
            apply_bug_fix: apply_bug_fix_param.get(),
            roi_box_size_x: roi_box_size_x_param.get(),
            roi_box_size_y: roi_box_size_y_param.get(),
            roi_box_size_z: roi_box_size_z_param.get(),
            roi_box_position_x: roi_box_position_x_param.get(),
            roi_box_position_y: roi_box_position_y_param.get(),
            roi_box_position_z: roi_box_position_z_param.get(),
            min_range: min_range_param.get() as f32,
            max_range: max_range_param.get() as f32,
        };

        // Validate loaded configuration
        config.validate()?;

        // Log configuration summary
        config.log_summary(node);

        Ok(config)
    }

    /// Get initial ROI bounds for both LiDARs
    pub fn get_initial_roi_bounds(&self) -> HashMap<u8, RoiBounds> {
        let mut bounds = HashMap::new();

        let roi_bounds = RoiBounds {
            min_x: self.roi_box_position_x - self.roi_box_size_x / 2.0,
            max_x: self.roi_box_position_x + self.roi_box_size_x / 2.0,
            min_y: self.roi_box_position_y - self.roi_box_size_y / 2.0,
            max_y: self.roi_box_position_y + self.roi_box_size_y / 2.0,
            min_z: self.roi_box_position_z - self.roi_box_size_z / 2.0,
            max_z: self.roi_box_position_z + self.roi_box_size_z / 2.0,
        };

        bounds.insert(1, roi_bounds.clone());
        bounds.insert(2, roi_bounds);

        bounds
    }

    /// Validate configuration parameters
    pub fn validate(&self) -> Result<()> {
        // Validate file paths exist
        if !Path::new(&self.board_config_file).exists() {
            return Err(eyre::eyre!(
                "Board config file not found: {}",
                self.board_config_file
            ));
        }

        if !Path::new(&self.detector_config_file).exists() {
            return Err(eyre::eyre!(
                "Detector config file not found: {}",
                self.detector_config_file
            ));
        }

        if !Path::new(&self.aruco_pattern_file).exists() {
            return Err(eyre::eyre!(
                "ArUco pattern file not found: {}",
                self.aruco_pattern_file
            ));
        }

        // Validate ranges
        if self.max_queue_size == 0 {
            return Err(eyre::eyre!("max_queue_size must be > 0"));
        }

        if self.sync_tolerance_ms == 0 {
            return Err(eyre::eyre!("sync_tolerance_ms must be > 0"));
        }

        if self.min_range >= self.max_range {
            return Err(eyre::eyre!("min_range must be < max_range"));
        }

        if self.min_range < 0.0 {
            return Err(eyre::eyre!("min_range must be >= 0"));
        }

        // Validate ROI parameters
        if self.roi_box_size_x <= 0.0 || self.roi_box_size_y <= 0.0 || self.roi_box_size_z <= 0.0 {
            return Err(eyre::eyre!("ROI box dimensions must be > 0"));
        }

        Ok(())
    }

    /// Log configuration summary
    pub fn log_summary(&self, _node: &Node) {
        log_info!("multi_wayside_node", "Multi-Wayside Configuration:");
        log_info!(
            "multi_wayside_node",
            "  Board config: {}",
            self.board_config_file
        );
        log_info!(
            "multi_wayside_node",
            "  Detector config: {}",
            self.detector_config_file
        );
        log_info!(
            "multi_wayside_node",
            "  ArUco pattern: {}",
            self.aruco_pattern_file
        );
        log_info!(
            "multi_wayside_node",
            "  Max queue size: {}",
            self.max_queue_size
        );
        log_info!(
            "multi_wayside_node",
            "  Sync tolerance: {}ms",
            self.sync_tolerance_ms
        );
        log_info!(
            "multi_wayside_node",
            "  Same face mode: {}",
            self.same_face_mode
        );
        log_info!(
            "multi_wayside_node",
            "  Apply bug fix: {}",
            self.apply_bug_fix
        );
        log_info!(
            "multi_wayside_node",
            "  ROI box size: {:.1}×{:.1}×{:.1}m",
            self.roi_box_size_x,
            self.roi_box_size_y,
            self.roi_box_size_z
        );
        log_info!(
            "multi_wayside_node",
            "  ROI box position: ({:.1}, {:.1}, {:.1})",
            self.roi_box_position_x,
            self.roi_box_position_y,
            self.roi_box_position_z
        );
        log_info!(
            "multi_wayside_node",
            "  Range filter: {:.1}m to {:.1}m",
            self.min_range,
            self.max_range
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_config() {
        let config = MultiWaysideConfig::default();

        assert_eq!(config.max_queue_size, 100);
        assert_eq!(config.sync_tolerance_ms, 100);
        assert!(config.same_face_mode);
        assert!(!config.apply_bug_fix);
        assert_eq!(config.roi_box_size_x, 4.0);
        assert_eq!(config.roi_box_position_x, 2.0);
    }

    #[test]
    fn test_get_initial_roi_bounds() {
        let config = MultiWaysideConfig::default();
        let bounds = config.get_initial_roi_bounds();

        assert_eq!(bounds.len(), 2);
        assert!(bounds.contains_key(&1));
        assert!(bounds.contains_key(&2));

        let roi1 = &bounds[&1];
        assert_eq!(roi1.min_x, 0.0); // 2.0 - 4.0/2.0
        assert_eq!(roi1.max_x, 4.0); // 2.0 + 4.0/2.0
    }

    #[test]
    fn test_validate_config() {
        let mut config = MultiWaysideConfig::default();

        // Invalid max_queue_size
        config.max_queue_size = 0;
        assert!(config.validate().is_err());

        config.max_queue_size = 100;

        // Invalid range
        config.min_range = 10.0;
        config.max_range = 5.0;
        assert!(config.validate().is_err());

        config.min_range = 0.5;
        config.max_range = 50.0;

        // Invalid ROI size
        config.roi_box_size_x = 0.0;
        assert!(config.validate().is_err());
    }

    #[test]
    fn test_from_node_with_parameters() {
        use rclrs::{Context, CreateBasicExecutor, InitOptions};

        let context = Context::new(std::env::args(), InitOptions::default()).unwrap();
        let executor = context.create_basic_executor();
        let node = executor.create_node("test_parameter_node").unwrap();

        // This test verifies that the parameter loading mechanism works
        // In practice, the config files may not exist, so validation will fail
        // but the parameter declaration and loading should work
        let result = MultiWaysideConfig::from_node(&node);

        // The result may fail due to missing config files, but it should fail
        // during validation, not during parameter loading
        match result {
            Ok(_config) => {
                // If it succeeds, that's great - all files exist
                println!("Parameter loading successful - all config files exist");
            }
            Err(e) => {
                // Expected case - config files don't exist
                let error_msg = e.to_string();
                assert!(
                    error_msg.contains("not found") || error_msg.contains("File not found"),
                    "Error should be about missing files, got: {}",
                    error_msg
                );
                println!("Parameter loading successful - failed on file validation as expected");
            }
        }
    }
}
