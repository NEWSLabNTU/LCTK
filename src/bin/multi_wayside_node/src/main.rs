use eyre::Result;
use rclrs::{
    log_info, Context, CreateBasicExecutor, InitOptions, RclrsErrorFilter, SpinOptions, ToLogParams,
};
use std::sync::Arc;

// Modules
mod calibration;
mod config;
mod detection;
mod node;
mod pointcloud;
mod roi;
mod types;
mod utils;
mod visualization;

// Imports from modules
use config::MultiWaysideConfig;
use detection::{DefaultDetectionSynchronizer, DetectionPipeline, HollowBoardDetectionProcessor};
use pointcloud::{DefaultPointCloudParser, RangeFilter};
use roi::DefaultRoiManager;
use visualization::{
    DefaultBoardMarkerGenerator, DefaultRoiMarkerGenerator, DefaultTextMarkerGenerator,
};

const LOGGER_NAME: &str = env!("CARGO_BIN_NAME");

/// Multi-wayside node using modular architecture
pub struct MultiWaysideNode {
    // Core processing pipeline
    detection_pipeline: Arc<
        DetectionPipeline<
            DefaultPointCloudParser,
            RangeFilter,
            DefaultRoiManager,
            HollowBoardDetectionProcessor,
        >,
    >,

    // Detection synchronization
    synchronizer: Arc<DefaultDetectionSynchronizer>,

    // Visualization generators
    board_marker_generator: Arc<DefaultBoardMarkerGenerator>,
    roi_marker_generator: Arc<DefaultRoiMarkerGenerator>,
    text_marker_generator: Arc<DefaultTextMarkerGenerator>,

    // Configuration
    config: MultiWaysideConfig,
}

impl MultiWaysideNode {
    pub fn new(node: &rclrs::Node) -> Result<Self> {
        log_info!(
            LOGGER_NAME,
            "Initializing MultiWaysideNode with modular architecture"
        );

        // Load configuration
        let config = MultiWaysideConfig::from_node(node)?;
        config.validate()?;
        config.log_summary(node);

        // Create point cloud parser
        let parser = Arc::new(DefaultPointCloudParser);

        // Create point cloud filter
        let filter = Arc::new(RangeFilter::new(config.min_range, config.max_range));

        // Create ROI manager with initial bounds
        let initial_roi_bounds = config.get_initial_roi_bounds();
        let roi_manager = Arc::new(DefaultRoiManager::with_initial_bounds(initial_roi_bounds));

        // Create board detector
        let detector = Arc::new(HollowBoardDetectionProcessor::from_config_file(
            &config.detector_config_file,
        )?);

        // Create detection pipeline
        let detection_pipeline = Arc::new(DetectionPipeline::new(
            parser,
            filter,
            roi_manager,
            detector,
        ));

        // Create detection synchronizer
        let synchronizer = Arc::new(DefaultDetectionSynchronizer::new(
            config.max_queue_size,
            config.sync_tolerance_ms,
        ));

        // Create visualization generators
        let board_marker_generator = Arc::new(DefaultBoardMarkerGenerator);
        let roi_marker_generator = Arc::new(DefaultRoiMarkerGenerator);
        let text_marker_generator = Arc::new(DefaultTextMarkerGenerator);

        log_info!(
            LOGGER_NAME,
            "MultiWaysideNode components initialized successfully"
        );

        Ok(Self {
            detection_pipeline,
            synchronizer,
            board_marker_generator,
            roi_marker_generator,
            text_marker_generator,
            config,
        })
    }

    pub fn get_detection_pipeline(
        &self,
    ) -> &Arc<
        DetectionPipeline<
            DefaultPointCloudParser,
            RangeFilter,
            DefaultRoiManager,
            HollowBoardDetectionProcessor,
        >,
    > {
        &self.detection_pipeline
    }

    pub fn get_synchronizer(&self) -> &Arc<DefaultDetectionSynchronizer> {
        &self.synchronizer
    }

    pub fn get_board_marker_generator(&self) -> &Arc<DefaultBoardMarkerGenerator> {
        &self.board_marker_generator
    }

    pub fn get_roi_marker_generator(&self) -> &Arc<DefaultRoiMarkerGenerator> {
        &self.roi_marker_generator
    }

    pub fn get_text_marker_generator(&self) -> &Arc<DefaultTextMarkerGenerator> {
        &self.text_marker_generator
    }

    pub fn get_config(&self) -> &MultiWaysideConfig {
        &self.config
    }
}

fn main() -> Result<()> {
    // Initialize ROS 2
    let context = Context::new(std::env::args(), InitOptions::new())?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node(LOGGER_NAME)?;

    // Create the modular multi-wayside node
    let _multi_wayside_node = MultiWaysideNode::new(&node)?;
    log_info!(
        LOGGER_NAME,
        "MultiWaysideNode started successfully with modular architecture"
    );

    // Note: In the refactored architecture, the actual ROS 2 interfaces (subscribers, publishers, services)
    // would be set up in the node module. For now, we have the core architecture in place.
    // The next step would be to implement the node/publishers.rs, node/subscribers.rs, and node/services.rs
    // modules to handle the ROS 2 communication.

    log_info!(LOGGER_NAME, "Node ready for ROS 2 interface implementation");

    // Spin the executor
    executor
        .spin(SpinOptions::default())
        .first_error()
        .map_err(|err| eyre::eyre!("Failed to spin executor: {err}"))
}
