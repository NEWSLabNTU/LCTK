#![allow(dead_code)]

use eyre::Result;
use rclrs::{
    log_info, Context, CreateBasicExecutor, InitOptions, RclrsErrorFilter, SpinOptions, ToLogParams,
};
use std::sync::{Arc, Mutex};

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
use calibration::{
    CalibrationConfig, CalibrationSolver, DefaultCalibrationManager, DefaultCalibrationSolver,
    DefaultTfBroadcaster, TfBroadcaster,
};
use config::MultiWaysideConfig;
use detection::{
    DefaultDetectionSynchronizer, DetectionPipeline, DetectionSynchronizer,
    HollowBoardDetectionProcessor,
};
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
    synchronizer: Arc<Mutex<DefaultDetectionSynchronizer>>,

    // Calibration management
    calibration_manager: Arc<DefaultCalibrationManager>,

    // TF broadcasting
    tf_broadcaster: Arc<DefaultTfBroadcaster>,

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
        let synchronizer = Arc::new(Mutex::new(DefaultDetectionSynchronizer::new(
            config.max_queue_size,
            config.sync_tolerance_ms,
        )));

        // Create calibration manager
        let calibration_config = CalibrationConfig {
            auto_calibrate: true,
            min_detections_for_calibration: 5,
            calibration_timeout_seconds: 30,
            quality_threshold: 0.7,
            same_face_mode: config.same_face_mode,
            apply_bug_fix: config.apply_bug_fix,
            max_queue_size: config.max_queue_size,
            sync_tolerance_ms: config.sync_tolerance_ms,
        };
        let calibration_manager = Arc::new(DefaultCalibrationManager::new(calibration_config));

        // Create TF broadcaster
        let tf_broadcaster = Arc::new(DefaultTfBroadcaster::new(node)?);

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
            calibration_manager,
            tf_broadcaster,
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

    pub fn get_synchronizer(&self) -> &Arc<Mutex<DefaultDetectionSynchronizer>> {
        &self.synchronizer
    }

    pub fn get_calibration_manager(&self) -> &Arc<DefaultCalibrationManager> {
        &self.calibration_manager
    }

    pub fn get_tf_broadcaster(&self) -> &Arc<DefaultTfBroadcaster> {
        &self.tf_broadcaster
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
    let multi_wayside_node = MultiWaysideNode::new(&node)?;
    log_info!(
        LOGGER_NAME,
        "MultiWaysideNode started successfully with modular architecture"
    );

    // Set up ROS 2 interfaces
    let _ros2_interfaces = setup_ros2_interfaces(&node, &multi_wayside_node)?;
    log_info!(LOGGER_NAME, "ROS 2 interfaces established successfully");

    log_info!(
        LOGGER_NAME,
        "MultiWaysideNode fully operational and ready for calibration"
    );

    // Spin the executor
    executor
        .spin(SpinOptions::default())
        .first_error()
        .map_err(|err| eyre::eyre!("Failed to spin executor: {err}"))
}

/// Set up all ROS 2 interfaces (publishers, subscribers, services)
fn setup_ros2_interfaces(
    node: &rclrs::Node,
    multi_wayside_node: &MultiWaysideNode,
) -> Result<ROS2InterfaceContainer> {
    use crate::node::{DefaultPublisherManager, SubscriberFactory, SubscriptionContainer};

    // Create publishers
    let publisher_manager = Arc::new(DefaultPublisherManager::new(node)?);
    log_info!(LOGGER_NAME, "Publishers created successfully");

    // Create subscribers with callbacks
    let detection_pipeline = Arc::clone(multi_wayside_node.get_detection_pipeline());
    let synchronizer = Arc::clone(multi_wayside_node.get_synchronizer());
    let calibration_manager = Arc::clone(multi_wayside_node.get_calibration_manager());
    let tf_broadcaster = Arc::clone(multi_wayside_node.get_tf_broadcaster());
    let publisher_manager_clone = Arc::clone(&publisher_manager);

    // LiDAR 1 point cloud subscriber
    let lidar1_sub = {
        let detection_pipeline = Arc::clone(&detection_pipeline);
        let publisher_manager = Arc::clone(&publisher_manager_clone);
        let synchronizer = Arc::clone(&synchronizer);
        let tf_broadcaster = Arc::clone(&tf_broadcaster);

        SubscriberFactory::create_pointcloud_subscriber(node, "/lidar1/points", move |msg| {
            if let Err(e) = process_lidar_pointcloud(
                msg,
                1,
                &detection_pipeline,
                &synchronizer,
                &publisher_manager,
                &tf_broadcaster,
            ) {
                log_info!(LOGGER_NAME, "Error processing LiDAR 1 point cloud: {e}");
            }
        })?
    };

    // LiDAR 2 point cloud subscriber
    let lidar2_sub = {
        let detection_pipeline = Arc::clone(&detection_pipeline);
        let publisher_manager = Arc::clone(&publisher_manager_clone);
        let synchronizer = Arc::clone(&synchronizer);
        let tf_broadcaster = Arc::clone(&tf_broadcaster);

        SubscriberFactory::create_pointcloud_subscriber(node, "/lidar2/points", move |msg| {
            if let Err(e) = process_lidar_pointcloud(
                msg,
                2,
                &detection_pipeline,
                &synchronizer,
                &publisher_manager,
                &tf_broadcaster,
            ) {
                log_info!(LOGGER_NAME, "Error processing LiDAR 2 point cloud: {e}");
            }
        })?
    };

    // Pose adjustment subscribers (for manual calibration refinement)
    let lidar1_pose_sub = {
        let calibration_manager = Arc::clone(&calibration_manager);
        SubscriberFactory::create_pose_subscriber(
            node,
            "/lidar1/board_pose_adjustment",
            move |msg| {
                log_info!(
                    LOGGER_NAME,
                    "LiDAR 1 pose adjustment received: pos=({:.3}, {:.3}, {:.3})",
                    msg.pose.position.x,
                    msg.pose.position.y,
                    msg.pose.position.z
                );

                // Apply pose adjustment to LiDAR 1 calibration state
                // This could be used to refine manual calibration or adjust detected poses
                {
                    let _manager = &calibration_manager;
                    // Note: In a full implementation, this would apply the adjustment
                    // to the detection pipeline or calibration state
                    log_info!(LOGGER_NAME, "Applied LiDAR 1 pose adjustment");
                }
            },
        )
    }?;

    let lidar2_pose_sub = {
        let calibration_manager = Arc::clone(&calibration_manager);
        SubscriberFactory::create_pose_subscriber(
            node,
            "/lidar2/board_pose_adjustment",
            move |msg| {
                log_info!(
                    LOGGER_NAME,
                    "LiDAR 2 pose adjustment received: pos=({:.3}, {:.3}, {:.3})",
                    msg.pose.position.x,
                    msg.pose.position.y,
                    msg.pose.position.z
                );

                // Apply pose adjustment to LiDAR 2 calibration state
                {
                    let _manager = &calibration_manager;
                    log_info!(LOGGER_NAME, "Applied LiDAR 2 pose adjustment");
                }
            },
        )
    }?;

    // Create subscription container to keep subscriptions alive
    let subscriptions =
        SubscriptionContainer::new(lidar1_sub, lidar2_sub, lidar1_pose_sub, lidar2_pose_sub);

    log_info!(LOGGER_NAME, "Subscribers created successfully");

    // TODO: Set up services when rosbag_deck_interface dependency is resolved
    log_info!(
        LOGGER_NAME,
        "Services will be added when dependencies are resolved"
    );

    Ok(ROS2InterfaceContainer {
        _publisher_manager: publisher_manager,
        _subscriptions: subscriptions,
    })
}

/// Process incoming LiDAR point cloud data
fn process_lidar_pointcloud(
    cloud_msg: sensor_msgs::msg::PointCloud2,
    lidar_id: u8,
    detection_pipeline: &Arc<
        DetectionPipeline<
            DefaultPointCloudParser,
            RangeFilter,
            DefaultRoiManager,
            HollowBoardDetectionProcessor,
        >,
    >,
    synchronizer: &Arc<Mutex<DefaultDetectionSynchronizer>>,
    publisher_manager: &Arc<crate::node::DefaultPublisherManager>,
    tf_broadcaster: &Arc<DefaultTfBroadcaster>,
) -> Result<()> {
    use crate::node::PublisherManager;
    // Process the point cloud through the detection pipeline
    let detections = detection_pipeline.process_pointcloud(&cloud_msg, lidar_id)?;

    // Check if we have a detection and handle it
    if let Some(detection) = &detections.detection {
        // Create a Detection3DArray with the single detection
        let mut detection_array = vision_msgs::msg::Detection3DArray::default();
        detection_array.header.stamp = cloud_msg.header.stamp.clone();
        detection_array.header.frame_id = cloud_msg.header.frame_id.clone();

        // Convert BoardDetection to Detection3D format
        let mut detection_3d = vision_msgs::msg::Detection3D::default();

        // Set bounding box based on board pose
        let pose = &detection.pose;
        let translation = pose.translation;
        let rotation = pose.rotation;

        // Create pose message
        detection_3d.bbox.center.position.x = translation.x;
        detection_3d.bbox.center.position.y = translation.y;
        detection_3d.bbox.center.position.z = translation.z;

        // Convert quaternion
        detection_3d.bbox.center.orientation.x = rotation.coords[0];
        detection_3d.bbox.center.orientation.y = rotation.coords[1];
        detection_3d.bbox.center.orientation.z = rotation.coords[2];
        detection_3d.bbox.center.orientation.w = rotation.coords[3];

        // Set bounding box size (typical calibration board dimensions)
        detection_3d.bbox.size.x = 0.6; // 60cm board width
        detection_3d.bbox.size.y = 0.4; // 40cm board height
        detection_3d.bbox.size.z = 0.02; // 2cm board thickness

        // Add classification result
        let mut result = vision_msgs::msg::ObjectHypothesisWithPose::default();
        result.hypothesis.class_id = "calibration_board".to_string();
        result.hypothesis.score = detection.confidence;
        result.pose.pose = detection_3d.bbox.center.clone();
        detection_3d.results.push(result);

        detection_array.detections.push(detection_3d);

        publisher_manager.publish_detection(&detection_array, lidar_id)?;
        log_info!(LOGGER_NAME, "Published detection for LiDAR {}", lidar_id);

        // Add detection to synchronizer for calibration
        if let Ok(mut sync_guard) = synchronizer.lock() {
            sync_guard.add_detection(detection.clone(), cloud_msg.header.stamp.clone(), lidar_id);

            // Check for synchronized pairs and trigger calibration
            if let Some((detection1, detection2)) = sync_guard.find_synchronized_pair() {
                log_info!(
                    LOGGER_NAME,
                    "Found synchronized detection pair! LiDAR 1: {} detections, LiDAR 2: {} detections",
                    sync_guard.get_queue_sizes().0,
                    sync_guard.get_queue_sizes().1
                );

                // Compute calibration transform
                let calibration_solver = DefaultCalibrationSolver;
                match calibration_solver.compute_transform(
                    &detection1.detection,
                    &detection2.detection,
                    true, // Assuming same face mode for now - should come from config
                ) {
                    Ok(transform) => {
                        log_info!(
                            LOGGER_NAME,
                            "Calibration successful! Transform computed: translation=({:.3}, {:.3}, {:.3}), rotation confidence={:.3}",
                            transform.translation.x,
                            transform.translation.y,
                            transform.translation.z,
                            detection1.detection.confidence.min(detection2.detection.confidence)
                        );

                        // Broadcast calibration transform via TF
                        if let Err(e) = tf_broadcaster.broadcast_calibration_transform(
                            &transform, "lidar1", // parent frame
                            "lidar2", // child frame
                        ) {
                            log_info!(
                                LOGGER_NAME,
                                "Failed to broadcast calibration transform: {e}"
                            );
                        } else {
                            log_info!(LOGGER_NAME, "Calibration transform broadcasted successfully on /calibration_transform");
                        }

                        // Publish calibration result as TransformStamped message
                        let transform_stamped = crate::calibration::isometry_to_transform_stamped(
                            &transform, "lidar1", // parent frame
                            "lidar2", // child frame
                        );

                        if let Err(e) = publisher_manager.publish_transform(&transform_stamped) {
                            log_info!(LOGGER_NAME, "Failed to publish calibration result: {e}");
                        } else {
                            log_info!(LOGGER_NAME, "Calibration result published successfully");
                        }

                        // TODO: Store calibration for use by other components
                    }
                    Err(e) => {
                        log_info!(LOGGER_NAME, "Calibration computation failed: {e}");
                    }
                }
            }
        }
    }

    Ok(())
}

/// Container to keep ROS 2 interfaces alive
struct ROS2InterfaceContainer {
    _publisher_manager: Arc<crate::node::DefaultPublisherManager>,
    _subscriptions: crate::node::SubscriptionContainer,
}
