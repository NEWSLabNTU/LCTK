//! Rerun visualization example for the board-fitter library.

mod args;
mod viewer;

use anyhow::Result;
use board_fitter::{
    BoardDetector, DataCallback, DebugConfig, DebugContext, DebugData, DetectionConfig, PointCloud,
};
use board_fitter_config::BoardConfig;
use std::{
    fs,
    sync::{mpsc, Arc},
};
use viewer::Viewer;

/// A callback implementation that sends DebugData over an MPSC channel.
struct ChannelDataCallback {
    sender: mpsc::Sender<DebugData>,
}

impl DataCallback for ChannelDataCallback {
    fn on_intermediate_data(&self, _stage: &str, data: &DebugData) {
        // Clone the data to send it over the channel, as DebugData is Clone
        self.sender.send(data.clone()).unwrap();
    }

    fn on_point_cloud(&self, _stage: &str, _cloud: &PointCloud) {
        // This callback is also called, but we primarily use on_intermediate_data
        // which might contain the point cloud within DebugData::PointCloud.
        // If specific handling for raw point clouds is needed, it can be added here.
        // For now, we rely on DebugData::PointCloud being sent via on_intermediate_data.
    }
}

fn main() -> Result<()> {
    let args = args::parse_args();

    // Initialize the Viewer
    let viewer = Viewer::new(args.connect, args.serve)?;

    // Load board configuration
    let config_str = fs::read_to_string(args.board_config)?;
    let board_config: BoardConfig = json5::from_str(&config_str)?;

    // Load point cloud
    let point_cloud = board_fitter::io::load_pcd(std::path::Path::new(&args.pcd_file))?;
    viewer.log_point_cloud(&point_cloud)?;

    // Create a channel to send debug data from the callback to the main thread
    let (tx, rx) = mpsc::channel();

    // Create the callback instance and wrap it in an Arc
    let data_callback_instance = ChannelDataCallback { sender: tx };
    let data_callback_arc = Arc::new(data_callback_instance);

    // Create a DebugConfig (default for now, can be customized later)
    let debug_config = DebugConfig::default();

    // Create DebugContext with the data callback
    let debug_context = DebugContext::new(debug_config).with_data_callback(data_callback_arc);

    // Configure and build the detector
    let detection_config = DetectionConfig {
        board_config: board_config.clone(),
        min_confidence: args.min_confidence,
        timeout_ms: args.timeout,
        ..DetectionConfig::new_with_default(board_config.clone())
    };

    let mut detector = BoardDetector::new_with_debug(detection_config, Some(debug_context));

    // Clone the point cloud for the viewer since the detector will take ownership
    let point_cloud_for_viewer = point_cloud.clone();

    // Run detection in a separate thread to allow the main thread to process debug data
    let detection_thread = std::thread::spawn(move || detector.detect(&point_cloud));

    // Process debug data from the channel
    // This loop will run until the sender (in the detector's thread) is dropped
    while let Ok(data) = rx.recv() {
        match data {
            DebugData::PlaneData { planes, .. } => {
                viewer.log_planes(&planes)?;
            }
            DebugData::PointCloud { cloud, metadata: _ } => {
                // This is for intermediate point clouds, e.g., after ROI filtering
                // For now, we'll just log it as a generic point cloud.
                // A more sophisticated approach might log it to a specific path based on metadata.
                viewer.log_point_cloud(&cloud)?;
            }
            DebugData::CircleData { holes, .. } => {
                viewer.log_holes(&holes)?;
            }
            // TODO: Handle other DebugData variants as needed
            _ => {}
        }
    }

    // Wait for detection to finish and log the final result
    if let Ok(Ok(result)) = detection_thread.join() {
        println!("Found {} boards", result.detections.len());
        viewer.log_detections(&result.detections, &point_cloud_for_viewer)?;
    }

    Ok(())
}
