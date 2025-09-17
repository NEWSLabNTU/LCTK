//! Simple example showing how to get intermediate outputs from BoardDetector

use board_fitter::debug::{DataCallback, DebugData};

/// Simple callback that prints intermediate data
#[allow(dead_code)]
struct PrintingCallback;

#[allow(dead_code)]
impl DataCallback for PrintingCallback {
    fn on_intermediate_data(&self, stage: &str, data: &DebugData) {
        match data {
            DebugData::PlaneData { planes, .. } => {
                println!("Stage '{}': Found {} planes", stage, planes.len());
            }
            DebugData::CircleData { holes, .. } => {
                println!("Stage '{}': Found {} holes", stage, holes.len());
            }
            DebugData::DetectionResult { detections, .. } => {
                println!("Stage '{}': Found {} boards", stage, detections.len());
            }
            _ => {}
        }
    }

    fn on_point_cloud(&self, stage: &str, cloud: &board_fitter::PointCloud) {
        println!(
            "Stage '{}': Processing {} points",
            stage,
            cloud.points.len()
        );
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("Simple Debug Example");
    println!("===================");
    println!();
    println!("This example shows how to use debug callbacks to get intermediate outputs.");
    println!("To use it, you need to:");
    println!("1. Create your BoardConfig with board definitions");
    println!("2. Load or generate your point cloud data");
    println!("3. Replace the todo!() placeholders in this file");
    println!();
    println!("Example usage:");
    println!("```rust");
    println!("// Load board configuration from YAML");
    println!("let board_config = BoardConfig::from_yaml_file(\"config/board.yaml\")?;");
    println!();
    println!("// Create debug configuration");
    println!("let debug_config = DebugConfigBuilder::new()");
    println!("    .capture_stages([\"plane_detection\", \"hole_detection\"])");
    println!("    .build();");
    println!();
    println!("// Create debug context with callback");
    println!("let mut debug_context = DebugContext::new(debug_config);");
    println!("debug_context.data_callback = Some(Arc::new(PrintingCallback));");
    println!();
    println!("// Create detector with debug enabled");
    println!("let detector = BoardDetectorBuilder::new(board_config)");
    println!("    .with_debug(debug_config) // Pass debug_config, not debug_context");
    println!("    .build()?;");
    println!("```");

    Ok(())

    // Commented out to avoid compilation errors with todo!()
    /*
    // Load your board configuration
    let board_config = BoardConfig {
        board: todo!("Load your board config here"),
        detection: None,
        metadata: None,
    };

    // Create debug configuration to capture specific stages
    let debug_config = DebugConfigBuilder::new()
        .capture_stages([
            "plane_detection", // Capture plane detection results
            "diamond_fitting", // Capture diamond square fitting
            "hole_detection",  // Capture hole detection results
        ])
        .build();

    // Create debug context with callback
    let mut debug_context = DebugContext::new(debug_config.clone());
    debug_context.data_callback = Some(Arc::new(PrintingCallback));

    // Create detector with debug enabled
    let detector = BoardDetectorBuilder::new(board_config)
        .with_debug(debug_config) // Pass debug_config, not debug_context
        .timeout_ms(10000)
        .build()?;

    // Load your point cloud
    let point_cloud = todo!("Load your point cloud here");

    // Run detection - intermediate outputs will be printed via callback
    let result = detector.detect(&point_cloud)?;

    println!("Final result: {} boards detected", result.detections.len());

    Ok(())
    */
}
