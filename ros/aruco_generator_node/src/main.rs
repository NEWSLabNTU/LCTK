use anyhow::Result;
use aruco_generator::ArucoGenerator;
use calibration_target::ValidatedTarget;
use clap::Parser;
use std::path::PathBuf;

#[derive(Debug, Parser)]
#[command(name = "aruco_generator_node")]
#[command(about = "Generate the fiducial paper specified by a Target Definition")]
struct Args {
    /// Path to a Target Definition JSON5 file.
    #[arg(long)]
    pub target_config: PathBuf,

    /// Output image path.
    #[arg(short, long)]
    pub output: PathBuf,

    /// Raster resolution. Physical dimensions still come from the target.
    #[arg(long, default_value_t = 300.0)]
    pub dpi: f64,

    /// Enable preview mode (display the generated markers)
    #[arg(long)]
    pub preview: bool,
}

fn main() -> Result<()> {
    let args = Args::parse();

    let bytes = std::fs::read(&args.target_config)?;
    let target = ValidatedTarget::parse_json5(&bytes)?;

    ArucoGenerator::generate_target_image(&target, args.dpi, &args.output, args.preview)?;

    println!(
        "Generated target fiducial for {}@{} at {}",
        target.target_id(),
        target.revision(),
        args.output.display()
    );

    Ok(())
}
