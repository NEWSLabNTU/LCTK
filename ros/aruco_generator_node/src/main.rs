use anyhow::Result;
use aruco_generator::{ArucoGenerator, Config};
use clap::Parser;
use std::path::PathBuf;

#[derive(Debug, Parser)]
#[command(name = "aruco_generator_node")]
#[command(about = "Generate ArUco markers from configuration file")]
struct Args {
    /// Path to configuration file (TOML or JSON format)
    #[arg(short, long)]
    pub config: PathBuf,

    /// Enable preview mode (display the generated markers)
    #[arg(long)]
    pub preview: bool,
}

fn main() -> Result<()> {
    let args = Args::parse();

    // Load configuration from file
    let config = Config::from_file(&args.config)?;

    // Generate ArUco markers
    ArucoGenerator::generate_from_config(&config, args.preview)?;

    println!("ArUco markers generated successfully!");

    Ok(())
}
