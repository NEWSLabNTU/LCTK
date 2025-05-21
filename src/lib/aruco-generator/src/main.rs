use anyhow::Result;
use aruco_generator::{ArucoGenerator, Config, InteractiveBuilder};
use clap::Parser;

#[derive(Debug, Parser)]
struct Args {
    #[clap(long)]
    pub preview: bool,

    /// Path to configuration file (TOML format)
    #[clap(long, short)]
    pub config: Option<String>,
}

fn main() -> Result<()> {
    let args = Args::parse();

    // Handle config file loading
    if let Some(config_path) = &args.config {
        let config = Config::from_file(config_path)?;
        ArucoGenerator::generate_from_config(&config, args.preview)?;
        return Ok(());
    }

    // Interactive mode (original behavior)
    let config = InteractiveBuilder::build_interactive_config()?;
    ArucoGenerator::generate_from_config(&config, args.preview)?;

    Ok(())
}
