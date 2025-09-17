//! Command-line argument parsing for the Rerun visualization example.

use clap::Parser;

#[derive(Parser, Debug)]
#[clap(author, version, about, long_about = None)]
pub struct Args {
    /// Path to the PCD file to process.
    #[clap(long, value_parser)]
    pub pcd_file: String,

    /// Path to the board configuration JSON5 file.
    #[clap(long, value_parser)]
    pub board_config: String,

    /// Minimum confidence threshold for detections.
    #[clap(long, value_parser, default_value_t = 0.7)]
    pub min_confidence: f64,

    /// Timeout for the detection process in milliseconds.
    #[clap(long, value_parser, default_value_t = 3000)]
    pub timeout: u64,

    /// Enable verbose debug logging.
    #[clap(long, action)]
    pub verbose: bool,

    /// Connect to a Rerun server at this address.
    #[clap(long)]
    pub connect: Option<String>,

    /// Serve a Rerun web viewer on this port.
    #[clap(long, default_value = "9090")]
    pub serve: Option<String>,
}

pub fn parse_args() -> Args {
    Args::parse()
}
