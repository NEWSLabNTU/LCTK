mod common;
mod config;
mod detector;
mod fuse_gui;
mod select_gui;

use crate::common::*;
use clap::Parser as _;

#[derive(clap::Parser)]
struct Opts {
    #[clap(long)]
    pub config: PathBuf,
}

#[async_rt::main]
async fn main() -> Result<()> {
    let opts = Opts::parse();
    crate::detector::detector(opts.config).unwrap();

    Ok(())
}
