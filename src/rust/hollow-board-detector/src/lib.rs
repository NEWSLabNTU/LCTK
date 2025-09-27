pub mod algo;
pub mod config;
pub mod detection;
pub mod detector;

pub use crate::{
    config::Config,
    detection::{BoardIcpState, Detection},
    detector::Detector,
};

/// Initialize logging for the hollow-board-detector library.
/// This should be called once at the beginning of your application.
pub fn init_logging() {
    env_logger::init();
}
