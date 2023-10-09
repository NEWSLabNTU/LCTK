use aruco_config::multi_aruco::MultiArucoPattern;
use hollow_board_detector::Config as BoardDetectorConfig;
use serde::Deserialize;
use serde_loader::{AbsPathBuf, Json5Path};

#[derive(Debug, Clone, Deserialize)]
pub struct Config {
    pub using_same_face_of_marker: bool,
    pub output_dir: AbsPathBuf,
    pub aruco_pattern: Json5Path<MultiArucoPattern>,
    pub board_detector: Json5Path<BoardDetectorConfig>,
    pub pcap1_config: PcapConfig,
    pub pcap2_config: PcapConfig,
}

#[derive(Debug, Clone, Deserialize)]
pub struct PcapConfig {
    pub sensor: SensorType,
    pub file_path: AbsPathBuf,
    pub frame_selected: Option<usize>,
    pub filter: generic_point_filter::Filter,
}

#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SensorType {
    PuckHires,
    Vlp32c,
}
