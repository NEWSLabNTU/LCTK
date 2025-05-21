use anyhow::{ensure, Result};
use aruco_config::ArucoDictionary;
use serde::{Deserialize, Serialize};
use std::path::Path;

/// Configuration for ArUco marker generation
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum Config {
    /// Single ArUco marker
    SingleAruco(SingleArucoConfig),
    /// Single ChArUco board
    SingleCharuco(SingleCharucoConfig),
    /// Multiple ArUco markers arranged in a grid
    MultipleArucos(MultipleArucosConfig),
}

/// Configuration for single ArUco marker generation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SingleArucoConfig {
    /// ArUco dictionary to use
    pub dictionary: ArucoDictionary,
    /// Output file path
    pub output_path: String,
    // TODO: Add specific parameters for single ArUco when implemented
}

/// Configuration for single ChArUco board generation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SingleCharucoConfig {
    /// ArUco dictionary to use
    pub dictionary: ArucoDictionary,
    /// Number of squares per side of the board
    #[serde(default = "default_squares_per_side")]
    pub squares_per_side: u16,
    /// Border bits for the markers
    #[serde(default = "default_border_bits")]
    pub border_bits: u16,
    /// Ratio of marker size to square size (must be between 0.0 and 1.0)
    #[serde(default = "default_marker_to_square_ratio")]
    pub marker_to_square_length_ratio: f64,
    /// Paper size in millimeters
    #[serde(default = "default_paper_size")]
    pub paper_size_mm: f64,
    /// Margin size in millimeters
    #[serde(default = "default_margin_size")]
    pub margin_size_mm: f64,
    /// Resolution in dots per inch
    #[serde(default = "default_dpi")]
    pub dpi: f64,
    /// Output file path
    pub output_path: String,
}

/// Configuration for multiple ArUco markers generation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MultipleArucosConfig {
    /// ArUco dictionary to use
    pub dictionary: ArucoDictionary,
    /// Number of squares per side of the grid
    #[serde(default = "default_squares_per_side_u32")]
    pub num_squares_per_side: u32,
    /// Board size in millimeters
    #[serde(default = "default_board_size")]
    pub board_size_mm: f64,
    /// Board border size in millimeters
    #[serde(default = "default_board_border_size")]
    pub board_border_size_mm: f64,
    /// Ratio of marker size to square size (must be between 0.0 and 1.0)
    #[serde(default = "default_marker_to_square_ratio")]
    pub marker_square_size_ratio: f64,
    /// Border bits for the markers
    #[serde(default = "default_border_bits_u32")]
    pub border_bits: u32,
    /// Resolution in dots per inch
    #[serde(default = "default_dpi")]
    pub dpi: f64,
    /// Marker IDs to use
    pub marker_ids: MarkerIds,
    /// Output file path (optional, can be auto-generated)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub output_path: Option<String>,
}

/// Configuration for marker ID selection
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum MarkerIds {
    /// Use randomly generated marker IDs
    Random,
    /// Use specific marker IDs
    Specific { ids: Vec<u32> },
}

impl Config {
    /// Load configuration from a TOML file
    pub fn from_file<P>(path: P) -> Result<Self>
    where
        P: AsRef<Path>,
    {
        let content = std::fs::read_to_string(path)?;
        let config = toml::from_str(&content)?;
        Ok(config)
    }

    /// Save configuration to a TOML file
    pub fn to_file<P>(&self, path: P) -> Result<()>
    where
        P: AsRef<Path>,
    {
        let content = toml::to_string_pretty(self)?;
        std::fs::write(path, content)?;
        Ok(())
    }

    /// Generate the output image based on the configuration
    pub fn generate_image(&self, preview: bool) -> Result<()> {
        use crate::{generate_multiple_arucos_image, generate_single_charuco_image};

        match self {
            Config::SingleAruco(_config) => {
                // TODO: Implement single ArUco generation
                todo!("Single ArUco generation not yet implemented");
            }
            Config::SingleCharuco(config) => {
                generate_single_charuco_image(config, preview)?;
            }
            Config::MultipleArucos(config) => {
                generate_multiple_arucos_image(config, preview)?;
            }
        }
        Ok(())
    }

    /// Validate the configuration
    pub fn validate(&self) -> Result<()> {
        match self {
            Config::SingleAruco(_config) => {
                // TODO: Add validation for single ArUco when implemented
            }
            Config::SingleCharuco(config) => {
                ensure!(
                    config.marker_to_square_length_ratio > 0.0
                        && config.marker_to_square_length_ratio < 1.0,
                    "marker_to_square_length_ratio must be between 0.0 and 1.0"
                );
                ensure!(
                    config.margin_size_mm * 2.0 < config.paper_size_mm,
                    "margin_size_mm * 2 must be less than paper_size_mm"
                );
                ensure!(config.dpi > 0.0, "dpi must be positive");
                ensure!(config.paper_size_mm > 0.0, "paper_size_mm must be positive");
                ensure!(
                    config.squares_per_side > 0,
                    "squares_per_side must be positive"
                );
                ensure!(config.border_bits > 0, "border_bits must be positive");
            }
            Config::MultipleArucos(config) => {
                ensure!(
                    config.marker_square_size_ratio > 0.0 && config.marker_square_size_ratio < 1.0,
                    "marker_square_size_ratio must be between 0.0 and 1.0"
                );
                ensure!(
                    config.board_border_size_mm >= 0.0
                        && config.board_size_mm - config.board_border_size_mm * 2.0 > 0.0,
                    "board_border_size_mm must be non-negative and board_size_mm - board_border_size_mm * 2 must be positive"
                );
                ensure!(config.dpi > 0.0, "dpi must be positive");
                ensure!(config.board_size_mm > 0.0, "board_size_mm must be positive");
                ensure!(
                    config.num_squares_per_side >= 1,
                    "num_squares_per_side must be at least 1"
                );
                ensure!(config.border_bits > 0, "border_bits must be positive");

                // Validate marker IDs count matches grid size
                if let MarkerIds::Specific { ids } = &config.marker_ids {
                    let expected_count =
                        (config.num_squares_per_side * config.num_squares_per_side) as usize;
                    ensure!(
                        ids.len() == expected_count,
                        "marker_ids count ({}) must match grid size ({})",
                        ids.len(),
                        expected_count
                    );
                }
            }
        }
        Ok(())
    }
}

// Default value functions
fn default_squares_per_side() -> u16 {
    2
}

fn default_squares_per_side_u32() -> u32 {
    2
}

fn default_border_bits() -> u16 {
    1
}

fn default_border_bits_u32() -> u32 {
    1
}

fn default_marker_to_square_ratio() -> f64 {
    0.8
}

fn default_paper_size() -> f64 {
    500.0
}

fn default_margin_size() -> f64 {
    50.0
}

fn default_board_size() -> f64 {
    500.0
}

fn default_board_border_size() -> f64 {
    10.0
}

fn default_dpi() -> f64 {
    300.0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_config_serialization() {
        let config = Config::MultipleArucos(MultipleArucosConfig {
            dictionary: ArucoDictionary::DICT_5X5_1000,
            num_squares_per_side: 2,
            board_size_mm: 500.0,
            board_border_size_mm: 10.0,
            marker_square_size_ratio: 0.8,
            border_bits: 1,
            dpi: 300.0,
            marker_ids: MarkerIds::Random,
            output_path: None,
        });

        let toml_str = toml::to_string_pretty(&config).unwrap();
        println!("Serialized config:\n{}", toml_str);

        let deserialized: Config = toml::from_str(&toml_str).unwrap();
        match (&config, &deserialized) {
            (Config::MultipleArucos(_), Config::MultipleArucos(_)) => { /* OK */ }
            _ => panic!("Config variant mismatch"),
        }
    }

    #[test]
    fn test_config_validation() {
        let config = Config::MultipleArucos(MultipleArucosConfig {
            dictionary: ArucoDictionary::DICT_5X5_1000,
            num_squares_per_side: 2,
            board_size_mm: 500.0,
            board_border_size_mm: 10.0,
            marker_square_size_ratio: 0.8,
            border_bits: 1,
            dpi: 300.0,
            marker_ids: MarkerIds::Specific {
                ids: vec![1, 2, 3, 4],
            },
            output_path: None,
        });

        assert!(config.validate().is_ok());

        // Test invalid marker count
        let invalid_config = Config::MultipleArucos(MultipleArucosConfig {
            dictionary: ArucoDictionary::DICT_5X5_1000,
            num_squares_per_side: 2,
            board_size_mm: 500.0,
            board_border_size_mm: 10.0,
            marker_square_size_ratio: 0.8,
            border_bits: 1,
            dpi: 300.0,
            marker_ids: MarkerIds::Specific {
                ids: vec![1, 2, 3], // Wrong count - should be 4 for 2x2 grid
            },
            output_path: None,
        });
        assert!(invalid_config.validate().is_err());
    }
}
