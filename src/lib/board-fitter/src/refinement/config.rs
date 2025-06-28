//! Configuration loading and validation for ICP refinement
//!
//! This module provides utilities for loading ICP configuration from files
//! and validating the settings.

use super::{ConvergenceCriteria, IcpRefinementConfig, IcpStageConfig};
use anyhow::{Context, Result};
use serde::{Deserialize, Serialize};
use std::{fs, path::Path};

/// Load ICP refinement configuration from a YAML file
pub fn load_icp_config<P: AsRef<Path>>(path: P) -> Result<IcpRefinementConfig> {
    let path = path.as_ref();
    let content = fs::read_to_string(path)
        .with_context(|| format!("Failed to read config file: {}", path.display()))?;

    let config: IcpRefinementConfig = serde_yaml::from_str(&content)
        .with_context(|| format!("Failed to parse config file: {}", path.display()))?;

    validate_config(&config)?;

    Ok(config)
}

/// Save ICP refinement configuration to a YAML file
pub fn save_icp_config<P: AsRef<Path>>(config: &IcpRefinementConfig, path: P) -> Result<()> {
    let path = path.as_ref();
    let content = serde_yaml::to_string(config).context("Failed to serialize configuration")?;

    fs::write(path, content)
        .with_context(|| format!("Failed to write config file: {}", path.display()))?;

    Ok(())
}

/// Validate ICP configuration
pub fn validate_config(config: &IcpRefinementConfig) -> Result<()> {
    // Validate global settings
    if config.num_threads == 0 {
        anyhow::bail!("Number of threads must be greater than 0");
    }

    // Validate each stage
    validate_stage_config(&config.square_pose_refinement, "square_pose_refinement")?;
    validate_stage_config(&config.hole_pattern_alignment, "hole_pattern_alignment")?;
    validate_stage_config(&config.board_pose_refinement, "board_pose_refinement")?;
    validate_stage_config(&config.temporal_alignment, "temporal_alignment")?;

    Ok(())
}

/// Validate a single stage configuration
fn validate_stage_config(config: &IcpStageConfig, stage_name: &str) -> Result<()> {
    if !config.enabled {
        return Ok(()); // Skip validation for disabled stages
    }

    // Validate iterations
    if config.max_iterations == 0 {
        anyhow::bail!("{}: max_iterations must be greater than 0", stage_name);
    }

    if config.max_iterations > 1000 {
        tracing::warn!(
            "{}: max_iterations {} is very high, may impact performance",
            stage_name,
            config.max_iterations
        );
    }

    // Validate convergence criteria
    validate_convergence_criteria(&config.convergence_criteria, stage_name)?;

    // Validate downsampling resolution
    if let Some(resolution) = config.downsampling_resolution {
        if resolution <= 0.0 {
            anyhow::bail!("{}: downsampling_resolution must be positive", stage_name);
        }
        if resolution > 1.0 {
            tracing::warn!(
                "{}: downsampling_resolution {} is very large",
                stage_name,
                resolution
            );
        }
    }

    // Validate num_neighbors
    if config.num_neighbors == 0 {
        anyhow::bail!("{}: num_neighbors must be greater than 0", stage_name);
    }

    if config.num_neighbors > 100 {
        tracing::warn!(
            "{}: num_neighbors {} is very high, may impact performance",
            stage_name,
            config.num_neighbors
        );
    }

    Ok(())
}

/// Validate convergence criteria
fn validate_convergence_criteria(criteria: &ConvergenceCriteria, stage_name: &str) -> Result<()> {
    if criteria.rotation_epsilon <= 0.0 {
        anyhow::bail!("{}: rotation_epsilon must be positive", stage_name);
    }

    if criteria.translation_epsilon <= 0.0 {
        anyhow::bail!("{}: translation_epsilon must be positive", stage_name);
    }

    if criteria.rotation_epsilon > 0.1 {
        tracing::warn!(
            "{}: rotation_epsilon {} is very large (>0.1 radians)",
            stage_name,
            criteria.rotation_epsilon
        );
    }

    if criteria.translation_epsilon > 0.1 {
        tracing::warn!(
            "{}: translation_epsilon {} is very large (>0.1 meters)",
            stage_name,
            criteria.translation_epsilon
        );
    }

    Ok(())
}

/// Create example configuration file
pub fn create_example_config<P: AsRef<Path>>(path: P) -> Result<()> {
    let config = IcpRefinementConfig::default();
    save_icp_config(&config, path)?;
    Ok(())
}

/// Configuration builder for programmatic setup
pub struct IcpConfigBuilder {
    config: IcpRefinementConfig,
}

impl IcpConfigBuilder {
    /// Create new builder with default configuration
    pub fn new() -> Self {
        Self {
            config: IcpRefinementConfig::default(),
        }
    }

    /// Enable or disable CUDA
    pub fn with_cuda(mut self, enable: bool) -> Self {
        self.config.enable_cuda = enable;
        self
    }

    /// Set number of CPU threads
    pub fn with_threads(mut self, num_threads: usize) -> Self {
        self.config.num_threads = num_threads;
        self
    }

    /// Configure square pose refinement stage
    pub fn with_square_refinement(mut self, enabled: bool) -> Self {
        self.config.square_pose_refinement.enabled = enabled;
        self
    }

    /// Configure hole pattern alignment stage
    pub fn with_hole_alignment(mut self, enabled: bool) -> Self {
        self.config.hole_pattern_alignment.enabled = enabled;
        self
    }

    /// Configure board pose refinement stage
    pub fn with_board_refinement(mut self, enabled: bool) -> Self {
        self.config.board_pose_refinement.enabled = enabled;
        self
    }

    /// Configure temporal alignment stage
    pub fn with_temporal_alignment(mut self, enabled: bool) -> Self {
        self.config.temporal_alignment.enabled = enabled;
        self
    }

    /// Build and validate configuration
    pub fn build(self) -> Result<IcpRefinementConfig> {
        validate_config(&self.config)?;
        Ok(self.config)
    }
}

impl Default for IcpConfigBuilder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::TempDir;

    #[test]
    fn test_config_validation() {
        let mut config = IcpRefinementConfig::default();
        assert!(validate_config(&config).is_ok());

        // Invalid num_threads
        config.num_threads = 0;
        assert!(validate_config(&config).is_err());
        config.num_threads = 4;

        // Invalid max_iterations
        config.square_pose_refinement.max_iterations = 0;
        assert!(validate_config(&config).is_err());
        config.square_pose_refinement.max_iterations = 20;

        // Invalid convergence criteria
        config
            .board_pose_refinement
            .convergence_criteria
            .rotation_epsilon = 0.0;
        assert!(validate_config(&config).is_err());
    }

    #[test]
    fn test_config_save_load() -> Result<()> {
        let temp_dir = TempDir::new()?;
        let config_path = temp_dir.path().join("test_config.yaml");

        // Save default config
        let original = IcpRefinementConfig::default();
        save_icp_config(&original, &config_path)?;

        // Load and compare
        let loaded = load_icp_config(&config_path)?;
        assert_eq!(loaded.num_threads, original.num_threads);
        assert_eq!(loaded.enable_cuda, original.enable_cuda);

        Ok(())
    }

    #[test]
    fn test_config_builder() {
        let config = IcpConfigBuilder::new()
            .with_cuda(true)
            .with_threads(8)
            .with_temporal_alignment(true)
            .build()
            .unwrap();

        assert!(config.enable_cuda);
        assert_eq!(config.num_threads, 8);
        assert!(config.temporal_alignment.enabled);
    }
}
