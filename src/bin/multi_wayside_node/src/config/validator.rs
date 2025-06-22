use eyre::Result;
use std::path::Path;

/// Trait for validating configuration
pub trait ConfigValidator {
    fn validate_files(&self, files: &[&str]) -> Result<()>;
    fn validate_ranges(&self, ranges: &[(f64, f64, &str)]) -> Result<()>;
    fn validate_positive_values(&self, values: &[(f64, &str)]) -> Result<()>;
}

/// Default implementation of ConfigValidator
pub struct DefaultConfigValidator;

impl ConfigValidator for DefaultConfigValidator {
    fn validate_files(&self, files: &[&str]) -> Result<()> {
        for file_path in files {
            if !Path::new(file_path).exists() {
                return Err(eyre::eyre!("File not found: {}", file_path));
            }
        }
        Ok(())
    }

    fn validate_ranges(&self, ranges: &[(f64, f64, &str)]) -> Result<()> {
        for (min_val, max_val, name) in ranges {
            if min_val >= max_val {
                return Err(eyre::eyre!(
                    "{}: min ({}) must be < max ({})",
                    name,
                    min_val,
                    max_val
                ));
            }
        }
        Ok(())
    }

    fn validate_positive_values(&self, values: &[(f64, &str)]) -> Result<()> {
        for (value, name) in values {
            if *value <= 0.0 {
                return Err(eyre::eyre!("{} must be > 0, got {}", name, value));
            }
        }
        Ok(())
    }
}

/// Validate directory structure for multi-wayside node
pub fn validate_directory_structure(base_path: &str) -> Result<()> {
    let _validator = DefaultConfigValidator;

    // Check if base directory exists
    if !Path::new(base_path).exists() {
        return Err(eyre::eyre!("Base directory not found: {}", base_path));
    }

    // Expected subdirectories
    let expected_dirs = ["config", "launch", "scripts"];

    for dir in &expected_dirs {
        let dir_path = Path::new(base_path).join(dir);
        if !dir_path.exists() {
            return Err(eyre::eyre!(
                "Required directory not found: {}",
                dir_path.display()
            ));
        }
    }

    Ok(())
}

/// Validate that required config files exist
pub fn validate_config_files(base_path: &str) -> Result<()> {
    let validator = DefaultConfigValidator;

    let config_files = vec![
        format!("{}/config/hollow_board.yaml", base_path),
        format!("{}/config/detector.yaml", base_path),
        format!("{}/config/aruco_pattern.json5", base_path),
    ];

    let config_file_refs: Vec<&str> = config_files.iter().map(|s| s.as_str()).collect();
    validator.validate_files(&config_file_refs)
}

/// Validate numerical parameters
pub fn validate_numerical_params(
    max_queue_size: usize,
    sync_tolerance_ms: u64,
    min_range: f32,
    max_range: f32,
    roi_sizes: (f64, f64, f64),
) -> Result<()> {
    let validator = DefaultConfigValidator;

    // Validate positive values
    let positive_values = [
        (max_queue_size as f64, "max_queue_size"),
        (sync_tolerance_ms as f64, "sync_tolerance_ms"),
        (min_range as f64, "min_range"),
        (max_range as f64, "max_range"),
        (roi_sizes.0, "roi_box_size_x"),
        (roi_sizes.1, "roi_box_size_y"),
        (roi_sizes.2, "roi_box_size_z"),
    ];

    validator.validate_positive_values(&positive_values)?;

    // Validate ranges
    let ranges = [(min_range as f64, max_range as f64, "range filter")];

    validator.validate_ranges(&ranges)?;

    // Validate specific limits
    if max_queue_size > 10000 {
        return Err(eyre::eyre!(
            "max_queue_size too large: {} (max: 10000)",
            max_queue_size
        ));
    }

    if sync_tolerance_ms > 10000 {
        return Err(eyre::eyre!(
            "sync_tolerance_ms too large: {}ms (max: 10000ms)",
            sync_tolerance_ms
        ));
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;
    use tempfile::TempDir;

    #[test]
    fn test_validate_files_success() {
        let temp_dir = TempDir::new().unwrap();
        let file_path = temp_dir.path().join("test.txt");
        fs::write(&file_path, "test").unwrap();

        let validator = DefaultConfigValidator;
        let result = validator.validate_files(&[file_path.to_str().unwrap()]);
        assert!(result.is_ok());
    }

    #[test]
    fn test_validate_files_missing() {
        let validator = DefaultConfigValidator;
        let result = validator.validate_files(&["/nonexistent/file.txt"]);
        assert!(result.is_err());
        assert!(result.unwrap_err().to_string().contains("File not found"));
    }

    #[test]
    fn test_validate_ranges_success() {
        let validator = DefaultConfigValidator;
        let ranges = [(0.5, 10.0, "test_range")];
        let result = validator.validate_ranges(&ranges);
        assert!(result.is_ok());
    }

    #[test]
    fn test_validate_ranges_failure() {
        let validator = DefaultConfigValidator;
        let ranges = [(10.0, 5.0, "test_range")];
        let result = validator.validate_ranges(&ranges);
        assert!(result.is_err());
        assert!(result
            .unwrap_err()
            .to_string()
            .contains("min (10) must be < max (5)"));
    }

    #[test]
    fn test_validate_positive_values_success() {
        let validator = DefaultConfigValidator;
        let values = [(1.0, "test_value"), (0.1, "another_value")];
        let result = validator.validate_positive_values(&values);
        assert!(result.is_ok());
    }

    #[test]
    fn test_validate_positive_values_failure() {
        let validator = DefaultConfigValidator;
        let values = [(0.0, "test_value")];
        let result = validator.validate_positive_values(&values);
        assert!(result.is_err());
        assert!(result.unwrap_err().to_string().contains("must be > 0"));
    }

    #[test]
    fn test_validate_numerical_params() {
        // Valid parameters
        let result = validate_numerical_params(100, 100, 0.5, 50.0, (4.0, 4.0, 2.0));
        assert!(result.is_ok());

        // Invalid range
        let result = validate_numerical_params(100, 100, 50.0, 0.5, (4.0, 4.0, 2.0));
        assert!(result.is_err());

        // Too large queue size
        let result = validate_numerical_params(20000, 100, 0.5, 50.0, (4.0, 4.0, 2.0));
        assert!(result.is_err());
        assert!(result
            .unwrap_err()
            .to_string()
            .contains("max_queue_size too large"));
    }
}
