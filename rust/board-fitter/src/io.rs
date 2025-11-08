//! I/O utilities for loading external test data

use crate::types::PointCloud;
use anyhow::{anyhow, Result};
use nalgebra::Point3;
use std::{
    fs::File,
    io::{BufRead, BufReader},
    path::Path,
};

/// Supported point cloud file formats
#[derive(Debug, Clone, Copy)]
pub enum PointCloudFormat {
    /// Point Cloud Data (.pcd)
    Pcd,
    /// Polygon File Format (.ply)
    Ply,
    /// Simple XYZ ASCII (.xyz)
    Xyz,
}

impl PointCloudFormat {
    /// Detect format from file extension
    pub fn from_extension(path: &Path) -> Result<Self> {
        match path.extension().and_then(|s| s.to_str()) {
            Some("pcd") => Ok(PointCloudFormat::Pcd),
            Some("ply") => Ok(PointCloudFormat::Ply),
            Some("xyz") => Ok(PointCloudFormat::Xyz),
            Some(ext) => Err(anyhow!("Unsupported file format: {}", ext)),
            None => Err(anyhow!(
                "Cannot determine file format from path: {:?}",
                path
            )),
        }
    }
}

/// Load point cloud from file
pub fn load_point_cloud<P: AsRef<Path>>(path: P) -> Result<PointCloud> {
    let path = path.as_ref();
    let format = PointCloudFormat::from_extension(path)?;

    match format {
        PointCloudFormat::Pcd => load_pcd(path),
        PointCloudFormat::Ply => load_ply(path),
        PointCloudFormat::Xyz => load_xyz(path),
    }
}

/// Load PCD (Point Cloud Data) format
pub fn load_pcd(path: &Path) -> Result<PointCloud> {
    let file = File::open(path)?;
    let reader = BufReader::new(file);
    let mut lines = reader.lines();

    // Parse header
    let mut points_count = 0;
    let mut data_format = String::new();
    let mut fields = Vec::new();
    let mut in_header = true;

    // Read header
    while in_header {
        let line = lines
            .next()
            .ok_or_else(|| anyhow!("Unexpected end of file"))??;

        if line.starts_with("POINTS") {
            points_count = line
                .split_whitespace()
                .nth(1)
                .ok_or_else(|| anyhow!("Invalid POINTS line"))?
                .parse::<usize>()?;
        } else if line.starts_with("FIELDS") {
            fields = line.split_whitespace().skip(1).map(String::from).collect();
        } else if line.starts_with("DATA") {
            data_format = line
                .split_whitespace()
                .nth(1)
                .ok_or_else(|| anyhow!("Invalid DATA line"))?
                .to_string();
            in_header = false;
        }
    }

    if data_format != "ascii" {
        return Err(anyhow!(
            "Only ASCII PCD format is currently supported, found: {}. Try converting with: pcl_convert_pcd_ascii_binary input.pcd output.pcd 0",
            data_format
        ));
    }

    // Find field indices
    let x_idx = fields
        .iter()
        .position(|f| f == "x")
        .ok_or_else(|| anyhow!("Missing 'x' field in PCD"))?;
    let y_idx = fields
        .iter()
        .position(|f| f == "y")
        .ok_or_else(|| anyhow!("Missing 'y' field in PCD"))?;
    let z_idx = fields
        .iter()
        .position(|f| f == "z")
        .ok_or_else(|| anyhow!("Missing 'z' field in PCD"))?;
    let intensity_idx = fields.iter().position(|f| f == "intensity");

    // Read points
    let mut points = Vec::new();
    let mut intensities = Vec::new();

    for line in lines {
        let line = line?;
        if line.trim().is_empty() {
            continue;
        }

        let values: Vec<&str> = line.split_whitespace().collect();
        if values.len() < fields.len() {
            continue; // Skip incomplete lines
        }

        let x: f64 = values[x_idx].parse()?;
        let y: f64 = values[y_idx].parse()?;
        let z: f64 = values[z_idx].parse()?;

        points.push(Point3::new(x, y, z));

        if let Some(idx) = intensity_idx {
            if let Ok(intensity) = values[idx].parse::<f32>() {
                intensities.push(intensity);
            }
        }
    }

    let frame_id = path
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("unknown")
        .to_string();

    // Validate point count matches header declaration
    if points.len() != points_count {
        return Err(anyhow!(
            "Point count mismatch: header declares {} points but found {}",
            points_count,
            points.len()
        ));
    }

    Ok(PointCloud {
        points,
        intensities: if intensities.is_empty() {
            None
        } else {
            Some(intensities)
        },
        colors: None,
        timestamp: std::time::Instant::now(),
        frame_id,
    })
}

/// Load PLY (Polygon File Format)
fn load_ply(path: &Path) -> Result<PointCloud> {
    let file = File::open(path)?;
    let reader = BufReader::new(file);
    let mut lines = reader.lines();

    let mut vertex_count = 0;
    let mut in_header = true;
    let mut properties = Vec::new();

    // Parse header
    while in_header {
        let line = lines
            .next()
            .ok_or_else(|| anyhow!("Unexpected end of file"))??;

        if line.starts_with("element vertex") {
            vertex_count = line
                .split_whitespace()
                .nth(2)
                .ok_or_else(|| anyhow!("Invalid element vertex line"))?
                .parse::<usize>()?;
        } else if line.starts_with("property") {
            let parts: Vec<&str> = line.split_whitespace().collect();
            if parts.len() >= 3 {
                properties.push(parts[2].to_string());
            }
        } else if line == "end_header" {
            in_header = false;
        }
    }

    // Find property indices
    let x_idx = properties
        .iter()
        .position(|p| p == "x")
        .ok_or_else(|| anyhow!("Missing 'x' property in PLY"))?;
    let y_idx = properties
        .iter()
        .position(|p| p == "y")
        .ok_or_else(|| anyhow!("Missing 'y' property in PLY"))?;
    let z_idx = properties
        .iter()
        .position(|p| p == "z")
        .ok_or_else(|| anyhow!("Missing 'z' property in PLY"))?;

    // Read vertices
    let mut points = Vec::new();

    for line in lines.take(vertex_count) {
        let line = line?;
        let values: Vec<&str> = line.split_whitespace().collect();

        if values.len() < properties.len() {
            continue;
        }

        let x: f64 = values[x_idx].parse()?;
        let y: f64 = values[y_idx].parse()?;
        let z: f64 = values[z_idx].parse()?;

        points.push(Point3::new(x, y, z));
    }

    let frame_id = path
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("unknown")
        .to_string();

    Ok(PointCloud {
        points,
        intensities: None,
        colors: None,
        timestamp: std::time::Instant::now(),
        frame_id,
    })
}

/// Load XYZ format (simple ASCII: x y z per line)
fn load_xyz(path: &Path) -> Result<PointCloud> {
    let file = File::open(path)?;
    let reader = BufReader::new(file);
    let mut points = Vec::new();

    for line in reader.lines() {
        let line = line?;
        let line = line.trim();

        if line.is_empty() || line.starts_with('#') {
            continue; // Skip empty lines and comments
        }

        let values: Vec<&str> = line.split_whitespace().collect();
        if values.len() >= 3 {
            let x: f64 = values[0].parse()?;
            let y: f64 = values[1].parse()?;
            let z: f64 = values[2].parse()?;

            points.push(Point3::new(x, y, z));
        }
    }

    let frame_id = path
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("unknown")
        .to_string();

    Ok(PointCloud {
        points,
        intensities: None,
        colors: None,
        timestamp: std::time::Instant::now(),
        frame_id,
    })
}

/// Download external test data on-the-fly
pub mod downloader {
    use super::*;
    use std::{fs, path::PathBuf};

    /// External test data configuration
    pub struct ExternalDataConfig {
        pub cache_dir: PathBuf,
        pub auto_download: bool,
    }

    impl Default for ExternalDataConfig {
        fn default() -> Self {
            Self {
                cache_dir: PathBuf::from("test_data/external"),
                auto_download: true,
            }
        }
    }

    /// Download and cache external test data
    pub struct TestDataDownloader {
        config: ExternalDataConfig,
    }

    impl TestDataDownloader {
        pub fn new(config: ExternalDataConfig) -> Self {
            Self { config }
        }

        /// Get cached data or download if needed
        pub fn get_dataset(&self, name: &str) -> Result<PointCloud> {
            let cache_path = self.config.cache_dir.join(name);

            // Check if already cached
            if cache_path.exists() {
                return load_point_cloud(&cache_path);
            }

            // Download if auto-download is enabled
            if self.config.auto_download {
                self.download_dataset(name)?;
                return load_point_cloud(&cache_path);
            }

            Err(anyhow!(
                "Dataset '{}' not found and auto-download disabled",
                name
            ))
        }

        fn download_dataset(&self, name: &str) -> Result<()> {
            // Create cache directory
            fs::create_dir_all(&self.config.cache_dir)?;

            // Download based on dataset name
            match name {
                "pcl/table_scene_lms400.pcd" => {
                    self.download_url(
                        "https://raw.githubusercontent.com/PointCloudLibrary/data/master/tutorials/table_scene_lms400.pcd",
                        &self.config.cache_dir.join(name)
                    )
                },
                "open3d/fragment.ply" => {
                    self.download_url(
                        "https://github.com/isl-org/Open3D/raw/main/examples/test_data/fragment.ply",
                        &self.config.cache_dir.join(name)
                    )
                },
                _ => Err(anyhow!("Unknown dataset: {}", name))
            }
        }

        #[cfg(feature = "download")]
        fn download_url(&self, url: &str, path: &Path) -> Result<()> {
            use std::io::Write;

            // Create parent directory
            if let Some(parent) = path.parent() {
                fs::create_dir_all(parent)?;
            }

            // Download using reqwest (would need to add dependency)
            let response = reqwest::blocking::get(url)?;
            let mut file = File::create(path)?;
            file.write_all(&response.bytes()?)?;

            Ok(())
        }

        #[cfg(not(feature = "download"))]
        fn download_url(&self, _url: &str, _path: &Path) -> Result<()> {
            Err(anyhow!(
                "Download feature not enabled. Please run the download script manually."
            ))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;
    use tempfile::NamedTempFile;

    #[test]
    fn test_load_xyz_format() {
        let mut temp_file = NamedTempFile::new().unwrap();
        writeln!(temp_file, "1.0 2.0 3.0").unwrap();
        writeln!(temp_file, "4.0 5.0 6.0").unwrap();
        writeln!(temp_file, "# This is a comment").unwrap();
        writeln!(temp_file, "7.0 8.0 9.0").unwrap();
        temp_file.flush().unwrap();

        let cloud = load_xyz(temp_file.path()).unwrap();

        assert_eq!(cloud.points.len(), 3);
        assert_eq!(cloud.points[0], Point3::new(1.0, 2.0, 3.0));
        assert_eq!(cloud.points[1], Point3::new(4.0, 5.0, 6.0));
        assert_eq!(cloud.points[2], Point3::new(7.0, 8.0, 9.0));
    }

    #[test]
    fn test_format_detection() {
        assert!(matches!(
            PointCloudFormat::from_extension(Path::new("test.pcd")).unwrap(),
            PointCloudFormat::Pcd
        ));
        assert!(matches!(
            PointCloudFormat::from_extension(Path::new("test.ply")).unwrap(),
            PointCloudFormat::Ply
        ));
        assert!(matches!(
            PointCloudFormat::from_extension(Path::new("test.xyz")).unwrap(),
            PointCloudFormat::Xyz
        ));

        assert!(PointCloudFormat::from_extension(Path::new("test.unknown")).is_err());
    }
}
