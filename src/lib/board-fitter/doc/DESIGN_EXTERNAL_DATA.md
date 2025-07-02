# External Test Data Guide

This document explains how to use external verified datasets with the board-fitter library for comprehensive testing and validation.

## Overview

The board-fitter supports three approaches for using external verified datasets:

1. **Git Committed Small Samples** - Small verified samples committed to the repository
2. **Download Script** - Automated downloading of larger datasets  
3. **On-the-fly Download** - Automatic downloading during tests (optional feature)

## 1. Git Committed Small Samples

### Location
Small verified test samples are committed in `test_data/small_samples/`:

```
test_data/small_samples/
├── perfect_diamond_board.xyz     # Perfect 1m diamond board (39 points, <1KB)
├── verification_checksums.txt    # SHA256 checksums for verification
└── README.md                     # Sample documentation
```

### Usage
```rust
use board_fitter::io::load_point_cloud;

#[test]
fn test_with_committed_sample() {
    let cloud = load_point_cloud("test_data/small_samples/perfect_diamond_board.xyz")
        .expect("Failed to load committed sample");
    
    // Test with known good data
    assert_eq!(cloud.points.len(), 39);
}
```

### Benefits
- ✅ Always available in CI/CD
- ✅ Fast test execution  
- ✅ Version controlled
- ✅ No network dependencies

### Limitations
- ❌ Limited dataset size (<1KB each)
- ❌ Cannot include large real-world datasets

## 2. Download Script

### Usage
Run the download script to fetch external datasets:

```bash
# Download all external test data
./scripts/download_test_data.sh

# Or use the Makefile
make setup-data
```

### Available Datasets

#### PCL (Point Cloud Library)
- `test_data/external/pcl/table_scene_lms400.pcd` (460KB)
  - Table scene with planar surfaces
  - Good for plane detection testing
  - Source: https://github.com/PointCloudLibrary/data

- `test_data/external/pcl/table_scene_mug_stereo_textured.pcd` (2.7MB)
  - Organized point cloud with color information
  - Tests color processing pipeline

#### Open3D  
- `test_data/external/open3d/fragment.ply` (1.2MB)
  - Point cloud fragment with surface normals
  - Source: https://github.com/isl-org/Open3D

#### ROS Calibration
- `test_data/external/ros/calibration_board_sample.pcd` (Generated)
  - Synthetic calibration board in ROS PCD format
  - Simulates real ROS calibration scenarios

#### MRPT
- `test_data/external/mrpt/sample_point_cloud.xyz` (100KB)
  - Sample from MRPT dataset collection
  - Source: https://github.com/MRPT/mrpt

#### Synthetic Test Cases
- `test_data/external/synthetic/perfect_board.xyz` (Generated)
- `test_data/external/synthetic/noisy_board.xyz` (2cm noise)
- `test_data/external/synthetic/occluded_board.xyz` (30% occlusion)

### Script Features
- ✅ Automatic verification of downloads
- ✅ Resume interrupted downloads
- ✅ Progress indicators
- ✅ Creates directory structure
- ✅ Generates metadata documentation

### Integration with Tests
```rust
#[test]
fn test_pcl_data() {
    let data_path = "test_data/external/pcl/table_scene_lms400.pcd";
    
    // Skip if data not available
    if !Path::new(data_path).exists() {
        eprintln!("Skipping PCL test - run ./scripts/download_test_data.sh");
        return;
    }
    
    let cloud = load_point_cloud(data_path).unwrap();
    // ... test logic
}
```

## 3. On-the-fly Download (Optional)

### Enable Feature
Add to `Cargo.toml`:
```toml
[features]
download = ["reqwest"]
```

Build with download support:
```bash
cargo test --features download
```

### Usage
```rust
use board_fitter::io::downloader::{ExternalDataConfig, TestDataDownloader};

#[test]
#[cfg(feature = "download")]
fn test_auto_download() {
    let config = ExternalDataConfig::default();
    let downloader = TestDataDownloader::new(config);
    
    // Automatically downloads if not cached
    let cloud = downloader.get_dataset("pcl/table_scene_lms400.pcd").unwrap();
    assert!(!cloud.points.is_empty());
}
```

### Configuration
```rust
let config = ExternalDataConfig {
    cache_dir: PathBuf::from("custom_cache"),
    auto_download: true,
};
```

## Supported File Formats

### PCD (Point Cloud Data)
```
# .PCD v0.7 - Point Cloud Data file format  
VERSION 0.7
FIELDS x y z intensity
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH 400
HEIGHT 1
POINTS 400
DATA ascii
1.0 2.0 3.0 128
...
```

### PLY (Polygon File Format)
```
ply
format ascii 1.0
element vertex 1000
property float x
property float y  
property float z
end_header
1.0 2.0 3.0
...
```

### XYZ (Simple ASCII)
```
# Comments start with #
1.0 2.0 3.0
4.0 5.0 6.0
...
```

## Build Integration

### Makefile Targets
```bash
make setup-data      # Download external data
make test-external   # Run tests with external data
make clean-data      # Remove downloaded data
make bench           # Run benchmarks with external data
```

### CI/CD Integration
```yaml
# GitHub Actions example
- name: Setup test data
  run: |
    if [ "${{ matrix.include-external-data }}" = "true" ]; then
      make setup-data
    fi

- name: Run tests
  run: |
    if [ "${{ matrix.include-external-data }}" = "true" ]; then
      make test-external
    else
      make test-unit
    fi
```

## Best Practices

### Test Organization
1. **Always provide fallback**: Tests should skip gracefully if external data isn't available
2. **Document data sources**: Include licensing and source information  
3. **Verify checksums**: Validate downloaded data integrity
4. **Cache wisely**: Use `.gitignore` to exclude large datasets from version control

### Performance Considerations
1. **Lazy loading**: Only download data when needed
2. **Parallel downloads**: Use the download script for batch operations
3. **Cache management**: Clean old data periodically

### Cross-Platform Support
```bash
# Works on Linux/macOS/Windows (with WSL)
if command -v wget &> /dev/null; then
    wget -O "$output" "$url"
elif command -v curl &> /dev/null; then
    curl -L -o "$output" "$url"
fi
```

## Troubleshooting

### Common Issues

#### Download Failures
```bash
# Check network connectivity
curl -I https://github.com

# Manual download
wget https://raw.githubusercontent.com/PointCloudLibrary/data/master/tutorials/table_scene_lms400.pcd
```

#### Permission Issues
```bash
# Make script executable
chmod +x scripts/download_test_data.sh

# Check directory permissions
ls -la test_data/
```

#### Disk Space
```bash
# Check available space
df -h

# Clean old data
make clean-data
```

#### Format Issues
```rust
// Debug point cloud loading
match load_point_cloud(path) {
    Ok(cloud) => println!("Loaded {} points", cloud.points.len()),
    Err(e) => eprintln!("Load failed: {}", e),
}
```

### Getting Help
1. Check the download script output for detailed error messages
2. Verify file integrity with checksums
3. Test with small committed samples first
4. Check network connectivity and firewall settings

## Data Sources and Licenses

| Dataset | Source | License | Size | Purpose |
|---------|--------|---------|------|---------|
| PCL tutorials | https://github.com/PointCloudLibrary/data | BSD | ~3MB | Plane detection |
| Open3D samples | https://github.com/isl-org/Open3D | MIT | ~1MB | Normal processing |
| MRPT datasets | https://github.com/MRPT/mrpt | BSD | ~100KB | General validation |
| Synthetic | Generated | MIT | ~10KB | Controlled testing |

**Note**: Each dataset retains its original license. Please refer to source repositories for full license terms.