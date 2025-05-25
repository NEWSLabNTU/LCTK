pub mod config;

use anyhow::{bail, Result};
use aruco_config::MultiArucoPattern;
use config::MrptCalibration;
use opencv::{
    aruco, calib3d,
    core::{no_array, Point2i, Scalar},
    highgui,
    imgproc::{self, HersheyFonts, LINE_8},
    prelude::*,
};
use sensor_msgs::msg::CameraInfo;
use serde::{Deserialize, Serialize};
use std::{fs, path::Path};

/// ArUco detector configuration
#[derive(Debug, Clone)]
pub struct ArucoDetectorConfig {
    pub camera_info: CameraInfo,
    pub aruco_pattern: MultiArucoPattern,
}

impl ArucoDetectorConfig {
    /// Load configuration from intrinsics file and pattern file
    pub fn from_files(intrinsics_file: &Path, pattern_file: &Path) -> Result<Self> {
        // Load camera intrinsics
        let mrpt_calib: MrptCalibration = {
            let yaml_text = fs::read_to_string(intrinsics_file)?;
            serde_yaml::from_str(&yaml_text)?
        };

        let camera_info: CameraInfo = mrpt_calib.to_camera_info()?;
        let aruco_pattern: MultiArucoPattern = {
            let json5_text = fs::read_to_string(pattern_file)?;
            json5::from_str(&json5_text)?
        };

        Ok(Self {
            camera_info,
            aruco_pattern,
        })
    }
}

/// ArUco detection result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DetectionResult {
    pub markers_found: bool,
    pub marker_ids: Vec<i32>,
    pub markers: Vec<serde_json::Value>,
}

/// ArUco detector implementation
pub struct ArucoDetector {
    detector: aruco_detector::multi_aruco::Detector,
    config: ArucoDetectorConfig,
}

impl ArucoDetector {
    /// Create a new ArUco detector from configuration
    pub fn new(config: ArucoDetectorConfig) -> Result<Self> {
        let detector = aruco_detector::multi_aruco::Builder {
            pattern: config.aruco_pattern.clone(),
            camera_info: config.camera_info.clone(),
        }
        .build()?;

        Ok(Self { detector, config })
    }

    /// Detect ArUco markers in an image
    pub fn detect_markers(&self, image: &Mat) -> Result<DetectionResult> {
        if image.empty() {
            bail!("Input image is empty");
        }

        let detection = self.detector.detect_markers(image)?;

        if let Some(detection) = detection {
            let marker_ids: Vec<i32> = detection.id().to_vec();
            let markers: Vec<serde_json::Value> = detection
                .markers()
                .map(|marker| serde_json::to_value(marker).unwrap_or(serde_json::Value::Null))
                .collect();

            Ok(DetectionResult {
                markers_found: true,
                marker_ids,
                markers,
            })
        } else {
            Ok(DetectionResult {
                markers_found: false,
                marker_ids: Vec::new(),
                markers: Vec::new(),
            })
        }
    }

    /// Detect markers and visualize results on image
    pub fn detect_and_visualize(&self, image: &Mat) -> Result<(DetectionResult, Mat)> {
        let detection_result = self.detect_markers(image)?;
        let visualization = self.create_visualization(image, &detection_result)?;
        Ok((detection_result, visualization))
    }

    /// Create visualization of detection results
    pub fn create_visualization(&self, image: &Mat, result: &DetectionResult) -> Result<Mat> {
        let mut display_image = Mat::default();

        // Convert CameraInfo to OpenCV matrices
        let camera_matrix = Mat::from_slice(&self.config.camera_info.k)?.reshape(1, 3)?;
        let dist_coeffs = Mat::from_slice(&self.config.camera_info.d)?;

        // Undistort the image
        calib3d::undistort(
            image,
            &mut display_image,
            &camera_matrix,
            &dist_coeffs,
            &mut no_array(),
        )?;

        let draw_text = |image: &mut Mat, text: &str, (x, y), (b, g, r)| -> Result<()> {
            imgproc::put_text(
                image,
                text,
                Point2i { x, y },
                HersheyFonts::FONT_HERSHEY_SIMPLEX as i32,
                1.0,
                Scalar::new(b, g, r, 0.0),
                2,
                LINE_8,
                false,
            )?;
            Ok(())
        };

        // Draw detection results
        if result.markers_found {
            // Draw text
            draw_text(
                &mut display_image,
                &format!("Found ArUco IDs: {:?}", result.marker_ids),
                (10, 50),
                (0.0, 255.0, 0.0),
            )?;

            // Re-detect for drawing (we need the actual detection object)
            if let Some(detection) = self.detector.detect_markers(image)? {
                aruco::draw_detected_markers(
                    &mut display_image,
                    detection.corners(),
                    detection.id(),
                    Scalar::new(0.0, 255.0, 0.0, 0.0), // green color
                )?;
            }
        } else {
            draw_text(
                &mut display_image,
                "No ArUco detected",
                (10, 50),
                (0.0, 0.0, 255.0), // red color
            )?;
        }

        Ok(display_image)
    }

    /// Display image with detection results
    pub fn show_visualization(&self, visualization: &Mat, window_name: &str) -> Result<()> {
        highgui::imshow(window_name, visualization)?;
        println!("Press any key to close the window...");
        highgui::wait_key(0)?;
        highgui::destroy_all_windows()?;
        Ok(())
    }

    /// Get camera info
    pub fn camera_info(&self) -> &CameraInfo {
        &self.config.camera_info
    }

    /// Get ArUco pattern
    pub fn aruco_pattern(&self) -> &MultiArucoPattern {
        &self.config.aruco_pattern
    }
}
