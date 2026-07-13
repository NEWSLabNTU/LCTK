pub mod config;

use anyhow::{bail, Result};
use aruco_config::{ArucoDetectorParams, MultiArucoPattern};
use aruco_detector::multi_aruco::ImageMarker;
use config::MrptCalibration;
use opencv::{
    core::{Point2i, Scalar},
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
    /// Geometry of the printed board.
    pub aruco_pattern: MultiArucoPattern,
    /// Tuning of the detector itself, including sub-pixel corner refinement (H-08).
    pub detector_params: ArucoDetectorParams,
}

impl ArucoDetectorConfig {
    /// Load configuration from intrinsics, pattern, and (optionally) detector-params files.
    ///
    /// `detector_params_file` may be `None`, in which case the defaults apply — which enable
    /// sub-pixel refinement. It is a separate file from the pattern because the pattern describes
    /// the *printed board* and is also read by `aruco_generator_node` to produce it.
    pub fn from_files(
        intrinsics_file: &Path,
        pattern_file: &Path,
        detector_params_file: Option<&Path>,
    ) -> Result<Self> {
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

        let detector_params = match detector_params_file {
            Some(path) => {
                let json5_text = fs::read_to_string(path)?;
                json5::from_str(&json5_text)?
            }
            None => ArucoDetectorParams::default(),
        };

        Ok(Self {
            camera_info,
            aruco_pattern,
            detector_params,
        })
    }
}

/// ArUco detection result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DetectionResult {
    pub markers_found: bool,
    pub marker_ids: Vec<i32>,
    pub markers: Vec<ImageMarker>,
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
            detector_params: config.detector_params,
        }
        .build()?;

        Ok(Self { detector, config })
    }

    /// Rectify a raw camera image with the configured intrinsics.
    ///
    /// Visualization only — detection runs on the raw image. See [`ArucoDetector::detect_markers`].
    pub fn rectify(&self, image: &Mat) -> Result<Mat> {
        if image.empty() {
            bail!("Input image is empty");
        }

        self.detector.rectify(image)
    }

    /// Detect ArUco markers in a **raw (distorted)** image.
    ///
    /// Corners are refined sub-pixel on the raw image, then mapped into the rectified frame.
    /// Do not pass a rectified image (that would correct it twice — C-03).
    pub fn detect_markers(&self, image: &Mat) -> Result<DetectionResult> {
        if image.empty() {
            bail!("Input image is empty");
        }

        let detection = self.detector.detect_markers(image)?;

        if let Some(detection) = detection {
            let marker_ids: Vec<i32> = detection.id().to_vec();
            let markers: Vec<ImageMarker> = detection.markers().collect();

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

    /// Detect markers in a **raw** image and visualize the result on its rectified counterpart.
    pub fn detect_and_visualize(&self, image: &Mat) -> Result<(DetectionResult, Mat)> {
        let detection_result = self.detect_markers(image)?;
        let rectified = self.rectify(image)?;
        let visualization = self.create_visualization(&rectified, &detection_result)?;
        Ok((detection_result, visualization))
    }

    /// Create visualization of detection results.
    ///
    /// `image` must be the **rectified** counterpart of the raw frame that produced `result`,
    /// because the corners in `result` live in the rectified frame.
    pub fn create_visualization(&self, image: &Mat, result: &DetectionResult) -> Result<Mat> {
        let mut display_image = image.clone();

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

            // Draw the corners we already have. Re-detecting here would be wrong: `image` is the
            // rectified frame, and the detector consumes raw frames.
            let green = Scalar::new(0.0, 255.0, 0.0, 0.0);
            for marker in &result.markers {
                for i in 0..4 {
                    let a = marker.corners[i];
                    let b = marker.corners[(i + 1) % 4];
                    imgproc::line(
                        &mut display_image,
                        Point2i {
                            x: a.x.round() as i32,
                            y: a.y.round() as i32,
                        },
                        Point2i {
                            x: b.x.round() as i32,
                            y: b.y.round() as i32,
                        },
                        green,
                        2,
                        LINE_8,
                        0,
                    )?;
                }
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
