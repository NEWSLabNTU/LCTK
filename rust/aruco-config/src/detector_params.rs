use serde::{Deserialize, Serialize};

/// How `detectMarkers` localises the marker corners.
///
/// OpenCV's default is [`CornerRefinementMethod::NONE`], which returns the raw contour/quad
/// intersections quantised to roughly the pixel grid. That is the H-08 bug: corner localisation
/// error is the direct input noise of the PnP solve, and with only 16 corners per pose there is no
/// redundancy to average it away.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub enum CornerRefinementMethod {
    /// No refinement. OpenCV's default, and the reason for H-08.
    NONE,
    /// Iterative gradient search (`cornerSubPix`) in a `win_size` window around each corner.
    /// Most accurate on well-resolved markers.
    #[default]
    SUBPIX,
    /// Fit lines to the marker's contour points and intersect adjacent sides. No window to tune;
    /// degrades more gracefully on small or motion-blurred markers.
    CONTOUR,
    /// The AprilTag 2 quad-detection approach. Replaces the detection pipeline rather than
    /// refining the corners of the existing one; not evaluated.
    APRILTAG,
}

/// Corner localisation parameters.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct CornerRefinementParams {
    pub method: CornerRefinementMethod,
    /// Half-width of the `cornerSubPix` search window, in pixels. Only used by
    /// [`CornerRefinementMethod::SUBPIX`].
    ///
    /// Must stay well under half the spacing between two corners of the same marker, or the
    /// windows of adjacent corners overlap and pull each other off-target. The board's markers
    /// span ~200 px at 1.5 m and ~35 px at 6 m, so a window of 5 is safe across the working range.
    pub win_size: i32,
    pub max_iterations: i32,
    pub min_accuracy: f64,
}

impl Default for CornerRefinementParams {
    fn default() -> Self {
        Self {
            method: CornerRefinementMethod::default(),
            win_size: 5,
            max_iterations: 30,
            min_accuracy: 0.01,
        }
    }
}

/// Adaptive-threshold sweep used to find marker candidates.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct AdaptiveThreshParams {
    pub win_size_min: i32,
    pub win_size_max: i32,
    pub win_size_step: i32,
}

impl Default for AdaptiveThreshParams {
    fn default() -> Self {
        Self {
            win_size_min: 13,
            win_size_max: 33,
            win_size_step: 10,
        }
    }
}

/// Tuning for the ArUco *detector*, as opposed to [`crate::MultiArucoPattern`], which describes the
/// *printed board*. They are separate because `aruco_generator_node` reads the pattern to print a
/// physical target and has no business seeing detection tuning.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
#[serde(default)]
pub struct ArucoDetectorParams {
    pub corner_refinement: CornerRefinementParams,
    pub adaptive_thresh: AdaptiveThreshParams,
}

#[cfg(feature = "with-opencv")]
mod with_opencv {
    use super::{ArucoDetectorParams, CornerRefinementMethod};
    use anyhow::{ensure, Result};
    use opencv::{aruco, core::Ptr, prelude::*};

    impl CornerRefinementMethod {
        pub fn to_opencv(self) -> i32 {
            let method = match self {
                Self::NONE => aruco::CornerRefineMethod::CORNER_REFINE_NONE,
                Self::SUBPIX => aruco::CornerRefineMethod::CORNER_REFINE_SUBPIX,
                Self::CONTOUR => aruco::CornerRefineMethod::CORNER_REFINE_CONTOUR,
                Self::APRILTAG => aruco::CornerRefineMethod::CORNER_REFINE_APRILTAG,
            };
            method as i32
        }
    }

    impl ArucoDetectorParams {
        /// Build the OpenCV `DetectorParameters`.
        ///
        /// This is the ONE place `DetectorParameters` is constructed. It previously lived, twice
        /// and divergently, inside `aruco-detector` (L-11).
        pub fn to_opencv_params(&self, border_bits: u32) -> Result<Ptr<aruco::DetectorParameters>> {
            let Self {
                corner_refinement,
                adaptive_thresh,
            } = *self;

            ensure!(
                corner_refinement.win_size >= 1,
                "corner_refinement.win_size must be >= 1, got {}",
                corner_refinement.win_size
            );
            ensure!(
                adaptive_thresh.win_size_min >= 3
                    && adaptive_thresh.win_size_max >= adaptive_thresh.win_size_min
                    && adaptive_thresh.win_size_step >= 1,
                "adaptive_thresh must satisfy 3 <= win_size_min <= win_size_max and win_size_step >= 1"
            );

            let mut params = aruco::DetectorParameters::create()?;
            params.set_marker_border_bits(border_bits as i32);

            params.set_adaptive_thresh_win_size_min(adaptive_thresh.win_size_min);
            params.set_adaptive_thresh_win_size_max(adaptive_thresh.win_size_max);
            params.set_adaptive_thresh_win_size_step(adaptive_thresh.win_size_step);

            params.set_corner_refinement_method(corner_refinement.method.to_opencv());
            params.set_corner_refinement_win_size(corner_refinement.win_size);
            params.set_corner_refinement_max_iterations(corner_refinement.max_iterations);
            params.set_corner_refinement_min_accuracy(corner_refinement.min_accuracy);

            Ok(params)
        }
    }
}
