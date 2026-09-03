use anyhow::{ensure, Context, Result};
use aruco_config::{ArucoDetectorParams, MultiArucoPattern};
use calibration_target::ValidatedTarget;
use indexmap::IndexSet;
use itertools::izip;
use log::info;
use nalgebra::{Isometry3, Point2, Point3};
use opencv::{
    aruco,
    aruco::Dictionary,
    calib3d, core as core_cv,
    core::{Mat, Point2f, Ptr, Vector},
    prelude::*,
    types::VectorOfMat,
};
use sensor_msgs::msg::CameraInfo;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// An ArUco marker on an image.
#[derive(Clone, Debug)]
pub struct Detection {
    pub id: i32,
    pub corners: [Point2<f32>; 4],
    pub pose: Isometry3<f64>,
}

impl Detection {
    pub fn center(&self) -> Point3<f64> {
        self.pose.translation.vector.into()
    }
}

/// An ArUco marker on an image.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ImageMarker {
    pub id: i32,
    pub corners: [Point2<f32>; 4],
}

/// A detected marker paired with its canonical target-local object points.
///
/// Array index is the correspondence contract: OpenCV image corners
/// `[top-left, top-right, bottom-right, bottom-left]` pair with target-local
/// `[right, top, left, bottom]` for the diamond-mounted paper.
#[derive(Clone, Debug)]
pub struct TargetMarkerCorrespondences {
    pub id: u32,
    pub image_corners: [Point2<f32>; 4],
    pub object_corners: [Point3<f64>; 4],
}

impl ImageMarker {
    pub fn target_correspondences(
        &self,
        target: &ValidatedTarget,
    ) -> Result<TargetMarkerCorrespondences> {
        let id = u32::try_from(self.id).context("detected ArUco ID must not be negative")?;
        let object_corners = target
            .marker_corners_by_id()
            .get(&id)
            .copied()
            .with_context(|| {
                format!("ArUco ID {id} is not part of target {}", target.target_id())
            })?;
        Ok(TargetMarkerCorrespondences {
            id,
            image_corners: self.corners,
            object_corners,
        })
    }
}

#[derive(Clone, Debug)]
pub struct ImageDetection {
    id: Vector<i32>,
    corners: VectorOfMat,
}

// HACK: workaround that Mat is not Sync.
unsafe impl Sync for ImageDetection {}

impl ImageDetection {
    pub fn markers(&self) -> impl Iterator<Item = ImageMarker> + '_ {
        izip!(&self.corners, &self.id).map(|(corners_mat, id)| {
            // Extract corner points from Mat (4x1x2 or 1x4x2 matrix)
            let mut corners_vec = Vec::new();
            for i in 0..4 {
                let pt: &Point2f = corners_mat
                    .at_2d(0, i)
                    .unwrap_or_else(|_| corners_mat.at_2d(i, 0).unwrap());
                corners_vec.push(Point2::new(pt.x, pt.y));
            }

            ImageMarker {
                id,
                corners: corners_vec.try_into().unwrap(),
            }
        })
    }

    /// Get a reference to the image detection's id.
    pub fn id(&self) -> &Vector<i32> {
        &self.id
    }

    /// Get a reference to the image detection's corners.
    pub fn corners(&self) -> &VectorOfMat {
        &self.corners
    }
}

#[derive(Debug, Clone)]
pub struct Builder {
    pub pattern: MultiArucoPattern,
    pub camera_info: CameraInfo,
    /// Detector tuning. Defaults enable sub-pixel corner refinement (H-08).
    pub detector_params: ArucoDetectorParams,
}

impl Builder {
    /// Build a detector profile from a Target Definition. Detector tuning and camera
    /// intrinsics stay caller-owned; all printed geometry comes from `target`.
    pub fn from_target(
        target: &ValidatedTarget,
        camera_info: CameraInfo,
        detector_params: ArucoDetectorParams,
    ) -> Result<Self> {
        Ok(Self {
            pattern: MultiArucoPattern::from_target(target)?,
            camera_info,
            detector_params,
        })
    }

    pub fn build(self) -> Result<Detector> {
        let Self {
            pattern,
            camera_info,
            detector_params,
        } = self;

        let MultiArucoPattern {
            num_squares_per_side,
            border_bits,
            ref marker_ids,
            ..
        } = pattern;

        let marker_ids: IndexSet<u32> = marker_ids.iter().cloned().collect();

        // check if marker IDs are unique
        ensure!(
            marker_ids.len() == num_squares_per_side.pow(2) as usize,
            "ArUco IDs must be unique"
        );

        // Validate the detector params once, at construction, rather than per frame.
        detector_params.to_opencv_params(border_bits)?;

        Ok(Detector {
            pattern,
            camera_info,
            detector_params,
            marker_ids,
        })
    }
}

#[derive(Debug, Clone)]
pub struct Detector {
    pattern: MultiArucoPattern,
    camera_info: CameraInfo,
    detector_params: ArucoDetectorParams,
    marker_ids: IndexSet<u32>,
}

impl Detector {
    /// Rectify a raw image with this detector's intrinsics.
    ///
    /// This is a **visualization utility only**. It is NOT part of the detection path: detection
    /// runs on the raw frame (see [`Detector::detect_markers`]), because `undistort` resamples the
    /// image bilinearly and that blunts exactly the gradients sub-pixel corner refinement depends
    /// on. Use this to produce a debug overlay in the same frame the corners are reported in.
    pub fn rectify(&self, mat: &Mat) -> Result<Mat> {
        let camera_matrix = self.camera_matrix()?;
        let distortion_coefs = self.distortion_coefs()?;

        let mut rectified = Mat::default();
        calib3d::undistort(
            mat,
            &mut rectified,
            &camera_matrix,
            &distortion_coefs,
            &core_cv::no_array(),
        )?;

        Ok(rectified)
    }

    fn camera_matrix(&self) -> Result<Mat> {
        Ok(Mat::from_slice(&self.camera_info.k)?.reshape(1, 3)?)
    }

    fn distortion_coefs(&self) -> Result<Mat> {
        Ok(Mat::from_slice(&self.camera_info.d)?)
    }

    /// Map detected corners from the raw (distorted) frame into the rectified frame.
    ///
    /// `P = K` is what makes the output pixel coordinates; omitting it would yield *normalized*
    /// coordinates instead, which is the easy way to get this silently and subtly wrong. The
    /// iterative form is used over the default 5-iteration one because the default leaves residual
    /// error under strong distortion.
    fn undistort_corners(&self, corners: &VectorOfMat) -> Result<VectorOfMat> {
        let camera_matrix = self.camera_matrix()?;
        let distortion_coefs = self.distortion_coefs()?;
        let eye = Mat::eye(3, 3, core_cv::CV_64FC1)?.to_mat()?;

        let criteria = core_cv::TermCriteria::new(
            core_cv::TermCriteria_Type::COUNT as i32 + core_cv::TermCriteria_Type::EPS as i32,
            20,
            1e-8,
        )?;

        corners
            .iter()
            .map(|marker_corners| -> Result<Mat> {
                let mut undistorted = Mat::default();
                calib3d::undistort_points_iter(
                    &marker_corners,
                    &mut undistorted,
                    &camera_matrix,
                    &distortion_coefs,
                    &eye,
                    &camera_matrix,
                    criteria,
                )?;
                Ok(undistorted)
            })
            .collect()
    }

    /// Detect the configured multi-ArUco pattern in a **raw (distorted)** image.
    ///
    /// Detection and sub-pixel refinement run on the unresampled sensor image; the resulting
    /// corners are then mapped into the rectified frame with `undistortPoints`. Corners are
    /// therefore returned in the rectified frame, so downstream PnP uses this detector's `K` with
    /// zero distortion coefficients — the same contract as before, but now exact at the point
    /// level rather than approximated via a warped image.
    ///
    /// Do NOT pass a rectified image here; that would correct it twice (C-03).
    pub fn detect_markers(&self, mat: &Mat) -> Result<Option<ImageDetection>> {
        let Self {
            ref pattern,
            ref marker_ids,
            ref detector_params,
            ..
        } = *self;
        let MultiArucoPattern {
            dictionary,
            border_bits,
            ..
        } = *pattern;

        let dictionary: Ptr<Dictionary> = dictionary.to_opencv_dictionary()?;

        // find aruco markers, with sub-pixel corner refinement (H-08) on the RAW image
        let (aruco_corners_vec, aruco_ids) = {
            let mut corners_vec = VectorOfMat::new();
            let mut ids = Vector::<i32>::new();

            let parameters = detector_params.to_opencv_params(border_bits)?;

            #[allow(clippy::unnecessary_mut_passed)]
            aruco::detect_markers(
                mat,
                &dictionary,
                &mut corners_vec,
                &mut ids,
                &parameters,
                &mut core_cv::no_array(), // rejected_img_points
                &mut core_cv::no_array(),
                &mut core_cv::no_array(),
            )?;

            if !ids.is_empty() {
                info!("found ArUco IDs: {:?}", ids.to_vec());
            }

            // check if detection is consistent with config
            let detected_ids_set: IndexSet<_> = ids.iter().map(|id| id as u32).collect();
            if marker_ids != &detected_ids_set {
                return Ok(None);
            }

            // reorder ids to the same order of that in config
            let id_to_index: HashMap<_, _> = ids
                .iter()
                .enumerate()
                .map(|(index, id)| (id, index))
                .collect();

            let reordered_ids: Vector<i32> = marker_ids.iter().map(|&id| id as i32).collect();
            let reordered_corners_vec: VectorOfMat = marker_ids
                .iter()
                .map(|&id| {
                    let index = id_to_index[&(id as i32)];
                    corners_vec.get(index).unwrap()
                })
                .collect();

            (reordered_corners_vec, reordered_ids)
        };

        // Map the refined corners from the raw frame into the rectified frame. Downstream PnP
        // consumes these with `K` and zero distortion.
        let aruco_corners_vec = self.undistort_corners(&aruco_corners_vec)?;

        Ok(Some(ImageDetection {
            id: aruco_ids,
            corners: aruco_corners_vec,
        }))
    }

    /// Detect any ArUco markers in a **raw (distorted)** image.
    ///
    /// As with [`Detector::detect_markers`], corners are refined on the raw image and returned in
    /// the rectified frame.
    pub fn detect_single_aruco(&self, mat: &Mat) -> Result<Vec<ImageMarker>> {
        let Self {
            ref pattern,
            ref detector_params,
            ..
        } = *self;
        let MultiArucoPattern {
            dictionary,
            border_bits,
            ..
        } = *pattern;

        let dictionary: Ptr<Dictionary> = dictionary.to_opencv_dictionary()?;

        // find aruco markers
        let mut corners_vec = VectorOfMat::new();
        let mut ids = Vector::<i32>::new();

        let parameters = detector_params.to_opencv_params(border_bits)?;

        #[allow(clippy::unnecessary_mut_passed)]
        aruco::detect_markers(
            mat,
            &dictionary,
            &mut corners_vec,
            &mut ids,
            &parameters,
            &mut core_cv::no_array(),
            &mut core_cv::no_array(),
            &mut core_cv::no_array(),
        )?;

        if !ids.is_empty() {
            info!("found ArUco IDs: {:?}", ids.to_vec());
        }

        let corners_vec = self.undistort_corners(&corners_vec)?;

        // convert to ImageMarker
        let markers: Vec<ImageMarker> = izip!(&corners_vec, &ids)
            .map(|(corners_mat, id)| {
                // Extract corner points from Mat (4x1x2 or 1x4x2 matrix)
                let mut corners_vec = Vec::new();
                for i in 0..4 {
                    let pt: &Point2f = corners_mat
                        .at_2d(0, i)
                        .unwrap_or_else(|_| corners_mat.at_2d(i, 0).unwrap());
                    corners_vec.push(Point2::new(pt.x, pt.y));
                }

                ImageMarker {
                    id,
                    corners: corners_vec.try_into().unwrap(),
                }
            })
            .collect();

        Ok(markers)
    }
}
