use anyhow::{ensure, Result};
use aruco_config::MultiArucoPattern;
use cv_convert::{OpenCvPose, TryToCv};
use indexmap::IndexSet;
use itertools::{iproduct, izip};
use log::info;
use measurements::Length;
use nalgebra::{Isometry3, Point2, Point3};
use noisy_float::prelude::*;
use opencv::{
    aruco,
    aruco::Dictionary,
    calib3d, core as core_cv,
    core::{Mat, Point2f, Point3d, Ptr, Vector},
    prelude::*,
};
use serde::{Deserialize, Serialize};
use serde_types::CameraIntrinsics;
use std::{collections::HashMap, ops::Div as _};

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

/// An ArUco marker on an image with pose estimation.
#[derive(Clone, Debug)]
pub struct ImagePoseMarker {
    pub id: i32,
    pub corners: [Point2<f32>; 4],
    pub pose: Isometry3<f64>,
}

#[derive(Clone, Debug)]
pub struct ImageDetection {
    id: Vector<i32>,
    corners: Vector<Vector<Point2f>>,
    marker_size: Length,
    camera_matrix: Mat,
    distortion_coefs: Mat,
    pattern: MultiArucoPattern,
}

// HACK: workaround that Mat is not Sync.
unsafe impl Sync for ImageDetection {}

impl ImageDetection {
    pub fn markers(&self) -> impl Iterator<Item = ImageMarker> + '_ {
        izip!(&self.corners, &self.id).map(|(corners, id)| {
            let corners: Vec<Point2<f32>> =
                corners.into_iter().map(|p| Point2::new(p.x, p.y)).collect();

            ImageMarker {
                id,
                corners: corners.try_into().unwrap(),
            }
        })
    }

    pub fn estimate_pose(self) -> Result<PoseEstimation> {
        // compute marker poses
        let mut rvec = Vector::<Point3d>::new();
        let mut tvec = Vector::<Point3d>::new();

        aruco::estimate_pose_single_markers(
            &self.corners,
            self.marker_size.as_meters() as f32,
            &self.camera_matrix,
            &self.distortion_coefs,
            &mut rvec,
            &mut tvec,
            &mut core_cv::no_array(),
            // EstimateParameters::create()?,
        )?;

        Ok(PoseEstimation {
            image_det: self,
            tvec,
            rvec,
        })
    }

    /// Get a reference to the image detection's id.
    pub fn id(&self) -> &Vector<i32> {
        &self.id
    }

    /// Get a reference to the image detection's corners.
    pub fn corners(&self) -> &Vector<Vector<Point2f>> {
        &self.corners
    }
}

#[derive(Clone, Debug)]
pub struct PoseEstimation {
    image_det: ImageDetection,
    tvec: Vector<Point3d>,
    rvec: Vector<Point3d>,
}

impl PoseEstimation {
    pub fn markers(&self) -> impl Iterator<Item = ImagePoseMarker> + '_ {
        izip!(
            &self.image_det.corners,
            &self.image_det.id,
            &self.rvec,
            &self.tvec
        )
        .map(|(corners, id, rvec, tvec)| {
            let corners: Vec<Point2<f32>> =
                corners.into_iter().map(|p| Point2::new(p.x, p.y)).collect();
            let pose: Isometry3<f64> = OpenCvPose { rvec, tvec }.try_to_cv().unwrap();

            ImagePoseMarker {
                id,
                corners: corners.try_into().unwrap(),
                pose,
            }
        })
    }

    pub fn fit_icp(self, params: Params) -> Result<IcpRegression> {
        let Self {
            tvec,
            image_det: ImageDetection { pattern, .. },
            ..
        } = &self;
        let MultiArucoPattern {
            num_squares_per_side,
            ..
        } = *pattern;
        let Params {
            max_icp_iterations,
            icp_pose_weight_threshold,
            icp_rejection_threshold,
            ..
        } = params;
        ensure!(
            max_icp_iterations >= 1,
            "max_icp_iterations must be positive, but get 0"
        );
        let square_size = pattern.square_size();

        let init_source_points = || {
            iproduct!(0..num_squares_per_side, 0..num_squares_per_side).map(|(row, col)| {
                let x = (col as f64 - num_squares_per_side as f64 / 2.0 + 0.5) * square_size;
                let y = (row as f64 - num_squares_per_side as f64 / 2.0 + 0.5) * square_size;
                Point3::new(x.as_meters(), y.as_meters(), 0.0)
            })
        };

        // let max_icp_iterations = 10000;
        // let icp_pose_weight_threshold = 5e-13;
        const TERMINATION_STEP: usize = 16;

        let target_points: Vec<_> = tvec.iter().map(|p| Point3::new(p.x, p.y, p.z)).collect();

        let (pose, icp_losses, _) =
            (0..max_icp_iterations).fold((Isometry3::identity(), vec![], 0), |state, _step| {
                // check step count
                let (_, _, termination_count) = state;
                if termination_count > TERMINATION_STEP {
                    return state;
                }

                let (pose, mut losses, termination_count) = state;

                let align_pose: Isometry3<f64> = {
                    let source_points = init_source_points().map(|p| {
                        let p: [f64; 3] = p.into();
                        Point3::from(p)
                    });
                    let target_points = target_points.iter().map(|&p| {
                        let p: [f64; 3] = p.into();
                        Point3::from(p)
                    });

                    let pairs = izip!(source_points, target_points);
                    newslab_geom_algo::kabsch_na(pairs).unwrap()
                };

                let new_termination_count = {
                    let pose_weight = {
                        let translation_weight = align_pose.translation.vector.norm().powi(2);
                        let rotation_weight = align_pose
                            .rotation
                            .axis_angle()
                            .map(|(_, angle)| angle)
                            .unwrap_or(0.0);
                        translation_weight + rotation_weight
                    };
                    if pose_weight <= icp_pose_weight_threshold.raw() {
                        termination_count + 1
                    } else {
                        0
                    }
                };

                let new_pose = align_pose * pose;

                let loss = {
                    let source_points = init_source_points();

                    izip!(source_points, target_points.iter())
                        .map(|(source_point, target_point)| (source_point - target_point).norm())
                        .sum::<f64>()
                        .div(target_points.len() as f64)
                };
                losses.push(loss);

                (new_pose, losses, new_termination_count)
            });

        let min_icp_loss = icp_losses.iter().cloned().map(r64).min().map(R64::raw);

        // reject result if loss is too large
        let is_valid = matches!(min_icp_loss, Some(loss) if loss <= icp_rejection_threshold.raw());
        let pose = is_valid.then_some(pose);

        Ok(IcpRegression {
            pose_est: self,
            pose,
            min_icp_loss,
            icp_losses,
        })
    }
}

#[derive(Clone, Debug)]
pub struct IcpRegression {
    pose_est: PoseEstimation,
    pose: Option<Isometry3<f64>>,
    min_icp_loss: Option<f64>,
    icp_losses: Vec<f64>,
}

impl IcpRegression {
    pub fn markers(&self) -> impl Iterator<Item = ImagePoseMarker> + '_ {
        self.pose_est.markers()
    }

    /// Get the icp regression's pose.
    pub fn pose(&self) -> Option<Isometry3<f64>> {
        self.pose
    }

    /// Get the icp regression's min icp loss.
    pub fn min_icp_loss(&self) -> Option<f64> {
        self.min_icp_loss
    }

    /// Get a reference to the icp regression's icp losses.
    pub fn icp_losses(&self) -> &[f64] {
        self.icp_losses.as_ref()
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Params {
    pub max_icp_iterations: usize,
    pub icp_pose_weight_threshold: R64,
    pub icp_rejection_threshold: R64,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Builder {
    pub pattern: MultiArucoPattern,
    pub camera_intrinsic: CameraIntrinsics,
}

impl Builder {
    pub fn build(self) -> Result<Detector> {
        let Self {
            pattern,
            camera_intrinsic,
        } = self;

        let MultiArucoPattern {
            board_size,
            board_border_size,
            num_squares_per_side,
            marker_square_size_ratio,
            ref marker_ids,
            ..
        } = pattern;

        let square_size = (board_size - board_border_size * 2.0) / num_squares_per_side as f64;
        let marker_size = square_size * marker_square_size_ratio.raw();
        let marker_ids: IndexSet<u32> = marker_ids.iter().cloned().collect();

        // check if marker IDs are unique
        ensure!(
            marker_ids.len() == num_squares_per_side.pow(2) as usize,
            "ArUco IDs must be unique"
        );

        Ok(Detector {
            pattern,
            camera_intrinsic,
            marker_size,
            marker_ids,
        })
    }
}

#[derive(Debug, Clone)]
pub struct Detector {
    pattern: MultiArucoPattern,
    camera_intrinsic: CameraIntrinsics,
    marker_size: Length,
    marker_ids: IndexSet<u32>,
}

impl Detector {
    pub fn detect_markers(&self, mat: &Mat) -> Result<Option<ImageDetection>> {
        let Self {
            ref pattern,
            ref camera_intrinsic,
            ref marker_ids,
            marker_size,
            ..
        } = *self;
        let MultiArucoPattern {
            dictionary,
            border_bits,
            ..
        } = *pattern;

        let dictionary: Ptr<Dictionary> = dictionary.to_opencv_dictionary()?;
        let camera_matrix: Mat = (&camera_intrinsic.camera_matrix).into();
        let distortion_coefs: Mat = (&camera_intrinsic.distortion_coefs).into();
        let mut canvas = Mat::default();

        // undistord image
        calib3d::undistort(
            mat,
            &mut canvas,
            &camera_matrix,
            &distortion_coefs,
            &core_cv::no_array(),
        )?;

        // find aruco markers
        let (aruco_corners_vec, aruco_ids) = {
            let mut corners_vec = Vector::<Vector<Point2f>>::new();
            let mut ids = Vector::<i32>::new();

            let parameters = {
                let mut params = aruco::DetectorParameters::create()?;
                params.set_marker_border_bits(border_bits as i32);
                params.set_adaptive_thresh_win_size_min(13);
                params.set_adaptive_thresh_win_size_max(33);
                params.set_adaptive_thresh_win_size_step(2);
                params.set_adaptive_thresh_win_size_step(10);
                params.set_corner_refinement_min_accuracy(0.01);
                params
            };

            aruco::detect_markers(
                &canvas,
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
            let reordered_corners_vec: Vector<Vector<Point2f>> = marker_ids
                .iter()
                .map(|&id| {
                    let index = id_to_index[&(id as i32)];
                    corners_vec.get(index).unwrap()
                })
                .collect();

            (reordered_corners_vec, reordered_ids)
        };

        Ok(Some(ImageDetection {
            id: aruco_ids,
            corners: aruco_corners_vec,
            marker_size,
            camera_matrix,
            distortion_coefs,
            pattern: pattern.clone(),
        }))
    }

    pub fn detect_single_aruco(&self, mat: &Mat) -> Result<Vec<ImageMarker>> {
        let Self {
            ref pattern,
            ref camera_intrinsic,
            ..
        } = *self;
        let MultiArucoPattern {
            dictionary,
            border_bits,
            ..
        } = *pattern;

        let dictionary: Ptr<Dictionary> = dictionary.to_opencv_dictionary()?;
        let camera_matrix: Mat = (&camera_intrinsic.camera_matrix).into();
        let distortion_coefs: Mat = (&camera_intrinsic.distortion_coefs).into();
        let mut canvas = Mat::default();

        // undistort image
        calib3d::undistort(
            mat,
            &mut canvas,
            &camera_matrix,
            &distortion_coefs,
            &core_cv::no_array(),
        )?;

        // find aruco markers
        let mut corners_vec = Vector::<Vector<Point2f>>::new();
        let mut ids = Vector::<i32>::new();

        let parameters = {
            let mut params = aruco::DetectorParameters::create()?;
            params.set_marker_border_bits(border_bits as i32);
            params.set_adaptive_thresh_win_size_min(13);
            params.set_adaptive_thresh_win_size_max(33);
            params.set_adaptive_thresh_win_size_step(2);
            params.set_adaptive_thresh_win_size_step(10);
            params.set_corner_refinement_min_accuracy(0.01);
            params
        };

        aruco::detect_markers(
            &canvas,
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

        // convert to ImageMarker
        let markers: Vec<ImageMarker> = izip!(&corners_vec, &ids)
            .map(|(corners, id)| {
                let corners: Vec<Point2<f32>> =
                    corners.into_iter().map(|p| Point2::new(p.x, p.y)).collect();

                ImageMarker {
                    id,
                    corners: corners.try_into().unwrap(),
                }
            })
            .collect();

        Ok(markers)
    }
}
