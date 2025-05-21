use anyhow::{ensure, Error, Result};
use approx::abs_diff_eq;
#[cfg(feature = "with-nalgebra")]
use nalgebra as na;
use noisy_float::prelude::*;
#[cfg(feature = "with-opencv")]
use opencv::{core as core_cv, prelude::*};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(try_from = "CameraMatrixUnchecked", into = "CameraMatrixUnchecked")]
pub struct CameraMatrix(pub [[R64; 3]; 3]);

impl Default for CameraMatrix {
    fn default() -> Self {
        Self::identity()
    }
}

impl CameraMatrix {
    pub fn identity() -> Self {
        CameraMatrix([
            [r64(1.0), r64(0.0), r64(0.0)],
            [r64(0.0), r64(1.0), r64(0.0)],
            [r64(0.0), r64(0.0), r64(1.0)],
        ])
    }

    pub fn fx(&self) -> R64 {
        self.0[0][0]
    }

    pub fn fy(&self) -> R64 {
        self.0[1][1]
    }

    pub fn cx(&self) -> R64 {
        self.0[0][2]
    }

    pub fn cy(&self) -> R64 {
        self.0[1][2]
    }
}

#[cfg(feature = "with-nalgebra")]
impl From<&CameraMatrix> for na::Matrix3<f64> {
    fn from(from: &CameraMatrix) -> Self {
        use slice_of_array::prelude::*;
        let values: Vec<_> = from.0.flat().iter().map(|val| val.raw()).collect();
        Self::from_row_slice(&values)
    }
}

#[cfg(feature = "with-opencv")]
impl From<&CameraMatrix> for core_cv::Mat {
    fn from(from: &CameraMatrix) -> Self {
        use slice_of_array::prelude::*;
        Self::from_exact_iter(from.0.flat().iter().map(|val| val.raw()))
            .unwrap()
            .reshape(1, 3)
            .unwrap()
    }
}

#[cfg(feature = "with-opencv")]
impl From<CameraMatrix> for core_cv::Mat {
    fn from(from: CameraMatrix) -> Self {
        (&from).into()
    }
}

impl From<CameraMatrix> for CameraMatrixUnchecked {
    fn from(from: CameraMatrix) -> Self {
        Self(from.0)
    }
}

impl TryFrom<CameraMatrixUnchecked> for CameraMatrix {
    type Error = Error;

    fn try_from(from: CameraMatrixUnchecked) -> Result<Self, Self::Error> {
        let mat = from.0;
        ensure!(
            mat[0][1] == 0.0
                && mat[1][0] == 0.0
                && mat[2][0] == 0.0
                && mat[2][1] == 0.0
                && abs_diff_eq!(mat[2][2].raw(), 1.0)
        );
        Ok(Self(mat))
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct CameraMatrixUnchecked(pub [[R64; 3]; 3]);
