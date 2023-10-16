use crate::ArucoDictionary;
use measurements::Length;
use noisy_float::prelude::*;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct MultiArucoPattern {
    pub marker_ids: Vec<u32>,
    pub dictionary: ArucoDictionary,
    #[serde(with = "serde_types::serde_length")]
    pub board_size: Length,
    #[serde(with = "serde_types::serde_length")]
    pub board_border_size: Length,
    pub marker_square_size_ratio: R64,
    pub num_squares_per_side: u32,
    pub border_bits: u32,
}

impl MultiArucoPattern {
    pub fn paper_size(&self) -> Length {
        self.board_size
    }

    pub fn square_size(&self) -> Length {
        let Self {
            board_size,
            board_border_size,
            num_squares_per_side,
            ..
        } = *self;
        (board_size - board_border_size * 2.0) / num_squares_per_side as f64
    }

    pub fn marker_size(&self) -> Length {
        self.square_size() * self.marker_square_size_ratio.raw()
    }
}

#[cfg(feature = "with-opencv")]
mod with_opencv {
    use super::MultiArucoPattern;
    use anyhow::{ensure, Result};
    use opencv::{
        core::{Rect, Scalar, CV_8UC1},
        prelude::*,
    };

    impl MultiArucoPattern {
        pub fn to_opencv_mat(&self, dpi: f64) -> Result<Mat> {
            let Self {
                ref marker_ids,
                dictionary,
                board_size,
                board_border_size,
                num_squares_per_side,
                border_bits,
                ..
            } = *self;
            let opencv_dictionary = dictionary.to_opencv_dictionary()?;

            let marker_size_pixels = (self.marker_size().as_inches() * dpi) as i32;
            let square_size_pixels = (self.square_size().as_inches() * dpi) as i32;
            let board_size_pixels = (board_size.as_inches() * dpi) as i32;
            let board_border_size_pixels = (board_border_size.as_inches() * dpi) as i32;

            let init_image = Mat::new_rows_cols_with_default(
                board_size_pixels,
                board_size_pixels,
                CV_8UC1,
                Scalar::new(255.0, 255.0, 255.0, 0.0),
            )?;

            marker_ids
                .iter()
                .enumerate()
                .map(|(index, &marker_id)| -> Result<_> {
                    let row = index / num_squares_per_side as usize;
                    let col = index % num_squares_per_side as usize;
                    let x_offset = board_border_size_pixels
                        + square_size_pixels * col as i32
                        + (square_size_pixels - marker_size_pixels) / 2;
                    let y_offset = board_border_size_pixels
                        + square_size_pixels * row as i32
                        + (square_size_pixels - marker_size_pixels) / 2;

                    let mut marker_image = Mat::default();
                    opencv_dictionary.draw_marker(
                        marker_id as i32,
                        marker_size_pixels,
                        &mut marker_image,
                        border_bits as i32,
                    )?;
                    Ok((x_offset, y_offset, marker_image))
                })
                .try_fold(init_image, |board_image, args_result| -> Result<_> {
                    let (x_offset, y_offset, marker_image) = args_result?;
                    let mut roi_image = Mat::roi(
                        &board_image,
                        Rect::new(x_offset, y_offset, marker_size_pixels, marker_size_pixels),
                    )?;
                    ensure!(marker_image.channels() == roi_image.channels());
                    ensure!(marker_image.rows() == roi_image.rows());
                    ensure!(marker_image.cols() == roi_image.cols());
                    ensure!(y_offset + marker_image.rows() < board_image.rows());
                    ensure!(x_offset + marker_image.cols() < board_image.cols());
                    marker_image.copy_to(&mut roi_image)?;
                    Ok(board_image)
                })
        }
    }
}
