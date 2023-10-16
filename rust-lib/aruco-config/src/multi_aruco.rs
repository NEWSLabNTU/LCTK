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
