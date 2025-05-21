use aruco_config::{ArucoDictionary, MultiArucoPattern};
use measurements::Length;
use noisy_float::prelude::*;

fn main() {
    let pattern = MultiArucoPattern {
        marker_ids: vec![149, 391, 385, 482],
        dictionary: ArucoDictionary::DICT_5X5_1000,
        board_size: Length::from_millimeters(500.0),
        board_border_size: Length::from_millimeters(10.0),
        marker_square_size_ratio: r64(0.8),
        num_squares_per_side: 2,
        border_bits: 1,
    };

    let _image = pattern.to_opencv_mat(300.0).unwrap();
}
