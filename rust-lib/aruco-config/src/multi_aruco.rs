use measurements::Length;
use noisy_float::prelude::*;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct MultiArucoPattern {
    pub marker_ids: Vec<u32>,
    pub dictionary: ArucoDictionary,
    #[serde(with = "common_types::serde_length")]
    pub board_size: Length,
    #[serde(with = "common_types::serde_length")]
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

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ArucoDictionary {
    #[serde(rename = "original")]
    Original,
    #[serde(rename = "4x4")]
    D4x4,
    #[serde(rename = "5x5")]
    D5x5,
    #[serde(rename = "6x6")]
    D6x6,
    #[serde(rename = "7x7")]
    D7x7,
}

#[cfg(feature = "with-opencv")]
mod with_opencv {
    use super::*;
    use opencv::{aruco, core::Ptr};

    impl TryFrom<&ArucoDictionary> for Ptr<aruco::Dictionary> {
        type Error = opencv::Error;

        fn try_from(from: &ArucoDictionary) -> Result<Self, Self::Error> {
            let dict_name: aruco::PREDEFINED_DICTIONARY_NAME = From::from(from);
            let dict = aruco::get_predefined_dictionary(dict_name)?;
            Ok(dict)
        }
    }

    impl TryFrom<ArucoDictionary> for Ptr<aruco::Dictionary> {
        type Error = opencv::Error;

        fn try_from(from: ArucoDictionary) -> Result<Self, Self::Error> {
            TryFrom::try_from(&from)
        }
    }

    impl From<&ArucoDictionary> for aruco::PREDEFINED_DICTIONARY_NAME {
        fn from(from: &ArucoDictionary) -> Self {
            match from {
                ArucoDictionary::Original => aruco::PREDEFINED_DICTIONARY_NAME::DICT_ARUCO_ORIGINAL,
                ArucoDictionary::D4x4 => aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_1000,
                ArucoDictionary::D5x5 => aruco::PREDEFINED_DICTIONARY_NAME::DICT_5X5_1000,
                ArucoDictionary::D6x6 => aruco::PREDEFINED_DICTIONARY_NAME::DICT_6X6_1000,
                ArucoDictionary::D7x7 => aruco::PREDEFINED_DICTIONARY_NAME::DICT_7X7_1000,
            }
        }
    }

    impl From<ArucoDictionary> for aruco::PREDEFINED_DICTIONARY_NAME {
        fn from(from: ArucoDictionary) -> Self {
            aruco::PREDEFINED_DICTIONARY_NAME::from(&from)
        }
    }
}
