use serde::{Deserialize, Serialize};

#[derive(
    Debug,
    Clone,
    Copy,
    PartialEq,
    Eq,
    Serialize,
    Deserialize,
    strum::Display,
    strum::AsRefStr,
    strum::EnumString,
    strum::FromRepr,
    strum::VariantNames,
)]
#[allow(non_camel_case_types)]
#[repr(u8)]
pub enum ArucoDictionary {
    DICT_4X4_50,
    DICT_4X4_100,
    DICT_4X4_250,
    DICT_4X4_1000,
    DICT_5X5_50,
    DICT_5X5_100,
    DICT_5X5_250,
    DICT_5X5_1000,
    DICT_6X6_50,
    DICT_6X6_100,
    DICT_6X6_250,
    DICT_6X6_1000,
    DICT_7X7_50,
    DICT_7X7_100,
    DICT_7X7_250,
    DICT_7X7_1000,
    DICT_ARUCO_ORIGINAL,
    DICT_APRILTAG_16h5,
    DICT_APRILTAG_25h9,
    DICT_APRILTAG_36h10,
    DICT_APRILTAG_36h11,
}

#[cfg(feature = "with-opencv")]
mod with_opencv {
    use super::ArucoDictionary;
    use opencv::{aruco, core::Ptr};

    impl ArucoDictionary {
        pub fn to_opencv_dictionary(&self) -> opencv::Result<Ptr<aruco::Dictionary>> {
            let dict_name = self.to_opencv_predefined_dictionary_name();
            let dict = aruco::get_predefined_dictionary(dict_name)?;
            Ok(dict)
        }

        pub fn to_opencv_predefined_dictionary_name(&self) -> aruco::PREDEFINED_DICTIONARY_NAME {
            use aruco::PREDEFINED_DICTIONARY_NAME as P;

            match self {
                Self::DICT_4X4_50 => P::DICT_4X4_50,
                Self::DICT_4X4_100 => P::DICT_4X4_100,
                Self::DICT_4X4_250 => P::DICT_4X4_250,
                Self::DICT_4X4_1000 => P::DICT_4X4_1000,
                Self::DICT_5X5_50 => P::DICT_5X5_50,
                Self::DICT_5X5_100 => P::DICT_5X5_100,
                Self::DICT_5X5_250 => P::DICT_5X5_250,
                Self::DICT_5X5_1000 => P::DICT_5X5_1000,
                Self::DICT_6X6_50 => P::DICT_6X6_50,
                Self::DICT_6X6_100 => P::DICT_6X6_100,
                Self::DICT_6X6_250 => P::DICT_6X6_250,
                Self::DICT_6X6_1000 => P::DICT_6X6_1000,
                Self::DICT_7X7_50 => P::DICT_7X7_50,
                Self::DICT_7X7_100 => P::DICT_7X7_100,
                Self::DICT_7X7_250 => P::DICT_7X7_250,
                Self::DICT_7X7_1000 => P::DICT_7X7_1000,
                Self::DICT_ARUCO_ORIGINAL => P::DICT_ARUCO_ORIGINAL,
                Self::DICT_APRILTAG_16h5 => P::DICT_APRILTAG_16h5,
                Self::DICT_APRILTAG_25h9 => P::DICT_APRILTAG_25h9,
                Self::DICT_APRILTAG_36h10 => P::DICT_APRILTAG_36h10,
                Self::DICT_APRILTAG_36h11 => P::DICT_APRILTAG_36h11,
            }
        }
    }
}
