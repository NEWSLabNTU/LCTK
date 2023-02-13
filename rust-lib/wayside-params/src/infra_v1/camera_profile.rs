use crate::common::*;
use common_types::DevicePathV1;
use serde_loader::AbsPathBuf;

pub use camera_focus::*;
pub use camera_interval::*;
pub use camera_resolution::*;

/// The camera device configuration.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct CameraConfig {
    #[serde(flatten)]
    pub device: DevicePathV1,
    pub profile: CameraProfile,
}

/// The generic camera configuration.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum CameraProfile {
    V4l(V4lProfile),
    VideoFile(VideoFileProfile),
}

impl CameraProfile {
    pub fn time_offset(&self) -> chrono::Duration {
        match self {
            Self::V4l(profile) => profile.time_offset,
            Self::VideoFile(profile) => profile.time_offset,
        }
    }
}

/// The v4l device configuration.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct V4lProfile {
    /// The device file path.
    pub device: AbsPathBuf,
    pub resolution: CameraResolution,
    pub interval: CameraInterval,
    #[serde(with = "common_types::serde_fourcc")]
    pub fourcc: [u8; 4],
    pub focus: Option<CameraFocus>,
    #[serde(
        with = "common_types::serde_chrono_duration",
        default = "common_types::zero_chrono_duration"
    )]
    pub time_offset: chrono::Duration,
    #[serde(default = "default_rotate_180")]
    pub rotate_180: bool,
}

#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct VideoFileProfile {
    pub video_file: AbsPathBuf,
    pub timestamps_file: AbsPathBuf,
    #[serde(
        with = "common_types::serde_chrono_duration",
        default = "common_types::zero_chrono_duration"
    )]
    pub time_offset: chrono::Duration,
    #[serde(default = "default_rotate_180")]
    pub rotate_180: bool,
}

fn default_rotate_180() -> bool {
    false
}

mod camera_focus {
    use super::*;

    #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
    pub enum CameraFocus {
        Auto,
        Value(usize),
    }

    impl Serialize for CameraFocus {
        fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
        where
            S: Serializer,
        {
            match *self {
                Self::Auto => "auto".serialize(serializer),
                Self::Value(val) => val.to_string().serialize(serializer),
            }
        }
    }

    impl<'de> Deserialize<'de> for CameraFocus {
        fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
        where
            D: Deserializer<'de>,
        {
            let text = String::deserialize(deserializer)?;
            let focus = match &*text {
                "auto" => Self::Auto,
                text => {
                    let value: usize = text
                        .parse()
                        .map_err(|_| D::Error::custom(format!("invalid focus value '{}'", text)))?;
                    Self::Value(value)
                }
            };
            Ok(focus)
        }
    }
}

mod camera_interval {
    use super::*;

    /// The camera interval that encodes into `NUMERATOR/DENOMINATOR` string,
    /// for example, `1/10` means 10 frames per second.
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
    pub struct CameraInterval {
        pub num: usize,
        pub deno: usize,
    }

    impl CameraInterval {
        pub fn ratio(&self) -> f64 {
            let Self { num, deno } = *self;
            num as f64 / deno as f64
        }
    }

    impl Serialize for CameraInterval {
        fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
        where
            S: Serializer,
        {
            let Self { num, deno } = *self;
            if deno == 0 {
                return Err(S::Error::custom("denominator must not be zero"));
            }
            format!("{}/{}", num, deno).serialize(serializer)
        }
    }

    impl<'de> Deserialize<'de> for CameraInterval {
        fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
        where
            D: Deserializer<'de>,
        {
            let text = String::deserialize(deserializer)?;
            let tokens: Vec<_> = text.split('/').collect();

            let err = || D::Error::custom(format!("invalid frame rate string '{}'", text));

            if tokens.len() != 2 {
                return Err(err());
            }

            let num: usize = tokens[0].parse().map_err(|_| err())?;
            let deno: usize = tokens[1].parse().map_err(|_| err())?;

            if deno == 0 {
                return Err(D::Error::custom("denominator must not be zero"));
            }

            Ok(Self { num, deno })
        }
    }
}

mod camera_resolution {
    use super::*;

    /// The resolution of the camera that encodes into `WIDTHxHEIGHT` string,
    /// for example, `1024x768`.
    #[derive(Debug, Clone, PartialEq, Eq, Hash)]
    pub struct CameraResolution {
        pub width: usize,
        pub height: usize,
    }

    impl Serialize for CameraResolution {
        fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
        where
            S: Serializer,
        {
            let Self { width, height } = *self;
            format!("{}x{}", width, height).serialize(serializer)
        }
    }

    impl<'de> Deserialize<'de> for CameraResolution {
        fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
        where
            D: Deserializer<'de>,
        {
            let text = String::deserialize(deserializer)?;
            let tokens: Vec<_> = text.split('x').collect();

            let err = || D::Error::custom(format!("invalid resolution string '{}'", text));

            if tokens.len() != 2 {
                return Err(err());
            }

            let width: usize = tokens[0].parse().map_err(|_| err())?;
            let height: usize = tokens[1].parse().map_err(|_| err())?;

            Ok(Self { width, height })
        }
    }
}
