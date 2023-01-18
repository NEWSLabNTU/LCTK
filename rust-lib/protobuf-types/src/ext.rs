use crate::{
    utils, BBox2D, BBoxPvrcnn, DetectorImage, DevicePath, Image, ImageFrame, LidarPoint,
    LidarScanMessage, MatcherFeedback, RecordTime, UnitQuaternion, VideoCaptureMessage,
};
use anyhow::{bail, Result};
use chrono::NaiveDateTime;
use std::{
    collections::HashSet,
    fmt::{self, Display},
    hash::{Hash, Hasher},
    ops::{Add, Bound, Bound::*, RangeBounds as _, Sub},
    time::Duration,
};

pub use image_format::*;
mod image_format {
    use super::*;

    const FORMAT_RGB3: [u8; 4] = [b'R', b'G', b'B', b'3'];
    const FORMAT_BGR3: [u8; 4] = [b'B', b'G', b'R', b'3'];

    #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
    pub enum ImageFormat {
        RGB3,
        BGR3,
    }

    impl ImageFormat {
        pub fn from_fourcc(fourcc: &[u8]) -> Result<Self> {
            let format = if fourcc == FORMAT_RGB3 {
                Self::RGB3
            } else if fourcc == FORMAT_BGR3 {
                Self::BGR3
            } else {
                bail!("unsupported fourcc '{}'", utils::escape_bytes(fourcc));
            };
            Ok(format)
        }

        pub fn to_fourcc(&self) -> [u8; 4] {
            match self {
                Self::RGB3 => FORMAT_RGB3,
                Self::BGR3 => FORMAT_BGR3,
            }
        }
    }
}

pub use time_upper_bound::*;
mod time_upper_bound {
    use super::*;

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum TimeUpperBound {
        Included(Duration),
        Excluded(Duration),
        None,
    }

    impl TimeUpperBound {
        pub fn contains(&self, timestamp: Duration) -> bool {
            use TimeUpperBound as T;

            match *self {
                T::None => false,
                T::Included(upper) => (Unbounded, Included(upper)).contains(&timestamp),
                T::Excluded(upper) => (Unbounded, Excluded(upper)).contains(&timestamp),
            }
        }
    }

    impl Add<Duration> for TimeUpperBound {
        type Output = Self;

        fn add(self, other: Duration) -> Self::Output {
            use TimeUpperBound as T;

            match self {
                T::None => T::None,
                T::Included(upper) => T::Included(upper + other),
                T::Excluded(upper) => T::Excluded(upper + other),
            }
        }
    }

    impl Sub<Duration> for TimeUpperBound {
        type Output = Self;

        fn sub(self, other: Duration) -> Self::Output {
            use TimeUpperBound as T;

            match self {
                T::None => T::None,
                T::Included(upper) => T::Included(upper - other),
                T::Excluded(upper) => T::Excluded(upper - other),
            }
        }
    }

    // impl PartialOrd for TimeUpperBound {
    //     fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
    //         Some(self.cmp(other))
    //     }
    // }

    // impl Ord for TimeUpperBound {
    //     fn cmp(&self, other: &Self) -> Ordering {
    //         match (&self.0, &other.0) {
    //             (Unbounded, Unbounded) => Equal,
    //             (Unbounded, _) => Less,
    //             (_, Unbounded) => Greater,
    //             (Included(lhs), Excluded(rhs)) if lhs == rhs => Greater,
    //             (Excluded(lhs), Included(rhs)) if lhs == rhs => Less,
    //             (Included(lhs), Included(rhs))
    //             | (Included(lhs), Excluded(rhs))
    //             | (Excluded(lhs), Included(rhs))
    //             | (Excluded(lhs), Excluded(rhs)) => lhs.cmp(rhs),
    //         }
    //     }
    // }

    impl Display for TimeUpperBound {
        fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> Result<(), fmt::Error> {
            use TimeUpperBound::*;

            match *self {
                None => write!(formatter, "< ∞"),
                Included(time) => {
                    write!(formatter, "<= {:?}", duration_to_unix_naive_datetime(&time))
                }
                Excluded(time) => {
                    write!(formatter, "< {:?}", duration_to_unix_naive_datetime(&time))
                }
            }
        }
    }

    fn duration_to_unix_naive_datetime(duration: &Duration) -> NaiveDateTime {
        let nanos = duration.as_nanos();
        let secs = nanos / 1_000_000_000;
        let nsecs = nanos % 1_000_000_000;
        NaiveDateTime::from_timestamp_opt(secs as i64, nsecs as u32).unwrap()
    }
}

impl DevicePath {
    pub fn to_device_path(&self) -> serde_types::DevicePath {
        self.try_into().unwrap()
    }
}

impl Eq for DevicePath {}

impl Eq for BBoxPvrcnn {}

impl Hash for BBoxPvrcnn {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.cluster_id.hash(state);
        self.label.hash(state);
    }
}

#[allow(clippy::derive_hash_xor_eq)]
impl Hash for DevicePath {
    fn hash<H>(&self, state: &mut H)
    where
        H: Hasher,
    {
        self.host.hash(state);
        self.device.hash(state);
    }
}

impl Display for DevicePath {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> Result<(), fmt::Error> {
        write!(f, "{}/{}", self.host, self.device)
    }
}

impl ImageFrame {
    pub fn device_path(&self) -> serde_types::DevicePath {
        (&self.device).try_into().unwrap()
    }
}

impl DetectorImage {
    pub fn device_path(&self) -> serde_types::DevicePath {
        serde_types::DevicePath::new(&self.host_name, &self.source_name)
    }
}

impl LidarScanMessage {
    pub fn device_path(&self) -> serde_types::DevicePath {
        (&self.device).try_into().unwrap()
    }
}

impl VideoCaptureMessage {
    pub fn device_path(&self) -> serde_types::DevicePath {
        (&self.device).try_into().unwrap()
    }
}

impl BBox2D {
    pub fn from_tlhw(tlhw: [f64; 4]) -> Option<Self> {
        let [t, l, h, w] = tlhw;
        (w >= 0.0 && h >= 0.0).then(|| Self {
            left: l,
            top: t,
            width: w,
            height: h,
        })
    }

    pub fn from_cycxhw(cycxhw: [f64; 4]) -> Option<Self> {
        let [cy, cx, h, w] = cycxhw;
        (w >= 0.0 && h >= 0.0).then(|| {
            let t = cy - h / 2.0;
            let l = cx - w / 2.0;
            Self {
                left: l,
                top: t,
                width: w,
                height: h,
            }
        })
    }

    pub fn from_tlbr(tlbr: [f64; 4]) -> Option<Self> {
        let [t, l, b, r] = tlbr;
        let w = r - l;
        let h = b - t;
        Self::from_tlhw([t, l, h, w])
    }

    pub fn tlhw(&self) -> [f64; 4] {
        let Self {
            top,
            left,
            height,
            width,
        } = *self;
        [top, left, height, width]
    }

    pub fn tlbr(&self) -> [f64; 4] {
        let Self {
            top,
            left,
            height,
            width,
        } = *self;
        let right = left + width;
        let bottom = top + height;
        [top, left, bottom, right]
    }

    pub fn cycxhw(&self) -> [f64; 4] {
        [self.center_y(), self.center_x(), self.height, self.width]
    }

    pub fn is_valid(&self) -> bool {
        self.width >= 0.0 && self.height >= 0.0
    }

    pub fn center_x(&self) -> f64 {
        self.left + self.width / 2.0
    }

    pub fn center_y(&self) -> f64 {
        self.top + self.height / 2.0
    }

    pub fn right(&self) -> f64 {
        self.left + self.width
    }

    pub fn bottom(&self) -> f64 {
        self.top + self.height
    }

    pub fn area(&self) -> f64 {
        self.height * self.width
    }

    pub fn intersection_area_with(&self, other: &Self) -> f64 {
        let left = self.left.max(other.left);
        let right = self.right().min(other.right());
        let top = self.top.max(other.top);
        let bottom = self.bottom().min(other.bottom());
        let width = right - left;
        let height = bottom - top;

        if width >= 0.0 && height >= 0.0 {
            width * height
        } else {
            0.0
        }
    }

    pub fn iou_with(&self, other: &Self) -> f64 {
        self.intersection_area_with(other) / (self.area() + other.area() + 1e-7)
    }
}

impl Image {
    pub fn format(&self) -> Result<ImageFormat> {
        ImageFormat::from_fourcc(&self.fourcc)
    }
}

impl RecordTime {
    pub fn timestamp_duration(&self) -> Duration {
        Duration::from_nanos(self.timestamp)
    }

    pub fn raw_timestamp_duration(&self) -> Option<Duration> {
        self.raw_timestamp.map(Duration::from_nanos)
    }
}

impl LidarPoint {
    pub fn xyz(&self) -> [f64; 3] {
        [self.x, self.y, self.z]
    }
}

pub use matcher_feedback::*;

mod matcher_feedback {
    use super::*;

    impl MatcherFeedback {
        pub fn get_accepted_devices(&self) -> HashSet<serde_types::DevicePath> {
            self.accepted_devices
                .iter()
                .map(|dev| dev.to_device_path())
                .collect()
        }

        pub fn to_upper_bound(&self) -> TimeUpperBound {
            match *self {
                Self {
                    inclusive: Some(inclusive),
                    accepted_max_timestamp: Some(accepted_max_timestamp),
                    ..
                } => {
                    let timestamp = Duration::from_nanos(accepted_max_timestamp);
                    if inclusive {
                        TimeUpperBound::Included(timestamp)
                    } else {
                        TimeUpperBound::Excluded(timestamp)
                    }
                }
                Self {
                    inclusive: None,
                    accepted_max_timestamp: None,
                    ..
                } => TimeUpperBound::None,
                _ => panic!("invalid message"),
            }
        }

        pub fn get_commit_timestamp(&self) -> Option<Duration> {
            Some(Duration::from_nanos(self.commit_timestamp?))
        }

        pub fn time_range(&self) -> (Bound<Duration>, Bound<Duration>) {
            let upper = match *self {
                Self {
                    inclusive: Some(included),
                    accepted_max_timestamp: Some(upper_ts),
                    ..
                } => {
                    let upper_ts = Duration::from_nanos(upper_ts);
                    if included {
                        Included(upper_ts)
                    } else {
                        Excluded(upper_ts)
                    }
                }
                Self {
                    inclusive: None,
                    accepted_max_timestamp: None,
                    ..
                } => Unbounded,
                _ => panic!("invalid message"),
            };
            let lower = match self.commit_timestamp {
                Some(ts) => Excluded(Duration::from_nanos(ts)),
                None => Unbounded,
            };
            (lower, upper)
        }
    }

    impl Eq for MatcherFeedback {}
}

impl UnitQuaternion {
    pub fn identity() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        }
    }
}

mod with_chrono {
    use super::*;

    impl RecordTime {
        pub fn timestamp_to_unix_naive_datetime(&self) -> NaiveDateTime {
            let secs = self.timestamp / 1_000_000_000;
            let nsecs = self.timestamp % 1_000_000_000;
            NaiveDateTime::from_timestamp_opt(secs as i64, nsecs as u32).unwrap()
        }

        pub fn raw_timestamp_to_unix_naive_datetime(&self) -> Option<NaiveDateTime> {
            let raw_timestamp = self.raw_timestamp?;
            let secs = raw_timestamp / 1_000_000_000;
            let nsecs = raw_timestamp % 1_000_000_000;
            Some(NaiveDateTime::from_timestamp_opt(secs as i64, nsecs as u32).unwrap())
        }
    }
}

#[cfg(feature = "with-nalgebra")]
mod with_nalgebra {
    use super::*;
    use crate::{BBox3D, BBoxPvrcnn, UnitQuaternion};
    use nalgebra as na;

    impl From<&BBoxPvrcnn> for BBox3D {
        fn from(from: &BBoxPvrcnn) -> Self {
            Self {
                center_x: from.x,
                center_y: from.y,
                center_z: from.z,
                size_x: from.dx,
                size_y: from.dy,
                size_z: from.dz,
                rotation: Some(UnitQuaternion::from(
                    na::geometry::UnitQuaternion::from_euler_angles(0.0, 0.0, from.heading),
                )),
            }
        }
    }

    impl From<&mut BBoxPvrcnn> for BBox3D {
        fn from(from: &mut BBoxPvrcnn) -> Self {
            Self {
                center_x: from.x,
                center_y: from.y,
                center_z: from.z,
                size_x: from.dx,
                size_y: from.dy,
                size_z: from.dz,
                rotation: Some(UnitQuaternion::from(
                    na::geometry::UnitQuaternion::from_euler_angles(0.0, 0.0, from.heading),
                )),
            }
        }
    }

    impl BBox2D {
        pub fn vertex(&self, x_choice: bool, y_choice: bool) -> na::Point2<f64> {
            let Self {
                width,
                height,
                left,
                top,
            } = *self;

            let x = if x_choice { left + width } else { left };
            let y = if y_choice { top + height } else { top };
            na::Point2::new(x, y)
        }
    }

    impl BBox3D {
        pub fn center(&self) -> na::Point3<f64> {
            let Self {
                center_x,
                center_y,
                center_z,
                ..
            } = *self;
            na::Point3::new(center_x, center_y, center_z)
        }

        pub fn vertex(&self, x_choice: bool, y_choice: bool, z_choice: bool) -> na::Point3<f64> {
            let point = {
                let x = self.size_x / 2.0 * if x_choice { 1.0 } else { -1.0 };
                let y = self.size_y / 2.0 * if y_choice { 1.0 } else { -1.0 };
                let z = self.size_z / 2.0 * if z_choice { 1.0 } else { -1.0 };
                na::Point3::new(x, y, z)
            };
            self.pose() * point
        }

        pub fn vertices(&self) -> Vec<na::Point3<f64>> {
            (0b000..=0b111)
                .map(|mask| self.vertex(mask & 0b001 != 0, mask & 0b010 != 0, mask & 0b100 != 0))
                .collect()
        }

        pub fn pose(&self) -> na::Isometry3<f64> {
            let rotation = self
                .rotation
                .as_ref()
                .map(Into::into)
                .unwrap_or_else(na::UnitQuaternion::identity);
            let translation = na::Translation3::new(self.center_x, self.center_y, self.center_z);
            na::Isometry3::from_parts(translation, rotation)
        }
    }

    impl BBoxPvrcnn {
        pub fn pose(&self) -> na::Isometry3<f64> {
            let rotation = na::UnitQuaternion::from_euler_angles(0.0, 0.0, self.heading);
            let translation = na::Translation3::new(self.x, self.y, self.z);
            na::Isometry3::from_parts(translation, rotation)
        }

        pub fn vertex(&self, x_choice: bool, y_choice: bool, z_choice: bool) -> na::Point3<f64> {
            let point = {
                let x = self.dx / 2.0 * if x_choice { 1.0 } else { -1.0 };
                let y = self.dy / 2.0 * if y_choice { 1.0 } else { -1.0 };
                let z = self.dz / 2.0 * if z_choice { 1.0 } else { -1.0 };
                na::Point3::new(x, y, z)
            };
            self.pose() * point
        }

        pub fn vertices(&self) -> Vec<na::Point3<f64>> {
            (0b000..=0b111)
                .map(|mask| self.vertex(mask & 0b001 != 0, mask & 0b010 != 0, mask & 0b100 != 0))
                .collect()
        }
    }

    impl LidarPoint {
        pub fn to_na_point(&self) -> na::Point3<f64> {
            let Self { x, y, z, .. } = *self;
            na::Point3::new(x, y, z)
        }
    }
}

#[cfg(feature = "with-opencv")]
mod with_opencv {
    use super::*;
    use crate::convert::{ColorSpace, MatWithColor};
    use opencv::{core as core_cv, imgproc};
    use std::convert::TryInto;

    impl Image {
        pub fn opencv_bgr_mat(&self) -> Result<core_cv::Mat> {
            let MatWithColor { mut image, color } = self.try_into()?;

            let image = match color {
                ColorSpace::Rgb => {
                    imgproc::cvt_color(
                        &image.clone(),
                        &mut image,
                        imgproc::COLOR_RGB2BGR,
                        0, // dst_cn
                    )?;
                    image
                }
                ColorSpace::Bgr => image,
            };

            Ok(image)
        }

        pub fn opencv_rgb_mat(&self) -> Result<core_cv::Mat> {
            let MatWithColor { mut image, color } = self.try_into()?;

            let image = match color {
                ColorSpace::Bgr => {
                    imgproc::cvt_color(
                        &image.clone(),
                        &mut image,
                        imgproc::COLOR_BGR2RGB,
                        0, // dst_cn
                    )?;
                    image
                }
                ColorSpace::Rgb => image,
            };

            Ok(image)
        }
    }
}
