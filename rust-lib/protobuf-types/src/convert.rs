pub use native::*;

mod native {
    use crate::{DevicePath, HorizontalCoord, LidarPoint, Point3D};
    use anyhow::Error;
    use std::{
        cmp::Ordering::*,
        f64::consts::{FRAC_PI_2, PI},
    };

    impl From<&Point3D> for HorizontalCoord {
        fn from(from: &Point3D) -> Self {
            let Point3D { x, y, z } = *from;
            let distance = (x.powi(2) + y.powi(2) + z.powi(2)).sqrt();
            let altitude = z.atan2((x.powi(2) + y.powi(2)).sqrt());
            let azimuth = match (x.partial_cmp(&0.0), y.partial_cmp(&0.0)) {
                (None, _) | (_, None) => 0.0,
                (Some(Greater), Some(Greater)) => PI * 2.0 - y.atan2(x),
                (Some(Greater), Some(Equal)) => 0.0,
                (Some(Greater), Some(Less)) => -y.atan2(x),
                (Some(Equal), Some(Greater)) => PI + FRAC_PI_2,
                (Some(Equal), Some(Equal)) => 0.0,
                (Some(Equal), Some(Less)) => FRAC_PI_2,
                (Some(Less), _) => PI - y.atan2(x),
            };
            Self {
                distance,
                altitude,
                azimuth,
            }
        }
    }

    impl From<Point3D> for HorizontalCoord {
        fn from(from: Point3D) -> Self {
            Self::from(&from)
        }
    }

    impl From<&HorizontalCoord> for Point3D {
        fn from(from: &HorizontalCoord) -> Self {
            let HorizontalCoord {
                distance,
                azimuth,
                altitude,
            } = *from;

            let z = distance * altitude.sin();
            let planar_dist = distance * altitude.cos();
            let x = planar_dist * azimuth.cos();
            let y = -planar_dist * azimuth.sin();

            Self { x, y, z }
        }
    }

    impl From<HorizontalCoord> for Point3D {
        fn from(from: HorizontalCoord) -> Self {
            Self::from(&from)
        }
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
    pub enum ColorSpace {
        Rgb,
        Bgr,
    }

    impl From<&LidarPoint> for Point3D {
        fn from(from: &LidarPoint) -> Self {
            let LidarPoint { x, y, z, .. } = *from;
            Self { x, y, z }
        }
    }

    impl From<LidarPoint> for Point3D {
        fn from(from: LidarPoint) -> Self {
            (&from).into()
        }
    }

    impl TryFrom<&DevicePath> for serde_types::DevicePath {
        type Error = Error;

        fn try_from(from: &DevicePath) -> Result<Self, Self::Error> {
            let DevicePath { host, device } = from;
            Self::try_new(host, device)
        }
    }

    impl From<serde_types::DevicePath> for DevicePath {
        fn from(from: serde_types::DevicePath) -> Self {
            Self {
                host: from.host().to_string(),
                device: from.device().to_string(),
            }
        }
    }

    impl From<&serde_types::DevicePath> for DevicePath {
        fn from(from: &serde_types::DevicePath) -> Self {
            Self {
                host: from.host().to_string(),
                device: from.device().to_string(),
            }
        }
    }

    #[cfg(test)]
    mod tests {
        use super::*;
        use approx::assert_abs_diff_eq;
        use rand::prelude::*;

        #[test]
        fn cartesian_horizontal_coord_convert() {
            let mut rng = rand::thread_rng();

            for _ in 0..10000 {
                let [x, y, z]: [f64; 3] = rng.gen();
                let orig = Point3D { x, y, z };
                let sph: HorizontalCoord = (&orig).into();
                let recon: Point3D = (&sph).into();

                assert_abs_diff_eq!(orig.x, recon.x, epsilon = 1e-8);
                assert_abs_diff_eq!(orig.y, recon.y, epsilon = 1e-8);
                assert_abs_diff_eq!(orig.z, recon.z, epsilon = 1e-8);
            }
        }
    }
}

#[cfg(feature = "with-opencv")]
pub use with_opencv::*;
#[cfg(feature = "with-opencv")]
mod with_opencv {
    use super::ColorSpace;
    use crate::{ext::ImageFormat, Image, LidarPoint, Point3D};
    use anyhow::{ensure, Error, Result};
    use opencv::{core, imgproc, prelude::*};
    use std::convert::TryFrom;

    impl From<&Point3D> for core::Point3d {
        fn from(from: &Point3D) -> Self {
            let Point3D { x, y, z, .. } = *from;
            Self::new(x, y, z)
        }
    }

    impl From<Point3D> for core::Point3d {
        fn from(from: Point3D) -> Self {
            Self::from(&from)
        }
    }

    impl From<&LidarPoint> for core::Point3d {
        fn from(from: &LidarPoint) -> Self {
            let LidarPoint { x, y, z, .. } = *from;
            Self::new(x, y, z)
        }
    }

    impl From<LidarPoint> for core::Point3d {
        fn from(from: LidarPoint) -> Self {
            Self::from(&from)
        }
    }

    impl TryFrom<&Image> for MatWithColor {
        type Error = Error;

        fn try_from(image: &Image) -> Result<Self> {
            let format = image.format()?;
            let Image {
                ref data,
                width,
                height,
                channels,
                ..
            } = *image;

            let (mat, color) = match format {
                ImageFormat::RGB3 => {
                    ensure!(data.len() == (height * width * channels) as usize);
                    ensure!(channels == 3);
                    let mat =
                        core::Mat::from_slice(data)?.reshape(channels as i32, height as i32)?;
                    (mat, ColorSpace::Rgb)
                }
                ImageFormat::BGR3 => {
                    ensure!(data.len() == (height * width * channels) as usize);
                    ensure!(channels == 3);
                    let mat =
                        core::Mat::from_slice(data)?.reshape(channels as i32, height as i32)?;
                    (mat, ColorSpace::Bgr)
                }
            };
            Ok(Self { image: mat, color })
        }
    }

    impl TryFrom<Image> for MatWithColor {
        type Error = Error;

        fn try_from(image: Image) -> Result<Self> {
            TryFrom::try_from(&image)
        }
    }

    impl TryFrom<&MatWithColor> for Image {
        type Error = Error;

        fn try_from(image: &MatWithColor) -> Result<Self> {
            let MatWithColor { ref image, color } = *image;
            let height = image.rows();
            let width = image.cols();

            let (format, channels, data) = match color {
                ColorSpace::Rgb => {
                    let pixels: &[core::Vec3b] = image.data_typed()?;
                    debug_assert!(pixels.len() == (height * width) as usize);
                    let bytes: Vec<u8> =
                        pixels.iter().flat_map(|pixel| Vec::from(**pixel)).collect();
                    (ImageFormat::RGB3, 3, bytes)
                }
                ColorSpace::Bgr => {
                    let pixels: &[core::Vec3b] = image.data_typed()?;
                    debug_assert!(pixels.len() == (height * width) as usize);
                    let bytes: Vec<u8> =
                        pixels.iter().flat_map(|pixel| Vec::from(**pixel)).collect();
                    (ImageFormat::BGR3, 3, bytes)
                }
            };

            Ok(Self {
                fourcc: format.to_fourcc().to_vec(),
                height: height as u64,
                width: width as u64,
                channels,
                data,
            })
        }
    }

    impl TryFrom<MatWithColor> for Image {
        type Error = Error;

        fn try_from(image: MatWithColor) -> Result<Self> {
            TryFrom::try_from(&image)
        }
    }

    #[derive(Debug, Clone)]
    pub struct MatWithColor {
        pub image: core::Mat,
        pub color: ColorSpace,
    }

    impl MatWithColor {
        pub fn to_bgr(&self) -> Result<Self> {
            let Self {
                image: input,
                color,
            } = self;
            let output = match color {
                ColorSpace::Bgr => input.clone(),
                ColorSpace::Rgb => {
                    let mut output = Mat::default();
                    imgproc::cvt_color(
                        input,
                        &mut output,
                        imgproc::COLOR_RGB2BGR,
                        0, // dst_cn
                    )?;
                    output
                }
            };
            Ok(Self {
                image: output,
                color: ColorSpace::Bgr,
            })
        }

        pub fn to_rgb(&self) -> Result<Self> {
            let Self {
                image: input,
                color,
            } = self;
            let output = match color {
                ColorSpace::Rgb => input.clone(),
                ColorSpace::Bgr => {
                    let mut output = Mat::default();
                    imgproc::cvt_color(
                        input,
                        &mut output,
                        imgproc::COLOR_BGR2RGB,
                        0, // dst_cn
                    )?;
                    output
                }
            };
            Ok(Self {
                image: output,
                color: ColorSpace::Rgb,
            })
        }

        pub fn into_bgr(self) -> Result<Self> {
            let Self {
                image: input,
                color,
            } = self;
            let output = match color {
                ColorSpace::Bgr => input,
                ColorSpace::Rgb => {
                    let mut output = Mat::default();
                    imgproc::cvt_color(
                        &input,
                        &mut output,
                        imgproc::COLOR_RGB2BGR,
                        0, // dst_cn
                    )?;
                    output
                }
            };
            Ok(Self {
                image: output,
                color: ColorSpace::Bgr,
            })
        }

        pub fn into_rgb(self) -> Result<Self> {
            let Self {
                image: input,
                color,
            } = self;
            let output = match color {
                ColorSpace::Rgb => input,
                ColorSpace::Bgr => {
                    let mut output = Mat::default();
                    imgproc::cvt_color(
                        &input,
                        &mut output,
                        imgproc::COLOR_BGR2RGB,
                        0, // dst_cn
                    )?;
                    output
                }
            };
            Ok(Self {
                image: output,
                color: ColorSpace::Rgb,
            })
        }
    }

    #[cfg(test)]
    mod tests {
        use super::*;
        use rand::prelude::*;
        use std::convert::TryInto;

        #[test]
        fn convert_image_opencv() -> Result<()> {
            let mut rng = rand::thread_rng();
            let height = 64;
            let width = 48;
            let channels = 3;

            let mut buf = vec![0u8; height * width * channels];
            rng.fill(&mut *buf);

            let mat = core::Mat::from_slice(&buf)?.reshape(channels as i32, height as i32)?;
            ensure!(
                mat.rows() == height as i32
                    && mat.cols() == width as i32
                    && mat.channels() == channels as i32
            );

            let input = MatWithColor {
                image: mat,
                color: ColorSpace::Bgr,
            };
            let proto_image: Image = (&input).try_into()?;
            let output: MatWithColor = proto_image.try_into()?;

            ensure!(
                output.image.rows() == height as i32
                    && output.image.cols() == width as i32
                    && output.image.channels() == channels as i32
                    && output.color == input.color
            );
            ensure!(
                input.image.data_typed::<core::Vec3b>()?
                    == output.image.data_typed::<core::Vec3b>()?
            );

            Ok(())
        }
    }
}

#[cfg(feature = "with-image")]
pub use with_image::*;
#[cfg(feature = "with-image")]
mod with_image {
    use crate::{ext::ImageFormat, Image};
    use anyhow::{bail, ensure, Error, Result};
    use image::{flat::SampleLayout, ColorType, DynamicImage, FlatSamples, Rgb, RgbImage};
    use std::convert::{TryFrom, TryInto};

    impl TryFrom<Image> for FlatSamples<Vec<u8>> {
        type Error = Error;

        fn try_from(image: Image) -> Result<Self> {
            let Image {
                fourcc,
                width,
                height,
                channels,
                data: image_data,
                ..
            } = image;

            let samples = match ImageFormat::from_fourcc(&fourcc)? {
                ImageFormat::RGB3 => FlatSamples {
                    samples: image_data,
                    layout: SampleLayout {
                        channels: channels as u8,
                        channel_stride: 1,
                        width: width as u32,
                        width_stride: channels as usize,
                        height: height as u32,
                        height_stride: (width * channels) as usize,
                    },
                    color_hint: Some(ColorType::Rgb8),
                },
                ImageFormat::BGR3 => bail!("BGR3 is not supported"),
            };

            Ok(samples)
        }
    }

    impl<'a> TryFrom<&'a Image> for DynamicImage {
        type Error = Error;

        fn try_from(image: &'a Image) -> Result<Self> {
            let Image {
                ref fourcc,
                data: ref image_data,
                width,
                height,
                channels,
                ..
            } = *image;

            let buffer = match ImageFormat::from_fourcc(fourcc)? {
                ImageFormat::RGB3 => {
                    let stride = width * channels;
                    ensure!(channels == 3, "the channel must be 3 for RGB image");
                    ensure!(
                        image_data.len() == height as usize * stride as usize,
                        "the size of image buffer is not correct, expect {} but found {}",
                        height as usize * stride as usize,
                        image_data.len()
                    );

                    let rgb_image = RgbImage::from_fn(width as u32, height as u32, |x, y| {
                        let offset =
                            (y as usize * stride as usize) + x as usize * channels as usize;
                        let [r, g, b] = match image_data[offset..(offset + 3)] {
                            [r, g, b] => [r, g, b],
                            _ => unreachable!(),
                        };
                        Rgb::from([r, g, b])
                    });
                    DynamicImage::ImageRgb8(rgb_image)
                }
                ImageFormat::BGR3 => {
                    bail!("BGR3 is not supported");
                }
            };

            Ok(buffer)
        }
    }

    impl TryFrom<Image> for DynamicImage {
        type Error = Error;

        fn try_from(from: Image) -> Result<Self> {
            (&from).try_into()
        }
    }

    impl<'a> TryFrom<&'a Image> for FlatSamples<&'a [u8]> {
        type Error = Error;

        fn try_from(image: &'a Image) -> Result<Self> {
            let Image {
                ref fourcc,
                width,
                height,
                channels,
                data: ref image_data,
            } = *image;

            let samples = match ImageFormat::from_fourcc(fourcc)? {
                ImageFormat::RGB3 => FlatSamples {
                    samples: &**image_data,
                    layout: SampleLayout {
                        channels: channels as u8,
                        channel_stride: 1,
                        width: width as u32,
                        width_stride: channels as usize,
                        height: height as u32,
                        height_stride: (width * channels) as usize,
                    },
                    color_hint: Some(ColorType::Rgb8),
                },
                ImageFormat::BGR3 => bail!("BGR3 is not supported"),
            };

            Ok(samples)
        }
    }
}

#[cfg(feature = "with-nalgebra")]
pub use with_nalgebra::*;
#[cfg(feature = "with-nalgebra")]
mod with_nalgebra {
    use crate::{Isometry3D, LidarPoint, Point3D, UnitQuaternion, Vector3D};
    use nalgebra as na;

    impl From<&Point3D> for na::Point3<f64> {
        fn from(from: &Point3D) -> Self {
            Self::new(from.x, from.y, from.z)
        }
    }

    impl From<Point3D> for na::Point3<f64> {
        fn from(from: Point3D) -> Self {
            From::from(&from)
        }
    }

    impl From<&na::Point3<f64>> for Point3D {
        fn from(from: &na::Point3<f64>) -> Self {
            Self {
                x: from.x,
                y: from.y,
                z: from.z,
            }
        }
    }

    impl From<na::Point3<f64>> for Point3D {
        fn from(from: na::Point3<f64>) -> Self {
            From::from(&from)
        }
    }

    impl From<&LidarPoint> for na::Point3<f64> {
        fn from(from: &LidarPoint) -> Self {
            Self::new(from.x, from.y, from.z)
        }
    }

    impl From<LidarPoint> for na::Point3<f64> {
        fn from(from: LidarPoint) -> Self {
            From::from(&from)
        }
    }

    impl From<&Isometry3D> for na::Isometry3<f64> {
        fn from(from: &Isometry3D) -> Self {
            let Isometry3D {
                rotation,
                translation,
            } = from;
            Self::from_parts(translation.into(), rotation.into())
        }
    }

    impl From<Isometry3D> for na::Isometry3<f64> {
        fn from(from: Isometry3D) -> Self {
            (&from).into()
        }
    }

    impl From<&na::Isometry3<f64>> for Isometry3D {
        fn from(from: &na::Isometry3<f64>) -> Self {
            let na::Isometry3 {
                rotation,
                translation,
            } = from;
            Self {
                translation: translation.into(),
                rotation: rotation.into(),
            }
        }
    }

    impl From<na::Isometry3<f64>> for Isometry3D {
        fn from(from: na::Isometry3<f64>) -> Self {
            (&from).into()
        }
    }

    impl From<&UnitQuaternion> for na::UnitQuaternion<f64> {
        fn from(from: &UnitQuaternion) -> Self {
            let UnitQuaternion { x, y, z, w } = *from;
            Self::new_normalize(na::Quaternion::new(w, x, y, z))
        }
    }

    impl From<UnitQuaternion> for na::UnitQuaternion<f64> {
        fn from(from: UnitQuaternion) -> Self {
            (&from).into()
        }
    }

    impl From<&na::UnitQuaternion<f64>> for UnitQuaternion {
        fn from(from: &na::UnitQuaternion<f64>) -> Self {
            Self {
                x: from.i,
                y: from.j,
                z: from.k,
                w: from.w,
            }
        }
    }

    impl From<na::UnitQuaternion<f64>> for UnitQuaternion {
        fn from(from: na::UnitQuaternion<f64>) -> Self {
            (&from).into()
        }
    }

    impl From<&Vector3D> for na::Translation3<f64> {
        fn from(from: &Vector3D) -> Self {
            let Vector3D { x, y, z } = *from;
            Self::new(x, y, z)
        }
    }

    impl From<Vector3D> for na::Translation3<f64> {
        fn from(from: Vector3D) -> Self {
            (&from).into()
        }
    }

    impl From<&na::Translation3<f64>> for Vector3D {
        fn from(from: &na::Translation3<f64>) -> Self {
            Self {
                x: from.vector.x,
                y: from.vector.y,
                z: from.vector.z,
            }
        }
    }

    impl From<na::Translation3<f64>> for Vector3D {
        fn from(from: na::Translation3<f64>) -> Self {
            (&from).into()
        }
    }
}
