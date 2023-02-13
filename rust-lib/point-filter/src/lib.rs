use nalgebra as na;
use noisy_float::prelude::*;
use serde::{Deserialize, Serialize};
use std::ops::{Bound, Range, RangeBounds as _};

pub use filter::*;
mod filter {
    use super::*;

    /// The generic point cloud filter.
    #[derive(Debug, Clone, Serialize, Deserialize)]
    #[serde(tag = "type", rename_all = "snake_case")]
    pub enum PointFilter {
        PlanarBox(PlanarBoxFilter),
        Intensity(IntensityFilter),
        All(All),
        Any(Any),
        Not(Not),
        True,
        False,
    }

    impl PointFilter {
        pub fn contains(&self, point: &na::Point3<f64>, intensity: Option<f64>) -> bool {
            match self {
                Self::PlanarBox(filter) => filter.contains(point),
                Self::Intensity(filter) => filter.contains(intensity),
                Self::All(filter) => filter.contains(point, intensity),
                Self::Any(filter) => filter.contains(point, intensity),
                Self::Not(filter) => filter.contains(point, intensity),
                Self::True => true,
                Self::False => false,
            }
        }
    }

    impl From<IntensityFilter> for PointFilter {
        fn from(v: IntensityFilter) -> Self {
            Self::Intensity(v)
        }
    }

    impl From<PlanarBoxFilter> for PointFilter {
        fn from(v: PlanarBoxFilter) -> Self {
            Self::PlanarBox(v)
        }
    }

    impl From<All> for PointFilter {
        fn from(v: All) -> Self {
            Self::All(v)
        }
    }

    impl From<Any> for PointFilter {
        fn from(v: Any) -> Self {
            Self::Any(v)
        }
    }

    impl From<Not> for PointFilter {
        fn from(v: Not) -> Self {
            Self::Not(v)
        }
    }
}

pub use inclusive_box_filter::*;
mod inclusive_box_filter {
    use super::*;

    #[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
    pub struct PlanarBoxFilterConfig {
        #[serde(with = "common_types::serde_bound")]
        pub z_bound: (Bound<R64>, Bound<R64>),
        pub size_x: R64,
        pub size_y: R64,
        pub center_x: R64,
        pub center_y: R64,
        pub azimuth_degrees: R64,
    }

    #[derive(Debug, Clone, Serialize, Deserialize)]
    #[serde(from = "PlanarBoxFilterConfig", into = "PlanarBoxFilterConfig")]
    pub struct PlanarBoxFilter {
        pose: na::Isometry3<f64>,
        inverse_pose: na::Isometry3<f64>,
        size_x: f64,
        size_y: f64,
        #[serde(with = "common_types::serde_bound")]
        pub z_bound: (Bound<R64>, Bound<R64>),
    }

    impl PlanarBoxFilter {
        pub fn size_x(&self) -> f64 {
            self.size_x
        }

        pub fn size_y(&self) -> f64 {
            self.size_y
        }

        pub fn pose(&self) -> &na::Isometry3<f64> {
            &self.pose
        }

        pub fn inverse_pose(&self) -> &na::Isometry3<f64> {
            &self.inverse_pose
        }

        pub fn z_bound(&self) -> &(Bound<R64>, Bound<R64>) {
            &self.z_bound
        }

        pub fn contains(&self, point: &na::Point3<f64>) -> bool {
            let point = self.inverse_pose * point;
            self.z_bound.contains(&r64(point.z))
                && point.x >= -self.size_x / 2.0
                && point.x <= self.size_x / 2.0
                && point.y >= -self.size_y / 2.0
                && point.y <= self.size_y / 2.0
        }
    }

    impl From<PlanarBoxFilterConfig> for PlanarBoxFilter {
        fn from(config: PlanarBoxFilterConfig) -> Self {
            let pose = {
                let trans =
                    na::Translation3::new(config.center_x.raw(), config.center_y.raw(), 0.0);
                let rot = na::UnitQuaternion::from_euler_angles(
                    0.0,
                    0.0,
                    config.azimuth_degrees.raw().to_radians(),
                );
                na::Isometry3::from_parts(trans, rot)
            };

            Self {
                pose,
                inverse_pose: pose.inverse(),
                size_x: config.size_x.raw(),
                size_y: config.size_y.raw(),
                z_bound: config.z_bound,
            }
        }
    }

    impl From<PlanarBoxFilter> for PlanarBoxFilterConfig {
        fn from(from: PlanarBoxFilter) -> Self {
            let PlanarBoxFilter {
                pose:
                    na::Isometry3 {
                        rotation,
                        translation,
                    },
                size_x,
                size_y,
                z_bound,
                ..
            } = from;

            Self {
                z_bound,
                size_x: r64(size_x),
                size_y: r64(size_y),
                center_x: r64(translation.x),
                center_y: r64(translation.y),
                azimuth_degrees: r64(rotation.euler_angles().2),
            }
        }
    }
}

pub use intensity_filter::*;
mod intensity_filter {
    use super::*;

    #[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
    pub struct IntensityFilterConfig {
        pub min: R64,
        pub max: Option<R64>,
    }

    #[derive(Debug, Clone, Serialize, Deserialize)]
    #[serde(from = "IntensityFilterConfig", into = "IntensityFilterConfig")]
    pub struct IntensityFilter {
        range: Range<N64>,
    }

    impl IntensityFilter {
        pub fn contains(&self, intensity: Option<f64>) -> bool {
            // TODO: change intensity to f64 type
            // true if intensity is not defined
            let intensity = if let Some(intensity) = intensity {
                intensity
            } else {
                return true;
            };

            // reject NaN
            let intensity = if let Some(intensity) = N64::try_new(intensity) {
                intensity
            } else {
                return false;
            };

            self.range.contains(&intensity)
        }
    }

    impl From<IntensityFilterConfig> for IntensityFilter {
        fn from(config: IntensityFilterConfig) -> Self {
            let min = config.min.raw();
            let max = config.max.map(|max| max.raw()).unwrap_or(f64::INFINITY);

            let min = n64(min);
            let max = n64(max);

            let range = min..max;
            Self { range }
        }
    }

    impl From<IntensityFilter> for IntensityFilterConfig {
        fn from(config: IntensityFilter) -> Self {
            let min = r64(config.range.start.raw());
            let max = {
                let max = config.range.end;
                max.is_finite().then(|| r64(max.raw()))
            };
            Self { min, max }
        }
    }
}

pub use combinators::*;
mod combinators {
    use super::*;

    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct All {
        pub filters: Vec<PointFilter>,
    }

    impl All {
        pub fn contains(&self, point: &na::Point3<f64>, intensity: Option<f64>) -> bool {
            self.filters
                .iter()
                .all(|filter| filter.contains(point, intensity))
        }
    }

    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct Any {
        pub filters: Vec<PointFilter>,
    }

    impl Any {
        pub fn contains(&self, point: &na::Point3<f64>, intensity: Option<f64>) -> bool {
            self.filters
                .iter()
                .any(|filter| filter.contains(point, intensity))
        }
    }

    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct Not {
        pub filter: Box<PointFilter>,
    }

    impl Not {
        pub fn contains(&self, point: &na::Point3<f64>, intensity: Option<f64>) -> bool {
            !self.filter.contains(point, intensity)
        }
    }
}

#[cfg(feature = "with-kiss3d")]
mod with_kiss3d {
    use super::*;
    use kiss3d::window::Window;
    use kiss3d_utils::WindowPlotExt as _;
    use std::ops::Bound::*;

    impl PointFilter {
        pub fn render_kiss3d(&self, window: &mut Window) {
            render_recursive(self, window, true);
        }
    }

    fn render_recursive(filter: &PointFilter, window: &mut Window, is_positive: bool) {
        use PointFilter as F;

        let pos_color = na::Point3::new(0.0, 1.0, 0.0);
        let neg_color = na::Point3::new(1.0, 0.0, 0.0);

        let major_color = if is_positive { &pos_color } else { &neg_color };

        match filter {
            F::PlanarBox(pbox) => {
                draw_planar_box(window, pbox, major_color);
            }
            F::Intensity(_) => {}
            F::All(all) => {
                all.filters.iter().for_each(|filter| {
                    render_recursive(filter, window, is_positive);
                });
            }
            F::Any(any) => {
                any.filters.iter().for_each(|filter| {
                    render_recursive(filter, window, is_positive);
                });
            }
            F::Not(Not { filter }) => {
                render_recursive(filter, window, !is_positive);
            }
            F::True => {}
            F::False => {}
        }
    }

    fn draw_planar_box(window: &mut Window, filter: &PlanarBoxFilter, color: &na::Point3<f32>) {
        let pose: na::Isometry3<f32> = na::convert_ref(filter.pose());
        let size_x = filter.size_x();
        let size_y = filter.size_y();
        let z_bound = filter.z_bound();

        match (z_bound.start_bound(), z_bound.end_bound()) {
            (Unbounded, Unbounded) => {}
            (Included(_) | Excluded(_), Unbounded) | (Unbounded, Included(_) | Excluded(_)) => {
                window.draw_rect((size_x, size_y), pose, color);
            }
            (Included(z_min) | Excluded(z_min), Included(z_max) | Excluded(z_max)) => {
                let size = na::Point3::new(size_x, size_y, (*z_max - *z_min).raw());
                window.draw_box(size, pose, color);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::ops::Bound::*;

    #[test]
    fn inclusive_box_filter() {
        let filter: PlanarBoxFilter = PlanarBoxFilterConfig {
            z_bound: (Unbounded, Unbounded),
            size_x: r64(5.0),
            size_y: r64(10.0),
            center_x: r64(1.0),
            center_y: r64(2.0),
            azimuth_degrees: r64(0.0),
        }
        .into();
        assert!(filter.contains(&na::Point3::new(3.4, 2.0, 0.0)));
        assert!(!filter.contains(&na::Point3::new(3.6, 2.0, 0.0)));
    }
}
