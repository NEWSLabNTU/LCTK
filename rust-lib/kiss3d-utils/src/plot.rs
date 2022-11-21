use crate::common::*;

/// Graphics extension for kiss3d [Window].
pub trait WindowPlotExt {
    fn draw_axes<T>(&mut self, pose: T, length: f32)
    where
        T: Into<Isometry3<f32>>;

    fn draw_point<P1, P2>(&mut self, a: P1, color: P2)
    where
        P1: PlotPoint,
        P2: PlotPoint;

    fn draw_line_ext<P1, P2, P3>(&mut self, a: P1, b: P2, color: P3)
    where
        P1: PlotPoint,
        P2: PlotPoint,
        P3: PlotPoint;

    fn draw_box<P, T, C>(&mut self, size: P, pose: T, color: C)
    where
        P: PlotPoint,
        T: Into<Isometry3<f32>>,
        C: PlotPoint;

    fn draw_rect<H, W, T, C>(&mut self, size: (H, W), pose: T, color: C)
    where
        f32: SupersetOf<H>,
        f32: SupersetOf<W>,
        T: Into<Isometry3<f32>>,
        C: PlotPoint;
}

impl WindowPlotExt for Window {
    fn draw_axes<T>(&mut self, pose: T, length: f32)
    where
        T: Into<Isometry3<f32>>,
    {
        let pose = pose.into();

        let origin = pose * Point3::origin();
        let x_end = pose * Point3::new(length, 0.0, 0.0);
        let y_end = pose * Point3::new(0.0, length, 0.0);
        let z_end = pose * Point3::new(0.0, 0.0, length);

        self.draw_line(&origin, &x_end, &Point3::new(1.0, 0.0, 0.0));
        self.draw_line(&origin, &y_end, &Point3::new(0.0, 1.0, 0.0));
        self.draw_line(&origin, &z_end, &Point3::new(0.0, 0.0, 1.0));
    }

    fn draw_point<P1, P2>(&mut self, a: P1, color: P2)
    where
        P1: PlotPoint,
        P2: PlotPoint,
    {
        self.draw_point(&a.into_point3(), &color.into_point3())
    }

    fn draw_line_ext<P1, P2, P3>(&mut self, a: P1, b: P2, color: P3)
    where
        P1: PlotPoint,
        P2: PlotPoint,
        P3: PlotPoint,
    {
        self.draw_line(&a.into_point3(), &b.into_point3(), &color.into_point3())
    }

    fn draw_box<P, T, C>(&mut self, size: P, pose: T, color: C)
    where
        P: PlotPoint,
        T: Into<Isometry3<f32>>,
        C: PlotPoint,
    {
        let size = size.into_point3();
        let color = color.into_point3();
        let sx = size.x / 2.0;
        let sy = size.y / 2.0;
        let sz = size.z / 2.0;
        let pose = pose.into();

        [[sx, sy, sz], [-sx, -sy, sz], [sx, -sy, -sz], [-sx, sy, -sz]]
            .into_iter()
            .flat_map(|[px, py, pz]| {
                [
                    [Point3::new(px, py, pz), Point3::new(-px, py, pz)],
                    [Point3::new(px, py, pz), Point3::new(px, -py, pz)],
                    [Point3::new(px, py, pz), Point3::new(px, py, -pz)],
                ]
            })
            .for_each(|[a, b]| {
                let a = pose * a;
                let b = pose * b;
                self.draw_line(&a, &b, &color);
            });
    }

    fn draw_rect<H, W, T, C>(&mut self, size: (H, W), pose: T, color: C)
    where
        f32: SupersetOf<H>,
        f32: SupersetOf<W>,
        T: Into<Isometry3<f32>>,
        C: PlotPoint,
    {
        let (sx, sy) = size;
        let sx = f32::from_subset(&sx) / 2.0;
        let sy = f32::from_subset(&sy) / 2.0;
        let color = color.into_point3();
        let pose = pose.into();

        let vertices: Vec<_> = [[sx, sy], [-sx, sy], [-sx, -sy], [sx, -sy]]
            .into_iter()
            .map(|[sx, sy]| pose * na::Point3::new(sx, sy, 0.0))
            .collect();

        let iter = vertices.iter().cycle();
        izip!(iter.clone(), iter.skip(1))
            .take(4)
            .for_each(|(a, b)| {
                self.draw_line(a, b, &color);
            });
    }
}

pub use point::*;
mod point {
    use super::*;

    /// Generic 3D point that can be converted to [Point3<f32>](Point3).
    pub trait PlotPoint {
        fn into_point3(self) -> Point3<f32>;
    }

    impl<T> PlotPoint for Point3<T>
    where
        T: Scalar,
        f32: SupersetOf<T>,
    {
        fn into_point3(self) -> Point3<f32> {
            na::convert(self)
        }
    }

    impl<T> PlotPoint for &Point3<T>
    where
        T: Scalar,
        f32: SupersetOf<T>,
    {
        fn into_point3(self) -> Point3<f32> {
            na::convert_ref(self)
        }
    }

    impl<T1, T2, T3> PlotPoint for (T1, T2, T3)
    where
        f32: SupersetOf<T1>,
        f32: SupersetOf<T2>,
        f32: SupersetOf<T3>,
    {
        fn into_point3(self) -> Point3<f32> {
            let (c1, c2, c3) = self;
            Point3::new(
                f32::from_subset(&c1),
                f32::from_subset(&c2),
                f32::from_subset(&c3),
            )
        }
    }

    impl<T1, T2, T3> PlotPoint for &(T1, T2, T3)
    where
        f32: SupersetOf<T1>,
        f32: SupersetOf<T2>,
        f32: SupersetOf<T3>,
    {
        fn into_point3(self) -> Point3<f32> {
            let (c1, c2, c3) = self;
            Point3::new(
                f32::from_subset(c1),
                f32::from_subset(c2),
                f32::from_subset(c3),
            )
        }
    }

    impl<T> PlotPoint for [T; 3]
    where
        f32: SupersetOf<T>,
    {
        fn into_point3(self) -> Point3<f32> {
            (&self).into_point3()
        }
    }

    impl<T> PlotPoint for &[T; 3]
    where
        f32: SupersetOf<T>,
    {
        fn into_point3(self) -> Point3<f32> {
            let [c1, c2, c3] = self;
            Point3::new(
                f32::from_subset(c1),
                f32::from_subset(c2),
                f32::from_subset(c3),
            )
        }
    }
}
