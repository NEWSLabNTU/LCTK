use na::coordinates::XYZ;
use nalgebra as na;
use sample_consensus::Model;
use simba::scalar::SubsetOf;

#[derive(Debug, Clone)]
pub struct PlaneModel {
    pub center: na::Point3<f64>,
    pub normal: na::Unit<na::Vector3<f64>>,
}

impl PlaneModel {
    pub fn pose(&self) -> na::Isometry3<f64> {
        let rotation = na::UnitQuaternion::from_axis_angle(&self.normal, 0.0);
        let translation = {
            let XYZ { x, y, z } = *self.center;
            na::Translation3::new(x, y, z)
        };
        na::Isometry3::from_parts(translation, rotation)
    }
}

impl<T> Model<na::Point3<T>> for PlaneModel
where
    T: na::Scalar + na::ClosedSub + SubsetOf<f64>,
{
    fn residual(&self, point: &na::Point3<T>) -> f64 {
        let point: na::Point3<f64> = na::convert_ref(point);
        let vec = point - self.center;
        vec.dot(&self.normal).abs()
    }
}

impl<T> Model<&na::Point3<T>> for PlaneModel
where
    T: na::Scalar + na::ClosedSub + SubsetOf<f64>,
{
    fn residual(&self, point: &&na::Point3<T>) -> f64 {
        self.residual(*point)
    }
}

impl<T> Model<[T; 3]> for PlaneModel
where
    T: na::Scalar + na::ClosedSub + SubsetOf<f64> + Copy,
{
    fn residual(&self, &[x, y, z]: &[T; 3]) -> f64 {
        let point = na::Point3::new(x, y, z);
        self.residual(&point)
    }
}

impl<T> Model<&[T; 3]> for PlaneModel
where
    T: na::Scalar + na::ClosedSub + SubsetOf<f64> + Copy,
{
    fn residual(&self, &&[x, y, z]: &&[T; 3]) -> f64 {
        let point = na::Point3::new(x, y, z);
        self.residual(&point)
    }
}
