use nalgebra as na;
use sample_consensus::Model;

#[derive(Debug, Clone)]
pub struct PlaneModel {
    pub center: na::Point3<f64>,
    pub normal: na::Unit<na::Vector3<f64>>,
}

impl Model<na::Point3<f64>> for PlaneModel {
    fn residual(&self, point: &na::Point3<f64>) -> f64 {
        let vec = point - self.center;
        vec.dot(&self.normal).abs()
    }
}
