use crate::model::PlaneModel;
use nalgebra as na;
use sample_consensus::Estimator;

#[derive(Debug, Clone, Default)]
pub struct PlaneEstimator {
    _private: [u8; 0],
}

impl PlaneEstimator {
    pub fn new() -> Self {
        Self::default()
    }
}

impl Estimator<na::Point3<f64>> for PlaneEstimator {
    type Model = PlaneModel;
    type ModelIter = Option<Self::Model>;
    const MIN_SAMPLES: usize = 3;

    fn estimate<I>(&self, mut data: I) -> Self::ModelIter
    where
        I: Iterator<Item = na::Point3<f64>> + Clone,
    {
        let center = data.next().unwrap();
        let p1 = data.next().unwrap();
        let vec1 = p1 - center;

        // compute plane parameters
        let mut p2 = data.next().unwrap();
        let normal = loop {
            let vec2 = p2 - center;
            let vec2 = vec2.cross(&vec1);
            let vec2_norm = vec2.norm();
            if vec2_norm <= 1e-7 {
                p2 = match data.next() {
                    Some(point) => point,
                    None => return None,
                };
                continue;
            }

            break na::Unit::new_normalize(vec2);
        };

        // if there are remaining data points, check if the plane fits.
        for point in data {
            let vec = point - center;
            let cos = vec.dot(&normal) / vec.norm();
            if cos >= 1e-8 {
                return None;
            }
        }

        let model = PlaneModel { center, normal };

        Some(model)
    }
}
