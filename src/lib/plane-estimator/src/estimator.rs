use crate::model::PlaneModel;
use nalgebra as na;
use sample_consensus::Estimator;
use simba::scalar::SubsetOf;

#[derive(Debug, Clone, Default)]
pub struct PlaneEstimator {
    _private: (),
}

impl PlaneEstimator {
    pub fn new() -> Self {
        Self::default()
    }
}

impl<T> Estimator<na::Point3<T>> for PlaneEstimator
where
    T: na::Scalar + na::ClosedSub + SubsetOf<f64>,
{
    type Model = PlaneModel;
    type ModelIter = Option<Self::Model>;

    const MIN_SAMPLES: usize = 3;

    fn estimate<I>(&self, data: I) -> Self::ModelIter
    where
        I: Iterator<Item = na::Point3<T>> + Clone,
    {
        let mut data = data.map(|point| {
            let point: na::Point3<f64> = na::convert(point);
            point
        });

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

impl<'a, T> Estimator<&'a na::Point3<T>> for PlaneEstimator
where
    T: na::Scalar + na::ClosedSub + SubsetOf<f64>,
{
    type Model = PlaneModel;
    type ModelIter = Option<Self::Model>;

    const MIN_SAMPLES: usize = 3;

    fn estimate<I>(&self, data: I) -> Self::ModelIter
    where
        I: Iterator<Item = &'a na::Point3<T>> + Clone,
    {
        self.estimate(data.cloned())
    }
}

impl<T> Estimator<[T; 3]> for PlaneEstimator
where
    T: na::Scalar + na::ClosedSub + SubsetOf<f64> + Copy,
{
    type Model = PlaneModel;
    type ModelIter = Option<Self::Model>;

    const MIN_SAMPLES: usize = 3;

    fn estimate<I>(&self, data: I) -> Self::ModelIter
    where
        I: Iterator<Item = [T; 3]> + Clone,
    {
        let iter = data.map(|[x, y, z]| na::Point3::new(x, y, z));
        self.estimate(iter)
    }
}

impl<'a, T> Estimator<&'a [T; 3]> for PlaneEstimator
where
    T: na::Scalar + na::ClosedSub + SubsetOf<f64> + Copy,
{
    type Model = PlaneModel;
    type ModelIter = Option<Self::Model>;

    const MIN_SAMPLES: usize = 3;

    fn estimate<I>(&self, data: I) -> Self::ModelIter
    where
        I: Iterator<Item = &'a [T; 3]> + Clone,
    {
        let iter = data.map(|&[x, y, z]| na::Point3::new(x, y, z));
        self.estimate(iter)
    }
}
