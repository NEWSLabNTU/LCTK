use collected::{AddVal, SumVal};
use nalgebra as na;
use std::borrow::Borrow;
unzip_n::unzip_n!(2);
unzip_n::unzip_n!(4);

/// Computes the optimal rotation from a point set to target point set.
///
/// The function accepts an iterator of point pairs `(P, Q)`,
/// where `Q` is target point for `P`.
pub fn kabsch<P, Q>(
    input_target_pairs: impl IntoIterator<Item = (P, Q)>,
) -> Option<na::Isometry3<f64>>
where
    P: Borrow<na::Point3<f64>>,
    Q: Borrow<na::Point3<f64>>,
{
    let (input_points, target_points) = input_target_pairs.into_iter().unzip_n_vec();

    // compute centroids
    let input_centroid = centroid(input_points.iter().map(Borrow::borrow))?;
    let target_centroid = centroid(target_points.iter().map(Borrow::borrow))?;

    // translate centroid to origin
    let input_points: Vec<_> = input_points
        .into_iter()
        .map(|point| point.borrow() - input_centroid)
        .collect();
    let target_points: Vec<_> = target_points
        .into_iter()
        .map(|point| point.borrow() - target_centroid)
        .collect();

    let input_matrix = na::Matrix3xX::from_columns(&*input_points);
    let target_matrix = na::Matrix3xX::from_columns(&*target_points);
    let covariance = input_matrix * target_matrix.transpose();

    let svd = na::SVD::new(covariance, true, true);
    let svd_u = svd.u.unwrap();
    let svd_v_t = svd.v_t.unwrap();

    let d_sign = (svd_u * svd_v_t).determinant().signum();
    let rotation_t =
        svd_u * na::Matrix3::from_diagonal(&na::Vector3::new(1.0, 1.0, d_sign)) * svd_v_t;

    // it transposes the matrix and converts from DMatrix to Matrix3 type.
    // `rotation_t.as_slice()` iterates elements in column-major order,
    // and construct matrix by `from_row_slice_generic()` in row-major order.
    let rotation =
        na::Matrix3::from_row_slice_generic(na::Const::<3>, na::Const::<3>, rotation_t.as_slice());

    let quaternion = na::UnitQuaternion::from_matrix(&rotation);

    let translation: na::Translation3<f64> =
        (target_centroid.coords - quaternion * input_centroid.coords).into();

    let isometry = na::Isometry3 {
        translation,
        rotation: quaternion,
    };
    Some(isometry)
}

pub fn fit_rotation<P, Q>(
    input_target_pairs: impl IntoIterator<Item = (P, Q)>,
) -> Option<na::UnitQuaternion<f64>>
where
    P: Borrow<na::Point3<f64>>,
    Q: Borrow<na::Point3<f64>>,
{
    let (input_points, target_points) = input_target_pairs
        .into_iter()
        .map(|(input, target)| (input.borrow().coords, target.borrow().coords))
        .unzip_n_vec();
    if input_points.is_empty() {
        return None;
    }

    let input_matrix = na::Matrix3xX::from_columns(&*input_points);
    let target_matrix = na::Matrix3xX::from_columns(&*target_points);
    let covariance = input_matrix * target_matrix.transpose();

    let svd = na::SVD::new(covariance, true, true);
    let svd_u = svd.u.unwrap();
    let svd_v_t = svd.v_t.unwrap();

    let d_sign = (svd_u * svd_v_t).determinant().signum();
    let rotation_t =
        svd_u * na::Matrix3::from_diagonal(&na::Vector3::new(1.0, 1.0, d_sign)) * svd_v_t;

    // it transposes the matrix and converts from DMatrix to Matrix3 type.
    // `rotation_t.as_slice()` iterates elements in column-major order,
    // and construct matrix by `from_row_slice_generic()` in row-major order.
    let rotation =
        na::Matrix3::from_row_slice_generic(na::Const::<3>, na::Const::<3>, rotation_t.as_slice());

    let quaternion = na::UnitQuaternion::from_matrix(&rotation);

    Some(quaternion)
}

/// Computes the centroid point from an iterator of points, and normalize the points
/// so that the centroid becomes zero.
pub fn normalize_centroid<P>(points: impl IntoIterator<Item = P>) -> Vec<na::Point3<f64>>
where
    P: Borrow<na::Point3<f64>>,
{
    let points: Vec<_> = points.into_iter().collect();
    let centroid = centroid(points.iter().map(Borrow::borrow));

    match centroid {
        Some(centroid) => points
            .into_iter()
            .map(|point| na::Point3::from(point.borrow() - centroid))
            .collect(),
        None => {
            vec![]
        }
    }
}

/// Computes the centroid from an iterator of points.
pub fn centroid<P>(iter: impl IntoIterator<Item = P>) -> Option<na::Point3<f64>>
where
    P: Borrow<na::Point3<f64>>,
{
    let (x_sum, y_sum, z_sum, num_points): (AddVal<f64>, AddVal<f64>, AddVal<f64>, SumVal<usize>) =
        iter.into_iter()
            .map(|point| {
                let point = point.borrow();
                (point.x, point.y, point.z, 1)
            })
            .unzip_n();

    let num_points = num_points.into_inner();
    let x_mean = x_sum.into_inner()? / num_points as f64;
    let y_mean = y_sum.into_inner()? / num_points as f64;
    let z_mean = z_sum.into_inner()? / num_points as f64;

    Some(na::Point3::new(x_mean, y_mean, z_mean))
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;
    use itertools::izip;

    #[test]
    fn centroid_test() {
        let num_points = 16;
        let points: Vec<na::Point3<f64>> = (0..num_points)
            .map(|_| na::Vector3::new_random().into())
            .collect();
        let centroid = super::centroid(points.clone()).unwrap();

        let normalized_points = points
            .into_iter()
            .map(|point| na::Point3::from(point - centroid));
        let zero = super::centroid(normalized_points).unwrap();
        assert_abs_diff_eq!(zero, na::Point3::origin());
    }

    #[test]
    fn kabsch_test() {
        let num_points = 16;
        let input_points: Vec<na::Point3<f64>> = (0..num_points)
            .map(|_| na::Vector3::new_random().into())
            .collect();
        let target_points: Vec<na::Point3<f64>> = (0..num_points)
            .map(|_| na::Vector3::new_random().into())
            .collect();

        let transform = super::kabsch(izip!(input_points.clone(), target_points.clone())).unwrap();

        let optimized_input_points = input_points.into_iter().map(|point| transform * point);
        let unit_transform = super::kabsch(izip!(optimized_input_points, target_points)).unwrap();
        assert_abs_diff_eq!(unit_transform, na::Isometry3::identity(), epsilon = 1e-5);
    }

    #[test]
    fn rotation_optimize_test() {
        let num_points = 16;
        let input_points: Vec<na::Point3<f64>> = (0..num_points)
            .map(|_| na::Vector3::new_random().into())
            .collect();
        let target_points: Vec<na::Point3<f64>> = (0..num_points)
            .map(|_| na::Vector3::new_random().into())
            .collect();

        let input_points = super::normalize_centroid(input_points);
        let target_points = super::normalize_centroid(target_points);

        let rotation =
            super::fit_rotation(izip!(input_points.clone(), target_points.clone())).unwrap();

        let optimized_input_points: Vec<_> = input_points
            .into_iter()
            .map(|point| rotation * point)
            .collect();
        let unit_rotation =
            super::fit_rotation(izip!(optimized_input_points, target_points)).unwrap();

        assert_abs_diff_eq!(
            unit_rotation,
            na::UnitQuaternion::identity(),
            epsilon = 1e-5
        );
    }
}
